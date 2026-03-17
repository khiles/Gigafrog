#!/usr/bin/env python3
import math
from numbers import Number

from cereal import car, custom, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles
from openpilot.frogpilot.controls.lib.neural_network_feedforward import LatControlNNFF

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    self.FPCP = messaging.log_from_bytes(self.params.get("FrogPilotCarParams", block=True), custom.FrogPilotCarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.FPCP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    # Override-resume blend: track transition from inactive→active so the curvature
    # target ramps from physical to OP-desired over OVERRIDE_RESUME_BLEND_S instead of
    # stepping immediately (which saturates the carcontroller 5°/frame rate cap).
    self._lat_was_active = False
    self._resume_curvature = 0.0
    self._resume_blend_t = 1.5  # start "complete" so first engage blends from current angle

    # steeringPressed blend: when driver releases the wheel, blend curvature back to OP's
    # desired target over POST_PRESS_BLEND_S and halve the jerk rate during that window
    # to prevent a sudden lateral snap.
    self._steer_was_pressed = False
    self._post_press_blend_t = 1.5  # start "complete"
    self._post_press_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

    # FrogPilot variables
    self.sm = self.sm.extend(['liveDelay', 'frogpilotCarState', 'frogpilotPlan'])

    self.frogpilot_toggles = get_frogpilot_toggles()

    if self.CP.lateralTuning.which() == "torque" and (self.frogpilot_toggles.nnff or self.frogpilot_toggles.nnff_lite):
      self.LaC = LatControlNNFF(self.CP, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    # FrogPilot variables
    if hasattr(self.LaC, "pid") and self.CP.lateralTuning.which() != "pid":
      self.LaC.pid._k_p = self.frogpilot_toggles.steerKp

    if self.sm.updated['liveDelay'] and hasattr(self.LaC, "update_live_delay"):
      self.LaC.update_live_delay(self.sm['liveDelay'].lateralDelay)

    self.frogpilot_toggles = get_frogpilot_toggles(self.sm)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and (torque_params.useParams or self.frogpilot_toggles.force_auto_tune):
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    CC.latActive = (self.sm['selfdriveState'].active or self.sm['frogpilotCarState'].alwaysOnLateralEnabled) and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill) and self.sm['frogpilotPlan'].lateralCheck
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and not self.sm['frogpilotCarState'].pauseLongitudinal and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(min(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits, self.frogpilot_toggles), self.frogpilot_toggles.max_desired_acceleration))

    # Steering PID loop and lateral MPC
    # Three curvature-target modes, in priority order:
    #  1. latActive False             → track physical angle (no snap on re-engage)
    #  2. latActive rising edge       → 1.5 s blend from physical to OP desired
    #  3. steeringPressed falling edge→ 1.5 s blend from physical to OP desired,
    #                                   jerk rate halved for extra smoothness
    #  4. steady-state                → track OP model output
    #
    # NOTE: steeringPressed does NOT suppress OP's curvature target. If we
    # tracked physical when steeringPressed, OP would follow the driver's
    # current angle (often straight) and refuse to turn curved roads — the
    # car-level cooperative override (Tesla hands_authority, torque blend) is
    # the right place to handle driver input, not here.
    OVERRIDE_RESUME_BLEND_S = 1.5
    POST_PRESS_BLEND_S = 1.5

    # Detect steeringPressed falling edge (driver just released wheel)
    if CC.latActive and self._steer_was_pressed and not CS.steeringPressed:
      self._post_press_curvature = self.curvature  # seed from physical, not desired (avoids lag-induced snap)
      self._post_press_blend_t = 0.0
    self._steer_was_pressed = CS.steeringPressed

    # Detect latActive rising edge
    if CC.latActive and not self._lat_was_active:
      self._resume_curvature = self.curvature  # seed from physical, not desired (avoids lag-induced snap)
      self._resume_blend_t = 0.0
    self._lat_was_active = CC.latActive

    jerk_scale = 1.0
    if not CC.latActive:
      # Inactive: track physical so re-engage is smooth
      new_desired_curvature = self.curvature
    elif self._resume_blend_t < OVERRIDE_RESUME_BLEND_S:
      # latActive rising edge blend
      self._resume_blend_t = min(self._resume_blend_t + DT_CTRL, OVERRIDE_RESUME_BLEND_S)
      alpha = self._resume_blend_t / OVERRIDE_RESUME_BLEND_S
      new_desired_curvature = self._resume_curvature + alpha * (model_v2.action.desiredCurvature - self._resume_curvature)
    elif self._post_press_blend_t < POST_PRESS_BLEND_S:
      # steeringPressed release blend — also halve jerk rate to prevent lateral snap
      self._post_press_blend_t = min(self._post_press_blend_t + DT_CTRL, POST_PRESS_BLEND_S)
      alpha = self._post_press_blend_t / POST_PRESS_BLEND_S
      new_desired_curvature = self._post_press_curvature + alpha * (model_v2.action.desiredCurvature - self._post_press_curvature)
      jerk_scale = 0.5 + 0.5 * alpha  # ramps 0.5 → 1.0 over blend window
    else:
      new_desired_curvature = model_v2.action.desiredCurvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll, jerk_scale)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay,
                                                       self.calibrated_pose,
                                                       self.sm['modelV2'],
                                                       self.frogpilot_toggles)
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)

    # OPGM variables
    if len(long_plan.speeds):
      actuators.speed = long_plan.speeds[-1]

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling) or self.sm["frogpilotCarState"].forceCoast)

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
