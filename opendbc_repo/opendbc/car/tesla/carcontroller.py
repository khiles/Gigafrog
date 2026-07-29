import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.body_controls_cancel_sends = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1,):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      # HW3 uses a separate party-bus DBC (tesla_raven_party) that has no counter/checksum
      # for DAS_bodyControls. The BCM lives on the chassis bus (first panda's bus 1), so add
      # a chassis packer using tesla_can.dbc (which does have counter/checksum) so the
      # indicator override message goes directly to the BCM rather than relying on AP forwarding.
      if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1  # also set by CarState.__init__; don't rely on construction order
        self.packers[CANBUS.chassis] = CANPacker(dbc_names[Bus.chassis])
      self.tesla_can = TeslaCANRaven(self.packers)
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < 3

    # Gradually reduce steering authority as driver pressure increases instead of a hard cut.
    # Level 0-1: full authority, Level 2 (medium pressure): 50% authority, Level 3+: disabled.
    # This creates a smooth cooperative hand-off rather than an abrupt disengage.
    hands_authority = float(np.interp(CS.hands_on_level, [0, 1, 2, 3], [1.0, 1.0, 0.5, 0.0])) if CC.latActive else 0.0
    # Blend angle command towards current steering angle when authority < 1
    steer_angle_cmd = (CS.out.steeringAngleDeg +
                       hands_authority * (actuators.steeringAngleDeg - CS.out.steeringAngleDeg))

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(steer_angle_cmd, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)

      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, actuators.speed))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # Keep the turn indicator on for the full duration of an openpilot-commanded
    # lane change. CC.leftBlinker/rightBlinker are driven by the model's lane
    # change state (not the physical stalk), so they stay True past the 3-flash
    # auto-cancel that normally kills the blinker mid-maneuver. Only transmit while
    # requesting an indicator, plus a short NONE tail so the BCM reliably cancels;
    # outside of that the stock AP owns DAS_bodyControls (auto wipers/high beam).
    if self.frame % 10 == 0:
      blinker_cmd = CC.leftBlinker or CC.rightBlinker
      if blinker_cmd:
        self.body_controls_cancel_sends = 10  # ~1s of NONE frames once the request drops
      if blinker_cmd or self.body_controls_cancel_sends > 0:
        if blinker_cmd:
          turn_indicator, turn_reason = (1 if CC.leftBlinker else 2), 6  # LEFT/RIGHT, DAS_ACTIVE_COMMANDED_LANE_CHANGE
        else:
          self.body_controls_cancel_sends -= 1
          turn_indicator, turn_reason = 0, 0   # NONE
        cntr = (self.frame // 10) % 16
        can_sends.append(self.tesla_can.create_body_controls(cntr, turn_indicator, turn_reason))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
