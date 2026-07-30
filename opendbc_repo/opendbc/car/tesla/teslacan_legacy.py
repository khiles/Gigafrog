from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, v_target=None):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      if accel < 0:
        set_speed = 0
      elif v_target is not None and v_target > 0:
        set_speed = max(v_target * CV.MS_TO_KPH, v_ego * CV.MS_TO_KPH)
      else:
        set_speed = V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_body_controls(self, counter, turn_indicator, turn_reason, stock=None):
    # HW3 (Model S/X HW3) carries the BCM on the chassis bus (bus 5, tesla_can.dbc),
    # which has DAS_bodyControlsCounter and DAS_bodyControlsChecksum. Send directly there
    # so the BCM receives the indicator override without relying on AP forwarding.
    # HW1/HW2 send on the party bus (bus 0, also tesla_can.dbc); panda relays to chassis.
    if CANBUS.chassis in self.packers:
      bus = CANBUS.chassis
    else:
      bus = CANBUS.party
    packer = self.packers[bus]

    # This message owns the headlights, high beams, wipers and hazards as well as the turn
    # indicator, and while we are commanding the indicator the panda blocks the stock
    # Autopilot's copy — so the BCM acts on OUR values for all of them, not just the indicator.
    # Mirror whatever the stock AP last asked for and override only the indicator, so its auto
    # headlights and auto wipers keep working through a lane change.
    #
    # Getting this wrong is not benign. DAS_highLowBeamOffReason was previously left out of the
    # dict entirely, so it packed as 0 — which the DBC defines as HIGH_BEAM_ON. That flashed the
    # high beams on every lane change.
    if stock is not None:
      headlight = int(stock["DAS_headlightRequest"])
      hazard = int(stock["DAS_hazardLightRequest"])
      wiper = int(stock["DAS_wiperSpeed"])
      beam_decision = int(stock["DAS_highLowBeamDecision"])
      beam_off_reason = int(stock["DAS_highLowBeamOffReason"])
    else:
      # No stock frame seen — request nothing at all rather than guessing a state.
      headlight = 3        # INVALID = no DAS headlight request
      hazard = 3           # SNA = no DAS hazard request
      wiper = 15           # INVALID = no DAS wiper request
      beam_decision = 3    # SNA = no DAS beam request
      beam_off_reason = 5  # SNA. NOT 0 — 0 is HIGH_BEAM_ON

    values = {
      "DAS_headlightRequest": headlight,
      "DAS_hazardLightRequest": hazard,
      "DAS_wiperSpeed": wiper,
      "DAS_turnIndicatorRequest": turn_indicator,
      "DAS_turnIndicatorRequestReason": turn_reason,
      "DAS_highLowBeamDecision": beam_decision,
      "DAS_highLowBeamOffReason": beam_off_reason,
      "DAS_bodyControlsCounter": counter,
    }
    data = packer.make_can_msg("DAS_bodyControls", bus, values)[1]
    values["DAS_bodyControlsChecksum"] = self.checksum(0x3E9, data[:7])
    return packer.make_can_msg("DAS_bodyControls", bus, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)
