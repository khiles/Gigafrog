#!/usr/bin/env python3
"""Lateral nudge away from static roadside obstacles.

The model is a closed-loop lane keeper, so a lateral offset can't simply be commanded —
any sustained curvature bias is fought by the model's restoring action, and the achieved
offset isn't directly observable from the actuators.

This servos on the model's own lane geometry instead: shifting right by 0.5 m makes both
lane lines appear 0.5 m further left, so their mean y is a direct, near-zero-lag
measurement of the achieved offset. Radar clearance sets the *setpoint* only, never the
feedback — servoing on clearance would have no fixed point past a continuous row of parked
cars, and the integrator would march the car into the oncoming lane.

Everything here is in the model/calibrated frame, where **positive is right**.

Which side the centre line is on comes from ``left_hand_traffic`` (derived from the
persistent ``IsRhdDetected`` param): under right-hand traffic the centre line is
laneLines[1] and oncoming approaches on the left; under left-hand traffic it's laneLines[2]
and oncoming approaches on the right. radard's oncoming detector uses the same flag.
"""
import numpy as np

from cereal import log
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import EGO_HALF_WIDTH

LaneChangeState = log.LaneChangeState

# Servo
A_MAX = 0.40        # m/s^2, commanded lateral acceleration cap
A_RATE = 0.50       # m/s^3, jerk limit on the command itself
T_LOOK = 2.5        # s
KP = 2.0 / T_LOOK**2  # s^-2. The arc feedforward 2u/d^2 with d = v*T_LOOK collapses to
                      # 2u/T_LOOK^2, which is speed-independent and *is* the P term
KI = 0.20           # s^-3. Slow relative to the model's ~0.5-1 Hz lane-keeping bandwidth,
                    # which is the stability condition for wrapping an outer loop around it
I_LEAK_TAU = 1.0    # s

# Setpoint shaping
RAMP_ON = 0.25      # m/s, setpoint growth
RAMP_OFF = 0.50     # m/s, decay toward zero
RAMP_RETREAT = 0.80 # m/s, decay when the budget itself shrank, i.e. oncoming appeared
DEADBAND_BOTH = 0.15  # m, minimum net demand before acting on a two-sided squeeze
SIDE_LATCH_S = 3.0  # s a committed side must be clear before we may reverse
INHIBIT_S = 2.0     # s of suppression after a lane change or driver steering input

# Budget
MARGIN_LANE = 0.15  # m
MARGIN_EDGE = 0.35  # m
CAP_NO_REF = 0.35   # m, hard cap when neither lane lines nor road edges are usable
LANE_PROB_MIN = 0.5
LINE_STD_MAX = 0.5

# Curvature derate. 0.005 1/m is roughly a 200 m radius. Zeroing the command here also
# keeps us clear of the region where clip_curvature's lateral accel clamp would bind,
# which is what would otherwise wind the integrator up against an invisible saturation.
CURVE_BP = [0.005, 0.015]
CURVE_V = [1.0, 0.0]

REF_NONE, REF_LANE, REF_EDGE = 0, 1, 2


def measured_offset(model_data):
  """Achieved lateral offset from the lane/road centre, in metres, positive = right.

  Returns (offset, reference). The offset is None when no reference is usable.
  """
  probs = model_data.laneLineProbs
  stds = model_data.laneLineStds
  if len(probs) >= 4 and len(stds) >= 4:
    if min(probs[1], probs[2]) > LANE_PROB_MIN and max(stds[1], stds[2]) < LINE_STD_MAX:
      return -(model_data.laneLines[1].y[0] + model_data.laneLines[2].y[0]) / 2.0, REF_LANE

  edge_stds = model_data.roadEdgeStds
  if len(edge_stds) >= 2 and max(edge_stds[0], edge_stds[1]) < LINE_STD_MAX:
    return -(model_data.roadEdges[0].y[0] + model_data.roadEdges[1].y[0]) / 2.0, REF_EDGE

  return None, REF_NONE


def lateral_budget(model_data, v_ego, oncoming_clear, frogpilot_toggles, offset=0.0):
  """How far from the lane centre the path may be shifted each way, as two positive
  magnitudes in metres.

  Evaluated at three look-ahead points and minimised, so a lane that narrows ahead binds
  now rather than two seconds from now.

  The lane lines move with us, so the raw numbers below are "room still remaining from
  where we are". ``offset`` (our current position right of centre) converts them into
  absolute limits from the lane centre — which is the frame the setpoint lives in. Mixing
  the two frames makes the budget shrink as the car moves into it, and the servo limit
  cycles.
  """
  look = float(np.clip(v_ego * 2.0, 8.0, 30.0))
  xs = (0.0, look / 2.0, look)

  probs = model_data.laneLineProbs
  stds = model_data.laneLineStds
  edge_stds = model_data.roadEdgeStds

  lanes_ok = (len(probs) >= 4 and len(stds) >= 4 and
              min(probs[1], probs[2]) > LANE_PROB_MIN and max(stds[1], stds[2]) < LINE_STD_MAX)
  edges_ok = len(edge_stds) >= 2 and max(edge_stds[0], edge_stds[1]) < LINE_STD_MAX

  if not lanes_ok and not edges_ok:
    return CAP_NO_REF, CAP_NO_REF

  def edge_limit(index, x):
    edge = model_data.roadEdges[index]
    y = float(np.interp(x, edge.x, edge.y))
    return (-y if index == 0 else y) - EGO_HALF_WIDTH - MARGIN_EDGE

  left = right = 1e3
  for x in xs:
    if lanes_ok:
      left = min(left, -float(np.interp(x, model_data.laneLines[1].x, model_data.laneLines[1].y)) - EGO_HALF_WIDTH - MARGIN_LANE)
      right = min(right, float(np.interp(x, model_data.laneLines[2].x, model_data.laneLines[2].y)) - EGO_HALF_WIDTH - MARGIN_LANE)
    if edges_ok:
      left = min(left, edge_limit(0, x))
      right = min(right, edge_limit(1, x))

  # Crossing the centre line extends both the lane-line budget and the offset cap. It has
  # to extend the cap too: on a normal lane the lane-line budget already exceeds the
  # default cap, so extending only the budget would leave the toggle doing nothing.
  # The road edge is the hard limit and still clamps — on a residential street the centre
  # line is one side and the far kerb is the other. Crossing the centre is permitted,
  # crossing the kerb never is.
  cap_left = cap_right = frogpilot_toggles.obstacle_nudge_max_offset
  if lanes_ok and frogpilot_toggles.obstacle_nudge_cross_center_line and oncoming_clear:
    overshoot = frogpilot_toggles.obstacle_nudge_max_center_line_overshoot
    if frogpilot_toggles.left_hand_traffic:
      right += overshoot
      cap_right += overshoot
      if edges_ok:
        right = min(right, min(edge_limit(1, x) for x in xs))
    else:
      left += overshoot
      cap_left += overshoot
      if edges_ok:
        left = min(left, min(edge_limit(0, x) for x in xs))

  # Convert from room-remaining into absolute limits either side of the lane centre
  return float(np.clip(left - offset, 0.0, cap_left)), float(np.clip(right + offset, 0.0, cap_right))


class FrogPilotNudge:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.crossing_center_line = False

    self.a_cmd = 0.0
    self.i_term = 0.0
    self.offset_measured = 0.0
    self.offset_target = 0.0

    self.inhibit_t = 0.0
    self.side_clear_t = 0.0
    self.nudge_side = 0

  def reset(self):
    self.offset_target = _toward_zero(self.offset_target, RAMP_OFF)
    self.i_term *= np.exp(-DT_MDL / I_LEAK_TAU)
    self.a_cmd = _toward_zero(self.a_cmd, A_RATE)
    self.crossing_center_line = False
    if self.offset_target == 0.0:
      self.offset_measured = 0.0
      self.nudge_side = 0

  def update(self, v_ego, sm, frogpilot_toggles):
    model_data = sm["modelV2"]
    carstate = sm["carState"]
    radar_state = sm["frogpilotRadarState"]

    driver_active = carstate.steeringPressed or carstate.leftBlinker or carstate.rightBlinker
    lane_changing = model_data.meta.laneChangeState != LaneChangeState.off
    if driver_active or lane_changing:
      self.inhibit_t = INHIBIT_S
    else:
      self.inhibit_t = max(0.0, self.inhibit_t - DT_MDL)

    engaged = sm["carControl"].latActive and self.frogpilot_planner.lateral_check
    in_speed_range = frogpilot_toggles.obstacle_nudge_min_speed <= v_ego <= frogpilot_toggles.obstacle_nudge_max_speed

    if not (frogpilot_toggles.obstacle_nudge and engaged and in_speed_range and self.inhibit_t == 0.0
            and len(model_data.position.x)):
      self.reset()
      return

    # Measure first: the budget is expressed relative to the lane centre, which needs to
    # know where we currently are.
    u_meas, reference = measured_offset(model_data)
    self.offset_measured = 0.0 if reference == REF_NONE else float(u_meas)

    oncoming_clear = not radar_state.oncomingDetected
    budget_left, budget_right = lateral_budget(model_data, v_ego, oncoming_clear,
                                               frogpilot_toggles, self.offset_measured)

    u_raw = self._demand(radar_state, frogpilot_toggles)

    curve_scale = float(np.interp(abs(self.frogpilot_planner.road_curvature), CURVE_BP, CURVE_V))
    u_clipped = float(np.clip(u_raw, -budget_left, budget_right)) * curve_scale

    # Come back fastest when we're offset toward oncoming traffic and some has appeared:
    # there the budget collapsed under us rather than the demand merely easing.
    retreating = not oncoming_clear and self.offset_target < 0.0

    if abs(u_clipped) < abs(self.offset_target):
      rate = RAMP_RETREAT if retreating else RAMP_OFF
    else:
      rate = RAMP_ON
    self.offset_target = _rate_limit(self.offset_target, u_clipped, rate)

    if reference == REF_NONE:
      # Open loop: nothing observes the achieved offset, so keep the authority tiny
      self.offset_target = float(np.clip(self.offset_target, -CAP_NO_REF, CAP_NO_REF))
      error = self.offset_target
      self.i_term *= np.exp(-DT_MDL / I_LEAK_TAU)
    else:
      error = self.offset_target - self.offset_measured
      i_max = A_MAX / KI
      self.i_term = float(np.clip(self.i_term + error * DT_MDL, -i_max, i_max))

    if abs(self.offset_target) < 0.02:
      self.i_term *= np.exp(-DT_MDL / I_LEAK_TAU)

    a_raw = frogpilot_toggles.obstacle_nudge_gain * (KP * error + KI * self.i_term)
    a_raw = float(np.clip(a_raw, -A_MAX, A_MAX))
    self.a_cmd = _rate_limit(self.a_cmd, a_raw, A_RATE)

    self.crossing_center_line = self._is_crossing(model_data, frogpilot_toggles.left_hand_traffic)

  def _demand(self, radar_state, frogpilot_toggles):
    """Net lateral demand in metres, positive = move right."""
    min_clearance = frogpilot_toggles.obstacle_nudge_min_clearance
    obstacle_left = radar_state.staticObstacleLeft
    obstacle_right = radar_state.staticObstacleRight

    need_right = max(0.0, min_clearance - obstacle_left.clearance) if obstacle_left.detected else 0.0
    need_left = max(0.0, min_clearance - obstacle_right.clearance) if obstacle_right.detected else 0.0
    u_raw = need_right - need_left

    # A symmetric squeeze commands nothing: there is nowhere to go, and holding centre is
    # the honest answer. The net difference also converges on the midpoint of an asymmetric
    # one rather than picking a winner and lurching.
    if obstacle_left.detected and obstacle_right.detected:
      if abs(u_raw) < DEADBAND_BOTH:
        u_raw = 0.0
      u_raw = float(np.clip(u_raw, -0.5 * frogpilot_toggles.obstacle_nudge_max_offset,
                            0.5 * frogpilot_toggles.obstacle_nudge_max_offset))

    return self._latch_side(u_raw)

  def _latch_side(self, u_raw):
    """Refuse to reverse sides until the committed side has been continuously clear.

    Without this, alternating rows of parked cars produce left-right hunting.
    """
    side = 0 if u_raw == 0.0 else (1 if u_raw > 0 else -1)

    if self.nudge_side == 0:
      self.nudge_side = side
      self.side_clear_t = 0.0
      return u_raw

    if side == self.nudge_side:
      self.side_clear_t = 0.0
      return u_raw

    self.side_clear_t += DT_MDL
    if self.side_clear_t >= SIDE_LATCH_S:
      self.nudge_side = side
      self.side_clear_t = 0.0
      return u_raw

    return 0.0

  def _is_crossing(self, model_data, left_hand_traffic):
    """True when we are actually on or over the centre line.

    The lane lines already move with us, so no offset arithmetic is needed: if the centre
    line has come inside our body edge, we're on it. Under left-hand traffic the centre
    line is laneLines[2] and we approach it by moving right.
    """
    index = 2 if left_hand_traffic else 1
    if (self.offset_target <= 0.0) if left_hand_traffic else (self.offset_target >= 0.0):
      return False

    probs = model_data.laneLineProbs
    if len(probs) < 4 or probs[index] <= LANE_PROB_MIN:
      return False

    edge = float(model_data.laneLines[index].y[0])
    return edge < EGO_HALF_WIDTH if left_hand_traffic else edge > -EGO_HALF_WIDTH

  @property
  def lateral_accel(self):
    return self.a_cmd


def _rate_limit(value, target, rate):
  step = rate * DT_MDL
  return float(np.clip(target, value - step, value + step))


def _toward_zero(value, rate):
  step = rate * DT_MDL
  if abs(value) <= step:
    return 0.0
  return value - step if value > 0 else value + step
