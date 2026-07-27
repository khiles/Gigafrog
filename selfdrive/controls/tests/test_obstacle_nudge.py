import numpy as np
import pytest

from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.common.frogpilot_variables import EGO_HALF_WIDTH
from openpilot.frogpilot.controls.lib import frogpilot_nudge as N


class Line:
  def __init__(self, y):
    self.x = [float(i) for i in range(0, 61, 5)]
    self.y = [y] * len(self.x)


def make_model(offset=0.0, lane_half=1.85, edge_half=4.0, lanes_ok=True, edges_ok=True):
  """offset is our position right of lane centre, so the lines shift left by that much."""
  md = SimpleNamespace()
  md.laneLines = [Line(-lane_half * 2 - offset), Line(-lane_half - offset),
                  Line(lane_half - offset), Line(lane_half * 2 - offset)]
  md.laneLineProbs = [0.9] * 4 if lanes_ok else [0.1] * 4
  md.laneLineStds = [0.2] * 4 if lanes_ok else [0.9] * 4
  md.roadEdges = [Line(-edge_half - offset), Line(edge_half - offset)]
  md.roadEdgeStds = [0.2, 0.2] if edges_ok else [0.9, 0.9]
  md.position = SimpleNamespace(x=[float(i) for i in range(0, 101, 5)], y=[0.0] * 21)
  md.meta = SimpleNamespace(laneChangeState=N.LaneChangeState.off)
  return md


def obstacle(clearance):
  return SimpleNamespace(detected=True, clearance=clearance, dRel=15.0, yRel=0.0, count=2)


NO_OBSTACLE = SimpleNamespace(detected=False, clearance=0.0, dRel=0.0, yRel=0.0, count=0)


def make_sm(md, obs_left=NO_OBSTACLE, obs_right=NO_OBSTACLE, oncoming=False, pressed=False):
  return {
    "modelV2": md,
    "carState": SimpleNamespace(steeringPressed=pressed, leftBlinker=False, rightBlinker=False),
    "carControl": SimpleNamespace(latActive=True),
    "frogpilotRadarState": SimpleNamespace(staticObstacleLeft=obs_left,
                                           staticObstacleRight=obs_right,
                                           oncomingDetected=oncoming),
  }


def toggles(**kw):
  base = dict(obstacle_nudge=True, obstacle_nudge_cross_center_line=False,
              obstacle_nudge_gain=1.0, obstacle_nudge_max_center_line_overshoot=0.0,
              obstacle_nudge_max_offset=0.5, obstacle_nudge_max_speed=20.0,
              obstacle_nudge_min_clearance=1.0, obstacle_nudge_min_speed=4.5,
              obstacle_nudge_trigger_distance=30.0, left_hand_traffic=False)
  base.update(kw)
  return SimpleNamespace(**base)


class Plant:
  """Stand-in for the model as a lane keeper, correcting both cross-track error and
  lateral velocity. The damping term matters — an undamped spring would ring forever and
  say nothing useful about the controller."""
  def __init__(self, k_pos=0.55, k_vel=1.1):
    self.y = 0.0
    self.vy = 0.0
    self.k_pos = k_pos
    self.k_vel = k_vel

  def step(self, a_cmd):
    a = a_cmd - self.k_pos * self.y - self.k_vel * self.vy
    self.vy = float(np.clip(self.vy + a * DT_MDL, -1.5, 1.5))
    self.y += self.vy * DT_MDL
    return self.y


def simulate(frogpilot_toggles, steps=200, v_ego=12.0, road_curvature=0.0, **sm_kwargs):
  planner = SimpleNamespace(lateral_check=True, road_curvature=road_curvature)
  nudge = N.FrogPilotNudge(planner)
  plant = Plant()
  history = []

  for i in range(steps):
    md = make_model(offset=plant.y)
    kwargs = {k: (v(i) if callable(v) else v) for k, v in sm_kwargs.items()}
    nudge.update(v_ego, make_sm(md, **kwargs), frogpilot_toggles)
    plant.step(nudge.a_cmd)
    history.append(SimpleNamespace(target=nudge.offset_target, measured=nudge.offset_measured,
                                   a_cmd=nudge.a_cmd, true_y=plant.y))

  return nudge, history


def test_left_obstacle_nudges_right():
  # yRel is left-positive but the model frame is right-positive, so a left-side obstacle
  # must produce a positive (rightward) offset. Getting this backwards steers into the car.
  _, history = simulate(toggles(), obs_left=obstacle(0.3))
  assert history[-1].target > 0.2
  assert history[-1].true_y > 0.2


def test_right_obstacle_nudges_left():
  _, history = simulate(toggles(), obs_right=obstacle(0.3))
  assert history[-1].target < -0.2
  assert history[-1].true_y < -0.2


def test_measurement_tracks_true_position():
  _, history = simulate(toggles(), obs_left=obstacle(0.3))
  assert abs(history[-1].measured - history[-1].true_y) < 0.05


def test_acceleration_and_jerk_limits():
  _, history = simulate(toggles(), obs_left=obstacle(0.0))
  assert all(abs(h.a_cmd) <= N.A_MAX + 1e-6 for h in history)
  pairs = list(zip(history, history[1:], strict=False))
  assert all(abs(b.a_cmd - a.a_cmd) <= N.A_RATE * DT_MDL + 1e-6 for a, b in pairs)
  assert all(abs(b.target - a.target) <= N.RAMP_RETREAT * DT_MDL + 1e-6 for a, b in pairs)


def test_offset_does_not_limit_cycle():
  # The budget is measured from the current position while the setpoint is measured from
  # lane centre. If those frames aren't reconciled the budget shrinks as the car moves
  # into it and the servo oscillates.
  _, history = simulate(toggles(), obs_left=obstacle(-5.0), steps=400)
  settled = [h.target for h in history[200:]]
  assert max(settled) - min(settled) < 0.02
  assert abs(history[-1].target) <= 0.5 + 1e-6


def test_symmetric_squeeze_holds_center():
  _, history = simulate(toggles(), obs_left=obstacle(0.3), obs_right=obstacle(0.3))
  assert abs(history[-1].target) < 0.02


def test_driver_input_suppresses_nudge():
  nudge, _ = simulate(toggles(), obs_left=obstacle(0.2), pressed=lambda i: i > 100)
  assert abs(nudge.a_cmd) < 0.05


def test_curvature_derate_zeroes_command():
  nudge, _ = simulate(toggles(), obs_left=obstacle(0.2), road_curvature=0.02)
  assert abs(nudge.offset_target) < 0.02


def test_speed_range_gate():
  nudge, _ = simulate(toggles(), v_ego=2.0, obs_left=obstacle(0.2))
  assert abs(nudge.offset_target) < 0.02
  nudge, _ = simulate(toggles(), v_ego=35.0, obs_left=obstacle(0.2))
  assert abs(nudge.offset_target) < 0.02


def test_retreats_when_oncoming_appears():
  frogpilot_toggles = toggles(obstacle_nudge_cross_center_line=True,
                              obstacle_nudge_max_center_line_overshoot=0.3)
  planner = SimpleNamespace(lateral_check=True, road_curvature=0.0)
  nudge = N.FrogPilotNudge(planner)
  plant = Plant()

  peak = 0.0
  for i in range(300):
    md = make_model(offset=plant.y)
    nudge.update(12.0, make_sm(md, obs_right=obstacle(0.2), oncoming=i > 150), frogpilot_toggles)
    plant.step(nudge.a_cmd)
    if i == 150:
      peak = nudge.offset_target

  assert abs(nudge.offset_target) < abs(peak)


@pytest.mark.parametrize("oncoming,expected_left", [(False, 0.8), (True, 0.5)])
def test_center_line_overshoot_gated_on_oncoming(oncoming, expected_left):
  # The overshoot has to raise the cap as well as the lane-line budget. On a normal lane
  # the lane budget already exceeds the default cap, so raising only the budget would
  # leave the toggle doing nothing at all.
  frogpilot_toggles = toggles(obstacle_nudge_cross_center_line=True,
                              obstacle_nudge_max_center_line_overshoot=0.3)
  left, right = N.lateral_budget(make_model(), 12.0, not oncoming, frogpilot_toggles)
  assert left == pytest.approx(expected_left)
  assert right == pytest.approx(0.5)


def test_road_edge_clamps_overshoot():
  frogpilot_toggles = toggles(obstacle_nudge_cross_center_line=True,
                              obstacle_nudge_max_center_line_overshoot=0.6,
                              obstacle_nudge_max_offset=1.0)
  left, _ = N.lateral_budget(make_model(edge_half=2.4), 12.0, True, frogpilot_toggles)
  assert left <= 2.4 - EGO_HALF_WIDTH - N.MARGIN_EDGE + 1e-6


def test_no_reference_falls_back_to_hard_cap():
  left, right = N.lateral_budget(make_model(lanes_ok=False, edges_ok=False), 12.0, True, toggles())
  assert left == right == N.CAP_NO_REF


@pytest.mark.parametrize("oncoming", [False, True])
def test_left_hand_traffic_mirrors_the_overshoot(oncoming):
  # Under left-hand traffic the centre line is laneLines[2], so the overshoot has to extend
  # the RIGHT budget. Extending the left would push toward the kerb instead.
  frogpilot_toggles = toggles(obstacle_nudge_cross_center_line=True,
                              obstacle_nudge_max_center_line_overshoot=0.3,
                              left_hand_traffic=True)
  left, right = N.lateral_budget(make_model(), 12.0, not oncoming, frogpilot_toggles)
  assert left == pytest.approx(0.5)
  assert right == pytest.approx(0.8 if not oncoming else 0.5)


def test_left_hand_traffic_crossing_detection():
  planner = SimpleNamespace(lateral_check=True, road_curvature=0.0)
  nudge = N.FrogPilotNudge(planner)

  lht = toggles(left_hand_traffic=True)
  rht = toggles(left_hand_traffic=False)

  # Sitting 1.0 m right of centre puts laneLines[2] at 0.85 m, inside the body half width
  md = make_model(offset=1.0)
  nudge.offset_target = 0.5
  assert nudge._is_crossing(md, True)      # LHT: moving right crosses the centre line
  assert not nudge._is_crossing(md, False)  # RHT: moving right heads for the kerb

  nudge.offset_target = -0.5
  assert not nudge._is_crossing(md, True)
  assert lht.left_hand_traffic and not rht.left_hand_traffic
