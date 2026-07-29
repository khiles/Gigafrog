"""Lane-position measurement.

The sign is the part worth pinning down, because it is derived rather than measured. The model's
lateral frame is right-positive and laneLines[1]/[2] are the ego lane's left/right lines, so a
positive offset means the lane centre is to the RIGHT of the car — the car is sitting left of
where it should be. test_sign_is_lane_centre_relative_to_car is the guard on that.

The gates matter as much as the maths. An unseen lane line still regresses to a plausible-looking
y, so without the probability gate the offset becomes confident nonsense rather than absent.
"""
import pytest

from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib import frogpilot_lane_centering as LC


def make_model(left_y=-1.8, right_y=1.8, left_prob=0.9, right_prob=0.9):
  """laneLines is [far-left, left, right, far-right]; only indices 1 and 2 are read."""
  line = lambda y: SimpleNamespace(y=[y] * 33)
  return SimpleNamespace(
    laneLines=[line(left_y - 3.6), line(left_y), line(right_y), line(right_y + 3.6)],
    laneLineProbs=[0.1, left_prob, right_prob, 0.1],
  )


def run(lc, seconds, model=None, v_ego=20.0, lane_change=False):
  model = model if model is not None else make_model()
  for _ in range(max(1, int(seconds / DT_MDL))):
    lc.update(model, v_ego, lane_change)
  return lc


# ---- maths and sign ----

def test_centred_car_reads_zero():
  lc = run(LC.FrogPilotLaneCentering(), 10.0, make_model(left_y=-1.8, right_y=1.8))
  assert lc.lane_offset_valid
  assert lc.lane_offset_filtered == pytest.approx(0.0, abs=1e-3)
  assert lc.lane_width == pytest.approx(3.6)


def test_sign_is_lane_centre_relative_to_car():
  # Car 0.4m LEFT of centre: both lines shift positive, so the centre is to the right.
  lc = run(LC.FrogPilotLaneCentering(), 30.0, make_model(left_y=-1.4, right_y=2.2))
  assert lc.lane_offset_filtered > 0
  assert lc.lane_offset_filtered == pytest.approx(0.4, abs=0.02)

  # Car 0.4m RIGHT of centre.
  lc = run(LC.FrogPilotLaneCentering(), 30.0, make_model(left_y=-2.2, right_y=1.4))
  assert lc.lane_offset_filtered < 0
  assert lc.lane_offset_filtered == pytest.approx(-0.4, abs=0.02)


def test_instantaneous_offset_is_not_filtered():
  lc = LC.FrogPilotLaneCentering()
  lc.update(make_model(left_y=-1.4, right_y=2.2), 20.0, False)
  assert lc.lane_offset == pytest.approx(0.4)
  # one sample in, the filter has barely moved
  assert abs(lc.lane_offset_filtered) < abs(lc.lane_offset)


def test_filter_is_slow():
  """The point is systematic offset, not noise. A single frame must barely move the output."""
  lc = LC.FrogPilotLaneCentering()
  lc.update(make_model(left_y=-1.4, right_y=2.2), 20.0, False)
  one_frame = lc.lane_offset_filtered
  assert one_frame < 0.4 * (DT_MDL / LC.OFFSET_TAU) * 2


# ---- gates: absent must mean absent, never a confident zero ----

def test_low_confidence_suppresses():
  lc = run(LC.FrogPilotLaneCentering(), 10.0,
           make_model(left_y=-1.4, right_y=2.2, left_prob=LC.MIN_LANE_LINE_PROB - 0.1))
  assert not lc.lane_offset_valid
  assert lc.lane_offset_filtered == 0.0

  lc = run(LC.FrogPilotLaneCentering(), 10.0,
           make_model(left_y=-1.4, right_y=2.2, right_prob=LC.MIN_LANE_LINE_PROB - 0.1))
  assert not lc.lane_offset_valid


def test_lane_change_suppresses():
  lc = run(LC.FrogPilotLaneCentering(), 10.0, make_model(left_y=-1.4, right_y=2.2),
           lane_change=True)
  assert not lc.lane_offset_valid


def test_low_speed_suppresses():
  lc = run(LC.FrogPilotLaneCentering(), 10.0, make_model(left_y=-1.4, right_y=2.2),
           v_ego=LC.MIN_SPEED - 1.0)
  assert not lc.lane_offset_valid


@pytest.mark.parametrize("width", [1.5, 6.0])
def test_implausible_lane_width_suppresses(width):
  half = width / 2.0
  lc = run(LC.FrogPilotLaneCentering(), 10.0, make_model(left_y=-half, right_y=half))
  assert not lc.lane_offset_valid


def test_losing_confidence_clears_a_held_offset():
  """A stale offset must not keep being reported once the lines go away."""
  lc = run(LC.FrogPilotLaneCentering(), 30.0, make_model(left_y=-1.4, right_y=2.2))
  assert lc.lane_offset_filtered > 0.3
  lc.update(make_model(left_prob=0.1), 20.0, False)
  assert not lc.lane_offset_valid
  assert lc.lane_offset_filtered == 0.0


def test_short_lane_line_arrays_do_not_crash():
  lc = LC.FrogPilotLaneCentering()
  lc.update(SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False)
  assert not lc.lane_offset_valid


# ---- the control constraint, encoded ----

def test_there_is_no_integrator():
  """A constant offset must produce a constant reading, not a growing one. The model actively
  opposes any downstream bias, so an integrator would wind up until it saturated."""
  lc = run(LC.FrogPilotLaneCentering(), 30.0, make_model(left_y=-1.4, right_y=2.2))
  settled = lc.lane_offset_filtered
  run(lc, 60.0, make_model(left_y=-1.4, right_y=2.2))
  assert lc.lane_offset_filtered == pytest.approx(settled, abs=0.01)
  assert lc.lane_offset_filtered <= 0.45  # bounded by the input, not accumulating


def test_module_publishes_no_control_output():
  """Stage A is measurement only. If a demand/gain/curvature attribute appears here, the staging
  discipline has been broken and this test should be updated deliberately, not incidentally."""
  lc = run(LC.FrogPilotLaneCentering(), 5.0)
  banned = [a for a in dir(lc)
            if any(w in a.lower() for w in ("curvature", "demand", "gain", "accel"))]
  assert banned == []
