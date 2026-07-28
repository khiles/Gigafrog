import math
import pytest

from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib import frogpilot_pose as P


def measurement(x=0.0, y=0.0, z=0.0, valid=True):
  return SimpleNamespace(x=x, y=y, z=z, xStd=0.0, yStd=0.0, zStd=0.0, valid=valid)


def make_pose(yaw_rate=0.0, v_forward=0.0, pitch=0.0, vertical_accel=0.0,
              posenet_ok=True, sensors_ok=True, valid=True):
  return SimpleNamespace(
    orientationNED=measurement(y=pitch, valid=valid),
    velocityDevice=measurement(x=v_forward, valid=valid),
    accelerationDevice=measurement(z=vertical_accel, valid=valid),
    angularVelocityDevice=measurement(z=yaw_rate, valid=valid),
    posenetOK=posenet_ok,
    sensorsOK=sensors_ok,
  )


def test_cornering_is_the_centripetal_term():
  # 0.2 rad/s at 20 m/s is 4 m/s^2. accelerationDevice.y would read ~0 here, which is why
  # the derivation uses yaw rate x speed instead.
  pose = P.FrogPilotPose()
  pose.update(make_pose(yaw_rate=0.2, v_forward=20.0), v_ego=20.0)
  assert pose.cornering_acceleration == pytest.approx(4.0)


def test_cornering_sign_is_right_positive():
  # Device frame is [Forward, Right, Down]; positive yaw about Down is a right-hand turn
  pose = P.FrogPilotPose()
  pose.update(make_pose(yaw_rate=0.2, v_forward=20.0), v_ego=20.0)
  assert pose.cornering_acceleration > 0

  pose.update(make_pose(yaw_rate=-0.2, v_forward=20.0), v_ego=20.0)
  assert pose.cornering_acceleration < 0


def test_cornering_zero_below_min_speed():
  pose = P.FrogPilotPose()
  pose.update(make_pose(yaw_rate=0.5, v_forward=0.5), v_ego=0.5)
  assert pose.cornering_acceleration == 0.0


def test_gradient_is_a_fraction_and_clamped():
  pose = P.FrogPilotPose()
  pose.update(make_pose(pitch=math.atan(0.10), v_forward=20.0), v_ego=20.0)
  assert pose.road_gradient == pytest.approx(0.10, abs=1e-6)

  # A wild pitch is calibration error, not a 300% hill
  pose.update(make_pose(pitch=1.2, v_forward=20.0), v_ego=20.0)
  assert pose.road_gradient == pytest.approx(P.MAX_GRADIENT)


def test_roughness_ignores_a_constant_offset():
  # A steady vertical bias is not rough road; only variation about the mean counts.
  pose = P.FrogPilotPose()
  for _ in range(int(20 / DT_MDL)):
    pose.update(make_pose(vertical_accel=3.0, v_forward=20.0), v_ego=20.0)
  assert pose.road_roughness < 0.05


def test_roughness_responds_to_variation():
  pose = P.FrogPilotPose()
  for i in range(int(20 / DT_MDL)):
    bump = 2.0 if i % 4 < 2 else -2.0
    pose.update(make_pose(vertical_accel=bump, v_forward=20.0), v_ego=20.0)
  assert pose.road_roughness > 1.0


def test_smooth_road_reads_lower_than_rough_road():
  smooth, rough = P.FrogPilotPose(), P.FrogPilotPose()
  for i in range(int(20 / DT_MDL)):
    smooth.update(make_pose(vertical_accel=0.1 if i % 4 < 2 else -0.1, v_forward=20.0), v_ego=20.0)
    rough.update(make_pose(vertical_accel=3.0 if i % 4 < 2 else -3.0, v_forward=20.0), v_ego=20.0)
  assert rough.road_roughness > smooth.road_roughness * 5


def test_bad_sensors_zero_everything():
  pose = P.FrogPilotPose()
  for _ in range(int(5 / DT_MDL)):
    pose.update(make_pose(yaw_rate=0.2, v_forward=20.0, pitch=0.1, vertical_accel=2.0), v_ego=20.0)
  assert pose.cornering_acceleration != 0.0

  pose.update(make_pose(yaw_rate=0.2, v_forward=20.0, sensors_ok=False), v_ego=20.0)
  assert pose.cornering_acceleration == 0.0
  assert pose.road_gradient == 0.0
  assert pose.road_roughness == 0.0


def test_invalid_measurements_are_ignored():
  pose = P.FrogPilotPose()
  pose.update(make_pose(yaw_rate=0.2, v_forward=20.0, pitch=0.1, valid=False), v_ego=20.0)
  assert pose.cornering_acceleration == 0.0
  assert pose.road_gradient == 0.0
