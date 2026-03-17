import capnp
import numpy as np
from abc import abstractmethod, ABC
from types import SimpleNamespace

from openpilot.selfdrive.locationd.helpers import Pose


# Output authority ramps to 0 in DRIVER_BLEND_RAMP_S when driver overrides,
# and back to 1 in the same time after release.  Generalises Toyota's
# TORQUE_WIND_DOWN / MAX_LTA_DRIVER_TORQUE_ALLOWANCE pattern to all cars.
DRIVER_BLEND_RAMP_S = 0.3


class LatControl(ABC):
  def __init__(self, CP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    self.sat_check_min_speed = 10.

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

    # Driver torque blend: gradually reduce OP output when driver overrides so
    # there is no abrupt step when steeringPressed transitions.
    self._driver_blend = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, curvature_limited: bool, lat_delay: float, calibrated_pose: Pose, model_data: capnp._DynamicStructReader, frogpilot_toggles: SimpleNamespace):
    pass

  def reset(self):
    self.sat_time = 0.
    self._driver_blend = 1.0

  def _update_driver_blend(self, CS) -> float:
    """Ramp output authority [0..1] down when driver overrides, up when released.
    Returns the current blend scale to multiply against the torque output."""
    target = 0.0 if CS.steeringPressed else 1.0
    step = self.dt / DRIVER_BLEND_RAMP_S
    self._driver_blend = float(np.clip(
      self._driver_blend + np.sign(target - self._driver_blend) * step,
      0.0, 1.0
    ))
    return self._driver_blend

  def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
      self.sat_time += self.dt
    else:
      self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)
