#!/usr/bin/env python3
"""Measured lane position — where the car actually sits between the lane lines.

Nothing in this fork consumes lane-line *position*. modeld derives desired curvature from the
model's predicted yaw trajectory (`get_curvature_from_plan`, modeld.py:59-63, which uses two of
the fifteen plan channels), and controlsd only blends and rate-limits it. So the car's offset
within the lane is unmeasured and unreported, which makes "it doesn't sit centred" impossible to
diagnose.

The measurement is always on. The trim that acts on it is off by default and separately gated —
see below.

Sign convention: the model's lateral frame is right-positive. laneLines[1] is the ego lane's
left line and laneLines[2] the right, so a centred car reads left ~= -half_width and
right ~= +half_width, and `offset` is ~0. If the car sits left of centre both lines shift
positive, so **positive offset means the lane centre is to the right of the car**, i.e. the car
is left of where it should be.

That sign is derived, not measured. Confirm it against a drive before anything acts on it.

The trim
--------
Real-world testing confirmed a consistent left bias on a UK right-hand-drive car, which is what
the trim corrects. It is proportional only, and that is not a simplification:

The model is a closed loop that cannot be opened — it servos the car to *its* idea of centre, so
a bias added downstream is actively opposed. An integrator would wind up fighting it until it
saturated. Proportional accepts a partial correction and stays stable, which also means the
steady-state shift is a fraction of the demand, set by the ratio of our gain to the model's.

The retired obstacle nudge also established that the demand has to be an absolute setpoint in the
lane-centre frame; a relative demand mixed with a frame that moves with the car produced a limit
cycle. `lane_offset` is measured from lane centre, so it already is one.

A likely root cause of the bias, worth ruling out before tuning the gain up: openpilot assumes
the camera sits on the vehicle centreline, and liveCalibration learns pitch and yaw but not
lateral translation. A device mounted left of centre makes the model believe the car is further
right than it is, and it steers left to compensate. If that is the case, remounting fixes it
properly and the trim is only papering over it.
"""
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

# The point is systematic offset, not chasing noise, so the reported value is slow.
OFFSET_TAU = 3.0

MIN_LANE_LINE_PROB = 0.5   # matches ldw.py's visibility threshold
MIN_SPEED = 5.0            # m/s
MIN_LANE_WIDTH = 2.5       # m, narrower than this and we are not looking at a lane
MAX_LANE_WIDTH = 4.5       # m

# Trim. Still modest — this corrects a standing bias, it is not a lane-keeping controller, and it
# works against a model that actively pushes back.
#
# The first values were too weak to fix a real kerb-hugging bias, and worse, the cap was the
# binding constraint rather than the gain: at a 0.4m offset the demand hit 0.3 m/s^2 by dial 1.43,
# so turning the dial higher did nothing at all. Both were raised together so the dial has real
# range across its span. 1.0 m/s^2 is still well under the ~3.0 m/s^2 the curvature limiter allows
# and the ~3.6 the car itself permits; at 20 m/s it is a 400m-radius drift, not a swerve.
TRIM_DEADBAND = 0.05       # m, below this there is nothing worth correcting
TRIM_GAIN = 1.2            # (m/s^2) per m of offset, before the user's own gain
TRIM_MAX_ACCEL = 1.0       # m/s^2, hard cap on the demand
# How fast the demand may change. A standing bias does not appear suddenly, so a correction for
# one has no business moving quickly — and this is what stops the trim wandering. Raising the gain
# to fix kerb-hugging made the correction strong enough to visibly fight the model, and a
# proportional term fighting a closed loop is exactly what oscillates. Slew-limiting it means the
# trim physically cannot move at the frequencies that feel like the car hunting around.
TRIM_SLEW = 0.15           # m/s^2 per second — about 7s from nothing to the cap


class FrogPilotLaneCentering:
  def __init__(self):
    self.lane_offset = 0.0            # instantaneous, m, right-positive
    self.lane_offset_filtered = 0.0   # slow, m — the one worth reading
    self.lane_offset_valid = False
    self.lane_width = 0.0
    self.trim_lateral_accel = 0.0     # m/s^2, right-positive; 0 unless the trim is enabled

    self.offset_filter = FirstOrderFilter(0.0, OFFSET_TAU, DT_MDL)

  def update_trim(self, enabled, gain):
    """Proportional correction toward lane centre. Deliberately has no integrator — see above."""
    if not enabled:
      self.trim_lateral_accel = 0.0   # switched off means off immediately, not a ramp
      return

    # Losing the measurement or entering the deadband releases the trim, but at the same bounded
    # rate — snapping to zero would be a step change in steering, which is the thing being avoided.
    if not self.lane_offset_valid or abs(self.lane_offset_filtered) < TRIM_DEADBAND:
      self._slew_to(0.0)
      return

    error = self.lane_offset_filtered

    # Positive offset means the lane centre is to the right, so the correction is to the right,
    # which is also positive in this frame — no sign flip.
    demand = (error - math.copysign(TRIM_DEADBAND, error)) * TRIM_GAIN * gain
    demand = float(np.clip(demand, -TRIM_MAX_ACCEL, TRIM_MAX_ACCEL))
    self._slew_to(demand)

  def _slew_to(self, demand):
    """Move toward the demand at a bounded rate rather than jumping to it."""
    step = TRIM_SLEW * DT_MDL
    self.trim_lateral_accel = float(np.clip(demand,
                                            self.trim_lateral_accel - step,
                                            self.trim_lateral_accel + step))

  def update(self, model_v2, v_ego, lane_change_active):
    lane_lines = model_v2.laneLines
    probs = model_v2.laneLineProbs

    # Both ego-lane lines have to be there. Without a gate an unseen line regresses to
    # something plausible-looking and the offset becomes confident nonsense.
    if len(lane_lines) < 3 or len(probs) < 3:
      self.reset()
      return

    if min(probs[1], probs[2]) < MIN_LANE_LINE_PROB or v_ego < MIN_SPEED or lane_change_active:
      self.reset()
      return

    left_y = lane_lines[1].y[0]
    right_y = lane_lines[2].y[0]
    lane_width = right_y - left_y

    if not MIN_LANE_WIDTH <= lane_width <= MAX_LANE_WIDTH:
      self.reset()
      return

    self.lane_width = float(lane_width)
    self.lane_offset = float((left_y + right_y) / 2.0)
    self.offset_filter.update(self.lane_offset)
    self.lane_offset_filtered = float(self.offset_filter.x)
    self.lane_offset_valid = True

  def reset(self):
    self.lane_offset = 0.0
    self.lane_offset_filtered = 0.0
    self.lane_offset_valid = False
    self.lane_width = 0.0
    self.offset_filter.x = 0.0
