#!/usr/bin/env python3
"""Measured lane position — where the car actually sits between the lane lines.

Nothing in this fork consumes lane-line *position*. modeld derives desired curvature from the
model's predicted yaw trajectory (`get_curvature_from_plan`, modeld.py:59-63, which uses two of
the fifteen plan channels), and controlsd only blends and rate-limits it. So the car's offset
within the lane is unmeasured and unreported, which makes "it doesn't sit centred" impossible to
diagnose.

This measures it and nothing else. There is deliberately no control output here — see the note
on the model being a closed loop, below.

Sign convention: the model's lateral frame is right-positive. laneLines[1] is the ego lane's
left line and laneLines[2] the right, so a centred car reads left ~= -half_width and
right ~= +half_width, and `offset` is ~0. If the car sits left of centre both lines shift
positive, so **positive offset means the lane centre is to the right of the car**, i.e. the car
is left of where it should be.

That sign is derived, not measured. Confirm it against a drive before anything acts on it.

Why no controller lives here yet
--------------------------------
The model is a closed loop that cannot be opened: it servos the car to *its* idea of centre. A
bias added downstream is actively opposed — the model sees the resulting offset and corrects
back. Two consequences that would otherwise be found the hard way:

  * An integrator winds up fighting the model until it saturates. Any trim must be proportional
    only.
  * Steady-state authority is a fraction of the demand, set by the ratio of our gain to the
    model's, so a capped demand shifts the car less than the arithmetic suggests.

The retired obstacle nudge also established that the demand has to be an absolute setpoint in the
lane-centre frame; a relative demand mixed with a frame that moves with the car produced a limit
cycle.
"""
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

# The point is systematic offset, not chasing noise, so the reported value is slow.
OFFSET_TAU = 3.0

MIN_LANE_LINE_PROB = 0.5   # matches ldw.py's visibility threshold
MIN_SPEED = 5.0            # m/s
MIN_LANE_WIDTH = 2.5       # m, narrower than this and we are not looking at a lane
MAX_LANE_WIDTH = 4.5       # m


class FrogPilotLaneCentering:
  def __init__(self):
    self.lane_offset = 0.0            # instantaneous, m, right-positive
    self.lane_offset_filtered = 0.0   # slow, m — the one worth reading
    self.lane_offset_valid = False
    self.lane_width = 0.0

    self.offset_filter = FirstOrderFilter(0.0, OFFSET_TAU, DT_MDL)

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
