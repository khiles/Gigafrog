#!/usr/bin/env python3
"""Warn that a curve is coming, before the car starts slowing for it.

The Curve Speed Controller has always slowed for curves, but nothing told the driver it was about
to — the on-screen curve widget only appears once CSC is already reducing speed, so the first sign
is the deceleration itself. Everything needed to say it earlier was already being computed and
thrown away.

Two sources, because they see different distances:

  * the model — `time_to_curve` from calculate_road_curvature, the time to the sharpest point of
    the predicted path. Always available, but limited to the model's own ~192m horizon.
  * the car — Tesla's UI_csaRoadCurvature, a map-derived curvature valid over a stated range of
    up to 510m. Much further ahead, but only on cars that send it.

Whichever sees a curve first wins, and the source is reported so a wrong map reading is visible on
screen rather than silently trusted.

Severity is predicted lateral acceleration at the current speed, v^2 * |curvature| — the same
quantity CSC tunes against. That makes the threshold mean "sharp enough that you would want to
slow" rather than "the road bends a bit", and it makes the warning arrive earlier at speed, which
is the correct behaviour.

Display only. Nothing here feeds control; CSC does the slowing exactly as it did before. That is
deliberate: it means a wrong warning from the unverified map branch is annoying rather than
dangerous.
"""
from openpilot.common.realtime import DT_MDL

# Warn when the predicted lateral acceleration through the curve exceeds this. Roughly the point
# where a curve stops being a gentle bend and starts being something you lift for.
CURVE_MIN_LATERAL_ACCEL = 1.8   # m/s^2

# Only warn about curves this close in time — further out is noise, and the model's horizon is
# only about 10s anyway.
CURVE_MAX_TIME = 12.0           # s
# And no closer than this: below it you are effectively already in the curve and can see it, so a
# "curve ahead" warning is pointless. Both sources are held to it.
CURVE_MIN_TIME = 1.5            # s
CURVE_MIN_SPEED = 8.0           # m/s, below this a "curve ahead" warning is meaningless

# Once shown, hold it briefly so a curve flickering in and out of the threshold does not strobe
# the pill on and off.
CURVE_HOLD = 1.5                # s

SOURCE_NONE, SOURCE_MODEL, SOURCE_MAP = 0, 1, 2


def _severity(curvature, v_ego):
  """Predicted lateral acceleration through a curve of this curvature at this speed."""
  return v_ego**2 * abs(curvature)


def map_curve_onset(c2, c3, range_m, min_curvature):
  """How far ahead the map says the road first reaches `min_curvature`, or None within the range.

  The DBC gives C2 in 1/m and C3 in 1/m^2 (tesla_can.dbc:476-478), so curvature is linear in
  distance: curvature(x) = C2 + C3*x. That makes this solvable rather than something to sample.

  Note this asks where the curve becomes SHARP, not where curvature peaks. Those differ, and the
  difference matters: with C3 = 0 the peak is at x = 0, which would report a curve you are already
  driving through as the most imminent warning available. What the driver wants is how far until
  it tightens.
  """
  # Already at or beyond it — the curve is here, not ahead
  if abs(c2) >= min_curvature:
    return 0.0, abs(c2)

  if c3 == 0.0:
    return None   # constant and below threshold: never gets sharp within the range

  # Linear, so solve directly for each sign and take the nearest crossing that lies in range
  candidates = [(k - c2) / c3 for k in (min_curvature, -min_curvature)]
  ahead = sorted(x for x in candidates if 0.0 <= x <= range_m)
  if not ahead:
    return None
  x = ahead[0]
  return x, abs(c2 + c3 * x)


class FrogPilotCurveAhead:
  def __init__(self):
    self.curve_ahead = False
    self.time_to_curve = 0.0
    self.distance_to_curve = 0.0
    self.lateral_accel = 0.0
    self.source = SOURCE_NONE

    self.hold_t = 0.0

  def update(self, v_ego, road_curvature, time_to_curve, car_state, use_map):
    if v_ego < CURVE_MIN_SPEED:
      self.reset()
      return

    best = None   # (time, distance, lateral accel, source)

    # Model. road_curvature and time_to_curve are already computed every frame by the planner;
    # this is the first thing to actually use time_to_curve for anything the driver can see.
    accel = _severity(road_curvature, v_ego)
    if accel >= CURVE_MIN_LATERAL_ACCEL and CURVE_MIN_TIME <= time_to_curve <= CURVE_MAX_TIME:
      best = (time_to_curve, time_to_curve * v_ego, accel, SOURCE_MODEL)

    # Car's map. Gated on the liveness flag the carstate reader already sets, so on a car that
    # does not send UI_csaRoadCurvature this branch simply never contributes.
    if use_map and car_state.mapCurvatureValid and car_state.mapCurvatureRange > 0:
      # Curvature that would produce the threshold acceleration at this speed
      min_curvature = CURVE_MIN_LATERAL_ACCEL / max(v_ego, 1.0)**2
      onset = map_curve_onset(car_state.mapCurvatureC2, car_state.mapCurvatureC3,
                              car_state.mapCurvatureRange, min_curvature)
      if onset is not None:
        distance, curvature = onset
        time_ahead = distance / max(v_ego, 1.0)
        if CURVE_MIN_TIME <= time_ahead <= CURVE_MAX_TIME:
          # Earlier warning wins — the whole point of a second, longer-range source
          if best is None or time_ahead < best[0]:
            best = (time_ahead, distance, _severity(curvature, v_ego), SOURCE_MAP)

    if best is not None:
      self.time_to_curve, self.distance_to_curve, self.lateral_accel, self.source = best
      self.curve_ahead = True
      self.hold_t = CURVE_HOLD
    elif self.hold_t > 0:
      self.hold_t -= DT_MDL   # hold the last warning briefly rather than strobing at the threshold
    else:
      self.reset()

  def reset(self):
    self.curve_ahead = False
    self.time_to_curve = 0.0
    self.distance_to_curve = 0.0
    self.lateral_accel = 0.0
    self.source = SOURCE_NONE
    self.hold_t = 0.0
