#!/usr/bin/env python3
"""Live diagnostic for the obstacle nudge.

Run onroad while driving past parked cars:

    python frogpilot/tools/debug_obstacle_nudge.py

Walks the whole chain in order and prints where it stops:
  toggles -> raw radar -> static classification -> radard summary -> planner -> controlsd
"""
import time

import cereal.messaging as messaging

from cereal import car
from openpilot.common.params import Params
from openpilot.frogpilot.common.frogpilot_variables import EGO_HALF_WIDTH, get_frogpilot_toggles

MovingState = car.RadarData.RadarPoint.MovingState

MOVING_STATE_NAMES = {MovingState.indeterminate: "indet", MovingState.moving: "moving",
                      MovingState.stopped: "stopped", MovingState.standing: "standing"}


def main():
  params = Params()
  print("CarParams present:", params.get("CarParams") is not None)

  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  print(f"fingerprint={CP.carFingerprint}  radarUnavailable={CP.radarUnavailable}")
  if CP.radarUnavailable:
    print("\n!! radarUnavailable is True — the feature is gated off entirely. Stop here.")
    return

  sm = messaging.SubMaster(["carState", "carControl", "controlsState", "liveTracks", "modelV2",
                           "radarState", "frogpilotPlan", "frogpilotRadarState"])

  print("\nRaw param values:")
  for key in ("ObstacleNudge", "ObstacleNudgeCrossCenterLine", "ObstacleNudgeGain",
              "ObstacleNudgeMaxCenterLineOvershoot", "ObstacleNudgeMaxOffset",
              "ObstacleNudgeMaxSpeed", "ObstacleNudgeMinClearance", "ObstacleNudgeMinSpeed",
              "ObstacleNudgeTriggerDistance"):
    print(f"  {key:38s} = {params.get(key)}")
  print(f"  {'IsMetric':38s} = {params.get_bool('IsMetric')}")
  print(f"  {'TuningLevel':38s} = {params.get('TuningLevel')}")

  printed_toggles = False
  last = 0.0

  while True:
    sm.update(100)
    if not sm.updated["modelV2"]:
      continue

    toggles = get_frogpilot_toggles(sm)

    if not printed_toggles:
      print("\nResolved toggles (what the code actually sees, SI units):")
      for name in sorted(vars(toggles)):
        if name.startswith("obstacle_nudge"):
          print(f"  {name:44s} = {getattr(toggles, name)}")
      if not getattr(toggles, "obstacle_nudge", False):
        print("\n!! obstacle_nudge is False. Either the toggle is off, TuningLevel < 3,")
        print("   or CarParams says no radar. Nothing downstream will run.")
      printed_toggles = True

    if time.monotonic() - last < 0.5:
      continue
    last = time.monotonic()

    CS = sm["carState"]
    v_ego = CS.vEgo
    md = sm["modelV2"]
    frs = sm["frogpilotRadarState"]
    fp = sm["frogpilotPlan"]

    points = sm["liveTracks"].points
    extended = sum(1 for p in points if p.extended)
    states: dict[str, int] = {}
    for p in points:
      name = MOVING_STATE_NAMES.get(p.movingState, "?")
      states[name] = states.get(name, 0) + 1

    in_speed = toggles.obstacle_nudge_min_speed <= v_ego <= toggles.obstacle_nudge_max_speed

    print("\n" + "=" * 78)
    print(f"v_ego={v_ego:5.1f} m/s ({v_ego * 2.237:4.1f} mph)   latActive={sm['carControl'].latActive}   " +
          f"speed gate [{toggles.obstacle_nudge_min_speed:.1f}, {toggles.obstacle_nudge_max_speed:.1f}] -> {in_speed}")
    if not in_speed:
      print("  !! OUT OF SPEED RANGE — the nudge is held at zero.")

    print(f"radar: {len(points):2d} points, {extended} extended, movingState {states}")
    if points and extended == 0:
      print("  !! No point reports extended=True. The Tesla radar_interface isn't populating")
      print("     the new fields - check that opendbc rebuilt and this is the continental radar.")

    # What the classifier sees, before radard's own filtering
    print("  nearest few points (dRel, yRel, vRel, state, dZ, len, pExist, pNonObs):")
    for p in sorted(points, key=lambda q: q.dRel)[:6]:
      print(f"    d={p.dRel:6.1f}  y={p.yRel:+6.2f}  v={p.vRel:+6.1f}  " +
            f"{MOVING_STATE_NAMES.get(p.movingState, '?'):8s} dZ={p.dZ:+5.2f} " +
            f"len={p.length:4.1f} pE={p.probExist:5.1f} pNO={p.probNonObstacle:5.1f}")

    left, right = frs.staticObstacleLeft, frs.staticObstacleRight
    print(f"radard summary: LEFT det={left.detected} n={left.count} clearance={left.clearance:+.2f} d={left.dRel:.1f}  |  " +
          f"RIGHT det={right.detected} n={right.count} clearance={right.clearance:+.2f} d={right.dRel:.1f}")
    print(f"                oncomingDetected={frs.oncomingDetected}")
    if not left.detected and not right.detected and extended:
      print("  !! Radar sees points but nothing confirmed static. The gates in")
      print("     Track.is_static_obstacle are rejecting them — compare the columns above.")

    need_r = max(0.0, toggles.obstacle_nudge_min_clearance - left.clearance) if left.detected else 0.0
    need_l = max(0.0, toggles.obstacle_nudge_min_clearance - right.clearance) if right.detected else 0.0
    print(f"demand: need_right={need_r:.2f} need_left={need_l:.2f} " +
          f"(min_clearance={toggles.obstacle_nudge_min_clearance:.2f}, ego half width={EGO_HALF_WIDTH})")

    print(f"planner: target={fp.nudgeOffsetTarget:+.3f} measured={fp.nudgeOffsetMeasured:+.3f} " +
          f"a_lat={fp.nudgeLateralAccel:+.3f} crossing={fp.nudgeCrossingCenterLine}")

    if md.laneLineProbs and md.laneLineStds:
      print(f"model: laneLineProbs[1,2]=({md.laneLineProbs[1]:.2f},{md.laneLineProbs[2]:.2f}) " +
            f"stds=({md.laneLineStds[1]:.2f},{md.laneLineStds[2]:.2f}) " +
            f"roadEdgeStds=({md.roadEdgeStds[0]:.2f},{md.roadEdgeStds[1]:.2f})")
      print(f"       laneLines y@0 = L{md.laneLines[1].y[0]:+.2f} R{md.laneLines[2].y[0]:+.2f}  " +
            f"roadEdges y@0 = L{md.roadEdges[0].y[0]:+.2f} R{md.roadEdges[1].y[0]:+.2f}")

    print(f"controlsd: desiredCurvature={sm['controlsState'].desiredCurvature:+.5f} " +
          f"model={md.action.desiredCurvature:+.5f} " +
          f"delta={sm['controlsState'].desiredCurvature - md.action.desiredCurvature:+.5f}")


if __name__ == "__main__":
  main()
