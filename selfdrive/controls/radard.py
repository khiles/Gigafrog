#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from types import SimpleNamespace
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.lib.desire_helper import LaneChangeDirection, LaneChangeState

from openpilot.frogpilot.common.frogpilot_variables import EGO_HALF_WIDTH, THRESHOLD, get_frogpilot_toggles


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# FrogPilot variables
MovingState = car.RadarData.RadarPoint.MovingState
ObjectClass = car.RadarData.RadarPoint.ObjectClass

# Static roadside obstacle qualification
STATIC_SPEED_THRESHOLD = 0.8    # m/s, absolute world-frame speed below this is static
OVERHEAD_DZ = 1.6               # m, above this is a bridge/gantry/overhead sign
UNDERPASS_DZ = -0.8             # m, below this is a manhole cover or road furniture
MIN_OBSTACLE_LENGTH = 0.30      # m
MAX_OBSTACLE_LENGTH = 7.0       # m, the DBC signal saturates at 7.875
MIN_PROB_EXIST = 50.0           # %
MAX_PROB_NON_OBSTACLE = 50.0    # %
MIN_STATIC_DREL = 1.0           # m

# Oncoming traffic qualification
ONCOMING_SPEED_THRESHOLD = -2.5  # m/s world-frame; more negative means travelling toward us
ONCOMING_MAX_TTC = 6.0           # s
ONCOMING_MIN_LATERAL = 0.5       # m left of the predicted path before it counts as oncoming
ONCOMING_MIN_DREL = 2.0          # m
ONCOMING_MAX_DREL = 120.0        # m
ONCOMING_LATCH_S = 4.0           # s hold after the last sighting


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    # FrogPilot variables
    self.leadLeft = False
    self.leadRight = False

    self.leadTrackID = 0

    self.radarfulFilter = FirstOrderFilter(0, 1, self.K_A[0][1])

    # Extended radar attributes, only meaningful when self.extended is True
    self.extended = False
    self.movingState = MovingState.indeterminate
    self.objectClass = ObjectClass.unknown
    self.length = 0.0
    self.dZ = 0.0
    self.probExist = 0.0
    self.probNonObstacle = 0.0

    self.staticFilter = FirstOrderFilter(0.0, 0.5, DT_MDL)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float, ext=None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # Copy the extended attributes out as plain scalars — the capnp reader's buffer is recycled
    if ext is not None:
      (self.extended, self.movingState, self.objectClass,
       self.length, self.dZ, self.probExist, self.probNonObstacle) = ext

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret

  # FrogPilot variables
  def potential_blindspot(self, left: bool) -> bool:
    """True when this track occupies the blind spot zone on the given side.

    Provides software BSM for vehicles without OEM hardware sensors by checking
    whether a radar track is positioned alongside the car:
      - Longitudinal: from 5 m behind to 30 m ahead of the ego vehicle
      - Lateral: 1.5 – 4.5 m from centre (roughly one lane width each side)
      - Speed: track must be moving (vLeadK >= 1 m/s) to exclude parked cars

    ``yRel`` is stored as -LAT_DIST in the openpilot convention, so
    ``-yRel`` gives the lateral position in the standard vehicle frame
    (positive = left, negative = right).
    """
    if self.vLeadK < 1.0:
      return False
    if not (-5.0 < self.dRel < 30.0):
      return False
    lat_pos = -self.yRel
    if left:
      return 1.5 < lat_pos < 4.5
    else:
      return -4.5 < lat_pos < -1.5

  def potential_adjacent_lead(self, left: bool, model_data: capnp._DynamicStructReader):
    if self.vLeadK < 1 or self.leadTrackID == self.identifier:
      return False

    far_left_lane = np.interp(self.dRel, model_data.laneLines[0].x, model_data.laneLines[0].y)
    left_lane = np.interp(self.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
    right_lane = np.interp(self.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)
    far_right_lane = np.interp(self.dRel, model_data.laneLines[3].x, model_data.laneLines[3].y)

    self.leadLeft = far_left_lane < -self.yRel < left_lane and self.dRel < model_data.position.x[-1]
    self.leadRight = right_lane < -self.yRel < far_right_lane and self.dRel < model_data.position.x[-1]

    if left:
      return self.leadLeft
    else:
      return self.leadRight

  def path_lateral(self, model_data: capnp._DynamicStructReader) -> float:
    """Signed lateral offset of this track from the predicted ego path, in metres.

    Measured against the *path* rather than the car centreline so the result stays
    correct through curves. Returned in the model/calibrated frame, where positive is
    right — ``yRel`` is left-positive, hence the negation.
    """
    y_obj = -self.yRel
    y_path = float(np.interp(self.dRel, model_data.position.x, model_data.position.y))
    return y_obj - y_path

  def path_clearance(self, model_data: capnp._DynamicStructReader) -> float:
    """Lateral gap in metres between the ego body edge and this track."""
    return abs(self.path_lateral(model_data)) - EGO_HALF_WIDTH

  def is_static_obstacle(self) -> bool:
    """True when this track is a confirmed stationary roadside object.

    Parked cars, wheelie bins, bollards, stopped cyclists. Radars that report
    ``movingState`` (currently only the Tesla Continental) are authoritative; everything
    else defaults to ``indeterminate`` and falls through to the speed test, so no
    per-brand check is needed here.
    """
    if not self.measured or self.cnt < 5:
      return False

    if self.extended:
      if self.probExist < MIN_PROB_EXIST or self.probNonObstacle > MAX_PROB_NON_OBSTACLE:
        return False
      # Height gate: reject bridges, gantries, overhead signs and low road furniture
      if not (UNDERPASS_DZ < self.dZ < OVERHEAD_DZ):
        return False
      # Size gate. Length == 0 means the radar didn't segment it, which isn't disqualifying
      if self.length > 0.0 and not (MIN_OBSTACLE_LENGTH <= self.length <= MAX_OBSTACLE_LENGTH):
        return False

    if self.movingState in (MovingState.standing, MovingState.stopped):
      return True
    if self.movingState == MovingState.moving:
      return False
    return abs(self.vLeadK) < STATIC_SPEED_THRESHOLD

  def is_oncoming(self, model_data: capnp._DynamicStructReader, left_hand_traffic: bool = False) -> bool:
    """True when this track is traffic approaching us in the oncoming lane.

    Which side that is depends on the country: oncoming is on the left under right-hand
    traffic and on the right under left-hand traffic (UK, Ireland, Japan, Australia).

    ``vLeadK`` is world-frame absolute speed, so an approaching vehicle reads negative.
    Class is only used to exclude pedestrians — a motorcycle closing at 15 m/s must count.
    """
    if self.movingState in (MovingState.standing, MovingState.stopped):
      return False
    if self.movingState != MovingState.moving and abs(self.vLeadK) <= STATIC_SPEED_THRESHOLD:
      return False
    if self.vLeadK > ONCOMING_SPEED_THRESHOLD:
      return False
    if self.objectClass == ObjectClass.pedestrian:
      return False
    if not (ONCOMING_MIN_DREL < self.dRel < ONCOMING_MAX_DREL):
      return False

    # path_lateral is right-positive, so oncoming sits positive under left-hand traffic
    lateral = self.path_lateral(model_data)
    if left_hand_traffic:
      if lateral < ONCOMING_MIN_LATERAL:
        return False
    elif lateral > -ONCOMING_MIN_LATERAL:
      return False

    return self.dRel / max(-self.vRel, 0.1) < ONCOMING_MAX_TTC

  def potential_far_lead(self, lead_msg: capnp._DynamicStructReader, model_data: capnp._DynamicStructReader):
    left_lane = np.interp(self.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
    right_lane = np.interp(self.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)

    if left_lane < -self.yRel < right_lane and self.dRel < model_data.position.x[-1] and self.vLeadK > 1:
      self.radarfulFilter.update(1)
      return True
    else:
      self.radarfulFilter.update(0)
      return False


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, model_data: capnp._DynamicStructReader, tracks: dict[int, Track], frogpilot_toggles: SimpleNamespace):
  # FrogPilot variables
  if model_data.meta.laneChangeState == LaneChangeState.laneChangeStarting and frogpilot_toggles.human_lane_changes:
    direction = model_data.meta.laneChangeDirection

    if direction == LaneChangeDirection.left:
      left_tracks = [track for track in tracks.values() if track.leadLeft]
      if left_tracks:
        return min(left_tracks, key=lambda c: c.dRel)

    elif direction == LaneChangeDirection.right:
      right_tracks = [track for track in tracks.values() if track.leadRight]
      if right_tracks:
        return min(right_tracks, key=lambda c: c.dRel)

  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, model_data: capnp._DynamicStructReader,
             frogpilot_plan: capnp._DynamicStructReader, frogpilot_toggles: SimpleNamespace,
             low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > frogpilot_toggles.lead_detection_probability:
    track = match_vision_to_track(v_ego, lead_msg, model_data, tracks, frogpilot_toggles)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
  elif (track is None) and ready and (lead_msg.prob > frogpilot_toggles.lead_detection_probability):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  if low_speed_override and not lead_dict['status'] and len(tracks) > 0:
    far_lead_tracks = [c for c in tracks.values() if c.potential_far_lead(lead_msg, model_data) and c.radarfulFilter.x >= THRESHOLD]
    if len(far_lead_tracks) > 0:
      closest_track = min(far_lead_tracks, key=lambda c: c.dRel)
      lead_dict = closest_track.get_RadarState()

  # FrogPilot variables
  for track in tracks.values():
    track.leadTrackID = lead_dict.get('radarTrackId', -1)

  if 'dRel' in lead_dict:
    lead_dict['dRel'] -= frogpilot_plan.increasedStoppedDistance

  return lead_dict


# FrogPilot variables
def get_adjacent_lead(tracks: dict[int, Track], model_data: capnp._DynamicStructReader, left: bool = True) -> dict[str, Any]:
  lead_dict = {'status': False}

  adjacent_tracks = [c for c in tracks.values() if c.potential_adjacent_lead(left, model_data)]
  if len(adjacent_tracks) > 0:
    closest_track = min(adjacent_tracks, key=lambda c: c.dRel)
    lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_static_obstacle(tracks: dict[int, Track], model_data: capnp._DynamicStructReader,
                        trigger_distance: float, left: bool = True) -> dict[str, Any]:
  """Summarise the confirmed static objects on one side of the predicted path.

  Reports the *worst* object rather than the nearest: the threat is whichever one pinches
  the path hardest, which is not necessarily the closest. ``count`` lets the planner tell
  a single wheelie bin apart from a continuous row of parked cars.
  """
  obstacle = {'detected': False, 'count': 0}

  model_length = model_data.position.x[-1] if len(model_data.position.x) else 0.0

  candidates = []
  for track in tracks.values():
    if track.staticFilter.x < THRESHOLD:
      continue
    if not (MIN_STATIC_DREL < track.dRel < min(trigger_distance, model_length)):
      continue
    # path_lateral is right-positive, so a left-side object sits at a negative value
    if (track.path_lateral(model_data) < 0.0) != left:
      continue
    candidates.append(track)

  if candidates:
    worst = min(candidates, key=lambda c: c.path_clearance(model_data))
    obstacle = {
      'detected': True,
      'dRel': float(worst.dRel),
      'yRel': float(worst.yRel),
      'clearance': float(worst.path_clearance(model_data)),
      'count': min(len(candidates), 255),
    }

  return obstacle


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    # FrogPilot variables
    self.frogpilot_radar_state = custom.FrogPilotRadarState.new_message()

    self.oncoming_hold_t = 0.0

    self.frogpilot_toggles = get_frogpilot_toggles()

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured,
                           (pt.extended, pt.movingState, pt.objectClass,
                            pt.length, pt.dZ, pt.probExist, pt.probNonObstacle)]
              for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], rpt[4])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, sm['modelV2'], sm['frogpilotPlan'], self.frogpilot_toggles, low_speed_override=True)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, sm['modelV2'], sm['frogpilotPlan'], self.frogpilot_toggles, low_speed_override=False)

    # FrogPilot variables
    if self.ready and (self.frogpilot_toggles.adjacent_lead_tracking or self.frogpilot_toggles.human_lane_changes):
      self.frogpilot_radar_state.leadLeft = get_adjacent_lead(self.tracks, sm['modelV2'], left=True)
      self.frogpilot_radar_state.leadRight = get_adjacent_lead(self.tracks, sm['modelV2'], left=False)

    # Software BSM: radar-based blind spot detection for vehicles without OEM hardware.
    # Runs unconditionally; selfdrived and UI only use it when hardware BSM is absent.
    if self.ready:
      self.frogpilot_radar_state.softwareBsmLeft = any(
        t.potential_blindspot(left=True) for t in self.tracks.values()
      )
      self.frogpilot_radar_state.softwareBsmRight = any(
        t.potential_blindspot(left=False) for t in self.tracks.values()
      )

    # Static roadside obstacles and oncoming traffic for the obstacle nudge
    if self.ready and self.frogpilot_toggles.obstacle_nudge and len(sm['modelV2'].position.x):
      model_data = sm['modelV2']

      for track in self.tracks.values():
        track.staticFilter.update(1.0 if track.is_static_obstacle() else 0.0)

      trigger_distance = self.frogpilot_toggles.obstacle_nudge_trigger_distance
      self.frogpilot_radar_state.staticObstacleLeft = get_static_obstacle(self.tracks, model_data, trigger_distance, left=True)
      self.frogpilot_radar_state.staticObstacleRight = get_static_obstacle(self.tracks, model_data, trigger_distance, left=False)

      # Max-hold latch. It only ever extends "occupied", so a few dropped radar frames
      # can never unlatch it — flicker-proof in the direction that matters.
      left_hand_traffic = self.frogpilot_toggles.left_hand_traffic
      if any(t.is_oncoming(model_data, left_hand_traffic) for t in self.tracks.values()):
        self.oncoming_hold_t = ONCOMING_LATCH_S
      else:
        self.oncoming_hold_t = max(0.0, self.oncoming_hold_t - DT_MDL)
      self.frogpilot_radar_state.oncomingDetected = self.oncoming_hold_t > 0.0
    else:
      # The message is reused every cycle, so clear it rather than leaving stale detections
      self.oncoming_hold_t = 0.0
      self.frogpilot_radar_state.staticObstacleLeft = {'detected': False, 'count': 0}
      self.frogpilot_radar_state.staticObstacleRight = {'detected': False, 'count': 0}
      self.frogpilot_radar_state.oncomingDetected = False

    self.frogpilot_toggles = get_frogpilot_toggles(sm)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

    # FrogPilot variables
    frogpilot_radar_msg = messaging.new_message("frogpilotRadarState")
    frogpilot_radar_msg.valid = self.radar_state_valid
    frogpilot_radar_msg.frogpilotRadarState = self.frogpilot_radar_state
    pm.send("frogpilotRadarState", frogpilot_radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2',
                           ignore_valid=['frogpilotPlan'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  # FrogPilot variables
  sm = sm.extend(['frogpilotPlan'])
  pm = pm.extend(['frogpilotRadarState'])

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
