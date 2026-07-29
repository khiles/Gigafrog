"""Demultiplexing UI_driverAssistRoadSign, and the liveness gate the map/solar reads depend on.

Two things make this worth testing rather than eyeballing:

1. The CAN parser has no multiplex support. dbc.py's SGM_RE matches the multiplexed signal form
   and then discards the multiplexor token, so UI_stopSignStopLineDist (m1),
   UI_trafficLightStopLineDist (m2), UI_baseMapSpeedLimitMPS (m3) and UI_meanFleetSplineSpeedMPS
   (m4) all decode off the same bits on every frame regardless of mode. Read naively, a map speed
   limit reads as a stop-line distance and vice versa. test_mode_does_not_leak is the guard.

2. Every CANParser on this platform is built with an empty message list, so a message that is not
   on the bus decodes silently to zero. Presence of these messages on a real Raven is unverified,
   so absent must mean "unknown", never a confident zero. The absent/stale tests are the primary
   signal here, not edge cases.
"""
from collections import defaultdict

from opendbc.car.tesla.carstate import CarState, ROAD_SIGN_MIN_CONF, _msg_alive

MSG = "UI_driverAssistRoadSign"
NOW = 10_000_000_000
FRESH = NOW - 100_000_000     # 100ms ago
STALE = NOW - 2_000_000_000   # 2s ago


class FakeVL(dict):
  """Mirrors VLDict: a message is registered on first __getitem__, which is also what
  populates ts_nanos and vl_all. Reading ts_nanos before that would KeyError."""
  def __init__(self, cp):
    super().__init__()
    self.cp = cp

  def __getitem__(self, key):
    if key not in self:
      self.cp.register(key)
    return super().__getitem__(key)


class FakeCP:
  def __init__(self, frames=None, ts=None, now=NOW):
    self._frames = frames or {}
    self._ts = ts or {}
    self.last_nonempty_nanos = now
    self.vl = FakeVL(self)
    self.vl_all: dict = {}
    self.ts_nanos: dict = {}

  def register(self, msg):
    dict.__setitem__(self.vl, msg, defaultdict(float))
    self.vl_all[msg] = self._frames.get(msg, defaultdict(list))
    self.ts_nanos[msg] = defaultdict(int, self._ts.get(msg, {}))


def frames(modes, **sigs):
  d = defaultdict(list)
  d["UI_roadSign"] = list(modes)
  for name, vals in sigs.items():
    d[name] = list(vals)
  return d


def holder():
  """A CarState with only the state update_road_sign touches, so this test does not need
  a CarParams or a full car interface."""
  cs = CarState.__new__(CarState)
  cs.road_sign = {}
  return cs


def rotation():
  """One full mode rotation, 0 through 4, with a distinct value in each mode."""
  return frames([0, 1, 2, 3, 4],
                UI_stopSignStopLineDist=[0, 25.0, 0, 0, 0],
                UI_stopSignStopLineConf=[0, 90, 0, 0, 0],
                UI_trafficLightStopLineDist=[0, 0, 42.25, 0, 0],
                UI_trafficLightStopLineConf=[0, 0, 88, 0, 0],
                UI_baseMapSpeedLimitMPS=[0, 0, 0, 13.4, 0],
                UI_meanFleetSplineSpeedMPS=[0, 0, 0, 0, 15.1])


# ---- liveness ----

def test_absent_message_is_not_alive():
  cs = holder()
  assert cs.update_road_sign(FakeCP()) is False
  assert cs.road_sign == {}


def test_stale_message_is_not_alive():
  cs = holder()
  cp = FakeCP(frames={MSG: rotation()}, ts={MSG: {"UI_roadSign": STALE}})
  assert cs.update_road_sign(cp) is False
  assert cs.road_sign == {}


def test_losing_the_message_clears_retained_values():
  # Stale map data must not keep being reported once the message stops arriving
  cs = holder()
  cs.update_road_sign(FakeCP(frames={MSG: rotation()}, ts={MSG: {"UI_roadSign": FRESH}}))
  assert cs.road_sign
  cs.update_road_sign(FakeCP())
  assert cs.road_sign == {}


def test_msg_alive_registers_lazily():
  # ts_nanos only exists after the message is registered, so _msg_alive must touch vl first
  cp = FakeCP(ts={"UI_solarData": {"UI_isSunUp": FRESH}})
  assert _msg_alive(cp, "UI_solarData", "UI_isSunUp") is True


def test_msg_alive_false_for_never_seen():
  assert _msg_alive(FakeCP(), "UI_solarData", "UI_isSunUp") is False


# ---- demultiplexing ----

def test_full_rotation_lands_in_the_right_fields():
  cs = holder()
  assert cs.update_road_sign(FakeCP(frames={MSG: rotation()},
                                    ts={MSG: {"UI_roadSign": FRESH}})) is True
  assert cs.road_sign == {"stopSignDistance": 25.0, "trafficLightDistance": 42.25,
                          "mapSpeedLimit": 13.4, "fleetMeanSpeed": 15.1}


def test_mode_does_not_leak():
  """The trap: mode 3's bits also decode as UI_stopSignStopLineDist with a high confidence.
  Only mapSpeedLimit may be set."""
  cs = holder()
  cp = FakeCP(frames={MSG: frames([3], UI_baseMapSpeedLimitMPS=[13.4],
                                  UI_stopSignStopLineDist=[13.4],
                                  UI_stopSignStopLineConf=[99])},
              ts={MSG: {"UI_roadSign": FRESH}})
  cs.update_road_sign(cp)
  assert cs.road_sign == {"mapSpeedLimit": 13.4}


def test_mode_missing_from_a_cycle_retains_its_value():
  # The modes rotate, so most cycles contain only some of them
  cs = holder()
  cs.update_road_sign(FakeCP(frames={MSG: frames([3], UI_baseMapSpeedLimitMPS=[13.4])},
                             ts={MSG: {"UI_roadSign": FRESH}}))
  cs.update_road_sign(FakeCP(frames={MSG: frames([4], UI_meanFleetSplineSpeedMPS=[15.1])},
                             ts={MSG: {"UI_roadSign": FRESH}}))
  assert cs.road_sign == {"mapSpeedLimit": 13.4, "fleetMeanSpeed": 15.1}


def test_low_confidence_is_rejected():
  cs = holder()
  cp = FakeCP(frames={MSG: frames([1], UI_stopSignStopLineDist=[25.0],
                                  UI_stopSignStopLineConf=[ROAD_SIGN_MIN_CONF - 1])},
              ts={MSG: {"UI_roadSign": FRESH}})
  cs.update_road_sign(cp)
  assert cs.road_sign == {}


def test_unknown_mode_is_ignored():
  cs = holder()
  cp = FakeCP(frames={MSG: frames([7, 15], UI_baseMapSpeedLimitMPS=[99.0, 99.0])},
              ts={MSG: {"UI_roadSign": FRESH}})
  assert cs.update_road_sign(cp) is True
  assert cs.road_sign == {}


def test_later_frame_wins_within_a_cycle():
  cs = holder()
  cp = FakeCP(frames={MSG: frames([3, 3], UI_baseMapSpeedLimitMPS=[13.4, 8.9])},
              ts={MSG: {"UI_roadSign": FRESH}})
  cs.update_road_sign(cp)
  assert cs.road_sign == {"mapSpeedLimit": 8.9}
