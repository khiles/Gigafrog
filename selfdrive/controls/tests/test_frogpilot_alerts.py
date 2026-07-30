"""Speeding and tailgating alert logic.

Both run off values that were already computed with no consumer: the speed limit
controller's target/offset/source, and the following controller's t_follow.

These are read from the planner objects, NOT from sm["frogpilotPlan"]. frogpilot_events
runs inside the process that *publishes* frogpilotPlan, so it is not in that SubMaster —
reading it there raised KeyError on the device. The fakes below mirror the real object
graph so that mistake cannot be re-made silently.
"""
import pytest

from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib import frogpilot_events as E


# Only what frogpilot_process actually subscribes to. Anything else must come from the
# planner, so a KeyError here is a real bug rather than a missing fake.
def make_sm(v_ego=20.0, lead_d_rel=60.0, lead_status=True, traffic_mode=False):
  return {
    "carState": SimpleNamespace(vEgo=v_ego),
    "frogpilotCarState": SimpleNamespace(trafficModeEnabled=traffic_mode),
    "radarState": SimpleNamespace(leadOne=SimpleNamespace(status=lead_status, dRel=lead_d_rel)),
  }


def make_planner(limit=20.0, offset=0.0, source="Map Data", limit_changed=False,
                 t_follow=1.45, tracking_lead=True):
  """Mirrors the real graph: planner -> frogpilot_vcruise -> slc, and -> frogpilot_following."""
  slc = SimpleNamespace(source=source,
                        speed_limit_changed_timer=(1.0 if limit_changed else 0.0))
  return SimpleNamespace(
    frogpilot_vcruise=SimpleNamespace(slc_target=limit, slc_offset=offset, slc=slc),
    frogpilot_following=SimpleNamespace(t_follow=t_follow),
    tracking_lead=tracking_lead,
  )


def toggles(**kw):
  base = dict(speed_limit_exceeded_alert=True, speed_limit_exceeded_margin=2.0, tailgating_alert=True)
  base.update(kw)
  return SimpleNamespace(**base)


class Events:
  """Stand-in for the Events container — records what was added."""
  def __init__(self):
    self.added = []

  def add(self, name):
    self.added.append(name)


def make_handler(tracking_lead=True, **planner_kwargs):
  handler = E.FrogPilotEvents.__new__(E.FrogPilotEvents)
  handler.events = Events()
  handler.frogpilot_planner = make_planner(tracking_lead=tracking_lead, **planner_kwargs)
  handler.speeding_t = 0.0
  handler.speeding_armed = True
  handler.speeding_since_alert = 0.0
  handler.tailgating_t = 0.0
  handler.tailgating_since_alert = E.TAILGATING_REPEAT
  return handler


PLANNER_KEYS = {"limit", "offset", "source", "limit_changed", "t_follow"}


def _apply_planner(handler, kwargs):
  """Planner-owned values live on the planner, not on sm."""
  planner_kwargs = {k: v for k, v in kwargs.items() if k in PLANNER_KEYS}
  if planner_kwargs:
    tracking = handler.frogpilot_planner.tracking_lead
    handler.frogpilot_planner = make_planner(tracking_lead=tracking, **planner_kwargs)
  return {k: v for k, v in kwargs.items() if k not in PLANNER_KEYS}


def run_speeding(handler, seconds, **kwargs):
  sm_kwargs = _apply_planner(handler, kwargs)
  for _ in range(int(seconds / DT_MDL)):
    handler.update_speeding(make_sm(**sm_kwargs), toggles())


def run_tailgating(handler, seconds, **kwargs):
  sm_kwargs = _apply_planner(handler, kwargs)
  for _ in range(int(seconds / DT_MDL)):
    handler.update_tailgating(make_sm(**sm_kwargs), toggles())


# ---- speeding ----

def test_speeding_fires_when_sustained_over():
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=30.0, limit=20.0)
  assert E.FrogPilotEventName.speedLimitExceeded in handler.events.added


def test_speeding_silent_when_no_limit_known():
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=40.0, limit=0.0, source="None")
  assert handler.events.added == []


def test_speeding_silent_while_limit_is_changing():
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=30.0, limit=20.0, limit_changed=True)
  assert handler.events.added == []


def test_speeding_respects_the_margin():
  # 21 m/s against a 20 m/s limit is inside the 2 m/s margin
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=21.0, limit=20.0)
  assert handler.events.added == []


def test_speeding_respects_the_offset():
  # offset raises the effective limit, so 24 against 20+3+2 is under
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=24.0, limit=20.0, offset=3.0)
  assert handler.events.added == []


def test_speeding_needs_to_be_sustained():
  handler = make_handler()
  run_speeding(handler, E.SPEEDING_SUSTAIN - 0.5, v_ego=30.0, limit=20.0)
  assert handler.events.added == []


def test_speeding_does_not_chatter_at_the_threshold():
  # Hovering either side of the limit must not produce a stream of alerts
  handler = make_handler()
  for i in range(int(60.0 / DT_MDL)):
    v_ego = 23.0 if (i // int(2.0 / DT_MDL)) % 2 == 0 else 21.5
    handler.update_speeding(make_sm(v_ego=v_ego), toggles())
  assert len(handler.events.added) <= 1


def test_speeding_rearms_after_slowing_properly():
  handler = make_handler()
  run_speeding(handler, 10.0, v_ego=30.0, limit=20.0)
  first = len(handler.events.added)
  run_speeding(handler, 10.0, v_ego=15.0, limit=20.0)   # clearly back under
  run_speeding(handler, 10.0, v_ego=30.0, limit=20.0)
  assert len(handler.events.added) > first


def test_speeding_off_when_toggle_off():
  handler = make_handler()
  for _ in range(int(10.0 / DT_MDL)):
    handler.update_speeding(make_sm(v_ego=30.0), toggles(speed_limit_exceeded_alert=False))
  assert handler.events.added == []


# ---- tailgating ----

def test_tailgating_fires_when_too_close():
  # 1.45s tFollow, fires below 65% of it; 15m at 20m/s is 0.75s
  handler = make_handler()
  run_tailgating(handler, 10.0, v_ego=20.0, lead_d_rel=15.0)
  assert E.FrogPilotEventName.tailgating in handler.events.added


def test_tailgating_silent_at_a_normal_gap():
  handler = make_handler()
  run_tailgating(handler, 10.0, v_ego=20.0, lead_d_rel=40.0)
  assert handler.events.added == []


def test_tailgating_suppressed_in_traffic_mode():
  handler = make_handler()
  run_tailgating(handler, 10.0, v_ego=20.0, lead_d_rel=10.0, traffic_mode=True)
  assert handler.events.added == []


def test_tailgating_suppressed_below_min_speed():
  handler = make_handler()
  run_tailgating(handler, 10.0, v_ego=5.0, lead_d_rel=3.0)
  assert handler.events.added == []


def test_tailgating_needs_a_tracked_lead():
  handler = make_handler(tracking_lead=False)
  run_tailgating(handler, 10.0, v_ego=20.0, lead_d_rel=10.0)
  assert handler.events.added == []

  handler = make_handler()
  run_tailgating(handler, 10.0, v_ego=20.0, lead_d_rel=10.0, lead_status=False)
  assert handler.events.added == []


def test_tailgating_does_not_nag_continuously():
  handler = make_handler()
  run_tailgating(handler, 120.0, v_ego=20.0, lead_d_rel=10.0)
  # 120s at a 30s repeat interval should be a handful, not hundreds
  assert 1 <= len(handler.events.added) <= 5


def test_tailgating_scales_with_the_drivers_own_gap():
  # The same 25m gap is fine at a short tFollow and too close at a long one
  short = make_handler()
  run_tailgating(short, 10.0, v_ego=20.0, lead_d_rel=25.0, t_follow=1.0)
  assert short.events.added == []

  long = make_handler()
  run_tailgating(long, 10.0, v_ego=20.0, lead_d_rel=25.0, t_follow=3.0)
  assert E.FrogPilotEventName.tailgating in long.events.added


SISSY_EVENT_NAMES = ["sissyLaneHugging", "sissyHardBraking", "sissyCornering",
                     "sissyTailgating", "sissySpeeding", "sissyDistracted", "sissyPraise"]


@pytest.mark.parametrize("event", ["speedLimitExceeded", "tailgating"] + SISSY_EVENT_NAMES)
def test_events_are_outside_the_random_event_range(event):
  # frogpilot_events tests RANDOM_EVENT_START <= event <= RANDOM_EVENT_END, so a new
  # non-random event inside 17..28 would be treated as a random event
  value = getattr(E.FrogPilotEventName, event)
  assert not (E.RANDOM_EVENT_START <= value <= E.RANDOM_EVENT_END)


@pytest.mark.parametrize("event", SISSY_EVENT_NAMES)
def test_sissy_taunts_can_never_affect_driving(event):
  """The one that matters. selfdrived/state.py reacts to USER_DISABLE, IMMEDIATE_DISABLE,
  SOFT_DISABLE, OVERRIDE_* and NO_ENTRY; PERMANENT appears nowhere in it. A joke must never be
  able to disengage the car or block engagement, so every taunt must be PERMANENT and nothing
  else."""
  from openpilot.selfdrive.selfdrived.events import ET, FROGPILOT_EVENTS
  entry = FROGPILOT_EVENTS[getattr(E.FrogPilotEventName, event)]
  assert set(entry.keys()) == {ET.PERMANENT}, f"{event} has non-cosmetic event types: {set(entry)}"


@pytest.mark.parametrize("trigger", ["lane_hugging", "hard_braking", "cornering",
                                     "tailgating", "speeding", "distracted"])
def test_every_taunt_trigger_has_ten_lines(trigger):
  from openpilot.selfdrive.selfdrived.events import SISSY_TAUNTS
  assert len(SISSY_TAUNTS[trigger]) == 10
  for line_1, line_2 in SISSY_TAUNTS[trigger]:
    assert line_1, "line 1 is what gets hashed and spoken; it cannot be empty"


def test_praise_pool_exists():
  from openpilot.selfdrive.selfdrived.events import SISSY_PRAISE
  assert len(SISSY_PRAISE) == 10
  for line_1, _ in SISSY_PRAISE:
    assert line_1


def test_speech_key_ignores_line_two():
  """Line 2 carries the live offence count. If it fed the hash, every repeat would resolve to a
  file that does not exist and the taunt would be silent."""
  from openpilot.selfdrive.ui.soundd import taunt_speech_key
  assert taunt_speech_key("SAME") == taunt_speech_key("SAME")
  assert taunt_speech_key("A") != taunt_speech_key("B")


def test_anti_repeat_cannot_deadlock():
  """History longer than the pool must fall back rather than run out of choices."""
  from openpilot.selfdrive.selfdrived import events as E2
  pool = [("a", ""), ("b", "")]
  for _ in range(50):
    assert 0 <= E2.sissy_pick("_deadlock_probe", pool) < len(pool)


def test_anti_repeat_spreads_picks():
  from openpilot.selfdrive.selfdrived import events as E2
  pool = [(str(i), "") for i in range(10)]
  picks = [E2.sissy_pick("_spread_probe", pool) for _ in range(200)]
  # with a history of 4 no index may repeat inside any window of 5
  for i in range(len(picks) - 4):
    assert len(set(picks[i:i + 5])) == 5, f"repeat inside window at {i}: {picks[i:i + 5]}"


def test_alerts_only_read_subscribed_services():
  """The original bug: reading sm["frogpilotPlan"] from inside the process that publishes
  it. This fails loudly on any service frogpilot_process does not subscribe to."""
  SUBSCRIBED = {"carControl", "carState", "controlsState", "deviceState", "driverMonitoringState",
                "gpsLocation", "gpsLocationExternal", "liveParameters", "livePose", "managerState",
                "modelV2", "onroadEvents", "pandaStates", "radarState", "selfdriveState",
                "frogpilotCarState", "frogpilotRadarState", "frogpilotSelfdriveState",
                "frogpilotModelV2", "frogpilotOnroadEvents", "mapdOut"}

  class StrictSM(dict):
    def __getitem__(self, key):
      assert key in SUBSCRIBED, f"frogpilot_process does not subscribe to {key!r}"
      return super().__getitem__(key)

  handler = make_handler()
  sm = StrictSM(make_sm(v_ego=30.0, lead_d_rel=10.0))
  for _ in range(int(10.0 / DT_MDL)):
    handler.update_speeding(sm, toggles())
    handler.update_tailgating(sm, toggles())
