#!/usr/bin/env python3
"""Good Girl Mode speaks while the car is parked, which means it spends the car's battery.

Everything here is really one question in different shapes: can this thing talk more than it is
supposed to? The bound test at the bottom is the one that matters — it is the only thing standing
between a settings mistake and a car that will not start in the morning.
"""
import ast
import hashlib
import math
import unittest

from pathlib import Path
from types import SimpleNamespace

from openpilot.frogpilot.system.good_girl_mode import (CAR_BATTERY_CAPACITY_uWh, MAX_SESSION_S,
                                                       MIN_BATTERY_FRACTION, MIN_INTERVAL_S,
                                                       VOLTAGE_MARGIN_V, WARMUP_S, GoodGirlSession)

POOL = [(f"LINE {i}", f"sub {i}") for i in range(8)]
GOOD_BATTERY = CAR_BATTERY_CAPACITY_uWh
GOOD_VOLTAGE = 12.6


def toggles(**kwargs):
  base = {"good_girl_mode": True, "good_girl_interval": 300.0,
          "good_girl_session": 1800.0, "low_voltage_shutdown": 11.8}
  base.update(kwargs)
  return SimpleNamespace(**base)


def run(session, duration, tog, dt=0.1, started=False, battery=GOOD_BATTERY,
        voltage=GOOD_VOLTAGE, t0=0.0):
  """Tick a session for `duration` seconds, returning (lines, times)."""
  lines, times = [], []
  steps = int(duration / dt)
  for i in range(steps):
    now = t0 + i * dt
    line = session.update(now, started, POOL, tog, battery, voltage)
    if line is not None:
      lines.append(line)
      times.append(now)
  return lines, times


class TestGoodGirlMode(unittest.TestCase):
  def setUp(self):
    self.session = GoodGirlSession()

  # --- the bound that caps the energy cost -------------------------------------------------

  def test_cannot_speak_more_than_the_interval_allows(self):
    """A full-length session at the fastest possible settings has a hard line count.

    This is the property that bounds the battery cost. If it ever fails, the feature can talk
    for longer or more often than anyone signed up for. Do not delete it.
    """
    self.session.start(0.0)
    # Ask for far more than the floors permit, in both directions.
    tog = toggles(good_girl_interval=0.1, good_girl_session=99999.0)
    lines, _ = run(self.session, MAX_SESSION_S + 600.0, tog)

    self.assertLessEqual(len(lines), math.ceil(MAX_SESSION_S / MIN_INTERVAL_S))
    self.assertFalse(self.session.active)
    self.assertEqual(self.session.stop_reason, "session cap")

  def test_interval_floor_holds_against_a_faster_setting(self):
    self.session.start(0.0)
    _, times = run(self.session, 600.0, toggles(good_girl_interval=5.0))
    gaps = [b - a for a, b in zip(times, times[1:])]
    self.assertTrue(all(g >= MIN_INTERVAL_S - 1e-6 for g in gaps), gaps)

  def test_session_cap_ends_it(self):
    self.session.start(0.0)
    tog = toggles(good_girl_interval=60.0, good_girl_session=300.0)
    _, times = run(self.session, 900.0, tog)
    self.assertTrue(all(t < 300.0 for t in times), times)
    self.assertFalse(self.session.active)

  # --- aborts ------------------------------------------------------------------------------

  def test_ignition_stops_it_on_the_same_tick(self):
    self.session.start(0.0)
    run(self.session, 600.0, toggles(good_girl_interval=60.0))
    self.assertTrue(self.session.active)

    self.assertIsNone(self.session.update(601.0, True, POOL, toggles(), GOOD_BATTERY, GOOD_VOLTAGE))
    self.assertFalse(self.session.active)
    self.assertEqual(self.session.stop_reason, "ignition")

  def test_ignition_abort_is_sticky(self):
    """Switching the car on and off again must not resume the old session."""
    self.session.start(0.0)
    self.session.update(100.0, True, POOL, toggles(), GOOD_BATTERY, GOOD_VOLTAGE)
    lines, _ = run(self.session, 3600.0, toggles(good_girl_interval=60.0), t0=200.0)
    self.assertEqual(lines, [])

  def test_toggle_off_never_speaks(self):
    self.session.start(0.0)
    lines, _ = run(self.session, MAX_SESSION_S, toggles(good_girl_mode=False))
    self.assertEqual(lines, [])
    self.assertFalse(self.session.active)

  def test_low_battery_budget_stops_and_stays_stopped(self):
    self.session.start(0.0)
    low = MIN_BATTERY_FRACTION * CAR_BATTERY_CAPACITY_uWh - 1
    lines, _ = run(self.session, 600.0, toggles(good_girl_interval=60.0), battery=low)
    self.assertEqual(lines, [])
    self.assertEqual(self.session.stop_reason, "battery budget")

    # and it does not come back when the estimate recovers
    lines, _ = run(self.session, 3600.0, toggles(good_girl_interval=60.0), t0=700.0)
    self.assertEqual(lines, [])

  def test_missing_battery_estimate_is_treated_as_empty(self):
    self.session.start(0.0)
    lines, _ = run(self.session, 600.0, toggles(good_girl_interval=60.0), battery=None)
    self.assertEqual(lines, [])
    self.assertFalse(self.session.active)

  def test_low_voltage_stops_above_the_shutdown_threshold(self):
    self.session.start(0.0)
    tog = toggles(good_girl_interval=60.0, low_voltage_shutdown=11.8)
    # Above the device's own shutdown point, but inside our margin: we must yield first.
    lines, _ = run(self.session, 600.0, tog, voltage=11.8 + VOLTAGE_MARGIN_V - 0.05)
    self.assertEqual(lines, [])
    self.assertEqual(self.session.stop_reason, "low voltage")

  def test_missing_voltage_is_quiet_but_not_fatal(self):
    """Unlike the budget, a missing reading is transient — stay silent, keep the session."""
    self.session.start(0.0)
    lines, _ = run(self.session, 300.0, toggles(good_girl_interval=60.0), voltage=None)
    self.assertEqual(lines, [])
    self.assertTrue(self.session.active)

    lines, _ = run(self.session, 300.0, toggles(good_girl_interval=60.0), t0=300.0)
    self.assertGreater(len(lines), 0)

  # --- timing and content ------------------------------------------------------------------

  def test_silent_through_warmup(self):
    """soundd is not up yet; a line spoken here is simply lost."""
    self.session.start(0.0)
    lines, _ = run(self.session, WARMUP_S - 0.2, toggles(good_girl_interval=MIN_INTERVAL_S))
    self.assertEqual(lines, [])

  def test_first_line_waits_the_interval_not_just_the_warmup(self):
    self.session.start(0.0)
    _, times = run(self.session, 400.0, toggles(good_girl_interval=MIN_INTERVAL_S))
    self.assertGreaterEqual(times[0], MIN_INTERVAL_S)

  def test_lines_come_from_the_pool_without_immediate_repeats(self):
    self.session.start(0.0)
    lines, _ = run(self.session, MAX_SESSION_S, toggles(good_girl_interval=MIN_INTERVAL_S))
    self.assertGreater(len(lines), 10)
    self.assertTrue(all(l in [p[0] for p in POOL] for l in lines))
    for a, b in zip(lines, lines[1:]):
      self.assertNotEqual(a, b)

  def test_single_line_pool_terminates(self):
    """Anti-repeat must not deadlock when the pool is smaller than the history."""
    self.session.start(0.0)
    tog = toggles(good_girl_interval=MIN_INTERVAL_S)
    lines = []
    for i in range(int(600 / 0.1)):
      line = self.session.update(i * 0.1, False, [("ONLY", "one")], tog, GOOD_BATTERY, GOOD_VOLTAGE)
      if line is not None:
        lines.append(line)
    self.assertGreater(len(lines), 0)
    self.assertTrue(all(l == "ONLY" for l in lines))

  def test_empty_pool_is_silent_not_a_crash(self):
    self.session.start(0.0)
    tog = toggles(good_girl_interval=MIN_INTERVAL_S)
    for i in range(int(300 / 0.1)):
      self.assertIsNone(self.session.update(i * 0.1, False, [], tog, GOOD_BATTERY, GOOD_VOLTAGE))

  def test_never_speaks_without_a_start(self):
    """A session only begins on a real park, never on boot into offroad."""
    lines, _ = run(self.session, MAX_SESSION_S, toggles(good_girl_interval=MIN_INTERVAL_S))
    self.assertEqual(lines, [])


if __name__ == "__main__":
  unittest.main()


class TestGoodGirlSafetyInvariants(unittest.TestCase):
  """Source-level guards. Cheap, and they catch the failures that would be expensive.

  These assert on the real files rather than on behaviour, because the properties they protect
  (never blocking shutdown, never weakening CPU power save) have no runtime signal that would
  fail a normal test — they would simply flatten the car's battery some night.
  """
  ROOT = Path(__file__).resolve().parents[3]

  @staticmethod
  def _code_only(path):
    """Source with comments and docstrings stripped.

    Checking raw text would flag the module docstring that explains it never touches these,
    which is the opposite of the property being protected.
    """
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
      if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) \
         and node.body and isinstance(node.body[0], ast.Expr) \
         and isinstance(node.body[0].value, ast.Constant) and isinstance(node.body[0].value.value, str):
        node.body.pop(0)
    return ast.dump(tree)

  def test_never_touches_the_shutdown_path(self):
    """The device's own power management must remain solely in charge of shutting down."""
    forbidden = ("DisablePowerDown", "ForcePowerDown", "DoShutdown", "DeviceShutdown")
    for rel in ("frogpilot/system/good_girl_mode.py",
                "frogpilot/frogpilot_process.py",
                "selfdrive/ui/soundd.py"):
      code = self._code_only(self.ROOT / rel)
      for name in forbidden:
        self.assertNotIn(name, code, f"{rel} must not reference {name}")

  def test_cpu_power_save_is_not_weakened(self):
    """The amp override must not have touched what decides CPU power save.

    If this expression ever gains a Good Girl term, the big CPU cluster stays online during a
    parked session and the draw is several watts rather than the amp's ~100mW.
    """
    text = (self.ROOT / "system/hardware/hardwared.py").read_text()
    self.assertIn(
      'should_pwrsave = not onroad_conditions["ignition"] and '
      'msg.deviceState.screenBrightnessPercent < 1e-3', text)

  def test_soundd_offroad_gate_requires_a_live_session(self):
    """soundd may run offroad only while a session is actually live, never permanently."""
    src = (self.ROOT / "system/manager/process_config.py").read_text()
    tree = ast.parse(src)

    gate = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "soundd_gate")
    gate_src = ast.get_source_segment(src, gate)
    self.assertIn("GoodGirlActive", gate_src)
    self.assertIn("good_girl_mode", gate_src)

    # and soundd is actually wired to it
    for node in ast.walk(tree):
      if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "PythonProcess" \
         and node.args and getattr(node.args[0], "value", "") == "soundd":
        self.assertEqual(node.args[2].id, "soundd_gate")
        break
    else:
      self.fail("could not find the soundd process entry")

  def test_ignition_clears_the_params_soundd_and_the_manager_read(self):
    """Stopping the session is not enough — the params both other processes read must be cleared."""
    src = (self.ROOT / "frogpilot/frogpilot_process.py").read_text()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "frogpilot_thread")

    branch = next(h for n in ast.walk(fn) if isinstance(n, ast.If)
                  for h in [ast.get_source_segment(src, n) or ""]
                  if "started and not started_previously" in h.split("\n")[0])
    self.assertIn('good_girl.stop("ignition")', branch)
    self.assertIn('put_bool("GoodGirlActive", False)', branch)
    self.assertIn('remove("GoodGirlLine")', branch)


class TestSpeechAssets(unittest.TestCase):
  ROOT = Path(__file__).resolve().parents[3]

  def _pools(self):
    src = (self.ROOT / "selfdrive/selfdrived/events.py").read_text()
    out = {}
    for n in ast.parse(src).body:
      if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") in ("SISSY_TAUNTS", "GOOD_GIRL_LINES"):
        out[n.targets[0].id] = ast.literal_eval(n.value)
    return out

  def test_pool_shape(self):
    pool = self._pools()["GOOD_GIRL_LINES"]
    self.assertGreaterEqual(len(pool), 8)
    for entry in pool:
      self.assertEqual(len(entry), 2)
      self.assertTrue(all(isinstance(s, str) and s.strip() for s in entry), entry)

  def test_no_hash_collisions_across_pools(self):
    """A collision would silently make two different lines share one wav."""
    pools = self._pools()
    lines = [l1 for l1, _ in pools["GOOD_GIRL_LINES"]]
    for tiers in pools["SISSY_TAUNTS"].values():
      for tier_lines in tiers.values():
        lines += [l1 for l1, _ in tier_lines]
    keys = [hashlib.sha1(l.encode()).hexdigest()[:16] for l in lines]
    self.assertEqual(len(set(keys)), len(keys))

  def test_generator_and_soundd_agree_on_the_key(self):
    """These two hash functions must stay identical or every spoken line is silent."""
    gen = (self.ROOT / "frogpilot/tools/make_taunt_speech.py").read_text()
    snd = (self.ROOT / "selfdrive/ui/soundd.py").read_text()

    def body(src):
      fn = next(n for n in ast.parse(src).body
                if isinstance(n, ast.FunctionDef) and n.name == "taunt_speech_key")
      return ast.dump(ast.parse(ast.get_source_segment(src, fn.body[-1])))

    self.assertEqual(body(gen), body(snd))

  def test_every_line_has_rendered_audio(self):
    """Skipped rather than failed: the wavs are gitignored and deployed, not committed."""
    asset_dir = self.ROOT / "frogpilot/assets/taunt_speech"
    if not asset_dir.is_dir():
      self.skipTest("no rendered audio present")
    have = {p.stem for p in asset_dir.glob("*.wav")}
    for line_1, _ in self._pools()["GOOD_GIRL_LINES"]:
      key = hashlib.sha1(line_1.encode()).hexdigest()[:16]
      self.assertIn(key, have, f"no audio for {line_1!r} — re-run make_taunt_speech.py")
