#!/usr/bin/env python3
"""Good Girl Mode — speaks at the driver while the car is parked.

The drive is over, the engine is off, and they are still sitting there. This notices, and says
so, periodically, until the session runs out.

Everything that decides *whether to speak* lives here, and this module deliberately imports
nothing from openpilot beyond a single constant. No Params, no cereal, no clock of its own — the
time is injected. That is what makes the one thing worth testing testable: the bound on how often
it can possibly speak, and therefore on how much of the car's battery it can possibly burn.

Why the battery is the whole design
-----------------------------------
Offroad, the device integrates its real power draw against a 30 Wh virtual budget
(system/hardware/power_monitoring.py) and shuts itself down when that runs out. This feature is a
passive consumer of that budget: it never touches DisablePowerDown, ForcePowerDown or DoShutdown,
and if the shutdown timer fires mid-session the device powers off and the session dies with it.
That is the correct outcome and is not compensated for anywhere.

On top of that the session bounds itself, because a flat 12V on a Model S is not a minor
inconvenience — the car cannot be opened or started normally afterwards. Hence: a hard cap on
session length, a floor under the interval, and aborts on both the device's own battery estimate
and the measured voltage, all well before the shutdown logic would have to act.

Aborts are sticky. Once a session stops for any reason it stays stopped; only a fresh park
starts another. A session that could resume after aborting on low voltage would abort and resume
and abort again, which is precisely the behaviour the voltage guard exists to prevent.
"""
import random

from collections import deque

from openpilot.system.hardware.power_monitoring import CAR_BATTERY_CAPACITY_uWh

# Floors and ceilings the settings UI cannot cross. The UI offers a narrower range than these
# allow; these exist so that a hand-edited param cannot turn the feature into a continuous
# monologue that flattens the battery.
MIN_INTERVAL_S = 60.0
MAX_SESSION_S = 3600.0

# soundd is started by the manager reacting to a param, which it polls at about 1Hz, and then has
# to open the audio device. Speaking before it is up means the line is simply lost.
WARMUP_S = 20.0

# Stop while the device still has most of its budget left, rather than riding it down to the
# shutdown threshold. The point is to leave the car startable, not to extract every last taunt.
MIN_BATTERY_FRACTION = 0.60

# Stop this far *above* the user's own low-voltage shutdown setting, never at or below it. The
# device's shutdown must remain the thing that acts on low voltage; this just gets out of the way
# first so it is not the reason that threshold was reached.
VOLTAGE_MARGIN_V = 0.4

# How many recent lines to avoid repeating. Falls back to the full pool when the pool is smaller
# than the history, so a short pool cannot deadlock.
HISTORY = 4


class GoodGirlSession:
  def __init__(self):
    self.active = False
    self.start_t = 0.0
    self.last_spoken_t = 0.0
    self.stop_reason = ""
    self.recent = deque(maxlen=HISTORY)

  def start(self, now):
    """Begin a session. Called on a real park, never on boot into offroad."""
    self.active = True
    self.start_t = now
    # Seeded at the start so the first line waits out the warm-up rather than the full interval.
    self.last_spoken_t = now
    self.stop_reason = ""

  def stop(self, reason):
    self.active = False
    self.stop_reason = reason

  def update(self, now, started, pool, toggles, car_battery_uwh, voltage_v):
    """Return the line_1 to speak, or None.

    Order matters: the aborts come before the timing gates, so a session that should end does so
    on the tick it becomes true rather than at the next interval boundary.
    """
    # Ignition first and unconditionally. Nothing about this feature may outlive the car being
    # switched on, and it must not be possible for a later gate to return a line on this tick.
    if started:
      self.stop("ignition")
      return None

    if not getattr(toggles, "good_girl_mode", False):
      self.stop("disabled")
      return None

    if not self.active:
      return None

    session_cap = min(float(getattr(toggles, "good_girl_session", MAX_SESSION_S)), MAX_SESSION_S)
    if now - self.start_t >= session_cap:
      self.stop("session cap")
      return None

    # The device's own estimate of what it has left to spend. Absent means we cannot tell, and
    # the safe reading of "cannot tell" is to stop rather than to assume there is room.
    if car_battery_uwh is None or car_battery_uwh < MIN_BATTERY_FRACTION * CAR_BATTERY_CAPACITY_uWh:
      self.stop("battery budget")
      return None

    if voltage_v is not None:
      floor = float(getattr(toggles, "low_voltage_shutdown", 11.8)) + VOLTAGE_MARGIN_V
      if voltage_v < floor:
        self.stop("low voltage")
        return None
    else:
      # Unlike the budget above, a missing voltage reading is transient — pandad may simply not
      # have published yet — so this stays quiet without ending the session.
      return None

    if now - self.start_t < WARMUP_S:
      return None

    interval = max(float(getattr(toggles, "good_girl_interval", MIN_INTERVAL_S)), MIN_INTERVAL_S)
    if now - self.last_spoken_t < interval:
      return None

    line = self._pick(pool)
    if line is None:
      return None

    self.last_spoken_t = now
    return line

  def _pick(self, pool):
    """A line_1 from the pool, avoiding the recently used ones."""
    if not pool:
      return None

    choices = [i for i in range(len(pool)) if i not in self.recent]
    if not choices:                      # pool smaller than the history — allow anything
      choices = list(range(len(pool)))

    index = random.choice(choices)
    self.recent.append(index)
    return pool[index][0]
