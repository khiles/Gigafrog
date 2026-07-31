#!/usr/bin/env python3
import random

from cereal import log

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import TurnDirection
from openpilot.selfdrive.selfdrived.events import ET, EVENT_NAME, FROGPILOT_EVENT_NAME, EventName, FrogPilotEventName, Events

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, NON_DRIVING_GEARS

DEJA_VU_G_FORCE = 0.75
HAZARD_LOOKAHEAD = 8.0     # s of travel at the set speed
MIN_HAZARD_DISTANCE = 100  # m, so it still warns in time at low speed

# Speeding. Sustain and re-arm so it can't chatter either side of the threshold.
SPEEDING_SUSTAIN = 3.0     # s continuously over before it fires
SPEEDING_REARM = 2.0       # m/s back under the limit before it can fire again
SPEEDING_REPEAT = 60.0     # s minimum between repeats while still speeding

# Tailgating, measured against the driver's own chosen following time
TAILGATING_FRACTION = 0.65   # fires below this share of tFollow
TAILGATING_SUSTAIN = 4.0     # s continuously too close
TAILGATING_REPEAT = 30.0     # s minimum between repeats
TAILGATING_MIN_SPEED = 8.0   # m/s, no point nagging in slow traffic
RANDOM_EVENTS_CHANCE = 0.01 * DT_MDL
RANDOM_EVENTS_LENGTH = 5

RANDOM_EVENT_START = FrogPilotEventName.accel30
RANDOM_EVENT_END = FrogPilotEventName.youveGotMail

# Taunts. The wording lives in SISSY_TAUNTS in selfdrive/selfdrived/events.py, next to the other
# alert text — this module cannot own it, because events.py would have to import from here and
# this file already imports from events.py.
#
# Harsh by design, which makes the rate limiting matter more, not less — a constant stream
# of abuse on screen stops being funny and starts being a distraction.
SISSY_GLOBAL_COOLDOWN = 20.0     # s between taunts of any kind
SISSY_REPEAT = 45.0              # s before the same trigger can fire again
SISSY_MIN_SPEED = 5.0            # m/s

SISSY_LANE_OFFSET = 0.30         # m off centre
SISSY_LANE_SUSTAIN = 5.0         # s continuously off centre
SISSY_BRAKING_ACCEL = -2.8       # m/s^2
SISSY_CORNERING_ACCEL = 3.2      # m/s^2
# Only the early distraction window. Above this the real driver monitoring warning owns the
# screen, and a joke must never sit on top of it or delay it.
SISSY_DM_MIN_AWARENESS = 0.7
# Distraction has to be sustained, like every other trigger. isDistracted flickers on brief
# glances, sun glare and sunglasses, so acting on a single frame of it fires while the driver is
# looking straight at the road — which is exactly what happened on the car.
SISSY_DM_SUSTAIN = 2.5           # s continuously distracted before it says anything

SISSY_ACCEL = 2.2                # m/s^2, pulling away hard
SISSY_ROUGHNESS = 3.0            # m/s^2 RMS vertical — a pothole, not a coarse surface
# A lane change begun while that side is occupied. The stock blind spot warning already fires;
# this only adds commentary afterwards and must never race it, hence the alerts_empty gate above.
SISSY_STOP_LINE_OVERSHOOT = -1.0  # m past the line before it counts as having run it

SISSY_EVENTS = {
  "lane_hugging": FrogPilotEventName.sissyLaneHugging,
  "hard_braking": FrogPilotEventName.sissyHardBraking,
  "cornering": FrogPilotEventName.sissyCornering,
  "tailgating": FrogPilotEventName.sissyTailgating,
  "speeding": FrogPilotEventName.sissySpeeding,
  "distracted": FrogPilotEventName.sissyDistracted,
  "harsh_accel": FrogPilotEventName.sissyHarshAccel,
  "pothole": FrogPilotEventName.sissyPothole,
  "blind_spot": FrogPilotEventName.sissyBlindSpotChange,
  "stop_line": FrogPilotEventName.sissyStopLine,
}

# Explicit set, not an ordinal range. A range would silently capture any enumerant appended
# after the taunts, which is exactly the trap RANDOM_EVENT_START..END already sets.
SISSY_OFFENCE_EVENTS = frozenset(SISSY_EVENTS.values())

# Praise is deliberately not in SISSY_EVENTS: it is not an offence, must not be counted as one,
# and must not be picked by the offence selection below.
SISSY_PRAISE_EVENT = FrogPilotEventName.sissyPraise
SISSY_PRAISE_CLEAN_TIME = 300.0   # s of clean driving before it says anything nice
SISSY_PRAISE_REPEAT = 600.0       # s between compliments, so it stays rare enough to sting

# Cruelty dial, 1-5. Scales thresholds down and cooldowns down together.
#
# The cooldown floor is not negotiable: at maximum this should be relentless, not continuous. A
# taunt every couple of seconds stops being a joke and becomes a genuine distraction, and the
# suppression while a real alert is on screen has to keep working at every setting.
SISSY_COOLDOWN_FLOOR = 8.0        # s, absolute minimum between taunts at any cruelty

# Escalation. Tier index into SISSY_TIERS in events.py, chosen by the running offence total for
# the drive: below the first number it stays mild, past the second it is brutal for the rest of
# the drive unless you earn it back.
SISSY_TIER_THRESHOLDS = (3, 8)
# A compliment walks the total back, so a genuinely good stretch de-escalates rather than leaving
# you branded for one bad junction an hour ago.
SISSY_PRAISE_FORGIVENESS = 3


def sissy_tier_for(total):
  """Tier index for a drive's running offence total."""
  return sum(total >= t for t in SISSY_TIER_THRESHOLDS)


def sissy_scale(cruelty):
  """Return (threshold_scale, cooldown_scale) for a 1-5 cruelty setting."""
  cruelty = min(max(float(cruelty), 1.0), 5.0)
  frac = (cruelty - 1.0) / 4.0
  return 1.0 - 0.5 * frac, 1.0 - 0.75 * frac

class FrogPilotEvents:
  def __init__(self, FrogPilotPlanner, error_log, ThemeManager):
    self.frogpilot_planner = FrogPilotPlanner
    self.theme_manager = ThemeManager

    self.events = Events(frogpilot=True)

    self.always_on_lateral_enabled_previously = False
    self.previous_traffic_mode = False
    self.random_event_playing = False
    self.startup_seen = False
    self.stopped_for_light = False

    self.max_acceleration = 0
    self.random_event_timer = 0
    self.tracked_lead_distance = 0

    self.played_events = set()

    self.announced_hazard = ""

    self.speeding_t = 0.0
    self.speeding_armed = True
    self.speeding_since_alert = 0.0

    self.tailgating_t = 0.0
    # Seeded at the repeat interval so the first alert fires as soon as it's sustained,
    # rather than waiting a full repeat period before it can ever speak
    self.tailgating_since_alert = TAILGATING_REPEAT

    # Seeded past the cooldowns so the first taunt lands as soon as it is earned
    self.sissy_lane_t = 0.0
    self.sissy_since_taunt = SISSY_GLOBAL_COOLDOWN
    self.sissy_since_trigger = {trigger: SISSY_REPEAT for trigger in SISSY_EVENTS}
    # Per-drive tallies. This object is rebuilt on every onroad transition, so they reset by
    # themselves; the count of whichever trigger just fired is published for the alert text.
    self.sissy_counts = {trigger: 0 for trigger in SISSY_EVENTS}
    self.sissy_offence_count = 0
    self.sissy_total = 0
    self.sissy_tier = 0
    self.sissy_clean_t = 0.0
    self.sissy_lane_change_prev = False
    self.sissy_distracted_t = 0.0
    # Seeded past the repeat like the taunt cooldowns above, so the first compliment waits
    # only on the clean-time rather than on a full repeat period as well
    self.sissy_since_praise = SISSY_PRAISE_REPEAT

    self.error_log = error_log

  def update_sissy_mode(self, sm, frogpilot_toggles, alerts_empty):
    """Taunt the driver. Purely cosmetic — every event here is ET.PERMANENT, which the selfdrive
    state machine does not react to, so none of it can affect how the car drives.

    Same rule as everything else in this file: read the planner objects, never sm["frogpilotPlan"],
    because this runs inside the process that publishes that message.
    """
    self.sissy_since_taunt += DT_MDL
    self.sissy_since_praise += DT_MDL
    for trigger in self.sissy_since_trigger:
      self.sissy_since_trigger[trigger] += DT_MDL

    # Edge-detected before any of the gates below. If this only tracked past them, a lane change
    # begun while a real alert was on screen would look like a fresh one the moment that alert
    # cleared, and fire for a manoeuvre already well underway.
    lane_change = sm["modelV2"].meta.laneChangeState != log.LaneChangeState.off
    lane_change_started = lane_change and not self.sissy_lane_change_prev
    self.sissy_lane_change_prev = lane_change

    if not frogpilot_toggles.sissy_mode:
      self.sissy_lane_t = 0.0
      self.sissy_clean_t = 0.0
      return

    car_state = sm["carState"]
    # Never talk over a real alert, and stay quiet when parked or crawling
    if not alerts_empty or car_state.standstill or car_state.vEgo < SISSY_MIN_SPEED:
      self.sissy_lane_t = 0.0
      return

    thresh_scale, cooldown_scale = sissy_scale(frogpilot_toggles.sissy_cruelty)
    global_cooldown = max(SISSY_GLOBAL_COOLDOWN * cooldown_scale, SISSY_COOLDOWN_FLOOR)
    repeat = max(SISSY_REPEAT * cooldown_scale, SISSY_COOLDOWN_FLOOR)

    planner = self.frogpilot_planner

    # Lane hugging has to be sustained — a single frame off centre is a bend, not a habit
    centering = planner.frogpilot_lane_centering
    if centering.lane_offset_valid and abs(centering.lane_offset_filtered) > SISSY_LANE_OFFSET * thresh_scale:
      self.sissy_lane_t += DT_MDL
    else:
      self.sissy_lane_t = 0.0

    candidates = []
    if self.sissy_lane_t >= SISSY_LANE_SUSTAIN * thresh_scale:
      candidates.append("lane_hugging")
    if car_state.aEgo <= SISSY_BRAKING_ACCEL * thresh_scale:
      candidates.append("hard_braking")
    if abs(planner.frogpilot_pose.cornering_acceleration) >= SISSY_CORNERING_ACCEL * thresh_scale:
      candidates.append("cornering")
    if self.tailgating_t >= TAILGATING_SUSTAIN * thresh_scale:
      candidates.append("tailgating")
    if self.speeding_t >= SPEEDING_SUSTAIN * thresh_scale:
      candidates.append("speeding")

    # Only the early distraction window. Once awareness has decayed past this the real driver
    # monitoring warning owns the screen, and a joke must never sit on top of it or delay it.
    dm = sm["driverMonitoringState"]
    if dm.isActiveMode and dm.isDistracted and dm.awarenessStatus > SISSY_DM_MIN_AWARENESS:
      self.sissy_distracted_t += DT_MDL
    else:
      self.sissy_distracted_t = 0.0
    if self.sissy_distracted_t >= SISSY_DM_SUSTAIN * thresh_scale:
      candidates.append("distracted")

    if car_state.aEgo >= SISSY_ACCEL * thresh_scale:
      candidates.append("harsh_accel")

    # A pothole is a spike in vertical acceleration, not a rough surface — road_roughness is RMS
    # about its own slow mean (frogpilot_pose.py), so a consistently coarse road does not register.
    if planner.frogpilot_pose.road_roughness >= SISSY_ROUGHNESS * thresh_scale:
      candidates.append("pothole")

    # Starting a lane change with that side occupied. Only on the rising edge of the manoeuvre:
    # once committed, a car alongside is normal and would otherwise fire every frame.
    direction = sm["modelV2"].meta.laneChangeDirection
    occupied = (car_state.leftBlindspot if direction == log.LaneChangeDirection.left
                else car_state.rightBlindspot if direction == log.LaneChangeDirection.right
                else False)
    if lane_change_started and occupied:
      candidates.append("blind_spot")

    # Tesla's own map stop lines. Gated on roadSignValid, so a car that does not send
    # UI_driverAssistRoadSign never fires this rather than firing on a confident zero.
    fp_car_state = sm["frogpilotCarState"]
    if fp_car_state.roadSignValid:
      for distance in (fp_car_state.stopSignDistance, fp_car_state.trafficLightDistance):
        # -1 means "not seen yet"; only a genuinely negative distance means it is behind you
        if SISSY_STOP_LINE_OVERSHOOT > distance > -50.0 and car_state.vEgo > SISSY_MIN_SPEED:
          candidates.append("stop_line")
          break

    # A clean stretch is one with nothing to complain about at all, not merely a quiet cooldown,
    # so any live offence resets it even when the taunt itself is rate limited.
    if candidates:
      self.sissy_clean_t = 0.0
    else:
      self.sissy_clean_t += DT_MDL

    if self.sissy_since_taunt < global_cooldown:
      return

    ready = [t for t in candidates if self.sissy_since_trigger[t] >= repeat]

    if not ready:
      # Nothing to criticise — reward a genuinely long clean run, rarely
      if (self.sissy_clean_t >= SISSY_PRAISE_CLEAN_TIME * thresh_scale
          and self.sissy_since_praise >= SISSY_PRAISE_REPEAT):
        self.events.add(SISSY_PRAISE_EVENT)
        self.sissy_since_taunt = 0.0
        self.sissy_since_praise = 0.0
        self.sissy_clean_t = 0.0
        self.sissy_total = max(self.sissy_total - SISSY_PRAISE_FORGIVENESS, 0)
        self.sissy_tier = sissy_tier_for(self.sissy_total)
      return

    trigger = random.choice(ready)
    self.sissy_counts[trigger] += 1
    self.sissy_offence_count = self.sissy_counts[trigger]
    self.sissy_total += 1
    self.sissy_tier = sissy_tier_for(self.sissy_total)
    self.events.add(SISSY_EVENTS[trigger])
    self.sissy_since_taunt = 0.0
    self.sissy_since_trigger[trigger] = 0.0

  def update_speeding(self, sm, frogpilot_toggles):
    """Warn when over the posted limit. Distinct from EventName.speedTooHigh, which means
    'faster than the model's training data', not 'faster than the sign'.

    Reads the planner directly rather than frogpilotPlan: this runs inside the process that
    *publishes* that message, so it is not in the SubMaster.
    """
    vcruise = self.frogpilot_planner.frogpilot_vcruise
    limit = vcruise.slc_target

    # source is "None" whenever no limit is known, and the limit is unreliable while a
    # change is still being confirmed
    known = vcruise.slc.source != "None" and limit > 0 and vcruise.slc.speed_limit_changed_timer <= DT_MDL
    if not known:
      self.speeding_t = 0.0
      return

    threshold = limit + vcruise.slc_offset + frogpilot_toggles.speed_limit_exceeded_margin
    over = sm["carState"].vEgo > threshold

    self.speeding_since_alert += DT_MDL
    if over:
      self.speeding_t += DT_MDL
    else:
      self.speeding_t = 0.0
      # Re-arm only once clearly back under, so hovering at the limit can't retrigger
      if sm["carState"].vEgo < threshold - SPEEDING_REARM:
        self.speeding_armed = True

    # The timer above runs whenever a limit is known, so Sissy Mode can reuse it even with this
    # alert switched off; only the alert itself is gated on the toggle.
    if not frogpilot_toggles.speed_limit_exceeded_alert:
      return

    if self.speeding_t >= SPEEDING_SUSTAIN and (self.speeding_armed or self.speeding_since_alert >= SPEEDING_REPEAT):
      self.events.add(FrogPilotEventName.speedLimitExceeded)
      self.speeding_armed = False
      self.speeding_since_alert = 0.0

  def update_tailgating(self, sm, frogpilot_toggles):
    """Warn when the gap is well under the driver's own chosen following time.

    Most useful when openpilot is not controlling longitudinal, which is exactly when
    nothing else is watching the gap.
    """
    lead = sm["radarState"].leadOne
    v_ego = sm["carState"].vEgo
    # From the planner, not frogpilotPlan — see update_speeding
    t_follow = self.frogpilot_planner.frogpilot_following.t_follow

    eligible = self.frogpilot_planner.tracking_lead
    eligible &= lead.status and v_ego > TAILGATING_MIN_SPEED and t_follow > 0
    # Traffic mode deliberately runs short gaps
    eligible &= not sm["frogpilotCarState"].trafficModeEnabled

    if not eligible:
      self.tailgating_t = 0.0
      return

    self.tailgating_since_alert += DT_MDL
    if lead.dRel / v_ego < t_follow * TAILGATING_FRACTION:
      self.tailgating_t += DT_MDL
    else:
      self.tailgating_t = 0.0

    # As above: the timer is toggle-independent so Sissy Mode can reuse it, the alert is not.
    if not frogpilot_toggles.tailgating_alert:
      return

    if self.tailgating_t >= TAILGATING_SUSTAIN and self.tailgating_since_alert >= TAILGATING_REPEAT:
      self.events.add(FrogPilotEventName.tailgating)
      self.tailgating_t = 0.0
      self.tailgating_since_alert = 0.0

  def update(self, long_control_active, v_cruise, sm, frogpilot_toggles):
    current_alert = sm["selfdriveState"].alertType
    current_frogpilot_alert = sm["frogpilotSelfdriveState"].alertType

    alerts_empty = all(sm[state].alertText1 == "" and sm[state].alertText2 == "" for state in ["selfdriveState", "frogpilotSelfdriveState"])

    self.events.clear()

    acceleration = sm["carControl"].actuators.accel

    if long_control_active:
      self.max_acceleration = max(acceleration, self.max_acceleration)
    else:
      self.max_acceleration = 0

    if self.frogpilot_planner.frogpilot_vcruise.forcing_stop:
      self.events.add(FrogPilotEventName.forcingStop)

    if not self.frogpilot_planner.tracking_lead and sm["carState"].standstill and sm["carState"].gearShifter not in NON_DRIVING_GEARS:
      if not self.frogpilot_planner.model_stopped and self.stopped_for_light and frogpilot_toggles.green_light_alert:
        self.events.add(FrogPilotEventName.greenLight)

      self.stopped_for_light = self.frogpilot_planner.frogpilot_cem.stop_light_detected
    else:
      self.stopped_for_light = False

    if "holidayActive" not in self.played_events and self.startup_seen and alerts_empty and len(self.events) == 0 and frogpilot_toggles.current_holiday_theme != "stock":
      self.events.add(FrogPilotEventName.holidayActive)

    # Hazards come straight from OSM via mapd. Announce each one once as it comes into
    # range, keyed on the text so a different hazard on the same stretch still alerts.
    if frogpilot_toggles.map_hazard_alert:
      hazard = sm["mapdOut"].nextHazard
      distance = sm["mapdOut"].nextHazardDistance
      if hazard and 0 < distance < max(v_cruise * HAZARD_LOOKAHEAD, MIN_HAZARD_DISTANCE):
        if hazard != self.announced_hazard:
          self.events.add(FrogPilotEventName.mapHazard)
          self.announced_hazard = hazard
      elif not hazard:
        self.announced_hazard = ""

    self.update_speeding(sm, frogpilot_toggles)
    self.update_tailgating(sm, frogpilot_toggles)
    # After the two above, because it reuses their sustain timers
    self.update_sissy_mode(sm, frogpilot_toggles, alerts_empty)

    if self.frogpilot_planner.tracking_lead and sm["carState"].standstill and sm["carState"].gearShifter not in NON_DRIVING_GEARS:
      if self.tracked_lead_distance == 0:
        self.tracked_lead_distance = self.frogpilot_planner.lead_one.dRel

      lead_departing = self.frogpilot_planner.lead_one.dRel - self.tracked_lead_distance >= 1
      lead_departing &= self.frogpilot_planner.lead_one.vLead >= 1

      if lead_departing and frogpilot_toggles.lead_departing_alert:
        self.events.add(FrogPilotEventName.leadDeparting)
    else:
      self.tracked_lead_distance = 0

    if "nnffLoaded" not in self.played_events and self.startup_seen and alerts_empty and len(self.events) == 0 and self.frogpilot_planner.params.get("NNFFModelName") is not None and frogpilot_toggles.nnff:
      self.events.add(FrogPilotEventName.nnffLoaded)

    if self.random_event_playing:
      self.random_event_timer += DT_MDL

      if self.random_event_timer >= RANDOM_EVENTS_LENGTH:
        self.theme_manager.update_wheel_image(frogpilot_toggles.wheel_image)
        self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)

        self.random_event_playing = False
        self.random_event_timer = 0

    if not self.random_event_playing and frogpilot_toggles.random_events:
      if "accel30" not in self.played_events and 3.5 > self.max_acceleration >= 3.0 and acceleration < 1.5:
        self.events.add(FrogPilotEventName.accel30)

        self.theme_manager.update_wheel_image("accel30", random_event=True)
        self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)

        self.max_acceleration = 0

      elif "accel35" not in self.played_events and 4.0 > self.max_acceleration >= 3.5 and acceleration < 1.5:
        self.events.add(FrogPilotEventName.accel35)

        self.theme_manager.update_wheel_image("accel35", random_event=True)
        self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)

        self.max_acceleration = 0

      elif "accel40" not in self.played_events and self.max_acceleration >= 4.0 and acceleration < 1.5:
        self.events.add(FrogPilotEventName.accel40)

        self.theme_manager.update_wheel_image("accel40", random_event=True)
        self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)

        self.max_acceleration = 0

      if "dejaVuCurve" not in self.played_events and sm["carState"].vEgo > CRUISING_SPEED:
        if self.frogpilot_planner.lateral_acceleration >= DEJA_VU_G_FORCE * ACCELERATION_DUE_TO_GRAVITY:
          self.events.add(FrogPilotEventName.dejaVuCurve)

      if "hal9000" not in self.played_events and (ET.NO_ENTRY in current_alert or ET.NO_ENTRY in current_frogpilot_alert):
        self.events.add(FrogPilotEventName.hal9000)

      if f"{EVENT_NAME[EventName.steerSaturated]}/" in current_alert or f"{FROGPILOT_EVENT_NAME[FrogPilotEventName.goatSteerSaturated]}/" in current_frogpilot_alert:
        event_choices = []
        if "firefoxSteerSaturated" not in self.played_events:
          event_choices.append("firefoxSteerSaturated")
        if "goatSteerSaturated" not in self.played_events:
          event_choices.append("goatSteerSaturated")
        if "thisIsFineSteerSaturated" not in self.played_events:
          event_choices.append("thisIsFineSteerSaturated")

        if event_choices and random.random() < RANDOM_EVENTS_CHANCE:
          event_choice = random.choice(event_choices)

          if event_choice == "firefoxSteerSaturated":
            self.events.add(FrogPilotEventName.firefoxSteerSaturated)

            self.theme_manager.update_wheel_image("firefoxSteerSaturated", random_event=True)
            self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)
          elif event_choice == "goatSteerSaturated":
            self.events.add(FrogPilotEventName.goatSteerSaturated)

            self.theme_manager.update_wheel_image("goatSteerSaturated", random_event=True)
            self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)
          elif event_choice == "thisIsFineSteerSaturated":
            self.events.add(FrogPilotEventName.thisIsFineSteerSaturated)

            self.theme_manager.update_wheel_image("thisIsFineSteerSaturated", random_event=True)
            self.frogpilot_planner.params_memory.put_bool("UpdateWheelImage", True)

      if "vCruise69" not in self.played_events and 70 > max(sm["carState"].vCruise, sm["carState"].vCruiseCluster) * (1 if frogpilot_toggles.is_metric else CV.KPH_TO_MPH) >= 69:
        self.events.add(FrogPilotEventName.vCruise69)

      if f"{EVENT_NAME[EventName.fcw]}/" in current_alert or f"{EVENT_NAME[EventName.stockAeb]}/" in current_alert:
        event_choices = []
        if "toBeContinued" not in self.played_events:
          event_choices.append("toBeContinued")
        if "yourFrogTriedToKillMe" not in self.played_events:
          event_choices.append("yourFrogTriedToKillMe")

        if event_choices:
          event_choice = random.choice(event_choices)
          if event_choice == "toBeContinued":
            self.events.add(FrogPilotEventName.toBeContinued)
          elif event_choice == "yourFrogTriedToKillMe":
            self.events.add(FrogPilotEventName.yourFrogTriedToKillMe)

      if "youveGotMail" not in self.played_events and sm["frogpilotCarState"].alwaysOnLateralEnabled and not self.always_on_lateral_enabled_previously:
        if random.random() < RANDOM_EVENTS_CHANCE / DT_MDL:
          self.events.add(FrogPilotEventName.youveGotMail)

      self.always_on_lateral_enabled_previously = sm["frogpilotCarState"].alwaysOnLateralEnabled
      self.random_event_playing |= bool({event for event in self.events.names if RANDOM_EVENT_START <= event <= RANDOM_EVENT_END})

    if self.error_log.is_file():
      if frogpilot_toggles.random_events:
        self.events.add(FrogPilotEventName.openpilotCrashedRandomEvent)
      else:
        self.events.add(FrogPilotEventName.openpilotCrashed)

    if self.frogpilot_planner.frogpilot_vcruise.slc.speed_limit_changed_timer == DT_MDL and frogpilot_toggles.speed_limit_changed_alert:
      self.events.add(FrogPilotEventName.speedLimitChanged)

    self.startup_seen |= sm["frogpilotSelfdriveState"].alertText1 == frogpilot_toggles.startup_alert_top and sm["frogpilotSelfdriveState"].alertText2 == frogpilot_toggles.startup_alert_bottom

    if sm["frogpilotCarState"].trafficModeEnabled != self.previous_traffic_mode:
      if self.previous_traffic_mode:
        self.events.add(FrogPilotEventName.trafficModeInactive)
      else:
        self.events.add(FrogPilotEventName.trafficModeActive)

      self.previous_traffic_mode = sm["frogpilotCarState"].trafficModeEnabled

    if sm["frogpilotModelV2"].turnDirection == TurnDirection.turnLeft:
      self.events.add(FrogPilotEventName.turningLeft)
    elif sm["frogpilotModelV2"].turnDirection == TurnDirection.turnRight:
      self.events.add(FrogPilotEventName.turningRight)

    self.played_events.update(FROGPILOT_EVENT_NAME[event] for event in self.events.names)
