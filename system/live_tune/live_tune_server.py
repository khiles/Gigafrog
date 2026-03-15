#!/usr/bin/env python3
"""
Live Tune Dashboard — real-time parameter monitoring and tuning.

Auto-started by the manager on boot. Also runnable standalone:
  python3 -m openpilot.system.live_tune.live_tune_server

Open  http://<device-ip>:8765  in any browser on the same network.

Parameter write path
--------------------
1. Write to persistent Params() so the value survives reboots.
2. Mirror to Params(memory=True) for processes that read from shared memory.
3. Set FrogPilotTogglesUpdated=True in memory so frogpilot_process picks up
   all FrogPilot toggle changes immediately and republishes frogpilotPlan.
   Processes reading frogpilotToggles (controlsd, selfdrived, …) then see
   the new values within one planning cycle (~100 ms).
"""

import asyncio
import json
import logging
from typing import Any

from aiohttp import web, WSMsgType

from cereal import messaging
from openpilot.common.params import Params

PORT = 8765
CEREAL_SERVICES = ['liveTorqueParameters', 'liveParameters', 'carState', 'controlsState', 'deviceState']

# ── Parameter registry ────────────────────────────────────────────────────────
# type: 'bool'|'int'|'float'|'string'  category: tab grouping
# default: used by the reset-to-default (↺) button in the UI

def _p(label, cat, t, desc, dflt, **kw):
  return dict(label=label, category=cat, type=t, desc=desc, default=dflt, **kw)

B, I, F, S = 'bool', 'int', 'float', 'string'

PARAMS: dict[str, dict] = {
  # ── Core ──────────────────────────────────────────────────────────────────
  'ExperimentalMode':          _p('Experimental Mode',              'core', B, 'Enable openpilot experimental driving mode',             False),
  'AlphaLongitudinalEnabled':  _p('Alpha Longitudinal',             'core', B, 'Use openpilot for gas/brake on supported cars',          False),
  'LongitudinalPersonality':   _p('Longitudinal Personality',       'core', I, 'Default following-distance profile',                     1,   min=0, max=3, options={0:'Aggressive',1:'Standard',2:'Relaxed',3:'Traffic'}),

  # ── Lateral ───────────────────────────────────────────────────────────────
  'AlwaysOnLateral':           _p('Always On Lateral',              'lateral', B, 'Keep steering active even when cruise is disengaged',  False),
  'AlwaysOnLateralLKAS':       _p('AoL: Keep LKAS Active',          'lateral', B, 'Keep LKAS active when AlwaysOnLateral is enabled',     True),
  'AlwaysOnLateralHoldTime':   _p('AoL Hold Time (s)',              'lateral', F, 'Seconds to hold AoL after brake press',                1.5, min=0.5, max=5.0, step=0.5),
  'PauseAOLOnBrake':           _p('Pause AoL On Brake',             'lateral', B, 'Pause AlwaysOnLateral when brake is pressed',          False),
  'PauseLateralOnSignal':      _p('Pause Lateral On Signal',        'lateral', B, 'Pause lateral control while blinker is active',        False),
  'PauseLateralSpeed':         _p('Pause Lateral Below (mph)',      'lateral', F, 'Pause lateral below this speed (0=disabled)',          0.0, min=0.0, max=50.0, step=1.0),
  'LateralTune':               _p('Lateral Tune',                   'lateral', B, 'Enable FrogPilot lateral tuning improvements',         True),
  'QOLLateral':                _p('QoL Lateral',                    'lateral', B, 'Quality-of-life lateral control improvements',         True),
  'NudgelessLaneChange':       _p('Nudgeless Lane Change',          'lateral', B, 'Lane change on blinker only — no steering nudge',      True),
  'OneLaneChange':             _p('One Lane Change Per Signal',     'lateral', B, 'Only one lane change per blinker activation',          True),
  'LaneChangeTime':            _p('Lane Change Time (s)',           'lateral', F, 'Duration of the lane-change maneuver',                 1.0, min=0.0, max=5.0, step=0.5),
  'MinimumLaneChangeSpeed':    _p('Min Lane Change Speed (mph)',    'lateral', F, 'Minimum speed to allow automatic lane changes',        20.0, min=0.0, max=60.0, step=1.0),
  'HumanLaneChanges':          _p('Human-Like Lane Changes',        'lateral', B, 'More gradual, human-like lane change movement',        True),
  'TurnDesires':               _p('Turn Desires',                   'lateral', B, 'Use turn desires for better low-speed cornering',      False),
  'ForceTorqueController':     _p('Force Torque Controller',        'lateral', B, 'Force torque controller even on unsupported cars',     False),
  'CalibratedLateralAcceleration': _p('Calibrated Lat Accel',      'lateral', F, 'Max lateral acceleration used during calibration',     2.0, min=1.0, max=3.0, step=0.1),

  # ── Longitudinal ──────────────────────────────────────────────────────────
  'LongitudinalTune':          _p('Longitudinal Tune',              'longitudinal', B, 'Enable FrogPilot longitudinal tuning improvements', True),
  'QOLLongitudinal':           _p('QoL Longitudinal',               'longitudinal', B, 'Quality-of-life longitudinal control improvements',  True),
  'AggressiveFollow':          _p('Aggressive Follow (s)',           'longitudinal', F, 'Time gap for Aggressive personality',               1.25, min=1.0, max=5.0, step=0.05),
  'StandardFollow':            _p('Standard Follow (s)',            'longitudinal', F, 'Time gap for Standard personality',                  1.45, min=1.0, max=5.0, step=0.05),
  'RelaxedFollow':             _p('Relaxed Follow (s)',             'longitudinal', F, 'Time gap for Relaxed personality',                   1.75, min=1.0, max=5.0, step=0.05),
  'TrafficFollow':             _p('Traffic Follow (s)',             'longitudinal', F, 'Time gap for Traffic mode',                          0.5, min=0.5, max=5.0, step=0.05),
  'AccelerationProfile':       _p('Acceleration Profile',           'longitudinal', I, 'Throttle aggressiveness profile',                    2,   min=0, max=3, options={0:'Normal',1:'Eco',2:'Sport',3:'Sport+'}),
  'DecelerationProfile':       _p('Deceleration Profile',          'longitudinal', I, 'Braking aggressiveness profile',                     1,   min=0, max=2, options={0:'Normal',1:'Eco',2:'Sport'}),
  'CurveSpeedController':      _p('Curve Speed Controller',         'longitudinal', B, 'Automatically slow down for curves',                 False),
  'HumanAcceleration':         _p('Human-Like Acceleration',        'longitudinal', B, 'Smoother, more natural acceleration profiles',        False),
  'HumanFollowing':            _p('Human-Like Following',           'longitudinal', B, 'Smoother following distance adjustments',             False),
  'CustomCruise':              _p('Cruise Increment (mph)',         'longitudinal', F, 'Cruise speed button increment',                      1.0, min=0.5, max=5.0, step=0.5),
  'CustomCruiseLong':          _p('Cruise Long-Press Inc. (mph)',   'longitudinal', F, 'Cruise speed long-press increment',                  5.0, min=1.0, max=15.0, step=1.0),
  'IncreasedStoppedDistance':  _p('Extra Stopped Distance (m)',     'longitudinal', F, 'Extra stopping distance behind lead vehicle',         0.0, min=0.0, max=10.0, step=0.5),
  'MaxDesiredAcceleration':    _p('Max Desired Accel (m/s²)',       'longitudinal', F, 'Maximum desired longitudinal acceleration',           4.0, min=0.5, max=4.0, step=0.1),
  'ReverseCruise':             _p('Reverse Cruise Buttons',         'longitudinal', B, 'Reverse cruise button direction (+ decreases speed)', False),
  'ForceStops':                _p('Force Stops',                    'longitudinal', B, 'Force openpilot to stop at detected stop lines',      False),

  # ── Conditional Experimental ──────────────────────────────────────────────
  'ConditionalExperimental':   _p('Conditional Experimental',       'ce', B, 'Auto-switch to experimental at intersections, curves, etc.', False),
  'CEStopLights':              _p('CE: Stop Lights',                'ce', B, 'Switch to experimental at stop lights',                      True),
  'CECurves':                  _p('CE: Curves',                     'ce', B, 'Switch to experimental on sharp curves',                     False),
  'CECurvesLead':              _p('CE: Curves with Lead',           'ce', B, 'Switch to experimental on curves when following a lead',     False),
  'CELead':                    _p('CE: Slow Lead',                  'ce', B, 'Switch to experimental when following a slow lead',          False),
  'CESlowerLead':              _p('CE: Slowing Lead',               'ce', B, 'Switch to experimental when lead slows significantly',       False),
  'CEStoppedLead':             _p('CE: Stopped Lead',               'ce', B, 'Switch to experimental when lead is stopped',               False),
  'CESpeed':                   _p('CE Below Speed (mph)',           'ce', F, 'Switch to experimental below this speed (0=disabled)',       0.0, min=0.0, max=130.0, step=5.0),
  'CESpeedLead':               _p('CE Below Speed w/ Lead (mph)',   'ce', F, 'Switch to experimental below this speed with a lead (0=off)', 0.0, min=0.0, max=130.0, step=5.0),
  'CESignalLaneDetection':     _p('CE: Signal+Lane Detection',      'ce', B, 'Use blinker/lane detection to improve CE decisions',         True),
  'CEModelStopTime':           _p('CE Model Stop Time (s)',         'ce', F, 'Seconds model must predict stop before switching to exp.',   8.0, min=0.0, max=20.0, step=1.0),
  'ShowCEMStatus':             _p('Show CE Status on HUD',          'ce', B, 'Show Conditional Experimental Mode status on HUD',           True),

  # ── Speed Limit Controller ─────────────────────────────────────────────────
  'SpeedLimitController':      _p('Speed Limit Controller',         'slc', B, 'Adjust cruise speed to match posted speed limit',           False),
  'ShowSpeedLimits':           _p('Show Speed Limits',              'slc', B, 'Show speed limit on HUD',                                  True),
  'SpeedLimitChangedAlert':    _p('Speed Limit Changed Alert',      'slc', B, 'Alert when the speed limit changes',                        False),
  'SLCConfirmation':           _p('Require Confirmation',           'slc', B, 'Require confirmation before applying new speed limit',      False),
  'SLCFallback':               _p('SLC Fallback',                   'slc', I, 'Action when no speed limit is available',                   2,   min=0, max=2, options={0:'None',1:'Set Speed',2:'Experimental Mode'}),
  'SLCOverride':               _p('SLC Override Method',            'slc', I, 'How to temporarily override the speed limit controller',    1,   min=0, max=2, options={0:'None',1:'Cruise Buttons',2:'Gas Pedal'}),
  'SetSpeedOffset':            _p('Speed Offset (mph)',             'slc', F, 'Offset applied to the set speed',                          0.0, min=-30.0, max=30.0, step=1.0),
  'UseVienna':                 _p('Use Vienna Signs (EU)',           'slc', B, 'Use Vienna (EU) signs instead of MUTCD (US)',               False),
  'SLCMapboxFiller':           _p('Mapbox Speed Filler',            'slc', B, 'Use Mapbox to fill in missing speed limit data',             True),
  'ShowCSCStatus':             _p('Show CSC Status on HUD',         'slc', B, 'Show Curve Speed Controller status on HUD',                 True),

  # ── Safety & Alerts ───────────────────────────────────────────────────────
  'LoudBlindspotAlert':        _p('Loud Blind Spot Alert',          'safety', B, 'Louder alert when changing lanes into an occupied blind spot', False),
  'GreenLightAlert':           _p('Green Light Alert',              'safety', B, 'Alert when traffic light ahead turns green',             False),
  'LeadDepartingAlert':        _p('Lead Departing Alert',           'safety', B, 'Alert when lead vehicle pulls away after a stop',        False),
  'BlindSpotPath':             _p('Blind Spot Path Overlay',        'safety', B, 'Highlight blind spot zones on the road view',            False),

  # ── Audio ─────────────────────────────────────────────────────────────────
  'AlertVolumeControl':        _p('Custom Alert Volumes',           'audio', B, 'Enable per-alert volume controls (101=stock)',            False),
  'EngageVolume':              _p('Engage Volume',                  'audio', I, 'Volume for engage sound (101=stock)',                     101, min=0, max=200),
  'DisengageVolume':           _p('Disengage Volume',               'audio', I, 'Volume for disengage sound (101=stock)',                  101, min=0, max=200),
  'RefuseVolume':              _p('Refuse Volume',                  'audio', I, 'Volume for refused engagement sound (101=stock)',         101, min=0, max=200),
  'PromptVolume':              _p('Prompt Volume',                  'audio', I, 'Volume for prompt alerts (101=stock)',                    101, min=0, max=200),
  'PromptDistractedVolume':    _p('Distracted Prompt Volume',       'audio', I, 'Volume for distracted driver prompt (101=stock)',         101, min=0, max=200),
  'WarningSoftVolume':         _p('Soft Warning Volume',            'audio', I, 'Volume for soft warning alerts (101=stock)',              101, min=0, max=200),
  'WarningImmediateVolume':    _p('Immediate Warning Volume',       'audio', I, 'Volume for immediate warning alerts (101=stock)',         101, min=0, max=200),
  'GoatScream':                _p('Goat Scream',                    'audio', B, 'Replace disengage sound with a goat scream',              False),
  'RandomEvents':              _p('Random Events',                  'audio', B, 'Enable random fun events during driving',                 False),
  'CustomAlerts':              _p('Custom FrogPilot Alerts',        'audio', B, 'Enable custom FrogPilot alert sounds and messages',       False),

  # ── Advanced Tuning ───────────────────────────────────────────────────────
  'ForceAutoTune':             _p('Force Auto Tune',                'tuning', B, 'Always use live-learned torque parameters',              False),
  'ForceAutoTuneOff':          _p('Disable Auto Tune',              'tuning', B, 'Permanently disable live torque learning',               False),
  'NNFF':                      _p('Neural Net FF (NNFF)',            'tuning', B, 'ML-based steering feedforward for torque-based cars',    False),
  'NNFFLite':                  _p('NNFF Lite',                      'tuning', B, 'Lighter-weight NNFF for older hardware',                  False),
  'AdvancedLateralTune':       _p('Advanced Lateral Tune',          'tuning', B, 'Unlock manual lateral tuning parameters',                False),
  'AdvancedLongitudinalTune':  _p('Advanced Longitudinal Tune',     'tuning', B, 'Unlock manual longitudinal tuning parameters',           False),
  'SteerFriction':             _p('Steer Friction Override',        'tuning', F, 'Override learned friction coefficient (0=use learned)',  0.0, min=0.0, max=0.5, step=0.005),
  'SteerKP':                   _p('Steer KP Override',              'tuning', F, 'Override lateral proportional gain (0=use learned)',     0.0, min=0.0, max=2.0, step=0.05),
  'SteerLatAccel':             _p('Steer Lat Accel Override',       'tuning', F, 'Override lateral acceleration limit (0=use stock)',      0.0, min=0.0, max=5.0, step=0.05),
  'SteerRatio':                _p('Steer Ratio Override',           'tuning', F, 'Override steering ratio (0=use stock)',                  0.0, min=0.0, max=25.0, step=0.1),
  'LongitudinalActuatorDelay': _p('Actuator Delay Override (s)',    'tuning', F, 'Override longitudinal actuator delay (0=use stock)',      0.0, min=0.0, max=1.0, step=0.01),
  'LeadDetectionThreshold':    _p('Lead Detection Threshold',       'tuning', I, 'Radar confidence threshold for lead detection (lower=more sensitive)', 35, min=10, max=70),

  # ── UI Customisation ──────────────────────────────────────────────────────
  'DeveloperUI':               _p('Developer UI',                   'ui', B, 'Show CPU/GPU/memory/FPS/IP debug overlay',                  False),
  'CustomUI':                  _p('Custom UI',                      'ui', B, 'Enable FrogPilot custom HUD widgets',                       False),
  'ModelUI':                   _p('Model Visualisation',            'ui', B, 'Enhanced path, lane-line and lead visualisation',            False),
  'QOLVisuals':                _p('QoL Visuals',                    'ui', B, 'Quality-of-life visual improvements',                        True),
  'AdjacentPath':              _p('Adjacent Path Overlay',          'ui', B, 'Show adjacent lane path overlays on HUD',                   False),
  'AdjacentPathMetrics':       _p('Adjacent Path Metrics',          'ui', B, 'Show lane-width numbers on adjacent path overlays',         False),
  'AccelerationPath':          _p('Acceleration Path Colours',      'ui', B, 'Colour-code path by acceleration/deceleration',             True),
  'Compass':                   _p('Compass',                        'ui', B, 'Show compass heading on HUD',                               False),
  'PedalsOnUI':                _p('Pedals on HUD',                  'ui', B, 'Show brake/gas pedal indicators on HUD',                    False),
  'DynamicPedalsOnUI':         _p('Dynamic Pedals (active-only)',   'ui', B, 'Show pedal indicators only when active',                    True),
  'RotatingWheel':             _p('Rotating Steering Wheel',        'ui', B, 'Show a rotating steering wheel icon on HUD',                True),
  'WheelSpeed':                _p('Show Wheel Speed',               'ui', B, 'Show wheel speed instead of GPS speed',                     False),
  'NavigationUI':              _p('Navigation UI',                  'ui', B, 'Show turn-by-turn navigation on HUD',                       True),
  'RoadNameUI':                _p('Road Name on HUD',               'ui', B, 'Show current road name on HUD',                            True),
  'OnroadDistanceButton':      _p('Onroad Distance Button',         'ui', B, 'Show tappable follow-distance button on HUD',               False),
  'DriverCamera':              _p('Driver Camera Feed',             'ui', B, 'Show driver-facing camera on HUD',                         False),
  'StoppedTimer':              _p('Stopped Timer',                  'ui', B, 'Show elapsed time when stopped behind lead vehicle',        False),
  'HideAlerts':                _p('Hide Non-Critical Alerts',       'ui', B, 'Hide non-critical HUD alerts',                             False),
  'HideLeadMarker':            _p('Hide Lead Marker',               'ui', B, 'Hide the lead vehicle marker on HUD',                      False),
  'HideMaxSpeed':              _p('Hide Max Speed',                 'ui', B, 'Hide the max speed display on HUD',                        False),
  'HideSpeed':                 _p('Hide Current Speed',             'ui', B, 'Hide current speed from HUD',                              False),
  'HideSpeedLimit':            _p('Hide Speed Limit',               'ui', B, 'Hide speed limit display from HUD',                        False),
  'LaneLinesWidth':            _p('Lane Lines Width (ft)',          'ui', F, 'Visual width of lane lines',                               4.0, min=0.0, max=8.0, step=0.5),
  'PathEdgeWidth':             _p('Path Edge Width (%)',            'ui', F, 'Width of path edge highlight',                             20.0, min=0.0, max=50.0, step=1.0),
  'PathWidth':                 _p('Path Width (ft)',                'ui', F, 'Visual width of the predicted path overlay',               6.1, min=1.0, max=10.0, step=0.1),
  'RoadEdgesWidth':            _p('Road Edges Width (ft)',          'ui', F, 'Visual width of road edge lines',                          2.0, min=0.0, max=8.0, step=0.5),
  'ScreenBrightness':          _p('Screen Brightness (1-101)',      'ui', I, 'Onroad display brightness (101=auto)',                     101, min=1, max=101),
  'CameraView':                _p('Camera View',                    'ui', I, 'Default camera feed shown on HUD',                         3,   min=0, max=3, options={0:'Auto',1:'Wide',2:'Driver',3:'Rear'}),

  # ── Device / System ───────────────────────────────────────────────────────
  'HigherBitrate':             _p('Higher Bitrate Recording',       'device', B, 'Record dashcam at higher quality (uses more storage)',  False),
  'IncreaseThermalLimits':     _p('Increase Thermal Limits',        'device', B, 'Allow device to run hotter before throttling',          False),
  'NoLogging':                 _p('Disable Logging',                'device', B, 'Do not record any driving data',                        False),
  'NoUploads':                 _p('Disable Uploads',                'device', B, 'Do not upload driving data to servers',                 False),

  # ── Mapbox ────────────────────────────────────────────────────────────────
  'MapboxPublicKey':           _p('Mapbox Public Key',              'mapbox', S, 'pk.eyJ1\u2026 \u2014 required for navigation maps',      '', placeholder='pk.eyJ1...'),
  'MapboxSecretKey':           _p('Mapbox Secret Key',              'mapbox', S, 'sk.eyJ1\u2026 \u2014 required for turn-by-turn routing', '', placeholder='sk.eyJ1...', secret=True),
}

# ── Param read / write helpers ───────────────────────────────────────────────

def _read_all_params() -> dict[str, Any]:
  p = Params()
  out: dict[str, Any] = {}
  for key, meta in PARAMS.items():
    try:
      if meta['type'] == 'bool':
        out[key] = p.get_bool(key)
      else:
        out[key] = p.get(key)   # returns native Python type via CPP_2_PYTHON
    except Exception:
      out[key] = None
  return out


def _read_stats() -> dict[str, Any]:
  try:
    stats = Params().get('FrogPilotStats') or {}
    return {
      'drives':       int(stats.get('Drives', 0)),
      'engages':      int(stats.get('Engages', 0)),
      'disengages':   int(stats.get('Disengages', 0)),
      'overrides':    int(stats.get('Overrides', 0)),
      'monthKm':      round(float(stats.get('CurrentMonthsMeters', 0)) / 1000.0, 1),
      'dayTimeHrs':   round(float(stats.get('DayTime', 0)) / 3600.0, 1),
      'nightTimeHrs': round(float(stats.get('NightTime', 0)) / 3600.0, 1),
    }
  except Exception:
    return {}


def _write_param(key: str, value: Any) -> bool:
  if key not in PARAMS:
    return False
  meta = PARAMS[key]
  params  = Params()
  params_m = Params(memory=True)

  try:
    if meta['type'] == 'bool':
      b = bool(value)
      params.put_bool(key, b)
      try:
        params_m.put_bool(key, b)
      except Exception:
        pass

    elif meta['type'] == 'int':
      iv = int(value)
      params.put(key, iv)
      try:
        params_m.put(key, iv)
      except Exception:
        pass

    elif meta['type'] == 'float':
      fv = float(value)
      params.put(key, fv)
      try:
        params_m.put(key, fv)
      except Exception:
        pass

    elif meta['type'] == 'string':
      sv = str(value)
      params.put(key, sv)
      try:
        params_m.put(key, sv)
      except Exception:
        pass

    # Signal FrogPilot to reload all toggle values on the next planning cycle
    try:
      params_m.put_bool("FrogPilotTogglesUpdated", True)
    except Exception:
      pass

    return True
  except Exception as exc:
    logging.getLogger('live_tune').warning('_write_param %s failed: %s', key, exc)
    return False


# ── WebSocket live-data loop ──────────────────────────────────────────────────

async def _ws_send_loop(ws: web.WebSocketResponse) -> None:
  sm = messaging.SubMaster(CEREAL_SERVICES)
  while not ws.closed:
    try:
      sm.update(0)
      data: dict[str, Any] = {}

      if sm.updated['liveTorqueParameters']:
        t = sm['liveTorqueParameters']
        data['torque'] = {
          'latAccelFactor': round(float(t.latAccelFactorFiltered), 4),
          'latAccelOffset': round(float(t.latAccelOffsetFiltered), 4),
          'friction':       round(float(t.frictionCoefficientFiltered), 4),
          'calPerc':        int(t.calPerc),
          'useParams':      bool(t.useParams),
        }

      if sm.updated['liveParameters']:
        lp = sm['liveParameters']
        data['vehicleParams'] = {
          'steerRatio':      round(float(lp.steerRatio), 3),
          'stiffnessFactor': round(float(lp.stiffnessFactor), 3),
          'angleOffsetDeg':  round(float(lp.angleOffsetAverageDeg), 3),
          'gyroBias':        round(float(lp.gyroBias), 4),
          'roll':            round(float(lp.roll), 4),
        }

      if sm.updated['carState']:
        cs = sm['carState']
        data['carState'] = {
          'vEgo':             round(float(cs.vEgo) * 3.6, 1),
          'steeringAngleDeg': round(float(cs.steeringAngleDeg), 1),
          'leftBlinker':      bool(cs.leftBlinker),
          'rightBlinker':     bool(cs.rightBlinker),
          'leftBlindspot':    bool(cs.leftBlindspot),
          'rightBlindspot':   bool(cs.rightBlindspot),
        }

      if sm.updated['controlsState']:
        ctrl = sm['controlsState']
        data['controls'] = {
          'enabled':       bool(ctrl.enabled),
          'lateralActive': bool(ctrl.lateralActive),
        }

      if sm.updated['deviceState']:
        ds = sm['deviceState']
        cpu_t = list(ds.cpuTempC)
        gpu_t = list(ds.gpuTempC)
        cpu_u = list(ds.cpuUsagePercent)
        data['device'] = {
          'cpuTempC':     round(max(cpu_t), 1) if cpu_t else 0.0,
          'gpuTempC':     round(max(gpu_t), 1) if gpu_t else 0.0,
          'memTempC':     round(float(ds.memoryTempC), 1),
          'maxTempC':     round(float(ds.maxTempC), 1),
          'thermalStatus': str(ds.thermalStatus),
          'fanSpeedPct':  int(ds.fanSpeedPercentDesired),
          'memUsagePct':  int(ds.memoryUsagePercent),
          'freeSpacePct': round(float(ds.freeSpacePercent), 1),
          'gpuUsagePct':  int(ds.gpuUsagePercent),
          'cpuUsagePct':  round(sum(cpu_u) / len(cpu_u), 1) if cpu_u else 0.0,
          'networkType':  str(ds.networkType),
          'powerDrawW':   round(float(ds.powerDrawW), 2),
        }

      if data:
        await ws.send_str(json.dumps({'type': 'live', 'data': data}))
    except Exception as exc:
      logging.getLogger('live_tune').warning('WS send error: %s', exc)

    await asyncio.sleep(0.25)


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse()
  await ws.prepare(request)
  request.app['websockets'].add(ws)

  send_task = asyncio.create_task(_ws_send_loop(ws))
  try:
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          cmd = json.loads(msg.data)
          if cmd.get('action') == 'set_param':
            ok = _write_param(cmd['key'], cmd['value'])
            ack: dict[str, Any] = {'type': 'ack', 'key': cmd['key'], 'success': ok}
            if not ok:
              ack['error'] = f'write failed for key={cmd["key"]!r}'
            await ws.send_str(json.dumps(ack))
          elif cmd.get('action') == 'reset_param':
            key = cmd.get('key', '')
            meta = PARAMS.get(key)
            if meta and 'default' in meta and meta['default'] != '':
              dflt = meta['default']
              ok = _write_param(key, dflt)
              ack = {'type': 'ack', 'key': key, 'success': ok, 'reset': True, 'value': dflt}
            else:
              ack = {'type': 'ack', 'key': key, 'success': False, 'error': 'no default defined'}
            await ws.send_str(json.dumps(ack))
        except Exception as exc:
          await ws.send_str(json.dumps({'type': 'error', 'message': str(exc)}))
      elif msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
        break
  finally:
    send_task.cancel()
    request.app['websockets'].discard(ws)
  return ws


async def api_get_params(request: web.Request) -> web.Response:
  values = _read_all_params()
  result = {k: {'meta': v, 'value': values.get(k)} for k, v in PARAMS.items()}
  return web.json_response(result)


async def api_post_params(request: web.Request) -> web.Response:
  body = await request.json()
  key, value = body.get('key'), body.get('value')
  if not key or value is None:
    raise web.HTTPBadRequest(text='Missing key or value')
  if not _write_param(key, value):
    raise web.HTTPBadRequest(text=f'Unknown or invalid parameter: {key}')
  return web.json_response({'success': True})


async def api_stats(request: web.Request) -> web.Response:
  return web.json_response(_read_stats())


async def api_export(request: web.Request) -> web.Response:
  values = _read_all_params()
  payload = {k: v for k, v in values.items() if v is not None}
  return web.Response(
    body=json.dumps(payload, indent=2).encode(),
    content_type='application/json',
    headers={'Content-Disposition': 'attachment; filename="frogpilot-config.json"'},
  )


async def api_import(request: web.Request) -> web.Response:
  body = await request.json()
  written, failed = 0, []
  for k, v in body.items():
    if k in PARAMS:
      if _write_param(k, v):
        written += 1
      else:
        failed.append(k)
  return web.json_response({'written': written, 'failed': failed})


# ── Dashboard HTML ────────────────────────────────────────────────────────────

_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>openpilot Live Tune Dashboard</title>
<style>
:root{
  --bg:#0d1117;--bg2:#161b22;--bg3:#21262d;--brd:#30363d;
  --txt:#e6edf3;--txt2:#c9d1d9;--muted:#8b949e;--muted2:#484f58;
  --blue:#58a6ff;--grn:#3fb950;--grn2:#238636;--grn3:#2ea043;
  --red:#f85149;--ylw:#d29922;--fcs:#388bfd;--blu2:#1f6feb;--sw-off:#555d68;
}
body.light{
  --bg:#f6f8fa;--bg2:#ffffff;--bg3:#f1f3f5;--brd:#d0d7de;
  --txt:#24292f;--txt2:#24292f;--muted:#57606a;--muted2:#8c959f;
  --blue:#0969da;--grn:#1a7f37;--grn2:#1f883d;--grn3:#2da44e;
  --red:#cf222e;--ylw:#9a6700;--fcs:#0969da;--blu2:#0550ae;--sw-off:#8c959f;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:-apple-system,'Segoe UI',sans-serif;transition:background .2s,color .2s}
header{background:var(--bg2);border-bottom:1px solid var(--brd);padding:8px 14px;display:flex;align-items:center;gap:8px;position:sticky;top:0;z-index:10;flex-wrap:wrap}
header h1{font-size:15px;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:var(--grn);flex-shrink:0}
.dot.off{background:var(--red)}
.hdr-right{display:flex;align-items:center;gap:7px;margin-left:auto;flex-wrap:wrap}
.conn{font-size:11px;color:var(--muted)}
#searchInput{background:var(--bg3);color:var(--txt);border:1px solid var(--brd);border-radius:20px;padding:4px 12px;font-size:12px;width:160px;outline:none}
#searchInput:focus{border-color:var(--fcs)}
.hdr-btn{background:var(--bg3);color:var(--muted);border:1px solid var(--brd);border-radius:6px;padding:4px 9px;font-size:11px;cursor:pointer;font-weight:600}
.hdr-btn:hover{background:var(--brd);color:var(--txt)}
.tabs{display:flex;gap:3px;padding:7px 12px;background:var(--bg);border-bottom:1px solid var(--bg3);overflow-x:auto;position:sticky;top:39px;z-index:9;scrollbar-width:none}
.tabs::-webkit-scrollbar{display:none}
.tab{padding:4px 11px;border-radius:20px;font-size:12px;font-weight:600;cursor:pointer;white-space:nowrap;background:var(--bg3);color:var(--muted);border:none;transition:all .15s}
.tab.active{background:var(--grn2);color:#fff}
.page{display:none;padding:10px 12px;max-width:1200px}
.page.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:10px}
.card{background:var(--bg2);border:1px solid var(--brd);border-radius:10px;padding:12px}
.card h2{font-size:11px;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:.07em;margin-bottom:9px}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid var(--bg3)}
.row:last-child{border-bottom:none}
.lbl{color:var(--muted);font-size:12px}
.val{font-size:13px;font-weight:600;font-variant-numeric:tabular-nums;color:var(--blue);transition:color .2s}
.val.flash{color:var(--grn)}
.bar-wrap{height:5px;background:var(--bg3);border-radius:3px;margin-top:4px;overflow:hidden}
.bar{height:100%;background:var(--grn2);border-radius:3px;transition:width .4s}
.bar.warn{background:var(--ylw)}.bar.crit{background:var(--red)}
.trow{display:flex;justify-content:space-between;align-items:flex-start;padding:7px 0;border-bottom:1px solid var(--bg3)}
.trow:last-child{border-bottom:none}
.tleft{display:flex;flex-direction:column;gap:2px;flex:1;min-width:0;padding-right:8px}
.tlbl{color:var(--txt2);font-size:13px;display:flex;align-items:center;gap:5px}
.tdesc{color:var(--muted);font-size:11px}
.ack{font-size:10px;color:var(--grn);opacity:0;transition:opacity .3s}
.ack.show{opacity:1}
.ctrl-grp{display:flex;align-items:center;gap:5px;flex-shrink:0}
.sw{display:inline-block;position:relative;width:40px;height:22px;flex-shrink:0}
.sw input{opacity:0;width:0;height:0}
.track{position:absolute;cursor:pointer;top:0;right:0;bottom:0;left:0;background:var(--sw-off);border-radius:22px;transition:background .2s}
.track:before{content:'';position:absolute;width:16px;height:16px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:transform .2s;box-shadow:0 1px 3px rgba(0,0,0,.4)}
input:checked+.track{background:var(--grn2)}
input:checked+.track:before{transform:translateX(18px)}
.rst{background:none;border:none;color:var(--muted2);font-size:14px;cursor:pointer;padding:1px 3px;border-radius:3px;line-height:1;flex-shrink:0}
.rst:hover{color:var(--muted);background:var(--bg3)}
select{background:var(--bg3);color:var(--txt);border:1px solid var(--brd);border-radius:6px;padding:4px 7px;font-size:12px;cursor:pointer}
.num-wrap{display:flex;align-items:center;gap:4px}
.num-inp{background:var(--bg3);color:var(--txt);border:1px solid var(--brd);border-radius:6px;padding:4px 6px;font-size:12px;width:72px;text-align:right}
.num-inp:focus{outline:none;border-color:var(--fcs)}
.apply-btn{background:var(--grn2);color:#fff;border:none;border-radius:6px;padding:4px 8px;font-size:11px;cursor:pointer;font-weight:600}
.apply-btn:hover{background:var(--grn3)}
.str-inp{background:var(--bg3);color:var(--txt);border:1px solid var(--brd);border-radius:6px;padding:5px 9px;font-size:12px;width:100%;margin-top:5px}
.str-inp:focus{outline:none;border-color:var(--fcs)}
.save-btn{margin-top:6px;background:var(--blu2);color:#fff;border:none;border-radius:6px;padding:5px 12px;font-size:12px;cursor:pointer;font-weight:600}
.save-btn:hover{background:var(--fcs)}
.bs-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:7px}
.bs{text-align:center;padding:7px 4px;border-radius:8px;font-size:12px;font-weight:700;background:var(--bg3);color:var(--muted);border:1px solid transparent;transition:all .15s}
.bs.occupied{background:#ff000022;color:#ff6b6b;border-color:#ff6b6b}
.bs.blinker{background:#ffaa0022;color:#ffaa00;border-color:#ffaa00}
.bs.danger{background:#ff000055;color:#ff4444;border:2px solid #ff4444;animation:bsp .5s infinite alternate}
@keyframes bsp{from{opacity:.7}to{opacity:1}}
.t-ok{color:var(--grn)}.t-warn{color:var(--ylw)}.t-red{color:var(--red)}.t-crit{color:#ff0000;font-weight:900}
.stat-row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--bg3);font-size:12px}
.stat-row:last-child{border-bottom:none}
.stat-l{color:var(--muted)}.stat-v{color:var(--txt);font-weight:600}
.toast{position:fixed;bottom:18px;left:50%;transform:translateX(-50%);padding:9px 16px;border-radius:8px;font-size:13px;font-weight:500;z-index:100;opacity:0;transition:opacity .3s;pointer-events:none;max-width:92vw;text-align:center}
.toast.err{background:#b91c1c;color:#fef2f2}.toast.ok{background:#166534;color:#dcfce7}
.toast.show{opacity:1}
</style>
</head>
<body>
<header>
  <div class="dot off" id="dot"></div>
  <h1>openpilot &mdash; Live Tune</h1>
  <div class="hdr-right">
    <input type="text" id="searchInput" placeholder="&#128269; Search params&hellip;" oninput="filterParams(this.value)">
    <button id="themeBtn" class="hdr-btn" onclick="toggleTheme()" title="Toggle light/dark mode">&#9790;</button>
    <button class="hdr-btn" onclick="exportConfig()">&#8595; Export</button>
    <label class="hdr-btn" style="cursor:pointer">&#8593; Import<input type="file" accept=".json" style="display:none" onchange="importConfig(event)"></label>
    <span class="conn" id="connLbl">Connecting&hellip;</span>
  </div>
</header>
<div class="toast err" id="toast"></div>

<div class="tabs">
  <button class="tab active" onclick="showPage('live',this)">Live</button>
  <button class="tab" onclick="showPage('core',this)">Core</button>
  <button class="tab" onclick="showPage('lateral',this)">Lateral</button>
  <button class="tab" onclick="showPage('longitudinal',this)">Longitudinal</button>
  <button class="tab" onclick="showPage('ce',this)">Cond. Exp</button>
  <button class="tab" onclick="showPage('slc',this)">Speed Limit</button>
  <button class="tab" onclick="showPage('safety',this)">Safety</button>
  <button class="tab" onclick="showPage('audio',this)">Audio</button>
  <button class="tab" onclick="showPage('tuning',this)">Tuning</button>
  <button class="tab" onclick="showPage('ui',this)">UI</button>
  <button class="tab" onclick="showPage('device',this)">Device</button>
  <button class="tab" onclick="showPage('mapbox',this)">Mapbox</button>
</div>

<!-- ── LIVE DATA ───────────────────────────────────────────── -->
<div class="page active" id="page-live">
<div class="grid">
  <div class="card">
    <h2>Live Torque Parameters</h2>
    <div class="row"><span class="lbl">Lat Accel Factor</span><span class="val" id="v-latAccelFactor">&mdash;</span></div>
    <div class="row"><span class="lbl">Lat Accel Offset</span><span class="val" id="v-latAccelOffset">&mdash;</span></div>
    <div class="row"><span class="lbl">Friction</span><span class="val" id="v-friction">&mdash;</span></div>
    <div class="row"><span class="lbl">Calibration</span><span class="val" id="v-calPerc">&mdash;</span></div>
    <div class="bar-wrap"><div class="bar" id="calBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:6px"><span class="lbl">Using Live Params</span><span class="val" id="v-useParams">&mdash;</span></div>
  </div>
  <div class="card">
    <h2>Learned Vehicle Parameters</h2>
    <div class="row"><span class="lbl">Steer Ratio</span><span class="val" id="v-steerRatio">&mdash;</span></div>
    <div class="row"><span class="lbl">Stiffness Factor</span><span class="val" id="v-stiffnessFactor">&mdash;</span></div>
    <div class="row"><span class="lbl">Angle Offset</span><span class="val" id="v-angleOffsetDeg">&mdash;</span></div>
    <div class="row"><span class="lbl">Gyro Bias</span><span class="val" id="v-gyroBias">&mdash;</span></div>
    <div class="row"><span class="lbl">Roll</span><span class="val" id="v-roll">&mdash;</span></div>
  </div>
  <div class="card">
    <h2>Car State</h2>
    <div class="row"><span class="lbl">Speed</span><span class="val" id="v-vEgo">&mdash;</span></div>
    <div class="row"><span class="lbl">Steering Angle</span><span class="val" id="v-steeringAngle">&mdash;</span></div>
    <div class="row"><span class="lbl">Engaged</span><span class="val" id="v-engaged">&mdash;</span></div>
    <div class="bs-grid">
      <div class="bs" id="bsLeft">&larr; Left</div>
      <div class="bs" id="bsRight">Right &rarr;</div>
    </div>
  </div>
</div>
</div>

<!-- ── DEVICE (live stats + settings injected below) ──────── -->
<div class="page" id="page-device">
<div class="grid">
  <div class="card">
    <h2>Thermals</h2>
    <div class="row"><span class="lbl">CPU Temp</span><span class="val" id="d-cpuTemp">&mdash;</span></div>
    <div class="row"><span class="lbl">GPU Temp</span><span class="val" id="d-gpuTemp">&mdash;</span></div>
    <div class="row"><span class="lbl">Memory Temp</span><span class="val" id="d-memTemp">&mdash;</span></div>
    <div class="row"><span class="lbl">Max Temp</span><span class="val" id="d-maxTemp">&mdash;</span></div>
    <div class="row"><span class="lbl">Thermal Status</span><span class="val" id="d-thermalStatus">&mdash;</span></div>
    <div class="row"><span class="lbl">Fan Speed</span><span class="val" id="d-fanSpeed">&mdash;</span></div>
  </div>
  <div class="card">
    <h2>System Resources</h2>
    <div class="row"><span class="lbl">RAM Usage</span><span class="val" id="d-memUsage">&mdash;</span></div>
    <div class="bar-wrap"><div class="bar" id="d-memBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:6px"><span class="lbl">Free Storage</span><span class="val" id="d-freeSpace">&mdash;</span></div>
    <div class="bar-wrap"><div class="bar" id="d-storBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:6px"><span class="lbl">CPU Usage (avg)</span><span class="val" id="d-cpuUsage">&mdash;</span></div>
    <div class="row"><span class="lbl">GPU Usage</span><span class="val" id="d-gpuUsage">&mdash;</span></div>
    <div class="row"><span class="lbl">Network</span><span class="val" id="d-network">&mdash;</span></div>
    <div class="row"><span class="lbl">Power Draw</span><span class="val" id="d-power">&mdash;</span></div>
  </div>
  <div class="card">
    <h2>Driving Stats</h2>
    <div class="stat-row"><span class="stat-l">Total Drives</span><span class="stat-v" id="ds-drives">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">Engages</span><span class="stat-v" id="ds-engages">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">Disengages</span><span class="stat-v" id="ds-disengages">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">Overrides</span><span class="stat-v" id="ds-overrides">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">This Month (km)</span><span class="stat-v" id="ds-monthKm">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">Day Drive Time</span><span class="stat-v" id="ds-dayTime">&mdash;</span></div>
    <div class="stat-row"><span class="stat-l">Night Drive Time</span><span class="stat-v" id="ds-nightTime">&mdash;</span></div>
  </div>
</div>
<!-- device settings injected here by buildPages() -->
</div>

<!-- Param pages filled by JS -->
<div class="page" id="page-core"></div>
<div class="page" id="page-lateral"></div>
<div class="page" id="page-longitudinal"></div>
<div class="page" id="page-ce"></div>
<div class="page" id="page-slc"></div>
<div class="page" id="page-safety"></div>
<div class="page" id="page-audio"></div>
<div class="page" id="page-tuning"></div>
<div class="page" id="page-ui"></div>
<div class="page" id="page-mapbox"></div>

<script>
// Apply saved theme immediately to avoid flash of wrong theme
(function(){
  if(localStorage.getItem('theme')==='light'){
    document.body.classList.add('light');
  }
})();

const PARAMS_META = __PARAMS_META__;
const WS_URL = `ws://${location.hostname}:${location.port}/ws`;
let ws = null, retryMs = 1000, currentPage = 'live';

function toggleTheme() {
  const isLight = document.body.classList.toggle('light');
  localStorage.setItem('theme', isLight ? 'light' : 'dark');
  document.getElementById('themeBtn').textContent = isLight ? '\u2600' : '\u263E';
}

// Sync button icon on load
document.addEventListener('DOMContentLoaded', function(){
  const btn = document.getElementById('themeBtn');
  if(btn) btn.textContent = document.body.classList.contains('light') ? '\u2600' : '\u263E';
});

const CAT_TITLES = {
  core:'Core Settings', lateral:'Lateral Control', longitudinal:'Longitudinal Control',
  ce:'Conditional Experimental', slc:'Speed Limit Controller', safety:'Safety & Alerts',
  audio:'Audio & Volumes', tuning:'Advanced Tuning', ui:'UI Customisation',
  device:'Device Settings', mapbox:'Mapbox API Keys',
};

function showPage(id, btn) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById('page-' + id).classList.add('active');
  if (btn) btn.classList.add('active');
  currentPage = id;
  const q = document.getElementById('searchInput').value;
  if (q) filterParams(q);
}

function buildPages() {
  const cats = {};
  for (const [key, meta] of Object.entries(PARAMS_META)) {
    const c = meta.category || 'core';
    if (!cats[c]) cats[c] = [];
    cats[c].push([key, meta]);
  }
  for (const [cat, items] of Object.entries(cats)) {
    const el = document.getElementById('page-' + cat);
    if (!el || !items.length) continue;
    const grid = document.createElement('div');
    grid.className = 'grid';
    grid.style.marginTop = '10px';
    const card = document.createElement('div');
    card.className = 'card';
    card.innerHTML = `<h2>${CAT_TITLES[cat] || cat}</h2>`;
    items.forEach(([key, meta]) => card.appendChild(buildControl(key, meta)));
    grid.appendChild(card);
    el.appendChild(grid);
  }
}

function buildControl(key, meta) {
  const row = document.createElement('div');
  row.className = 'trow';
  row.dataset.search = (meta.label + ' ' + (meta.desc || '')).toLowerCase();
  const hasDefault = meta.default !== undefined && meta.default !== null && meta.default !== '';
  const rst = hasDefault ? `<button class="rst" title="Reset to default" onclick="resetParam('${key}')">&#8635;</button>` : '';
  const ack = `<span class="ack" id="ack-${key}">\u2713</span>`;
  const left = `<div class="tleft"><div class="tlbl">${meta.label}${ack}</div>${meta.desc ? `<div class="tdesc">${meta.desc}</div>` : ''}</div>`;
  if (meta.type === 'string') {
    row.innerHTML = `<div style="width:100%"><div class="tlbl">${meta.label}${ack}</div>${meta.desc ? `<div class="tdesc" style="margin:3px 0 5px">${meta.desc}</div>` : ''}<input class="str-inp" type="${meta.secret?'password':'text'}" id="ctrl-${key}" placeholder="${meta.placeholder||''}" autocomplete="off"><div style="display:flex;align-items:center;gap:8px;margin-top:5px"><button class="save-btn" onclick="applyStr('${key}')">Save</button>${rst}</div></div>`;
    return row;
  }
  let ctrl = '';
  if (meta.type === 'bool') {
    ctrl = `<label class="sw"><input type="checkbox" id="ctrl-${key}" onchange="setParam('${key}',this.checked)"><span class="track"></span></label>`;
  } else if (meta.options) {
    const opts = Object.entries(meta.options).map(([v,l])=>`<option value="${v}">${l}</option>`).join('');
    ctrl = `<select id="ctrl-${key}" onchange="setParam('${key}',${meta.type==='float'?'parseFloat':'parseInt'}(this.value))">${opts}</select>`;
  } else {
    const step = meta.step || (meta.type === 'float' ? 0.01 : 1);
    ctrl = `<div class="num-wrap"><input class="num-inp" type="number" id="ctrl-${key}" step="${step}" min="${meta.min??''}" max="${meta.max??''}" onkeydown="if(event.key==='Enter')applyNum('${key}')"><button class="apply-btn" onclick="applyNum('${key}')">Set</button></div>`;
  }
  row.innerHTML = left + `<div class="ctrl-grp">${ctrl}${rst}</div>`;
  return row;
}

function applyNum(key) {
  const el = document.getElementById('ctrl-' + key), meta = PARAMS_META[key];
  const v = meta.type === 'float' ? parseFloat(el.value) : parseInt(el.value);
  if (!isNaN(v)) setParam(key, v);
}
function applyStr(key) { setParam(key, document.getElementById('ctrl-' + key).value.trim()); }
function resetParam(key) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({action:'reset_param', key})); }

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    document.getElementById('dot').className = 'dot';
    document.getElementById('connLbl').textContent = 'Connected';
    retryMs = 1000; loadParams(); loadStats();
  };
  ws.onclose = ws.onerror = () => {
    document.getElementById('dot').className = 'dot off';
    document.getElementById('connLbl').textContent = 'Reconnecting\u2026';
    setTimeout(connect, retryMs); retryMs = Math.min(retryMs * 2, 16000);
  };
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'live') updateLive(msg.data);
    else if (msg.type === 'ack') {
      if (msg.success) { flashAck(msg.key); if (msg.reset && msg.value !== undefined) applyCtrl(msg.key, msg.value); }
      else showToast('\u26a0 Failed to write "' + msg.key + '"' + (msg.error ? ': ' + msg.error : ''), 'err');
    }
  };
}

function updateLive(d) {
  if (d.torque) {
    sv('latAccelFactor', d.torque.latAccelFactor); sv('latAccelOffset', d.torque.latAccelOffset);
    sv('friction', d.torque.friction); sv('calPerc', d.torque.calPerc + '%');
    document.getElementById('calBar').style.width = d.torque.calPerc + '%';
    const up = document.getElementById('v-useParams');
    up.textContent = d.torque.useParams ? '\u2713 Yes' : '\u2717 No';
    up.style.color = d.torque.useParams ? '#3fb950' : '#f85149';
  }
  if (d.vehicleParams) {
    sv('steerRatio', d.vehicleParams.steerRatio); sv('stiffnessFactor', d.vehicleParams.stiffnessFactor);
    sv('angleOffsetDeg', d.vehicleParams.angleOffsetDeg + '\u00b0');
    sv('gyroBias', d.vehicleParams.gyroBias); sv('roll', d.vehicleParams.roll);
  }
  if (d.carState) {
    sv('vEgo', d.carState.vEgo + ' km/h');
    sv('steeringAngle', d.carState.steeringAngleDeg + '\u00b0');
    updateBS(d.carState.leftBlinker, d.carState.leftBlindspot, d.carState.rightBlinker, d.carState.rightBlindspot);
  }
  if (d.controls) {
    const el = document.getElementById('v-engaged');
    el.textContent = d.controls.enabled ? '\u2713 Engaged' : '\u2717 Off';
    el.style.color = d.controls.enabled ? '#3fb950' : '#8b949e';
  }
  if (d.device) updateDevice(d.device);
}

function sv(id, v) {
  const el = document.getElementById('v-' + id); if (!el) return;
  el.textContent = v; el.classList.add('flash'); setTimeout(() => el.classList.remove('flash'), 300);
}
function st(id, v) { const el = document.getElementById(id); if (el) el.textContent = v; }

function updateBS(blL, bsL, blR, bsR) {
  const L = document.getElementById('bsLeft'), R = document.getElementById('bsRight');
  L.className = 'bs' + (bsL && blL ? ' danger' : bsL ? ' occupied' : blL ? ' blinker' : '');
  R.className = 'bs' + (bsR && blR ? ' danger' : bsR ? ' occupied' : blR ? ' blinker' : '');
  L.innerHTML = bsL ? '\u26a0 LEFT BLOCKED' : '\u2190 Left';
  R.innerHTML = bsR ? 'RIGHT BLOCKED \u26a0' : 'Right \u2192';
}

function updateDevice(d) {
  const tMap = {'thermalstatus.green':'t-ok','thermalstatus.yellow':'t-warn','thermalstatus.red':'t-red','thermalstatus.danger':'t-crit'};
  const tLabel = {'thermalstatus.green':'OK \u2713','thermalstatus.yellow':'Warm','thermalstatus.red':'Hot','thermalstatus.danger':'CRITICAL!'};
  const sk = (d.thermalStatus||'').toLowerCase();
  st('d-cpuTemp', d.cpuTempC + '\u00b0C'); st('d-gpuTemp', d.gpuTempC + '\u00b0C');
  st('d-memTemp', d.memTempC + '\u00b0C'); st('d-maxTemp', d.maxTempC + '\u00b0C');
  const tsEl = document.getElementById('d-thermalStatus');
  if (tsEl) { tsEl.textContent = tLabel[sk] || d.thermalStatus; tsEl.className = 'val ' + (tMap[sk] || ''); }
  st('d-fanSpeed', d.fanSpeedPct + '%');
  st('d-memUsage', d.memUsagePct + '%'); st('d-cpuUsage', d.cpuUsagePct + '%');
  st('d-gpuUsage', d.gpuUsagePct + '%'); st('d-power', d.powerDrawW + ' W');
  const nets = {'networktype.wifi':'Wi-Fi','networktype.cell4g':'4G LTE','networktype.cell5g':'5G','networktype.ethernet':'Ethernet','networktype.none':'None','networktype.cell2g':'2G','networktype.cell3g':'3G'};
  st('d-network', nets[(d.networkType||'').toLowerCase()] || d.networkType || '\u2014');
  st('d-freeSpace', d.freeSpacePct + '%');
  const sBar = document.getElementById('d-storBar');
  if (sBar) { sBar.style.width = (100-d.freeSpacePct)+'%'; sBar.className = 'bar'+(d.freeSpacePct<10?' crit':d.freeSpacePct<30?' warn':''); }
  const mBar = document.getElementById('d-memBar');
  if (mBar) { mBar.style.width = d.memUsagePct+'%'; mBar.className = 'bar'+(d.memUsagePct>90?' crit':d.memUsagePct>70?' warn':''); }
}

async function loadStats() {
  try {
    const s = await (await fetch('/api/stats')).json();
    st('ds-drives', s.drives ?? '\u2014'); st('ds-engages', s.engages ?? '\u2014');
    st('ds-disengages', s.disengages ?? '\u2014'); st('ds-overrides', s.overrides ?? '\u2014');
    st('ds-monthKm', s.monthKm != null ? s.monthKm + ' km' : '\u2014');
    st('ds-dayTime', s.dayTimeHrs != null ? s.dayTimeHrs + ' h' : '\u2014');
    st('ds-nightTime', s.nightTimeHrs != null ? s.nightTimeHrs + ' h' : '\u2014');
  } catch(e) { console.warn('loadStats:', e); }
}

async function loadParams() {
  try {
    const data = await (await fetch('/api/params')).json();
    for (const [key, info] of Object.entries(data)) {
      if (info.value === null || info.value === undefined) continue;
      applyCtrl(key, info.value);
    }
  } catch(e) { console.error('loadParams:', e); }
}

function applyCtrl(key, value) {
  const el = document.getElementById('ctrl-' + key); if (!el) return;
  if (el.type === 'checkbox') el.checked = !!value;
  else if (el.tagName === 'SELECT') el.value = String(value);
  else el.value = value;
}

function setParam(key, value) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({action:'set_param', key, value}));
}

function filterParams(q) {
  const lq = q.toLowerCase();
  const page = document.getElementById('page-' + currentPage); if (!page) return;
  page.querySelectorAll('.trow[data-search]').forEach(row => {
    row.style.display = (!lq || row.dataset.search.includes(lq)) ? '' : 'none';
  });
}

async function exportConfig() {
  try {
    const resp = await fetch('/api/export');
    const blob = await resp.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = 'frogpilot-config.json'; a.click(); URL.revokeObjectURL(url);
  } catch(e) { showToast('Export failed: ' + e.message, 'err'); }
}

async function importConfig(ev) {
  const file = ev.target.files[0]; if (!file) return;
  try {
    const text = await file.text();
    const data = JSON.parse(text);
    const resp = await fetch('/api/import', {method:'POST', body:JSON.stringify(data), headers:{'Content-Type':'application/json'}});
    const r = await resp.json();
    showToast('\u2713 Imported ' + r.written + ' params' + (r.failed?.length ? ' (' + r.failed.length + ' failed)' : ''), r.failed?.length ? 'err' : 'ok');
    setTimeout(loadParams, 500);
  } catch(e) { showToast('Import failed: ' + e.message, 'err'); }
  ev.target.value = '';
}

let _toastTimer = null;
function showToast(msg, type='err') {
  const el = document.getElementById('toast');
  el.textContent = msg; el.className = 'toast ' + type + ' show';
  clearTimeout(_toastTimer); _toastTimer = setTimeout(() => el.classList.remove('show'), 4000);
}

function flashAck(key) {
  const el = document.getElementById('ack-' + key); if (!el) return;
  el.classList.add('show'); setTimeout(() => el.classList.remove('show'), 1800);
}

buildPages();
connect();
</script>
</body>
</html>
"""


async def dashboard_handler(request: web.Request) -> web.Response:
  # Inject PARAMS metadata as JSON so the JS can build controls dynamically
  params_json = json.dumps({k: {ek: ev for ek, ev in v.items() if ek != 'secret'}
                            for k, v in PARAMS.items()})
  html = _HTML.replace('__PARAMS_META__', params_json)
  return web.Response(text=html, content_type='text/html')


async def _on_shutdown(app: web.Application) -> None:
  for ws in list(app['websockets']):
    await ws.close()


def main() -> None:
  logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
  log = logging.getLogger('live_tune')

  app = web.Application()
  app['websockets']: set[web.WebSocketResponse] = set()
  app.on_shutdown.append(_on_shutdown)

  app.router.add_get('/',             dashboard_handler)
  app.router.add_get('/ws',           websocket_handler)
  app.router.add_get('/api/params',   api_get_params)
  app.router.add_post('/api/params',  api_post_params)
  app.router.add_get('/api/stats',    api_stats)
  app.router.add_get('/api/export',   api_export)
  app.router.add_post('/api/import',  api_import)

  log.info('Live Tune Dashboard → http://0.0.0.0:%d', PORT)
  web.run_app(app, host='0.0.0.0', port=PORT, print=None)


if __name__ == '__main__':
  main()
