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
CEREAL_SERVICES = ['liveTorqueParameters', 'liveParameters', 'carState', 'controlsState']

# ── Parameter definitions ────────────────────────────────────────────────────
# type: 'bool' | 'int' | 'float' | 'string'
# category: used to group cards in the UI

PARAMS: dict[str, dict] = {

  # ── Experimental / Personality ──────────────────────────────────────────
  'ExperimentalMode': {
    'type': 'bool', 'label': 'Experimental Mode',
    'category': 'core', 'desc': 'Enable openpilot experimental driving mode',
  },
  'AlphaLongitudinalEnabled': {
    'type': 'bool', 'label': 'Alpha Longitudinal',
    'category': 'core', 'desc': 'Use openpilot for gas/brake on supported cars',
  },
  'LongitudinalPersonality': {
    'type': 'int', 'label': 'Longitudinal Personality', 'category': 'core',
    'min': 0, 'max': 3,
    'options': {0: 'Aggressive', 1: 'Standard', 2: 'Relaxed', 3: 'Traffic'},
    'desc': 'Default following-distance profile',
  },
  'ConditionalExperimental': {
    'type': 'bool', 'label': 'Conditional Experimental Mode',
    'category': 'core', 'desc': 'Auto-switch to experimental mode at intersections, curves, etc.',
  },

  # ── Lateral control ─────────────────────────────────────────────────────
  'AlwaysOnLateral': {
    'type': 'bool', 'label': 'Always On Lateral',
    'category': 'lateral', 'desc': 'Keep steering active even when cruise is disengaged',
  },
  'NudgelessLaneChange': {
    'type': 'bool', 'label': 'Nudgeless Lane Change',
    'category': 'lateral', 'desc': 'Lane change triggers on blinker only (no steering nudge)',
  },
  'OneLaneChange': {
    'type': 'bool', 'label': 'One Lane Change Per Signal',
    'category': 'lateral', 'desc': 'Complete only one lane change per blinker activation',
  },
  'PauseLateralOnSignal': {
    'type': 'bool', 'label': 'Pause Lateral On Signal',
    'category': 'lateral', 'desc': 'Temporarily stop lateral control while blinker is active',
  },
  'TurnDesires': {
    'type': 'bool', 'label': 'Turn Desires',
    'category': 'lateral', 'desc': 'Use turn desires for better low-speed cornering',
  },

  # ── Longitudinal control ─────────────────────────────────────────────────
  'AggressiveFollow': {
    'type': 'float', 'label': 'Aggressive Follow Distance (s)', 'category': 'longitudinal',
    'min': 1.0, 'max': 5.0, 'step': 0.1, 'desc': 'Time gap for Aggressive personality',
  },
  'StandardFollow': {
    'type': 'float', 'label': 'Standard Follow Distance (s)', 'category': 'longitudinal',
    'min': 1.0, 'max': 5.0, 'step': 0.1, 'desc': 'Time gap for Standard personality',
  },
  'RelaxedFollow': {
    'type': 'float', 'label': 'Relaxed Follow Distance (s)', 'category': 'longitudinal',
    'min': 1.0, 'max': 5.0, 'step': 0.1, 'desc': 'Time gap for Relaxed personality',
  },
  'TrafficFollow': {
    'type': 'float', 'label': 'Traffic Follow Distance (s)', 'category': 'longitudinal',
    'min': 0.5, 'max': 5.0, 'step': 0.1, 'desc': 'Time gap for Traffic mode',
  },
  'CurveSpeedController': {
    'type': 'bool', 'label': 'Curve Speed Controller',
    'category': 'longitudinal', 'desc': 'Automatically slow down for curves',
  },
  'HumanAcceleration': {
    'type': 'bool', 'label': 'Human-Like Acceleration',
    'category': 'longitudinal', 'desc': 'Smoother, more natural acceleration profiles',
  },
  'HumanFollowing': {
    'type': 'bool', 'label': 'Human-Like Following',
    'category': 'longitudinal', 'desc': 'Smoother following distance adjustments',
  },
  'AccelerationProfile': {
    'type': 'int', 'label': 'Acceleration Profile', 'category': 'longitudinal',
    'min': 0, 'max': 3,
    'options': {0: 'Normal', 1: 'Eco', 2: 'Sport', 3: 'Sport+'},
    'desc': 'Throttle aggressiveness profile',
  },
  'DecelerationProfile': {
    'type': 'int', 'label': 'Deceleration Profile', 'category': 'longitudinal',
    'min': 0, 'max': 2,
    'options': {0: 'Normal', 1: 'Eco', 2: 'Sport'},
    'desc': 'Braking aggressiveness profile',
  },
  'SpeedLimitController': {
    'type': 'bool', 'label': 'Speed Limit Controller',
    'category': 'longitudinal', 'desc': 'Adjust cruise speed to match posted limit',
  },

  # ── Safety & Alerts ──────────────────────────────────────────────────────
  'LoudBlindspotAlert': {
    'type': 'bool', 'label': 'Loud Blind Spot Alert',
    'category': 'safety', 'desc': 'Louder alert when changing lanes into an occupied blind spot',
  },
  'GreenLightAlert': {
    'type': 'bool', 'label': 'Green Light Alert',
    'category': 'safety', 'desc': 'Alert when traffic light ahead turns green',
  },
  'LeadDepartingAlert': {
    'type': 'bool', 'label': 'Lead Departing Alert',
    'category': 'safety', 'desc': 'Alert when the lead vehicle pulls away after a stop',
  },
  'BlindSpotPath': {
    'type': 'bool', 'label': 'Blind Spot Path Overlay',
    'category': 'safety', 'desc': 'Highlight blind spot zones on the road view',
  },

  # ── Advanced tuning ──────────────────────────────────────────────────────
  'ForceAutoTune': {
    'type': 'bool', 'label': 'Force Auto Tune',
    'category': 'tuning', 'desc': 'Always use live-learned torque parameters (ignores static car data)',
  },
  'NNFF': {
    'type': 'bool', 'label': 'Neural Network Feedforward (NNFF)',
    'category': 'tuning', 'desc': 'ML-based steering feedforward for torque cars',
  },
  'NNFFLite': {
    'type': 'bool', 'label': 'NNFF Lite',
    'category': 'tuning', 'desc': 'Lighter-weight NNFF variant for lower-end hardware',
  },
  'AdvancedLateralTune': {
    'type': 'bool', 'label': 'Advanced Lateral Tune',
    'category': 'tuning', 'desc': 'Unlock advanced lateral tuning options',
  },
  'SteerFriction': {
    'type': 'float', 'label': 'Steer Friction Override', 'category': 'tuning',
    'min': 0.0, 'max': 0.5, 'step': 0.005, 'desc': 'Override learned friction coefficient',
  },
  'SteerKP': {
    'type': 'float', 'label': 'Steer KP Override', 'category': 'tuning',
    'min': 0.1, 'max': 2.0, 'step': 0.05, 'desc': 'Override lateral proportional gain',
  },
  'ForceAutoTuneOff': {
    'type': 'bool', 'label': 'Disable Auto Tune',
    'category': 'tuning', 'desc': 'Permanently disable live torque learning (use static car values)',
  },

  # ── UI Customisation ─────────────────────────────────────────────────────
  'DeveloperUI': {
    'type': 'bool', 'label': 'Developer UI',
    'category': 'ui', 'desc': 'Show CPU/GPU/memory usage, IP address, FPS, etc.',
  },
  'CustomUI': {
    'type': 'bool', 'label': 'Custom UI Elements',
    'category': 'ui', 'desc': 'Enable FrogPilot custom HUD widgets',
  },
  'ModelUI': {
    'type': 'bool', 'label': 'Model Visualisation',
    'category': 'ui', 'desc': 'Enhanced path, lane-line, and lead visualisation',
  },
  'AdjacentPath': {
    'type': 'bool', 'label': 'Adjacent Path Overlay',
    'category': 'ui', 'desc': 'Show adjacent lane path overlays on the HUD',
  },
  'AdjacentPathMetrics': {
    'type': 'bool', 'label': 'Adjacent Path Metrics',
    'category': 'ui', 'desc': 'Show lane-width numbers on adjacent path overlays',
  },
  'PathWidth': {
    'type': 'float', 'label': 'Path Width (m)', 'category': 'ui',
    'min': 1.0, 'max': 4.0, 'step': 0.1, 'desc': 'Visual width of the predicted path overlay',
  },
  'ScreenBrightness': {
    'type': 'int', 'label': 'Screen Brightness (%)', 'category': 'ui',
    'min': 1, 'max': 100, 'desc': 'Onroad display brightness',
  },
  'CameraView': {
    'type': 'int', 'label': 'Camera View', 'category': 'ui',
    'min': 0, 'max': 3,
    'options': {0: 'Auto', 1: 'Wide', 2: 'Driver', 3: 'Rear'},
    'desc': 'Default camera feed shown on HUD',
  },

  # ── Device Management ────────────────────────────────────────────────────
  'HigherBitrate': {
    'type': 'bool', 'label': 'Higher Bitrate Recording',
    'category': 'device', 'desc': 'Record dashcam at higher quality (uses more storage)',
  },
  'IncreaseThermalLimits': {
    'type': 'bool', 'label': 'Increase Thermal Limits',
    'category': 'device', 'desc': 'Allow device to run hotter before throttling (use with caution)',
  },
  'NoLogging': {
    'type': 'bool', 'label': 'Disable Logging',
    'category': 'device', 'desc': 'Do not record any driving data',
  },
  'NoUploads': {
    'type': 'bool', 'label': 'Disable Uploads',
    'category': 'device', 'desc': 'Do not upload driving data to comma/FrogPilot servers',
  },

  # ── Mapbox API Keys ──────────────────────────────────────────────────────
  'MapboxPublicKey': {
    'type': 'string', 'label': 'Mapbox Public Key',
    'category': 'mapbox', 'desc': 'pk.eyJ1… — required for navigation maps',
    'placeholder': 'pk.eyJ1...',
  },
  'MapboxSecretKey': {
    'type': 'string', 'label': 'Mapbox Secret Key',
    'category': 'mapbox', 'desc': 'sk.eyJ1… — required for turn-by-turn routing',
    'placeholder': 'sk.eyJ1...', 'secret': True,
  },
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
        raw = p.get(key)
        if raw is None:
          out[key] = None
          continue
        val = raw.decode() if isinstance(raw, bytes) else str(raw)
        if meta['type'] == 'int':
          out[key] = int(val)
        elif meta['type'] == 'float':
          out[key] = float(val)
        else:
          out[key] = val  # string
    except Exception:
      out[key] = None
  return out


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


# ── Dashboard HTML ────────────────────────────────────────────────────────────

_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>openpilot Live Tune Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:-apple-system,'Segoe UI',sans-serif}
header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;gap:10px;position:sticky;top:0;z-index:10}
header h1{font-size:16px;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:#3fb950;flex-shrink:0}
.dot.off{background:#f85149}
.conn{margin-left:auto;font-size:11px;color:#8b949e}
.tabs{display:flex;gap:4px;padding:10px 16px;background:#0d1117;border-bottom:1px solid #21262d;overflow-x:auto;position:sticky;top:45px;z-index:9}
.tab{padding:5px 14px;border-radius:20px;font-size:12px;font-weight:600;cursor:pointer;white-space:nowrap;background:#21262d;color:#8b949e;border:none;transition:all .15s}
.tab.active{background:#238636;color:#fff}
.page{display:none;padding:14px;max-width:1100px}
.page.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}
.card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px}
.card h2{font-size:11px;font-weight:700;color:#8b949e;text-transform:uppercase;letter-spacing:.07em;margin-bottom:10px}
.row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #21262d}
.row:last-child{border-bottom:none}
.lbl{color:#8b949e;font-size:12px}
.val{font-size:13px;font-weight:600;font-variant-numeric:tabular-nums;color:#58a6ff;transition:color .2s}
.val.flash{color:#3fb950}
.bar-wrap{height:5px;background:#21262d;border-radius:3px;margin-top:5px;overflow:hidden}
.bar{height:100%;background:#238636;border-radius:3px;transition:width .3s}
/* Toggle */
.trow{display:flex;justify-content:space-between;align-items:flex-start;padding:8px 0;border-bottom:1px solid #21262d}
.trow:last-child{border-bottom:none}
.tleft{display:flex;flex-direction:column;gap:2px;flex:1;min-width:0;padding-right:10px}
.tlbl{color:#c9d1d9;font-size:13px;display:flex;align-items:center;gap:6px}
.tdesc{color:#8b949e;font-size:11px}
.ack{font-size:10px;color:#3fb950;opacity:0;transition:opacity .3s}
.ack.show{opacity:1}
.sw{display:inline-block;position:relative;width:40px;height:22px;flex-shrink:0;margin-top:2px}
.sw input{opacity:0;width:0;height:0}
.track{position:absolute;cursor:pointer;top:0;right:0;bottom:0;left:0;background:#555d68;border-radius:22px;transition:background .2s}
.track:before{content:'';position:absolute;width:16px;height:16px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:transform .2s;box-shadow:0 1px 3px rgba(0,0,0,.4)}
input:checked+.track{background:#238636}
input:checked+.track:before{transform:translateX(18px)}
/* Select */
select{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;padding:4px 8px;font-size:12px;cursor:pointer}
/* Number input */
.num-wrap{display:flex;align-items:center;gap:6px}
.num-inp{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;padding:4px 8px;font-size:12px;width:80px;text-align:right}
.num-inp:focus{outline:none;border-color:#388bfd}
.apply-btn{background:#238636;color:#fff;border:none;border-radius:6px;padding:4px 10px;font-size:11px;cursor:pointer;font-weight:600}
.apply-btn:hover{background:#2ea043}
/* Text input */
.str-inp{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;padding:6px 10px;font-size:12px;width:100%;margin-top:6px}
.str-inp:focus{outline:none;border-color:#388bfd}
.save-btn{margin-top:8px;background:#1f6feb;color:#fff;border:none;border-radius:6px;padding:6px 14px;font-size:12px;cursor:pointer;font-weight:600}
.save-btn:hover{background:#388bfd}
/* Blind-spot */
.bs-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
.bs{text-align:center;padding:8px 4px;border-radius:8px;font-size:12px;font-weight:700;background:#21262d;color:#8b949e;border:1px solid transparent;transition:all .15s}
.bs.occupied{background:#ff000022;color:#ff6b6b;border-color:#ff6b6b}
.bs.blinker{background:#ffaa0022;color:#ffaa00;border-color:#ffaa00}
.bs.danger{background:#ff000055;color:#ff4444;border:2px solid #ff4444;animation:bsp .5s infinite alternate}
@keyframes bsp{from{opacity:.7}to{opacity:1}}
/* Software BSM badge */
.sw-bsm-badge{font-size:10px;background:#1f3a5f;color:#58a6ff;border:1px solid #388bfd;border-radius:4px;padding:1px 5px;margin-left:6px}
/* Toast notifications */
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#b91c1c;color:#fef2f2;padding:10px 18px;border-radius:8px;font-size:13px;font-weight:500;z-index:100;opacity:0;transition:opacity .3s;pointer-events:none;max-width:90vw;text-align:center}
.toast.show{opacity:1}
</style>
</head>
<body>

<header>
  <div class="dot off" id="dot"></div>
  <h1>openpilot &mdash; Live Tune Dashboard</h1>
  <span class="conn" id="connLbl">Connecting&hellip;</span>
</header>

<div class="toast" id="toast"></div>
<div class="tabs">
  <button class="tab active" onclick="showPage('live')">Live Data</button>
  <button class="tab" onclick="showPage('core')">Core</button>
  <button class="tab" onclick="showPage('lateral')">Lateral</button>
  <button class="tab" onclick="showPage('longitudinal')">Longitudinal</button>
  <button class="tab" onclick="showPage('safety')">Safety</button>
  <button class="tab" onclick="showPage('tuning')">Tuning</button>
  <button class="tab" onclick="showPage('ui')">UI</button>
  <button class="tab" onclick="showPage('device')">Device</button>
  <button class="tab" onclick="showPage('mapbox')">Mapbox</button>
</div>

<!-- ── LIVE DATA ─────────────────────────────────────────────────── -->
<div class="page active" id="page-live">
<div class="grid">

  <div class="card">
    <h2>Live Torque Parameters</h2>
    <div class="row"><span class="lbl">Lat Accel Factor</span><span class="val" id="v-latAccelFactor">&mdash;</span></div>
    <div class="row"><span class="lbl">Lat Accel Offset</span><span class="val" id="v-latAccelOffset">&mdash;</span></div>
    <div class="row"><span class="lbl">Friction</span><span class="val" id="v-friction">&mdash;</span></div>
    <div class="row"><span class="lbl">Calibration</span><span class="val" id="v-calPerc">&mdash;</span></div>
    <div class="bar-wrap"><div class="bar" id="calBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:8px">
      <span class="lbl">Using Live Params</span>
      <span class="val" id="v-useParams">&mdash;</span>
    </div>
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
    <p style="font-size:10px;color:#484f58;margin-top:6px;text-align:center">
      <span class="sw-bsm-badge">SW BSM</span> = software radar-based detection (legacy vehicles)
    </p>
  </div>

</div>
</div><!-- /page-live -->

<!-- Remaining pages are generated by JS from PARAMS_META -->
<div class="page" id="page-core"></div>
<div class="page" id="page-lateral"></div>
<div class="page" id="page-longitudinal"></div>
<div class="page" id="page-safety"></div>
<div class="page" id="page-tuning"></div>
<div class="page" id="page-ui"></div>
<div class="page" id="page-device"></div>
<div class="page" id="page-mapbox"></div>

<script>
// Injected from server
const PARAMS_META = __PARAMS_META__;

const WS_URL = `ws://${location.hostname}:${location.port}/ws`;
let ws = null, retryMs = 1000;
let paramValues = {};

// ── tab navigation ────────────────────────────────────────────────────────────
function showPage(id) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById('page-' + id).classList.add('active');
  event.currentTarget.classList.add('active');
}

// ── build param pages ─────────────────────────────────────────────────────────
function buildPages() {
  const pages = {core:[],lateral:[],longitudinal:[],safety:[],tuning:[],ui:[],device:[],mapbox:[]};
  for (const [key, meta] of Object.entries(PARAMS_META)) {
    const cat = meta.category || 'core';
    if (pages[cat]) pages[cat].push([key, meta]);
  }
  for (const [cat, items] of Object.entries(pages)) {
    const el = document.getElementById('page-' + cat);
    if (!el || !items.length) continue;
    const grid = document.createElement('div');
    grid.className = 'grid';
    const card = document.createElement('div');
    card.className = 'card';
    card.innerHTML = `<h2>${catTitle(cat)}</h2>`;
    items.forEach(([key, meta]) => {
      card.appendChild(buildControl(key, meta));
    });
    grid.appendChild(card);
    el.appendChild(grid);
  }
}

function catTitle(c) {
  return {core:'Core Settings',lateral:'Lateral Control',longitudinal:'Longitudinal Control',
          safety:'Safety & Alerts',tuning:'Advanced Tuning',ui:'UI Customisation',
          device:'Device Management',mapbox:'Mapbox API Keys'}[c] || c;
}

function buildControl(key, meta) {
  const row = document.createElement('div');
  row.className = 'trow';
  const left = `<div class="tleft"><div class="tlbl">${meta.label}<span class="ack" id="ack-${key}">\u2713</span></div>${meta.desc ? `<div class="tdesc">${meta.desc}</div>` : ''}</div>`;

  let ctrl = '';
  if (meta.type === 'bool') {
    ctrl = `<label class="sw"><input type="checkbox" id="ctrl-${key}" onchange="setParam('${key}',this.checked)"><span class="track"></span></label>`;
  } else if (meta.type === 'int' && meta.options) {
    const opts = Object.entries(meta.options).map(([v,l]) => `<option value="${v}">${l}</option>`).join('');
    ctrl = `<select id="ctrl-${key}" onchange="setParam('${key}',parseInt(this.value))">${opts}</select>`;
  } else if (meta.type === 'int' || meta.type === 'float') {
    const step = meta.step || (meta.type === 'float' ? 0.01 : 1);
    ctrl = `<div class="num-wrap"><input class="num-inp" type="number" id="ctrl-${key}" step="${step}" min="${meta.min ?? ''}" max="${meta.max ?? ''}" onkeydown="if(event.key==='Enter')applyNum('${key}')"><button class="apply-btn" onclick="applyNum('${key}')">Set</button></div>`;
  } else if (meta.type === 'string') {
    // String params get their own layout (full-width)
    row.innerHTML = `<div style="width:100%"><div class="tlbl">${meta.label}</div>${meta.desc ? `<div class="tdesc" style="margin:3px 0 6px">${meta.desc}</div>` : ''}<input class="str-inp" type="${meta.secret ? 'password' : 'text'}" id="ctrl-${key}" placeholder="${meta.placeholder || ''}" autocomplete="off"><button class="save-btn" onclick="applyStr('${key}')">Save</button><span class="ack" id="ack-${key}" style="margin-left:8px;font-size:11px">\u2713 Saved</span></div>`;
    return row;
  }
  row.innerHTML = left + `<div style="flex-shrink:0">${ctrl}</div>`;
  return row;
}

function applyNum(key) {
  const el = document.getElementById('ctrl-' + key);
  const meta = PARAMS_META[key];
  const v = meta.type === 'float' ? parseFloat(el.value) : parseInt(el.value);
  if (!isNaN(v)) setParam(key, v);
}
function applyStr(key) {
  const el = document.getElementById('ctrl-' + key);
  setParam(key, el.value.trim());
}

// ── WebSocket ────────────────────────────────────────────────────────────────
function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    document.getElementById('dot').className = 'dot';
    document.getElementById('connLbl').textContent = 'Connected';
    retryMs = 1000;
    loadParams();
  };
  ws.onclose = ws.onerror = () => {
    document.getElementById('dot').className = 'dot off';
    document.getElementById('connLbl').textContent = 'Reconnecting\u2026';
    setTimeout(connect, retryMs);
    retryMs = Math.min(retryMs * 2, 16000);
  };
  ws.onmessage = e => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'live') updateLive(msg.data);
    else if (msg.type === 'ack') {
      if (msg.success) flashAck(msg.key);
      else showToast(`\u26a0 Failed to write "${msg.key}"` + (msg.error ? `: ${msg.error}` : ''));
    }
  };
}

function updateLive(d) {
  if (d.torque) {
    sv('latAccelFactor', d.torque.latAccelFactor);
    sv('latAccelOffset', d.torque.latAccelOffset);
    sv('friction', d.torque.friction);
    sv('calPerc', d.torque.calPerc + '%');
    document.getElementById('calBar').style.width = d.torque.calPerc + '%';
    const up = document.getElementById('v-useParams');
    up.textContent = d.torque.useParams ? '\u2713 Yes' : '\u2717 No';
    up.style.color = d.torque.useParams ? '#3fb950' : '#f85149';
  }
  if (d.vehicleParams) {
    sv('steerRatio', d.vehicleParams.steerRatio);
    sv('stiffnessFactor', d.vehicleParams.stiffnessFactor);
    sv('angleOffsetDeg', d.vehicleParams.angleOffsetDeg + '\u00b0');
    sv('gyroBias', d.vehicleParams.gyroBias);
    sv('roll', d.vehicleParams.roll);
  }
  if (d.carState) {
    sv('vEgo', d.carState.vEgo + ' km/h');
    sv('steeringAngle', d.carState.steeringAngleDeg + '\u00b0');
    updateBS(d.carState.leftBlinker, d.carState.leftBlindspot,
             d.carState.rightBlinker, d.carState.rightBlindspot);
  }
  if (d.controls) {
    const el = document.getElementById('v-engaged');
    el.textContent = d.controls.enabled ? '\u2713 Engaged' : '\u2717 Off';
    el.style.color = d.controls.enabled ? '#3fb950' : '#8b949e';
  }
}

function sv(id, v) {
  const el = document.getElementById('v-' + id);
  if (!el) return;
  el.textContent = v;
  el.classList.add('flash');
  setTimeout(() => el.classList.remove('flash'), 300);
}

function updateBS(blL, bsL, blR, bsR) {
  const L = document.getElementById('bsLeft');
  const R = document.getElementById('bsRight');
  // bsL/bsR may be software-derived (merged in backend), shown with SW badge if hardware BSM absent
  L.className = 'bs' + (bsL && blL ? ' danger' : bsL ? ' occupied' : blL ? ' blinker' : '');
  R.className = 'bs' + (bsR && blR ? ' danger' : bsR ? ' occupied' : blR ? ' blinker' : '');
  L.innerHTML = bsL ? '\u26a0 LEFT BLOCKED' : '\u2190 Left';
  R.innerHTML = bsR ? 'RIGHT BLOCKED \u26a0' : 'Right \u2192';
}

let _toastTimer = null;
function showToast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => el.classList.remove('show'), 4000);
}

function flashAck(key) {
  const el = document.getElementById('ack-' + key);
  if (!el) return;
  el.classList.add('show');
  setTimeout(() => el.classList.remove('show'), 1800);
}

function setParam(key, value) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({action: 'set_param', key, value}));
  }
}

async function loadParams() {
  try {
    const data = await (await fetch('/api/params')).json();
    for (const [key, info] of Object.entries(data)) {
      if (info.value === null || info.value === undefined) continue;
      const el = document.getElementById('ctrl-' + key);
      if (!el) continue;
      if (el.type === 'checkbox') el.checked = !!info.value;
      else if (el.tagName === 'SELECT') el.value = String(info.value);
      else el.value = info.value;   // number or text input
    }
  } catch (e) { console.error('loadParams:', e); }
}

// ── init ──────────────────────────────────────────────────────────────────────
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

  app.router.add_get('/',            dashboard_handler)
  app.router.add_get('/ws',          websocket_handler)
  app.router.add_get('/api/params',  api_get_params)
  app.router.add_post('/api/params', api_post_params)

  log.info('Live Tune Dashboard → http://0.0.0.0:%d', PORT)
  web.run_app(app, host='0.0.0.0', port=PORT, print=None)


if __name__ == '__main__':
  main()
