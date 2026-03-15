#!/usr/bin/env python3
"""
Live Tune Dashboard — real-time parameter monitoring and tuning via web browser.

Usage:
  python3 live_tune_server.py

Then open http://<device-ip>:8765 in any browser on the same network.
The dashboard streams live torque/vehicle parameters and allows on-the-fly
adjustment of key openpilot settings without rebooting.
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

# Parameters exposed for live read/write — edit this dict to add more
TUNABLE_PARAMS: dict[str, dict] = {
  'ExperimentalMode': {
    'type': 'bool',
    'label': 'Experimental Mode',
  },
  'AlphaLongitudinalEnabled': {
    'type': 'bool',
    'label': 'Alpha Longitudinal',
  },
  'LongitudinalPersonality': {
    'type': 'int',
    'label': 'Longitudinal Personality',
    'min': 0,
    'max': 3,
    'options': {0: 'Aggressive', 1: 'Standard', 2: 'Relaxed', 3: 'Traffic'},
  },
  'ForceAutoTune': {
    'type': 'bool',
    'label': 'Force Auto Tune (FrogPilot)',
  },
  'TrafficMode': {
    'type': 'bool',
    'label': 'Traffic Mode (FrogPilot)',
  },
}


def _read_all_params() -> dict[str, Any]:
  params = Params()
  out: dict[str, Any] = {}
  for key, meta in TUNABLE_PARAMS.items():
    try:
      if meta['type'] == 'bool':
        out[key] = params.get_bool(key)
      elif meta['type'] == 'int':
        raw = params.get(key)
        out[key] = int(raw.decode() if isinstance(raw, bytes) else raw) if raw is not None else None
      elif meta['type'] == 'float':
        raw = params.get(key)
        out[key] = float(raw.decode() if isinstance(raw, bytes) else raw) if raw is not None else None
      else:
        out[key] = None
    except Exception:
      out[key] = None
  return out


def _write_param(key: str, value: Any) -> bool:
  if key not in TUNABLE_PARAMS:
    return False
  params = Params()
  meta = TUNABLE_PARAMS[key]
  try:
    if meta['type'] == 'bool':
      params.put_bool(key, bool(value))
    elif meta['type'] == 'int':
      params.put(key, str(int(value)))
    elif meta['type'] == 'float':
      params.put(key, str(float(value)))
    return True
  except Exception:
    return False


async def _ws_send_loop(ws: web.WebSocketResponse) -> None:
  """Stream live cereal data to the connected WebSocket client at 4 Hz."""
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
          'enabled':      bool(ctrl.enabled),
          'lateralActive': bool(ctrl.lateralActive),
        }

      if data:
        await ws.send_str(json.dumps({'type': 'live', 'data': data}))
    except Exception as exc:
      logging.getLogger('live_tune').warning('WS send error: %s', exc)

    await asyncio.sleep(0.25)  # 4 Hz


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
            success = _write_param(cmd['key'], cmd['value'])
            await ws.send_str(json.dumps({'type': 'ack', 'key': cmd['key'], 'success': success}))
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
  result = {
    key: {'meta': meta, 'value': values.get(key)}
    for key, meta in TUNABLE_PARAMS.items()
  }
  return web.json_response(result)


async def api_post_params(request: web.Request) -> web.Response:
  body = await request.json()
  key = body.get('key')
  value = body.get('value')
  if not key or value is None:
    raise web.HTTPBadRequest(text='Missing key or value')
  if not _write_param(key, value):
    raise web.HTTPBadRequest(text=f'Unknown or invalid parameter: {key}')
  return web.json_response({'success': True})


_DASHBOARD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>openpilot Live Tune Dashboard</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d1117;color:#e6edf3;font-family:-apple-system,'Segoe UI',sans-serif;min-height:100vh}
  header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;gap:10px}
  header h1{font-size:18px;font-weight:600}
  .dot{width:10px;height:10px;border-radius:50%;background:#3fb950;flex-shrink:0}
  .dot.off{background:#f85149}
  .conn{margin-left:auto;font-size:12px;color:#8b949e}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:14px;padding:16px}
  .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:16px}
  .card h2{font-size:11px;font-weight:700;color:#8b949e;text-transform:uppercase;letter-spacing:.07em;margin-bottom:12px}
  .row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #21262d}
  .row:last-child{border-bottom:none}
  .lbl{color:#8b949e;font-size:13px}
  .val{font-size:14px;font-weight:600;font-variant-numeric:tabular-nums;color:#58a6ff;transition:color .2s}
  .val.flash{color:#3fb950}
  .bar-wrap{height:6px;background:#21262d;border-radius:3px;margin-top:6px;overflow:hidden}
  .bar{height:100%;background:#238636;border-radius:3px;transition:width .3s}
  .toggle-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0}
  .toggle-lbl{color:#c9d1d9;font-size:14px}
  .sw{position:relative;width:44px;height:24px;flex-shrink:0}
  .sw input{opacity:0;width:0;height:0}
  .track{position:absolute;cursor:pointer;inset:0;background:#30363d;border-radius:24px;transition:background .2s}
  .track:before{content:'';position:absolute;width:18px;height:18px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:transform .2s}
  input:checked+.track{background:#238636}
  input:checked+.track:before{transform:translateX(20px)}
  .sel-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0}
  select{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;padding:5px 8px;font-size:13px;cursor:pointer}
  .bs-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .bs{text-align:center;padding:10px 6px;border-radius:8px;font-size:13px;font-weight:600;background:#21262d;color:#8b949e;border:1px solid transparent;transition:all .15s}
  .bs.occupied{background:#ff000022;color:#ff6b6b;border-color:#ff6b6b}
  .bs.blinker{background:#ffaa0022;color:#ffaa00;border-color:#ffaa00}
  .bs.danger{background:#ff000055;color:#ff4444;border:2px solid #ff4444;animation:pulse .5s infinite alternate}
  @keyframes pulse{from{opacity:.7}to{opacity:1}}
  .ack{font-size:11px;color:#3fb950;margin-left:6px;opacity:0;transition:opacity .3s}
  .ack.show{opacity:1}
</style>
</head>
<body>
<header>
  <div class="dot off" id="dot"></div>
  <h1>openpilot &mdash; Live Tune Dashboard</h1>
  <span class="conn" id="connLbl">Connecting&hellip;</span>
</header>

<div class="grid">

  <!-- Live Torque -->
  <div class="card">
    <h2>Live Torque Parameters</h2>
    <div class="row"><span class="lbl">Lat Accel Factor</span><span class="val" id="v-latAccelFactor">&mdash;</span></div>
    <div class="row"><span class="lbl">Lat Accel Offset</span><span class="val" id="v-latAccelOffset">&mdash;</span></div>
    <div class="row"><span class="lbl">Friction</span><span class="val" id="v-friction">&mdash;</span></div>
    <div class="row">
      <span class="lbl">Calibration</span>
      <span class="val" id="v-calPerc">&mdash;</span>
    </div>
    <div class="bar-wrap"><div class="bar" id="calBar" style="width:0%"></div></div>
    <div class="row" style="margin-top:8px">
      <span class="lbl">Using Live Params</span>
      <span class="val" id="v-useParams">&mdash;</span>
    </div>
  </div>

  <!-- Vehicle Params -->
  <div class="card">
    <h2>Learned Vehicle Parameters</h2>
    <div class="row"><span class="lbl">Steer Ratio</span><span class="val" id="v-steerRatio">&mdash;</span></div>
    <div class="row"><span class="lbl">Stiffness Factor</span><span class="val" id="v-stiffnessFactor">&mdash;</span></div>
    <div class="row"><span class="lbl">Angle Offset</span><span class="val" id="v-angleOffsetDeg">&mdash;</span></div>
    <div class="row"><span class="lbl">Gyro Bias</span><span class="val" id="v-gyroBias">&mdash;</span></div>
    <div class="row"><span class="lbl">Roll</span><span class="val" id="v-roll">&mdash;</span></div>
  </div>

  <!-- Car State & Blind Spots -->
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

  <!-- Live Adjustments -->
  <div class="card">
    <h2>Live Adjustments</h2>

    <div class="toggle-row">
      <span class="toggle-lbl">Experimental Mode<span class="ack" id="ack-ExperimentalMode">&#10003;</span></span>
      <label class="sw">
        <input type="checkbox" id="ExperimentalMode"
               onchange="setParam('ExperimentalMode', this.checked)">
        <span class="track"></span>
      </label>
    </div>

    <div class="toggle-row">
      <span class="toggle-lbl">Alpha Longitudinal<span class="ack" id="ack-AlphaLongitudinalEnabled">&#10003;</span></span>
      <label class="sw">
        <input type="checkbox" id="AlphaLongitudinalEnabled"
               onchange="setParam('AlphaLongitudinalEnabled', this.checked)">
        <span class="track"></span>
      </label>
    </div>

    <div class="toggle-row">
      <span class="toggle-lbl">Force Auto Tune<span class="ack" id="ack-ForceAutoTune">&#10003;</span></span>
      <label class="sw">
        <input type="checkbox" id="ForceAutoTune"
               onchange="setParam('ForceAutoTune', this.checked)">
        <span class="track"></span>
      </label>
    </div>

    <div class="toggle-row">
      <span class="toggle-lbl">Traffic Mode<span class="ack" id="ack-TrafficMode">&#10003;</span></span>
      <label class="sw">
        <input type="checkbox" id="TrafficMode"
               onchange="setParam('TrafficMode', this.checked)">
        <span class="track"></span>
      </label>
    </div>

    <div class="sel-row">
      <span class="toggle-lbl">Personality<span class="ack" id="ack-LongitudinalPersonality">&#10003;</span></span>
      <select id="LongitudinalPersonality"
              onchange="setParam('LongitudinalPersonality', parseInt(this.value))">
        <option value="0">Aggressive</option>
        <option value="1">Standard</option>
        <option value="2">Relaxed</option>
        <option value="3">Traffic</option>
      </select>
    </div>
  </div>

</div><!-- .grid -->

<script>
const WS_URL = `ws://${location.hostname}:${location.port}/ws`;
let ws = null, retryMs = 1000;

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
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.type === 'live') updateLive(msg.data);
    else if (msg.type === 'ack' && msg.success) flashAck(msg.key);
  };
}

function updateLive(d) {
  if (d.torque) {
    setVal('latAccelFactor', d.torque.latAccelFactor);
    setVal('latAccelOffset', d.torque.latAccelOffset);
    setVal('friction', d.torque.friction);
    setVal('calPerc', d.torque.calPerc + '%');
    document.getElementById('calBar').style.width = d.torque.calPerc + '%';
    const up = document.getElementById('v-useParams');
    up.textContent = d.torque.useParams ? '\u2713 Yes' : '\u2717 No';
    up.style.color = d.torque.useParams ? '#3fb950' : '#f85149';
  }
  if (d.vehicleParams) {
    setVal('steerRatio', d.vehicleParams.steerRatio);
    setVal('stiffnessFactor', d.vehicleParams.stiffnessFactor);
    setVal('angleOffsetDeg', d.vehicleParams.angleOffsetDeg + '\u00b0');
    setVal('gyroBias', d.vehicleParams.gyroBias);
    setVal('roll', d.vehicleParams.roll);
  }
  if (d.carState) {
    setVal('vEgo', d.carState.vEgo + ' km/h');
    setVal('steeringAngle', d.carState.steeringAngleDeg + '\u00b0');
    updateBS(d.carState.leftBlinker,  d.carState.leftBlindspot,
             d.carState.rightBlinker, d.carState.rightBlindspot);
  }
  if (d.controls) {
    const el = document.getElementById('v-engaged');
    el.textContent = d.controls.enabled ? '\u2713 Engaged' : '\u2717 Off';
    el.style.color = d.controls.enabled ? '#3fb950' : '#8b949e';
  }
}

function setVal(id, v) {
  const el = document.getElementById('v-' + id);
  if (!el) return;
  el.textContent = v;
  el.classList.add('flash');
  setTimeout(() => el.classList.remove('flash'), 300);
}

function updateBS(blL, bsL, blR, bsR) {
  const L = document.getElementById('bsLeft');
  const R = document.getElementById('bsRight');
  L.className = 'bs' + (bsL && blL ? ' danger' : bsL ? ' occupied' : blL ? ' blinker' : '');
  R.className = 'bs' + (bsR && blR ? ' danger' : bsR ? ' occupied' : blR ? ' blinker' : '');
  L.innerHTML = bsL ? '\u26a0 LEFT BLOCKED' : '\u2190 Left';
  R.innerHTML = bsR ? 'RIGHT BLOCKED \u26a0' : 'Right \u2192';
}

function flashAck(key) {
  const el = document.getElementById('ack-' + key);
  if (!el) return;
  el.classList.add('show');
  setTimeout(() => el.classList.remove('show'), 1500);
}

function setParam(key, value) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ action: 'set_param', key, value }));
  }
}

async function loadParams() {
  try {
    const data = await (await fetch('/api/params')).json();
    for (const [key, info] of Object.entries(data)) {
      if (info.value === null || info.value === undefined) continue;
      const el = document.getElementById(key);
      if (!el) continue;
      if (el.type === 'checkbox') el.checked = !!info.value;
      else if (el.tagName === 'SELECT') el.value = String(info.value);
    }
  } catch (e) { console.error('loadParams:', e); }
}

connect();
</script>
</body>
</html>
"""


async def dashboard_handler(request: web.Request) -> web.Response:
  return web.Response(text=_DASHBOARD_HTML, content_type='text/html')


async def _on_shutdown(app: web.Application) -> None:
  for ws in list(app['websockets']):
    await ws.close()


def main() -> None:
  logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
  log = logging.getLogger('live_tune')

  app = web.Application()
  app['websockets']: set[web.WebSocketResponse] = set()
  app.on_shutdown.append(_on_shutdown)

  app.router.add_get('/',           dashboard_handler)
  app.router.add_get('/ws',         websocket_handler)
  app.router.add_get('/api/params', api_get_params)
  app.router.add_post('/api/params', api_post_params)

  log.info('Live Tune Dashboard → http://0.0.0.0:%d', PORT)
  log.info('Open http://<device-ip>:%d in your browser', PORT)
  web.run_app(app, host='0.0.0.0', port=PORT, print=None)


if __name__ == '__main__':
  main()
