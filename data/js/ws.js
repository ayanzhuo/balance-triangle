// /js/ws.js
import { state } from './state.js';
import { setStatus, appendLog } from './dom.js';
import { feedCharts, applyChartConfig } from './charts.js';
import { setAttitude } from './att3d.js';
import { fillPidToUI, applySliderConfig } from './pid.js';

let ws = null, timer = null;

// 允许外部订阅 telemetry（例如 app.js 用来更新摔倒指示灯）
let _onTelemetry = null;
export function setOnTelemetry(fn) { _onTelemetry = fn; }

export function wsSend(obj) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

export function openWS() {
  const protocol = (location.protocol === 'http:') ? 'ws://' : 'wss://';
  const url = protocol + location.host + '/ws';
  try {
    ws = new WebSocket(url);
    ws.onopen = () => {
      state.connected = true; setStatus('connected');
      wsSend({ type: 'get_pid' }); appendLog('[SEND] get_pid');
    };
    ws.onclose = () => { state.connected = false; setStatus('reconnecting…'); retry(); };
    ws.onerror  = () => { state.connected = false; setStatus('error'); };
    ws.onmessage = onMsg;
  } catch (e) {
    retry();
  }
}
function retry() { clearTimeout(timer); timer = setTimeout(openWS, 1000); }

function onMsg(ev) {
  let msg = null; try { msg = JSON.parse(ev.data); } catch {}
  if (!msg || !msg.type) return;

  switch (msg.type) {
    case 'telemetry': {
      if (Number.isFinite(msg.pitch) && Number.isFinite(msg.roll) && Number.isFinite(msg.yaw)) {
        setAttitude(msg.pitch, msg.roll, msg.yaw);
      }
      if (state.chartsOn && Array.isArray(msg.d) && msg.d.length >= 9) {
        feedCharts(...msg.d);
      }
      // 通知订阅者（例如 app.js）做额外处理：摔倒指示灯等
      if (_onTelemetry) _onTelemetry(msg);
      break;
    }
    case 'ui_config': {
      if (Array.isArray(msg.charts)) applyChartConfig(msg.charts);
      if (Array.isArray(msg.sliders)) applySliderConfig(msg.sliders);
      break;
    }
    case 'pid': {
      if (msg.param) fillPidToUI(msg.param);
      break;
    }
    case 'info': {
      if (msg.text) appendLog('[INFO] ' + msg.text);
      break;
    }
  }
}

export async function syncState() {
  try {
    const r = await fetch('/api/state');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const s = await r.json();
    if (typeof s.hz === 'number') {
      const el = document.querySelector('#rateHz');
      if (el) el.value = s.hz;
    }
    if (typeof s.running === 'boolean') {
      const sw = document.querySelector('#runSwitch');
      if (sw) sw.checked = !!s.running;
    }
    appendLog('[INIT] /api/state ok -> running=' + s.running + ', hz=' + s.hz);
  } catch (e) {
    appendLog('[INIT] /api/state fail: ' + e.message);
  }
}
