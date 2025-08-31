// /js/led.js  —— LED 控制台（独立于 app.js，可与其共享 /ws 通道）
const els = {
  mode: document.getElementById('ledMode'),
  dir: document.getElementById('ledDir'),
  color: document.getElementById('ledColor'),
  interval: document.getElementById('ledInterval'),
  bright: document.getElementById('ledBright'),
  decay: document.getElementById('ledDecay'),
  theater: document.getElementById('ledTheaterStep'),
  breathSpd: document.getElementById('ledBreathSpd'),
  rainbowSpd: document.getElementById('ledRainbowSpd'),
  apply: document.getElementById('ledApplyBtn'),
  read: document.getElementById('ledReadBtn'),
  power: document.getElementById('ledPowerBtn'),
  blackout: document.getElementById('ledBlackoutBtn'),
  preview: document.getElementById('ledPreviewBtn'),
  state: document.getElementById('ledState'),
  strip: document.getElementById('ledStripPreview'),
};

const NUM = 21; // 你的硬件数量
let ws, connected = false;

// -------- 预览条渲染 ----------
function initPreview() {
  if (!els.strip) return;
  els.strip.innerHTML = '';
  for (let i = 0; i < NUM; i++) {
    const d = document.createElement('div');
    d.className = 'led-dot';
    d.dataset.idx = String(i);
    els.strip.appendChild(d);
  }
}
function hexToRgb(hex) {
  const m = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return m ? { r: parseInt(m[1],16), g: parseInt(m[2],16), b: parseInt(m[3],16) } : {r:0,g:0,b:0};
}
function paintPreview(mode, phase = 0) {
  const dots = els.strip?.children; if (!dots) return;
  const n = dots.length;
  const {r,g,b} = hexToRgb(els.color.value);
  const dir = parseInt(els.dir.value, 10) || 1;

  // 清空
  for (let i = 0; i < n; i++) dots[i].style.background = 'rgb(12,12,12)';

  const wrap = i => (i % n + n) % n;
  switch (parseInt(mode,10)) {
    case 0: // CHASE
      dots[wrap(phase*dir)].style.background = `rgb(${r},${g},${b})`;
      break;
    case 1: // COMET
      for (let t = 0; t < 5; t++) {
        const k = Math.max(0, 1 - t*0.22);
        const ir = Math.round(r*k), ig = Math.round(g*k), ib = Math.round(b*k);
        dots[wrap((phase - t)*dir)].style.background = `rgb(${ir},${ig},${ib})`;
      }
      break;
    case 2: // BOUNCE
      {
        const cycle = (n-1)*2;
        const p = phase % cycle;
        const idx = p < n ? p : (cycle - p);
        dots[idx].style.background = `rgb(${r},${g},${b})`;
      }
      break;
    case 3: // THEATER
      {
        const step = parseInt(els.theater.value,10) || 3;
        for (let i = 0; i < n; i++)
          if (((i + phase) % step) === 0)
            dots[i].style.background = `rgb(${r},${g},${b})`;
      }
      break;
    case 4: // RAINBOW
      for (let i = 0; i < n; i++) {
        const hue = (i*256/n + phase) & 255;
        let rr,gg,bb, pos = 255 - hue;
        if (pos < 85)      { rr = 255 - pos*3; gg = 0;          bb = pos*3; }
        else if (pos <170){ pos-=85; rr = 0;           gg = pos*3; bb = 255-pos*3; }
        else              { pos-=170; rr = pos*3;      gg = 255-pos*3; bb = 0; }
        dots[i].style.background = `rgb(${rr},${gg},${bb})`;
      }
      break;
    case 5: // BREATH
      {
        const k = 0.5 - 0.5 * Math.cos(2 * Math.PI * ((phase % 256) / 256));
        dots.forEach?.(() => { }); // noop for HTMLCollection
        for (let i = 0; i < n; i++) {
          const ir = Math.round(r * k), ig = Math.round(g * k), ib = Math.round(b * k);
          dots[i].style.background = `rgb(${ir},${ig},${ib})`;
        }
      }
      break;
    case 6: // 自定义模式1
      {
        // 自定义模式1的实现
      }
      break;
    case 7: // 自定义模式2
      {
        // 自定义模式2的实现
      }
      break;
    case 8: // 自定义模式3
      {
        // 自定义模式3的实现
      }
      break;
  }
}

// 简单预览动画（本地不影响设备）
let previewTimer = null, previewPhase = 0;
function startPreview() {
  stopPreview();
  const iv = Math.max(5, Math.min(200, parseInt(els.interval.value,10)||40));
  previewTimer = setInterval(() => {
    paintPreview(els.mode.value, previewPhase++);
  }, iv);
}
function stopPreview() {
  if (previewTimer) { clearInterval(previewTimer); previewTimer = null; }
}

// -------- WebSocket ----------
function connectWS() {
  try {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => { connected = true; els.state.textContent = '已连接'; };
    ws.onclose = () => { connected = false; els.state.textContent = '未连接'; };
    ws.onerror = () => { connected = false; els.state.textContent = '连接错误'; };
    ws.onmessage = ev => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'led_state') {
          // 回填设备状态
          const s = msg.state || {};
          els.mode.value = s.mode ?? els.mode.value;
          els.dir.value  = s.dir ?? els.dir.value;
          els.interval.value = s.interval ?? els.interval.value;
          els.bright.value   = s.brightness ?? els.bright.value;
          if (s.color) els.color.value = s.color;
          els.decay.value = s.tailDecay ?? els.decay.value;
          els.theater.value = s.theaterStep ?? els.theater.value;
          els.breathSpd.value = s.breathSpeed ?? els.breathSpd.value;
          els.rainbowSpd.value = s.rainbowSpeed ?? els.rainbowSpd.value;
          els.state.textContent = '已读取';
        }
      } catch {}
    };
  } catch (e) {
    console.error(e);
  }
}

// -------- 发送命令 ----------
function send(type, body={}) {
  if (!connected || !ws) return;
  ws.send(JSON.stringify({ type, ...body }));
}
function applyToDevice() {
  const payload = {
    mode: parseInt(els.mode.value,10)||0,
    dir: parseInt(els.dir.value,10)||1,
    interval: Math.max(5, Math.min(200, parseInt(els.interval.value,10)||40)),
    brightness: Math.max(1, Math.min(255, parseInt(els.bright.value,10)||80)),
    color: els.color.value, // "#rrggbb"
    tailDecay: Math.max(0, Math.min(255, parseInt(els.decay.value,10)||48)),
    theaterStep: Math.max(2, Math.min(6, parseInt(els.theater.value,10)||3)),
    breathSpeed: Math.max(1, Math.min(16, parseInt(els.breathSpd.value,10)||8)),
    rainbowSpeed: Math.max(1, Math.min(16, parseInt(els.rainbowSpd.value,10)||4)),
  };
  send('led_set', { cfg: payload });
  els.state.textContent = '已下发';
}

// -------- 事件绑定 ----------
function bindUI() {
  els.apply?.addEventListener('click', applyToDevice);
  els.read?.addEventListener('click', () => send('led_get'));
  els.power?.addEventListener('click', () => send('led_power', { on: true }));
  els.blackout?.addEventListener('click', () => send('led_power', { on: false }));
  els.preview?.addEventListener('click', () => {
    if (previewTimer) { stopPreview(); els.preview.textContent = '本地预览'; }
    else { startPreview(); els.preview.textContent = '停止预览'; }
  });

  // UI动则改预览
  ['mode','dir','color','interval','bright','decay','theater','breathSpd','rainbowSpd'].forEach(id=>{
    const e = els[id]; e && e.addEventListener('input', ()=> {
      if (previewTimer) { stopPreview(); startPreview(); }
      else { paintPreview(els.mode.value, previewPhase); }
    });
  });
}

(function main(){
  initPreview();
  paintPreview(els.mode?.value ?? 0, 0);
  bindUI();
  connectWS();
})();
