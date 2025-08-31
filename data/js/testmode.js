// /js/testmode.js
import { $ } from './dom.js';
import { wsSend } from './ws.js';

let currentMode = 0; // 'speed' | 'torque' | 'position'
let enabled = false;

function setActive(btns, mode) {
  btns.forEach(b => b.classList.toggle('active', b.dataset.mode === mode));
}

function clamp10(v) {
  if (!Number.isFinite(v)) return 0;
  if (v > 10) return 10;
  if (v < -10) return -10;
  return v;
}

function sendState(rawValue, once = false) {
  // 后端要求 -10..10，直接发送
  const value = clamp10(rawValue);
  wsSend({
    type: 'test_mode',
    enabled,
    mode,
    value,
    once
  });
}

export function initTestMode() {
  const card = $('#testModeCard');
  const modeBtnsWrap = $('#modeBtns');
  const slider = $('#testValue');
  const numBox = $('#testValueNum');
  const valueOut = $('#testValOut');

  if (!card || !modeBtnsWrap || !slider || !numBox || !valueOut) return;

  // 初值显示
  const syncOut = (v) => { valueOut.textContent = clamp10(v).toFixed(2); };

  // 模式互斥高亮
  const btns = Array.from(modeBtnsWrap.querySelectorAll('button[data-mode]'));
  setActive(btns, currentMode);
  btns.forEach(btn => {
    btn.addEventListener('click', () => {
      currentMode = btn.dataset.mode;
      setActive(btns, currentMode);
      const v = clamp10(parseFloat(slider.value || '0'));
      // 模式切换时带一次性值，便于后端切通道
      sendState(v, true);
    });
  });

  // —— 滑条 → 数字输入（以及显示/发送）——
  const onSlider = () => {
    const v = clamp10(parseFloat(slider.value || '0'));
    numBox.value = v.toFixed(2);
    syncOut(v);
    if (enabled) sendState(v, false);
  };
  slider.addEventListener('input', onSlider);
  slider.addEventListener('change', onSlider);

  // —— 数字输入 → 滑条（以及显示/发送）——
  const onNumber = () => {
    let v = clamp10(parseFloat(numBox.value));
    // 同步回输入框，避免用户输超范围
    numBox.value = Number.isFinite(v) ? v.toFixed(2) : '0.00';
    slider.value = v;            // 推动滑条一起走
    syncOut(v);
    if (enabled) sendState(v, false);
  };
  numBox.addEventListener('input', onNumber);
  numBox.addEventListener('change', onNumber);

  // 初始化一次显示同步
  onSlider();
}

export function enableTestMode(on) {
  enabled = !!on;
  const pidCard = $('#pidCard');
  const testCard = $('#testModeCard');

  if (enabled) {
    pidCard?.classList.add('hidden');
    testCard?.classList.remove('hidden');
  } else {
    testCard?.classList.add('hidden');
    pidCard?.classList.remove('hidden');
  }
  // 通知后端进入/退出（不带值）
  wsSend({ type: 'test_mode', enabled, mode});
}
