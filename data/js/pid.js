// /js/pid.js
import { state } from './state.js';
import { $ } from './dom.js';
import { wsSend } from './ws.js';

// 所有需要管理的键（含新增 yawP/yawI/yawD 与 zero）
const KEYS = [
  { k: 'key01', fix: 3 }, { k: 'key02', fix: 3 }, { k: 'key03', fix: 3 },
  { k: 'key04', fix: 3 }, { k: 'key05', fix: 3 }, { k: 'key06', fix: 3 },
  { k: 'key07', fix: 3 }, { k: 'key08', fix: 3 }, { k: 'key09', fix: 3 },
  { k: 'key10', fix: 3 }, { k: 'key11', fix: 3 }, { k: 'key12', fix: 3 },
  { k: 'key13', fix: 2 }
];

// 绑定一个键：滑条与数字框双向联动
function bindKey(k, fix) {
  const sv = $('#sv_' + k);
  const nv = $('#nv_' + k);
  const rg = $('#rg_' + k);

  // 初值写入 state（保持与表单一致）
  if (nv) {
    const v0 = parseFloat(nv.value || '0') || 0;
    state.pidParam[k] = v0;
    if (sv) sv.textContent = v0.toFixed(fix);
    if (rg) {
      const lo = parseFloat(rg.min);
      const hi = parseFloat(rg.max);
      let cv = v0;
      if (Number.isFinite(lo) && cv < lo) cv = lo;
      if (Number.isFinite(hi) && cv > hi) cv = hi;
      rg.value = String(cv);
    }
  }

  // 滑条 → 数字
  if (rg) {
    const onRange = () => {
      const v = parseFloat(rg.value || '0') || 0;
      if (nv) nv.value = v.toFixed(fix);
      if (sv) sv.textContent = v.toFixed(fix);
      state.pidParam[k] = v;
    };
    rg.addEventListener('input', onRange);
    rg.addEventListener('change', onRange);
  }

  // 数字 → 滑条
  if (nv) {
    // ✨ MOD: 允许中间状态（'', '-', '.'），输入时不强制格式化，仅在可解析时同步
    nv.addEventListener('input', () => {
      const raw = nv.value;
      const v = parseFloat(raw);
      if (Number.isFinite(v)) {
        if (sv) sv.textContent = v.toFixed(fix);
        if (rg) {
          // ✨ MOD: 仅滑条被夹紧，文本框不被改写
          const lo = parseFloat(rg.min);
          const hi = parseFloat(rg.max);
          let cv = v;
          if (Number.isFinite(lo) && cv < lo) cv = lo;
          if (Number.isFinite(hi) && cv > hi) cv = hi;
          rg.value = String(cv);
        }
        state.pidParam[k] = v;
      }
      // 无法解析（中间态）则不更新 state / slider
    });

    // ✨ MOD: 在 change / blur / Enter 时统一格式化并更新
    const commit = () => {
      let v = parseFloat(nv.value);
      if (!Number.isFinite(v)) v = 0;
      nv.value = v.toFixed(fix);           // 这时才格式化
      if (sv) sv.textContent = v.toFixed(fix);
      if (rg) {
        const lo = parseFloat(rg.min);
        const hi = parseFloat(rg.max);
        let cv = v;
        if (Number.isFinite(lo) && cv < lo) cv = lo;
        if (Number.isFinite(hi) && cv > hi) cv = hi;
        rg.value = String(cv);
      }
      state.pidParam[k] = v;
    };
    nv.addEventListener('change', commit);
    nv.addEventListener('blur', commit);
    nv.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') commit();
    });
  }
}

export function initPID() {
  // 绑定所有键
  KEYS.forEach(({ k, fix }) => bindKey(k, fix));
  // 按钮沿用原有
  $('#btnPidSend').onclick = () => wsSend({ type: 'set_pid', param: state.pidParam });
  $('#btnPidPull').onclick = () => wsSend({ type: 'get_pid' });
}

// 后端回填：按键名同步到 UI & state
export function fillPidToUI(obj) {
  Object.keys(obj).forEach((k) => {
    if (!(k in state.pidParam)) return; // 未声明的键忽略
    const v = +obj[k];
    if (!Number.isFinite(v)) return;
    const fix = (k === 'zero') ? 2 : 3;

    const sv = document.querySelector('#sv_' + k);
    const nv = document.querySelector('#nv_' + k);
    const rg = document.querySelector('#rg_' + k);

    if (sv) sv.textContent = v.toFixed(fix);
    if (nv) nv.value = v.toFixed(fix);
    if (rg) {
      const lo = parseFloat(rg.min);
      const hi = parseFloat(rg.max);
      let cv = v;
      if (Number.isFinite(lo) && cv < lo) cv = lo;
      if (Number.isFinite(hi) && cv > hi) cv = hi;
      rg.value = String(cv);
    }
    state.pidParam[k] = v;
  });
}

export function applySliderConfig(cfg) {
  if (!Array.isArray(cfg)) return;
  cfg.forEach((g, gi) => {
    const title = document.querySelector(`#pidGroup${gi + 1}`);
    if (title && g.group) title.textContent = g.group;
    if (Array.isArray(g.names)) {
      g.names.forEach((name, si) => {
        const label = document.querySelector(`#pid${gi + 1}Label${si + 1}`);
        if (label) label.textContent = name;
      });
    }
  });
}
