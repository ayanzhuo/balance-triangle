// /js/dom.js
export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

export const logEl = $('#log');
export const setStatus = t => $('#status').textContent = '状态: ' + t;
export function logLine(t){ logEl.textContent = String(t); }
export function appendLog(t){
  const el = logEl;
  const atBottom = (el.scrollTop + el.clientHeight >= el.scrollHeight - 2);
  el.textContent += '\n' + t;
  if (atBottom) el.scrollTop = el.scrollHeight;
}
