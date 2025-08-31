// /js/joystick.js
import { state } from './state.js';
import { $, appendLog } from './dom.js';
import { wsSend } from './ws.js';

export function initJoystick(){
  const joy = $('#joystick'), stick = $('#stick'), out = $('#joyOut');
  const R = joy.clientWidth / 2, H = stick.clientWidth / 2, MAX = R - H;

  function setStick(px,py){ stick.style.left=(px+R)+'px'; stick.style.top=(py+R)+'px'; }
  setStick(0,0);

  function report(px,py){
    const nx = px / MAX, ny = -py / MAX;
    const r = Math.min(1, Math.hypot(nx,ny));
    const ang = Math.atan2(ny,nx) * 180 / Math.PI;
    out.textContent = `x: ${nx.toFixed(2)}  y: ${ny.toFixed(2)}  r: ${r.toFixed(2)}  θ: ${ang.toFixed(1)}°`;
    state.joy.x = nx; state.joy.y = ny; state.joy.deg = ang;
  }

  function posFromEvent(e){
    const rect = joy.getBoundingClientRect(), cx = rect.left + rect.width/2, cy = rect.top + rect.height/2;
    const x = (e.touches? e.touches[0].clientX : e.clientX) - cx;
    const y = (e.touches? e.touches[0].clientY : e.clientY) - cy;
    const d = Math.hypot(x,y); if (d<=MAX) return {x,y}; const k = MAX/d; return {x:x*k,y:y*k};
  }

  function start(e){ state.joy.dragging = true; const p = posFromEvent(e); setStick(p.x,p.y); report(p.x,p.y); send(); }
  function move(e){ if(!state.joy.dragging) return; const p = posFromEvent(e); setStick(p.x,p.y); report(p.x,p.y); }
  function end(){ state.joy.dragging = false; setStick(0,0); report(0,0); wsSend({type:'joy', x:0, y:0, deg:0}); appendLog('[SEND] joy 0,0,0'); }

  joy.addEventListener('pointerdown', e=>{ joy.setPointerCapture(e.pointerId); start(e); });
  joy.addEventListener('pointermove', move);
  joy.addEventListener('pointerup', end);
  joy.addEventListener('pointercancel', end);
  joy.addEventListener('pointerleave', end);
  joy.addEventListener('touchstart', start, {passive:false});
  joy.addEventListener('touchmove', e=>{ e.preventDefault(); move(e); }, {passive:false});
  joy.addEventListener('touchend', end);

  // 20Hz 节流
  setInterval(()=>{ if(!state.joy.dragging) return; const t=performance.now(); if(t-state.joy.lastSend<50) return; state.joy.lastSend=t; send(); }, 20);
  function send(){ wsSend({ type:'joy', x:state.joy.x, y:state.joy.y, deg:state.joy.deg }); }
}
