// /js/state.js
export const state = {
  connected: false,
  chartsOn: true,
  vis: true,
  joy: { x: 0, y: 0, deg: 0, dragging: false, lastSend: 0 },
  pidParam: {},               // 9 个 PID
  attZero: { roll: 0, yaw: 0 },
  charts: { ch1: null, ch2: null, ch3: null },
  three: { scene: null, camera: null, renderer: null, robot: null, pcb: null }
};
