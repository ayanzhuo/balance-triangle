// /js/att3d.js
import { state } from './state.js';
import { $, appendLog } from './dom.js';

// ====== 基础构造工具 ======
const mkMat = (hex,m=.1,r=.6)=> new THREE.MeshStandardMaterial({ color:hex, metalness:m, roughness:r });
const mkBox = (w,h,d,mat)=> new THREE.Mesh(new THREE.BoxGeometry(w,h,d), mat);
const mkCyl = (rT,rB,h,mat,seg=36)=> new THREE.Mesh(new THREE.CylinderGeometry(rT,rB,h,seg), mat);

// ====== 车轮（如需）保留（当前不使用，但留着以便扩展） ======
function createMainWheel(radius=0.18, width=0.08){
  const grp = new THREE.Group();
  const tire = mkCyl(radius, radius, width, mkMat(0x2a2d31, .25, .4), 36); tire.rotation.z = Math.PI/2; grp.add(tire);
  const rim  = mkCyl(radius*.82, radius*.82, width*.55, mkMat(0x606a76, .25, .55), 36); rim.rotation.z = Math.PI/2; grp.add(rim);
  const hub  = mkCyl(radius*.28, radius*.28, width*.7, mkMat(0x9aa3ad, .2, .4), 24); hub.rotation.z = Math.PI/2; grp.add(hub);
  return grp;
}

// ====== 三棱柱机身（倒放） ======
// 使用 CylinderGeometry 的 3 段（seg=3）近似三棱柱：
// - 默认长轴沿 Y，这里转到 Z 轴当作机身“长度”
// - 通过 rotation.z 微调，使三角截面一个顶点朝下
function createTriPrism(len = 0.90, rad = 0.26) {
  const grp = new THREE.Group();

  // 主体：三棱柱（沿 Z 轴）
  const tri = new THREE.Mesh(
    new THREE.CylinderGeometry(rad, rad, len, 3, 1, false),
    mkMat(0x5a6069, .2, .6)
  );
  tri.rotation.x = Math.PI / 2; // 把“长度”对齐到 Z 轴
  tri.rotation.z = Math.PI / 6; // 让三角的一只顶点朝下，可微调角度

  grp.add(tri);

  // 装饰条：给一点层次感（可删）
  const stripe = new THREE.Mesh(
    new THREE.BoxGeometry(len * 0.95, 0.02, 0.04),
    mkMat(0xd7dbe2, .1, .9)
  );
  stripe.rotation.y = Math.PI / 2; // 条纹沿 Z
  stripe.position.y = 0.05;        // 稍稍离开中心
  grp.add(stripe);

  return grp;
}

// ====== 初始化 3D 场景 ======
export function init3D(){
  if (typeof THREE === 'undefined') { appendLog('[WARN] Three.js 未加载'); return; }
  const host = $('#robotCanvas'); host.innerHTML = "";
  const scene = new THREE.Scene(); scene.background = new THREE.Color(0xf6f7fb);

  const w = host.clientWidth, h = 360;
  const camera = new THREE.PerspectiveCamera(40, w/h, .1, 100);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(w,h); renderer.setPixelRatio(window.devicePixelRatio||1);
  host.appendChild(renderer.domElement);

  // 光照与地面
  scene.add(new THREE.HemisphereLight(0xffffff,0x8899aa,.95));
  const dir = new THREE.DirectionalLight(0xffffff,.8); dir.position.set(3,5,2); scene.add(dir);
  const grid = new THREE.GridHelper(12,24,0xcccccc,0xeeeeee); grid.position.y=0; scene.add(grid);

  // 轨道相机（简易）
  const orbit = { target:new THREE.Vector3(0,0.7,0), r:5.0, theta:0.8, phi:1.0, minR:2, maxR:10 };
  function applyOrbit(){
    const {r,theta,phi,target} = orbit;
    const p = new THREE.Vector3(r*Math.sin(phi)*Math.cos(theta), r*Math.cos(phi), r*Math.sin(phi)*Math.sin(theta));
    camera.position.copy(target.clone().add(p));
    camera.lookAt(target);
  }
  applyOrbit();

  // 机器人实体：以倒放三棱柱替代原底盘/车轮
  const robot = new THREE.Group(); scene.add(robot);
  const triRad = 0.26;          // 与 createTriPrism 的 rad 一致
  const triLen = 0.90;          // 机身长度（沿 Z）
  const tri = createTriPrism(triLen, triRad);
  robot.add(tri);

  // 让最低的三角顶点恰落地（grid 在 y=0）
  // 三角截面最低点约为 -rad，因此整体上抬 rad 即可
  robot.position.y = triRad;

  // （可选）若你想更真实的“支点”视觉：
  // const tip = new THREE.Mesh(new THREE.SphereGeometry(0.01,16,16), mkMat(0x333333,.2,.5));
  // tip.position.set(0, -triRad, 0); // 三角最低点附近
  // robot.add(tip);

  // 交互：拖拽旋转/平移 + 滚轮缩放
  const el = renderer.domElement;
  let isDrag=false, btn=0, lastX=0,lastY=0;
  el.addEventListener('pointerdown', e=>{ isDrag=true; btn=e.button; lastX=e.clientX; lastY=e.clientY; el.setPointerCapture(e.pointerId); });
  el.addEventListener('pointerup', e=>{ isDrag=false; el.releasePointerCapture(e.pointerId); });
  el.addEventListener('pointerleave', ()=>{ isDrag=false; });
  el.addEventListener('contextmenu', e=> e.preventDefault());
  el.addEventListener('pointermove', e=>{
    if(!isDrag) return;
    const dx=e.clientX-lastX, dy=e.clientY-lastY; lastX=e.clientX; lastY=e.clientY;
    const rotS=0.005, panS=0.002*orbit.r;
    if(btn===2 || e.ctrlKey){
      const right=new THREE.Vector3(); camera.getWorldDirection(right); right.cross(camera.up).normalize();
      const up=camera.up.clone().normalize();
      orbit.target.addScaledVector(right, -dx*panS);
      orbit.target.addScaledVector(up, dy*panS);
    }else{
      orbit.theta -= dx*rotS;
      orbit.phi   -= dy*rotS;
      const eps=0.05; orbit.phi = Math.max(eps, Math.min(Math.PI-eps, orbit.phi));
    }
    applyOrbit();
  });
  el.addEventListener('wheel', e=>{
    e.preventDefault();
    const k=Math.pow(1.1, e.deltaY>0?1:-1);
    orbit.r = Math.max(orbit.minR, Math.min(orbit.maxR, orbit.r*k));
    applyOrbit();
  }, {passive:false});
  window.addEventListener('resize', ()=>{
    const w2 = host.clientWidth;
    camera.aspect=w2/h; camera.updateProjectionMatrix();
    renderer.setSize(w2,h);
  });

  // 高度联动（仍然有效，如果你想锁死也可移除）
  const hs = document.querySelector('#heightSlider'), hv = document.querySelector('#heightVal');
  const applyH = ()=>{
    const y = parseFloat(hs?.value ?? '0');
    robot.position.y = triRad + y; // 以三角顶点触地为基准，再增加高度偏移
    if (hv) hv.textContent = y.toFixed(2)+' m';
  };
  if (hs) { hs.addEventListener('input', applyH); hs.addEventListener('change', applyH); }
  applyH();

  state.three = { scene, camera, renderer, robot, pcb: null }; // pcb 不再使用，置空
  setAttitude(0,0,0);
  renderer.setAnimationLoop(()=> renderer.render(scene,camera));
  appendLog('[INIT] 3D ready (tri-prism)');
}

// ====== 姿态联动（保持不变） ======
export function setAttitude(pitchDeg, rollDeg, yawDeg){
  const t = state.three; if (!t?.robot) return;
  const r = Math.PI/180;
  const rollAdj = (rollDeg - state.attZero.roll);
  const yawAdj  = (yawDeg  - state.attZero.yaw);

  // 绕机器人自身坐标的欧拉：X=俯仰，Z=横滚，Y=航向
  t.robot.rotation.x = pitchDeg*r;
  t.robot.rotation.z = rollAdj*r;
  t.robot.rotation.y = yawAdj*r;

  const out = document.querySelector('#attOut');
  if (out) {
    out.textContent = `pitch: ${pitchDeg.toFixed(1)}°  roll: ${rollAdj.toFixed(1)}°  yaw: ${yawAdj.toFixed(1)}°`;
  }
}
