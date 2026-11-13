import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { TransformControls } from './vendor/TransformControls.js';

// 每次改网页都同时改这里和 web/version.json；刷新后右上角应显示这个版本
const WEB_VERSION = '2026-11-13.1';
const webverEl = document.getElementById('pill-webver');
if (webverEl) webverEl.textContent = `网页 v${WEB_VERSION}`;

// ---------------------------------------------------------------------------
// 场景（B 系：z 向上，单位 mm）
// ---------------------------------------------------------------------------
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0f15);
scene.fog = new THREE.Fog(0x0a0f15, 2600, 5200);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 20000);
camera.up.set(0, 0, 1);
camera.position.set(-720, -900, 900);

const controls = new OrbitControls(camera, canvas);
controls.target.set(170, 0, 260);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 120;
controls.maxDistance = 4200;

scene.add(new THREE.AmbientLight(0x9fb4c8, 0.85));
const d1 = new THREE.DirectionalLight(0xffffff, 1.5);
d1.position.set(600, -500, 1100);
scene.add(d1);
const d2 = new THREE.DirectionalLight(0x88aaff, 0.5);
d2.position.set(-700, 600, 500);
scene.add(d2);

// 网格
function makeGrid() {
  const pos = [], col = [];
  const cMinor = new THREE.Color(0x1b2836);
  const cMajor = new THREE.Color(0x2c4761);
  const x0 = -200, x1 = 600, y0 = -400, y1 = 400, step = 50;
  for (let x = x0; x <= x1; x += step) {
    pos.push(x, y0, 0, x, y1, 0);
    const c = (x % 200 === 0) ? cMajor : cMinor;
    col.push(c.r, c.g, c.b, c.r, c.g, c.b);
  }
  for (let y = y0; y <= y1; y += step) {
    pos.push(x0, y, 0, x1, y, 0);
    const c = (y % 200 === 0) ? cMajor : cMinor;
    col.push(c.r, c.g, c.b, c.r, c.g, c.b);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true }));
}
scene.add(makeGrid());

// 桌面（点击用，不可见）
const plane = new THREE.Mesh(
  new THREE.PlaneGeometry(800, 800),
  new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false, colorWrite: false })
);
plane.position.set(200, 0, 0);
scene.add(plane);

// B 系坐标轴
const axes = new THREE.Group();
axes.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 170, 0xff6b6b, 34, 16));
axes.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 170, 0x6bffb0, 34, 16));
axes.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 170, 0x6bb6ff, 34, 16));
scene.add(axes);

// ---------------------------------------------------------------------------
// 臂的简化几何
// ---------------------------------------------------------------------------
const UP = new THREE.Vector3(0, 1, 0);

class ArmView {
  constructor(color, opacity) {
    this.group = new THREE.Group();
    this.mat = new THREE.MeshStandardMaterial({
      color, metalness: 0.25, roughness: 0.55,
      transparent: opacity < 1, opacity, depthWrite: opacity >= 1,
    });
    this.sphereMat = this.mat;
    this.caps = [];
    this.base = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), this.mat);
    this.group.add(this.base);
    this.probe = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 18), this.mat);
    this.group.add(this.probe);
    this.tip = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 10), this.mat);
    this.group.add(this.tip);
    this.axisLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity: Math.min(1, opacity + .5) })
    );
    this.group.add(this.axisLine);
    scene.add(this.group);
  }
  setBodies(caps, base) {
    while (this.caps.length < caps.length) {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 14), this.mat);
      const s = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 10), this.sphereMat);
      this.group.add(m); this.group.add(s);
      this.caps.push({ m, s });
    }
    this.caps.forEach((c, i) => {
      if (i >= caps.length) { c.m.visible = c.s.visible = false; return; }
      const b = caps[i];
      const p0 = new THREE.Vector3(...b.p0), p1 = new THREE.Vector3(...b.p1);
      const dir = p1.clone().sub(p0);
      const len = Math.max(0.01, dir.length());
      const mid = p0.clone().add(p1).multiplyScalar(0.5);
      c.m.visible = c.s.visible = true;
      c.m.position.copy(mid);
      c.m.quaternion.setFromUnitVectors(UP, dir.clone().normalize());
      c.m.scale.set(b.r, len, b.r);
      c.s.position.copy(mid);
      c.s.scale.set(b.r, b.r, b.r);
    });
    if (base) {
      this.base.position.set(...base.center);
      this.base.scale.set(...base.size);
    }
  }
  setProbe(p) {
    if (!p) { this.probe.visible = this.tip.visible = false; return; }
    this.probe.visible = this.tip.visible = true;
    const p0 = new THREE.Vector3(...p.p0), p1 = new THREE.Vector3(...p.p1);
    const dir = p1.clone().sub(p0);
    const len = Math.max(0.01, dir.length());
    const mid = p0.clone().add(p1).multiplyScalar(0.5);
    this.probe.position.copy(mid);
    this.probe.quaternion.setFromUnitVectors(UP, dir.clone().normalize());
    const gkey = `${p.r0.toFixed(2)}|${p.r1.toFixed(2)}|${len.toFixed(1)}`;
    if (this._probeGKey !== gkey) {
      this.probe.geometry.dispose();
      this.probe.geometry = new THREE.CylinderGeometry(Math.max(p.r1, 0.2), Math.max(p.r0, 0.2), len, 18);
      this._probeGKey = gkey;
    }
    const tipR = p.r1 < 2 ? 0.9 : 2.0;
    this.tip.position.copy(p1);
    this.tip.scale.set(tipR, tipR, tipR);
    this.axisLine.geometry.dispose();
    this.axisLine.geometry = new THREE.BufferGeometry().setFromPoints([p0, p1]);
  }
  setScene(s) {
    if (!s) { this.group.visible = false; return; }
    this.group.visible = true;
    this.setBodies(s.capsules, s.base);
    this.setProbe(s.probe);
  }
}

const actual = new ArmView(0x8fb3d1, 1.0);
const preview = new ArmView(0xffb14e, 0.34);

// ---------------------------------------------------------------------------
// 18.1 搁架占位（PC 已用 build123d 建模；这里只按外形包络画）
// ---------------------------------------------------------------------------
const parkGroup = new THREE.Group();
scene.add(parkGroup);
let _parkKey = '';
const SHELF_MAT = new THREE.MeshStandardMaterial({ color: 0x6f7d8c, metalness: 0.1, roughness: 0.85 });
const GROOVE_MAT = new THREE.MeshStandardMaterial({ color: 0x2b333d, metalness: 0.0, roughness: 1.0 });
const TUBE_MAT = new THREE.MeshStandardMaterial({ color: 0x9aa7b4, metalness: 0.5, roughness: 0.4 });

function buildPark(shelf) {
  if (!shelf || !shelf.center) { parkGroup.visible = false; _parkKey = ''; return; }
  const key = JSON.stringify(shelf);
  if (key === _parkKey) return;
  _parkKey = key;
  parkGroup.visible = true;
  while (parkGroup.children.length) parkGroup.remove(parkGroup.children[0]);
  const bx = shelf.base_x / 2, by = shelf.base_y / 2;
  const tx = shelf.top_x / 2, ty = shelf.top_y / 2;
  const H = shelf.height, tip = shelf.groove_tip_z;
  const gw = Math.min((shelf.groove_width || 36) / 2, by);
  const B = [[-bx, -by, 0], [bx, -by, 0], [bx, by, 0], [-bx, by, 0]];
  const Tm = [[-tx, -ty, H], [tx, -ty, H], [tx, ty, H], [-tx, ty, H]];
  const v = [];
  const quad = (a, b, c, d) => v.push(...a, ...b, ...c, ...a, ...c, ...d);
  for (let i = 0; i < 4; i++) {
    const j = (i + 1) % 4;
    quad(B[i], B[j], Tm[j], Tm[i]);
  }
  // 顶面（V 槽两侧的窄条）
  quad([-tx, -ty, H], [tx, -ty, H], [tx, -gw, H], [-tx, -gw, H]);
  quad([-tx, gw, H], [tx, gw, H], [tx, ty, H], [-tx, ty, H]);
  const gBody = new THREE.BufferGeometry();
  gBody.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  gBody.computeVertexNormals();
  const body = new THREE.Mesh(gBody, SHELF_MAT);
  body.material.side = THREE.DoubleSide;
  parkGroup.add(body);
  // V 槽两面（深色）
  const vg = [];
  const q2 = (a, b, c, d) => vg.push(...a, ...b, ...c, ...a, ...c, ...d);
  q2([-tx, -gw, H], [tx, -gw, H], [tx, 0, tip], [-tx, 0, tip]);
  q2([-tx, gw, H], [tx, gw, H], [tx, 0, tip], [-tx, 0, tip]);
  const gG = new THREE.BufferGeometry();
  gG.setAttribute('position', new THREE.Float32BufferAttribute(vg, 3));
  gG.computeVertexNormals();
  const groove = new THREE.Mesh(gG, GROOVE_MAT);
  groove.material.side = THREE.DoubleSide;
  parkGroup.add(groove);
  // 铝管占位（停机时前臂 Ø35，轴线 136.6；只在停机姿态显示）
  const tube = new THREE.Mesh(new THREE.CylinderGeometry(17.5, 17.5, 100, 18), TUBE_MAT);
  tube.rotation.z = Math.PI / 2;
  tube.position.set(0, 0, shelf.tube_axis_z || (tip + 27.6));
  tube.visible = false;
  parkGroup.add(tube);
  parkGroup.userData = { shelf, tube };
  parkGroup.position.set(shelf.center[0], shelf.center[1], 0);
  parkGroup.rotation.z = (shelf.rotate_deg || 0) * Math.PI / 180;
}

function setParkTube(visible) {
  const u = parkGroup.userData || {};
  if (u.tube) u.tube.visible = !!visible;
}

// 目标标记
const targetGroup = new THREE.Group();
const ring = new THREE.Mesh(
  new THREE.RingGeometry(9, 13, 32),
  new THREE.MeshBasicMaterial({ color: 0xffb14e, side: THREE.DoubleSide, transparent: true, opacity: 0.9 })
);
ring.position.z = 0.5;
const lineMat = new THREE.LineDashedMaterial({ color: 0xffb14e, dashSize: 8, gapSize: 6, transparent: true, opacity: 0.7 });
const vline = new THREE.Line(new THREE.BufferGeometry(), lineMat);
const dot = new THREE.Mesh(new THREE.SphereGeometry(3, 12, 10),
  new THREE.MeshBasicMaterial({ color: 0xffb14e }));
targetGroup.add(ring, dot, vline);
targetGroup.visible = false;
scene.add(targetGroup);

// 20.8：停机实际下降路径（后端给的 TCP 点，橙色虚线）
const descentMat = new THREE.LineDashedMaterial({
  color: 0xffb14e, dashSize: 10, gapSize: 6, transparent: true, opacity: 0.85,
});
const descentLine = new THREE.Line(new THREE.BufferGeometry(), descentMat);
descentLine.visible = false;
scene.add(descentLine);
let _descentKey = '';
function setDescentPath(points) {
  if (!points || points.length < 2) {
    descentLine.visible = false; _descentKey = ''; return;
  }
  const key = JSON.stringify(points);
  if (key === _descentKey) { descentLine.visible = true; return; }
  _descentKey = key;
  const g = new THREE.BufferGeometry().setFromPoints(
    points.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
  descentLine.geometry.dispose();
  descentLine.geometry = g;
  descentLine.computeLineDistances();
  descentLine.visible = true;
}

let _markerKey = '';
function setTargetMarker(t) {
  if (!t) { targetGroup.visible = false; _markerKey = ''; return; }
  targetGroup.visible = true;
  const key = `${t[0].toFixed(1)}|${t[1].toFixed(1)}|${t[2].toFixed(1)}`;
  if (key === _markerKey) return;
  _markerKey = key;
  ring.position.set(t[0], t[1], 0.6);
  dot.position.set(t[0], t[1], t[2]);
  const g = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(t[0], t[1], 0), new THREE.Vector3(t[0], t[1], t[2])
  ]);
  vline.geometry.dispose(); vline.geometry = g; vline.computeLineDistances();
}

// ---------------------------------------------------------------------------
// 3D 拖动抓手（TransformControls）
// ---------------------------------------------------------------------------
const gizmoProxy = new THREE.Object3D();
gizmoProxy.position.set(0, 234, 576.2);
scene.add(gizmoProxy);
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.setMode('translate');
gizmo.setSize(1.35);
gizmo.space = 'world';
gizmo.translationSnap = 1;      // 1 mm 一格；按住 Shift 可微调（three 默认行为）
gizmo.attach(gizmoProxy);
gizmo.visible = false;
gizmo.enabled = false;
scene.add(gizmo);

let gizmoOn = false;
let gizmoDragging = false;
let gizmoHasTarget = false;
let gizmoTimer = null;
let gizmoLastDrag = 0;
let reachTimer = null;
let reachBusy = false;
let reachSeq = 0;
let lastReach = null;      // 最近一个精确 IK 可达的点
const SAFE_GIZMO = [300.0, 0.0, 200.0];

function clampGizmoTarget() {
  if (!document.getElementById('gizmo-limit').checked) return false;
  const p = gizmoProxy.position;
  let x = p.x, y = p.y, z = p.z, changed = false;
  const zMin = 22.0, zMax = 320.0;
  if (z < zMin) { z = zMin; changed = true; }
  if (z > zMax) { z = zMax; changed = true; }
  // 底座禁区：z<200 时 r>=95
  let r = Math.hypot(x, y);
  if (z < 200 && r < 95) {
    if (r < 1e-6) { x = 95; y = 0; } else { x *= 95 / r; y *= 95 / r; }
    changed = true;
  }
  // 肩点球：手腕（TCP+90z）到 (0,116.5,120.5) 不超过 448
  const sx = 0, sy = 116.5, sz = 120.5, dMax = 448;
  let dx = x - sx, dy = y - sy, dz = (z + 90) - sz;
  const d = Math.hypot(dx, dy, dz);
  if (d > dMax) {
    const k = dMax / d;
    x = sx + dx * k; y = sy + dy * k; z = sz + dz * k - 90;
    changed = true;
  }
  // 夹完再保证 z 和底座禁区
  if (z < zMin) { z = zMin; changed = true; }
  if (z > zMax) { z = zMax; changed = true; }
  r = Math.hypot(x, y);
  if (z < 200 && r < 95) {
    if (r < 1e-6) { x = 95; y = 0; } else { x *= 95 / r; y *= 95 / r; }
    changed = true;
  }
  if (changed) p.set(x, y, z);
  return changed;
}
const hudEl = document.getElementById('gizmo-hud');

function updateGizmoHud() {
  if (!hudEl) return;
  if (!gizmoOn) { hudEl.classList.add('hidden'); return; }
  hudEl.classList.remove('hidden');
  document.getElementById('gh-x').textContent = gizmoProxy.position.x.toFixed(1);
  document.getElementById('gh-y').textContent = gizmoProxy.position.y.toFixed(1);
  document.getElementById('gh-z').textContent = gizmoProxy.position.z.toFixed(1);
}
function setHudStatus(text, cls) {
  const el = document.getElementById('gh-status');
  if (el) { el.textContent = text; el.className = cls || ''; }
}
function scheduleReachCheck() {
  updateGizmoHud();
  if (!gizmoOn) return;
  setHudStatus('检查中…', 'check');
  clearTimeout(reachTimer);
  const seq = ++reachSeq;
  reachTimer = setTimeout(() => checkReach(seq), 180);
}
async function checkReach(seq) {
  if (!gizmoOn || seq !== reachSeq || reachBusy) return;
  reachBusy = true;
  const target = gizmoTargetArray();
  try {
    const r = await apiPost('/api/reach_tcp', {
      target,
      free_yaw: document.getElementById('gizmo-free-yaw').checked,
    });
    if (seq !== reachSeq || !gizmoOn) return;
    if (r.reachable) {
      lastReach = [target[0], target[1], target[2]];
      setHudStatus(`可达 ✓  残差 ${r.pos_err_mm} mm / 轴 ${r.axis_err_deg}°`, 'ok');
    } else {
      const why = r.reason || `残差 ${r.pos_err_mm} mm / 轴 ${r.axis_err_deg}°`;
      setHudStatus('不可达 ✗  ' + why, 'bad');
    }
  } catch (e) {
    if (seq === reachSeq) setHudStatus('检查失败：' + e.message, 'bad');
  } finally {
    reachBusy = false;
    if (seq !== reachSeq && gizmoOn) scheduleReachCheck();
  }
}

function gizmoTargetArray() {
  return [gizmoProxy.position.x, gizmoProxy.position.y, gizmoProxy.position.z];
}

function setGizmoFromTcp(useSafe = false) {
  let t = null;
  if (!useSafe && STATE && STATE.preview && STATE.preview.target) {
    t = STATE.preview.target;
  } else if (!useSafe && STATE && STATE.tcp) {
    t = STATE.tcp;
  } else {
    t = SAFE_GIZMO;
  }
  gizmoProxy.position.set(t[0], t[1], t[2]);
  clampGizmoTarget();
  lastReach = null;
  // 固定住抓手：否则下一次状态推送会把它拉回实际 TCP
  gizmoHasTarget = true;
}

async function gizmoSend(preview, allowSnap = true) {
  if (!gizmoOn || !TOKEN) return;
  const target = gizmoTargetArray();
  const free = document.getElementById('gizmo-free-yaw').checked;
  setHudStatus('规划中…', 'check');
  try {
    const r = await apiPost('/api/move_tcp', {
      target,
      axis: [0, 0, -1],
      yaw: Number(document.getElementById('yaw').value),
      free_yaw: free,
      preview,
    });
    if (r.preview) {
      setTargetMarker(r.preview.target || target);
      if (STATE) STATE.preview = r.preview;
    }
    setHudStatus('可达 · 已预演', 'ok');
    if (!preview) log(`3D 目标 (${target.map((v) => v.toFixed(1)).join(', ')}) 已下发`, 'g');
  } catch (e) {
    if (allowSnap && document.getElementById('gizmo-limit').checked && lastReach) {
      gizmoProxy.position.set(lastReach[0], lastReach[1], lastReach[2]);
      updateGizmoHud();
      log(`目标不可达，已退回上一个可达点 (${lastReach.map((v) => v.toFixed(1)).join(', ')})`, 'w');
      return gizmoSend(preview, false);
    }
    setHudStatus('不可达 · ' + e.message, 'bad');
    log('3D 目标不可达：' + e.message
        + `；目标 (${target.map((v) => v.toFixed(1)).join(', ')})。`
        + '可达大致：z 20…480、水平 ≤455、z<200 时 r≥90（底座区），'
        + '往可达区拖或调 z。', 'b');
  }
}

function scheduleGizmoPreview() {
  gizmoHasTarget = true;
  clearTimeout(gizmoTimer);
  gizmoTimer = setTimeout(() => { if (!gizmoDragging) gizmoSend(true); }, 300);
}

gizmo.addEventListener('dragging-changed', (e) => {
  gizmoDragging = e.value;
  if (e.value) gizmoLastDrag = performance.now();
  controls.enabled = !e.value;
  if (!e.value) {
    setTimeout(() => { gizmoDragging = false; }, 120);
    // 松手：预演模式只画；直接执行模式立即走
    const preview = EXEC_MODE !== 'direct';
    gizmoSend(preview).then(() => {
      if (!preview) log('3D 拖动 → 直接执行', 'g');
      else log('3D 拖动 → 预演；点“执行”下发', 'g');
    });
  }
});

gizmo.addEventListener('objectChange', () => {
  if (!gizmoOn) return;
  if (clampGizmoTarget()) setHudStatus('已夹到可达边界', 'check');
  scheduleReachCheck();
  scheduleGizmoPreview();
});

const gizmoModeEl = document.getElementById('gizmo-mode');
gizmoModeEl.addEventListener('change', () => {
  gizmoOn = gizmoModeEl.checked;
  gizmo.visible = gizmoOn;
  gizmo.enabled = gizmoOn;
  if (gizmoOn) {
    gizmo.attach(gizmoProxy);
    setGizmoFromTcp(true);
    scheduleReachCheck();
    log('3D 拖动已开：拖红/绿/蓝箭头改 x/y/z（1 mm 一格）', 'g');
  } else {
    gizmo.detach();
    updateGizmoHud();
    log('3D 拖动已关', '');
  }
});
document.getElementById('gizmo-reset').addEventListener('click', () => {
  setGizmoFromTcp(false);
  scheduleReachCheck();
  log('抓手已回到当前 TCP（若当前姿态不适合工具向下，会显示不可达）', '');
});
for (const id of ['gizmo-free-yaw', 'gizmo-limit']) {
  document.getElementById(id).addEventListener('change', () => {
    lastReach = null;
    if (gizmoOn) { clampGizmoTarget(); scheduleReachCheck(); }
  });
}
document.getElementById('gizmo-safe').addEventListener('click', () => {
  setGizmoFromTcp(true);
  scheduleReachCheck();
  log('抓手已放到安全点 (300, 0, 200)', 'g');
});

// ---------------------------------------------------------------------------
// 交互 / 客户端
// ---------------------------------------------------------------------------
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let down = null;

// 控制权 / 配置
let CONFIG = null;
let CONFIG_VERSION = '';
let TOKEN = '';
let EXEC_MODE = 'preview';
let STATE = null;
let READONLY = true;
let CLIENT_ID = sessionStorage.getItem('arm-client-id') || '';
if (!CLIENT_ID) {
  CLIENT_ID = 'web-' + Math.random().toString(16).slice(2, 10);
  sessionStorage.setItem('arm-client-id', CLIENT_ID);
}
let sse = null;
let sseOk = false;
let pollTimer = null;
let acquireTimer = null;
let ACQUIRING = false;

async function getJSON(url) {
  const res = await fetch(url, { cache: 'no-store' });
  const j = await res.json().catch(() => ({}));
  if (!res.ok || j.ok === false) throw new Error(j.error || `HTTP ${res.status}`);
  return j;
}

async function postRaw(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
  const j = await res.json().catch(() => ({}));
  if (!res.ok || j.ok === false) throw new Error(j.error || `HTTP ${res.status}`);
  return j;
}

async function apiPost(url, body = {}) {
  if (!TOKEN) throw new Error('没有控制权（只读）');
  return postRaw(url, { ...body, token: TOKEN, config_version: CONFIG_VERSION });
}

// ---------------------------------------------------------------------------
// 初始化：配置版本 → 控制权 → SSE / 心跳
// ---------------------------------------------------------------------------
async function init() {
  try {
    CONFIG = await getJSON('/api/config');
    CONFIG_VERSION = CONFIG.config_version;
    // 直接读静态 version.json，避免执行器缓存导致的“服务器版本”误报
    try {
      const ver = await getJSON('/version.json');
      if (ver.version && ver.version !== WEB_VERSION) {
        if (webverEl) {
          webverEl.textContent = `网页 v${WEB_VERSION} → 最新 ${ver.version}，请刷新`;
          webverEl.classList.add('bad');
          webverEl.title = '浏览器里是旧版网页，按 Cmd+Shift+R 强制刷新';
        }
        log(`网页 ${WEB_VERSION} ≠ 服务器静态 ${ver.version}，请 Cmd+Shift+R`, 'b');
      }
    } catch (_) { /* 没有 version.json 就算了 */ }
    EXEC_MODE = CONFIG.exec_mode || 'preview';
    if (CONFIG.park && CONFIG.park.shelf) buildPark(CONFIG.park.shelf);
    log(`执行器 config_version = ${CONFIG_VERSION}`, 'g');
  } catch (e) {
    log('读执行器配置失败：' + e.message, 'b');
    setTimeout(init, 2000);
    return;
  }
  await tryAcquire();
  startEvents();
  setInterval(heartbeat, 200);
  updateModeUI();
}

async function tryAcquire() {
  if (ACQUIRING) return;
  ACQUIRING = true;
  try {
    const r = await postRaw('/api/acquire', {
      client_id: CLIENT_ID, config_version: CONFIG_VERSION,
      ...(TOKEN ? { token: TOKEN } : {}),
    });
    TOKEN = r.token;
    READONLY = false;
    EXEC_MODE = r.exec_mode || EXEC_MODE;
    log('取得控制权', 'g');
    updateModeUI();
  } catch (e) {
    READONLY = true;
    log('只读：' + e.message, 'w');
    if (!acquireTimer) {
      acquireTimer = setInterval(async () => {
        if (!READONLY) return;
        try {
          const r = await postRaw('/api/acquire', {
            client_id: CLIENT_ID, config_version: CONFIG_VERSION,
            ...(TOKEN ? { token: TOKEN } : {}),
          });
          TOKEN = r.token; READONLY = false;
          clearInterval(acquireTimer); acquireTimer = null;
          log('拿到控制权', 'g');
          updateModeUI();
        } catch (_) { /* 继续等 */ }
      }, 2000);
    }
  }
}

function startEvents() {
  if (sse) sse.close();
  sse = new EventSource('/api/events?client_id=' + encodeURIComponent(CLIENT_ID));
  sse.onopen = () => { sseOk = true; log('状态推送已连接（SSE ≥10 Hz）', 'g'); };
  sse.onmessage = (ev) => {
    try { applyState(JSON.parse(ev.data)); } catch (e) { /* ignore */ }
  };
  sse.onerror = () => {
    sseOk = false;
    if (!pollTimer) {
      pollTimer = setInterval(pollState, 300);
    }
  };
}

async function pollState() {
  try {
    const s = await getJSON('/api/state?client_id=' + encodeURIComponent(CLIENT_ID));
    applyState(s);
  } catch (_) { /* ignore */ }
}

async function heartbeat() {
  if (!TOKEN) return;
  try {
    await postRaw('/api/heartbeat', { token: TOKEN });
  } catch (e) {
    TOKEN = '';
    READONLY = true;
    updateModeUI();
    log('心跳失败，控制权已丢：' + e.message, 'w');
    tryAcquire();
  }
}

function syncVersion(s) {
  if (s && s.config_version && s.config_version !== CONFIG_VERSION) {
    const old = CONFIG_VERSION;
    CONFIG_VERSION = s.config_version;
    log('配置版本已更新（校准时会改配置），重新拿控制权', 'w');
    if (TOKEN) {
      postRaw('/api/acquire', { client_id: CLIENT_ID, config_version: CONFIG_VERSION,
        ...(TOKEN ? { token: TOKEN } : {}) })
        .then((r) => { TOKEN = r.token; })
        .catch((e) => log('重新拿控制权失败：' + e.message, 'w'));
    }
  }
}

let lastUiAt = 0;
function applyState(s) {
  syncVersion(s);
  if (CALIB_OPEN) updateCalib(s.calib);
  if (MEASURE_OPEN) updateMeasureUI(s);
  STATE = s;
  const nowUi = performance.now();
  if (!gizmoDragging && nowUi - lastUiAt > 80) {
    lastUiAt = nowUi;
    updateUI(s);
  }
  // 拖 3D 抓手时机械臂不会动，跳过臂的几何更新，减少拖动时的 CPU 占用
  if (!gizmoDragging) {
    if (s.scene) actual.setScene(s.scene); else actual.group.visible = false;
    if (s.preview && s.preview.scene) {
      preview.setScene(s.preview.scene);
      setTargetMarker(s.preview.target || s.preview.tcp);
      setDescentPath(s.preview.park && s.preview.park.descent_path);
    } else {
      preview.group.visible = false;
      setDescentPath(null);
    }
    if (!s.preview) setTargetMarker(null);
  }
  if (!s.connected) {
    if (EXEC_MODE !== 'preview') { EXEC_MODE = 'preview'; updateModeUI(); }
  } else if (s.exec_mode && s.exec_mode !== EXEC_MODE) {
    EXEC_MODE = s.exec_mode;
    updateModeUI();
  }
  if (gizmoOn) {
    if (!gizmoDragging && !gizmoHasTarget && s.tcp) {
      gizmoProxy.position.set(s.tcp[0], s.tcp[1], s.tcp[2]);
    }
    gizmo.enabled = !(s.motion && s.motion.busy);
  }
  // 没有控制方或控制方心跳已老化：尝试拿回来
  const c = s.control || {};
  if (!TOKEN && (c.owner === null || (c.age_s !== null && c.age_s > c.stale_s))) {
    const last = window.__lastAcquireTry || 0;
    if (!ACQUIRING && Date.now() - last > 2000) {
      window.__lastAcquireTry = Date.now();
      tryAcquire();
    }
  }
}

// ---------------------------------------------------------------------------
// 点击
// ---------------------------------------------------------------------------
canvas.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY }; });
canvas.addEventListener('pointerup', (e) => {
  if (!down) return;
  const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
  down = null;
  if (moved > 6 || e.button !== 0) return;
  const r = canvas.getBoundingClientRect();
  pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
  pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObject(plane, false)[0];
  if (!hit) return;
  const { x, y } = hit.point;
  if (x < -200 || x > 600 || y < -400 || y > 400) return;
  clickAt(Math.round(x), Math.round(y));
});

async function clickAt(x, y) {
  if (gizmoOn) {
    if (performance.now() - gizmoLastDrag > 400) {
      log('3D 拖动模式下请拖抓手；关掉 3D 再点网格', 'w');
    }
    return;
  }
  const yaw = Number(document.getElementById('yaw').value);
  try {
    const r = await apiPost('/api/click', { x, y, z: 20, yaw });
    if (r.preview) {
      preview.setScene(r.preview.scene);
      setTargetMarker(r.preview.target);
    }
    if (r.started) log(`点击 (${x}, ${y}) → 直接执行`, 'g');
    else log(`点击 (${x}, ${y}) → 预演（${r.segments.length} 段）`, 'g');
  } catch (e) {
    if (String(e.message).includes('正在运动')) log('正在运动，点击已忽略', 'w');
    else log('点击失败：' + e.message, 'b');
  }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const logEl = $('log');

function log(text, cls = '') {
  const div = document.createElement('div');
  const t = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  div.innerHTML = `<span class="t">${t}</span> `;
  const s = document.createElement('span');
  if (cls) s.className = cls;
  s.textContent = text;
  div.appendChild(s);
  logEl.prepend(div);
  while (logEl.childElementCount > 90) logEl.lastElementChild.remove();
}

function kv(lines) {
  return lines.map(([k, v, cls]) =>
    `<div class="line"><span class="muted">${k}</span><span class="${cls || ''}">${v}</span></div>`).join('');
}

function fmt(v, n = 1) {
  return (v === null || v === undefined) ? '—' : Number(v).toFixed(n);
}

function nearLabel(near) {
  if (!near || !near.length) return ['离限位<10°', '无', 'ok'];
  return ['离限位<10°', near.map((x) => `${x.joint} ${x.margin_deg}°`).join(' '), 'warn'];
}

function isOwner(s) {
  return !!(s && s.control && s.control.owner === CLIENT_ID && TOKEN);
}

function updateModeUI() {
  $('mode-preview').classList.toggle('active', EXEC_MODE === 'preview');
  $('mode-direct').classList.toggle('active', EXEC_MODE === 'direct');
  $('mode-hint').textContent = EXEC_MODE === 'direct'
    ? '点完立即执行；自动检查一项不少，不过就写明原因。'
    : '预演：只算不发，点“执行”也只画预览。';
}

let lastPhase = '';
let holdWarnAt = null;

function renderSafeCheck(s) {
  const el = $('safe-check');
  if (!el) return;
  const sc = s.safe_check;
  if (!sc || sc.ok === null || sc.ok === undefined) {
    el.className = 'safe-check unknown';
    el.textContent = (sc && sc.text) || '安全限位：未连接';
    return;
  }
  const rows = sc.rows || [];
  const bad = rows.filter((r) => !r.ok);
  el.className = 'safe-check ' + (bad.length ? 'bad' : 'ok');
  const head = bad.length
    ? '安全限位不合格：禁止使能/运动，先断电手动移回'
    : '六关节安全限位正常';
  const t = rows.map((r) =>
    `<td class="${r.ok ? '' : 'bad'}">${r.joint} ${r.q.toFixed(2)}°<br>` +
    `<span style="opacity:.65">${r.lo}..${r.hi}</span></td>`).join('');
  el.innerHTML = `<b>${head}</b><table><tr>${t}</tr></table>`;
}

function updateUI(s) {
  const owner = isOwner(s);
  $('conn-dot').className = 'dot' + (s.connected ? (s.startup_done ? ' on' : ' warn') : '');
  $('pill-mode').textContent = s.mode === 'mock' ? '模拟' : (s.mode === 'hw' ? '真机' : '未连接');
  $('pill-mode').classList.toggle('on', !!s.connected);
  const c = s.control || {};
  const pill = $('pill-control');
  if (owner) { pill.textContent = '控制权 你'; pill.className = 'pill on'; }
  else if (c.owner) { pill.textContent = `控制权 ${c.owner}`; pill.className = 'pill'; }
  else { pill.textContent = '控制权 无'; pill.className = 'pill'; }
  $('hdr-msg').textContent = (s.msg || '') +
    (s.config_stale ? ' · 配置已变，请重新连接' : '');
  $('foot-right').textContent = s.recorder ? `CSV: ${s.recorder}` : '';
  const tcpSel = $('tcp-preset');
  if (tcpSel && s.tcp_preset && tcpSel.value !== s.tcp_preset) tcpSel.value = s.tcp_preset;
  renderSafeCheck(s);
  const unsafe = !!(s.safe_check && s.safe_check.ok === false);

  const busy = s.motion && s.motion.busy;
  const dragActive = !!(s.drag && s.drag.active);
  const stale = !!s.config_stale;
  $('btn-enable').disabled = !s.connected || !owner || s.startup_done || busy || dragActive || stale;
  $('btn-reenable').disabled = !s.connected || !owner || busy || dragActive || unsafe || stale;
  $('btn-clear-errors').disabled = !s.connected || !owner || busy || dragActive;
  $('btn-park').disabled = !s.connected || !owner || busy || dragActive || unsafe ||
    stale || s.end_effector !== 'flange';
  $('btn-park-teach').disabled = !s.connected || !owner || busy || dragActive || unsafe ||
    stale || !s.startup_done || s.end_effector !== 'flange';
  const pblk = (s.park && s.park.block) || null;
  const blkEl = $('park-block');
  const ackBtn = $('btn-park-ack');
  if (blkEl) {
    if (pblk) {
      const d = pblk.diffs ? Object.entries(pblk.diffs).map(([k, v]) => `${k} ${v}°`).join(' ') : '';
      blkEl.style.display = 'block';
      blkEl.textContent = `启停比对不过（和 parked_q 差 ${pblk.max_diff_deg}°：${d}）——已拒绝一切运动。看实物确认后点下面的按钮。`;
    } else {
      blkEl.style.display = 'none';
    }
  }
  if (ackBtn) {
    ackBtn.style.display = pblk ? 'block' : 'none';
    ackBtn.disabled = !owner;
  }
  $('btn-recover').disabled = !s.connected || !owner || busy || dragActive || unsafe || stale;
  $('btn-unpark').disabled = !s.connected || !owner || busy || dragActive || unsafe || stale;
  $('btn-support-park').disabled = !s.connected || !owner || busy || dragActive || unsafe || stale;
  // 紧急失能故意不检查 unsafe；只要连接/控制权/不在运动/不在拖动
  $('btn-disable-all').disabled = !s.connected || !owner || busy || dragActive;
  $('btn-ee').disabled = !owner || busy || dragActive;
  $('btn-connect').disabled = busy || dragActive;
  $('btn-write-limits').disabled = !s.connected || !owner || busy || dragActive || unsafe;
  const canPreview = EXEC_MODE === 'preview';
  const offlineNote = document.getElementById('offline-note');
  if (offlineNote) offlineNote.style.display = s.connected ? 'none' : 'block';
  const roNote = document.getElementById('readonly-note');
  if (roNote) {
    const other = c.owner;
    roNote.style.display = (!owner && other) ? 'block' : 'none';
    if (other) roNote.textContent = `只读：控制权在 ${other}（另一个页面还在心跳）。关掉其他机器人页面，或点“重新取得控制权”。`;
  }
  $('btn-acquire').disabled = !s.connected || owner || busy || dragActive;
  $('btn-exec').disabled = !s.has_plan || busy || !owner || unsafe || stale;
  $('btn-stop').disabled = !s.connected && !busy;
  const ps = document.getElementById('plan-state');
  if (ps) {
    if (!owner) ps.textContent = '只读：另一个页面持有控制权，不能生成/执行计划。';
    else if (s.has_plan) ps.textContent = s.connected ? '计划已生成，可以执行（按当前模式）。' : '离线预演计划已生成，可以点“执行”看预演。';
    else ps.textContent = '还没有计划：拖到可达点或点网格。';
    ps.style.color = s.has_plan && owner ? '#6bffb0' : '#8a9bb0';
  }
  $('btn-home').disabled = busy || !owner || unsafe || stale;
  $('btn-home-plan').disabled = !s.connected || !owner || busy || unsafe || !s.startup_done || stale;
  $('mode-preview').disabled = !owner;
  $('mode-direct').disabled = !owner || !s.connected;
  document.querySelectorAll('#jog button').forEach((b) => { b.disabled = !owner || unsafe; });

  const tb = document.querySelector('#joints tbody');
  const nearSet = new Set((s.near_limit || []).map((x) => x.joint));
  tb.innerHTML = (s.joints || []).map((j) => {
    const cls = j.error ? 'err' : (nearSet.has(j.name) ? 'near' : '');
    const state = j.enabled === false ? '失能' : (j.enabled ? (j.error ? '报警' : '使能') : '—');
    return `<tr class="${cls}"><td>${j.name}</td><td>${fmt(j.out_deg, 2)}</td>` +
      `<td>${fmt(j.target_deg, 2)}</td><td>${fmt(j.current_a, 2)}</td>` +
      `<td>${fmt(j.temp_c, 0)}</td><td>${state}</td></tr>`;
  }).join('');

  if (s.preview) {
    const p = s.preview;
    const ik = p.ik || ((p.segments && p.segments[p.segments.length - 1]) || {}).ik || {};
    const q = p.q ? p.q.map((v) => v.toFixed(1)).join(' ') : '—';
    $('target-info').innerHTML = kv([
      ['目标', p.target ? `(${p.target[0]}, ${p.target[1]}, ${p.target[2]})` : '—'],
      ['TCP 预览', p.tcp ? `(${fmt(p.tcp[0])}, ${fmt(p.tcp[1])}, ${fmt(p.tcp[2])})` : '—'],
      ['q (°)', q],
      ['残差 mm', ik.pos_err_mm !== undefined ? fmt(ik.pos_err_mm, 3) : '—',
        (ik.pos_err_mm !== undefined && ik.pos_err_mm <= 0.2) ? 'ok' : 'bad'],
      ['轴误差 °', ik.axis_err_deg !== undefined ? fmt(ik.axis_err_deg, 3) : '—'],
      ['偏航 ψ', p.free_yaw ? '自由' : `${fmt(p.yaw, 0)}°`],
    ]);
    $('fence-info').innerHTML = kv([
      ['围栏', p.fence && p.fence.ok ? '通过' : '不通过', p.fence && p.fence.ok ? 'ok' : 'bad'],
      ['原因', (p.fence && p.fence.reasons && p.fence.reasons.length) ? p.fence.reasons.join('；') : '—'],
      ['运动件离地 mm', fmt(p.fence && p.fence.min_ground_mm, 1)],
      ['TCP 高度 mm', fmt(p.fence && p.fence.tcp_z_mm, 1)],
      ['最大水平 r mm', fmt(p.fence && p.fence.max_reach_mm, 1)],
      nearLabel(p.near_limit),
    ]);
  } else if (s.fence) {
    $('fence-info').innerHTML = kv([
      ['围栏（当前姿态）', s.fence.ok ? '通过' : '不通过', s.fence.ok ? 'ok' : 'bad'],
      ['运动件离地 mm', fmt(s.fence.min_ground_mm, 1)],
      ['TCP 高度 mm', fmt(s.fence.tcp_z_mm, 1)],
      nearLabel(s.near_limit),
    ]);
  }

  // 18.2 末端 / 载荷 / 容量
  const eeSel = $('ee');
  if (eeSel && s.end_effector && eeSel.value !== s.end_effector) eeSel.value = s.end_effector;
  const caps = s.capacities || {};
  const lused = s.limits_used || {};
  const capTxt = ['J2', 'J3', 'J4', 'J5']
    .map((k) => `${k} ${caps[k] === null || caps[k] === undefined ? '—' : caps[k].toFixed(2)}`)
    .join('  ');
  const srcTxt = ['J2', 'J3', 'J4', 'J5'].map((k) => {
    if (k === 'J3') return 'J3 固定6.50';
    const x = lused[k] || {};
    return `${k} ${x.a === null || x.a === undefined ? '—' : Number(x.a).toFixed(2) + 'A'} ${x.source || '—'}`;
  }).join('  ');
  if ($('ee-info')) $('ee-info').innerHTML = kv([
    ['末端', s.end_effector || '—'],
    ['载荷 kg', fmt(s.payload_kg, 2)],
    ['容量 N·m', capTxt],
    ['限流来源', srcTxt],
  ]);
  const hold = s.hold || {};
  if ($('torque-info')) $('torque-info').innerHTML = kv([
    ['保持力矩', hold.ratio === null || hold.ratio === undefined ? '—' : `${(hold.ratio * 100).toFixed(0)}%`,
      hold.ratio > 0.7 ? 'bad' : (hold.ratio > 0.5 ? 'warn' : 'ok')],
    ['>70% 已持续', hold.since_s === null || hold.since_s === undefined ? '—' : `${hold.since_s} s`],
    ['提醒', hold.warn || '—', hold.warn ? 'bad' : ''],
  ]);
  const park = s.park || {};
  const pstep = park.step || {};
  const pchk = park.check || {};
  if ($('park-info')) $('park-info').innerHTML = kv([
    ['停机阶段', pstep.phase || '—',
      ['catch_failed', 'aborted', 'error', 'disable_aborted'].includes(pstep.phase) ? 'bad' : ''],
    ['I₀ J2/J3', pstep.I0 ? `${fmt(pstep.I0.J2, 2)} / ${fmt(pstep.I0.J3, 2)} A` : '—'],
    ['I₁ J2/J3', pstep.I1 ? `${fmt(pstep.I1.J2, 2)} / ${fmt(pstep.I1.J3, 2)} A` : '—'],
    ['下降', pstep.z === null || pstep.z === undefined ? '—'
      : `z ${fmt(pstep.z, 1)} mm · 第 ${pstep.step || 0} 步`, pstep.phase === 'catch_failed' ? 'bad' : ''],
    ['当前 J2', pstep.I2 === null || pstep.I2 === undefined ? '—' : `${fmt(pstep.I2, 2)} A`],
    ['搁架 side', park.park_side],
    ['实际下降', (park.descent && park.descent.note) || '—'],
    ['教过 P', park.park_taught ? `是 · ${park.park_taught_at || ''}` : '否',
      park.park_taught ? 'ok' : ''],
    ['parked_q', pchk.has_parked ? (pchk.ok ? '一致（在搁架上）' : `差 ${pchk.max_diff_deg}°`)
      : '无', pchk.has_parked && pchk.ok === false ? 'bad' : ''],
  ]);
  setParkTube(pstep.phase === 'at_park' || pstep.phase === 'parked' ||
    pstep.phase === 'disabled_J2' || pstep.phase === 'disabled_J1');

  const m = s.motion || {};
  const phaseTxt = {
    idle: '空闲', preview: '预演中', preview_done: '预演完成', running: '运行',
    moving: '运动中', arrived: '到位', done: '完成', aborted: '已中止',
    stopping: '停止中', error: '错误',
  }[m.phase] || m.phase || '—';
  $('motion-hint').textContent = `状态：${phaseTxt}${m.label ? ' · ' + m.label : ''}` +
    (m.reason ? ` · ${m.reason}` : '') + (m.T ? ` · T=${m.T}s` : '') +
    '　（空格 = 停）';
  if (m.phase && m.phase !== lastPhase) {
    lastPhase = m.phase;
    if (['done', 'arrived', 'aborted', 'error'].includes(m.phase)) {
      log(`运动 ${phaseTxt}${m.reason ? '：' + m.reason : ''}`,
        m.phase === 'aborted' || m.phase === 'error' ? 'b' : 'g');
    }
  }
  // 16.4 第 7 条：保持超过 60 s 提醒
  if (m.phase === 'aborted' && m.finished) {
    const age = Date.now() / 1000 - Number(m.finished);
    if (age > 60 && holdWarnAt !== m.finished) {
      holdWarnAt = m.finished;
      log('保持已超过 60 s：请恢复运动或执行停机（18.3）', 'b');
    }
  } else {
    holdWarnAt = null;
  }
}

// ---------------------------------------------------------------------------
// 按钮
// ---------------------------------------------------------------------------
$('btn-acquire').addEventListener('click', async () => {
  await tryAcquire();
  if (!READONLY) log('已取得控制权', 'g');
});

$('btn-connect').addEventListener('click', async () => {
  const mode = $('mode').value;
  try {
    $('btn-connect').disabled = true;
    log(`连接 ${mode} ...`);
    const r = await apiPost('/api/connect', { mode, ip: $('ip').value.trim() });
    if (r.report && r.report.mismatch && r.report.mismatch.length) {
      log('ID↔SN 不一致：' + r.report.mismatch.join('；'), 'b');
    } else {
      log(r.msg || '已连接', 'g');
    }
    $('connect-hint').textContent = r.msg || '';
  } catch (e) {
    log('连接失败：' + e.message, 'b');
  } finally {
    $('btn-connect').disabled = false;
  }
});

$('btn-clear-errors').addEventListener('click', async () => {
  if (!confirm('先确认六颗机械都没有被顶住/卡住、臂已支撑。\n会逐颗短暂零扭矩使能来读/清报警锁存，不出力、不运动。继续？')) return;
  try {
    const r = await apiPost('/api/clear_errors', {});
    (r.rows || []).forEach((x) => log(
      `${x.name}: ${x.before_text || '—'} → ${x.after_text || '读不到'} ${x.cleared ? '（已清）' : '（清失败）'}` +
      (x.pos_deg === null || x.pos_deg === undefined ? '' :
        `，位置 ${x.pos_deg}°${x.out_of_limit ? '（超出安全限位，先手动移回！）' : ''}`),
      x.after === '0x0000' || x.after === null ? 'g' : 'b'));
    log(r.msg || '清除完成', r.ok ? 'g' : 'b');
  } catch (e) { log('清除报警失败：' + e.message, 'b'); alert('清除报警失败：' + e.message); }
});

async function enableArm() {
  if (!confirm('臂已支撑好、人站在直流空开旁？\n将 J6→J1 逐个零扭矩使能（电流模式 0 A），回读限流/限位/母线电压，做成链检查，然后切 S 曲线保持。')) return;
  try {
    log('使能中（J6→J1）...');
    const r = await apiPost('/api/enable', { confirmed_supported: true });
    (r.report.zero_torque || []).forEach((z) => log(
      `零扭矩使能 ${z.name}: 模式 ${z.mode}，电流 ${z.current_a} A`, 'g'));
    (r.report.enabled || []).forEach((e) => log(
      `${e.name}: 限流 ${e.current_limit_a} A，限位 ${JSON.stringify(e.pos_limits_deg)}°，` +
      `母线 ${e.voltage_v} V，梯形 ${e.profile_deg_s}°/s，模式 ${e.mode}`, ''));
    if (r.report.chain) {
      const ch = r.report.chain;
      log('成链检查：最高 ' + ch.max_v + ' V，' +
        Object.entries(ch.voltages).map(([k, v]) => `${k} ${v}V`).join(' '),
        'g');
    }
    (r.report.warnings || []).forEach((w) => log(w, 'w'));
    if (r.report.elapsed_s) log(`启动耗时 ${r.report.elapsed_s} s`, 'g');
    log('启动完成：零扭矩读核 + 成链检查通过，已切 S 曲线保持', 'g');
  } catch (e) {
    log('使能失败：' + e.message, 'b');
    alert('使能失败（执行器已按启动序列停下）：\n' + e.message);
  }
}
$('btn-enable').addEventListener('click', enableArm);
$('btn-reenable').addEventListener('click', enableArm);

$('btn-home-plan').addEventListener('click', async () => {
  try {
    const r = await apiPost('/api/home', { preview: true });
    const segs = r.segments || (r.preview && r.preview.segments) || [];
    const names = segs.map((x) => x.label).filter(Boolean);
    log(`回零计划已生成：${names.length ? names.join(' → ') : '(空)'}；` +
        '确认后切直接执行点“执行”。', 'w');
    return r;
  } catch (e) { log('生成回零计划失败：' + e.message, 'b'); alert('生成回零计划失败：' + e.message); }
});

$('btn-exec').addEventListener('click', async () => {
  const preview = EXEC_MODE === 'preview';
  if (!preview && !confirm('执行当前预演计划？\n确认刚预演过、工作区清空、臂已支撑、手在直流空开旁？')) return;
  try {
    log(preview ? '执行按钮：预演（不发写命令）' : '执行！', preview ? '' : 'w');
    await apiPost('/api/execute', { preview });
  } catch (e) {
    log('执行失败：' + e.message, 'b');
  }
});

$('btn-stop').addEventListener('click', doStop);

function doStop() {
  const drag = STATE && STATE.drag && STATE.drag.active;
  const p = drag ? apiPost('/api/drag/stop', {}) : postRaw('/api/stop', {});
  p.then((r) => log(
    drag ? (r && r.warning ? r.warning : '已退出拖动，切回位置保持') : '已按停（原地保持）',
    r && r.warning ? 'b' : 'w'))
    .catch((e) => log('停失败：' + e.message, 'b'));
}

window.addEventListener('keydown', (e) => {
  if (e.code !== 'Space') return;
  const t = document.activeElement && document.activeElement.tagName;
  if (['INPUT', 'TEXTAREA', 'SELECT', 'BUTTON'].includes(t)) return;
  e.preventDefault();
  doStop();
});

$('btn-home').addEventListener('click', async () => {
  const preview = EXEC_MODE === 'preview';
  try {
    if (!preview) {
      const p = STATE && STATE.preview;
      const hasHome = !!(STATE && STATE.has_plan && p && (p.segments || [])
        .some((s) => s.label === '回零位'));
      if (!hasHome) {
        await apiPost('/api/home', { preview: true });
        log('已生成回零预演：先检查轨迹/围栏；60 秒内再点一次“回零位”才会执行。', 'w');
        return;
      }
      if (!confirm('确认这是刚预演过的回零轨迹？\n工作区清空、J3 没有顶住限位、臂已支撑、手在直流空开旁？')) return;
    }
    if (preview) {
      log('回零位（预演）：只画轨迹，电机不动。要执行请切“直接执行”，再点两次回零位。', 'w');
    } else {
      log('回零位（执行）', 'w');
    }
    await apiPost('/api/home', { preview });
  } catch (e) { log('回零失败：' + e.message, 'b'); }
});

// ---------------------------------------------------------------------------
// 18.2 末端 / 载荷
// ---------------------------------------------------------------------------
$('btn-ee').addEventListener('click', async () => {
  const ee = $('ee').value;
  const payload = Number($('payload').value) || 0;
  if (ee === 'gripper' && !confirm('确认夹爪 v2 已经装回、线走好、开合核对过？\n带爪之前 J2 必须已经是 6 A（16.3 第 2 条）。')) return;
  try {
    const r = await apiPost('/api/end_effector', { end_effector: ee, payload_kg: payload });
    log(`末端 → ${ee}，载荷 ${payload} kg；容量 ${JSON.stringify(r.capacities)}`, 'g');
    if (r.warn) log(r.warn, 'w');
  } catch (e) { log('改末端配置失败：' + e.message, 'b'); }
});

// ---------------------------------------------------------------------------
// 18.3 停机 / 从停机启动 / 扶持停机
// ---------------------------------------------------------------------------
function parkPrereq() {
  return {
    gripper_empty: $('pk-empty').checked,
    workspace_clear: $('pk-clear').checked,
    shelf_ready: $('pk-shelf').checked,
  };
}

$('btn-park').addEventListener('click', async () => {
  const prereq = parkPrereq();
  if (!prereq.workspace_clear || !prereq.shelf_ready) {
    alert('停机前提没勾选：工作区已清空、搁架已放好（垫片调好）。');
    return;
  }
  try {
    log('停机：生成预演 ...', 'w');
    const p = await apiPost('/api/park', { preview: true, prereq });
    (p.segments || []).forEach((s) => log(`  ${s.label}  T=${s.T}s`));
    if (p.preview && p.preview.park && p.preview.park.note) log(p.preview.park.note, 'w');
    if (p.descent && p.descent.note) log('下降路径：' + p.descent.note, 'w');
    if (!confirm('停机预演已生成（见日志）。执行停机？\n\n抬高 → 收拢+折腕+转 J1 → 预停机点（读 I₀）\n→ 下降到 P（读 I₁）→ 搁架接住判定 → 按 J6→J1 失能。\n\n全程人不碰臂。')) return;
    await apiPost('/api/park', { preview: false, prereq });
    log('停机流程已启动', 'w');
  } catch (e) {
    log('停机失败：' + e.message, 'b');
    alert('停机失败：' + e.message);
  }
});

$('btn-unpark').addEventListener('click', async () => {
  if (!confirm('把 J2/J3 同步抬回预停机点、离开搁架？\n确认搁架在下面、人可以扶住。')) return;
  try {
    await apiPost('/api/park/unpark', { preview: false });
    log('抬回预停机点已开始（离开搁架后可以把搁架拿走）', 'w');
  } catch (e) { log('抬回失败：' + e.message, 'b'); }
});

$('btn-park-ack').addEventListener('click', async () => {
  if (!confirm('已经看过实物：停机后臂确实被动过/姿态不对，确认没问题、解除运动拦截？')) return;
  try {
    const r = await apiPost('/api/park/ack', {});
    log(r.msg || '已解除启停比对拦截', 'g');
  } catch (e) { log('解除失败：' + e.message, 'b'); }
});

$('btn-park-teach').addEventListener('click', async () => {
  if (!STATE || !STATE.startup_done) {
    alert('还没使能：实际位置要先完成零扭矩启动才能读。');
    return;
  }
  const q = (STATE.joints || []).map((j) => j.out_deg);
  if (q.length !== 6 || q.some((v) => v === null || v === undefined)) {
    alert('读不到完整六关节位置，不能记录。');
    return;
  }
  const txt = q.map((v, i) => `J${i + 1} ${Number(v).toFixed(2)}°`).join('  ');
  if (!confirm(`把当前六关节位置记录为搁架停机位置？\n\n${txt}\n\n只记录，不运动；会覆盖 config/park.json 的 poses.park。`)) return;
  try {
    const r = await apiPost('/api/park/teach', { confirmed: true });
    log('停机位置已记录：' + (r.q || []).map((v) => v.toFixed(1)).join(' / ') +
        `（J1 侧 ${r.park_side}）`, 'g');
  } catch (e) { log('记录停机位置失败：' + e.message, 'b'); alert('记录失败：' + e.message); }
});

$('btn-recover').addEventListener('click', async () => {
  try {
    const r = await apiPost('/api/recover', {});
    log(r.msg || '已恢复', 'g');
  } catch (e) { log('恢复失败：' + e.message, 'b'); }
});

$('btn-disable-all').addEventListener('click', async () => {
  if (!confirm('紧急全部失能：六颗立即断使能，臂没有刹车，会掉下来。\n确认臂已支撑/扶住，人站在空开旁？')) return;
  if (!confirm('再次确认：现在立即全部失能？\n能用“停机”或“扶持停机”时不要用这个。')) return;
  try {
    await apiPost('/api/disable_all', { confirmed: true, reason: 'web emergency' });
    log('紧急全部失能：已断使能（臂可能已掉下；必要时拉空开）', 'b');
  } catch (e) { log('紧急失能失败：' + e.message, 'b'); alert('紧急失能失败：' + e.message); }
});

$('btn-support-park').addEventListener('click', async () => {
  if (!confirm('扶持停机（备用）：六颗一起失能。\n必须：臂已在零位、人已用手扶住前臂。继续？')) return;
  if (!confirm('再确认一次：已扶住，现在全部失能？')) return;
  try {
    await apiPost('/api/park/support', { confirmed: true });
    log('扶持停机：已全部失能', 'w');
  } catch (e) { log('扶持停机失败：' + e.message, 'b'); }
});

// ---------------------------------------------------------------------------
// 18.4 Kt 标定
// ---------------------------------------------------------------------------
let KT_OPEN = false;
function openKt() {
  KT_OPEN = true;
  $('kt-modal').classList.remove('hidden');
  refreshKt();
}
function closeKt() {
  KT_OPEN = false;
  $('kt-modal').classList.add('hidden');
}
$('btn-kt').addEventListener('click', openKt);
$('kt-close').addEventListener('click', closeKt);

async function refreshKt() {
  try {
    const r = await getJSON('/api/kt/points');
    renderKtPoints(r);
  } catch (e) {
    $('kt-points').textContent = '读标定点失败：' + e.message;
  }
}

function renderKtPoints(r) {
  const rows = (r.points || []).map((p, i) =>
    `<tr><td>${p.group}</td><td>${p.label}</td><td>${p.q.map((v) => Number(v).toFixed(0)).join(', ')}</td>` +
    `<td>${p.load_kg}</td><td>${p.joints.join(',')}</td>` +
    `<td><button class="btn kt-goto" data-i="${i}" style="padding:1px 6px">走到</button></td></tr>`).join('');
  $('kt-points').innerHTML =
    `<table style="width:100%;border-collapse:collapse;font-size:11px">` +
    `<thead><tr><th>组</th><th>点</th><th>q</th><th>挂重kg</th><th>拟合关节</th><th></th></tr></thead>` +
    `<tbody>${rows}</tbody></table>`;
  $('kt-points').querySelectorAll('.kt-goto').forEach((b) => {
    b.addEventListener('click', async () => {
      const p = r.points[Number(b.dataset.i)];
      if (!confirm(`走到 ${p.label}：q = ${p.q.join(', ')}？\n先预演确认围栏/力矩预检。`)) return;
      try {
        const pv = await apiPost('/api/kt/goto', { q: p.q, preview: true });
        log(`Kt ${p.label} 预演：${(pv.preview.segments || []).length} 段`, 'g');
        if (!confirm('预演完成。执行走到该姿态？')) return;
        await apiPost('/api/kt/goto', { q: p.q, preview: false });
        $('kt-load').value = p.load_kg;
        $('kt-joints').value = p.joints.join(',');
      } catch (e) { log('走到标定点失败：' + e.message, 'b'); }
    });
  });
  const rr = (r.rows || []).map((x) =>
    `<tr><td>${x.joint}</td><td>${x.label || ''}</td><td>${x.load_kg}</td>` +
    `<td>${Number(x.current).toFixed(3)}</td><td>${Number(x.tau).toFixed(3)}</td>` +
    `<td>${x.full_scale_a === null ? '—' : x.full_scale_a}</td></tr>`).join('');
  $('kt-rows').innerHTML = rr
    ? `<table style="width:100%;border-collapse:collapse"><thead><tr><th>关节</th><th>点</th>` +
      `<th>挂重</th><th>I A</th><th>τ N·m</th><th>满量程 A</th></tr></thead><tbody>${rr}</tbody></table>`
    : '<span class="muted">还没有采样</span>';
}
$('kt-refresh').addEventListener('click', refreshKt);
$('kt-reset').addEventListener('click', async () => {
  try { await apiPost('/api/kt/reset', {}); refreshKt(); }
  catch (e) { log(e.message, 'b'); }
});
$('kt-sample').addEventListener('click', async () => {
  const joints = $('kt-joints').value.split(',').map((x) => x.trim().toUpperCase()).filter(Boolean);
  const load = Number($('kt-load').value) || 0;
  if (!confirm(`对 ${joints.join(',')} 采样？挂重 ${load} kg，取 20 个样本。\n确认重物已挂稳、人不在 +x 方向。`)) return;
  try {
    const r = await apiPost('/api/kt/sample', { joints, load_kg: load, label: `挂${load}kg` });
    log(`Kt 采样：${r.rows.map((x) => `${x.joint} ${x.current} A / τ ${x.tau}`).join('；')}`, 'g');
    refreshKt();
  } catch (e) { log('Kt 采样失败：' + e.message, 'b'); }
});
$('kt-fit').addEventListener('click', async () => {
  try {
    const r = await apiPost('/api/kt/fit', { save: true });
    const lines = Object.entries(r.fit || {}).map(([k, v]) =>
      `${k}: Kt=${v.kt_eff === null ? '—' : v.kt_eff} b=${v.offset === null ? '—' : v.offset} ` +
      `R²=${v.r2 === null ? '—' : v.r2}（${v.n} 点）`);
    $('kt-fit-out').innerHTML = lines.map((x) => `<div>${x}</div>`).join('') +
      `<div class="muted">R²≥0.95：${(r.r2_ok || []).join(',') || '无'}；容量 ${JSON.stringify(r.capacities)}</div>`;
    log('Kt 拟合：' + lines.join('；'), 'g');
    refreshKt();
  } catch (e) { log('Kt 拟合失败：' + e.message, 'b'); }
});

$('btn-write-limits').addEventListener('click', async () => {
  if (!confirm('按 config/joints.json 把六颗的限位/限流/限速写进 EEPROM（不动零位）？\n写的时候如果某颗原来失能会短暂零扭矩使能；请先扶住/支撑好。')) return;
  try {
    const r = await apiPost('/api/limits/write', { confirmed: true });
    (r.rows || []).forEach((x) => log(
      `${x.name}: 限位 ${JSON.stringify(x.limits_readback)}°，限流 ${x.current_limit_a} A，速度 ${x.speed_limit_rpm} RPM ${x.ok ? 'OK' : 'FAIL'}`,
      x.ok ? 'g' : 'b'));
    log(r.msg || '写限位完成', r.ok ? 'g' : 'b');
  } catch (e) { log('写限位失败：' + e.message, 'b'); alert('写限位失败：' + e.message); }
});

$('tcp-preset').addEventListener('change', async (e) => {
  const next = e.target.value;
  if (next !== 'flange') {
    const label = { probe: '临时针（实测前只能预演）', gripper_tip: '夹爪 tip 161.65',
                    gripper_center: '夹爪中心 139.15' }[next] || next;
    if (!confirm(`当前配置是空法兰（L=0）。\n确认已经实际装上 ${label}，并用卡尺核对过长度？\n没装就切过去，TCP 会差 90~181 mm。`)) {
      e.target.value = (STATE && STATE.tcp_preset) || 'flange';
      return;
    }
  }
  try {
    const r = await apiPost('/api/tcp', { preset: next });
    if (r && r.yaw !== undefined) {
      const y = $('yaw');
      y.value = String(r.yaw);
      $('yaw-val').textContent = `${r.yaw}°`;
    }
    lastReach = null;
    if (gizmoOn) { clampGizmoTarget(); scheduleReachCheck(); }
    log('TCP → ' + next + (r && r.yaw !== undefined ? `（ψ=${r.yaw}°）` : ''));
  } catch (err) { log(err.message, 'b'); }
});

const yawEl = $('yaw');
yawEl.addEventListener('input', () => { $('yaw-val').textContent = `${yawEl.value}°`; });
yawEl.addEventListener('change', async () => {
  try {
    await apiPost('/api/yaw', { yaw: Number(yawEl.value) });
    $('yaw-val').textContent = `${yawEl.value}°`;
    lastReach = null;
    if (gizmoOn) scheduleReachCheck();
  } catch (e) { log(e.message, 'b'); }
});

$('mode-preview').addEventListener('click', async () => {
  try { await apiPost('/api/exec_mode', { mode: 'preview' }); EXEC_MODE = 'preview'; updateModeUI(); }
  catch (e) { log(e.message, 'b'); }
});
$('mode-direct').addEventListener('click', async () => {
  if (!confirm('直接执行模式：点网格、拖 3D 松手、点“执行”或“回零位”都会立即动电机。\n确认已预演、工作区清空、臂已支撑、手在直流空开旁？')) return;
  try { await apiPost('/api/exec_mode', { mode: 'direct' }); EXEC_MODE = 'direct'; updateModeUI(); }
  catch (e) { log(e.message, 'b'); }
});

// 点动按钮
const JOG = [-5, -1, 1, 5];
const jogEl = $('jog');
['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].forEach((j) => {
  const n = document.createElement('span');
  n.className = 'jn'; n.textContent = j; jogEl.appendChild(n);
  JOG.forEach((d) => {
    const b = document.createElement('button');
    b.className = 'btn';
    b.textContent = (d > 0 ? '+' : '') + d;
    b.addEventListener('click', async () => {
      // 直接执行模式下点动也直接动；预演模式下只有勾了“点动直接执行”才动
      const live = EXEC_MODE === 'direct' || document.getElementById('jog-live').checked;
      const preview = !live;
      if (live && !confirm(
        `实际点动 ${j} ${d > 0 ? '+' : ''}${d}°？\n臂已支撑、工作区清空、人在直流空开旁？`)) return;
      try {
        await apiPost('/api/jog', { joint: j, delta: d, preview });
        log(`点动 ${j} ${d > 0 ? '+' : ''}${d}°${preview ? '（预演）' : '（实际）'}`,
            preview ? '' : 'w');
      } catch (e) { log('点动失败：' + e.message, 'b'); }
    });
    jogEl.appendChild(b);
  });
});

// ---------------------------------------------------------------------------
// 校准向导（16.3）
// ---------------------------------------------------------------------------
let CALIB_OPEN = false;

function openCalib() {
  CALIB_OPEN = true;
  $('calib-modal').classList.remove('hidden');
  CALIB_SIG = '';
  updateCalib(STATE && STATE.calib);
}
function closeCalib() {
  CALIB_OPEN = false;
  $('calib-modal').classList.add('hidden');
}
$('btn-calib').addEventListener('click', openCalib);
$('calib-close').addEventListener('click', closeCalib);

function calibStageText(c) {
  const map = {
    ready: '零扭矩中，等摆姿态', zeroed: '已设零，等点动',
    jogged: '已点动，等选方向', done: '完成', error: '出错', idle: '空闲',
  };
  if (!c) return '—';
  return map[c.stage] || c.stage;
}

const CALIB_ORDER = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1'];
let CALIB_STATE = null;
let CALIB_SIG = '';

function calibSignature(c) {
  if (!c) return 'null';
  return JSON.stringify({
    active: c.active, joint: c.joint, stage: c.stage, msg: c.msg,
    params: c.params, done: c.done, zeroed: c.zeroed,
    sign_checked: c.sign_checked, signs: c.signs, all_done: c.all_done,
    readonly: READONLY,
  });
}

function updateCalib(c) {
  CALIB_STATE = c;
  const sig = calibSignature(c);
  if (sig !== CALIB_SIG) {
    CALIB_SIG = sig;
    renderCalib(c);
  } else {
    updateCalibLive(c);
  }
}

function updateCalibLive(c) {
  if (!c) return;
  const set = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  const pos = (c.out_deg === null || c.out_deg === undefined) ? '—' : Number(c.out_deg).toFixed(2);
  set('calib-out', pos + '°');
  set('calib-turns', (c.pos_turns === null || c.pos_turns === undefined) ? '—' : Number(c.pos_turns).toFixed(4) + ' R');
  const batt = (c.battery_v === null || c.battery_v === undefined) ? '—' : Number(c.battery_v).toFixed(2);
  const err = c.error ? `0x${Number(c.error).toString(16).toUpperCase().padStart(4, '0')}` : '正常';
  set('calib-batt', `${batt} V · ${err}`);
  set('calib-mode', `${c.mode ?? '—'} · ${c.current_a ?? '—'} A`);
  set('calib-stage', calibStageText(c));
}

$('calib-body').addEventListener('click', (e) => {
  const st = e.target.closest && e.target.closest('.calib-start');
  if (st) { startCalib(st.dataset.joint); return; }
  const ac = e.target.closest && e.target.closest('.calib-act');
  if (ac) { calibAct(ac.dataset.act, ac.dataset.sign); return; }
  if (e.target.closest && (e.target.closest('#calib-next') || e.target.closest('#calib-next2'))) {
    goNext();
  }
  if (e.target.closest && e.target.closest('#calib-reacquire')) {
    tryAcquire().then(() => updateCalib(STATE && STATE.calib));
  }
});

function goNext() {
  const c = CALIB_STATE || {};
  const nextJ = CALIB_ORDER.find((n) => !(c.done && c.done[n]));
  if (nextJ) startCalib(nextJ);
  else closeCalib();
}

function renderCalib(c) {
  if (!c) { $('calib-body').innerHTML = '<p class="muted">还没有校准状态。</p>'; return; }
  const rows = CALIB_ORDER.map((n) => {
    let st = '未校准', cls = '';
    if (c.done && c.done[n]) { st = '完成'; cls = 'done'; }
    else if (c.joint === n && c.active) { st = calibStageText(c); cls = 'pending'; }
    else if (c.zeroed && c.zeroed[n]) { st = '已设零，待方向'; cls = 'pending'; }
    const active = c.joint === n && c.active;
    return `<div class="calib-joint-row">
      <div class="calib-joint-name">${n}</div>
      <div><div class="calib-status ${cls}">${st}</div>
        <div class="calib-status">sign=${c.signs ? c.signs[n] : '?'} ${c.sign_checked && c.sign_checked[n] ? '· 已核对' : ''}</div></div>
      <div>${active ? '<span class="calib-status pending">进行中</span>' :
        `<button class="btn calib-start" data-joint="${n}" ${READONLY ? 'disabled' : ''}>${c.done && c.done[n] ? '重做' : '开始'}</button>`}</div>
    </div>`;
  }).join('');

  let activeBlock = '';
  if (c.active) {
    const pos = c.out_deg === null || c.out_deg === undefined ? '—' : c.out_deg.toFixed(2);
    const battery = c.battery_v === null || c.battery_v === undefined ? '—' : c.battery_v.toFixed(2);
    const err = c.error ? `0x${c.error.toString(16).toUpperCase().padStart(4, '0')}` : '正常';
    activeBlock = `
      <div class="calib-ref"><b>${c.joint}</b>：${c.reference}</div>
      <div class="calib-live">
        <div><span>输出端角度</span><b id="calib-out">${pos}°</b></div>
        <div><span>电机位置</span><b id="calib-turns">${c.pos_turns === null ? '—' : c.pos_turns.toFixed(4)} R</b></div>
        <div><span>电池 / 错误</span><b id="calib-batt" style="font-size:13px">${battery} V · ${err}</b></div>
        <div><span>模式 / 电流</span><b id="calib-mode" style="font-size:13px">${c.mode ?? '—'} · ${c.current_a ?? '—'} A</b></div>
        <div><span>阶段</span><b id="calib-stage" style="font-size:13px">${calibStageText(c)}</b></div>
      </div>
      ${c.params ? `<div class="calib-note">已写：限位 ${c.params.limits_deg[0]}…${c.params.limits_deg[1]}°，
        限流 ${c.params.current_limit_a ?? '出厂值不动'} A，速度限制 ${c.params.speed_limit_rpm} RPM，
        零位日期 ${c.params.zero_date}</div>` : ''}
      <div class="calib-note">正方向的物理表现：${c.expected}</div>
    `;
  }

  let actions = '';
  if (c.active) {
    if (c.stage === 'ready') {
      actions = `<button class="btn primary calib-act" data-act="set_zero">已摆好参考姿态 → 设为零位</button>`;
    } else if (c.stage === 'zeroed') {
      actions = `<button class="btn primary calib-act" data-act="jog">点动 +5°（电机正方向）</button>`;
    } else if (c.stage === 'jogged') {
      actions = `
        <button class="btn primary calib-act" data-act="sign" data-sign="1">方向符合 → sign=+1</button>
        <button class="btn primary calib-act" data-act="sign" data-sign="-1">方向相反 → sign=−1</button>
        <button class="btn calib-act" data-act="return">退回零位（重新看）</button>`;
    } else if (c.stage === 'done') {
      actions = `<button class="btn primary" id="calib-next">继续下一颗</button>`;
    } else if (c.stage === 'error') {
      actions = `<button class="btn calib-act" data-act="abort">结束并失能</button>`;
    }
    actions += ` <button class="btn danger calib-act" data-act="abort">中止并失能</button>`;
  }

  $('calib-body').innerHTML = `
    <div class="calib-note" style="margin-top:0">
      顺序固定 J6 → J5 → J4 → J3 → J2 → J1。每颗关节校准时其余关节全部失能，
      <b>臂必须已经支撑好</b>，人站在直流空开旁。设零流程：清 Homing → 设零 → 回读
      → 写限位/限流/限速 → 存 EEPROM → 全部回读比对。
    </div>
    <div class="calib-note">
      方向简化说法：轴沿 y 的 J2、J3、J4、J6，上面那一头往前走（远离人、往 +x）= 正；
      轴竖直的 J1、J5，从上往下看逆时针 = 正；夹爪张开（h 变大）= 正。<br>
      不要拿机械限位块设零：J1–J5 的零位都在机械行程正中间。J2 机械限位 ±100°，
      所以 EEPROM 写 ±95°、软件 ±90°。
    </div>
    ${c.active ? activeBlock : ''}
    ${c.active ? `<div class="calib-actions">${actions}</div>` : ''}
    ${!c.active && c.joint && c.stage === 'done' ? `<div class="calib-actions"><button class="btn primary" id="calib-next2">继续下一颗</button></div>` : ''}
    <div style="margin-top:14px">${rows}</div>
    ${READONLY ? '<div class="calib-note warn">当前是只读（没有控制权）。<button class="btn" id="calib-reacquire">重新拿控制权</button></div>' : ''}
    ${c.all_done ? '<div class="calib-note ok">六颗全部完成。关掉此面板，回到“连接”→ 连接真机 → 使能，开始 16.4 验收。</div>' : ''}
    ${c.msg ? `<div class="calib-note">${c.msg}</div>` : ''}
  `;

}

async function startCalib(joint) {
  if (READONLY) return;
  if (!confirm(`开始校准 ${joint}？\n\n会先让其余 5 个关节失能，再对 ${joint} 零扭矩使能（电流模式 0 A，输出端可用手转动）。\n臂必须已经支撑好，人站在直流空开旁。继续？`)) return;
  try {
    const r = await apiPost('/api/calib/prepare', { joint, confirmed_supported: true });
    STATE.calib = r.calib;
    updateCalib(r.calib);
    log(`${joint} 零扭矩使能，当前 ${r.calib.out_deg}°`, 'g');
  } catch (e) {
    log('校准准备失败：' + e.message, 'b');
    alert('校准准备失败：' + e.message);
  }
}

async function calibAct(act, sign) {
  if (READONLY) return;
  try {
    let r = null;
    if (act === 'set_zero') {
      if (!confirm('确认已经把关节摆到参考姿态（看上方说明）？\n将清除 Homing 并把当前位置设为零，回读 |q|<0.05° 才保存。')) return;
      r = await apiPost('/api/calib/set_zero', { joint: STATE.calib.joint });
    } else if (act === 'jog') {
      r = await apiPost('/api/calib/jog', { joint: STATE.calib.joint, delta: 5 });
    } else if (act === 'sign') {
      const s = Number(sign);
      if (!confirm(`确认实际方向对应 sign=${s > 0 ? '+1' : '−1'}？\n写入配置后会把关节退回零位并失能。`)) return;
      r = await apiPost('/api/calib/confirm_sign', { joint: STATE.calib.joint, sign: s });
    } else if (act === 'return') {
      r = await apiPost('/api/calib/return', { joint: STATE.calib.joint });
    } else if (act === 'abort') {
      if (!confirm('中止校准并失能目标关节？')) return;
      r = await apiPost('/api/calib/abort', {});
    }
    if (r && r.calib) {
      STATE.calib = r.calib;
      updateCalib(r.calib);
      log(r.calib.msg || '校准步骤完成', 'g');
    }
  } catch (e) {
    log('校准失败：' + e.message, 'b');
    alert('校准失败：' + e.message);
  }
}

// ---------------------------------------------------------------------------
// 手动拖动测量
// ---------------------------------------------------------------------------
let MEASURE_OPEN = false;

function openMeasure() {
  MEASURE_OPEN = true;
  $('measure-modal').classList.remove('hidden');
  updateMeasureUI(STATE || {});
}
function closeMeasure() {
  MEASURE_OPEN = false;
  $('measure-modal').classList.add('hidden');
}
$('btn-measure').addEventListener('click', openMeasure);
$('measure-close').addEventListener('click', closeMeasure);

function updateMeasureUI(s) {
  if (!MEASURE_OPEN) return;
  const tcp = s.tcp;
  const set = (id, text) => { const el = document.getElementById(id); if (el) el.textContent = text; };
  set('m-x', tcp ? Number(tcp[0]).toFixed(1) : '—');
  set('m-y', tcp ? Number(tcp[1]).toFixed(1) : '—');
  set('m-z', tcp ? Number(tcp[2]).toFixed(1) : '—');
  set('m-preset', s.tcp_preset || '—');
  const drag = !!(s.drag && s.drag.active);
  set('m-state', drag ? '零扭矩（可拖动）' : '位置保持');
  $('m-start').disabled = drag || !s.connected || READONLY;
  $('m-stop').disabled = !drag || READONLY;
  $('m-record').disabled = !s.connected || (s.motion && s.motion.busy);
  const rows = (s.measurements || []).slice().reverse().map((r, i) => {
    const t = r.tcp ? `(${r.tcp[0].toFixed(1)}, ${r.tcp[1].toFixed(1)}, ${r.tcp[2].toFixed(1)})` : '—';
    const a = r.actual ? `(${r.actual[0].toFixed(1)}, ${r.actual[1].toFixed(1)}, ${r.actual[2].toFixed(1)})` : '—';
    return `<tr><td>${(s.measurements.length - i)}</td><td>${r.time || ''}</td>` +
      `<td>${t}</td><td>${a}</td><td class="${r.err_mm > 5 ? 'bad' : (r.err_mm !== null ? 'ok' : '')}">` +
      `${r.err_mm === null || r.err_mm === undefined ? '—' : r.err_mm.toFixed(2)}</td></tr>`;
  }).join('');
  const tb = document.querySelector('#m-table tbody');
  if (tb) tb.innerHTML = rows || '<tr><td colspan="5" class="muted">还没有记录</td></tr>';
}

$('m-start').addEventListener('click', async () => {
  if (!confirm('所有关节将切到零扭矩（电流模式 0 A），臂会在重力下掉。\n确认已经用手扶住/支撑好末端，人站在直流空开旁？')) return;
  try {
    await apiPost('/api/drag/start', { confirmed: true });
    log('进入零扭矩拖动：扶住末端，拖到实际点', 'w');
    await pollState();
  } catch (e) { log('开始拖动失败：' + e.message, 'b'); alert('开始拖动失败：' + e.message); }
});

$('m-stop').addEventListener('click', async () => {
  try {
    await apiPost('/api/drag/stop', {});
    log('已切回位置保持', 'g');
    await pollState();
  } catch (e) { log('锁定失败：' + e.message, 'b'); }
});

$('m-record').addEventListener('click', async () => {
  const ax = $('m-ax').value.trim(), ay = $('m-ay').value.trim(), az = $('m-az').value.trim();
  let actual = null;
  if (ax !== '' || ay !== '' || az !== '') {
    if (ax === '' || ay === '' || az === '') {
      alert('实际点要么三个都填，要么都不填');
      return;
    }
    actual = [Number(ax), Number(ay), Number(az)];
    if (actual.some((v) => Number.isNaN(v))) { alert('实际坐标不是数字'); return; }
  }
  try {
    const r = await apiPost('/api/measure/record', { actual });
    log(`记录点：TCP (${r.record.tcp.join(', ')}) mm` +
      (r.record.err_mm !== null ? `，偏差 ${r.record.err_mm} mm` : ''), 'g');
    $('m-ax').value = ''; $('m-ay').value = ''; $('m-az').value = '';
    if (STATE) STATE.measurements = r.measurements;
    updateMeasureUI(STATE || {});
  } catch (e) { log('记录失败：' + e.message, 'b'); alert('记录失败：' + e.message); }
});

$('m-clear').addEventListener('click', async () => {
  if (!confirm('清空网页里的测量记录？（CSV 文件保留）')) return;
  try {
    await apiPost('/api/measure/clear', {});
    if (STATE) STATE.measurements = [];
    updateMeasureUI(STATE || {});
    log('测量记录已清空', 'w');
  } catch (e) { log('清空失败：' + e.message, 'b'); }
});

// ---------------------------------------------------------------------------
// 渲染
// ---------------------------------------------------------------------------
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas);
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

window.__arm = { gizmo, gizmoProxy, gizmoSend, setGizmoFromTcp, camera, renderer, controls, scene, setDrag(v) { gizmoDragging = !!v; }, get state() { return STATE; }, get mode() { return EXEC_MODE; }, get readonly() { return READONLY; } };

init();
setInterval(() => { if (!sseOk) pollState(); }, 500);
log('页面已加载。正在拿控制权…', 'g');

// ---------------------------------------------------------------------------
// 运行时页面快照（debug）：只保留最新一份，写到执行器 runs/web_state.json
// 记录所有按钮的亮/灰、可见性、文字，所有输入/下拉的当前值，以及所有带 id
// 的窗口显示文字。100 ms 一次；内容没变不发。页面只读时也发（不要求 token）。
// ---------------------------------------------------------------------------
function pageStateSummary(s) {
  if (!s) return null;
  return {
    connected: s.connected, startup_done: s.startup_done, msg: s.msg,
    exec_mode: s.exec_mode, tcp_preset: s.tcp_preset, yaw: s.yaw,
    has_plan: s.has_plan, control: s.control, safe_check: s.safe_check,
    motion: s.motion, drag: s.drag, hold: s.hold,
    park: s.park ? {
      park_side: s.park.park_side, park_taught: s.park.park_taught,
      park_taught_at: s.park.park_taught_at,
      step: s.park.step, block: s.park.block, check: s.park.check,
    } : null,
    near_limit: s.near_limit,
  };
}

function collectPageSnapshot() {
  const visible = (el) => {
    try { return !!(el.getClientRects && el.getClientRects().length); }
    catch (_) { return false; }
  };
  const textOf = (el) => (el.textContent || '').replace(/\s+/g, ' ').trim().slice(0, 500);
  const buttons = Array.from(document.querySelectorAll('button')).map((el) => ({
    id: el.id || '', text: textOf(el), disabled: !!el.disabled,
    visible: visible(el), class: el.className || '',
  }));
  const fields = Array.from(document.querySelectorAll('input,select,textarea')).map((el) => {
    const o = {
      id: el.id || '', tag: el.tagName.toLowerCase(), type: el.type || '',
      value: el.value, disabled: !!el.disabled, visible: visible(el),
    };
    if (el.tagName === 'INPUT' && (el.type === 'checkbox' || el.type === 'radio')) {
      o.checked = !!el.checked;
    }
    if (el.tagName === 'SELECT') {
      o.selected_text = el.selectedOptions && el.selectedOptions[0]
        ? el.selectedOptions[0].textContent.trim() : '';
      o.options = Array.from(el.options).map((x) => ({ value: x.value, text: x.text }));
    }
    return o;
  });
  const displays = {};
  document.querySelectorAll('[id]').forEach((el) => {
    const t = textOf(el);
    if (t) displays[el.id] = t;
  });
  return {
    snapshot_ts: new Date().toISOString(),
    page_version: WEB_VERSION,
    document_title: document.title,
    url: location.href,
    client_id: CLIENT_ID,
    readonly: !!READONLY,
    exec_mode: EXEC_MODE,
    buttons,
    fields,
    displays,
    state: pageStateSummary(STATE),
  };
}

let lastPageSnapshot = '';
function pushPageSnapshot() {
  try {
    const snap = collectPageSnapshot();
    const raw = JSON.stringify(snap);
    if (raw === lastPageSnapshot) return;
    lastPageSnapshot = raw;
    fetch('/api/debug/page_state', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: raw, cache: 'no-store',
    }).catch(() => {});
  } catch (_) { /* 快照绝不干扰主流程 */ }
}
setInterval(pushPageSnapshot, 100);
