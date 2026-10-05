// Helicóptero UTI (Template A, classe H135) · HUD e cenas do Gêmeo Digital.
// Esta página só reproduz telemetria calculada pelo Twin (tools/heli_scenes_export.py): posição, atitude, NR,
// torque, margens, eventos do SADPF e a previsão de ramos vêm da simulação física, não de animação.
import * as THREE from './vendor/three.module.min.js';

const W = 1920, H = 1080;
const params = new URLSearchParams(location.search);
const CAPTURE = params.has('capture');
if (CAPTURE) document.body.classList.add('capture');

// ------------------------------------------------------------------------------------------------ data
async function load(name) { const r = await fetch(`data/${name}.json`); if (!r.ok) throw new Error(name); return r.json(); }
const [DA, DB, DC, DM] = await Promise.all(['cat_a', 'advisory', 'autorotation', 'mission'].map(load));

const fmt = (v, d = 0) => (v === null || v === undefined || !isFinite(v)) ? '—' :
  (Math.abs(v) < 0.5 * Math.pow(10, -d) ? 0 : Number(v)).toLocaleString('pt-BR', { minimumFractionDigits: d, maximumFractionDigits: d });
const kg = (v) => `${fmt(v, 0)} kg`;

// column-oriented frames -> sampler with linear interpolation (0,1 s records)
function prepRun(run) {
  const f = run.frames, n = f.t.length;
  // cumulative rotor angle from NR (rad)
  const ang = new Float64Array(n);
  for (let i = 1; i < n; i++) ang[i] = ang[i - 1] + run.omega100 * (f.nr_pct[i - 1] / 100) * (f.t[i] - f.t[i - 1]);
  run._ang = ang; run._n = n; run.tEnd = f.t[n - 1];
  return run;
}
function sample(run, t) {
  const f = run.frames, n = run._n;
  t = Math.max(f.t[0], Math.min(t, f.t[n - 1]));
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (f.t[m] <= t) lo = m; else hi = m; }
  const u = (t - f.t[lo]) / Math.max(f.t[hi] - f.t[lo], 1e-9);
  const o = { t };
  for (const k in f) {
    const a = f[k][lo], b = f[k][hi];
    o[k] = (a === null || b === null) ? (a ?? b) : a + (b - a) * u;
  }
  o.rotor_angle = run._ang[lo] + (run._ang[hi] - run._ang[lo]) * u;
  o.idx = lo;
  return o;
}
for (const r of Object.values(DA.runs)) prepRun(r);
for (const r of Object.values(DB.runs)) prepRun(r);
for (const r of Object.values(DC.runs)) prepRun(r);

// ------------------------------------------------------------------------------------------------ renderer
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setSize(W, H);
renderer.setPixelRatio(1);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
renderer.outputColorSpace = THREE.SRGBColorSpace;
document.getElementById('viewport').appendChild(renderer.domElement);

function rng(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function hash(x, y) { let h = x * 374761393 + y * 668265263; h = (h ^ (h >> 13)) * 1274126177; return ((h ^ (h >> 16)) >>> 0) / 4294967295; }
function vnoise(x, y) {
  const xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi;
  const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
  const a = hash(xi, yi), b = hash(xi + 1, yi), c = hash(xi, yi + 1), d = hash(xi + 1, yi + 1);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}
function fbm(x, y, oct = 5) { let s = 0, a = 0.5, f = 1; for (let i = 0; i < oct; i++) { s += a * vnoise(x * f, y * f); a *= 0.5; f *= 2.03; } return s; }
function canvasTex(w, h, draw, repeat = 1) {
  const c = document.createElement('canvas'); c.width = w; c.height = h; draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8;
  if (repeat !== 1) { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat, repeat); }
  return t;
}

// a world = scene + ENU root (x forward, y left, z up) + sky + sun
function makeWorld({ skyTop, skyHor, fog, fogNear, fogFar, hemiSky, hemiGround, sunI = 2.6 }) {
  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(new THREE.Color(fog), fogNear, fogFar);
  const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);
  const sunDir = new THREE.Vector3(-0.45, 0.62, 0.55).normalize(); // three coords
  const g = new THREE.SphereGeometry(4500, 64, 32);
  const m = new THREE.ShaderMaterial({
    side: THREE.BackSide, depthWrite: false, fog: false,
    uniforms: { top: { value: new THREE.Color(skyTop) }, hor: { value: new THREE.Color(skyHor) }, sunDir: { value: sunDir } },
    vertexShader: 'varying vec3 vP; void main(){ vP = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
    fragmentShader: 'uniform vec3 top; uniform vec3 hor; uniform vec3 sunDir; varying vec3 vP; void main(){ vec3 d = normalize(vP); float h = clamp(d.y,0.0,1.0); vec3 c = mix(hor, top, pow(h,0.5)); float s = max(dot(d, sunDir),0.0); c += vec3(1.0,0.95,0.85)*pow(s,700.0)*0.8 + vec3(0.25,0.22,0.18)*pow(s,6.0)*0.35; gl_FragColor = vec4(c,1.0); }',
  });
  scene.add(new THREE.Mesh(g, m));
  scene.add(new THREE.HemisphereLight(hemiSky, hemiGround, 1.1));
  const sun = new THREE.DirectionalLight(0xfff4e2, sunI);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -40, right: 40, top: 40, bottom: -40, near: 1, far: 400 });
  sun.shadow.bias = -0.0004;
  scene.add(sun); scene.add(sun.target);
  return { scene, root, sun, sunDir };
}
const enu = (x, y, z) => new THREE.Vector3(x, z, -y);
function followSun(w, x, y, z, span = 40) {
  const c = enu(x, y, z);
  w.sun.position.copy(c).add(w.sunDir.clone().multiplyScalar(180));
  w.sun.target.position.copy(c);
  const s = w.sun.shadow.camera; s.left = -span; s.right = span; s.top = span; s.bottom = -span; s.updateProjectionMatrix();
}

function box(world, x0, x1, y0, y1, z0, z1, mat, shadow = true) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0), mat);
  m.position.set((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
  m.castShadow = shadow; m.receiveShadow = true; world.root.add(m); return m;
}
function mountains(world, radius, base, hMax, color, seed) {
  // distant ridge as a ring of terrain (illustrative)
  const seg = 360, rows = 6;
  const g = new THREE.BufferGeometry(), pos = [], idx = [];
  for (let i = 0; i <= seg; i++) {
    const a = i / seg * Math.PI * 2;
    const hgt = hMax * (0.35 + 0.65 * fbm(Math.cos(a) * 3 + seed, Math.sin(a) * 3 - seed, 5));
    for (let j = 0; j <= rows; j++) {
      const u = j / rows; const r = radius * (1 + 0.25 * (1 - u));
      pos.push(r * Math.cos(a), r * Math.sin(a), base + hgt * Math.sin(u * Math.PI / 2));
    }
  }
  for (let i = 0; i < seg; i++) for (let j = 0; j < rows; j++) {
    const a = i * (rows + 1) + j, b = a + rows + 1; idx.push(a, b, a + 1, b, b + 1, a + 1);
  }
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setIndex(idx); g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color, roughness: 1, flatShading: true }));
  world.root.add(m);
}
function groundPlane(world, size, z, tex, color = '#ffffff') {
  const m = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshStandardMaterial({ map: tex, color, roughness: 0.95 }));
  m.position.z = z; m.receiveShadow = true; world.root.add(m); return m;
}

// ------------------------------------------------------------------------------------------------ helicopter
// Generic light twin: 4-blade main rotor, ducted tail rotor (Fenestron type) with fin, skids, twin-engine cowling.
// No logos, registration or manufacturer livery. Built Y-up (x forward, y up, z right) and turned into ENU.
const HM = {
  body: new THREE.MeshStandardMaterial({ color: '#eceef0', metalness: 0.25, roughness: 0.38 }),
  cowl: new THREE.MeshStandardMaterial({ color: '#c8cdd3', metalness: 0.3, roughness: 0.42 }),
  belly: new THREE.MeshStandardMaterial({ color: '#8a9098', metalness: 0.3, roughness: 0.5 }),
  dark: new THREE.MeshStandardMaterial({ color: '#2a2e34', metalness: 0.4, roughness: 0.45 }),
  glass: new THREE.MeshStandardMaterial({ color: '#16222e', metalness: 0.7, roughness: 0.12 }),
  blade: new THREE.MeshStandardMaterial({ color: '#30353c', metalness: 0.3, roughness: 0.5 }),
  skid: new THREE.MeshStandardMaterial({ color: '#7b828a', metalness: 0.6, roughness: 0.35 }),
};
function cylBetween(a, b, r, mat, seg = 16) {
  const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, A.distanceTo(B), seg), mat);
  m.position.copy(A).add(B).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
  return m;
}
function ellipsoid(sx, sy, sz, mat, opts = []) {
  const g = new THREE.SphereGeometry(1, 64, 40, ...opts);
  const m = new THREE.Mesh(g, mat); m.scale.set(sx, sy, sz); return m;
}
function makeHeli(cgH, R) {
  const outer = new THREE.Group();            // ENU pose
  const m = new THREE.Group(); m.rotation.x = Math.PI / 2; outer.add(m); // Y-up model -> ENU
  // fuselage: cabin + rear pod (CG at the origin; skids at y = -cgH)
  const cab = ellipsoid(2.75, 0.95, 0.86, HM.body); cab.position.set(0.75, -0.1, 0); m.add(cab);
  const pod = ellipsoid(1.7, 0.82, 0.78, HM.body); pod.position.set(-0.85, 0.05, 0); m.add(pod);
  const belly = ellipsoid(2.4, 0.42, 0.8, HM.belly); belly.position.set(0.4, -0.62, 0); m.add(belly);
  // windscreen and cabin windows (dark glass patches on the same ellipsoid, slightly larger)
  const ws = ellipsoid(2.77, 0.965, 0.875, HM.glass, [Math.PI - 0.95, 1.9, 0.18, 1.05]); ws.position.copy(cab.position); m.add(ws);
  for (const s of [1, -1]) {
    const wl = ellipsoid(2.76, 0.96, 0.87, HM.glass, [s > 0 ? Math.PI / 2 - 0.22 : 3 * Math.PI / 2 - 0.3, 0.52, 0.55, 0.62]);
    wl.position.copy(cab.position); m.add(wl);
    const wr = ellipsoid(1.71, 0.83, 0.79, HM.glass, [s > 0 ? Math.PI / 2 - 0.45 : 3 * Math.PI / 2 - 0.45, 0.9, 0.62, 0.5]);
    wr.position.copy(pod.position); m.add(wr);
  }
  // engine cowling (two engines side by side under one fairing) and exhausts
  const cowl = new THREE.Mesh(new THREE.CapsuleGeometry(0.5, 2.0, 8, 24), HM.cowl);
  cowl.rotation.z = Math.PI / 2; cowl.scale.set(0.85, 1, 1.25); cowl.position.set(-0.75, 0.78, 0); m.add(cowl);
  const exhaust = [];
  for (const s of [1, -1]) {
    const e = cylBetween([-1.55, 0.95, 0.42 * s], [-2.05, 1.05, 0.52 * s], 0.13, HM.dark); m.add(e); exhaust.push(e);
  }
  // mast, hub, blades
  m.add(cylBetween([0.15, 1.1, 0], [0.15, 1.72, 0], 0.11, HM.dark));
  const hub = new THREE.Group(); hub.position.set(0.15, 1.72, 0); m.add(hub);
  const spin = new THREE.Group(); hub.add(spin);
  spin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.32, 0.16, 24), HM.dark));
  const chord = 0.288;
  for (let k = 0; k < 4; k++) {
    const holder = new THREE.Group(); holder.rotation.y = k * Math.PI / 2; spin.add(holder);
    const b = new THREE.Mesh(new THREE.BoxGeometry(R - 0.35, 0.045, chord), HM.blade);
    b.position.set(0.35 + (R - 0.35) / 2, 0, 0); b.rotation.x = 0.06; b.castShadow = true; holder.add(b);
    const tip = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.046, chord * 0.8), HM.cowl);
    tip.position.set(R - 0.12, 0, 0.02); holder.add(tip);
  }
  const blur = new THREE.Mesh(new THREE.CircleGeometry(R, 72), new THREE.MeshBasicMaterial({ color: 0x1d2228, transparent: true, opacity: 0.0, side: THREE.DoubleSide, depthWrite: false }));
  blur.rotation.x = -Math.PI / 2; blur.position.y = 0.02; hub.add(blur);
  // tail boom
  const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.42, 4.7, 24), HM.body);
  boom.rotation.z = Math.PI / 2; boom.position.set(-4.15, 0.3, 0); m.add(boom);
  // ducted tail rotor: shroud with a 1,0 m duct (TCDS R.009), fin above, stabiliser with end plates
  const sh = new THREE.Shape();
  sh.moveTo(-0.95, -0.55); sh.lineTo(0.75, -0.55); sh.quadraticCurveTo(1.05, -0.5, 1.0, 0.1);
  sh.lineTo(0.55, 0.8); sh.lineTo(-0.2, 2.15); sh.lineTo(-0.85, 2.2); sh.lineTo(-0.95, 0.9); sh.lineTo(-1.1, -0.2);
  sh.quadraticCurveTo(-1.1, -0.55, -0.95, -0.55);
  const hole = new THREE.Path(); hole.absarc(0, 0.15, 0.5, 0, Math.PI * 2, true); sh.holes.push(hole);
  const shroud = new THREE.Mesh(new THREE.ExtrudeGeometry(sh, { depth: 0.34, bevelEnabled: true, bevelThickness: 0.04, bevelSize: 0.04, bevelSegments: 2, curveSegments: 40 }), HM.body);
  shroud.position.set(-7.0, 0.15, -0.17); m.add(shroud);
  const duct = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 0.36, 40, 1, true), HM.belly);
  duct.material = HM.belly.clone(); duct.material.side = THREE.DoubleSide;
  duct.rotation.x = Math.PI / 2; duct.position.set(-7.0, 0.3, 0); m.add(duct);
  const fan = new THREE.Group(); fan.position.set(-7.0, 0.3, 0.02); m.add(fan);
  fan.add(new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.2, 20).rotateX(Math.PI / 2), HM.dark));
  for (let k = 0; k < 10; k++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.07, 0.02), HM.dark);
    const h = new THREE.Group(); h.rotation.z = k * Math.PI / 5; b.position.x = 0.3; b.rotation.x = 0.5; h.add(b); fan.add(h);
  }
  for (let k = 0; k < 3; k++) { // stator vanes
    const v = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.04, 0.03), HM.belly);
    const h = new THREE.Group(); h.rotation.z = 0.4 + k * 2.1; v.position.x = 0.3; h.add(v); h.position.z = -0.12; fan.add(h);
  }
  const stab = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.06, 2.6), HM.body); stab.position.set(-5.7, 0.38, 0); m.add(stab);
  for (const s of [1, -1]) { const ep = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.55, 0.05), HM.body); ep.position.set(-5.75, 0.45, 1.3 * s); m.add(ep); }
  // skids
  const yS = -cgH;
  for (const s of [1, -1]) {
    m.add(cylBetween([-1.55, yS, 1.05 * s], [1.75, yS, 1.05 * s], 0.05, HM.skid));
    m.add(cylBetween([1.75, yS, 1.05 * s], [2.15, yS + 0.3, 1.05 * s], 0.05, HM.skid));
    for (const x of [-0.85, 1.05]) m.add(cylBetween([x, -0.62, 0.45 * s], [x, yS, 1.05 * s], 0.055, HM.skid));
  }
  // failed-engine marker (shown after the SADPF detection)
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.55, 0.05, 10, 48), new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0 }));
  ring.rotation.y = Math.PI / 2; ring.position.set(-1.6, 1.0, 0.45); m.add(ring);
  m.traverse(o => { if (o.isMesh && !o.material.transparent) { o.castShadow = true; o.receiveShadow = true; } });
  return { group: outer, spin, blur, fan, ring };
}
function poseHeli(h, d, tNow, failT, detectT, blink) {
  h.group.position.set(d.x, d.y, d.z);
  h.group.rotation.set(THREE.MathUtils.degToRad(d.roll_deg), THREE.MathUtils.degToRad(d.pitch_deg), THREE.MathUtils.degToRad(d.yaw_deg), 'ZYX');
  h.spin.rotation.y = -d.rotor_angle % (Math.PI * 2);
  const sp = Math.min(1, Math.max(0, (d.nr_pct - 20) / 70));
  h.blur.material.opacity = 0.24 * sp;
  h.fan.rotation.z = (d.rotor_angle * 5.3) % (Math.PI * 2);
  h.ring.material.opacity = (detectT !== null && tNow >= detectT) ? (blink ? 0.85 : 0.45) : 0;
}

// ------------------------------------------------------------------------------------------------ world A: city, elevated heliport
const WA = makeWorld({ skyTop: '#5f93c9', skyHor: '#d8e6f2', fog: '#c9d9e8', fogNear: 600, fogFar: 4200, hemiSky: '#e4efff', hemiGround: '#5b5f55' });
{
  const deck = DA.condition.deck_height_m, half = DA.condition.deck_size_m / 2;
  const cityTex = canvasTex(1024, 1024, (x, w, h) => {
    x.fillStyle = '#7d8079'; x.fillRect(0, 0, w, h);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
      const v = hash(i, j); x.fillStyle = v > 0.7 ? '#6f8a5a' : v > 0.4 ? '#9a9a92' : '#8b8d86';
      x.fillRect(i * 256 + 18, j * 256 + 18, 220, 220);
    }
    for (let i = 0; i < 9000; i++) { x.fillStyle = 'rgba(60,60,55,0.10)'; x.fillRect(hash(i, 71) * w, hash(i, 72) * h, 3, 3); }
  }, 6000 / 360);
  groundPlane(WA, 6000, -deck, cityTex);
  // open plaza/park beyond the deck edge (no obstacles in the continue path, as in the model)
  const park = canvasTex(512, 512, (x, w, h) => { x.fillStyle = '#6e8f58'; x.fillRect(0, 0, w, h); for (let i = 0; i < 3000; i++) { const v = 90 + hash(i, 5) * 60; x.fillStyle = `rgba(${v * 0.7},${v},${v * 0.55},0.5)`; x.fillRect(hash(i, 6) * w, hash(i, 7) * h, 3, 3); } x.fillStyle = '#b9b3a3'; x.fillRect(0, h * 0.46, w, h * 0.08); }, 6);
  const pk = new THREE.Mesh(new THREE.PlaneGeometry(560, 140), new THREE.MeshStandardMaterial({ map: park, roughness: 1 }));
  pk.position.set(300, 0, -deck + 0.05); pk.receiveShadow = true; WA.root.add(pk);
  // hospital tower: footprint = the deck (20 m x 20 m, 30 m high)
  const winTex = canvasTex(512, 512, (x, w, h) => {
    x.fillStyle = '#d9d5cc'; x.fillRect(0, 0, w, h);
    for (let r = 0; r < 10; r++) for (let c = 0; c < 8; c++) { x.fillStyle = hash(r, c) > 0.85 ? '#5d7287' : '#3c4e60'; x.fillRect(c * 64 + 10, r * 51 + 12, 44, 26); }
  });
  const facade = new THREE.MeshStandardMaterial({ map: winTex, roughness: 0.7 });
  box(WA, -half, half, -half, half, -deck, -0.45, facade);
  // deck slab and markings (generic heliport marking: white H, touchdown circle, yellow perimeter)
  const deckTex = canvasTex(1024, 1024, (x, w, h) => {
    x.fillStyle = '#4b5056'; x.fillRect(0, 0, w, h);
    x.strokeStyle = '#f2d23a'; x.lineWidth = 22; x.strokeRect(20, 20, w - 40, h - 40);
    x.strokeStyle = '#f2f2f2'; x.lineWidth = 26; x.beginPath(); x.arc(w / 2, h / 2, w * 0.3, 0, Math.PI * 2); x.stroke();
    x.fillStyle = '#f2f2f2'; const s = w * 0.2; x.fillRect(w / 2 - s * 0.55, h / 2 - s * 0.8, s * 0.22, s * 1.6); x.fillRect(w / 2 + s * 0.33, h / 2 - s * 0.8, s * 0.22, s * 1.6); x.fillRect(w / 2 - s * 0.55, h / 2 - s * 0.11, s * 1.1, s * 0.22);
  });
  const deckMat = new THREE.MeshStandardMaterial({ map: deckTex, roughness: 0.8 });
  const slab = new THREE.Mesh(new THREE.BoxGeometry(2 * half, 2 * half, 0.45), [HM.belly, HM.belly, HM.belly, HM.belly, deckMat, HM.belly]);
  slab.position.set(0, 0, -0.225); slab.receiveShadow = true; slab.castShadow = true; WA.root.add(slab);
  // safety net frame around the deck
  const net = new THREE.MeshStandardMaterial({ color: '#30353b', roughness: 0.6 });
  for (const [x0, x1, y0, y1] of [[-half - 1.5, half + 1.5, half, half + 1.5], [-half - 1.5, half + 1.5, -half - 1.5, -half], [half, half + 1.5, -half, half], [-half - 1.5, -half, -half, half]])
    box(WA, x0, x1, y0, y1, -0.5, -0.4, net, false);
  // hospital low wing behind the tower and city blocks around (away from the continue path)
  const wing = new THREE.MeshStandardMaterial({ map: winTex, color: '#e6e1d6', roughness: 0.75 });
  box(WA, -70, -half, -28, 28, -deck, -deck + 13, wing);
  // city blocks (illustrative), instanced; the corridor ahead of the deck stays clear, as in the model
  const cityFacade = canvasTex(256, 512, (x, w, h) => {
    x.fillStyle = '#ffffff'; x.fillRect(0, 0, w, h);
    for (let r = 0; r < 24; r++) for (let c = 0; c < 10; c++) { x.fillStyle = hash(r + 40, c) > 0.8 ? '#8094a6' : '#5d6f80'; x.fillRect(c * 25.6 + 5, r * 21.3 + 6, 15, 10); }
  });
  const R1 = rng(7), unit = new THREE.BoxGeometry(1, 1, 1);
  const tint = ['#c8c3b8', '#a7aeb5', '#d6cfc0', '#9aa39f', '#b9b2a6'];
  const lists = tint.map(() => []);
  for (let bx = -1300; bx <= 1300; bx += 90) for (let by = -1300; by <= 1300; by += 90) {
    if (Math.abs(by) < 130 && bx > -60) continue;
    if (Math.hypot(bx, by) < 200) continue;
    const nb = 1 + Math.floor(R1() * 3);
    for (let k = 0; k < nb; k++) {
      const sx = 16 + R1() * 26, sy = 16 + R1() * 26, x = bx + (R1() - 0.5) * (70 - sx), y = by + (R1() - 0.5) * (70 - sy);
      const r = Math.hypot(bx, by);
      const hz = 8 + Math.pow(R1(), 1.8) * (r < 450 ? 28 : r < 900 ? 55 : 80);
      lists[Math.floor(R1() * tint.length)].push([x, y, sx, sy, hz]);
    }
  }
  const m4 = new THREE.Matrix4(), qI = new THREE.Quaternion();
  lists.forEach((L, i) => {
    const im = new THREE.InstancedMesh(unit, new THREE.MeshStandardMaterial({ map: cityFacade, color: tint[i], roughness: 0.85 }), L.length);
    L.forEach(([x, y, sx, sy, hz], j) => { m4.compose(new THREE.Vector3(x, y, -deck + hz / 2), qI, new THREE.Vector3(sx, sy, hz)); im.setMatrixAt(j, m4); });
    im.receiveShadow = true; WA.root.add(im);
  });
  mountains(WA, 2500, -deck, 700, '#7f92a3', 1.7);
  // deck-level reference line beyond the edge (to read the descent below the deck)
  const ln = new THREE.Group();
  const dashMat = new THREE.MeshBasicMaterial({ color: 0xffe14d, transparent: true, opacity: 0.9, fog: false });
  for (let x0 = half + 1; x0 < 260; x0 += 5) { const m = new THREE.Mesh(new THREE.BoxGeometry(3, 0.25, 0.25), dashMat); m.position.set(x0 + 1.5, -6, 0); ln.add(m); }
  WA.root.add(ln); WA.deckLine = ln;
  const drop = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.3, 1), new THREE.MeshBasicMaterial({ color: 0xff5a4e, fog: false }));
  WA.root.add(drop); WA.dropMark = drop;
}
const heliA = makeHeli(DA.runs.reject.cg_h, DA.runs.reject.rotor_radius_m);
WA.root.add(heliA.group);

// ------------------------------------------------------------------------------------------------ world B: rescue site (flat clearing)
const WB = makeWorld({ skyTop: '#5c8fc6', skyHor: '#dfe9f1', fog: '#cfdbe4', fogNear: 300, fogFar: 3500, hemiSky: '#e8f1ff', hemiGround: '#4f5a40' });
{
  const grass = canvasTex(1024, 1024, (x, w, h) => {
    x.fillStyle = '#6f8b4c'; x.fillRect(0, 0, w, h);
    for (let i = 0; i < 26000; i++) { const v = hash(i, 11); x.fillStyle = v > 0.5 ? 'rgba(92,120,58,0.55)' : 'rgba(140,160,92,0.45)'; x.fillRect(hash(i, 12) * w, hash(i, 13) * h, 2 + v * 3, 2 + v * 3); }
  }, 120);
  groundPlane(WB, 6000, 0, grass);
  const clear = canvasTex(512, 512, (x, w, h) => {
    x.fillStyle = '#a9a184'; x.fillRect(0, 0, w, h);
    for (let i = 0; i < 6000; i++) { const v = 120 + hash(i, 21) * 70; x.fillStyle = `rgba(${v},${v * 0.95},${v * 0.75},0.5)`; x.fillRect(hash(i, 22) * w, hash(i, 23) * h, 3, 3); }
  });
  const c = new THREE.Mesh(new THREE.CircleGeometry(22, 64), new THREE.MeshStandardMaterial({ map: clear, roughness: 1 }));
  c.position.z = 0.03; c.receiveShadow = true; WB.root.add(c);
  // landing markers (cones) and a generic ground vehicle silhouette
  const cone = new THREE.MeshStandardMaterial({ color: '#ff7a1a', roughness: 0.6 });
  for (let k = 0; k < 8; k++) { const a = k / 8 * Math.PI * 2; const m = new THREE.Mesh(new THREE.ConeGeometry(0.25, 0.7, 16), cone); m.rotation.x = Math.PI / 2; m.position.set(14 * Math.cos(a), 14 * Math.sin(a), 0.35); m.castShadow = true; WB.root.add(m); }
  box(WB, -38, -32, 18, 20.4, 0, 2.6, new THREE.MeshStandardMaterial({ color: '#e8e8e4', roughness: 0.5 }));
  // trees well clear of the site (the site is modelled flat and without obstacles)
  const trunk = new THREE.MeshStandardMaterial({ color: '#5a4632', roughness: 1 });
  const leaf = [new THREE.MeshStandardMaterial({ color: '#3f5f2c', roughness: 1, flatShading: true }), new THREE.MeshStandardMaterial({ color: '#4d6e33', roughness: 1, flatShading: true })];
  const cg = new THREE.ConeGeometry(1, 2.2, 7), tg = new THREE.CylinderGeometry(0.15, 0.2, 1, 6);
  for (let i = 0; i < 900; i++) {
    const a = hash(i, 41) * Math.PI * 2, r = 120 + Math.pow(hash(i, 42), 0.6) * 650;
    const x = r * Math.cos(a), y = r * Math.sin(a);
    if (Math.abs(y) < 60 && x > 0 && x < 700) continue; // the continue path stays clear
    const s = 4 + hash(i, 43) * 5;
    const t = new THREE.Mesh(tg, trunk); t.scale.set(s, s * 0.6, s); t.rotation.x = Math.PI / 2; t.position.set(x, y, s * 0.3); WB.root.add(t);
    const l = new THREE.Mesh(cg, leaf[i % 2]); l.scale.set(s * 0.9, s * 1.4, s * 0.9); l.rotation.x = Math.PI / 2; l.position.set(x, y, s * 0.6 + s * 1.4); l.castShadow = true; WB.root.add(l);
  }
  mountains(WB, 2600, 0, 600, '#73879a', 4.1);
}
const heliB = makeHeli(DB.runs.procedimento.cg_h, DB.runs.procedimento.rotor_radius_m);
WB.root.add(heliB.group);

// ------------------------------------------------------------------------------------------------ world C: open plain, two helicopters
const WC = makeWorld({ skyTop: '#557fb3', skyHor: '#dce6ee', fog: '#cad7e1', fogNear: 900, fogFar: 6000, hemiSky: '#e5eefc', hemiGround: '#5a5f45' });
const C_OFFSET_Y = 900; // the vertical case is flown 900 m to the side, so each view shows one aircraft
{
  const farm = canvasTex(1024, 1024, (x, w, h) => {
    const cols = ['#7d955a', '#a39a63', '#6c8a4c', '#93a46a', '#b3a874', '#5f7d43'];
    for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) { x.fillStyle = cols[Math.floor(hash(i, j) * cols.length)]; x.fillRect(i * 128, j * 128, 128, 128); }
    for (let i = 0; i < 20000; i++) { x.fillStyle = 'rgba(40,50,25,0.12)'; x.fillRect(hash(i, 51) * w, hash(i, 52) * h, 2, 2); }
    x.strokeStyle = 'rgba(70,60,40,0.55)'; x.lineWidth = 3; for (let i = 0; i <= 8; i++) { x.beginPath(); x.moveTo(i * 128, 0); x.lineTo(i * 128, h); x.stroke(); x.beginPath(); x.moveTo(0, i * 128); x.lineTo(w, i * 128); x.stroke(); }
  }, 12);
  groundPlane(WC, 14000, 0, farm);
  mountains(WC, 5200, 0, 700, '#7e90a1', 2.3);
  const trunk = new THREE.MeshStandardMaterial({ color: '#4d6e33', roughness: 1, flatShading: true });
  const g = new THREE.SphereGeometry(1, 7, 5);
  for (let i = 0; i < 1500; i++) { const x = -800 + hash(i, 61) * 3200, y = -800 + hash(i, 62) * 2600; const s = 3 + hash(i, 63) * 3; if (Math.abs(y) < 40 || Math.abs(y - C_OFFSET_Y) < 40) continue; const m = new THREE.Mesh(g, trunk); m.scale.set(s, s, s * 1.1); m.position.set(x, y, s); WC.root.add(m); }
}
const heliCV = makeHeli(DC.runs.vertical.cg_h, DC.runs.vertical.rotor_radius_m);
const heliCF = makeHeli(DC.runs.forward.cg_h, DC.runs.forward.rotor_radius_m);
WC.root.add(heliCV.group); WC.root.add(heliCF.group);

// ------------------------------------------------------------------------------------------------ cameras
const camA = new THREE.PerspectiveCamera(38, W / H, 0.5, 9000);
const camL = new THREE.PerspectiveCamera(42, (W / 2) / H, 0.5, 12000);
const camR = new THREE.PerspectiveCamera(42, (W / 2) / H, 0.5, 12000);
// place `cam` looking along -dir so that points A and B (ENU) project centred at NDC x = cx, spanning `span` in NDC
function frame2(cam, A, B, cx, span, dir) {
  dir = dir.clone().normalize();
  const M = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2, (A[2] + B[2]) / 2];
  let D = Math.max(30, Math.hypot(B[0] - A[0], B[1] - A[1], B[2] - A[2])) * 1.4, tgt = enu(...M);
  const pa = new THREE.Vector3(), pb = new THREE.Vector3();
  for (let it = 0; it < 6; it++) {
    cam.position.copy(enu(M[0] + dir.x * D, M[1] + dir.y * D, M[2] + dir.z * D));
    cam.up.set(0, 1, 0); cam.lookAt(tgt); cam.updateMatrixWorld(); cam.updateProjectionMatrix();
    pa.copy(enu(...A)).project(cam); pb.copy(enu(...B)).project(cam);
    const s = Math.abs(pb.x - pa.x), mid = (pa.x + pb.x) / 2;
    D *= s / span;
    // shift the target sideways (camera right vector) so the midpoint lands at cx
    const right = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0);
    const halfW = Math.tan(THREE.MathUtils.degToRad(cam.fov / 2)) * cam.aspect * cam.position.distanceTo(tgt);
    tgt.addScaledVector(right, (mid - cx) * halfW);
  }
  cam.position.copy(tgt).add(enu(dir.x * D, dir.y * D, dir.z * D).sub(enu(0, 0, 0)));
  cam.lookAt(tgt);
}
function look(cam, pos, at) { cam.position.copy(enu(...pos)); cam.up.set(0, 1, 0); cam.lookAt(enu(...at)); }

// ------------------------------------------------------------------------------------------------ HUD helpers
const $ = (id) => document.getElementById(id);
const tqBars = $('tqBars'); const bars = [];
for (let i = 0; i < 2; i++) {
  const d = document.createElement('div'); d.className = 'bar';
  d.innerHTML = `<span class="lbl">M${i + 1}</span><span class="track"><span class="fill"></span><span class="lim"></span></span><span class="val"></span>`;
  tqBars.appendChild(d); bars.push({ fill: d.querySelector('.fill'), lim: d.querySelector('.lim'), val: d.querySelector('.val') });
}
const TQ_SCALE = 140; // % at the right end of the bar
const firstEvent = (run, pred) => run.events.find(pred) || null;
function detectT(run) { const e = firstEvent(run, e => e.level === 3 && (e.code === 'engine_failure' || e.code === 'dual_engine_failure')); return e ? e.t : null; }
function sadpfEvent(run) { return firstEvent(run, e => e.level === 3 && (e.code === 'engine_failure' || e.code === 'dual_engine_failure')); }
function phaseT(run, name) { const p = (run.phases || []).find(p => p.phase === name); return p ? p.t : null; }

function setTelemetry(run, d, opts = {}) {
  const lim = run.torque_limits_pct;
  const code = ['TO', 'OEI30', 'OEI2', 'OEIC'][Math.round(d.rating)];
  if (opts.lims) setNR(opts.lims, run, d, d.t);
  const failed = run.fail_time !== null && run.fail_time !== undefined && d.t >= run.fail_time;
  for (let i = 0; i < 2; i++) {
    const v = d[`tq${i + 1}_pct`];
    bars[i].fill.style.width = `${Math.max(0, Math.min(100, 100 * v / TQ_SCALE))}%`;
    const isFailed = (opts.dual && failed) || (failed && i === 0);
    bars[i].fill.style.background = isFailed ? 'var(--bad)' : (v > lim[code] * 0.98 ? 'var(--warn)' : 'var(--ok)');
    bars[i].lim.style.left = `${Math.min(100, 100 * lim[code] / TQ_SCALE)}%`;
    bars[i].val.textContent = `${fmt(v, 0)} %`;
  }
  $('rating').textContent = `${['Decolagem (AEO)', 'OEI 30 s', 'OEI 2 min', 'OEI contínuo'][Math.round(d.rating)]} · lim. ${fmt(lim[code], 0)} %`;
  if (failed && !opts.dual) {
    const ts = d.t - run.fail_time;
    $('oei').textContent = ts < 30 ? `OEI 30 s · restam ${fmt(30 - ts, 1)} s` : ts < 150 ? `OEI 2 min · restam ${fmt(150 - ts, 0)} s` : 'OEI contínuo';
    $('oei').className = 'warnc';
  } else { $('oei').textContent = opts.dual && failed ? 'dois motores parados' : 'parado (AEO)'; $('oei').className = opts.dual && failed ? 'badc' : ''; }
  $('ias').textContent = `${fmt(d.ias_kt, 0)} kt`;
  $('vz').textContent = `${d.vz >= 0 ? '+' : ''}${fmt(d.vz, 1)} m/s (${fmt(d.vz / 0.3048 * 60, 0)} ft/min)`;
  if (opts.deck) {
    const over = Math.abs(d.x) <= DA.condition.deck_size_m / 2;
    $('hLbl').textContent = over ? 'Altura dos esquis sobre o deck' : 'Esquis: deck / rua';
    $('hgt').textContent = over ? `${fmt(d.skid_h, 1)} m` : `${fmt(d.skid_h, 1)} m / ${fmt(d.agl_skid, 1)} m`;
  } else { $('hLbl').textContent = 'Altura dos esquis'; $('hgt').textContent = `${fmt(d.agl_skid, 1)} m`; }
  const mg = d.margin_kw; $('marg').textContent = `${mg >= 0 ? '+' : ''}${fmt(mg, 0)} kW`; $('marg').className = mg < 0 ? 'badc' : mg < 30 ? 'warnc' : 'okc';
  $('vrs').textContent = d.vrs > 0.5 ? 'ALERTA' : 'fora da faixa'; $('vrs').className = d.vrs > 0.5 ? 'badc' : '';
  $('mass').textContent = `${fmt(d.mass_kg, 0)} kg`;
  $('cg').textContent = opts.cg ? `${fmt(opts.cg.sta_mm, 0)} mm (folga ${fmt(opts.cg.margin_fwd_mm, 0)})` : '—';
  $('fuel').textContent = `${fmt(d.fuel_kg, 1)} kg (${fmt(d.ff_kgh, 0)} kg/h)`;
  $('endu').textContent = d.endurance_min > 600 ? '—' : `${fmt(d.endurance_min, 0)} min`;
}
function setSadpf(run, t, extraLine = '') {
  const ev = sadpfEvent(run);
  const box = $('sadpf');
  if (!ev || t < ev.t) {
    box.className = 'box';
    $('sadpfL').textContent = 'NOMINAL';
    $('sadpfM').innerHTML = 'Torques dos dois motores equilibrados · NR no governador';
    return false;
  }
  box.className = 'box crit';
  $('sadpfL').textContent = 'NÍVEL 3 · CRÍTICO';
  const delay = run.fail_time !== null && run.fail_time !== undefined ? ev.t - run.fail_time : null;
  const msg = ev.message.replace(/\. (ABORTAR|PROSSEGUIR|AUTORROTAÇÃO)/, '.<br><b>$1').replace(/$/, '</b>');
  $('sadpfM').innerHTML = `${msg}${delay !== null ? `<br>Detecção ${fmt(delay, 2)} s após a falha` : ''}${extraLine}`;
  return true;
}
function setAdvisory(run, t) {
  const a = run.advisory, box = $('advisory');
  if (!a || t < a.t) { box.style.display = 'none'; return; }
  box.style.display = 'block';
  box.className = a.alert ? 'box alert' : 'box';
  const P = a.predicted, proc = a.procedure_action;
  const set = (id, k) => {
    const el = $(id); const ok = P[k].safe;
    el.className = `br ${ok ? 'safe' : 'unsafe'}${k === proc ? ' proc' : ''}`;
    el.querySelector('.bv').textContent = ok ? 'previsto seguro' : 'previsto inseguro';
  };
  set('advRej', 'abortar'); set('advCon', 'prosseguir');
  const up = (s) => s.toUpperCase();
  if (a.alert) {
    $('advL').textContent = `ALERTA CONSULTIVO · considerar ${up(a.advise)}`;
    $('advM').innerHTML = `O procedimento (TDP) indica <b>${up(proc)}</b>, previsto inseguro (${P[proc].reason.replace('tocou o heliponto ou o solo', 'toque no solo durante a aceleração')}). ${up(a.advise)} previsto seguro. O procedimento continua sendo o padrão.`;
  } else {
    $('advL').textContent = `Sem alerta · ${up(proc)} previsto ${P[proc].safe ? 'seguro' : 'inseguro'}`;
    $('advM').innerHTML = '';
  }
  const each = a.wall_time_each_s || [];
  $('advT').innerHTML = `Tempo de cálculo <b>${fmt(a.wall_time_s, 2)} s</b> · limite 1 s (reação do piloto) · ${a.mode === 'paralelo' ? 'ramos em paralelo' : 'ramos em sequência'} (abortar ${fmt(each[0], 2)} s, prosseguir ${fmt(each[1], 2)} s)<br>Medido em ambiente de desenvolvimento; não representa hardware embarcado.`;
}


// ---- NR limits (TCDS) and touchdown outcome (classified by the Twin, see procedures.classify_touchdown)
function nrLimitsAt(lims, d) { return d.n_eng >= 1 ? { lo: lims.power_on[0], hi: lims.power_on[1], regime: 'com motor' } : { lo: lims.power_off[0], hi: lims.power_off[1], regime: 'sem motor' }; }
function nrEpisodes(run, t) {
  return (run.nr_exceedances || []).filter(e => e.t0 <= t + 1e-6).map(e => {
    const ongoing = t <= e.t1 + 0.05, dur = Math.min(t, e.t1) - e.t0 + 0.1;
    return `<div class="ep ${ongoing ? '' : 'past'}">${ongoing ? '▲ ' : ''}NR ${e.kind} do limite ${e.regime} (${fmt(e.limit, 0)} %)${e.on_ground ? ', no solo' : ''}: ${e.kind === 'acima' ? 'máx.' : 'mín.'} ${fmt(e.peak, 1)} % · ${fmt(dur, 1)} s</div>`;
  }).join('');
}
function setNR(lims, run, d, t) {
  const L = nrLimitsAt(lims, d), el = $('nr');
  const out = d.nr_pct > L.hi || (d.nr_pct < L.lo && d.on_ground < 0.5);
  el.textContent = `${fmt(d.nr_pct, 1)} %${out ? (d.nr_pct > L.hi ? ' ▲ ACIMA DO LIMITE' : ' ▼ ABAIXO DO LIMITE') : ''}`;
  el.classList.toggle('nrbad', out);
  $('nrLimL').textContent = `Limites de NR ${L.regime} (TCDS)`;
  $('nrLim').textContent = `${fmt(L.lo, 0)}–${fmt(L.hi, 0)} %`;
  $('nrEx').innerHTML = nrEpisodes(run, t);
}
function showTouchdown(box, landing, t, thr, extra = '') {
  if (!landing || !landing.landed || t < landing.touchdown_t || !landing.outcome) { box.style.display = 'none'; return null; }
  const o = landing.outcome;
  box.className = `hud tdbox lv${o.level}${box.classList.contains('half') ? ' half' : ''}`;
  box.style.display = 'block';
  box.querySelector('.tdl').textContent = o.label;
  box.querySelector('.tdm').innerHTML = `Toque a <b>${fmt(o.sink_ms, 1)} m/s</b> na vertical${extra}. ${o.why.charAt(0).toUpperCase() + o.why.slice(1)}.<br><span style="opacity:.85">Faixas: ≤ ${fmt(thr.limit_29_725_ms, 1)} pouso · ≤ ${fmt(thr.reserve_29_727_ms, 2)} pouso duro · ≤ ${fmt(thr.seat_29_562_ms, 1)} m/s dano provável · acima: impacto (ESTIMADO)</span>`;
  return o;
}
function tint(on, x0 = 0, w = W) { const e = $('impactTint'); e.style.display = on ? 'block' : 'none'; e.style.left = `${x0}px`; e.style.width = `${w}px`; }

// H-V mini diagram (points from the Twin's H-V sweep) with the current position
function drawHV(hv, d, hRel) {
  const c = $('hv'), x = c.getContext('2d'), w = c.width, h = c.height;
  const pad = { l: 36, r: 8, t: 8, b: 24 }, vMax = 60, hMax = 32;
  const X = (v) => pad.l + (w - pad.l - pad.r) * v / vMax, Y = (z) => h - pad.b - (h - pad.t - pad.b) * z / hMax;
  x.clearRect(0, 0, w, h);
  x.strokeStyle = 'rgba(255,255,255,0.25)'; x.lineWidth = 1; x.font = '12px DejaVu Sans, sans-serif'; x.fillStyle = 'rgba(255,255,255,0.75)';
  for (let v = 0; v <= vMax; v += 20) { x.beginPath(); x.moveTo(X(v), Y(0)); x.lineTo(X(v), Y(hMax)); x.stroke(); if (v < vMax) x.fillText(`${v}`, X(v) - 6, h - 7); }
  for (let z = 0; z <= hMax; z += 10) { x.beginPath(); x.moveTo(X(0), Y(z)); x.lineTo(X(vMax), Y(z)); x.stroke(); x.fillText(`${z}`, 8, Y(z) + 4); }
  x.fillText('kt', w - 20, Y(0) - 6); x.fillText('m', X(0) + 6, 16);
  for (const p of hv.points) {
    if (p.v > vMax || p.h > hMax) continue;
    x.fillStyle = p.safe ? 'rgba(127,209,139,0.55)' : 'rgba(255,90,78,0.95)';
    x.beginPath(); x.arc(X(p.v), Y(p.h), p.safe ? 3 : 5, 0, Math.PI * 2); x.fill();
  }
  if (d) {
    const v = Math.max(0, Math.min(vMax, d.ias_kt)), z = Math.max(0, Math.min(hMax, hRel));
    x.strokeStyle = '#fff'; x.lineWidth = 2.5; x.beginPath(); x.arc(X(v), Y(z), 8, 0, Math.PI * 2); x.stroke();
    x.fillStyle = '#ffffff'; x.beginPath(); x.arc(X(v), Y(z), 3, 0, Math.PI * 2); x.fill();
  }
}

// ------------------------------------------------------------------------------------------------ scenes
const smooth = (u) => u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u);
const SCENES = {
  cat_a: { label: '(a) Categoria A', runs: { reject: 'Abortar', continue: 'Prosseguir' }, tEnd: (r) => DA.runs[r].tEnd },
  advisory: { label: '(b) Previsão de ramos', runs: { procedimento: 'Procedimento (padrão)', consultivo_seguido: 'Seguindo o alerta' }, tEnd: (r) => DB.runs[r].tEnd },
  autorotation: { label: '(c) Autorrotação', runs: { both: 'Vertical × à frente' }, tEnd: () => Math.max(DC.runs.vertical.tEnd, DC.runs.forward.tEnd) },
  missao: { label: '(d) Missão', runs: { resgate: 'Resgate', transferencia: 'Transferência', conservador: 'Conservador' }, tEnd: () => 0 },
};
function showCols(left, right, split) {
  $('left').style.display = left ? 'flex' : 'none';
  $('right').style.display = right ? 'flex' : 'none';
  $('split').style.display = split ? 'block' : 'none';
  $('flareNote').style.display = split ? 'block' : 'none';
}
function clock(t) { const mm = Math.floor(t / 60), ss = t - 60 * mm; $('clock').textContent = `T+${String(mm).padStart(2, '0')}:${ss.toFixed(1).padStart(4, '0')}`; }
function caption(s) { $('capText').textContent = s; $('caption').style.display = s ? '' : 'none'; $('caption').classList.toggle('mid', $('left').style.display !== 'none'); }

// key instants of a Category A run, taken from the telemetry
function catATimeline(run, vtKt) {
  const f = run.frames, n = run._n, half = DA.condition.deck_size_m / 2, ft = run.fail_time ?? Infinity;
  const tl = { decision: null, edge: null, below: null, min: null, minH: Infinity, vtoss: null, landed: phaseT(run, 'landed') };
  const dec = (run.phases || []).find(p => p.phase === 'reject' || p.phase === 'continue'); tl.decision = dec ? dec.t : null;
  for (let i = 0; i < n; i++) {
    if (f.t[i] < ft) continue;
    if (tl.edge === null && f.x[i] > half) tl.edge = f.t[i];
    if (tl.below === null && f.skid_h[i] < 0) tl.below = f.t[i];
    if (f.skid_h[i] < tl.minH) { tl.minH = f.skid_h[i]; tl.min = f.t[i]; }
    if (tl.vtoss === null && vtKt && f.tas_ms[i] >= 0.95 * vtKt * 0.514444) tl.vtoss = f.t[i];
  }
  tl.done = run.name === 'reject' ? (tl.landed !== null ? tl.landed + 1.0 : run.tEnd) : Math.max(tl.min ?? 0, tl.vtoss ?? 0) + 1.5;
  return tl;
}
const VT = DA.runs.continue.evaluation.vtoss_kt;
for (const r of Object.values(DA.runs)) r._tl = catATimeline(r, VT);

function renderCatA(runName, t) {
  const run = DA.runs[runName], d = sample(run, t), det = detectT(run), tl = run._tl;
  showCols(true, true, false);
  $('tlM').textContent = DA.title;
  $('tlC').textContent = `${DA.condition.label} · deck a ${fmt(DA.condition.deck_height_m, 0)} m da rua · ramo: ${runName === 'reject' ? 'abortar' : 'prosseguir'}`;
  $('sadpf').style.display = ''; $('cata').style.display = ''; $('mission').style.display = 'none'; $('hvBox').style.display = '';
  setSadpf(run, t);
  setAdvisory(run, t);
  setTelemetry(run, d, { deck: true, cg: DA.cg, lims: DA.nr_limits });
  { const o = showTouchdown($('tdBox'), run.landing, t, DA.touchdown_thresholds); tint(o && o.level === 3); $('tdBoxL').style.display = 'none'; $('tdBoxR').style.display = 'none'; }
  // Category A panel
  const M = DA.cat_a_mass, ev = run.evaluation, lit = run.literal_29_59c;
  $('m2960').innerHTML = `máx. <b>${fmt(M.g115_kg, 0)}–${fmt(M.g126_kg, 0)} kg</b><br><span style="font-size:13px;opacity:.75">faixa pelo duto G 1,15–1,26</span>`;
  $('m2959').innerHTML = `máx. <b>${fmt(M.literal_29_59c_kg, 0)} kg</b><br><span style="font-size:13px;opacity:.75">nunca abaixo de 15 ft do deck</span>`;
  $('cMass').textContent = `${fmt(DA.mass_kg, 0)} kg (máx. com G = 1,26)`;
  $('cTdp').textContent = `${fmt(DA.condition.tdp_height_m, 0)} m sobre o deck · ${fmt(VT, 0)} kt`;
  $('cBranch').textContent = tl.decision !== null && t >= tl.decision ? (runName === 'reject' ? 'ABORTAR (falha antes do TDP)' : 'PROSSEGUIR (falha após o TDP)') : (det !== null && t >= det ? 'reação do piloto (1 s)' : '—');
  let maxDrop = 0; for (let i = 0; i <= d.idx; i++) if (run.frames.t[i] >= (run.fail_time ?? 1e9)) maxDrop = Math.max(maxDrop, -run.frames.skid_h[i]);
  $('cDrop').innerHTML = runName === 'continue' ? `agora ${fmt(Math.max(0, -d.skid_h), 1)} · máx. ${fmt(Math.max(0, maxDrop), 1)} m` : 'não se aplica';
  $('cM30').textContent = `${ev.oei30_margin_hover_kw >= 0 ? '+' : ''}${fmt(ev.oei30_margin_hover_kw, 0)} kW`;
  $('cM30').className = ev.oei30_margin_hover_kw < 0 ? 'warnc' : 'okc';
  $('cM2').textContent = ev.oei2_margin_vtoss_kw !== undefined ? `${ev.oei2_margin_vtoss_kw >= 0 ? '+' : ''}${fmt(ev.oei2_margin_vtoss_kw, 0)} kW` : '—';
  const done = t >= tl.done;
  const res = (e) => done ? (e.safe ? `<span class="okc">✓ atende</span>` : `<span class="badc">✗ ${e.reason.replace(/^desceu a -?([\d.]+) m do nível do deck.*$/, (m, v) => `desceu ${fmt(+v, 1)} m abaixo do deck`)}</span>`) : '<span style="opacity:.7">em avaliação</span>';
  $('r2960').innerHTML = res(ev); $('r2959').innerHTML = res(lit);
  drawHV(DA.hv, d, Math.max(0, d.skid_h));
  $('hvT').textContent = 'DIAGRAMA ALTURA-VELOCIDADE (H-V)';
  $('hvN').textContent = `Pontos do Twin: ${DA.hv.condition}. Vermelho = falha de motor sem pouso seguro. Marcador: IAS e altura sobre o deck.`;
  // 3D
  poseHeli(heliA, d, t, det, det, Math.floor(t * 4) % 2 === 0);
  WA.deckLine.visible = runName === 'continue';
  const skidZ = d.z - run.cg_h, below = runName === 'continue' && skidZ < 0;
  WA.dropMark.visible = below;
  if (below) { WA.dropMark.scale.set(1, 1, -skidZ); WA.dropMark.position.set(d.x, d.y - 6, skidZ / 2); }
  followSun(WA, d.x, d.y, d.z, 45);
  if (runName === 'continue' && tl.edge !== null && t > tl.edge - 1.5) {
    // two-point framing: the deck edge and the aircraft inside the window between the HUD columns
    frame2(camA, [DA.condition.deck_size_m / 2, -DA.condition.deck_size_m / 2, 0], [d.x, d.y, d.z], 0.07, 0.62, new THREE.Vector3(0.3, -1, 0.06));
  } else {
    look(camA, [d.x - 20, -44, Math.max(d.z, 0) + 10], [d.x + 4, 0, Math.max(d.z, 0) * 0.6 - 4]);
  }
  renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H);
  renderer.render(WA.scene, camA);
  // caption from the timeline
  const ft = run.fail_time;
  let cap = '';
  if (ft === null || t < ft) cap = t < 3 ? `Decolagem vertical de heliponto elevado · ${fmt(DA.mass_kg, 0)} kg, na massa máxima Cat A` : `Subida vertical com os dois motores até o TDP (${fmt(DA.condition.tdp_height_m, 0)} m acima do deck)`;
  else if (det === null || t < det) cap = `Falha do motor 1 com os esquis a ${fmt(ev.fail_skid_h_m, 1)} m do deck (${ev.fail_skid_h_m < DA.condition.tdp_height_m ? 'antes' : 'depois'} do TDP)`;
  else if (tl.decision === null || t < tl.decision + 1.2) cap = `SADPF detecta a falha em ${fmt(det - ft, 2)} s pela divergência de torque → ${runName === 'reject' ? 'ABORTAR' : 'PROSSEGUIR'}`;
  else if (runName === 'reject') cap = tl.landed !== null && t >= tl.landed ? `Pouso no deck a ${fmt(ev.touchdown_sink_ms, 1)} m/s · atende ao critério do modelo (29.60) e ao literal 29.59(c)` : 'Abortar: descida de volta ao deck com potência OEI 30 s';
  else if (tl.edge === null || t < tl.edge) cap = 'Prosseguir: nariz para baixo, acelera com um motor sobre o deck';
  else if (t < tl.min + 1.0) cap = `Cruza a borda e desce abaixo do nível do deck: até ${fmt(-tl.minH, 1)} m (29.60(a)(3) exige determinar essa profundidade)`;
  else cap = `VTOSS (${fmt(VT, 0)} kt) e subida com um motor · 29.60 (modelo): ${ev.safe ? 'atende' : 'não atende'} · 29.59(c) literal: ${lit.safe ? 'atende' : 'não atende'}`;
  caption(cap);
  clock(t);
}

function renderAdvisory(runName, t) {
  const run = DB.runs[runName], d = sample(run, t), det = detectT(run);
  showCols(true, true, false);
  $('tlM').textContent = DB.title;
  $('tlC').textContent = `${DB.condition.label} · ${fmt(DB.mass_kg, 0)} kg, acima do limite do local (${fmt(DB.site_limit_kg, 0)} kg): caso de demonstração`;
  $('sadpf').style.display = ''; $('cata').style.display = 'none'; $('mission').style.display = 'none'; $('hvBox').style.display = '';
  setSadpf(run, t);
  setAdvisory(run, t);
  setTelemetry(run, d, { cg: DB.cg, lims: DB.nr_limits });
  { const o = showTouchdown($('tdBox'), run.landing, t, DB.touchdown_thresholds, run.name === 'procedimento' ? ' (contato durante a aceleração do prosseguir)' : ''); tint(o && o.level === 3); $('tdBoxL').style.display = 'none'; $('tdBoxR').style.display = 'none'; }
  drawHV(DA.hv, d, Math.max(0, d.agl_skid));
  $('hvT').textContent = 'DIAGRAMA ALTURA-VELOCIDADE (H-V)';
  $('hvN').textContent = `Pontos do Twin: ${DA.hv.condition}. Marcador: IAS e altura sobre o solo.`;
  poseHeli(heliB, d, t, det, det, Math.floor(t * 4) % 2 === 0);
  followSun(WB, d.x, d.y, d.z, 40);
  look(camA, [d.x * 0.8 - 24, -40, Math.max(d.z, 0) * 0.5 + 8], [d.x * 0.9 + 4, 0, Math.max(d.z, 0) * 0.6 + 1]);
  renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H);
  renderer.render(WB.scene, camA);
  const a = run.advisory, ft = run.fail_time, lt = phaseT(run, 'landed');
  let cap;
  if (ft === null || t < ft) cap = t < 4 ? 'Decolagem do local de resgate com o paciente a bordo' : `Subida vertical até o TDP (${fmt(DB.condition.tdp_height_m, 0)} m)`;
  else if (det === null || t < det) cap = `Falha do motor 1 com os esquis a ${fmt(run.evaluation.fail_skid_h_m, 1)} m, logo acima do TDP`;
  else if (a && t < a.t + 3.0) cap = a.alert ? `Previsão em ${fmt(a.wall_time_s, 2)} s: o ramo do procedimento (prosseguir) é previsto inseguro → alerta consultivo` : `Previsão de ramos em ${fmt(a.wall_time_s, 2)} s: procedimento confirmado`;
  else if (runName === 'procedimento') cap = run.evaluation.touched_down && d.on_ground > 0.5 ? `Procedimento seguido (padrão): ${run.evaluation.reason} durante a aceleração — a previsão estava certa` : 'Procedimento (padrão): prosseguir, nariz para baixo e aceleração com um motor';
  else cap = lt !== null && t >= lt ? `Seguindo o alerta: abortar, pouso no local a ${fmt(run.evaluation.touchdown_sink_ms, 1)} m/s · seguro pelo critério do modelo` : 'Seguindo o alerta consultivo: abortar, descida de volta ao local';
  caption(cap);
  clock(t);
}

function autoPanel(el, run, d, t) {
  const L = run.landing, det = detectT(run);
  const landed = L.landed && t >= L.touchdown_t;
  const goal = DC.goal, lim = nrLimitsAt(DC.nr_limits, d);
  const nrOut = d.nr_pct > lim.hi || (d.nr_pct < lim.lo && d.on_ground < 0.5);
  let h = `<div class="row"><span>Altura</span><b>${fmt(Math.max(0, d.agl_skid), 0)} m</b></div>
    <div class="row"><span>Razão de descida</span><b class="big">${fmt(Math.max(0, -d.vz), 1)} m/s</b></div>
    <div class="row"><span>Velocidade horizontal</span><b>${fmt(Math.hypot(d.vx, d.vy) / 0.514444, 0)} kt</b></div>
    <div class="row"><span>NR (rotor) · limites ${lim.regime} ${fmt(lim.lo, 0)}–${fmt(lim.hi, 0)} %</span><b class="${nrOut ? 'nrbad' : ''}">${fmt(d.nr_pct, 1)} %${nrOut ? (d.nr_pct > lim.hi ? ' ▲' : ' ▼') : ''}</b></div>
    <div class="nrex">${nrEpisodes(run, t)}</div>
    <div class="row"><span>Razão em regime: Twin · teoria</span><b>${fmt(run.rod_steady_ms, 1)} · ${fmt(run.rod_predicted_ms, 1)} m/s</b></div>`;
  if (det !== null && t >= det) h += `<div class="row"><span>SADPF</span><b class="badc">falha dupla detectada em ${fmt(run.sadpf.detection_delay_s, 2)} s</b></div>`;
  if (landed) {
    const gs = L.touchdown_ground_speed_ms / 0.514444, o = L.outcome;
    const ok = o.level === 0 && gs <= goal.ground_speed_kt;
    h += `<div class="result ${ok ? 'good' : 'bad'}">Toque: <b>${fmt(L.touchdown_sink_ms, 1)} m/s</b> na vertical, <b>${fmt(gs, 0)} kt</b> no solo → <b>${o.label}</b>${o.level === 0 && gs > goal.ground_speed_kt ? `<br>velocidade no solo acima da meta de ${fmt(goal.ground_speed_kt, 0)} kt (ESTIMADA): limitação do flare` : ''}</div>`;
  }
  el.innerHTML = h;
}
function renderAuto(t) {
  const V = DC.runs.vertical, F = DC.runs.forward;
  const dv = sample(V, t), df = sample(F, t);
  showCols(false, false, true);
  $('tlM').textContent = DC.title;
  $('tlC').textContent = `${DC.condition.label} · ${fmt(dv.mass_kg, 0)} kg · esquerda: a partir do pairado · direita: a ${fmt(F.v_glide_kt, 0)} kt (mínima razão de descida)`;
  autoPanel($('hLb'), V, dv, t);
  autoPanel($('hRb'), F, df, t);
  $('flareNote').textContent = DC.flare_limitation;
  $('tdBox').style.display = 'none';
  const oV = showTouchdown($('tdBoxL'), V.landing, t, DC.touchdown_thresholds);
  const oF = showTouchdown($('tdBoxR'), F.landing, t, DC.touchdown_thresholds);
  if (oV && oV.level === 3) tint(true, 0, W / 2); else if (oF && oF.level === 3) tint(true, W / 2, W / 2); else tint(false);
  const detV = detectT(V), detF = detectT(F);
  poseHeli(heliCV, { ...dv, y: dv.y + C_OFFSET_Y }, t, detV, detV, Math.floor(t * 4) % 2 === 0);
  poseHeli(heliCF, df, t, detF, detF, Math.floor(t * 4) % 2 === 0);
  renderer.setScissorTest(true);
  const hv = Math.max(dv.z, 2), hf = Math.max(df.z, 2);
  followSun(WC, dv.x, dv.y + C_OFFSET_Y, dv.z, 40);
  look(camL, [dv.x + 30, dv.y + C_OFFSET_Y - 30, hv + 6], [dv.x, dv.y + C_OFFSET_Y, hv + 5]);
  renderer.setViewport(0, 0, W / 2, H); renderer.setScissor(0, 0, W / 2, H); renderer.render(WC.scene, camL);
  followSun(WC, df.x, df.y, df.z, 40);
  look(camR, [df.x + 12, df.y - 38, hf + 6], [df.x - 4, df.y, hf + 5]);
  renderer.setViewport(W / 2, 0, W / 2, H); renderer.setScissor(W / 2, 0, W / 2, H); renderer.render(WC.scene, camR);
  renderer.setScissorTest(false);
  let cap;
  if (t < 2.0) cap = `Pairado (esquerda) e voo a ${fmt(F.v_glide_kt, 0)} kt (direita), a 300 m`;
  else if (t < Math.max(detV ?? 0, detF ?? 0) + 2) cap = 'Falha dos dois motores aos 2 s · SADPF: AUTORROTAÇÃO';
  else if (!(V.landing.landed && t >= V.landing.touchdown_t)) cap = `Em regime: ${fmt(V.rod_steady_ms, 1)} m/s na vertical × ${fmt(F.rod_steady_ms, 1)} m/s com velocidade à frente`;
  else if (!(F.landing.landed && t >= F.landing.touchdown_t)) cap = `Autorrotação vertical: ${V.landing.outcome.label.toLowerCase()} a ${fmt(V.landing.touchdown_sink_ms, 1)} m/s · a com velocidade à frente segue planando`;
  else cap = `Vertical: ${V.landing.outcome.label.toLowerCase()} (${fmt(V.landing.touchdown_sink_ms, 1)} m/s) × à frente: ${F.landing.outcome.label.toLowerCase()} (${fmt(F.landing.touchdown_sink_ms, 1)} m/s), a ${fmt(F.landing.touchdown_ground_speed_ms / 0.514444, 0)} kt no solo (limitação do flare)`;
  caption(cap);
  clock(t);
}

let missionCond = 3; // 1.500 m, ISA+25 (the binding case) by default
function renderMission(profile) {
  showCols(true, true, false);
  $('tlM').textContent = 'Painel de missão'; $('tlC').textContent = 'Configuração UTI padrão · tanque padrão (560 kg) · valores calculados pelo Twin';
  for (const id of ['sadpf', 'advisory', 'cata', 'tdBox', 'tdBoxL', 'tdBoxR']) $(id).style.display = 'none';
  tint(false);
  $('mission').style.display = 'block'; $('hvBox').style.display = 'none';
  const rows = DM.conditions, c = rows[missionCond], p = c.profiles[profile];
  $('profSel').innerHTML = Object.entries(DM.profiles).map(([k, v]) => `<button data-p="${k}" class="${k === profile ? 'on' : ''}">${v.split(' (')[0]}</button>`).join('');
  $('condSel').innerHTML = rows.map((r, i) => `<button data-c="${i}" class="${i === missionCond ? 'on' : ''}">${r.condition}</button>`).join('');
  let h = `<div class="row" style="opacity:.8"><span>${DM.profiles[profile]}</span></div>
    <div class="row"><span>Massa máx. Cat A na origem (G 1,15–1,26)</span><b>${fmt(c.cat_a_mass_g115_kg, 0)}–${fmt(c.cat_a_mass_kg, 0)} kg</b></div>
    <div class="row"><span>Combustível embarcado</span><b>${fmt(p.fuel_kg, 0)} kg · limite: ${p.fuel_limited_by}</b></div>
    <div class="row"><span>Massa de decolagem na origem</span><b>${fmt(p.takeoff_mass_kg, 0)} kg</b></div>
    <div class="row"><span>Raio de ação</span><b style="font-size:24px">${fmt(p.radius_nm, 0)} NM${p.radius_nm_g115 && Math.abs(p.radius_nm_g115 - p.radius_nm) > 0.5 ? ` <span style="font-size:15px;opacity:.75">(G 1,15: ${fmt(p.radius_nm_g115, 0)})</span>` : ''}</b></div>
    <div class="row"><span>Tempo de voo · reserva</span><b>${fmt(p.flight_time_min / 60, 1)} h · ${DM.reserve_min} min (${fmt(p.reserve_kg, 0)} kg)</b></div>
    <div class="row"><span>Decolagem de volta: raio máx. · curta</span><b>${fmt(p.return_takeoff_mass_kg, 0)} · ${fmt(p.return_takeoff_mass_max_kg, 0)} kg</b></div>`;
  if (profile === 'resgate') {
    h += `<div class="sep"></div>
    <div class="row"><span>Massa máx. no local, TDP 12 m (17 m)</span><b>${fmt(c.site_limit_tdp12_kg, 0)} (${fmt(c.site_limit_tdp17_kg, 0)}) kg</b></div>
    <div class="row"><span>Combustível máx. a bordo no local</span><b>${fmt(p.site_fuel_max_kg, 0)} kg</b></div>`;
    if (p.site_min_radius_full_fuel_nm > 0.5) {
      h += `<div class="warnbox"><b>Restrição de distância mínima:</b> com o tanque cheio, o local de resgate tem de estar a pelo menos <b>${fmt(p.site_min_radius_full_fuel_nm, 0)} NM</b> (${fmt(p.tdp17.site_min_radius_full_fuel_nm, 0)} NM com TDP de 17 m). Mais perto, abastecer menos na origem para chegar com no máximo ${fmt(p.site_fuel_max_kg, 0)} kg.</div>`;
    } else {
      h += `<div class="row"><span>Restrição de distância mínima</span><b class="okc">nenhuma com o tanque padrão</b></div>`;
    }
  }
  h += `<div class="src">Cruzeiro: ${DM.cruise_reference}. Reserva: ${DM.reserve_source}. Valores do Twin (docs/helicoptero/passo3_resultados.json).</div>`;
  $('missionBody').innerHTML = h;
  // right column: radius in every condition for this profile
  $('panel').innerHTML = `<div class="t">RAIO DE AÇÃO POR CONDIÇÃO · ${DM.profiles[profile].split(' (')[0].toUpperCase()}</div>` + rows.map((r, i) => {
    const q = r.profiles[profile];
    const lim = profile === 'resgate' && q.site_min_radius_full_fuel_nm > 0.5 ? `<br><span class="warnc" style="font-size:14px">local ≥ ${fmt(q.site_min_radius_full_fuel_nm, 0)} NM com tanque cheio</span>` : '';
    return `<div class="row" style="${i === missionCond ? 'color:var(--key)' : ''}"><span>${r.condition}${lim}</span><b>${fmt(q.radius_nm, 0)} NM</b></div>`;
  }).join('') + `<div class="sep"></div><div style="font-size:14px;opacity:.75;line-height:1.35">Raio limitado pelo tanque, pela massa Cat A na origem ou pela massa máxima no local de resgate (TDP 12 m).</div>`;
  // 3D: helicopter parked on the deck
  const run = DA.runs.reject, d = { ...sample(run, 0), nr_pct: 0 };
  poseHeli(heliA, d, 0, null, null, false);
  WA.deckLine.visible = false; WA.dropMark.visible = false;
  followSun(WA, 0, 0, 0, 30);
  look(camA, [-34, -40, 16], [4, 0, -2]);
  renderer.setScissorTest(false); renderer.setViewport(0, 0, W, H);
  renderer.render(WA.scene, camA);
  caption('');
  $('clock').textContent = 'Pré-voo';
}
const panelHTML = $('panel').innerHTML;

// ------------------------------------------------------------------------------------------------ public API (also used by the capture script)
let cur = { scene: params.get('scene') || 'cat_a', run: params.get('run') || null, t: parseFloat(params.get('t') || '0') };
function defaultRun(scene) { return Object.keys(SCENES[scene].runs)[0]; }
window.renderAt = (scene, run, t) => {
  cur = { scene, run: run || defaultRun(scene), t };
  if (scene !== 'missao' && !$('panel').querySelector('#nr')) { $('panel').innerHTML = panelHTML; tqBars.innerHTML = ''; rebuildBars(); }
  if (scene === 'cat_a') renderCatA(cur.run, t);
  else if (scene === 'advisory') renderAdvisory(cur.run, t);
  else if (scene === 'autorotation') renderAuto(t);
  else renderMission(cur.run);
  syncControls();
  return true;
};
function rebuildBars() {
  const el = $('tqBars'); bars.length = 0;
  for (let i = 0; i < 2; i++) {
    const d = document.createElement('div'); d.className = 'bar';
    d.innerHTML = `<span class="lbl">M${i + 1}</span><span class="track"><span class="fill"></span><span class="lim"></span></span><span class="val"></span>`;
    el.appendChild(d); bars.push({ fill: d.querySelector('.fill'), lim: d.querySelector('.lim'), val: d.querySelector('.val') });
  }
}
window.sceneInfo = () => ({
  cat_a: { reject: { fail: DA.runs.reject.fail_time, detect: detectT(DA.runs.reject), end: DA.runs.reject.tEnd, phases: DA.runs.reject.phases, timeline: DA.runs.reject._tl },
           continue: { fail: DA.runs.continue.fail_time, detect: detectT(DA.runs.continue), end: DA.runs.continue.tEnd, phases: DA.runs.continue.phases, timeline: DA.runs.continue._tl } },
  advisory: { procedimento: { fail: DB.runs.procedimento.fail_time, detect: detectT(DB.runs.procedimento), advisory_t: DB.runs.procedimento.advisory?.t, end: DB.runs.procedimento.tEnd },
              consultivo_seguido: { fail: DB.runs.consultivo_seguido.fail_time, end: DB.runs.consultivo_seguido.tEnd, phases: DB.runs.consultivo_seguido.phases } },
  autorotation: { vertical: DC.runs.vertical.landing, forward: DC.runs.forward.landing },
});

// ------------------------------------------------------------------------------------------------ interactive playback
let playing = false, speed = 1, last = 0;
function syncControls() {
  const st = $('sceneTabs'), rt = $('runTabs');
  st.innerHTML = Object.entries(SCENES).map(([k, s]) => `<button data-s="${k}" class="${k === cur.scene ? 'on' : ''}">${s.label}</button>`).join('');
  rt.innerHTML = Object.entries(SCENES[cur.scene].runs).map(([k, v]) => `<button data-r="${k}" class="${k === cur.run ? 'on' : ''}">${v}</button>`).join('');
  const tEnd = SCENES[cur.scene].tEnd(cur.run) || 1;
  $('scrub').value = Math.round(1000 * cur.t / tEnd);
  $('tlabel').textContent = `${fmt(cur.t, 1)} s`;
}
document.addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  if (b.dataset.s) { playing = false; window.renderAt(b.dataset.s, null, 0); }
  else if (b.dataset.r) { window.renderAt(cur.scene, b.dataset.r, cur.scene === 'missao' ? 0 : cur.t); }
  else if (b.dataset.p) { window.renderAt('missao', b.dataset.p, 0); }
  else if (b.dataset.c) { missionCond = +b.dataset.c; window.renderAt('missao', cur.run, 0); }
  else if (b.id === 'play') { playing = !playing; b.textContent = playing ? '❚❚' : '▶'; last = performance.now(); if (playing) requestAnimationFrame(tick); }
});
$('scrub').addEventListener('input', (e) => { const tEnd = SCENES[cur.scene].tEnd(cur.run) || 1; window.renderAt(cur.scene, cur.run, tEnd * e.target.value / 1000); });
$('speed').addEventListener('change', (e) => { speed = parseFloat(e.target.value); });
function tick(now) {
  if (!playing) return;
  const dt = Math.min(0.1, (now - last) / 1000) * speed; last = now;
  const tEnd = SCENES[cur.scene].tEnd(cur.run);
  let t = cur.t + dt; if (t > tEnd) { t = tEnd; playing = false; $('play').textContent = '▶'; }
  window.renderAt(cur.scene, cur.run, t);
  requestAnimationFrame(tick);
}
function fit() {
  const avail = CAPTURE ? { w: innerWidth, h: innerHeight } : { w: innerWidth, h: innerHeight - 52 };
  const s = Math.min(avail.w / W, avail.h / H);
  const f = $('frame'); f.style.transform = `scale(${s})`; f.style.left = `${(avail.w - W * s) / 2}px`; f.style.top = `${(avail.h - H * s) / 2}px`;
}
addEventListener('resize', fit); fit();
window.renderAt(cur.scene, cur.run, cur.t);
window.READY = true;
