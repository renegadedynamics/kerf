// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The page of the WebAssembly spike: starts .NET, meshes a .brep with Kerf and draws the buffers with WebGL 2.
// With ?auto=1 it meshes every file the server lists (twice for the smaller ones, to time a warm run) and posts the
// results to /report.
import { dotnet } from './_framework/dotnet.js';

const status = document.getElementById('status');
const auto = new URLSearchParams(location.search).get('auto') === '1';
// In auto mode each line also goes to the server's console, so a headless run shows its progress
const log = (line) => {
  status.textContent += '\n' + line; status.scrollTop = status.scrollHeight;
  if (auto) fetch('log', { method: 'POST', body: line }).catch(() => {});
};
window.addEventListener('error', (e) => log('error: ' + e.message));
window.addEventListener('unhandledrejection', (e) => log('error: ' + (e.reason?.stack ?? e.reason)));
log('page loaded');

const t0 = performance.now();
const { getAssemblyExports, getConfig, runMain } = await dotnet.create();
const exports = await getAssemblyExports(getConfig().mainAssemblyName);
const Demo = exports.Kerf.Kernel.WasmDemo.Interop;
await runMain();
const startupMs = performance.now() - t0;
log(`.NET started in ${startupMs.toFixed(0)} ms (${navigator.userAgent})`);

// ---- WebGL 2 ----
const canvas = document.getElementById('view');
const gl = canvas.getContext('webgl2', { antialias: true });
let scene = null;
let yaw = 0.6, pitch = 0.5, zoom = 1;

function shader(type, source) {
  const s = gl.createShader(type);
  gl.shaderSource(s, source);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}
function program(vs, fs) {
  const p = gl.createProgram();
  gl.attachShader(p, shader(gl.VERTEX_SHADER, vs));
  gl.attachShader(p, shader(gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(p);
  return p;
}
const shaded = gl && program(
  `#version 300 es
   uniform mat4 mvp; in vec3 position; in vec3 normal; out vec3 n;
   void main() { n = normal; gl_Position = mvp * vec4(position, 1.0); }`,
  `#version 300 es
   precision mediump float; uniform vec3 light; in vec3 n; out vec4 color;
   void main() { float d = abs(dot(normalize(n), light)); color = vec4(vec3(0.35, 0.5, 0.75) * (0.3 + 0.7 * d), 1.0); }`);
const lines = gl && program(
  `#version 300 es
   uniform mat4 mvp; in vec3 position;
   void main() { gl_Position = mvp * vec4(position, 1.0); gl_Position.z -= 0.0005 * gl_Position.w; }`,
  `#version 300 es
   precision mediump float; out vec4 color; void main() { color = vec4(0.1, 0.1, 0.1, 1.0); }`);

function buffer(target, data) {
  const b = gl.createBuffer();
  gl.bindBuffer(target, b);
  gl.bufferData(target, data, gl.STATIC_DRAW);
  return b;
}

function load() {
  const positions = new Float32Array(Demo.Positions().slice().buffer);
  const normals = new Float32Array(Demo.Normals().slice().buffer);
  const triangles = new Uint32Array(Demo.Triangles());
  const points = new Float32Array(Demo.Points().slice().buffer);
  const edgePoints = Demo.EdgePoints();
  // Polylines to segments
  const segments = [];
  for (let e = 0; e + 1 < edgePoints.length; e++)
    for (let p = edgePoints[e]; p + 1 < edgePoints[e + 1]; p++) segments.push(p, p + 1);
  const all = positions.length ? positions : points;
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < all.length; i += 3)
    for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], all[i + k]); max[k] = Math.max(max[k], all[i + k]); }
  if (!gl) return;
  scene = {
    center: min.map((v, k) => 0.5 * (v + max[k])),
    radius: 0.5 * Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) || 1,
    positions: buffer(gl.ARRAY_BUFFER, positions), normals: buffer(gl.ARRAY_BUFFER, normals),
    triangles: buffer(gl.ELEMENT_ARRAY_BUFFER, triangles), count: triangles.length,
    points: buffer(gl.ARRAY_BUFFER, points), segments: buffer(gl.ELEMENT_ARRAY_BUFFER, new Uint32Array(segments)), segmentCount: segments.length,
  };
  draw();
}

function multiply(a, b) {
  const r = new Float32Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + j] * b[i * 4 + k]; r[i * 4 + j] = s;
  }
  return r;
}

function draw() {
  if (!scene || !gl) return;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  gl.viewport(0, 0, w, h);
  gl.clearColor(1, 1, 1, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  const d = 3 * scene.radius / zoom, f = 1 / Math.tan(0.4), aspect = w / Math.max(h, 1), near = d / 100, far = d + 4 * scene.radius;
  const projection = new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) / (near - far), -1, 0, 0, 2 * far * near / (near - far), 0]);
  const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
  const rotation = new Float32Array([cy, sy * sp, -sy * cp, 0, 0, cp, sp, 0, sy, -cy * sp, cy * cp, 0, 0, 0, 0, 1]);
  const [x, y, z] = scene.center;
  const translate = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -x, -y, -z, 1]);
  const back = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -d, 1]);
  const mvp = multiply(projection, multiply(back, multiply(rotation, translate)));

  gl.useProgram(shaded);
  gl.uniformMatrix4fv(gl.getUniformLocation(shaded, 'mvp'), false, mvp);
  gl.uniform3f(gl.getUniformLocation(shaded, 'light'), -sy * cp, sp, cy * cp);
  const attribute = (p, name, b) => { const l = gl.getAttribLocation(p, name); if (l < 0) return; gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.enableVertexAttribArray(l); gl.vertexAttribPointer(l, 3, gl.FLOAT, false, 0, 0); };
  attribute(shaded, 'position', scene.positions);
  attribute(shaded, 'normal', scene.normals);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, scene.triangles);
  gl.drawElements(gl.TRIANGLES, scene.count, gl.UNSIGNED_INT, 0);

  gl.useProgram(lines);
  gl.uniformMatrix4fv(gl.getUniformLocation(lines, 'mvp'), false, mvp);
  attribute(lines, 'position', scene.points);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, scene.segments);
  gl.drawElements(gl.LINES, scene.segmentCount, gl.UNSIGNED_INT, 0);
}

let dragging = null;
canvas.addEventListener('pointerdown', (e) => { dragging = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointerup', () => { dragging = null; });
canvas.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  yaw += 0.01 * (e.clientX - dragging[0]); pitch += 0.01 * (e.clientY - dragging[1]);
  dragging = [e.clientX, e.clientY]; draw();
});
canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom *= Math.exp(-0.001 * e.deltaY); draw(); }, { passive: false });
window.addEventListener('resize', draw);

// ---- meshing ----
function mesh(name, bytes) {
  const percent = Number(document.getElementById('percent').value) || 0.1;
  const run = JSON.parse(Demo.Mesh(name, bytes, percent));
  if (run.ok) {
    log(`${name}: read ${run.readMs} ms, mesh ${run.meshMs} ms, extract ${run.extractMs} ms; ${run.faces} faces, ${run.edges} edges, ${run.nodes} nodes, ${run.triangles} triangles`);
    load();
  } else {
    log(`${name}: ${run.error}`);
  }
  return run;
}

async function fetchBytes(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return new Uint8Array(await response.arrayBuffer());
}

const select = document.getElementById('files');
let list = [];
try {
  list = await (await fetch('data/list.json')).json();
  select.innerHTML = '<option value="">a file from data/...</option>' + list.map((f) => `<option>${f.name}</option>`).join('');
} catch {
  select.style.display = 'none';
}
select.addEventListener('change', async () => { if (select.value) mesh(select.value, await fetchBytes('data/' + encodeURIComponent(select.value))); });
document.getElementById('pick').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (file) mesh(file.name, new Uint8Array(await file.arrayBuffer()));
});

if (auto) {
  const runs = [];
  const pass = async (files, label) => {
    for (const f of files) {
      log(`meshing ${f.name} (${f.size} bytes, ${label})...`);
      const bytes = await fetchBytes('data/' + encodeURIComponent(f.name));
      const run = mesh(f.name, bytes);
      run.pass = label;
      runs.push(run);
      await new Promise((resolve) => setTimeout(resolve, 0)); // let the page paint
    }
  };
  await pass(list, 'cold');
  await pass(list.filter((f) => f.size <= 600000), 'warm');
  await fetch('report', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                          body: JSON.stringify({ userAgent: navigator.userAgent, startupMs, webgl: !!gl, runs }) });
  log('Reported.');
}
