//! 自包含 HTML 预览：把网格写成**一个文件**，双击就能转着看。
//!
//! 为什么不是"接 Bevy 开窗口"：Bevy 需要窗口系统和 GPU 上下文，而这一层要解决的
//! 问题是"我切出来的东西到底长什么样"。一个 2 MB 的 HTML 谁都能打开、能发给别人、
//! 能塞进 issue；Bevy 窗口不能。
//!
//! 这里不引任何前端依赖——three.js 之类的 CDN 一旦离线就是白屏。渲染器是 200 行
//! 原生 WebGL2：两个方向光 + 菲涅尔边缘 + 地面网格 + 轨道相机。
//!
//! **数据是内嵌的**（base64 的 f32/u32 数组），不是外链资源，所以单文件可离线。

use cncsim_voxel::MeshData;
use std::io::Write;
use std::path::Path;

/// 导出结果，给调用方打日志用。
pub struct PreviewStats {
    pub vertices: usize,
    pub triangles: usize,
    pub bytes: usize,
}

/// 把网格写成自包含的 HTML。
pub fn write_html(path: &Path, mesh: &MeshData, title: &str) -> std::io::Result<PreviewStats> {
    let html = build_html(mesh, title);
    let stats = PreviewStats {
        vertices: mesh.positions.len(),
        triangles: mesh.indices.len() / 3,
        bytes: html.len(),
    };
    if let Some(dir) = path.parent()
        && !dir.as_os_str().is_empty()
    {
        std::fs::create_dir_all(dir)?;
    }
    let mut f = std::fs::File::create(path)?;
    f.write_all(html.as_bytes())?;
    Ok(stats)
}

/// 生成 HTML 文本（测试直接查它）。
pub fn build_html(src: &MeshData, title: &str) -> String {
    // 与窗口一致：导出页也把 90° 棱做成硬边（否则同一个零件在窗口和网页里
    // 棱上的明暗不一样）。只影响显示，见 `MeshData::split_crease_normals`。
    let mut owned = src.clone();
    // **不要在这里做"孤立碎屑清理"**：`check_mesh.py` 早先报过 6 个 3 顶点分量，
    // 追下去全是本函数 `split_crease_normals` 复制出来的顶点副本（位置相同、
    // 索引不同）。按位置重算连通分量是 **1 个**，几何上一直是完好的。
    owned.split_crease_normals(0.707);
    // 拆完硬边再平滑：每个顶点只累加同一侧的面，棱角不会被抹圆
    owned.recompute_smooth_normals();
    let mesh = &owned;
    // 顶点压成 f32 小端、索引 u32 小端，各自 base64
    let mut pos = Vec::with_capacity(mesh.positions.len() * 12);
    let mut nrm = Vec::with_capacity(mesh.positions.len() * 12);
    for (i, p) in mesh.positions.iter().enumerate() {
        for v in p {
            pos.extend_from_slice(&v.to_le_bytes());
        }
        let n = mesh.normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]);
        for v in n {
            nrm.extend_from_slice(&v.to_le_bytes());
        }
    }
    let mut idx = Vec::with_capacity(mesh.indices.len() * 4);
    for i in &mesh.indices {
        idx.extend_from_slice(&i.to_le_bytes());
    }
    // 颜色：`[调色板索引, 余量偏差+128, 保留, 255]`（ADR-005 编码）
    let mut col = Vec::with_capacity(mesh.positions.len() * 4);
    for i in 0..mesh.positions.len() {
        col.extend_from_slice(&mesh.colors.get(i).copied().unwrap_or([0, 128, 0, 255]));
    }

    // 包围盒（相机自动取景用）
    let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
    for p in &mesh.positions {
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    if mesh.positions.is_empty() {
        lo = [0.0; 3];
        hi = [1.0; 3];
    }

    TEMPLATE
        .replace("__TITLE__", &escape(title))
        .replace("__POS__", &base64(&pos))
        .replace("__NRM__", &base64(&nrm))
        .replace("__IDX__", &base64(&idx))
        .replace("__COL__", &base64(&col))
        .replace("__PALETTE__", &glsl_palette())
        .replace("__NVERT__", &mesh.positions.len().to_string())
        .replace("__NTRIS__", &(mesh.indices.len() / 3).to_string())
        .replace(
            "__LO__",
            &format!("[{},{},{}]", lo[0], lo[1], lo[2]),
        )
        .replace(
            "__HI__",
            &format!("[{},{},{}]", hi[0], hi[1], hi[2]),
        )
}

/// 把 [`crate::PALETTE`] 渲染成 GLSL 的 `vec3[16]` 初始化列表。
///
/// **不手抄一份到 JS 里**：窗口和导出页必须说同一种颜色，而"两份常量保持一致"
/// 是那种今天对、改一次配色就悄悄分家的约定。这里从同一份 Rust 数据生成，
/// 生成结果直接插进着色器源码。
fn glsl_palette() -> String {
    let items: Vec<String> = crate::PALETTE
        .iter()
        .map(|c| format!("vec3({:.3}, {:.3}, {:.3})", c[0], c[1], c[2]))
        .collect();
    format!("vec3[{}]({})", items.len(), items.join(", "))
}

/// HTML 转义（标题可能带 `<`）
fn escape(s: &str) -> String {    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// 标准 base64（带 padding）。不引 crate：这 20 行不值得一个依赖。
fn base64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b = [
            chunk[0],
            *chunk.get(1).unwrap_or(&0),
            *chunk.get(2).unwrap_or(&0),
        ];
        let n = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
        out.push(T[(n >> 18) as usize & 63] as char);
        out.push(T[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 {
            T[(n >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            T[n as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

const TEMPLATE: &str = r#"<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  html,body{margin:0;height:100%;overflow:hidden;background:#0f1216;color:#cfd6df;
            font:13px/1.5 -apple-system,"Segoe UI",system-ui,sans-serif}
  canvas{display:block;width:100%;height:100%;cursor:grab}
  canvas.drag{cursor:grabbing}
  #hud{position:fixed;left:12px;top:10px;pointer-events:none;text-shadow:0 1px 3px #000}
  #hud b{color:#eaf0f7;font-weight:600}
  #hud .k{color:#7d8b9c}
  #err{position:fixed;inset:0;display:none;place-items:center;padding:40px;
       background:#0f1216;color:#ff9a9a;font:13px/1.6 ui-monospace,Consolas,monospace;
       white-space:pre-wrap;overflow:auto}
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="hud">
  <div><b>__TITLE__</b></div>
  <div class="k">__NVERT__ 顶点 · __NTRIS__ 三角形</div>
  <div class="k">拖拽旋转 · 滚轮缩放 · 双击复位</div>
  <div class="k" style="margin-top:6px">
    <label>剖切 Z <input id="clip" type="range" step="0.2"></label>
    <span id="clipv" class="k"></span>
  </div>
</div>
<div id="err"></div>
<script>
"use strict";
const POS_B64 = "__POS__", NRM_B64 = "__NRM__", IDX_B64 = "__IDX__", COL_B64 = "__COL__";
const LO = __LO__, HI = __HI__;

function showErr(msg){
  const e = document.getElementById('err');
  e.style.display = 'grid';
  e.textContent = msg;
  console.error(msg);
}
function b64buf(s){
  const bin = atob(s);
  const u = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
  return u.buffer;
}
const POS = new Float32Array(b64buf(POS_B64));
const NRM = new Float32Array(b64buf(NRM_B64));
const IDX = new Uint32Array(b64buf(IDX_B64));
const COL = new Uint8Array(b64buf(COL_B64));

// 剖切滑块：范围取模型的 z 包围盒，默认拖到最大 = 不剖
const clipEl = document.getElementById('clip');
const clipLabel = document.getElementById('clipv');
clipEl.min = (LO[2] - 1).toFixed(2);
clipEl.max = (HI[2] + 1).toFixed(2);
clipEl.value = clipEl.max;
let clipZ = parseFloat(clipEl.max);
function syncClip(){
  clipZ = parseFloat(clipEl.value);
  clipLabel.textContent = clipZ >= parseFloat(clipEl.max) ? '(关)' : clipZ.toFixed(1) + ' mm';
}
clipEl.addEventListener('input', syncClip);
syncClip();

const canvas = document.getElementById('c');
const gl = canvas.getContext('webgl2', {antialias:true});
if (!gl) { showErr('这个浏览器不支持 WebGL2。\n换个浏览器（Chrome / Edge / Firefox 新版）再打开。'); }

const VS = `#version 300 es
in vec3 aPos;
in vec3 aNorm;
in vec4 aColor;
uniform mat4 uVP;
uniform mat4 uModel;
out vec3 vN;
out vec3 vP;
out vec4 vC;
void main(){
  vec4 wp = uModel * vec4(aPos, 1.0);
  vP = wp.xyz;
  vN = mat3(uModel) * aNorm;
  vC = aColor;
  gl_Position = uVP * wp;
}`;

const FS = `#version 300 es
precision highp float;
in vec3 vN;
in vec3 vP;
in vec4 vC;
uniform int uFlat;
// **剖切平面**：世界 z 超过它就丢弃片元。五轴件的内腔不剖开根本看不见。
uniform float uClipZ;
out vec4 o;
void main(){
  // 地面先返回——它不该被剖切平面裁掉（否则拖到负值地面就没了）
  if (uFlat == 1) { o = vec4(0.16, 0.19, 0.23, 1.0); return; }
  if (vP.z > uClipZ) discard;
  vec3 n = normalize(vN);
  // 剖开后看得到**背面**：翻转法向，否则内腔一片死黑（两个方向光都打不到）
  if (!gl_FrontFacing) n = -n;
  vec3 L1 = normalize(vec3(0.45, 0.80, 0.55));
  vec3 L2 = normalize(vec3(-0.65, -0.25, 0.45));
  float d = max(dot(n, L1), 0.0) * 0.78 + max(dot(n, L2), 0.0) * 0.30;
  // 菲涅尔边缘：让平面轮廓在暗背景上也能看清
  vec3 V = normalize(vec3(0.0, 0.0, 1.0));
  float rim = pow(1.0 - abs(dot(n, V)), 3.0) * 0.25;
  // 顶点色：R 是**调色板索引**（不是红色分量），G 是余量偏差编码（128 = 零）
  // 调色板由 Rust 侧 render::PALETTE 生成后插入，与窗口共用同一份数据。
  // 注意：这段 GLSL 在一个 JS 模板字符串里，注释中**不能出现反引号**——
  // 会提前结束模板串，报一句莫名其妙的 "Unexpected identifier 'render'"。
  vec3 pal[32] = __PALETTE__;
  // 直接查表（硬边）：色号 = 刀具号 × 10 段 + 进度段（creator_of_tool_segment）
  float idx = vC.r * 255.0;
  vec3 base = pal[int(clamp(idx + 0.5, 0.0, 31.0))];
  float bias = (vC.g * 255.0 - 128.0) / 128.0;   // −1..1
  base *= 1.0 - clamp(abs(bias), 0.0, 1.0) * 0.18;
  vec3 c = base * (0.32 + d) + vec3(0.35, 0.45, 0.60) * rim;
  o = vec4(c, 1.0);
}`;

function compile(type, src){
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
    showErr('着色器编译失败：\n' + gl.getShaderInfoLog(s));
    return null;
  }
  return s;
}
const prog = gl.createProgram();
const vs = compile(gl.VERTEX_SHADER, VS), fs = compile(gl.FRAGMENT_SHADER, FS);
if (vs && fs) {
  gl.attachShader(prog, vs); gl.attachShader(prog, fs);
  gl.bindAttribLocation(prog, 0, 'aPos');
  gl.bindAttribLocation(prog, 1, 'aNorm');
  gl.bindAttribLocation(prog, 2, 'aColor');
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) showErr('着色器链接失败：\n' + gl.getProgramInfoLog(prog));
}

// ── 网格 ──
const vao = gl.createVertexArray();
gl.bindVertexArray(vao);
const pbuf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, pbuf);
gl.bufferData(gl.ARRAY_BUFFER, POS, gl.STATIC_DRAW);
gl.enableVertexAttribArray(0);
gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
const nbuf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, nbuf);
gl.bufferData(gl.ARRAY_BUFFER, NRM, gl.STATIC_DRAW);
gl.enableVertexAttribArray(1);
gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
const cbuf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, cbuf);
gl.bufferData(gl.ARRAY_BUFFER, COL, gl.STATIC_DRAW);
gl.enableVertexAttribArray(2);
gl.vertexAttribPointer(2, 4, gl.UNSIGNED_BYTE, true, 0, 0);
const ibuf = gl.createBuffer();
gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibuf);
gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, IDX, gl.STATIC_DRAW);

// ── 地面参考网格（给一个"哪边是上"的锚点）──
function makeGrid(){
  const span = Math.max(HI[0]-LO[0], HI[2]-LO[2]) * 0.9 + 4.0;
  const cx = (HI[0]+LO[0])/2, cz = (HI[2]+LO[2])/2;
  const y = LO[1] - 0.8;
  const step = Math.max(1.0, Math.pow(10, Math.round(Math.log10(span/10))));
  const n = Math.ceil(span / step) + 1;
  const v = [];
  for (let i = -n; i <= n; i++) {
    v.push(cx + i*step, y, cz - n*step,  cx + i*step, y, cz + n*step);
    v.push(cx - n*step, y, cz + i*step,  cx + n*step, y, cz + i*step);
  }
  return new Float32Array(v);
}
const GRID = makeGrid();
const gvao = gl.createVertexArray();
gl.bindVertexArray(gvao);
const gbuf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, gbuf);
gl.bufferData(gl.ARRAY_BUFFER, GRID, gl.STATIC_DRAW);
gl.enableVertexAttribArray(0);
gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
gl.disableVertexAttribArray(1);
gl.vertexAttrib3f(1, 0, 1, 0);
gl.disableVertexAttribArray(2);
gl.vertexAttrib4f(2, 0.0, 128.0/255.0, 0.0, 1.0);

// ── 相机 ──
const center = [(HI[0]+LO[0])/2, (HI[1]+LO[1])/2, (HI[2]+LO[2])/2];
const radius = Math.max(1e-3, 0.5 * Math.hypot(HI[0]-LO[0], HI[1]-LO[1], HI[2]-LO[2]));
let yaw = -0.9, pitch = 0.55, dist = radius * 2.6;

function perspective(fovy, aspect, near, far){
  const f = 1 / Math.tan(fovy / 2);
  return new Float32Array([
    f/aspect,0,0,0,
    0,f,0,0,
    0,0,(far+near)/(near-far),-1,
    0,0,2*far*near/(near-far),0
  ]);
}
function sub(a,b){ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function cross(a,b){ return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
function dot(a,b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
function norm(a){ const l = Math.hypot(a[0],a[1],a[2]) || 1; return [a[0]/l,a[1]/l,a[2]/l]; }
function lookAt(eye, c, up){
  const z = norm(sub(eye, c));
  const x = norm(cross(up, z));
  const y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x,eye), -dot(y,eye), -dot(z,eye), 1
  ]);
}
function xform(m, v){
  return [
    m[0]*v[0] + m[4]*v[1] + m[8]*v[2] + m[12],
    m[1]*v[0] + m[5]*v[1] + m[9]*v[2] + m[13],
    m[2]*v[0] + m[6]*v[1] + m[10]*v[2] + m[14],
    m[3]*v[0] + m[7]*v[1] + m[11]*v[2] + m[15]
  ];
}
function mul(a, b){
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    o[c*4+r] = a[r]*b[c*4] + a[4+r]*b[c*4+1] + a[8+r]*b[c*4+2] + a[12+r]*b[c*4+3];
  }
  return o;
}
const IDENT = new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);

const uVP = gl.getUniformLocation(prog, 'uVP');
const uModel = gl.getUniformLocation(prog, 'uModel');
const uFlat = gl.getUniformLocation(prog, 'uFlat');
const uClipZ = gl.getUniformLocation(prog, 'uClipZ');

function resize(){
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.max(1, Math.floor(canvas.clientWidth * dpr));
  canvas.height = Math.max(1, Math.floor(canvas.clientHeight * dpr));
  gl.viewport(0, 0, canvas.width, canvas.height);
}
window.addEventListener('resize', resize);

function draw(){
  resize();
  gl.clearColor(0.059, 0.071, 0.086, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);

  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const dir = [Math.cos(yaw)*cp, sp, Math.sin(yaw)*cp];
  const eye = [center[0]+dir[0]*dist, center[1]+dir[1]*dist, center[2]+dir[2]*dist];
  const proj = perspective(0.8, canvas.width / canvas.height, Math.max(0.01, dist*0.01), dist*40 + radius*8);
  const view = lookAt(eye, center, [0,1,0]);
  const vp = mul(proj, view);

  // 探针：把包围盒 8 个角投到 NDC，给自动化检查用（也方便手抖时排查"为什么白屏"）
  {
    let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < 8; i++) {
      const c = [ (i&1)?HI[0]:LO[0], (i&2)?HI[1]:LO[1], (i&4)?HI[2]:LO[2] ];
      const p = xform(vp, c);
      if (p[3] <= 0) { lo = null; break; }
      for (let a = 0; a < 3; a++) {
        const q = p[a] / p[3];
        lo[a] = Math.min(lo[a], q);
        hi[a] = Math.max(hi[a], q);
      }
    }
    window.__cncsim = { verts: POS.length / 3, tris: IDX.length / 3, ndcLo: lo, ndcHi: hi };
  }

  gl.useProgram(prog);
  gl.uniformMatrix4fv(uVP, false, vp);
  gl.uniformMatrix4fv(uModel, false, IDENT);
  gl.uniform1f(uClipZ, clipZ);

  // 地面
  gl.uniform1i(uFlat, 1);
  gl.bindVertexArray(gvao);
  gl.drawArrays(gl.LINES, 0, GRID.length / 3);

  // 毛坯
  gl.uniform1i(uFlat, 0);
  gl.bindVertexArray(vao);
  gl.drawElements(gl.TRIANGLES, IDX.length, gl.UNSIGNED_INT, 0);
  requestAnimationFrame(draw);
}
if (gl) requestAnimationFrame(draw);

// ── 交互 ──
let drag = false, px = 0, py = 0;
canvas.addEventListener('pointerdown', e => {
  drag = true; px = e.clientX; py = e.clientY;
  canvas.classList.add('drag'); canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointerup', e => {
  drag = false; canvas.classList.remove('drag');
  try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
});
canvas.addEventListener('pointermove', e => {
  if (!drag) return;
  yaw   -= (e.clientX - px) * 0.008;
  pitch += (e.clientY - py) * 0.008;
  pitch = Math.max(-1.45, Math.min(1.45, pitch));
  px = e.clientX; py = e.clientY;
});
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  dist *= Math.exp(e.deltaY * 0.0012);
  dist = Math.max(radius * 0.15, Math.min(radius * 40, dist));
}, {passive:false});
canvas.addEventListener('dblclick', () => {
  yaw = -0.9; pitch = 0.55; dist = radius * 2.6;
});
</script>
</body>
</html>
"#;

#[cfg(test)]
mod tests {
    use super::*;

    fn tri() -> MeshData {
        let mut m = MeshData::default();
        m.positions = vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]];
        m.normals = vec![[0.0, 0.0, 1.0]; 3];
        m.indices = vec![0, 1, 2];
        m
    }

    #[test]
    fn base64_matches_rfc4648_vectors() {
        // RFC 4648 的标准测试向量：长度 1/2/3 的边界都要对
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn html_embeds_the_mesh_and_the_counts() {
        let html = build_html(&tri(), "测试件");
        assert!(html.contains("<title>测试件</title>"));
        assert!(html.contains("3 顶点 · 1 三角形"));
        // 12 字节 → 16 个 base64 字符（不含 padding）
        assert_eq!(base64(&[0u8; 12]).len(), 16);
        // 位置数据必须原样内嵌：3 个顶点 × 3 个 f32，小端
        let mut bytes = Vec::new();
        for p in &tri().positions {
            for v in p {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        assert!(html.contains(&base64(&bytes)), "位置数据没有内嵌进去");
    }

    #[test]
    fn title_is_escaped() {
        let html = build_html(&tri(), "<script>x</script>");
        assert!(html.contains("&lt;script&gt;"), "标题必须转义，否则能注入");
        assert!(!html.contains("<title><script>"));
    }

    #[test]
    fn written_file_is_self_contained() {
        // 不能有外链：单文件要能离线双击打开
        let html = build_html(&tri(), "t");
        for bad in ["http://", "https://", "src=\"//", "<link"] {
            assert!(!html.contains(bad), "不该有外部引用：{bad}");
        }
        assert!(html.contains("WebGL2"), "缺 WebGL2 就白屏，至少要能提示");
    }

    #[test]
    fn write_html_creates_parent_dirs() {
        // 落在 target/ 下而不是系统 temp：沙箱/CI 里 temp 可能不可写，
        // 且产物跟着构建目录一起清理，不会在用户机器上留垃圾
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/test-out/cncsim_preview_test");
        let path = dir.join("a").join("b.html");
        let _ = std::fs::remove_dir_all(&dir);
        let st = write_html(&path, &tri(), "t").expect("写文件");
        assert_eq!(st.triangles, 1);
        assert!(path.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
