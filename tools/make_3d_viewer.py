"""把 `CNCSIM_DUMP_DEV` 的逐点偏差 + 法向做成**可旋转的 3D 视图**（纯 canvas，无依赖）。

用法：python tools/make_3d_viewer.py out/imp_dev2.csv out/imp_3d.html "叶轮"

编码：每个点 10 字节 —— int16 x/y/z（0.01 mm）、int8 n×127、int8 d（0.005 mm）。
超差（|d| > 0.15）的点**全部保留**，其余按空间网格抽稀到目标点数——否则
442 个残料点会被 33 万好点稀释得看不见。
"""
import base64
import math
import sys

import numpy as np

src, dst = sys.argv[1], sys.argv[2]
name = sys.argv[3] if len(sys.argv) > 3 else "工件"
TARGET = 125000
# 第 4 个参数：去重格子边长（mm）。**加工面顶点**是等密度网格顶点，去重会把
# 本来就均匀的采样挖成麻点——那种输入传 0 关掉去重。
CELL = float(sys.argv[4]) if len(sys.argv) > 4 else 0.30
# 第 5 个参数：容差带（mm）。加工面顶点本身带体素离散误差（≈0.3），用 0.15 判
# 会把离散噪声全点亮；加工面视图一般给 0.5。
TOL = float(sys.argv[5]) if len(sys.argv) > 5 else 0.15
NOTE = sys.argv[6] if len(sys.argv) > 6 else ""

rows = np.loadtxt(src, delimiter=",")
P = rows[:, 0:3]
D = rows[:, 3]
N = rows[:, 4:7]
good = np.isfinite(D) & (np.abs(D) < 1e8)
P, D, N = P[good], D[good], N[good]

tol = TOL
bad = np.abs(D) > tol
hot = int(bad.sum())
# 统计必须在**抽稀之前**算：抽稀后分母变成图内点数，合格率会算成 98.7% 那种假数
n_res = int((D > tol).sum())
n_gouge = int((D < -tol).sum())
allp = int(len(D))
ok = 100.0 * (1 - (n_res + n_gouge) / allp)


# 抽稀：按 0.55 mm 的立方格投票，每格留一个（超差点不参与抽稀）
cell = CELL
key = np.floor(P / max(cell, 1e-6)).astype(np.int64)
key = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
keep = np.zeros(len(P), dtype=bool)
keep[bad] = True
if cell <= 0:
    keep[:] = True                      # 不去重：加工面本来就是均匀网格顶点
else:
    seen = {}
    for i in np.where(~bad)[0]:
        k = int(key[i])
        if k not in seen:
            seen[k] = i
            keep[i] = True
idx = np.where(keep)[0]
if len(idx) > TARGET:
    # 只抽"好点"，超差点一个不动——否则 442 个残料点会被抽走三成
    hot_idx = idx[bad[idx]]
    calm_idx = idx[~bad[idx]]
    k = max(1, TARGET - len(hot_idx))
    if len(calm_idx) > k:
        step = len(calm_idx) / k
        calm_idx = calm_idx[(np.arange(k) * step).astype(int)]
    idx = np.sort(np.concatenate([hot_idx, calm_idx]))

P, D, N = P[idx], D[idx], N[idx]
c = P.mean(axis=0)
P = P - c
scale = float(np.abs(P).max())

q = np.round(P * 100.0).astype(np.int16)
nq = np.round(np.clip(N, -1, 1) * 127.0).astype(np.int8)
dq = np.round(np.clip(D, -2.5, 2.5) * 200.0).astype(np.int8)

# **按点交错**写：每点 10 字节 = xyz(6) + n(3) + d(1)。
# 分段写（全部 xyz → 全部 n → 全部 d）会让解码端一读就串位，
# 症状是"满屏橙蓝"——法向的 127 被当成 d。
buf = np.empty((len(q), 10), dtype=np.uint8)
buf[:, 0:6] = q.astype("<i2").view(np.uint8).reshape(len(q), 6)
buf[:, 6:9] = nq.view(np.uint8).reshape(len(q), 3)
buf[:, 9] = dq.view(np.uint8)
blob = base64.b64encode(buf.tobytes()).decode()



html = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>{name} · 3D 偏差</title>
<style>
  * {{ box-sizing:border-box; }}
  body {{ margin:0; background:#0b0e12; color:#e8eef6;
         font:14px/1.6 "Segoe UI","Microsoft YaHei",system-ui,sans-serif; }}
  .wrap {{ max-width:1020px; margin:0 auto; padding:22px 22px 34px; }}
  h1 {{ font-size:20px; margin:0 0 2px; letter-spacing:.4px; }}
  .sub {{ color:#8b98a8; font-size:12.5px; margin-bottom:14px; }}
  .stage {{ position:relative; background:radial-gradient(120% 90% at 50% 0%,#141a21 0%,#0b0e12 70%);
            border:1px solid #222a33; border-radius:12px; overflow:hidden; }}
  canvas {{ display:block; width:100%; height:auto; cursor:grab; touch-action:none; }}
  canvas.drag {{ cursor:grabbing; }}
  .hud {{ position:absolute; left:14px; top:12px; font-size:12px; color:#93a1b1;
          line-height:1.7; pointer-events:none; }}
  .hud b {{ color:#e8eef6; font-weight:600; }}
  .dot {{ display:inline-block; width:7px; height:7px; border-radius:50%; margin-right:6px; }}
  .bar {{ position:absolute; right:14px; top:12px; display:flex; gap:8px; }}
  button {{ background:#1a222c; color:#cfd9e4; border:1px solid #2a343f; border-radius:7px;
            padding:5px 11px; font-size:12px; cursor:pointer; }}
  button:hover {{ background:#222c38; }}
  button.on {{ background:#24415e; border-color:#3d6f9e; color:#dbeaff; }}
  .legend {{ display:flex; gap:16px; flex-wrap:wrap; margin-top:14px; font-size:12px; color:#8b98a8; }}
  .sw {{ display:inline-flex; align-items:center; gap:6px; }}
  .sw i {{ width:20px; height:9px; border-radius:2px; }}
  .foot {{ color:#6f7b8a; font-size:12px; margin-top:12px; }}
</style></head><body><div class="wrap">
  <h1>{name} · 3D 偏差视图</h1>
  <div class="sub">{allp} 个点（图中 {len(P)} 点）· 拖动旋转 / 滚轮缩放 · 颜色＝到设计面的距离，明暗＝法向朝向</div>

  <div class="stage">
    <canvas id="cv" width="980" height="700"></canvas>
    <div class="hud">
      <div><span class="dot" style="background:#ff7a2f"></span>残料 <b>{n_res}</b> 点</div>
      <div><span class="dot" style="background:#4aa8ff"></span>过切 <b>{n_gouge}</b> 点</div>
      <div><span class="dot" style="background:#8fa3b8"></span>合格带内</div>
    </div>
    <div class="bar">
      <button id="spin" class="on">自动旋转</button>
      <button id="only">只看超差</button>
      <button id="reset">复位</button>
    </div>
  </div>

  <div class="legend">
    <span class="sw"><i style="background:linear-gradient(90deg,#969aa0,#c3c6ca)"></i>合格带内 |d| ≤ {TOL}</span>
    <span class="sw"><i style="background:linear-gradient(90deg,#ffb066,#ff3b1f)"></i>残料（切少了）</span>
    <span class="sw"><i style="background:linear-gradient(90deg,#74a8ff,#2b6fe0)"></i>过切（切多了）</span>
    <span style="margin-left:auto">带内（±{TOL}）{ok:.1f}% · 超差 {n_res + n_gouge} 点</span>
  </div>

  <div class="foot">{NOTE}</div>
</div>
<script>
const B64 = "{blob}";
const N = {len(P)}, SCALE = {scale:.4f};
const raw = atob(B64), bytes = new Uint8Array(raw.length);
for (let i=0;i<raw.length;i++) bytes[i] = raw.charCodeAt(i);
// 每点 10 字节 = xyz(int16×3) + n(int8×3) + d(int8)，**按点交错**。
// 不能用 `new Int16Array(buf, 0, N*3)` 那种分段解析——那假设"先全部 xyz"，
// 与写入布局不符，会让坐标整体错位（表现：形状散架、颜色乱）。
const dv = new DataView(bytes.buffer);
const PX = new Float32Array(N*3), NR = new Float32Array(N*3), DD = new Float32Array(N);
for (let i=0;i<N;i++) {{
  const o = i*10;
  PX[i*3]   = dv.getInt16(o,   true)/100;
  PX[i*3+1] = dv.getInt16(o+2, true)/100;
  PX[i*3+2] = dv.getInt16(o+4, true)/100;
  NR[i*3]   = dv.getInt8(o+6)/127;
  NR[i*3+1] = dv.getInt8(o+7)/127;
  NR[i*3+2] = dv.getInt8(o+8)/127;
  DD[i]     = dv.getInt8(o+9)/200;
}}

const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const W = cv.width, H = cv.height;
const img = ctx.createImageData(W, H);
const zbuf = new Float32Array(W*H);
const bg = new Uint8ClampedArray(W*H*4);
for (let y=0;y<H;y++) for (let x=0;x<W;x++) {{
  const t = y/H, o = (y*W+x)*4;
  bg[o] = 12 + 10*(1-t); bg[o+1] = 15 + 12*(1-t); bg[o+2] = 19 + 15*(1-t); bg[o+3] = 255;
}}

const qs = new URLSearchParams(location.search);
const num = (k, d) => qs.has(k) ? parseFloat(qs.get(k)) : d;
let yaw = num('yaw', 0.6), pitch = num('pitch', -0.45), zoom = num('zoom', 1.0);
let spin = !qs.has('static'), onlyBad = qs.has('only');

const TOL = {TOL};
function colorOf(d, shade, out) {{
  let r, g, b;
  if (d > TOL) {{
    const t = Math.min(1, (d-TOL)/0.35);
    r = 255; g = 176 - 115*t; b = 102 - 71*t;
  }} else if (d < -TOL) {{
    const t = Math.min(1, (-d-TOL)/0.2);
    r = 116 + 30*(1-t); g = 168 - 60*t; b = 255 - 31*t;
  }} else {{
    const t = Math.abs(d)/TOL;           // 带内：中性灰，越接近上限越亮
    r = 150 + 45*t; g = 154 + 42*t; b = 160 + 36*t;
  }}
  out[0] = r*shade; out[1] = g*shade; out[2] = b*shade;
}}

function draw() {{
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const s = (H*0.42/SCALE) * zoom;
  img.data.set(bg);
  zbuf.fill(1e9);
  const Lx = 0.35, Ly = 0.55, Lz = 0.76;    // 视空间光向
  const col = [0,0,0];
  for (let i=0;i<N;i++) {{
    const d = DD[i];
    // "只看超差"不是把好点删掉，而是把它们压暗——保留形状上下文，
    // 否则满屏几十个孤立的红蓝点，看不出残料落在哪条叶片的哪一侧。
    const dim = onlyBad && Math.abs(d) <= TOL;
    const x = PX[i*3], y = PX[i*3+1], z = PX[i*3+2];
    // yaw 绕 Y（工件轴）
    let x1 =  x*cy + z*sy, z1 = -x*sy + z*cy;
    // pitch 绕 X
    let y1 =  y*cp - z1*sp, z2 = y*sp + z1*cp;
    const sx = W*0.5 + x1*s, sy2 = H*0.52 - y1*s;
    const px = sx|0, py = sy2|0;
    if (px < 4 || py < 4 || px >= W-4 || py >= H-4) continue;
    // 法向：同一套旋转
    const nx = NR[i*3], ny = NR[i*3+1], nz = NR[i*3+2];
    let nx1 =  nx*cy + nz*sy, nz1 = -nx*sy + nz*cy;
    let ny1 =  ny*cp - nz1*sp, nz2 = ny*sp + nz1*cp;
    let lam = nx1*Lx + ny1*Ly + nz2*Lz;
    if (lam < 0) lam = -lam*0.35;              // 背面给一点环境光，别全黑
    const shade = (0.34 + 0.66*Math.min(1, lam)) * (dim ? 0.40 : 1);
    colorOf(d, shade, col);
    const depth = z2;
    // 超差点画大一圈：442 个点在 9 万点里否则根本看不见
    const r = (dim ? 1 : Math.max(1, Math.min(5, Math.round(1.9*zoom)))
                     + (Math.abs(d) > TOL ? 1 : 0));
    for (let dy=-r; dy<=r; dy++) {{
      const row = (py+dy)*W;
      for (let dx=-r; dx<=r; dx++) {{
        const p = row + px+dx;
        if (depth < zbuf[p]) {{ zbuf[p] = depth; const o = p*4;
          img.data[o]=col[0]; img.data[o+1]=col[1]; img.data[o+2]=col[2]; img.data[o+3]=255; }}
      }}
    }}
  }}
  ctx.putImageData(img, 0, 0);
}}

let drag = null;
cv.addEventListener('pointerdown', e => {{ drag = [e.clientX, e.clientY]; cv.classList.add('drag'); cv.setPointerCapture(e.pointerId); }});
cv.addEventListener('pointerup', e => {{ drag = null; cv.classList.remove('drag'); }});
cv.addEventListener('pointermove', e => {{
  if (!drag) return;
  yaw += (e.clientX - drag[0]) * 0.008;
  pitch += (e.clientY - drag[1]) * 0.006;
  pitch = Math.max(-1.5, Math.min(1.5, pitch));
  drag = [e.clientX, e.clientY];
  draw();
}});
cv.addEventListener('wheel', e => {{
  e.preventDefault();
  zoom *= Math.exp(-e.deltaY * 0.0012);
  zoom = Math.max(0.45, Math.min(4, zoom));
  draw();
}}, {{passive:false}});

const bs = document.getElementById('spin');
bs.classList.toggle('on', spin);
bs.onclick = () => {{ spin = !spin; bs.classList.toggle('on', spin); }};
const bo = document.getElementById('only');
bo.onclick = () => {{ onlyBad = !onlyBad; bo.classList.toggle('on', onlyBad); draw(); }};
document.getElementById('reset').onclick = () => {{ yaw=0.6; pitch=-0.45; zoom=1; draw(); }};

draw();
function loop() {{
  if (spin) {{ yaw += 0.0042; draw(); }}
  requestAnimationFrame(loop);
}}
loop();
</script>
</body></html>
"""

with open(dst, "w", encoding="utf-8") as f:
    f.write(html)
print(f"写出 {dst}（{len(html)/1024:.0f} KB）· 图内 {len(P)} 点 / 超差 {hot} 点")
