// 预览页面的 JS 冒烟测试：用**桩 WebGL** 跑一遍，确认它能初始化并画一次。
//
// 为什么需要它：预览是给人用浏览器打开的，这里没有浏览器。语法错、API 拼错、
// 矩阵写错，在 Rust 测试里全看不出来——用户打开只会看到一片空白。
//
// 用法：
//   node tools/check_preview.mjs out/preview.html
//
// 它检查：
//   1. <script> 能解析、能执行（不抛异常）
//   2. 顶点/索引数量与 HUD 上写的一致
//   3. 真的发出了 drawElements，且三角形数与数据对得上
//   4. 没有触发错误浮层（着色器编译/链接失败会走 showErr）
import fs from 'node:fs';
import vm from 'node:vm';

const path = process.argv[2] ?? 'out/preview.html';
const html = fs.readFileSync(path, 'utf8');

const fail = (msg) => {
  console.error('✗ ' + msg);
  process.exit(1);
};

const script = html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
if (!script) fail('没找到 <script> 块');

// ── 期望值：从 HTML 里抠出真实数据长度 ──
const idxB64 = html.match(/IDX_B64 = "([^"]*)"/)?.[1];
if (!idxB64) fail('没找到 IDX_B64（数据没内嵌？）');
const idxBytes = Buffer.from(idxB64, 'base64').length;
if (idxBytes % 12 !== 0) fail(`索引字节数 ${idxBytes} 不是 12 的倍数`);
const expectTris = idxBytes / 12;

const hud = html.match(/(\d+) 顶点 · (\d+) 三角形/);
if (!hud) fail('HUD 上没有顶点/三角形计数');
const expectVerts = Number(hud[1]);
if (Number(hud[2]) !== expectTris) fail(`HUD 说 ${hud[2]} 三角形，数据里是 ${expectTris}`);

// ── 桩 ──
const calls = { drawElements: 0, drawArrays: 0, count: 0, type: 0 };
const errors = [];

const gl = new Proxy(
  {},
  {
    get(_t, k) {
      if (typeof k !== 'string') return undefined;
      if (/^[A-Z0-9_]+$/.test(k)) return 1; // GL 常量
      switch (k) {
        case 'getShaderParameter':
        case 'getProgramParameter':
          return () => true;
        case 'getShaderInfoLog':
        case 'getProgramInfoLog':
          return () => '';
        case 'getUniformLocation':
          return () => ({});
        case 'getAttribLocation':
          return () => 0;
        case 'drawElements':
          return (mode, count, type) => {
            calls.drawElements++;
            calls.count = count;
            calls.type = type;
          };
        case 'drawArrays':
          return () => {
            calls.drawArrays++;
          };
        default:
          return () => ({});
      }
    },
  },
);

const canvas = {
  width: 0,
  height: 0,
  clientWidth: 800,
  clientHeight: 600,
  style: {},
  classList: { add() {}, remove() {} },
  getContext: () => gl,
  addEventListener() {},
  setPointerCapture() {},
  releasePointerCapture() {},
};

// 每个 id 一个独立元素：页面会按 id 取不同的控件（`clip` 是滑块、`clipv` 是标签），
// 共用一个对象的话 `clipEl.value = ...` 会污染标签，而且滑块缺 `addEventListener`
// 会直接抛异常（实测踩过）。
const mkEl = () => ({
  style: {},
  textContent: '',
  value: '',
  min: '',
  max: '',
  addEventListener() {},
});
const els = new Map();
const win = { addEventListener() {}, devicePixelRatio: 1 };
const document = {
  getElementById: (id) => {
    if (id === 'c') return canvas;
    if (!els.has(id)) els.set(id, mkEl());
    return els.get(id);
  },
};

let raf = null;
const sandbox = {
  document,
  window: win,
  requestAnimationFrame: (cb) => {
    raf = cb;
  },
  atob: (s) => Buffer.from(s, 'base64').toString('binary'),
  console: {
    log: () => {},
    error: (...a) => errors.push(a.join(' ')),
  },
  Math,
  Number,
  Float32Array,
  Uint8Array,
  Uint32Array,
};

try {
  vm.runInNewContext(script, sandbox, { timeout: 5000 });
} catch (e) {
  fail(`JS 执行抛异常：${e.message}`);
}

if (errors.length) fail('页面报了错：\n  ' + errors.join('\n  '));
const errEl = els.get('err');
if (errEl && (errEl.style.display === 'grid' || errEl.textContent)) {
  fail('触发了错误浮层：' + errEl.textContent);
}
if (typeof raf !== 'function') fail('没有启动绘制循环（requestAnimationFrame 没被调用）');

// 剖切平面：滑块 + 着色器 uniform 都要在。桩不做 GLSL 编译，所以这里只能查**文本**——
// 真正的编译失败会由页面的错误浮层兜住（上面那条断言）。
if (!html.includes('id="clip"')) fail('缺剖切滑块（id="clip"）');
if (!html.includes('uClipZ')) fail('着色器缺 uClipZ uniform');
if (!html.includes('discard')) fail('片元着色器缺 discard（剖切没实现）');

// 调色板：由 Rust 的 render::PALETTE 生成后插进 GLSL，必须是 16 项数组构造。
//
// 注释里的反引号会提前结束 JS 模板串（报的是 "Unexpected identifier 'xxx'"，指到
// 注释里的某个标识符，跟"着色器写错了"看起来毫无关系）——那一类由上面的
// "JS 执行抛异常"兜住，这里只查数据本身有没有插进去、项数对不对。
if (html.includes('__PALETTE__')) fail('调色板占位符没被替换（Rust 侧没走到替换？）');
// 项数必须与 Rust 的 `cncsim_voxel::PALETTE_LEN` 一致（现在是 32：
// 0 毛坯 + 每把刀 10 段 × 最多 3 把 + MIXED + SHANK）
const PALETTE_LEN = 32;
const palMatch = html.match(/vec3 pal\[(\d+)\] = vec3\[\d+\]\((.*?)\);/s);
if (!palMatch) fail('没找到 vec3 pal[N] 的数组构造');
const palDeclared = Number(palMatch[1]);
const palCount = (palMatch[2].match(/vec3\(/g) ?? []).length;
if (palDeclared !== PALETTE_LEN || palCount !== PALETTE_LEN) {
  fail(`调色板应有 ${PALETTE_LEN} 项，实际声明 ${palDeclared} / 初始化 ${palCount} 项`);
}

// 画一帧
try {
  raf();
} catch (e) {
  fail(`draw() 抛异常：${e.message}`);
}

if (calls.drawElements !== 1) fail(`应画一次三角形，实际 ${calls.drawElements} 次`);
if (calls.count !== expectTris * 3) fail(`drawElements 画了 ${calls.count} 个索引，应为 ${expectTris * 3}`);
if (calls.type !== 1) fail('索引类型不是 UNSIGNED_INT（大于 65535 个顶点会画错）');
if (calls.drawArrays !== 1) fail('地面网格没画');

// 相机是否真的把模型框进画面：包围盒 8 个角投到 NDC 后必须落在裁剪体内。
// 这一条能抓到 lookAt / perspective / 矩阵乘法写错——那类错误在浏览器里
// 表现为"一片空白"，而在桩测试里本来是看不出来的。
const probe = win.__cncsim;
if (!probe) fail('没有 NDC 探针（draw 没执行到？）');
if (probe.verts !== expectVerts) fail(`探针顶点数 ${probe.verts} ≠ ${expectVerts}`);
if (!probe.ndcLo) fail('包围盒有角落在相机后面（w ≤ 0）：相机朝向或距离不对');
for (let a = 0; a < 3; a++) {
  const [lo, hi] = [probe.ndcLo[a], probe.ndcHi[a]];
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) fail('NDC 出现 NaN/Inf：矩阵有错');
  if (lo < -1 || hi > 1) {
    fail(
      `模型超出视锥：轴 ${'xyz'[a]} NDC [${lo.toFixed(2)}, ${hi.toFixed(2)}] 越界 [-1,1]`,
    );
  }
}
// 还得占得够满，否则是"看得见但小成一个点"
const span = Math.max(...[0, 1, 2].map((a) => probe.ndcHi[a] - probe.ndcLo[a]));
if (span < 0.15) fail(`模型在画面里太小（NDC 跨度 ${span.toFixed(2)}）：取景距离不对`);

console.log(`✓ ${path}: ${expectVerts} 顶点 / ${expectTris} 三角形，绘制调用正常`);
