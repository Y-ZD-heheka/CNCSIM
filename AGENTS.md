# AGENTS.md

> 给 AI agent / 新接手者的入口。**先读这一页，再读 `docs/项目记忆.md`。**
>
> 最后更新：2026-11-02 · 13 crate / **469 测试全绿** / 3 个外部依赖全最新
> （`cargo clippy` 有 8 条历史遗留告警，见 §2 注）

---

## 1. 这是什么

Bevy + Rust 的**五轴 CNC 仿真验证器**，不是 CAM（不生成刀路）。

只回答四个问题：**会不会撞、会不会过切、实际跑多久、哪里出问题。**

**纯自用，不对外分发。** 由此可以读/抄 LinuxCNC（vismach、`5axiskins.c`、`g-code.adoc`）；但 GrabCAD 的"非商用"限制的是**使用场景**，公司商用仍适用 → 机床外观一律参数化生成。

---

## 2. 命令（改完代码必跑）

```bash
cargo test                        # 431 绿（17 ignored 是性能基准）
cargo clippy --all-targets        # 零新增；现有 8 条历史遗留（collide/gcode/voxel 旧代码），改到哪清到哪
python tools/check_deps.py        # 架构守卫：依赖白名单 + L0–L3 无 bevy + 禁用名
python tools/check_versions.py    # 依赖必须最新稳定版且精确锁定
```

**Python 解释器**（已装 numpy / markdown / pygments）：
`C:\Users\15796\.workbuddy-ai\binaries\python\envs\default\Scripts\python.exe`

### 运行时入口

```bash
cargo run                                              # 四段自检：运动学 → G代码 → 稀疏树 → 提取 → 端到端
cargo run --release -- --export out/preview.html       # 导自包含 WebGL2 预览（单文件，可离线发给别人）
node tools/check_preview.mjs out/preview.html          # 桩 WebGL 验页面本身（无显示器也能跑）
cargo run --release --features window -- --window      # 开 Bevy 窗口（首次编译付 Bevy 税）
cargo run -- --machine assets/machines/ac_trunnion.machine.ron   # 换机床，不重编译
cargo run -- --design part.stl                         # 设计模型从 STL 加载，算过切/残留
cargo run -- --demo pocket                             # 换回旧演示（面铣 + G41 方槽 + G83 孔）
cargo run --release -- --demo dome --export out/dome.html  # 50×50×100 铣 R15 半球凸台：T1 粗 + T2 锥度球头精（换刀端到端）
python tools/check_mesh.py out/dome.html --ref-sphere 25,25,84,15   # 网格体检：几何/拓扑/法线/颜色（连通分量按**位置**判，不受拆顶点副本干扰）
```

**截图检查**（无显示器时唯一能看画面的手段）：

```bash
# 三个方向的轴向正视图，隐藏刀具，停在 t=45 s（程序结束）
for v in x y z; do
  CNCSIM_VIEW=$v CNCSIM_HIDE_TOOL=1 CNCSIM_T=45 CNCSIM_CAM_SCALE=0.6 \
    ./target/release/cncsim-app --window --screenshot out/axis_$v.png
done
```

| 环境变量 | 作用 |
|---|---|
| `CNCSIM_T` | 起始回放时刻（**不自动播放**，否则截到哪一帧取决于帧率） |
| `CNCSIM_CAM_SCALE` | 相机距离倍率 |
| `CNCSIM_HIDE_TOOL=1` | 隐藏刀具（看切面时它会挡住最想看的区域） |
| `CNCSIM_VIEW=x\|y\|z\|-x\|-y\|-z` | 固定轴向正视图；**不挂轨道相机**（它会按自己的 yaw/pitch 覆盖 Transform） |
| `CNCSIM_PROF=1` | 打印每刀 求值/落盘 拆分 |
| `CNCSIM_THREADS=1` | 强制串行（对拍、定位并行问题） |

### 性能基准（默认 `#[ignore]`，改热路径后跑）

```bash
cargo test --release -p cncsim-voxel --test millout    -- --ignored --nocapture   # 铣光 200 刀
cargo test --release -p cncsim-voxel --test cut_perf   -- --ignored --nocapture   # 单刀成本
cargo test --release -p cncsim-voxel --test extract    -- --ignored --nocapture   # 200mm 提取（578 ms / 3200 万面）
cargo test --release -p cncsim-voxel --test lod        -- --ignored --nocapture   # LOD（实测 k=4: 252 ms / 200 万面 / 52 MB）
cargo test --release -p cncsim-voxel --test sweep_perf -- --ignored --nocapture   # 扫掠 SDF 单次求值成本
cargo test --release -p cncsim-voxel --test creator_perf -- --ignored --nocapture # 溯源开销 A/B（同窗交替，四档粒度）
cargo test --release -p cncsim-voxel --test creator_analysis -- --ignored --nocapture # 缓存/线程分析（工作集扫描）
for t in 1 8 24; do CNCSIM_THREADS=$t cargo test --release -p cncsim-voxel \
  --test creator_analysis thread_scaling -- --ignored --nocapture 2>/dev/null | grep THREADS; done
CNCSIM_PROF=1 ...                                       # 打印每刀 求值/落盘 拆分
CNCSIM_THREADS=1 ...                                    # 强制串行（对拍、定位并行问题）
```

### 文档

```bash
python tools/md2html.py <in.md> <out.html> "侧栏标题" "chip1|chip2"   # 改完 md 必须重渲染
```

---

## 3. 目录与分层

```
crates/           13 个 crate（依赖方向由 check_deps.py 守卫）
  math            零依赖 f64 线性代数（含非对称矩阵求逆）
  motion          Pose（tip + axis）+ slerp
  tool            刀具母线 Profile + sd_profile
  kin             FK / IK / RTCP / 奇异点 / 非线性误差
  config          .machine.ron → MachineTopology（唯一知道磁盘格式的 crate）
  gcode           文本 → 位姿（不做运动学）
  voxel           稀疏窄带 SDF + 扫掠 + CSG + Surface Nets + LOD + STL
  collide         保守推进 + GJK + EPA + episode 聚类
  sim             编排：run / compare / stock / job / playback
  hmi             控制状态机（急停、倍率、报警队列）
  render          GPU 缓冲 + 分块增量重建 + HTML 预览 + Bevy 外壳
  ui              面板数据层（纯函数，无 egui 依赖）
  app             CLI 与自检
docs/             20 篇 md + html（ADR-002…011 + 专题 + 项目记忆 + 开发日志）
tools/            守卫脚本、golden 生成、文档渲染、OpenVDB 对比基准
refs/linuxcnc/    LinuxCNC 源码（GPLv2，仅阅读，作为语义 oracle）
assets/           .machine.ron + MiSans 字体
testdata/         golden CSV + design_box.stl
```

### 测试数

| crate | 测试 | crate | 测试 |
|---|---|---|---|
| `voxel` | 149 | `gcode` | 117 |
| `sim` | 71 | `collide` | 34 |
| `kin` | 28 | `render` | 16 |
| `tool` | 10 | `math` / `motion` / `config` | 9 / 9 / 9 |
| `ui` | 6 | `hmi` | 3 |

---

## 4. 硬规定（违反即返工）

1. **Edition 2024**（`resolver = "3"`）；第三方依赖只用**精确最新稳定版** `version = "=x.y.z"`，**禁止 `^`**。加依赖前先 `check_versions.py --show <crate>`。
2. **L0–L3 零 Bevy 依赖**——正确性必须能纯 `cargo test` 锁定。
3. **没有 `cncsim-core`**；禁止 `common` / `utils` / `types` / `shared` 这类名字。
4. **`collide` 不依赖 `voxel`**（`DistanceField` trait 反转，接线在 `sim::stock`）。
5. **`gcode` 不做运动学**（输出位姿不是关节值，RTCP 归 `kin`）。
6. **`ui` 只依赖 `hmi`**，不直连 `sim`。UI 需要的 `Alarm`/`Severity` 由 `hmi` 显式重导出。
7. **`SparseGrid` 单一所有者**，跨线程只走 channel。禁止 `Arc<Mutex<SparseGrid>>`。
8. **结果必须逐位可复现**——任何破坏确定性的优化都不接受，哪怕更快。
9. **不建共享 `Diagnostic` 枚举**：各层自带 finding，`sim` 组合。
10. **`Pose.axis`** 是单位向量，**由刀尖指向主轴**（A=0 → +Z）。

### 工作方式

- **TDD 强制**：红 → 绿 → 重构。先写测试、跑一次确认它**因正确的原因失败**（不是拼写错）。禁止先写实现再补测试。
- **每个测试必须能被证伪**：断言要具体到能区分"做对了"和"做错了"。`assert!(x.is_ok())` 不算。
- **交付时报告红绿过程**：先写了什么测试、它如何失败、再如何通过。**主动证伪**（改坏一处 → 跑 → 复原）。
- **声称"完成"前必须实际跑过构建和测试**，报告真实输出。发现自己的说法被实测推翻时明确更正，不静默改口。

---

## 5. 当前进度

**L0–L3 内核全部落地，L4 窗口能开但显示增强未接。**

| 层 | 状态 |
|---|---|
| 运动学 | ✅ FK / IK / RTCP / 奇异点（条件数判据）/ 轴限位 / 非线性误差。543 行 golden 基准，RTCP↔FK 偏差 7.7e-14 |
| 机床配置 | ✅ `.machine.ron` → `MachineTopology`，换机床不重编译；`hardcoded_matches_file` 锁两份数据不漂移 |
| G 代码 | ✅ 117 测试。15 个模态组 / 25 个执行阶段 / 五轴 A·B·C / G68.2 倾斜平面 / **隐式模态运动** / 固定循环 G73·G76·G81–89 / G10·G52·G92 坐标改写 / G41·G42 刀径补偿 / `Trust` 分级 |
| 稀疏体素 | ✅ 三级树（root 也能存 tile）+ 量化 + 插值。40 亿体素填充 5.6 ms，叶 0 个 |
| 材料去除 | ✅ 扫掠 SDF + CSG + 行级分类 + 沿轴共享 + `cut_batch` |
| 表面提取 | ✅ Surface Nets（不用经典 MC）+ 单元格索引（块 + 位掩码） |
| 显示 LOD | ✅ 粗格重采样，`k ≤ band` 是硬约束（超限 panic） |
| 碰撞 | ✅ 保守推进 + GJK + EPA + episode 聚类 + 毛坯双向接线 |
| 仿真编排 | ✅ 端到端主循环 + 设计比对（过切/残留）+ 后台任务 + 回放 + HMI ingest |
| 显示层 | ✅ GPU 缓冲 + 分块增量重建（11×）+ 自包含 HTML 预览 + Bevy 窗口外壳（顶点色材质 / 刀具旋转体 + 刀刃刀柄分色 / `--screenshot`） |
| 切割面着色 | ✅ 色号 = **刀具号 × 10 段 + 进度段**（粗加工 1..10、精加工 11..20，20 色定性板：粗暗饱 / 精亮饱，段间色相 36°），硬边色块；溯源粒度默认 **每叶 1 B**（0.5 MB，可调 2³/4³/每体素）；**刀刃与切面同源**（同一个 `creator_of_tool_segment`、同一张 `PALETTE`）；`--export` 打切面颜色分布表 |
| 面板 | ✅ 数据层（横幅/色调/报警行，纯函数可测） |
| 刀具 / 换刀 | ✅ 锥度球头刀（母线相切于球头，不是折角）/ `ToolTable` 按**刀号**查（不是下标）/ **一份程序内换刀**：切、碰撞、着色、回放、活毛坯逐段按刀号取母线；`DesignModel::Boss`（盒∪球，半球凸台） |

### 性能锚点（24 线程 release，i7-13700K = 8P+8E）

| 场景 | 结果 |
|---|---|
| 200 mm 毛坯提取（3200 万面） | **578 ms**；LOD k=4 **257 ms / 200 万面 / 52 MB** |
| 铣光 50×50×100 / 200 刀 | 逐刀 **48 ms**（0.24 ms/刀）；整批 **16 ms**（0.08 ms/刀） |
| **建场**（同场景 `fill_box_exact`） | **14 ms**（按叶整块写回；原 47 ms） |
| 20 mm 单刀 | **~0.6 ms**（初版 11.9 ms，19×） |
| vs OpenVDB 14（端到端逐刀） | **8.0–9.4×**（铣光合计 **12.3×**） |

> **这些数字有 ±15% 的机器波动**：连续跑几十轮后 CPU 降频，铣光逐刀 47 → 56–65 ms、
> 建场 14 → 18–21。**单次测量不可信**——A/B 对比要在同一窗口交替跑，或看 `perf_cut_loop`
> 的 20 轮累计值。

---

## 6. 下一步（按价值排序）

0. **`creator_sub = 8` 还没默认启用**：它把溢出压到 0.025 mm，但内存 ×8（200 mm 毛坯 @h=0.05 约 128 MB）且切削慢 30%。默认 4 是拐点——除非有场景真的需要亚体素精度。
1. **剖切平面进 Bevy**——HTML 预览版已有（HUD 滑块 + `discard`），窗口里还没接。五轴件的内腔不剖开根本看不见。
2. **点选面 → 反查 creator**（ADR-005 §交互）。溯源数据已经有了，缺的是拾取 + 图例面板。
3. **`TileCache` 增量上传进窗口**（`render` 侧 11× 已实测，差 Bevy 那一步）。
4. **`SimJob` 进度条**（事件层已备，面板接一下）。
5. 自适应 LOD / `k=2` 按 level-1 分块 / 块索引第一遍并行 / `G95` 时间口径。
6. **小刀切削成本**：R0.5 锥度球头逐刀 3.4 ms（φ6 只要 0.24 ms）——小刀的 AABB
   小但 sd_profile 的固定成本占比高，值得给"球头/锥度"加一条解析快路径。

**不要做的**：通用网格碰撞、BVH、STEP（B-rep）解析、`cncsim-core`、共享 `Diagnostic` 枚举。

---

## 7. 改代码前读哪份文档

| 要改 | 先读 |
|---|---|
| 切削 / 扫掠 / 体素存储 | `docs/ADR-006-稀疏体素数据结构.md`、`docs/ADR-002-材料去除内核-SDF体素方案评估.md` |
| 表面提取 / LOD | `docs/ADR-006` §7.1、`docs/ADR-008-显示LOD与网格预算.md`、`docs/ADR-010-单元格索引.md` |
| 并行 / 性能 | `docs/ADR-009-段内并行.md`、`docs/并发与性能.md` |
| 碰撞 | `docs/ADR-004-碰撞检查方案.md` |
| G 代码语义 | `docs/G代码解释器设计.md`、`refs/linuxcnc/overview.adoc`（**权威清单在这里，别凭记忆**） |
| 运动学 / 机床参数 | `docs/机床建模-AC双转台.md`、`docs/ADR-007-机床模型表示.md` |
| 着色 / 溯源 | `docs/ADR-005-切割面着色与溯源.md` |
| 刀具形状 / 换刀 / 半球场景 | `docs/项目记忆.md` §21（母线切点、螺旋解析式、锥颈干涉） |
| 渲染缺陷体检 / 颜色麻点 | `docs/项目记忆.md` §22（四类检查、四个根因、法线细步长为什么失败） |
| 显示层 / 窗口 | `docs/ADR-011-显示层与预览导出.md` |
| HMI / 报错 | `docs/HMI与控制语义.md` |
| 精度问题 | `docs/计算精度分析.md` |
| 依赖取舍 / 自研边界（"为什么不用 X"） | `docs/ADR-003-自研技术栈边界与替代方案.md` |
| **任何"这条试过没有"** | **`docs/开发日志.md`**（含 9 条死胡同清单与全部证伪记录） |

---

## 8. 已知环境坑

- **`cargo test` 打印大量 `error deleting lock file`**——Windows 增量编译锁残留，无害，不是代码警告。
- **Bevy 0.20**：`shadows_enabled` → `shadow_maps_enabled`；`DefaultPlugins` 已含 picking（重复加 panic）；`Mesh` 属性是**带类型数组**（传扁平 `Vec<f32>` 编译过、**运行时** panic）；自定义着色器必须 WESL。
- **egui 自带字体无 CJK 字形**（中文全方框）→ 已嵌 MiSans（`assets/fonts/`）。
- **Rust 2024**：隐式借用模式里不能写 `&v`；`unsafe fn` 内每个不安全调用要显式包 `unsafe {}`。
- **`rayon` 只能经 `voxel::par::pool()`**——绕过它直接调 `par_iter` 会另建全局池（实测 50 线程抢 24 核）。
- **并行不是默认更快**：`blit_leaves_parallel` 在逐刀（256 KB）上比串行慢 **8.4 倍**，已加 1 MB 门槛。加并行前先算"每块工作 vs fork-join 唤醒（15–50 µs）"。
- **性能诊断不要用每刀 `eprintln!`**——打印比被测对象贵，测出来的分布全偏。用 `CNCSIM_PROF=1` + `prof_take()` / `prof_take_write()` / `prof_take_box()`（原子累加，跑完一次读）。
- **建场和切削要分开计时**：`PROF_BOX` 是建场（一次性），`PROF_EVAL/WRITE` 是切削（每刀）。混在一起看不出谁贵。
- **"数学上等价"的浮点改写要实测**：`cur * k_dist < x` 与 `cur < ceil(x * k_value)` 在实数上等价、浮点下不等价（见 `cut.rs::int_threshold_tests` 的反向测试）。
- **`uniform_value` 适合大块（早退），`count_ne` 必须小块（要数清楚）**。把后者也提到 32 会让铣光慢 8 ms。
- **单位陷阱（最高频的一类 bug）**：`RunConfig.stock_size` 是**体素**不是 mm；`kin` 旋转轴是**度**不是弧度；`D` 是寄存器号不是半径；`DVec3` 没有 `/ f64`。完整 11 条见 `docs/项目记忆.md` §16。
- 本机 cargo 走 rsproxy-sparse 镜像。

---

## 9. 记忆与文档索引

| 文档 | 什么时候读 |
|---|---|
| **`docs/项目记忆.md`** | **决策层单一真相源**：架构铁律、精度口径、性能锚点、死胡同、单位陷阱、测试方法论、待办 |
| **`docs/开发日志.md`** | 按日期的工作史：每天做了什么、拿到什么数字、踩了什么坑（含被推翻的中间结论） |
| `README.md` | 项目全貌 + 每块实现的详细说明（较长） |
| `docs/技术方案-五轴CNC仿真.md` | 13 章完整方案 |
| `docs/架构设计.md` | 分层、依赖白名单、守卫机制 |

`.workbuddy-ai/memory/MEMORY.md` 与 `.workbuddy/memory/README.md` 现在只是**指针**——不要再往那两个目录写笔记，统一写进 `docs/开发日志.md`。
