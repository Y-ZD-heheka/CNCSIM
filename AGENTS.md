# AGENTS.md

> 给 AI agent / 新接手者的入口。**先读这一页，再读 `docs/项目记忆.md`。**
>
> 最后更新：2026-11-06 · 14 crate / **533 测试全绿** / 5 个外部依赖全最新
> （`cargo clippy` 有 8 条历史遗留告警，见 §2 注；sim / app 零告警）

---

## 1. 这是什么

Bevy + Rust 的**五轴 CNC 仿真验证器**，另带一个 **3 轴简化 CAM**（STL → 高度场 → G 代码）用于自造测试程序。

只回答四个问题：**会不会撞、会不会过切、实际跑多久、哪里出问题。**

**纯自用，不对外分发。** 由此可以读/抄 LinuxCNC（vismach、`5axiskins.c`、`g-code.adoc`）；但 GrabCAD 的"非商用"限制的是**使用场景**，公司商用仍适用 → 机床外观一律参数化生成。

---

## 2. 命令（改完代码必跑）

```bash
cargo test                        # 477 绿（29 ignored 是性能基准/需外部输入）
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
cargo run --release -- --demo dome --export out/dome.html  # 50×50×40 铣 R15 半球凸台：T1 粗 + T2 锥度球头精（换刀端到端）
cargo run --release -- --cam assets/models/relief.stl --stock 16,16,3.5 --gcode out/relief.nc --diff out/relief.diff.html --export out/cam.html
# CAM（3 轴）：STL → G 代码 → 解释器 → 仿真 → Δz 热力图（`--cam` 在 `--machine` 之前分支）
cargo run --release -- --cam "C:/path/流云花瓶.stl" --stock 112,112,250 --rotary --gcode out/vase.nc --diff out/vase.diff.html --export out/vase.html
# CAM（A/C 回转 + RTCP）：粗（径向让刀螺旋）+ 精（法向摆角）→ G43.4 → 外轮廓 Δr
cargo run --release -- --cam "C:/path/流云花瓶.stl" --stock 112,112,250 --rotary --no-rough --gcode out/nr.nc
# 多方向开粗（+Z / ±X / ±Y，每道自带粗+精），一份程序
cargo run --release -- --cam stls/桌腿固定器.stl --multi --gcode out/leg.nc
# 定轴 3+2：把旋转轴锁到 -X，走普通三轴刀路（`--flat-finish` 用立铣刀精加工）
cargo run --release -- --cam stls/桌腿固定器.stl --axis=-1,0,0 --flat-finish --gcode out/32.nc
# 任意倾角的斜向精加工（只要 A 轴够得到）：40° 倾角 × 8 个方位
cargo run --release -- --cam stls/桌腿固定器.stl --multi --tilt 40 --tilt-n 8 --gcode out/t40.nc
# 关掉切割染色（不分配 creator、不写位图；几何结果完全一致，大件省几十 MB）
cargo run --release -- --cam stls/桌腿固定器.stl --multi --color --gcode out/color.nc
# 工序编排（默认开）：按可达覆盖率自动补方向，并把"够不到/没切好"分开报
cargo run --release -- --cam stls/桌腿固定器.stl --multi --h 0.6 --gcode out/orch.nc
# 对照实验：跳过粗加工，隔离"过切是粗加工留下的还是精加工造成的"
python tools/check_mesh.py out/dome.html --ref-sphere 25,25,24,15   # 网格体检：几何/拓扑/法线/颜色（连通分量按**位置**判，不受拆顶点副本干扰）
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
crates/           14 个 crate（依赖方向由 check_deps.py 守卫）
  math            零依赖 f64 线性代数（含非对称矩阵求逆）
  motion          Pose（tip + axis）+ slerp
  tool            刀具母线 Profile + sd_profile
  kin             FK / IK / RTCP / 奇异点 / 非线性误差
  config          .machine.ron → MachineTopology（唯一知道磁盘格式的 crate）
  gcode           文本 → 位姿（不做运动学）
  cam             三角网格 → 3 轴高度场刀路 / A/C 回转 RTCP 刀路 → G 代码（只依赖 math/tool）
  voxel           稀疏窄带 SDF + 扫掠 + CSG + Surface Nets + LOD + STL + 连通块清理
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
| `voxel` | 159 | `gcode` | 117 |
| `sim` | 98 | `collide` | 34 |
| `kin` | 28 | `render` | 18 |
| `cam` | 28 | `tool` | 10 |
| `math` / `motion` / `config` | 9 / 9 / 9 | `ui` | 6 |
| `hmi` | 3 | | |

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
| 切割面着色 | ✅ 色号 = **刀具号 × 10 段 + 进度段**（粗加工 1..10、精加工 11..20，20 色定性板：粗暗饱 / 精亮饱，段间色相 36°），硬边色块 + flat 插值；溯源粒度默认 **每体素 1 B**（512 B/叶，50×50×40 半球实测 40 MB，可调 1³/2³/4³）；**只有"这一刀真的搬动了这一格"（场值变了）且新面在一格内的体素继承颜色**；只擦过、没去料的刀不改色（CSG 的 `max` 不动值）；提取从空气侧角点取色；**刀刃与切面同源**（同一个 `creator_of_tool_segment`、同一张 `PALETTE`）；`--export` 打切面颜色分布表 |
| 面板 | ✅ 数据层（横幅/色调/报警行，纯函数可测） |
| 刀具 / 换刀 | ✅ 锥度球头刀（母线相切于球头，不是折角）/ `ToolTable` 按**刀号**查（不是下标）/ **一份程序内换刀**：切、碰撞、着色、回放、活毛坯逐段按刀号取母线；`DesignModel::Boss`（盒∪球，半球凸台） |
| CAM（3 轴） | ✅ STL → 高度场 → 粗加工（Z 层行切，按刀盘腐蚀）+ 精加工（球头平行，法向偏置）→ G 代码；`--cam` 一条链跑解释器 + 仿真 + Δz 热力图；**中空/倒扣件按每列独立层数检测并拒绝**（3 轴高度场只能做单值上表面）；STEP 明确拒绝（B-rep 先三角化） |
| CAM（回转 5 轴） | ✅ `--rotary`：0.5°×0.25 mm 细网格取真值轮廓（**双线性查询，不取 3×3 最大**——取最大等于跟着包络走，刀永远进不了凹谷）→ 粗加工（**平头 Ø16**，径向让刀螺旋，竖直刀轴，可证不啃伤，顺带车掉方料四角）+ 精加工（**Ø2 锥度球头** 尖端 R1/10° 锥颈，**A/C 沿外法向**，`G43.4 H1` 刀尖跟随）；A 超 110° 时**限位 + 沿法向抬刀 `R(1−cosδ)`**（不跳过，跳过会连出穿件的弦）；比对两套——**亚体素 SDF 零点二分**（窗 ±3.5 mm，精度 0.01 mm）出 θ×z 的 Δr 热力图，**名义曲面逐点有符号偏差**（`sim::surface_dev`，1.08 M 点）出深度分布 + 按所需 A 角分组 |
| CAM（多方向 / 3+2） | ✅ `--multi`：五道工序（+Z/±X/±Y）各带粗+精，串成一份程序；`--axis x,y,z` = **定轴 3+2**（锁旋转轴走三轴）。侧向工序用 `G43.4 H1` + A/C 真的摆刀轴（只转坐标不改刀轴会让刀杆垂直插进工件，实测每道过切 105 mm³）；单道做不了就跳过并报原因（`PlanOutput::skipped`）；`allow_undercut` 让孔/内腔/倒扣"切得到就切、切不到留料" |
| 任意刀轴角度 | ✅ 斜向精加工可用 `--tilt <度> --tilt-n <条>` 取**任意**倾角（默认 54.7°×4 体对角线）；唯一约束是机床 `A = acos(dir.z) ≤ 110°`，超了直接丢并报数。程序头 `G54`（工件坐标）、尾 `G49`→`G53 Z…`（机床坐标回退）→`M5`→`M30` |
| 切割染色开关 | ✅ **默认关**，`--color` 显式开（`RunConfig::colorize`）：关掉时 `creator` **一个字节都不分配**、不写子块位图、`changed_mask` 走 `count_changed` 快路径——不是"不显示"，是那份工作根本不发生。几何逐体素一致（有测试），creator 0 B |
| 形位公差 | ✅ `sim::form`：**ISO 1101 最小区域法**评平面度/圆柱度（区域宽度 `max−min`，基准用八方向模式搜索优化），同时给 LSQ 值作对照；表面点沿实际场梯度投影得到。分片=位置连通+法向 12°；分不出特征的归 `Freeform` 只报偏差。桌腿件 h=0.6：两个平面 0.028 / 0.360 mm（差 13 倍，整件均值看不出来） |
| 模型相似度 | ✅ `sim::similarity`：双向 Hausdorff / Chamfer / F1@τ（P/R）/ IoU·Dice / 法向一致 / 有符号偏置；正向=设计面→实际场，反向=实际网格顶点→设计 SDF（**反向不可省**：远处的残料正向看不见）。自比标定 H 0.177 / Chamfer 0.006 / IoU 1.0000 @h=0.5，见 `docs/模型相似度评价.md` |
| 可达性分析 | ✅ `sim::reach`：对每个设计面采样点判「面朝 + 射线不被挡」（球体追踪），存 `facing`/`masks` 两张位图；够不到的面按空间连通块分组并给建议方向、A/C、bbox。封闭内腔（法向各向抵消）如实报「没有单一方向」而不是丢掉 |
| 工序编排 | ✅ `--multi` 把**整个可达半球**（A≤110°、25 个候选方向）当候选集，用带权集合覆盖贪心选方向：3 轴与 3+2 在这里只是"方向不同"，用哪种由覆盖面积决定。桌腿件 78.6% → **86.4%**（= 几何上限）。`--no-auto-pass` 关掉 |
| 多方向保真度口径 | ✅ **三个口径并排**：体素（`surface_dev`，分辨力≈h）/ 解析（`sweep_deviation`，**与 h 无关**，点到刀具扫掠体并集的有符号距离）/ 体积（`compare_with_design`）。桌腿件 h=0.6：体素 合格 81.6% / 过切 5.8% / 最大 +1.55；解析 **85.0% / 0.6% / +0.27**。逐点归因（`attribute_points`）证明体素报的过切里 79% 没有任何刀具扫过——薄于一个体素的尖角被判成空气。**验收看解析值**；行距/坡度两个旋钮实测无效（见 §39） |
| 精加工刀径 | ✅ 默认按工件尺度 `(短边/18).clamp(0.8,3.0)`，`--finish-d` 覆盖。这是**唯一有效的保真度杠杆**（凹角残料 `R(1/sin(θ/2)−1)` 面积 ∝ √R）：Ø3 85.0% / Ø1.5 91.6% / Ø1.0 94.1%，切长 45 / 84 / 123 m |
| 刀路体检 | ✅ `--dry`（`RunConfig::dry_run`）：只走刀路不切材料，解析口径的数字一模一样（测试钉住段数/端点/刀号逐段一致）。桌腿件 139527 块 424 s → **51 s**（8.3×），调刀路时用它，最后再跑一次全量仿真出体积/预览 |
| 比对性能 | ✅ 解析过切加段空间索引 + 两段扫描（区域细扫 + 全局 8× 粗扫）：桌腿件 9 道 h=3 的仿真 158.8 → **38.5 s**（4.1×），其中解析过切 135.8 → 16.0 s；`gouge_max_depth` 与逐点值差 2% |
| 刀杆碰撞 | ✅ `Process::check_shank`：普通球头/平底刀 + 无倒扣时**不需要查**（drop-cutter 已保证整根刀柱在外）；锥度刀或有倒扣时建 `MeshIndex`（XY 分桶 + 点—三角形距离），沿刀轴**从刃长以上**取样，`距离 < 刀杆半径 + 间隙` 就放弃该点、留料，计入 `Stats::shank_skips` |
| 落刀几何 | ✅ 球头/平底刀都是 **drop-cutter**：球头刀心 = `max(z + √(R²−d²))`，平底刀 = `max_within`。替代原来的"刀尖 = 曲面 + R·n"——后者在凹角处会让球贴墙啃进去（实测 1.35 mm ≈ 一个刀半径） |
| 开粗效率 | ✅ `ap = 1.0×D`、`ae = 0.7×D`、`vf = 600+120D`，MRR 打印出来（Ø6 时 33.3 cm³/min）；法向留量 0.3 mm 给精加工 |
| 能力边界（叶轮实测） | ✅ 两个真叶轮（Ø100×40 / 5.1 万面、Ø80×41 / 15.5 万面）跑通全链路：解析合格 61.7% / 47.3%，**过切只 1.3% / 0.3%**（失败模式是"留料"不是"切坏"）。第二个件可达 86.3% 但可达面只 15.4% 合格——`HeightMap` 每列只认最上面那层，叶片侧壁在任何方向都不是最高面。**叶轮要五轴侧刃铣，本链路已到上限**，见 `docs/项目记忆.md` §40 |
| 形状保真度度量 | ✅ 三层口径：① 单向几何量（`Δz` / `Δr`）② 整实体体积（`compare_with_design`，过切/残料体积 + 定位回 G 代码行）③ 名义曲面逐点（`surface_dev`，正=过切深度、负=残料厚度）；口径与盲区见 `docs/项目记忆.md` §27 |
| 悬浮碎屑 | ✅ `RunConfig::drop_chips`：跑完做一次 6 邻域连通块标记，只留最大块——切下来还"挂"在场里的碎块按掉落的切屑处理。体素 CSG 只做去除、不管连通性，不清理的话预览上会浮着一层小方块（花瓶实测丢掉 67 万格） |

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

0. **五轴侧刃铣（叶轮类件的唯一出路）**。两个叶轮的实测把上限钉死了：定轴/3+2
   在叶片通道里只能贴"每列最高面"，可达面合格 15.4%。要加工叶片侧壁必须让刀轴
   沿直纹面走（侧刃铣），这是新增刀路生成器，不是加方向。做了它，叶轮类件才进
   能力范围；不做就明确排除。
1. **凹角清根（小刀只走残料）**——现在最该动的一处。解析口径把问题定死了：
   可达面上 8.3% 残料里 **3308 点是"刀走过了但切不到"**（凹角圆角），
   只有 232 点是覆盖漏洞；>1 mm 的 8305 点全在不可达面。刀径实验给出上限：
   全局 Ø1 能到 94.1%（可达面 93.8%），但切长从 45 m 涨到 123 m。
   差的是"只在小刀有用的地方用小刀"：残料点 → 每道方向的局部掩膜 →
   Ø1 只在这些格子走刀。`finish_pass` 里加一个掩膜参数即可，
   `Pass` 加 `rest: Option<Vec<DVec3>>`，用 `--dry` 量收益。
2. **二次装夹（翻面）**：桌腿件 13.6% 的面积在单次装夹下几何不可达
   （背面 1732 mm² + 遮挡 328 mm²）。解析口径下这块折合约 17% 的残料面积，
   是剩下最大的一块。做法已经清楚：绕 X 轴 180° 的位姿是**精确的体素索引
   置换**（`(i,j,k) → (i, ny−1−j, nz−1−k)`，因为旋转中心取格心），
   设计网格与 `cut_segs` 用同一个 `Frame` 变换即可；缺的是把两套工序的
   `cut_segs` 拼起来算解析口径、以及毛坯在第二次装夹时的"已切状态"传递。
3. **残料重切（小刀）**：新覆盖的 1177 mm² 里大部分仍是残料，Ø3 球头放不进去。
   需要"仿真后读残料 → 聚类 → 换 Ø1.5 补一道"，即真正的一轮闭环。
   （与 #0 是同一件事，合并做。）
3. **剖切平面进 Bevy**——HTML 预览版已有（HUD 滑块 + `discard`），窗口里还没接。五轴件的内腔不剖开根本看不见。
4. **点选面 → 反查 creator**（ADR-005 §交互）。溯源数据已经有了，缺的是拾取 + 图例面板。
5. **`TileCache` 增量上传进窗口**（`render` 侧 11× 已实测，差 Bevy 那一步）。
4. **`SimJob` 进度条**（事件层已备，面板接一下）。
5. 自适应 LOD / `k=2` 按 level-1 分块 / 块索引第一遍并行 / `G95` 时间口径。
6. **小刀切削成本**：R0.5 锥度球头逐刀 3.4 ms（φ6 只要 0.24 ms）——小刀的 AABB
   小但 sd_profile 的固定成本占比高，值得给"球头/锥度"加一条解析快路径。
7. **大毛坯显式降粒度**：默认 `creator_sub = 8` 换精度，R15 半球实测 49 MB；
   只关心几何时调回 1/2/4，或 `set_creator(0)` 关闭溯源。

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
