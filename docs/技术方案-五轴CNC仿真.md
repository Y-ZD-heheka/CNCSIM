# 五轴联动 CNC 切割仿真软件 — 技术方案

> 版本 v0.1 · 2026-10-28 · 基于 Bevy 0.20 / Rust 1.99
> 状态：设计草案（待评审）

---

## 0. 摘要

本文给出一个基于 **Bevy 0.20** 的五轴联动 CNC 加工仿真软件的技术方案。核心结论有三条：

1. **引擎选型：Bevy 0.20 可行且划算。** 0.20（2026-10-08 发布）把社区 `bevy_editor_cam` 上游化为内置 `PanOrbitCamera`，加上 0.19 引入的 `InfiniteGrid`、`TransformGizmo`、`DiagnosticsOverlay`，CAD 类应用最费时的"场景导航 + 调试可视化"部分基本白送。剩下的缺口（剖切平面、可编辑网格）都是小工作量。

2. **几何内核不自研，用 `cutrail`。** 这是一个 2026-10 发布的纯 Rust 材料去除内核（Apache-2.0），毛坯存三角网格、用 `manifold-rust` 做鲁棒布尔、原生支持 3 轴与 5 轴扫掠、带 STEP/STL/glTF 读写和设计模型比对。实测 10 万段 1mm 刀路、10mm 分块下每段中位数 2–6 ms，5 轴约为 3 轴的 1.7 倍。自研一套同等质量的网格布尔 + 5 轴扫掠是 6–12 人月的工作量，且正确性风险极高。

3. **不要追求"实时"仿真，要做"先算后播"。** 五轴精加工刀路动辄十万级线段，逐段网格布尔的成本注定高于真实加工时间。正确做法是后台算、前台播、带缓冲与背压——这与 Vericut / NCSIMUL 的实际架构一致。文档给出了可量化的性能预算。

此外，方案把**非线性误差**、**轴速超限导致的进给降速**、**奇异点**这三件五轴特有的问题做成一等公民——这是三轴仿真器（如 CAMotics）根本不具备、而现场调试五轴程序时最痛的地方，也是本产品最实际的差异化。

---

## 1. 目标与范围

### 1.1 产品定位

面向**机床操作工 / 工艺员 / CAM 编程员**的桌面端五轴加工仿真与验证工具。不是 CAM（不生成刀路），是**刀路验证器**：拿别人给的 G 代码，回答"会不会撞、会不会过切、实际要跑多久、哪里会出问题"。

### 1.2 v1 必须做到（验收口径）

| # | 能力 | 验收标准 |
|---|---|---|
| A1 | 读取 G 代码 | 支持 Fanuc 方言子集，含 G43.4 五轴联动、G68.2 倾斜工作平面 |
| A2 | 机床运动学 | 支持 6 种标准构型（AC/BC 摆头、AC/BC 转台、摆头+转台），构型由配置文件定义 |
| A3 | RTCP 验证 | 刀尖点跟随开启/关闭两种模式下轨迹一致，误差 < 1 µm |
| A4 | 材料去除 | 网格布尔，去除体积与解析解误差 < 1%，单块毛坯 200×200×100 mm 可跑通 |
| A5 | 碰撞检测 | 刀杆/刀柄/主轴 vs 毛坯/夹具/工作台，报出碰撞时刻与部件对 |
| A6 | 过切 | **解析扫掠点测试**：过切最大深度与位置 **±0.001 mm**；过切体积 ±1%（体素积分） |
| A6' | 残留 | 残留最大厚度 **±0.05 mm**（体素 h=0.1 的固有上限），可高亮 |
| A7 | 节拍预估 | 输出总加工时间，误差 < 5%（相对实测） |
| A8 | 交互 | 60 fps 下旋转/剖切/拖动时间轴不卡顿 |
| A9 | 规模 | 10 万段刀路、仿真时长 2 小时以内不崩、内存 < 2 GB |

### 1.3 明确不做（v1）

- 不做刀路生成 / 后处理（post-processor）
- 不做切削力、颤振、热变形、刀具磨损（v3 再说）
- 不做车削、车铣复合、线切割
- 不做多机协同 / 数字孪生联网
- 不做 Web 端（`cutrail` 支持 WASM，但 `manifold-rust` 在 WASM 下的性能与体积需单独验证，见 §11 R4）

### 1.4 自用场景的范围裁剪

前提：本工具**只在内部使用，不对外分发、不销售**。这个前提带来四个实质变化。

**(a) 许可证风险基本消失，但复用方式仍要克制**

GPLv2 的义务在"分发（convey）"时触发，私有使用不触发。所以 LinuxCNC 的 vismach 配置、`5axiskins.c` 的运动学实现、TWP remap 脚本都可以直接读、直接抄、直接链接，不需要做 clean-room。

但 GrabCAD 上大量模型写的是"仅限非商业用途"，**这个条款与分发无关**——只要公司用它挣钱就适用。而参数化生成只要 1 人日（§6.7），没必要冒这个模糊。

**(b) 可以 vendor 上游依赖**

`cutrail` 0.1.x 的 API 变动风险（§11 R1）在自用场景下直接消失：把它 `vendor/` 进仓库、锁定 commit，需要时自己打补丁，不跟上游。

**(c) 范围可以砍掉三分之一**

| 项 | 自用版 | 省下 | 理由 |
|---|---|---|---|
| 机床构型 | **只做你那一台** | 5 人日 | 6 套闭式解 + 锚点测试 → 1 套。但通用拓扑（`Vec<Joint>`）+ 数值逆解**保留**——它的边际成本极低（一个 FK + 一个 Newton），换来的是改机床参数不用改代码 |
| 产品化 | 全部砍掉 | 8 人日 | 安装包、自动更新、多语言、首次引导、崩溃上报、主题——自用全不需要 |
| 碰撞检测 | 先只做刀柄/刀杆 vs 毛坯 | 4 人日 | 机床自身碰撞（Z 轴 vs 转台）等真撞了再说 |
| 报告 | JSON + 屏幕高亮 | 3 人日 | PDF 排版对自用没有价值 |
| 剖切 / 刀路着色 | **保留** | 0 | 这是判断"切得对不对"的主要手段 |
| 过切 / 节拍 / 非线性误差 | **保留** | 0 | 这是工具存在的理由 |

合计约 **40 人日**（相对原 60 人日基线）。

> 在"尽可能自研"路线下（[ADR-003](ADR-003-自研技术栈边界与替代方案.md)），基线从 159.5 人日起算，这些裁剪同样适用，约落到 139.5 人日。

**(d) 反而应该增加一项投入**

自用最大的红利不是省钱，而是**能拿到权威的对照实现**——见 §8.9。这一项 +3 人日，但把项目最高风险项从"祈祷"变成"可验证"。

---

## 2. 技术选型调研

### 2.1 引擎候选对比

| 候选 | 优势 | 劣势 | 结论 |
|---|---|---|---|
| **Bevy 0.20** | ECS 天然适合"大量独立毛坯分块 + 大量关节实体"；Rust 端到端，几何内核可直接链接；0.20 内置 CAD 相机 | 每 ~3 个月破坏性更新；UI 生态弱于 Web | **选它** |
| three.js + WASM | Web 分发方便，生态成熟 | 几何内核必须跨 FFI 或重写；大网格更新受 JS 约束；Rust 内核 + JS 渲染两头调试 | 备选（若必须 Web） |
| Unity / Unreal | 渲染管线成熟，商业插件多（MachineWorks 有 Unity 版） | 几何内核要 C++/C# 互操作；许可成本；启动体积大 | 否 |
| 裸 wgpu + winit | 完全可控 | 等于自己写半个引擎，UI/资源/场景全要自建 | 否（除非团队有引擎经验） |

选 Bevy 的决定性理由不是渲染（三者都能渲染），而是**几何内核与业务逻辑能用同一种语言、同一套测试跑**。五轴运动学的正确性靠单元测试保证，而跨 FFI 的边界会让这类测试变得昂贵。

### 2.2 Bevy 0.20 现状（调研于 2026-10-28）

发布时间线：0.18.1（2026-03）→ 0.19.0（2026-06-19）→ 0.19.1（2026-08-13）→ **0.20.0（2026-10-08，227 位贡献者 / 817 个 PR）**。

对本项目直接相关的 0.20 特性：

| 特性 | 为什么重要 |
|---|---|
| `bevy_camera_controller::PanOrbitCamera` + `DefaultPanOrbitCameraPlugins` | CAD 式"围绕焦点旋转/平移/缩放"直接可用，省掉 2 周自研相机。**注意 feature 默认关闭**，需显式启用 `pan_orbit_camera` |
| WESL 着色器语言 | 自定义材质（剖切、过切高亮）必须写 `.wesl`；0.20 移除了旧版 Custom Bevy WGSL。这是**必须提前踩的坑**，见 §11 R2 |
| Mesh Shader 接入管线缓存 | v3 若做 GPU 端材料去除（体素）会用到 |
| `chain_weak()` / `after_weak()` | 仿真调度系统与渲染系统解耦，避免强排序拖累并行度 |
| 批量 `despawn_all` | 重置毛坯/重建分块时有用 |
| Panic 捕获转 error handler | 几何内核 panic 不应炸掉整个 UI |
| `Val::Em` / 上下文主题 | UI 排版 |

0.19 已提供、本项目同样依赖的：`InfiniteGridPlugin`（地面网格）、`TransformGizmoPlugin`（拖拽夹具）、`DiagnosticsOverlayPlugin`（FPS/网格统计）、`contact_shadows`（让刀具贴合毛坯的视觉判断更准）、`EditableText`（G 代码搜索框）。

**版本策略**：起步用 0.20。Bevy 每季度一次破坏性更新，把"升级到下一版"当作固定的迭代事项排进计划（每次约 3–5 人日，取决于自定义材质与 UI 的改动面）。

### 2.3 依赖清单与版本矩阵

**依赖原则：L2 领域内核全部自研，L1 引擎与呈现用现成**（详见 [`ADR-003-自研技术栈边界与替代方案.md`](ADR-003-自研技术栈边界与替代方案.md)）。

```toml
[workspace.dependencies]
# —— L1 引擎与呈现（不自研） ——
bevy        = "0.20"          # 需启用 pan_orbit_camera / gltf / png / debug 等 feature
bevy_egui   = "0.43"          # 0.43.x 对应 Bevy 0.20（0.40–0.42 对应 0.19）
egui_extras = "*"             # 表格 / 虚拟滚动列表
rfd         = "0.17"          # 原生文件对话框（属窗口系统，非领域逻辑）

# —— L0 语言级基础设施（不自研） ——
serde       = { version = "1", features = ["derive"] }

# —— L2 领域内核：自研，无外部依赖 ——
# cncsim-math     f64 向量/矩阵/四元数/最小二乘
# cncsim-kin      运动学（拓扑 + FK + 解耦 IK + RTCP）
# cncsim-gcode    字地址词法 + 模态状态机
# cncsim-voxel    稀疏窄带 SDF 内核 + MC 提取
# cncsim-collide  BVH + GJK/SAT
# （不再依赖 cutrail / manifold-rust / parry3d / gcode / truck / ron / thiserror / tracing）

[dev-dependencies]
# 修订 2026-10-29：proptest 已移除（两条被上游钉死的传递依赖拖累版本守卫），
# 改为 kin/tests/roundtrip.rs 里 20 行固定种子 xorshift。见 ADR-003 §2。
criterion = "*"               # 材料去除 / IK 基准（尚未引入）
```

**被移除的依赖及替代**：

| 移除 | 原因 | 替代 |
|---|---|---|
| `cutrail` + `manifold-rust` | 领域核心，且成本随加工细节增长 | 自研稀疏 SDF 体素（ADR-002 §3） |
| `parry3d` | 只需"最小距离 < 安全间隙"，不需要完整几何库 | 自研 BVH + GJK/SAT，用保守距离规避退化情形 |
| `gcode` | 只做词法，模态机本来就要自己写 | 自研字地址词法（约 300 行） |
| `truck`（STEP） | 自研 STEP 解析需 4 周以上 | v1 砍掉 STEP，设计模型用 STL |
| `ron` / `thiserror` / `tracing` | 不产生领域价值但可自研 | 极简配置解析 / `std::error::Error` / 30 行日志宏 |

**关于 `gcode` crate**：它只做**词法与语法**（把 `G1 X10. Y20. F500` 拆成字地址），**不维护模态状态**。模态组（G90/G91、G20/G21、G54–G59……）必须自己实现——这正好是我们要精确控制的部分，也是测试重点。既然模态要自研，词法也就一并自研了。

### 2.4 材料去除方案对比（关键决策）

| 方案 | 原理 | 精度 | 单段成本 | 欠切/侧凹 | 输出 | 代表实现 |
|---|---|---|---|---|---|---|
| **单轴 Dexel** | 沿一个方向存深度柱 | 中（受柱距限制） | 极低 | ❌ 不支持 | 高度场 | CAMotics |
| **Tri-Dexel** | 三个正交方向 Dexel | 中 | 低 | 近似支持 | 需重建 | 学术/部分商用 |
| **体素 / SDF** | 三维标量场 + Marching Cubes | 中（受分辨率限制，边缘圆钝） | 低–中（可 GPU 化） | ✅ | 网格 | 多数 GPU 仿真 |
| **网格布尔** | 毛坯网格 − 刀具扫掠体 | **高**（保尖锐边） | 中（ms 级） | ✅ | 网格 | **cutrail** / MachineWorks |

**选型结论（在"尽可能自研"前提下）：走自研稀疏 SDF 体素，不用网格布尔。**

先说网格布尔为什么曾经是最优解：五轴加工的价值在侧刃铣削与侧凹，Dexel 直接出局；稠密体素在 0.05 mm 精度下需 3.2×10¹⁰ 个体素（4 GB 起）；加工面的尖锐棱边是判断"切得对不对"的主要视觉依据，体素化会磨圆。所以早期结论是"用网格布尔 + cutrail"。

但自研要求改变了两件事：

- **网格布尔无法自研**：加工天然产生同深度底面、同路径侧壁，每次走刀都重合——这正是布尔最脆弱的退化情形。`cutrail` 架构文档实测：多个 Rust B-rep 内核在这些情形下"几乎全部失败"。Manifold 能撑住靠的是几十人年的鲁棒性工程。自研复现需 6–12 个月，且失败模式是偶发非流形，极难调试。
- **SDF 的精度短板可以绕开**：[ADR-002](ADR-002-材料去除内核-SDF体素方案评估.md) 指出 MC 会把棱边倒圆 h/2，撑不住 0.01 mm 过切判定。但自研路线**不需要用重建网格去比对**——过切改用**解析扫掠点测试**（设计曲面采样点是否落在刀具扫掠体内），精度是浮点级，与体素无关。详见 [ADR-003](ADR-003-自研技术栈边界与替代方案.md) §4。

于是分工变成：

| 用途 | 方法 | 精度 |
|---|---|---|
| 形状显示 / 体积 / 碰撞 / 残留 | 稀疏窄带 SDF（h=0.1 mm，约 130 MB） | 体积 0.3%，残留厚度 0.05 mm |
| **过切判定** | **解析扫掠点测试**（不经过体素） | **±0.001 mm** |
| 节拍 / 非线性误差 | 运动学直接计算 | 精确 |

**净收益**：单段成本与已加工细节无关（ADR-002 §1.1 的结构性弱点消失），且过切判定比网格比对更直接。**代价**：自研 45 人日，且残留厚度精度从"三角化误差"降到 0.05 mm 级。

**存储结构借鉴 OpenVDB**（详见 [`ADR-006-稀疏体素数据结构.md`](ADR-006-稀疏体素数据结构.md)）：三级树（root 哈希 → 32³ 内部节点 → 16³ 内部节点 → 8³ 叶）+ 位掩码 + 紧凑子数组，以及三种值（voxel / tile / background）。三个要点：

- **一个 level-2 节点覆盖 4096 体素/轴**（h=0.1 时 409.6 mm），所以 200 mm 毛坯的根只有 **1 条**条目——扁平 `HashMap` 方案要 12.5 万条。
- **`i8` 量化而非 `f32`**：窄带只需要符号 + h/8 精度，省 4 倍内存。
- **NanoVDB 只读，进不了仿真主循环**，只能做"仿真完成后导出"的线性快照。

`cutrail` 的实测数据（来自其 `docs/accuracy-and-performance.md`）：

- 默认弦高容差 `sweep_tolerance = 0.01 mm`，4 mm 刀具的去除体积误差约 **0.3%**
- 10 万段 1 mm 刀路 + 10 mm 分块：每段中位数从 **2 ms**（前 100 段）增长到 **6 ms**（后 100 段），内存稳定在 **92 MB**
- 5 轴扫掠约为 3 轴的 **1.7 倍**
- 99 秒机器人加工循环（100 Hz 采样）合并为 710 段，回放 **1.8 秒**，体积误差 0.04%

这几个数字直接决定了架构：**单段 ms 级 → 必须后台算；合并采样段 → 可以做到快于实时；分块 → 可以增量上传**。

### 2.5 与商用软件的功能对标

| 能力 | Vericut | NCSIMUL | CAMotics | 本方案 v1 |
|---|---|---|---|---|
| 五轴运动学 | ✅ 全构型 | ✅ | ❌ 仅 3 轴 | ✅ 6 构型 |
| 材料去除精度 | 高（B-rep/网格混合） | 高 | 中（Dexel） | 高（网格布尔） |
| 碰撞（含机床自身） | ✅ | ✅ | ❌ | ✅ 简化模型 |
| 非线性误差分析 | ✅ | ✅ | ❌ | ✅ |
| G 代码方言 | 极广 | 广 | 窄 | Fanuc 子集 |
| 价格 | 极贵 | 贵 | 免费 | — |

v1 不指望替代 Vericut，目标是把"五轴程序首件试切前的最低限度验证"做到免费/低成本可用。

---

## 3. 系统架构

### 3.1 分层

完整架构见 [`架构设计.md`](架构设计.md)（含依赖白名单与自动化守卫）。这里只列结果。

```
L5 装配    cncsim-app
              │
L4 呈现    cncsim-render ──┐   cncsim-ui ──┐
              │            │       │       │
L3 编排    └───┴── cncsim-sim ──────┴───────┘
                       │
        ┌──────────────┼──────────────┐
L2 几何内核  cncsim-voxel      cncsim-collide
        │              │              │
L1 领域算法  cncsim-kin ────┴── cncsim-gcode
        │              │              │
L0 词汇    cncsim-motion ── cncsim-tool
                       │
基座       cncsim-math
```

| crate | 一句话职责 | 依赖 |
|---|---|---|
| `cncsim-math` | f64 线性代数 | **无** |
| `cncsim-motion` | `Pose` / `MotionSegment` / 进给 / 时间戳 | math |
| `cncsim-tool` | 母线 profile / `CutterKind` / 刀柄 | math |
| `cncsim-kin` | 拓扑 / FK / 解耦 IK / RTCP / 奇异点 / 轴速 | math, motion |
| `cncsim-gcode` | 词法 / 模态机 / 方言 / `MotionProgram` | math, motion, tool |
| `cncsim-voxel` | 稀疏树 / 解析 SDF / 扫掠 / CSG / MC | math, motion, tool |
| `cncsim-collide` | GJK / 凸体 / 保守推进 / episode | math, motion, tool |
| `cncsim-sim` | 仿真调度 / 回放缓冲 / 后台线程 / 报告 | 以上全部 |
| `cncsim-hmi` | 人机控制：状态机 / 急停 / 倍率 / 报警队列 | math, sim |
| `cncsim-render` | Bevy 插件：场景、毛坯网格、剖切、着色 | + voxel, sim |
| `cncsim-ui` | bevy_egui 面板 | math, **hmi**（不直连 sim） |
| `cncsim-app` | 装配 / 窗口 / CLI / 文件 I/O | 全部 |

**这个分层不是为了好看，是为了可测。** L0–L3 零 Bevy 依赖，测试不需要 GPU、不需要窗口、不需要异步调度，`cargo test` 毫秒级跑完。五轴运动学的正确性 100% 由这层锁定；Bevy 层只负责"把算好的东西画出来"。

**三个关键的解耦决定**（详见架构设计 §2.3）：

1. **没有 `cncsim-core`**。公共类型按概念簇拆成 `math` / `motion` / `tool` 三个小 crate——`core` 会变成零内聚的类型垃圾场，扇入爆炸且掩盖设计问题。
2. **`collide` 不依赖 `voxel`**，通过 `DistanceField` trait 反转。这样碰撞可以脱离体素单独测试（用解析半空间做假实现，正是 ADR-004 §7.7 随机对拍需要的）。
3. **不建共享 `Diagnostic` 枚举**。各层自带 finding 类型，`sim` 组合——底层互不相识。

依赖白名单由 `tools/check_deps.py` 在 CI 校验，违规退出码 1。

### 3.2 Cargo workspace

```
cncsim/
├── Cargo.toml                 # [workspace] members，统一版本
├── crates/
│   ├── math/                  # cncsim-math      f64 线性代数，零依赖
│   ├── motion/                # cncsim-motion    Pose / MotionSegment
│   ├── tool/                  # cncsim-tool      刀具母线 profile
│   ├── kin/                   # cncsim-kin       FK / IK / RTCP
│   ├── gcode/                 # cncsim-gcode     词法 + 模态机
│   ├── voxel/                 # cncsim-voxel     稀疏树 + CSG + MC
│   ├── collide/               # cncsim-collide   GJK + 保守推进
│   ├── sim/                   # cncsim-sim       编排
│   ├── render/                # cncsim-render    Bevy 插件
│   ├── ui/                    # cncsim-ui        bevy_egui
│   └── app/                   # cncsim-app (bin) 装配
├── assets/
│   ├── machines/              # *.machine.ron 机床构型
│   ├── models/                # 机床/刀具 glTF
│   └── shaders/               # *.wesl 自定义材质
├── testdata/
│   ├── gcode/                 # 测试用 G 代码
│   └── golden/                # 体积/位姿基准数据
├── refs/                      # 外部参考（LinuxCNC 源码，仅阅读）
└── tools/                     # check_deps.py / verify_*.py / md2html.py
```

**目录名与 crate 名一一对应**（`crates/kin` ↔ `cncsim-kin`），`check_deps.py` 依赖这个约定推导 crate 名。

### 3.3 数据流

完整推导见 [`G代码到轴运动.md`](G代码到轴运动.md)。这里是全景：

```
G 代码文件
   │ ① 词法 + 模态状态机（cncsim-gcode）
   ▼
MotionProgram: Vec<Block>            ← 位姿（刀尖+刀轴）、进给模式、RTCP 状态
   │ ② 块 → 段：算时长（sim），由进给模式与轴速限共同决定
   ▼
Vec<Segment>                         ← 起止位姿 + 时长 + 插补空间
   │ ③ 按弦高容差细分 + 逆解（sim + kin），**流式消费，不存储**
   ▼
pose_at(t) → q(t)  ──┬──▶ 去除引擎（voxel）+ 碰撞（collide）   【后台线程】
                     │
                     └──▶ 回放：按当前时间重算同一函数          【主线程】
   ▼
TileUpdate / CollisionEpisode  ──channel──▶ Bevy 主线程更新 Mesh3d
```

**三个关键点：**

1. **插补空间由程序的 RTCP 状态自动决定**——`G43.4` 开则在位姿空间插补（刀尖走直线），关则在关节空间插补（刀尖走弧线）。选错的话误差是毫米级（算例 5.963 mm），而画面完全正常。
2. **点位不存储**。细分密度由弦高容差决定（每段约 5–50 点，不是固定 1 ms 的 3000 点）。计算流式消费，回放按需重算。生产比消费快约 200 倍，缓冲只需 256 点——见 [`点位预读与流水线缓冲.md`](点位预读与流水线缓冲.md)。
3. **计算与回放共用同一个 `pose_at` 函数**——否则"看到的"和"算的"不是一回事，画面在骗人。

**③ 是唯一的性能瓶颈**，也是唯一需要后台线程的环节。①② 是纯计算（µs 级），④⑤ 是 GPU 上传。

### 3.4 线程模型

完整讨论见 [`并发与性能.md`](并发与性能.md)。核心结论先说：

**材料去除是天然串行的**——第 N+1 段要读第 N 段切完的毛坯，**段间没有任何并发可加**。所以 `SparseGrid` **必须单一所有者**，这不是妥协是正确性要求（并行切削顺序不定会让结果不可复现）。

**真正的并行粒度是 brick**：一次切削的扫掠体可能覆盖多个 brick，而 brick 之间完全独立（各自的 SDF 数组、无共享）。所以是**零共享的 rayon 分片**，不需要锁也不需要原子操作。

```
主线程 (Bevy)          ：ECS 调度 + 渲染 + UI（不做任何几何计算）
├─ 预读线程 (专用)     ：插补 + IK，喂 PointStream（256 点环形缓冲）
├─ 仿真线程 (专用)     ：voxel + collide 独占 SparseGrid
│                        └─ 单次切削内部：brick 级 rayon 分片（≥4 个砖才并行）
└─ IO 线程             ：文件读取、报告导出
```

三条通道（全 SPSC）：

| 通道 | 方向 | 背压 |
|---|---|---|
| `cmd` | 主 → 仿真 | 容量 1，最新覆盖 |
| `point` | 预读 → 仿真 | 容量 256，**满了阻塞生产者**（自动压速） |
| `tile` | 仿真 → 主 | 容量 64，**满了阻塞仿真线程** |
| `stat` | 仿真 → 主 | 容量 8，**满了丢弃旧值** |

**注意两个缓冲的背压相反**：`point` 满了阻塞（压生产），`PlaybackBuffer` 满了丢旧的（否则仿真被"用户没在看"拖死）。

**关于"无锁"**：三个交接点都是 SPSC，用无锁环形缓冲确实该做——但**理由是消除抖动**（避免持锁线程被抢占导致 UI 卡顿），不是吞吐。实测贡献约 0.01%。真正让仿真快 3–6 倍的是 **SIMD 化 `sd_profile`** 与**热路径零分配**。

### 3.5 状态机

**两个状态机，职责不同**——混成一个是最常见的错误。

**（a）仿真调度状态**（在 `cncsim-sim`，Bevy 无关）

```rust
enum SimState {
    Idle,          // 无程序
    Parsing,       // 解析 G 代码
    Ready,         // 已解析，未开始
    Computing,     // 后台计算中
    Done,          // 计算完成，可自由回放
    Failed,        // 解析/仿真失败
}
```

**（b）操作控制状态**（在 `cncsim-hmi`，见 [`HMI与控制语义.md`](HMI与控制语义.md)）

```rust
enum ControlState {
    Idle,
    Ready { program_len: usize },
    Running { since: f64 },
    Paused { at: f64, reason: PauseReason },
    EStopped { at: f64, block: u32 },      // 必须复位才能离开
    Faulted { alarm: AlarmId },            // 必须复位才能离开
}
```

**为什么要分开**：`Computing` 是"后台在算什么"，`Running` 是"用户在看什么"。暂停只停回放，**计算继续**——这正是"先算后播"的自然结果。用户暂停时应该看到计算进度仍在涨。

`EStopped` / `Faulted` **必须复位**——这是安全语义，不能从急停直接"继续"。

### 3.6 ECS 资源与组件

```rust
// —— 资源（单例） ——
#[derive(Resource)] struct MachineModel(MachineTopology);   // 拓扑 + 关节限位
#[derive(Resource)] struct JointState { q: [f64; 5], tip: Pose };
#[derive(Resource)] struct PlaybackBuffer { frames: VecDeque<Frame>, cursor: usize };
#[derive(Resource)] struct SimClock { sim_time: f64, speed: f32, playing: bool };
#[derive(Resource)] struct Diagnostics(Vec<Diagnostic>);
#[derive(Resource)] struct StockHandle { tiles: Vec<Entity> };

// —— 组件 ——
#[derive(Component)] struct Joint { chain_index: u8, kind: JointKind };
#[derive(Component)] struct StockTile { id: TileId, dirty: bool };
#[derive(Component)] struct ToolEntity;                    // 刀具挂点
#[derive(Component)] struct Fixture;                       // 夹具（碰撞对象）
#[derive(Component)] struct ToolpathSegment { block_index: u32, motion: MotionKind };
```

### 3.7 仿真主循环（伪代码）

```rust
fn pump_simulation(
    mut engine: ResMut<SimEngineHandle>,   // 持 channel 端点
    program: Res<MotionProgram>,
    clock: Res<SimClock>,
) {
    // 1. 目标：始终保持"已算时间 >= 显示时间 + LOOKAHEAD"
    let target = clock.sim_time + LOOKAHEAD;          // 例如 +2.0 s
    while engine.computed_until < target && !engine.exhausted {
        let batch = program.steps_in(engine.computed_until, BATCH);   // 例如 64 段
        engine.send(Cmd::CutBatch(batch));            // 非阻塞发送
    }
    // 2. 排空回传的 tile 更新，限量上传（见 §6.2）
    engine.drain_tiles(MAX_UPLOAD_PER_FRAME);
}
```

---

## 4. 核心算法：五轴运动学

这是整个软件的**正确性核心**。所有后续（材料去除、碰撞、误差分析）都建立在"某个时刻刀具在工件坐标系的哪个位置、朝哪个方向"这个前提上。这部分做错，后面全错，而且错得很隐蔽——仿真看起来在动，实际位置偏了 0.5 mm。

### 4.1 机床拓扑的数据化描述

不要为每种构型写一份硬编码公式（12 种构型 = 12 份带符号陷阱的代码）。用**串联运动链**描述：

```rust
// cncsim-core/src/machine.rs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    /// 轴名，如 "X" / "A" / "C"
    pub name: String,
    pub kind: JointKind,
    /// 相对父节点的固定偏置（机床装配尺寸）
    pub offset: DVec3,
    /// 旋转关节的回转中心（相对 offset 之后的位置）
    pub pivot: DVec3,
    /// 方向符号：+1 / -1，用来适配不同厂的转向约定
    pub sign: f64,
    /// 行程/转角限位
    pub limit: (f64, f64),
    /// 该关节上的几何部件（显示外形 + 碰撞代理），见 ADR-007
    pub parts: Vec<Part>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum JointKind {
    Linear(Axis),                       // 平移
    Rotary(Axis),                       // 绕 axis 旋转
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Axis { X, Y, Z }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineTopology {
    /// 从"工件"到"刀具"的串联链，顺序即装配顺序
    pub chain: Vec<Joint>,
    /// 刀具长度（刀尖到主轴端面），RTCP 用
    pub tool_length: f64,
    /// 摆长：回转中心交点到主轴端面的距离
    pub pivot_distance: DVec3,
}
```

配置文件（`assets/machines/ac_head.machine.ron`，AC 摆头）：

```ron
(
  name: "AC 摆头龙门",
  chain: [
    (name: "X", kind: Linear(X), offset: (0.0, 0.0, 0.0), pivot: (0,0,0), sign:  1.0, limit: (-800.0, 800.0)),
    (name: "Y", kind: Linear(Y), offset: (0.0, 0.0, 0.0), pivot: (0,0,0), sign:  1.0, limit: (-500.0, 500.0)),
    (name: "Z", kind: Linear(Z), offset: (0.0, 0.0, 0.0), pivot: (0,0,0), sign:  1.0, limit: (-600.0,   0.0)),
    // C 轴为外环（靠近床身），A 轴为内环（靠近刀具）
    (name: "C", kind: Rotary(Z), offset: (0.0, 0.0, -150.0), pivot: (0,0,0), sign:  1.0, limit: (-360.0, 360.0)),
    (name: "A", kind: Rotary(X), offset: (0.0, 0.0, -120.0), pivot: (0,0,0), sign:  1.0, limit: (-110.0, 110.0)),
  ],
  tool_length: 80.0,
  pivot_distance: (0.0, 0.0, -200.0),
)
```

**为什么链顺序重要**：`C` 在外、`A` 在内，和 `A` 在外、`C` 在内，是两种不同的机床，运动学完全不同（前者是常见 AC 摆头，后者接近万向头）。把顺序写进配置，一个 `fk()` 函数覆盖全部构型。

### 4.2 正解（FK）

```rust
/// 返回 (工件系→刀具系 的变换, 刀尖点, 刀轴单位向量)
pub fn forward(chain: &[Joint], q: &[f64]) -> (DMat4, DVec3, DVec3) {
    let mut t = DMat4::IDENTITY;
    for (joint, &qi) in chain.iter().zip(q) {
        t = t * DMat4::from_translation(joint.offset);
        match joint.kind {
            JointKind::Linear(axis) => {
                t = t * DMat4::from_translation(axis.unit() * (joint.sign * qi));
            }
            JointKind::Rotary(axis) => {
                t = t * DMat4::from_translation(joint.pivot);
                t = t * DMat4::from_axis_angle(axis.unit(), joint.sign * qi.to_radians());
                t = t * DMat4::from_translation(-joint.pivot);
            }
        }
    }
    let tip  = t.transform_point3(DVec3::ZERO);      // 刀具系原点 = 刀尖
    let axis = t.transform_vector3(DVec3::Z);        // 刀具系 +Z = 刀轴
    (t, tip, axis.normalize())
}
```

AC 摆头（C 外 / A 内）的刀轴展开，可用来交叉验证实现：

```
n = Rz(C) · Rx(A) · ẑ = ( sinA·sinC,  -sinA·cosC,  cosA )
```

- `A = 0` → `n = (0,0,1)`（竖直向下前的机床 Z 向），符合直觉
- `A = 90°, C = 0` → `n = (0,-1,0)`
- `A = 90°, C = 90°` → `n = (1,0,0)`

BC 摆头（C 外 / B 内，B 绕 Y）：`n = ( sinB·cosC, sinB·sinC, cosB )`

### 4.3 逆解（IK）

不要为每种构型抄闭式公式——抄错一个符号，仿真就整体偏转，而且因为"看起来在动"极难发现。采用**解耦的两步法**，对所有"三直线轴正交 + 两旋转轴"的构型通用：

**第一步：由刀轴方向解两个旋转轴。** 只有 2 个未知数、2 个约束，用 Newton 迭代（数值雅可比，2×2 求逆），初值取闭式解或上一次的解：

```rust
pub fn solve_rotary(chain: &[Joint], target_axis: DVec3, seed: [f64; 2]) -> [f64; 2] {
    let mut q = seed;
    for _ in 0..8 {
        let n = axis_of(chain, q);                          // 由 FK 取刀轴
        let err = n.cross(target_axis);                     // 旋转误差（2 自由度）
        if err.length() < 1e-12 && n.dot(target_axis) > 0.0 { break; }
        let j = numeric_jacobian_2x2(chain, q);             // ∂n/∂q，中心差分
        q = q + solve2x2(j, err);
    }
    q
}
```

**第二步：由刀尖位置解三个直线轴。** 旋转轴定下来后，直线轴到刀尖的映射是**仿射**的（每个直线轴贡献一个固定方向的平移），闭式一步解出：

```rust
pub fn solve_linear(chain: &[Joint], q_rot: [f64; 2], target_tip: DVec3) -> [f64; 3] {
    // 把 q_rot 代入，算出旋转部分对刀尖的贡献 + 常量偏置
    let (base, dirs) = linear_affine_form(chain, q_rot);
    // dirs 为 3×3 矩阵（三直线轴方向在工件系下的表达）
    // 解 dirs · q_lin = target_tip - base
    let rhs = target_tip - base;
    let m = DMat3::from_cols(dirs[0], dirs[1], dirs[2]);
    let q_lin = m.inverse() * rhs;
    [q_lin.x, q_lin.y, q_lin.z]
}
```

**这个设计的价值**：闭式公式的符号约定因厂而异（Fanuc 的 A 正转方向和 Siemens 相反，转台和摆头的符号又互为逆），而 Newton 法只需要 FK 正确——FK 的正确性又可以由"手算三个位姿"的单元测试保证。**FK 是唯一的真相源，IK 是它的数值逆。**

**数值逆作为闭式解的测试预言（oracle）**：为 6 种标准构型各写一份闭式解（快、无迭代、可判断奇异），然后用 §8 的往返性质测试拿数值解做对照。闭式解写错，测试立刻红。

### 4.4 RTCP（刀尖点跟随）

这是五轴的核心，也是"五轴联动"和"3+2 定位"的分水岭。

**没有 RTCP 时**：程序里的 `X/Y/Z` 是**回转中心**的位置。转动 A/C 后刀尖会甩出去，编程员必须手工补偿，且只能在定位加工（3+2）下用。

**有 RTCP 时**：程序里的 `X/Y/Z` 是**刀尖**的位置，控制器实时算 `回转中心 = 刀尖 + L·n`。

```rust
/// RTCP 正补偿：给定刀尖位姿，求各直线轴应到的位置
pub fn rtcp_linear(chain: &[Joint], q_rot: [f64; 2], tip: DVec3, tool_len: f64) -> [f64; 3] {
    let n = axis_of(chain, q_rot);                 // 刀轴（指向主轴）
    let pivot_target = tip + n * tool_len;         // 回转中心目标
    solve_linear(chain, q_rot, pivot_target)
}
```

**RTCP 的不变量**（必须写成测试）：固定刀尖与刀轴，改变……不，更强的表述是——

> 给定同一段程序，在 RTCP 开启下，**任意时刻刀尖在工作坐标系的位置只由程序坐标决定，与两个旋转轴的角度无关**。

验证方法：构造一段"刀尖走直线、A 轴从 -30° 摆到 +30°"的程序，用 FK 逐点算刀尖，断言刀尖全部落在起点-终点连线上，偏差 < 1 µm。这个测试如果红，说明 `tool_length` 或 `pivot_distance` 的符号/基准搞错了——这是实践中最常见的 bug。

### 4.5 奇异点

AC 摆头在 `A = 0` 时，刀轴为 `(0,0,1)`，此时 `C` 轴绕自身轴旋转，**不改变刀轴方向**——雅可比退化，C 的逆解不唯一。

```rust
pub enum Singularity {
    /// |sin A| < eps：C 轴无定义
    RotaryAxisUndefined { joint: usize, angle: f64 },
    /// 刀轴与某直线轴近乎平行，该轴对刀尖位置贡献趋于 0
    LinearAxisDegenerate { joint: usize },
}

pub fn detect_singularity(chain: &[Joint], q: &[f64]) -> Option<Singularity> {
    const EPS: f64 = 1e-3;                          // ≈ 0.057°
    let a = q[rotary_index(chain, 0)];
    if a.to_radians().sin().abs() < EPS {
        return Some(Singularity::RotaryAxisUndefined { joint: 0, angle: a });
    }
    None
}
```

处理策略（与真实控制器一致）：**保持 C 轴不动**（hold last value），而不是让它跳 180°。UI 在时间轴上用红色标记奇异点区间——现场最典型的"过切/振纹"就发生在穿越奇异点的瞬间，能在屏幕上提前看到，是这个软件最有说服力的卖点之一。

### 4.6 插补模型与非线性误差

**这是本方案与三轴仿真器的关键差异点。**

五轴程序的两个相邻程序段之间，如果控制器做的是**关节空间线性插补**（每个插补周期把 5 个轴各走一小步），刀尖实际走的是一条**空间曲线**，偏离了 CAM 意图的直线。

**闭式公式**（推导与数值验证见 [`G代码到轴运动.md`](G代码到轴运动.md) §6）：

```
非线性误差 ≈ R · (1 − cos(Δθ/2))
```

`R` = 刀尖到回转中心的距离，`Δθ` = 该段转过的角度。这就是"弧与弦的拱高"。

**这个公式直接给出后处理的点位密度要求**——要求误差 < `ε` 时，每段最大摆角：

```
Δθ_max = 2 · acos(1 − ε/R)
```

`R = 175 mm` 时：ε=0.1 → 3.87°；**ε=0.01 → 1.22°**（弧长 3.7 mm）；ε=0.001 → 0.39°。

**注意**：RTCP 开启时控制器在**位姿空间**插补，刀尖走直线，这一段误差为零——非线性误差只在 RTCP 关闭或未做补偿时才出现。所以报这个指标前必须先确定程序的 RTCP 状态（见 [`G代码到轴运动.md`](G代码到轴运动.md) §2.1）。

```rust
pub struct NonlinearError {
    pub block_index: u32,
    pub max_deviation: f64,     // mm
    pub at_param: f64,          // 0..1，段内位置
    pub sampled: usize,
}

/// 计算一段程序在"关节线性插补"下的刀尖轨迹偏离直线的最大量
pub fn nonlinear_error(
    chain: &[Joint],
    q_start: &[f64], q_end: &[f64],
    tip_start: DVec3, tip_end: DVec3,
    samples: usize,
) -> NonlinearError {
    let dir = (tip_end - tip_start).normalize();
    let mut max_dev = 0.0; let mut at = 0.0;
    for k in 0..=samples {
        let t = k as f64 / samples as f64;
        let q: Vec<f64> = q_start.iter().zip(q_end)
            .map(|(a, b)| a + (b - a) * t).collect();      // 关节空间线性
        let (_, tip, _) = forward(chain, &q);
        let dev = ((tip - tip_start).cross(dir)).length(); // 到直线距离
        if dev > max_dev { max_dev = dev; at = t; }
    }
    NonlinearError { block_index: 0, max_deviation: max_dev, at_param: at, sampled: samples }
}
```

采样数取控制器插补周期（典型 1–2 ms）对应的步数，或按弧长自适应。报告里给出**每段最大偏差**，超过阈值（可配，默认 0.01 mm）的段在时间轴和 3D 视图里高亮。

> 真实控制器有"非线性误差补偿"（Fanuc 的 G43.4 + 高速高精模式会在内部细分程序段）。本软件默认模拟**无补偿**的裸插补，这是最坏情况，也是判断"后处理是否给足了点位密度"的正确口径。

> **实现**（`kin::nonlinear_error`，2026-10-30）：只用正解采样（关节值由 `sim::run`
> 逐块 `inverse` 拿到，上一块的解做下一块的种子）。两个早退：旋转轴全程不动 ⇒
> 直线轴仿射 ⇒ 刀尖恰在弦上，误差恒 0（三轴程序零成本）；起讫点重合（纯回转定点）
> 时弦退化为一点，改量到点的距离。单旋转轴机床对拍闭式到 **1e-9**。
> `RunConfig.machine = None` 时跳过；超 0.01 mm 报 `NONLINEAR`（Warning），
> `worst_pose` 记峰值时刻的实际刀尖、`max_nonlinear_line` 指回源文件。

### 4.7 轴速超限与实际进给

五轴联动的另一个陷阱：程序给的是**刀尖进给** `F`（mm/min），但旋转轴的角速度需求可能超出机床能力（典型 C 轴 30–60 rpm，A 轴 20–30 rpm）。控制器会自动降速，导致**实际节拍远长于程序预期**。

```rust
/// 返回该段在轴速约束下的最短耗时（秒）
pub fn min_block_time(
    chain: &[Joint], q_start: &[f64], q_end: &[f64],
    v_max_lin: [f64; 3], v_max_rot: [f64; 2],
) -> f64 {
    let mut t = 0.0_f64;
    let (mut li, mut ri) = (0, 0);
    for (joint, (a, b)) in chain.iter().zip(q_start.iter().zip(q_end)) {
        let d = (b - a).abs();
        match joint.kind {
            JointKind::Linear(_) => { t = t.max(d / v_max_lin[li]); li += 1; }
            JointKind::Rotary(_) => { t = t.max(d / v_max_rot[ri]); ri += 1; }
        }
    }
    t
}
```

对比 `programmed_time = 刀尖路径长 / F` 与 `min_block_time`，取大者即实际耗时。汇总得到**节拍预估**（验收项 A7）。UI 用"程序时间 vs 实际时间"的双色进度条展示——差距大的段说明这段五轴联动被轴速卡住了，值得回头改刀路。

---

### 4.8 两台参考机床（已建模、已数值验证）

运动学不能只停在公式上，得有两台参数完整、数值可验证的机器来锁住实现：

| 构型 | 文档 | 配置 | 特点 |
|---|---|---|---|
| **AC 摆头**（head-head） | 本文附录 A | — | 刀具转、工件不转；适合大件 |
| **AC 双转台**（摇篮/trunnion） | [`机床建模-AC双转台.md`](机床建模-AC双转台.md) | [`assets/machines/ac_trunnion.machine.ron`](../assets/machines/ac_trunnion.machine.ron) | 工件转、刀具不转；国产五轴主流 |

摇篮式的三个结论值得先记住：

1. **运动学只有一个几何参数 `L`**（A 轴中心 → 台面，取 180 mm）。常被当成参数的"机床 Z 参考面到 A 轴中心的距离 H"其实不影响刀尖相对工件的位姿——把 Z=0 定在 A 轴中心就把它吸收了。**少一个参数就少一个错的地方。**
2. **链顺序是 C → A → X → Y → Z**，旋转轴在**工件端**。这是与摆头式在配置层面的唯一区别，但 `sign` 要取 −1（工件转了 +A，表达"工具在工件系"必须用逆变换）。
3. **摆动吃行程与速度**：A 摆到 ±110° 时，Y 要多走 169 mm、Z 要多下沉 242 mm，纯补偿无切削；刀尖抬高 180 mm 时 A 轴 25 rpm 需要 Y 轴 **28.3 m/min** 的补偿速度——接近快移上限，控制器被迫降速。这是摇篮机的固有短板，也是 §4.7 轴速校核最该报出来的东西。

验证脚本 `tools/verify_trunnion.py`：RTCP ↔ FK 互逆偏差 **7.7×10⁻¹⁴**，RTCP 不变量漂移 **3.2×10⁻¹⁴**。

> **建模过程中查出两个符号错误**（RTCP 的 `(z−L)` 应为 `(z+L)`、刀轴 Y 分量符号反了）。第一个只让 Z 整体偏 360 mm，**画面看起来完全正常**——正是反复强调的那类 bug。手推公式之后必须用独立实现（齐次矩阵连乘）跑一遍锚点，两者一致才算数。

## 5. G 代码解释器

完整设计见 [`G代码解释器设计.md`](G代码解释器设计.md)。这里只列要点。

**这是继运动学之后最大的正确性风险**——解析错一行，后面所有运动都错，而画面依然"在动"。

### 5.1 边界：只做"文本 → 运动程序"

```
字节流 → 词法 → 模态状态机 → 坐标变换 → MotionProgram（不可变）
```

**输出的是工件坐标系下的位姿，不是关节值。** 解释器一旦调 `kin::inverse`，就绑死在某台机床上——换机床要改解析器。

### 5.2 一个必须分清的区别：TWP 归解释器，RTCP 归 kin

| | TWP（G68.2） | RTCP（G43.4） |
|---|---|---|
| 本质 | **坐标系变换**：程序坐标在一个倾斜平面里 | **机床运动学**：程序坐标是刀尖位置，控制器反解直线轴 |
| 依赖机床？ | **不依赖** | **强依赖**，每种构型公式不同 |
| 归谁 | **解释器** | **kin** |

判断标准：只需知道"工件怎么摆"→ 解释器；需要知道"机床几个轴、轴朝哪"→ kin。

### 5.3 模态组（Fanuc 子集）

| 组 | 指令 | 说明 |
|---|---|---|
| 1 运动 | G0 G1 G2 G3 | 直线/圆弧 |
| 2 平面 | G17 G18 G19 | 圆弧所在平面 |
| 3 距离 | G90 G91 | 绝对/增量 |
| 5 进给模式 | G93 G94 G95 | 倒数时间 / mm·min⁻¹ / mm·rev⁻¹ |
| 6 单位 | G20 G21 | 英寸/毫米 |
| 7 刀补 | G40 G41 G42 | 半径补偿 |
| 8 刀长 | G43 G44 G49 | 长度补偿（H 寄存器） |
| 12 工件系 | G54–G59, G54.1 P_ | 零点偏置 |
| 13 路径控制 | G61 G64 | 精确停/连续 |
| **15 五轴** | **G43.4 / G43.5** | **TCPM 开/关**（→ kin） |
| **16 五轴** | **G68.2 / G68.3 / G69** | **倾斜工作平面**（→ 解释器） |
| **17 五轴** | **G53.1** | **刀轴定向** |

### 5.4 块内执行顺序（最容易被忽略的坑）

一行 `G43 H7 G1 Z-5 F200` 里，这些字**不是同时生效的**：

```
进给模式 → F → S → T/M6 → M3/M4 → G43/G49/G41/G42
        → G54–G59 → G17–G19 → G90/G91 → 运动 → G4
```

**G43 必须在运动之前**（否则第一刀偏一个刀长），**G90/G91 必须在运动之前**（`G91 G1 X10` 与 `G1 X10 G91` 结果不同）。

顺序写错，单块测试全过，**只有组合块才会错**——所以测试必须专门覆盖"同一行多个模态字"。

### 5.5 跳过无法解析的块要标记"不可信"

方言差异大，一行不认识就整份失败会让工具不可用。但**不能一律跳过并警告**：

```gcode
G91
G999 X10     ← 跳过
G1 X10       ← 实际少走 10 mm
G90
G1 X100      ← 从这里开始一切偏 10 mm
```

**在 G91 下跳过一块增量运动，会让之后所有位置静默偏移。** 必须把整份结果标为 `Trust::Unreliable`，UI 顶部横幅提示，而不是记个行内警告。

这条设计的价值：**把"能跑但结果错"变成"明确告诉你别信"**。仿真工具最危险的输出就是看起来正常但实际错的。

### 5.6 方言抽象

| 系统 | 五轴开启 | 倾斜工作平面 | 刀轴定向 |
|---|---|---|---|
| Fanuc | `G43.4 H_` | `G68.2 X_ Y_ Z_ I_ J_ K_` | `G53.1` |
| Siemens 840D | `TRAORI(...)` | `CYCLE800(...)` | `ORIWKS` / `ORIMKS` |
| Heidenhain iTNC | `M128` / `TCPM` | `PLANE SPATIAL` | `CYCL DEF 19` |
| 国产（华中/广数） | 多为 `G43.4` 兼容 | `G68.2` | `G53.1` |

用 `trait Dialect`，v1 只有 `FanucDialect`。**它是测试接缝**（可以写极简 `TestDialect` 把模态机测试与方言细节解耦），**不是插件系统**——编译期确定，运行时不加载。

### 5.7 一次性解析，不做流式

整份读进内存（100k 块约 10 MB），解析成不可变 `MotionProgram`。**因为随机访问第 N 块是刚需**：时间轴拖动、点击报告跳行、断点定位都要 O(1) 拿块。流式会让这些都变复杂。

### 5.8 刀具表（G43 的 H 寄存器指向它）

解释器需要刀长（G43 H_）和半径（G41/G42），所以刀具定义属于 `cncsim-tool`，由解释器与几何内核共用：

```rust
// cncsim-tool
pub struct ToolDef {
    pub number: u16,
    pub kind: CutterKind,          // Flat / Ball / BullNose
    pub diameter: f64,
    pub corner_radius: f64,        // 圆鼻刀
    pub flute_length: f64,
    pub shank_diameter: f64,       // 碰撞检测用
    pub holder: Profile,           // 刀柄母线
    pub length_offset: f64,        // H 寄存器值
}
```

**必须与几何内核的刀具模型是同一份**——否则"仿真用的刀"和"碰撞用的刀"不是同一把，结果自相矛盾。

## 6. 材料去除与渲染

### 6.1 两级粒度：存储叶 vs 显示块

自研稀疏体素后有两个**不同**的粒度，别混：

| 粒度 | 尺寸 | 作用 | 数量级 |
|---|---|---|---|
| **存储叶**（OpenVDB 式 LeafNode） | 8³ 体素 = 0.8 mm @ h=0.1 | SDF 的存储与查询单位 | 39 万 |
| **显示块**（渲染单位） | 32³ 体素 = 3.2 mm | 一个 Bevy 实体 + 一个 Mesh | 约 4000 |

**为什么不能一叶一实体**：39 万个实体 × 每个一次 draw call，Bevy 和驱动都撑不住。显示块把 4³ = 64 个叶合成一个网格，实体数降到四千级。

**为什么显示块取 32³ 体素**：MC 的 cell 数是 31³ ≈ 3 万，三角面约 1 万——单个 Mesh 上传量约 0.5 MB，落在 §6.2 的 8 MB/帧预算内可以一次传 16 块。取 64³ 则单块 4 万面、上传 2 MB，粒度太粗，局部切削时会有明显跳变。

**显示块与 h 解耦**：块尺寸按**体素数**定义（32³），不按毫米。h 变了块就跟着变（h=0.05 时块变 1.6 mm），实体数随之 ×4——这是可接受的，因为 h 越小通常零件越小或用户接受更长的加载。

**脏传播**：切削改动的叶 → 标记所属显示块为脏 → 只重算脏块的 MC 并重传。这比 ADR-002 §3.4 的 brick 方案更细（0.8 mm vs 1.6 mm），局部切削的响应更好。

存储结构（树、位掩码、tile/background 值）见 [`ADR-006-稀疏体素数据结构.md`](ADR-006-稀疏体素数据结构.md)。

### 6.2 Bevy 侧的网格更新

**每块一个实体、一个独立 Mesh**，而不是一个大网格。理由：Bevy 的 `Mesh` 更新是整资源重传，大网格改一个角等于重传全部顶点。

```rust
#[derive(Component)]
struct StockTile { id: TileId, last_version: u64 }

fn apply_tile_updates(
    mut rx: ResMut<TileReceiver>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Mesh3d, &mut StockTile)>,
    mut budget: Local<UploadBudget>,
) {
    budget.reset_frame(MAX_UPLOAD_BYTES);              // 例如 8 MB/帧
    for update in rx.try_iter() {
        if !budget.take(update.mesh.byte_len()) { break; }   // 超出预算，留到下帧
        let Ok((mut mesh3d, mut tile)) = q.get_mut(update.entity) else { continue };
        let new_mesh = build_bevy_mesh(&update.mesh);  // TriMesh → Mesh（含法线）
        if let Some(m) = meshes.get_mut(&mesh3d.0) {
            *m = new_mesh;                             // 复用 handle，原地替换
        } else {
            mesh3d.0 = meshes.add(new_mesh);
        }
        tile.last_version = update.version;
    }
}
```

三个要点：

1. **`RenderAssetUsages`**：毛坯网格只在 GPU 侧用，设 `RenderAssetUsages::RENDER_WORLD` 可省一份 CPU 拷贝（约省一半内存）。但如果要做"导出 STL"，需要 `MAIN_WORLD`——导出时从 `cutrail` 侧取，不从 Bevy 取，所以仍可只留 RENDER_WORLD。
2. **法线**：加工面用**平面着色**（每个三角面独立法线，顶点不共享）比平滑着色更符合金属切削的视觉预期，且省去法线平均计算。代价是顶点数 ×3。200×200 mm 毛坯加工后的三角面数通常在 10⁵ 量级，×3 顶点 ≈ 1.5 MB，可接受。
3. **每帧上传预算**：不做预算限制，一次面铣全表面重算会在单帧内产生几十 MB 上传，直接掉帧。8 MB/帧 @ 60fps = 480 MB/s，是安全的。

### 6.3 剖切视图

工艺员判断内部型腔必须剖切。Bevy 无内置裁剪面，用自定义材质：

```wgsl
// assets/shaders/stock_section.wesl（0.20 起为 WESL，非旧版 WGSL）
@group(2) @binding(0) var<uniform> clip: vec4<f32>;   // xyz = 法线, w = 距离

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    if (dot(in.world_position.xyz, clip.xyz) - clip.w > 0.0) { discard; }
    return shade(in);
}
```

用 `ExtendedMaterial<StandardMaterial, SectionExt>` 继承 PBR 光照，只加一个裁剪 uniform——比从零写 PBR 省 90% 代码。**注意**：0.20 把旧版 "Custom Bevy WGSL" 换成了 WESL，`.wgsl` 里带 `#import` 之类的预处理指令会失效，必须转 `.wesl`。这是升级/起步时第一个会撞的坑。

剖切面需要"封口"（否则看到毛坯内壁的背面，视觉上是空的）。两个做法：简单版用 `cull_mode: None` + 双面光照；正确版在裁剪面处生成截面多边形（stencil 或 CPU 求交）。v1 用简单版，v2 再补。

### 6.4 刀路可视化

- `G0` 快移：虚线 / 半透明黄
- `G1` 切削：实线，按进给率映射色带（慢=蓝，快=红）
- `G2/G3`：按弦高离散成折线
- 五轴段额外画**刀轴短线**（每个采样点一根 5 mm 的短线），直观显示摆角变化
- 用 `Gizmos` 画（每帧重建，10 万段会有压力）→ 超过 5 万段时改为预生成 `Mesh` 的 `LineList` 拓扑，一次性上传

### 6.5 相机与交互

Bevy 0.20 内置 `PanOrbitCamera`：

```rust
app.add_plugins((MeshPickingPlugin, DefaultPanOrbitCameraPlugins));
commands.spawn((Camera3d::default(), PanOrbitCamera::default()));
```

需在 `Cargo.toml` 启用 `pan_orbit_camera` feature（默认关闭）。焦点用 `TransformGizmo` 或"双击毛坯表面设焦点"实现。

**与 egui 的输入冲突**：鼠标在 egui 面板上时不应驱动相机。`bevy_egui` 提供 `EguiContexts::ctx_mut().wants_pointer_input()`，在相机系统前加 `run_if` 判断。

### 6.6 性能预算（16.6 ms 帧）

| 环节 | 预算 | 说明 |
|---|---|---|
| ECS + 系统调度 | 1.0 ms | |
| 网格上传 | 3.0 ms | 8 MB/帧上限 |
| 渲染（10⁵ 三角面 + 阴影 + 网格地面） | 6.0 ms | 中端独显 |
| egui（面板 + 时间轴） | 2.0 ms | 表格用 `egui_extras` 虚拟滚动 |
| 余量 | 4.6 ms | |

后台仿真线程与主线程完全解耦，**仿真慢只影响"能看多远"，不影响帧率**——这是"先算后播"架构的核心收益。

### 6.7 机床可视化模型：参数化生成，不要下载

**结论：不要找现成的机床 3D 模型。用拓扑配置直接生成。**

#### 为什么现成模型不能用

一个从 GrabCAD / Sketchfab 下载的五轴机床模型给你的是**几千个三角面**，但它不告诉你：

- 两个旋转轴的回转中心交点在哪（pivot）
- 主轴端面到交点的距离（摆长，RTCP 的核心参数）
- 各轴的正方向（Fanuc 与 Siemens 的 A 轴转向相反）
- 行程与转角限位

而这四样**正是仿真正确性的全部依据**。从网格反推 pivot 在原理上可行（求两个旋转部件的公共轴线），但精度和工时都不划算，而且一旦错了，就是 §12 说的"整体偏 50 mm 但画面完全正常"那类 bug。

#### 模型来源盘点（均已核实）

| 来源 | 内容 | 许可证 | 能否直接用 |
|---|---|---|---|
| **LinuxCNC `vismach`** | **5 个真实五轴构型的完整可视化**：`bridgemill`、`max5`、`table-dual-rotary`、`table-rotary-tilting`、`table-rotary_spindle-rotary-nutating` | GPLv2 | 不能打包（传染性）；**是最有价值的参考** |
| **Open5x** | 开源五轴改造（Prusa i3 加双旋转台） | 需逐个确认 | 是 3D 打印挤出，不是铣削；仅参考构型 |
| **TraceParts / CADENAS PARTcommunity** | 100M+ **真实零部件**模型（导轨、丝杠、主轴、轴承、电机），STEP 等 14 种格式 | 制造商授权，供设计使用 | ✅ **最佳实用路线**（拼装用，见下） |
| Sketchfab（筛 CC 许可）/ 3D Warehouse | 少量机床模型 | **CC0/CC-BY 可商用，其余不行** | 需逐个确认 |
| GrabCAD | 大量机床模型 | **各模型不同**；条款页有访问限制，我未能核实具体条文 | ⚠️ 逐模型确认，多数禁止再分发 |

#### 四条路线怎么选

| 路线 | 精细度 | 工作量 | 主要风险 |
|---|---|---|---|
| A. 下载整机模型 | 高 | 低（找得到的话） | **拆不开关节** / 位姿非零 / 尺寸随意 / 许可不明 |
| **B. 厂商零部件拼装** | **高** | 1–2 周 | 低（尺寸真实、许可宽松） |
| C. 自己按手册尺寸建 | 中 | 1 周 | 无 |
| D. 方块 | 低 | 1 天 | 无 |

**推荐 B**：用 TraceParts 上的真实导轨/丝杠/主轴/轴承模型，配自己画的床身钣金拼出一台机床。

理由不是"省事"，而是**它同时解决了"精细"和"按关节拆分"两个问题**——你自己拼的，天然知道哪个零件属于哪个轴，不需要再去做 8.3 节的关节分配。而下载的整机模型往往是合并网格或静态装配体，拆开的工作量常常超过自己拼。

#### 一条必须说清的区分：精细 ≠ 准确

| | 影响观感 | 影响碰撞正确性 |
|---|---|---|
| 模型精细度 | ✅ | ❌（碰撞用 OBB，不用网格） |
| **模型尺寸是否真实** | ✅ | ✅ **如果 collider 设为 `Derived`** |

**尺寸随意的"示意模型"比方块更糟**——方块至少尺寸是你按手册填的，漂亮但比例错的模型会让碰撞检测在测一个假机床。

**折中方案**：下载模型只做外观，`collider` 一律用 `Explicit` 手填实测尺寸的 OBB。这样模型比例不准只影响观感，不影响正确性——这正是 ADR-007 "视觉与碰撞分离"设计的用处。

> 已做过一轮实测调研（含 Sketchfab 公开 API 枚举、LinuxCNC 源码下载），结论与候选清单见 [`机床模型调研-2026-10-28.md`](机床模型调研-2026-10-28.md)。核心发现：**"参数齐全"和"精美"在现实里是两个来源**——参数齐全的是 LinuxCNC 的 `5axiskins.c` + `5axis.ini`（已下载到 `refs/linuxcnc/`），精美的是 Sketchfab 上的 CC-BY 模型（需登录下载，且没有五轴整机）。

**LinuxCNC 的 vismach 是最值得读的参考**，理由有三：

1. 它的 `table-rotary_spindle-rotary-nutating` 配置附带完整的 **G68.2 / G68.3 / G68.4 / G69 / G53.1 / G53.3 / G53.6** 重映射实现，直接可作 §5.3 方言层的参照。它还支持**万向头（nutating，副旋转轴 0–90° 任意安装角）**，把 nutation 设成 90° 就退化成普通正交摆头——这个参数化思路比"为每种构型写一份代码"高明。
2. 它的五轴构型命名直接对应 ISO 轴序（`xyzac` / `xyzbc` / `xyzacb` / `xyzbca`），可用来交叉校验我们的拓扑配置。
3. **它的机床模型是代码里用基本体拼的**，不是网格文件。

第 3 点尤其重要。`bridgemill/5axisgui.py` 里，整台五轴摆头龙门铣就是三十来行：

```python
tool = Collection([HalTranslate([tooltip], c, "tool_length", 0,0,-1),
                   HalToolCylinder(c),
                   CylinderZ(pivot_len-(zb+za), 100, 0.0, 50),
                   Box(-100,-100,pivot_len-(zb+za), 100, 100,pivot_len-zb),
                   ...])
tool  = HalRotate([tool], c, "jb", 1,0,-1,0)      # B 轴
wrist = Collection([tool, Box(...), CylinderZ(...)])
wrist = HalRotate([wrist], c, "jc", 1,0,0,1)      # C 轴
ram   = Collection([wrist, Box(...)])
ram   = HalTranslate([ram], c, "jz", 0,0,1)       # Z
ram   = HalTranslate([ram], c, "jx", 1,0,0)       # X
ram   = HalTranslate([ram], c, "jy", 0,1,0)       # Y
```

注意 `pivot_len` 的注释：*"to agree with default in 5axiskins.c"*——**模型尺寸和运动学参数是同一个数**。这正是我们该抄的做法。

#### 我们的做法：配置即模型

完整设计见 [`ADR-007-机床模型表示.md`](ADR-007-机床模型表示.md)。要点：

机床模型是**三种表示、一份真相源**：

| 表示 | 用途 | 精度要求 |
|---|---|---|
| **运动学链** `Vec<Joint>` | FK/IK/RTCP | **f64 精确**（摆长错 0.1 mm 全盘错） |
| 显示外形 `Part::shape` | 渲染 | 厘米级，能认出来即可 |
| 碰撞代理 `Collider` | 碰撞检测 | **保守**（可大不可小） |

三者都挂在同一棵关节树上、用关节索引关联，**各自不存世界变换**——所以它们不可能与运动学脱节。这从结构上消灭了一整类 bug：外观说"还有 5 mm"而碰撞说"撞了"。

**碰撞代理必须外接**（circumscribe），内接会漏报。默认由外形 AABB 自动派生；L 形立柱、带圆鼻的主轴头这类异形件给 3–6 个凸体拼接，避免单个外接 OBB 过松导致大量误报。防护罩、拖链、线缆一律 `Collider::None`——撞了也不停机，参与碰撞只制造噪声。

**参数必须实测校准**：摆长、pivot 交点从手册装配图取名义值后要打表核对（1 小时流程见 ADR-007 §5）。**不要从下载的 CAD 模型反推 pivot**——拟合得出但精度和工时都不划算，错了极难发现。

6.5 人日（原估 1 人日只够"把方块拼出来"，补上了碰撞代理、6 种构型的配置与一致性测试）。

#### 想要好看时的升级路径

需要精细模型时，走 [`ADR-007`](ADR-007-机床模型表示.md) §8 的导入管线：

```
精细模型（glTF / STEP AP242）→ cncsim-import → .machine.ron + 引用 glTF 节点名
```

四个关键点：

1. **pivot 能不能自动提取，取决于源文件有没有装配信息**。STEP AP242（含 kinematics）和带装配约束的 CAD 原生文件有；AP203/214 只有装配树；平铺 STL/OBJ 什么都没有。**没有就必须打表量**——给错的 pivot 比让人手填更危险，因为手填时人会去量，自动生成的数值人不会怀疑。
2. **rest pose**：`local = inv(M_joint_world) · M_world` 里的关节世界变换必须是全轴归零时的值。模型在非零位姿导出会把偏移固化进去（C 转 30° 导出，之后转 30° 会显示 60°）。
3. **碰撞代理要聚类合并**：主轴头总成常有 50+ 个 leaf mesh，逐个生成 OBB 会让碰撞对爆炸。沿最长轴二分聚类到**每关节 ≤6 个 OBB**，在关节局部坐标系里做。
4. **先用方块跑通，再换精细模型**。精细模型会掩盖运动学错误——方块转错轴一眼看出，真实模型转错可能只是"某块钣金穿模"，容易被当成美术问题。换模型时只替换 `Shape`，运动学一行不动，然后对比两次的刀尖轨迹必须完全一致（1e-9）。

选 glTF 而不是 STL/OBJ：Bevy 内置 `bevy_gltf`。注意 glTF 节点变换**必须归零**（只保留几何），否则会和关节变换叠加，产生双重位移——这是导入外部模型最常见的错误。

---

### 6.8 切割面着色（按刀具/工序）

让用户直接看出"这块面是哪把刀切的"。完整设计见 [`ADR-005-切割面着色与溯源.md`](ADR-005-切割面着色与溯源.md)。

**本质是溯源问题，不是渲染问题。** 在 SDF 架构下几乎免费——毛坯表面本身就是"若干扫掠面的并集 + 原始毛坯面"，所以只要回答"这个点落在哪个扫掠面上"就得到了创建者：

```
creator(p) = 时间最晚的那个扫掠 t，使得 |sdf_t(p)| < 0.5h
```

三条设计决定：

1. **不按体素存 ID，按 brick 存候选表。** 每体素 `u8` 会让内存翻倍（130 → 260 MB）；按 brick 存 8 个候选只要 **2 MB**，提取时用解析 SDF 反查——反正 MC 每个三角形跨 8 个体素，仍需解析，提前做并没有省事。

2. **顶点属性存调色板索引，不存 RGB。** 切换"按刀具/按工序/单色"只改一张 256×1 纹理，**不重算仿真、不重传网格**。RGBA8 的 G 通道顺便编码余量偏差，于是四种模式共用同一份网格。

3. **着色模式要压掉金属反射。** PBR 的高光会让颜色随视角剧变，认不出是哪把刀。着色时把 `metallic` 降到 0、`roughness` 提到 0.8——Vericut 的着色模式看起来偏 unlit，不是偶然。

**交互才是价值所在**：点选毛坯表面 → 反查砖 → 反查创建者 → 显示"刀具 T7 / 第 3 序 / G 代码 N1240–N1380 / 12:34.5"，点击直接跳时间轴。这是"看到问题 → 定位到代码"的闭环。

6 人日。依赖 `cncsim-voxel`（需要 brick 结构与 MC 提取），排在材料去除内核之后。

---

## 7. 精度与碰撞

### 7.1 过切 / 残留

**已实现**（`sim::compare`，2026-10-30；设计模型见下）。口径与精度：

| 指标 | 方法 | 精度 |
|---|---|---|
| 过切/残留**体积** | 体素积分：毛坯场符号 × 设计 SDF 符号 | ± 一个表面层 |
| 过切**最大深度** | **解析扫掠点测试**：设计面采样点 × 每段扫掠 SDF | 掠切 ±5 µm，**不经过体素** |
| 残留**最大厚度** | 设计气区内材料体素到设计面的距离 + h/2 | ± h/2（0.05 mm 级 @ h=0.1） |
| 过切**区域** | 过切体素的包围盒 | 直接指认是哪个特征切多了 |

设计模型两种：**`Box`**（轴对齐盒，解析 SDF——铣平顶面/台阶/沉孔的基准）与
**`Field`**（STL → 采样 SDF，见 `voxel::meshvol`；要求水密网格）。
为什么过切深度不走体素、残留为什么不限工具走廊、盒 SDF 的角区语义，见 README「设计比对」节。

渲染上需要"高亮区域"，做法：

1. 从 `SimReport` 取比对结果（`gouge_region` 包围盒 + 体积；逐体素区域后续从 `compare` 取）
2. 在对应 tile 的网格上按顶点到设计曲面的有符号距离着色（红=过切，蓝=残留）
3. 或在 3D 视图叠加半透明的偏差云

v1 用**顶点色 + 独立材质**（不污染主材质），v2 可换偏差色谱图（类似 GOM 的彩虹图）。

### 7.2 碰撞检测

完整方案见 [`ADR-004-碰撞检查方案.md`](ADR-004-碰撞检查方案.md)。这里只列要点。

| 对象对 | 几何表示 | 判定方法 |
|---|---|---|
| **刀杆/刀柄表面 × 毛坯** | 旋转体表面采样（δ=0.1 mm） | **毛坯 SDF 符号**（主力，平面 1.6 µm / 曲面 2.0 µm） |
| 毛坯体素 × 刀杆/刀柄 | 体素 + 解析 SDF | 解析刀具 SDF 符号（兜底 + 定位，0.05 mm） |
| 夹具 / 台面 / 转台 / 主轴头 | OBB | **GJK 距离**（解析 support function，零误差） |
| 机床自身（滑枕 vs 转台等） | OBB 对 | GJK 距离 |

三个关键点：

1. **不需要通用网格碰撞。** 毛坯是体素不是网格，夹具与机床是凸体，刀具是旋转体——没有一对是"网格 vs 网格"，因此**不建 BVH**，粗筛用 AABB（n≈50，1250 次测试可忽略）。

2. **体素中心的距离值是精确的，但存成 `i8` 后只有 3.15 µm/档。** 毛坯 SDF 由解析距离的 CSG 结果算出（初始方块精确、每次切削 `max(d, -d_sweep)` 且 `d_sweep` 解析），**算出来的值是精确的**；但**存进 `i8` 的那一刻产生了 1.58 µm 的量化误差**。

   **所以有两个限制，都不是采样密度：**

   | 限制来源 | 数值（h=0.1） | 与 h 的关系 | 是否瓶颈 |
   |---|---|---|---|
   | 采样密度（方向 B） | `δ²/(2R)` = 0.17 µm | ∝ h² | 否 |
   | **`i8` 量化半档** | **1.58 µm** | ∝ h | **是（平面）** |
   | **三线性重建** `h²/(8R_c)` | **0.42 µm**（R_c=3） | ∝ h² | **是（曲面）** |

   **修正口径（三项分档）：平面 1.6 µm；曲面 R_c ≥ 0.8 mm 时 2.0 µm；R_c 更小时按 `1.6 + h²/(8R_c)` 增长**（φ0.5 球刀面 6.6 µm）。想更好就换 `i16`（2× 内存换 258× 量化精度）——**换 f64 零收益**，量化误差在存的时候就产生了。但注意：**小刀具精加工面的瓶颈是网格太粗，不是量化**，得先降 h。详见 [`计算精度分析.md`](计算精度分析.md) §2。

   采样方向仍然决定"采样密度"这一档（差三个数量级）：

   | 采样方向 | 保证阈值 | 原因 |
   |---|---|---|
   | 毛坯体素 → 刀柄 SDF（兜底/定位） | **0.05 mm**（h/2） | 穿透区是三维薄片；表面若卡在两个体素层中间，ε < h/2 时一个体素都不在里面 |
   | **刀柄表面 → 毛坯 SDF（主力检出）** | 采样密度 0.17 µm，**实际是平面 1.6 µm / 曲面 2.0 µm** | 穿透区在刀柄表面是**二维补丁**，采样不是瓶颈；瓶颈是 `sdf_stock` 的符号只有量化半档 + 重建误差的可信度 |

   同一个相交判定，换采样域就从毫米级变成亚微米级——因为二维补丁只需 `ε > δ²/(2R)`，三维薄片却要 `ε > h`。**方向 B 还有个额外好处：它只问"点在毛坯内"，窄带 0.4 mm 就够，不需要为覆盖刀柄半径去加宽带宽（那会让内存 ×75）。**

   夹具/台面/机床那一类用**解析 support function 的 GJK**，误差为零。

   **h 可调**（`GridConfig { h, band_voxels }`，加载期参数）。一个旋钮同时控制四件事：内存 ∝ 1/h²、速度 ∝ 1/h²、采样密度 ∝ h²（方向 B 是 `h²/8R_c`）、**量化 ∝ h**。**后两者取更紧的：h ≤ 0.1 时量化主导，总保证只按 h 线性改善**——h 减半内存 4 倍，误差只降一半。三个预设——快速 0.2 / 标准 0.1 / 精密 0.05 mm，也可由内存上限自动反推 `h = √(band·A/mem)`。矩阵见 [ADR-004](ADR-004-碰撞检查方案.md) §2.5.8。

   注意：可调**不能规避**薄壁问题——零件有 0.2 mm 以下的筋就必须选 h ≤ 0.05，可调只是让你按任务选，不是让粗网格也能覆盖细特征。

   三个前提缺一不可：① SDF 值的符号必须可信（钳制安全性已证）；② 扫掠 SDF 的**黄金分割迭代次数必须按行程自适应（6–24 次）**——短段 6 次误差 <1 µm，200mm 长段固定 6 次会到 16 µm（实测），下限不要降到 2 次；③ **`Full` brick 不能跳过**——否则刀杆深埋实体时会整段漏报。

3. **必须做保守推进，不能只靠离散采样。** 100 m/min 快移 = 1.67 mm/ms，2 mm 薄壁会被整步跳过——漏报比没有检测更危险。用距离下界算安全步长：

```rust
fn earliest_collision(pair, t0, t1, d: impl Fn(f64) -> f64, v: f64) -> Option<f64> {
    let mut t = t0;
    for _ in 0..64 {
        let dist = d(t);
        if dist < 1e-6 { return Some(t); }
        let dt_safe = dist / v;            // v 取该段任意点的最大速度上界
        if t + dt_safe >= t1 { return None; }
        t += dt_safe.max(1e-5);
    }
    Some(t)                                 // 迭代耗尽，保守报碰撞
}
```

`v` 不能用刀尖速度——五轴摆动时刀柄末端速度可达 `|v_tip| + |ω| × r_max`，远大于刀尖。取上界才能保证不漏报。

**性能关键**：GJK 是主要开销（100k 步约 6 秒）。用"关节组合 → 可能接近的部件对"查表预筛，通常能砍掉 80%。

**安全间隙**（v1.5）：窄带 SDF 的 band 只有 0.4 mm，无法回答"距离是否小于 5 mm"。加一层 2 mm 分辨率的粗欧氏距离场（约 2 MB，Felzenszwalb 可分离 EDT，60 行）即可。

**毛坯演化**（ADR-004 §5，最容易漏的一条）：碰撞必须随加工逐步评估——去除产生新型腔壁，刀柄可能撞上 1 秒前还不存在的面。三条要点：

1. **顺序：先去除、后碰撞**，否则螺旋下刀进型腔时会撞上"本段即将消失的材料"（误报）。
2. **brick 状态必须重分类**：被切空的 `Band`/`Full` brick 要降级为 `Empty`。一块切空却仍是 `Full` 的砖会让碰撞引擎把"穿过空腔的刀杆"持续判为碰撞。`Empty` 砖可永久跳过（只去料不加料，单调性）。
3. **段内不一致性可以忽略，且有严格理由**：保守推进在段内亚采样，但毛坯只有段级更新。不过——切移动时刀尖进给只有 0.05 mm/ms，毛坯变化极慢；快移时（1.67 mm/ms，才有穿透风险）毛坯**完全不变化**。两种效应不同源，不会同时放大。兜底 margin 取 `feed_per_step × 2 + h`。

### 7.3 报告

仿真结束输出结构化报告（JSON + 可打印 PDF 后续）：

```rust
pub struct SimReport {
    pub program: PathBuf,
    pub machine: String,
    pub total_blocks: usize,
    pub programmed_time_s: f64,
    pub actual_time_s: f64,
    pub removed_volume_mm3: f64,
    pub gouge_volume_mm3: f64,
    pub excess_volume_mm3: f64,
    pub collisions: Vec<Collision>,
    pub singularities: Vec<SingularityEvent>,
    pub nonlinear_violations: Vec<NonlinearError>,
    pub axis_limit_violations: Vec<AxisViolation>,
    pub max_feed_deviation_pct: f64,
}
```

---

## 8. 测试策略（TDD）

> 项目约定：**测试先行，红-绿-重构**。禁止先写实现再补测试。每个测试必须能被证伪——断言要具体到能区分"做对了"和"做错了"。

### 8.1 分层测试矩阵

| 层 | 测试类型 | 工具 | 运行时间 |
|---|---|---|---|
| core / kin | 单元 + 性质 | `#[test]` + `proptest` | < 100 ms |
| gcode | 单元 + 表驱动 | `#[test]` | < 50 ms |
| sim | 集成 + 金标准 | `#[test]` + 基准数据 | 秒级（`--release`） |
| render | headless 冒烟 | `App::new().add_plugins(MinimalPlugins)` | < 1 s |
| 端到端 | 场景回放 | 脚本 + 报告快照 | 分钟级（CI 夜间） |

### 8.2 运动学：往返性质测试（最重要）

```rust
proptest! {
    /// FK ∘ IK = 恒等：任意可达位姿，逆解后再正解必须回到原位姿
    #[test]
    fn ik_fk_roundtrip_ac_head(
        a in -100.0f64..100.0,
        c in -180.0f64..180.0,
        x in -500.0f64..500.0,
        y in -300.0f64..300.0,
        z in -400.0f64..0.0,
    ) {
        let m = MachineTopology::ac_head();
        let q = [x, y, z, c, a];
        let (_, tip0, axis0) = forward(&m.chain, &q);
        let q2 = inverse(&m.chain, tip0, axis0, q).unwrap();
        let (_, tip1, axis1) = forward(&m.chain, &q2);
        prop_assert!((tip1 - tip0).length() < 1e-9, "tip drift {}", (tip1 - tip0).length());
        prop_assert!((axis1 - axis0).length() < 1e-9, "axis drift");
    }
}
```

这个测试**必须先在只有 `unimplemented!()` 的状态下写**，确认它因为"功能不存在"而失败（红），再写 FK，再写 IK，逐层转绿。

### 8.3 具体数值锚点测试

性质测试抓不到"整体符号反了但自洽"的错误（FK 和 IK 一起错会互相抵消）。必须加**绝对数值锚点**：

```rust
#[test]
fn ac_head_axis_at_known_angles() {
    let m = MachineTopology::ac_head();
    // A=0 时刀轴竖直
    assert_vec3!(axis_of(&m.chain, &[0.0, 0.0, 0.0, 0.0, 0.0]), [0.0, 0.0, 1.0]);
    // A=90°, C=0 → 刀轴沿 -Y
    assert_vec3!(axis_of(&m.chain, &[0.0, 0.0, 0.0, 0.0, 90.0]), [0.0, -1.0, 0.0]);
    // A=90°, C=90° → 刀轴沿 +X
    assert_vec3!(axis_of(&m.chain, &[0.0, 0.0, 0.0, 90.0, 90.0]), [1.0, 0.0, 0.0]);
}
```

这三个锚点来自 §4.2 的手工推导，与实现无关——这才是独立的校验。

### 8.4 RTCP 不变量测试

```rust
#[test]
fn rtcp_keeps_tip_on_straight_line() {
    let m = MachineTopology::ac_head();
    let p0 = DVec3::new(0.0, 0.0, 50.0);
    let p1 = DVec3::new(100.0, 0.0, 50.0);
    for i in 0..=20 {
        let t = i as f64 / 20.0;
        let a = -30.0 + 60.0 * t;                        // A 从 -30° 摆到 +30°
        let tip_target = p0.lerp(p1, t);
        let q = rtcp_inverse(&m, tip_target, DVec3::Z, a, 0.0);
        let (_, tip, _) = forward(&m.chain, &q);
        // 到直线 p0→p1 的距离必须 < 1 µm
        let dev = ((tip - p0).cross((p1 - p0).normalize())).length();
        assert!(dev < 1e-3, "t={t} deviation={dev}");
    }
}
```

### 8.5 G 代码：表驱动 + 状态迁移

```rust
#[test]
fn modal_distance_mode() {
    let mut st = ModalState::default();
    // G91 之后，X10 是增量
    let b1 = parse_block("G91 X10.", &mut st);
    let b2 = parse_block("X10.", &mut st);
    assert_eq!(b1.target, vec![("X", 10.0)]);
    assert_eq!(b2.target, vec![("X", 20.0)]);          // 累积
    // G90 复位
    parse_block("G90 X5.", &mut st);
    let b3 = parse_block("X10.", &mut st);
    assert_eq!(b3.target, vec![("X", 10.0)]);          // 绝对
}
```

### 8.6 材料去除：金标准体积

```rust
#[test]
fn slot_removed_volume_matches_analytic() {
    // 60×40×20 毛坯，φ6 平底刀，在 z=15 处铣一条 40mm 长的槽
    let mut engine = Engine::new(ManifoldKernel, &[stock], opts).unwrap();
    engine.cut(&tool, &slot_segment).unwrap();
    let expected = 6.0 * 40.0 * (20.0 - 15.0);          // = 1200 mm³
    let actual = engine.total_removed_volume();
    assert!((actual - expected).abs() / expected < 0.005, "vol {actual} vs {expected}");
}
```

容差 0.5%——`cutrail` 官方数据是 0.3%，留一点余量。

### 8.7 基准测试（criterion）

```rust
fn bench_removal(c: &mut Criterion) {
    c.bench_function("cut_1mm_segment_10mm_tiles", |b| {
        b.iter(|| engine.cut(&tool, &seg))
    });
}
```

CI 里设阈值：单段 > 15 ms 则报警（相对基线回归 > 2×）。

### 8.8 测试数据管理

```
testdata/
├── gcode/
│   ├── ac_head_finish.nc          # 五轴精加工，含摆动
│   ├── rtcp_off.vs.on/            # 同一零件的 RTCP 开/关对照
│   ├── singularity_cross.nc       # 故意穿越 A=0
│   └── collision_expected.nc      # 已知会撞的坏程序
└── golden/
    ├── ac_head_finish.report.json # 体积/时间/诊断的基准值
    └── ac_head_finish.pose.csv    # 逐段位姿基准（数值回归）
```

金标准数据**由人工审核后提交**，CI 里比对。位姿基准用 1e-6 容差，体积用 0.5%。

### 8.9 用 LinuxCNC 做验证 oracle

运动学正确性是这个项目最难保证的部分：FK 写错、符号反了、摆长基准错了，画面**全都看起来正常**。手算锚点（§8.3）只能覆盖几个位姿，抓不到"大部分区间都对、只在某个象限错"这类问题。

自用场景下有一个免费的权威对照：**LinuxCNC 的 `5axiskins.c` + vismach 配置已经在真实机床上跑了二十年**。

做法：

1. 挑一个与你机床构型一致的 vismach 配置（§6.7 列了 5 个）。
2. 同一份 G 代码分别喂给 LinuxCNC 仿真配置和自己的程序，导出**逐插补周期的 5 个关节值**。
3. 逐点 diff。

```rust
#[test]
fn matches_linuxcnc_reference_trajectory() {
    let ours = run_ours("testdata/gcode/ac_head_finish.nc");
    let refr = load_csv("testdata/golden/linuxcnc_ac_head_finish.csv");
    assert_eq!(ours.len(), refr.len(), "插补周期数不一致，先查 SERVO_PERIOD");
    for (i, (a, b)) in ours.iter().zip(&refr).enumerate() {
        for j in 0..5 {
            assert!(
                (a.q[j] - b.q[j]).abs() < 1e-6,
                "block {i} joint {j}: {} vs {}", a.q[j], b.q[j]
            );
        }
    }
}
```

三个必须对齐的东西：

- **插补周期**：LinuxCNC 默认 `SERVO_PERIOD = 1 ms`。周期不同则点数不同，diff 无意义。
- **单位与符号约定**：这是最容易出错的地方，也正是这个测试的价值——符号一反，diff 立刻炸。
- **只对运动学 diff，不要对材料去除 diff**：两者算法完全不同（网格布尔 vs 体素），体积差异是设计内的，不是 bug。

成本约 3 人日（导出脚本 + 对齐 + 首个基准文件）。**这是自用场景下投入产出比最高的一件事**——它把"运动学是否可信"从主观判断变成一条会红的测试。

---

## 9. 里程碑

| 阶段 | 内容 | 交付物 | 估时 |
|---|---|---|---|
| **M0** 骨架 | workspace 分层、Bevy 空场景、egui 空面板、CI | `cargo run` 出窗口 | 3 人日 |
| **M1** 运动学 | FK/IK/RTCP/奇异点 + 完整测试 | `cncsim-kin` 全绿，6 构型配置 | 8 人日 |
| **M2** 三轴闭环 | G 代码解析（3 轴子集）+ cutrail 集成 + 毛坯渲染 | 能跑通一个 3 轴零件的完整仿真 | 10 人日 |
| **M3** 五轴联动 | G43.4/G68.2 + 五轴插补 + 关节动画 | 五轴零件可视化正确 | 12 人日 |
| **M4** 验证能力 | 碰撞、过切/残留、非线性误差、节拍 | 结构化报告 | 12 人日 |
| **M5** 打磨 | 剖切、刀路着色、时间轴、性能优化、打包 | v1.0 | 15 人日 |

合计约 **159.5 人日**（自研版，单人 7.1 个月）。其中材料去除内核 45、自研碰撞 21、G 代码解释器 21、机床模型 11（含精细导入）、切割面着色 6、HMI 6、稀疏树 3、SIMD 与零分配 3、点位缓冲 4——逐项见 [ADR-003](ADR-003-自研技术栈边界与替代方案.md) §7。不含美术资源本身。

**若分两步走**（ADR-003 §6）：第一步自研数学/运动学/G代码/碰撞四层、材料去除暂借 cutrail 当脚手架兼测试预言机，约 **86 人日**就能有可用工具；第二步再替换材料去除（+45 人日）。**建议按这个顺序**——先跑通，再拆桥。

**M1 是硬门槛**：运动学测试不全绿，不进 M2。这是唯一一个不允许压缩的阶段——**即使自用也不能砍**，因为后面所有环节都建立在它之上。

---

## 10. 关键设计决策（ADR 摘要）

| # | 决策 | 理由 | 代价 |
|---|---|---|---|
| D1 | 用 Bevy 0.20 而非 Web 技术栈 | 几何内核与业务逻辑同语言，测试成本低 | 每季度一次破坏性升级 |
| D2 | 材料去除用 `cutrail` 网格布尔，不自研 | 自研 6–12 人月且正确性风险高 | 依赖 0.1.x，API 可能变 |
| D3 | "先算后播"而非实时 | 五轴网格布尔物理上快不过真实加工 | 首帧有等待，需进度条 |
| D4 | 每块毛坯一个实体一个 Mesh | 增量更新，避免整网格重传 | 实体数多（数千），ECS 开销 |
| D5 | 运动学用"拓扑数据 + 数值逆解"而非硬编码公式 | 12 构型 12 份符号陷阱代码不可维护 | 逆解多几微秒（可忽略） |
| D6 | **三档精度**：标量 f64 / SIMD 热路径 f32 / 索引 i32 | 标量 f64 在 x86-64 上几乎免费（同吞吐）；SIMD 热路径 f32 宽 2×，而精度已被 `i8` 量化限死 | 边界处需显式转换；见 [`计算精度分析.md`](计算精度分析.md) |
| D7 | 非线性误差默认按"无补偿"模拟 | 最坏情况，判断点位密度是否足够的正确口径 | 与部分高端控制器的实测有差异 |
| D8 | v1 只做 Fanuc 方言 | 覆盖面最广，一套够用 | Siemens/Heidenhain 用户用不了 |

---

## 11. 风险与对策

| # | 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|---|
| R1 | ~~`cutrail` 0.1.x API 变动~~ | — | — | **已消解**：自研路线下 cutrail 仅作第一步的脚手架与测试预言机，通过 `RemovalEngine` trait 隔离，第二步完成后整体移除（ADR-003 §6） |
| R2 | Bevy 0.20 WESL 迁移，自定义材质写不出来 | 中 | 中 | M0 阶段先做一个最小 `ExtendedMaterial` 冒烟，确认 `.wesl` 写法；不要把材质留到 M5 |
| R3 | 网格布尔在密集刀路下内存/时间爆炸 | 中 | 高 | 分块 + 段合并（`SegmentBuilder` 的弦高容差）+ 分段容差（粗加工用 0.05 mm，精加工 0.01 mm）；设"最多 N 段后强制简化"上限 |
| R4 | WASM 目标不可行 | 中 | 低（v1 不做 Web） | 不承诺 Web；若要做，先测 `manifold-rust` 的 WASM 体积与性能 |
| R5 | 逆解在奇异点附近振荡 | 高 | 中 | Newton 加阻尼 + 步长上限 + 奇异点检测后"冻结"该轴 |
| R6 | 碰撞检测拖慢整体 | 中 | 中 | AABB 粗筛（不建 BVH）+ "关节组合 → 可能接近的部件对"查表预筛，通常砍掉 80% 的 GJK 调用（ADR-004 §8） |
| R11 | 快移跳过薄壁导致**漏报**（比没有检测更危险） | 中 | 高 | 保守推进（conservative advancement），用距离下界算安全步长；`v` 取该段任意点速度上界而非刀尖速度；迭代耗尽时保守报碰撞（ADR-004 §3） |
| R7 | 机床可视化模型缺失导致"看不清在干什么" | 高 | 低 | 参数化生成，不依赖外部模型（§6.7）：遍历拓扑链为每个关节产出一个 Box/Cylinder，按轴着色。LinuxCNC vismach 证明整台五轴机床三十行就够 |
| R8 | G 代码方言差异导致解析失败率高 | 高 | 中 | 解析失败时**不中断**，跳过该块并记录到诊断列表；UI 高亮无法解析的行 |
| R9 | 误用第三方资产引发许可纠纷 | 低（自用） | 低 | GPL 私有使用不触发分发义务，可直接复用 LinuxCNC 实现；但 GrabCAD 的"非商用"条款与分发无关，公司商用仍适用 → 一律参数化生成（§6.7） |
| R10 | 日后想把工具给同行用 | 中 | 中 | 自用期直接复用 GPL 代码会留下"要分发就得开源"的债。**若要保留分发的可能，运动学自己写（本来就要写）、只把 LinuxCNC 当测试 oracle（§8.9）**——测试用 GPL 程序不污染被测代码 |

---

## 12. 起步建议：前三个 TDD 循环

如果现在就动手，第一天应该写这三个测试（全部先红）：

**循环 1 — 关节变换**
```rust
#[test]
fn single_rotary_joint_rotates_about_its_axis() {
    // 一个绕 Z 转 90° 的关节，把 (1,0,0) 变成 (0,1,0)
    // 先写 assert，跑 cargo test 看到 "not yet implemented"
}
```

**循环 2 — AC 摆头刀轴**
```rust
#[test]
fn ac_head_tool_axis_at_a90_c0() {
    // assert 刀轴 == (0,-1,0)
}
```

**循环 3 — RTCP 不变量**
```rust
#[test]
fn rtcp_tip_stays_on_line_while_a_swings() {
    // assert 偏差 < 1e-3
}
```

循环 3 会失败得最"有价值"——它会逼你搞清 `tool_length` 到底是从刀尖量到主轴端面，还是量到回转中心。这个量搞错 50 mm，所有仿真结果整体偏移 50 mm，而画面看起来完全正常。**这就是为什么这个项目必须 TDD。**

---

## 13. 参考

**引擎**
- Bevy 0.20 发布公告：https://bevyengine.org/news/bevy-0-20/
- Bevy 0.19 发布公告：https://bevyengine.org/news/bevy-0-19/
- Bevy 示例（含 Pan Orbit Camera / Infinite Grid / Transform Gizmo）：https://bevyengine.org/examples
- `bevy_egui` 0.43（对应 Bevy 0.20）：https://lib.rs/crates/bevy_egui

**几何内核**
- `cutrail`：https://lib.rs/crates/cutrail ・ https://github.com/botrail/cutrail
  - `docs/architecture.md`（为什么毛坯用网格）
  - `docs/accuracy-and-performance.md`（精度控制与实测性能）
  - `docs/brep-export.md`（解析 B-rep 重建）
- `manifold-rust`（网格布尔）：https://github.com/larsbrubaker/manifold-rust
- 碰撞检测：见 [`ADR-004-碰撞检查方案.md`](ADR-004-碰撞检查方案.md)（含 GJK、保守推进、五轴碰撞文献）
- `truck`（STEP 读写）：https://github.com/ricosjp/truck

**五轴理论与标准**
- ISO 10791-7：加工中心试验条件 —— 五轴机床构型分类与精度检验
- Chen-Hua She, Zhao-Tang Huang, *Postprocessor development of a five-axis machine tool with nutating head and table configuration*, Int J Adv Manuf Technol, 2008 —— 摆头+转台构型的后处理推导
- 五轴 RTCP / TCPM 的控制器文档：Fanuc B-64484EN（G43.4）、Siemens 840D 编程手册（TRAORI）、Heidenhain TNC 640（TCPM）

**同类实现**
- CAMotics（开源 3 轴，Dexel）：https://camotics.org
- MachineWorks（商用，网格+混合）：https://www.machineworks.com
- CGTech Vericut（商用，事实标准）

---

## 附录 A：AC 摆头运动学推导

设 C 轴为外环（绕机床 Z），A 轴为内环（绕机床 X，随 C 一起转）。刀具系 `+Z` 为刀轴（指向主轴）。

变换链：`R = Rz(C) · Rx(A)`

```
Rx(A) · ẑ = (0, -sinA, cosA)ᵀ
Rz(C) · (0, -sinA, cosA)ᵀ = ( sinA·sinC,  -sinA·cosC,  cosA )ᵀ
```

逆解（`n` 为单位向量）：

```
A = arccos(nz)              ∈ [0°, 180°]（受机床限位约束，通常 [-110°, 110°]）
sinA = √(1 - nz²)
C = atan2(nx, -ny)          （因 nx = sinA·sinC, -ny = sinA·cosC）
```

退化：`sinA → 0` 时 C 无定义（奇异点）。

## 附录 B：BC 摆头（对照）

`R = Rz(C) · Ry(B)`：

```
n = ( sinB·cosC,  sinB·sinC,  cosB )ᵀ
B = arccos(nz)
C = atan2(ny, nx)
```

注意 AC 与 BC 的 C 表达式差一个符号结构（`atan2(nx, -ny)` vs `atan2(ny, nx)`），这正是"抄公式最容易抄错"的地方——用 §8.3 的锚点测试分别锁定。

## 附录 C：术语

| 术语 | 含义 |
|---|---|
| RTCP | Rotational Tool Center Point，刀尖点跟随。程序坐标即刀尖坐标 |
| TCPM | Tool Centre Point Management，Siemens/Heidenhain 对同一能力的叫法 |
| TWP | Tilted Work Plane，倾斜工作平面（G68.2），把编程坐标系倾斜到斜面 |
| 3+2 定位 | 两旋转轴只定位不联动，加工时静止 |
| 五轴联动 | 五轴同时插补，刀轴连续变化 |
| 非线性误差 | 关节空间线性插补导致的刀尖轨迹偏离直线的量 |
| 摆长 | 回转中心交点到主轴端面（或刀尖）的距离，RTCP 的核心参数 |
| 过切 | 切掉了设计模型以内的材料（gouge） |
| 残留 | 应该切掉但没切到的材料（excess） |
