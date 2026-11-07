# Rust CNC/CAM 软件技术调研与架构设计报告

> **项目定位**:输入 STL / STEP 文件 → 生成 G-code(三轴 / 3+2 定位五轴 / 五轴连续 RTCP)→ 内置 CNC 仿真(材料去除 + 机床运动)+ 加工结果与设计模型的差异对比验证。
> **技术路线**:Rust 编写;**核心算法(解析、几何、刀路、G-code、仿真、对比)全部自行实现,不依赖现成算法库**;仅窗口/渲染/并行/序列化等基础设施使用成熟 crate。
> **调研时间**:2026-11-06 ~ 2026-11-07(第 1~14 章为初版调研,第 15~22 章为补充调研)。除标注"未验证"外,所有事实均经实际访问来源页面核验;crate/项目状态以该日期为准。

---

## 目录

- [0. 执行摘要](#0-执行摘要)
- [1. 范围与总体架构](#1-范围与总体架构)
- [2. 输入模块:STL 解析(自研规格)](#2-输入模块stl-解析自研规格)
- [3. 输入模块:STEP 解析(自研分阶段路线)](#3-输入模块step-解析自研分阶段路线)
- [4. 几何基础库(自研清单)](#4-几何基础库自研清单)
- [5. 刀具路径生成算法(自研)](#5-刀具路径生成算法自研)
- [6. G-code 方言、解析与生成(自研)](#6-g-code-方言解析与生成自研)
- [7. 运动规划仿真(grbl 模型自研)](#7-运动规划仿真grbl-模型自研)
- [8. 材料去除仿真(自研:Z-map → SDF)](#8-材料去除仿真自研z-map--sdf)
- [9. 模型差异对比 / 验证(自研)](#9-模型差异对比--验证自研)
- [10. UI 与渲染(基础设施选型)](#10-ui-与渲染基础设施选型)
- [11. 测试与工程化](#11-测试与工程化)
- [12. 开源项目参考对照表](#12-开源项目参考对照表)
- [13. 路线图](#13-路线图)
- [14. 风险清单](#14-风险清单)
- [15. 特征自动识别(自研路线)](#15-特征自动识别自研路线)
- [16. 切削参数库、刀具库、刀柄与装夹](#16-切削参数库刀具库刀柄与装夹)
- [17. G-code 传输与探测(串口发送/探针/调平)](#17-g-code-传输与探测串口发送探针调平)
- [18. 2D 输入:DXF 与 SVG](#18-2d-输入dxf-与-svg)
- [19. 商业竞品与市场格局](#19-商业竞品与市场格局价格-2026-11-核验)
- [20. UI/UX 信息架构与桌面工程化](#20-uiux-信息架构与桌面工程化)
- [21. NURBS 数学与刀具接触公式(深潜)](#21-nurbs-数学与刀具接触公式深潜)
- [22. 性能基准与目标](#22-性能基准与目标)
- [23. 五轴机床运动学与 RTCP](#23-五轴机床运动学与-rtcp)
- [24. 五轴与 3+2 刀路、碰撞与仿真](#24-五轴与-3-2-刀路碰撞与仿真)
- [25. 来源汇总](#25-来源汇总)

---

## 0. 执行摘要

**核心结论(一句话)**:这款软件在 Rust 生态里几乎没有可直接复用的 CAM 算法资产,但每个子问题都有公开的算法论文/规范/参考源码可对照实现;最大的风险点是 **STEP 精确 B-rep(NURBS 曲面)** 和 **网格布尔鲁棒性**,两者都应通过"分阶段 + 绕行策略(网格档/切片法/SDF)"规避,而不是硬啃。

**自研原则下的依赖边界**:

| 类别 | 处理方式 | 说明 |
|---|---|---|
| 几何/刀路/G-code/仿真/对比算法 | **全部自研** | 本报告第 2~9 章均给出"照着写"的规格与文献 |
| 解析器(STL/STEP/G-code) | **手写递归下降/状态机** | 三种格式语法都简单,手写比引解析库更可控 |
| 数学基元(Vec3/Mat4/Quat) | **自研**(约 500 行) | 贯彻自研原则成本极低;glam/nalgebra 列为备选 |
| 窗口/GPU/GUI(winit、wgpu、egui) | 允许使用 | 基础设施,非算法库;自研窗口/图形栈无意义 |
| 并行/IO/序列化(rayon、memmap2、serde、thiserror、tracing) | 允许使用 | 同上 |
| 测试/打包(criterion、proptest、cargo-dist) | 允许使用 | 开发工具链 |
| 现成算法库(Clipper2、opencamlib、CAMotics、fidget…) | **只读源码学习,不链接** | 注意许可证(GPL 系源码严禁抄码,见 §14) |

**关键技术选型结论**:

1. **STL**:二进制 50 字节定长记录流,`memmap2` + 分块并行解析;强制交互询问单位(STL 无单位)。
2. **STEP**:自研 ISO 10303-21 解析器完全可行;几何转换分四档走——`TESSELLATED_*` / `FACETED_BREP` 网格档最先做,解析曲面(平面/柱/锥/球/环)其次,NURBS 最后;测试用 NIST 官方模型集。
3. **2D 布尔与偏置**:自研 Vatti 裁剪算法(Clipper2 是最佳学习参照,BSL-1.0 许可宽松,但其上游主仓库 2026-10 已归档,更应自研)。
4. **刀路 MVP**:轮廓偏置 + 等距环切挖槽 + 钻孔排序 + 平行栅格 drop-cutter 精加工 + 切片式粗加工 + 圆弧拟合;摆线/自适应清角、morphed spiral 放二期。
5. **G-code**:以 LinuxCNC RS274/NGC 文档为规范母本,grbl v1.1 为首个目标方言;自研"词法 → Block → 模态组状态机 → 指令 IR"四层。
6. **仿真时序**:自研 grbl 式规划器(junction deviation + 梯形剖面 + 双向传播),公式全部闭式,无需数值积分。
7. **材料去除**:MVP 用 Z-map 高度场(2.5D,三轴覆盖 90% 场景);二期 SDF(闭式扫掠体公式现成)+ Marching Cubes;**注意:"CAMotics 用 tri-dexel"是误传**,其真实架构是采样场 + 移动索引八叉树 + 扫掠深度函数 + MC33,已源码实证。
8. **差异对比**:L0 高度场逐列对比(周级工作量)→ L1 采样点-网格有符号距离 + 色图(点-三角形用 Eberly 算法,符号判定用广义环绕数)→ L2 双向 Hausdorff + 统计。
9. **UI**:egui/eframe(0.34 起 egui-wgpu 为默认后端)+ 自研 wgpu 渲染层;不用 bevy(过重)。
10. **补遗结论(第 15~22 章)**:特征识别走"选面半自动 → 规则自动(平面/圆柱分割 + 孔检测)"三级;切削参数库用"基准值+修正+来源标注"的区间模型;2D 输入 DXF 优先(bulge→圆弧公式);发送器默认字符计数协议(127 字节预算);竞品格局显示最稀薄档位为 $500–2500 的"真 3D + 现代 UI + 买断";NURBS 与圆鼻/锥刀 drop-cutter 公式已达到可照抄实现的深度;性能目标以公开基准外推并纳入 CI 回归。
11. **五轴升级(2026-11-07 追加,第 23~24 章)**:正式支持三轴 / 3+2 / 五轴连续(RTCP)三条加工路线;仿真从 Z-map 起步,但 **tri-dexel/SDF 优先级上调**(3+2 需多方位 Z-map 合并、五轴连续必须三向 dexel/体素);运动学按机床构型抽象(转台-转台 BC / 摆头-摆头 AC / 摆头-转台),输出端支持 **RTCP 开(控制器解算)/ 关(CAM 侧预变换)双模式**;grbl 系无旋转轴运动学,五轴发送目标收窄为 LinuxCNC(自定义 kinematics)或工业控制器;分级落地 A(3+2)→ B(五轴连续)→ C(swarf/机床全碰撞)。
12. **刀路自动生成(2026-11-07 补查,§5.7~5.10)**:两条自动化路线——纯几何一键流水线(MeshCAM 式:4 项必答输入 + 双角度阈值 Unified Finishing + 余料高度场接力;对标解剖含向导脚本级证据)与特征驱动工艺规划(CAPP 规则引擎:特征→操作映射表 + TAD 聚类 setup + 刀具聚集排序 + 公差驱动参数;CAMWorks TBM 为对标);曲面精加工增量首选 iso-scallop 行距调度(Suresh-Yang 框架,网格主曲率已有)与自动 pencil(z-map 残留法);AI 判断:规则引擎为主,LLM 仅作规则编纂助手,RL 不依赖。
13. **表面质量评价(2026-11-07 判定,§9.11)**:只做 A/B 两层——ISO 25178 参数计算器(A,挂 §9.10 三视图,~1 周)与加工前理论粗糙度预测(B,解析模型 + 可选局部高分辨小块,~2–4 周,联动目标 Sa 反解 stepover);**C 层(颤振/烧伤/ML 预测)为非目标**;关键边界:**主仿真网格不得用于 µm 级 Sa 计算**(量化淹没),主路径 = 解析模型。
14. **刀路质量补强与系统误差(2026-11-07 补查,§5.11/§9.12)**:进给自适应调度(恒 MRR 几何调制起步,逐点啮合查询复用仿真毛坯,三遍平滑;开源空白=差异点)、两级安全链接(skim/clearance + 三段折线 + 采样校验)、三轴方向投票(Lin & Koren 判据统一式:v1 投票/v2 PFD 分区)、可侧铣度检测器(可展双条件:角亏+κmin 母线;商业网格直检空白);**零点偏置:先诊断 h 标度(一阶约定 bug vs 二阶算法),再标定-补偿**(让刀典型 1–10µm < 格宽,v1 只警示);热变形/间隙/薄壁变形 = 报告免责项。
15. **刀路质量补强 II(2026-11-07 补查,§5.12/§5.13/§21.4)**:桶刀/圆弧段(行距增益 √(R_b/R_球),实测 70–95% 时间节省;sd_profile 环面片段复用圆鼻代码;3+2 原型先行,5 轴切触分配二期;纠偏:Autodesk "Generalized Machining" 论文查无实据);**光顺过渡与 JD 同几何**(r=T·cos(β/2)/(1−cos(β/2)),CAM 预插圆角减少 JD 限速点,实测 >26% 提速;双容差模型;样条仅后置可选);薄壁策略规则表(交替侧壁/渐进 RDOC/顺铣一致,Harvey 全文验证)。

---

## 1. 范围与总体架构

### 1.1 端到端流水线

```
┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│ STL/STEP │→ │ 几何预处理 │→ │ 工艺与刀路 │→ │ G-code    │→ │ 仿真      │
│ 文件解析 │   │ 焊接/修复  │   │ 生成      │   │ 后处理    │   │ 运动+去除 │
└─────────┘   └──────────┘   └──────────┘   └──────────┘   └────┬─────┘
                                                                 │
   ┌────────────────────────┐   ┌──────────────────────┐   ┌───▼──────┐
   │ 输出:G-code 文件/发送器 │← │ 验证:差异对比/统计    │← │ 回放渲染  │
   └────────────────────────┘   └──────────────────────┘   └──────────┘
```

各阶段之间的数据契约(即核心数据模型,全部可序列化、可 diff,便于测试):

```rust
// 伪代码骨架
Mesh        { verts: Vec<Vec3<f64>>, tris: Vec<[u32;3]>, meta }
Tool        { kind: Flat|Ball|Bull{rc}|Cone{angle}, dia, flute, ... }
MachineConf { axes: {max_vel, max_acc, limit}, jd, arc_tol, dialect }
Op          { kind: Profile|Pocket|Drill|Level|Rough|Waterline|Parallel, params }
Toolpath    { segments: Vec<Seg> }   // Seg = Line{p0,p1,feed} | Arc{..,cw} | Probe|Dwell
GcodeDoc    { lines: Vec<String>, ir: Vec<Instruction>, dialect: DialectProfile }
Plan        { blocks: Vec<Block> }   // 运动规划块(含 entry/exit/peak 速度)
SimResult   { heightmap | sdf_field, timeline: Vec<(t, ToolPose)>, stats }
Deviation   { samples: Vec<(pos, signed_dist)>, stats, classes: Gouge|Excess|Ok }
```

### 1.2 Cargo Workspace 划分

```
cnc/
├── crates/
│   ├── math        # Vec2/3/4, Mat4, Quat, AABB, Ray, Plane —— 自研
│   ├── geometry    # Mesh、BVH、2D clip/offset(Vatti)、修复、SDF、 Marching Cubes
│   ├── io-stl      # STL 读/写
│   ├── io-step     # ISO 10303-21 解析 + 实体图 + 曲面网格化
│   ├── cam         # 刀具/操作/刀路算法
│   ├── gcode       # 方言 profile、解析器、生成器、校验器
│   ├── motion      # grbl 式规划器仿真、时间估算
│   ├── kinematics  # 机床构型 IK/FK(BC 转台/AC 摆头/混合)、TCP/非 TCP 双模式输出(§23)
│   ├── sim         # Z-map / SDF 材料去除、扫掠体、碰撞
│   ├── verify      # 模型差异对比
│   └── app         # egui + wgpu:视口、面板、工程管理
└── fuzz/ tests/ assets/   # 语料:病态 STL、NIST STEP、黄金 G-code
```

划分原则:`math/geometry` 无 IO 依赖、纯函数核心(便于 fuzz 与 SIMD);`app` 是唯一碰 GUI 的 crate;所有 crate `#![forbid(unsafe_code)]` 除 `io-stl` 的零拷贝转型路径。

### 1.3 并发模型

- **解析**:STL/STEP/G-code 均为"可分块"数据 —— `memmap2` 映射 + rayon `par_chunks` 并行解码(G-code 行解析按行分块,后归并模态状态需顺序,采用"分块解析 + 顺序重放"两段式)。
- **刀路/仿真**:每个操作(op)、每个 Z 层、每个高度场条带都是独立任务,rayon `par_iter` + `chunks_mut` 写入不相交内存区。
- **UI 线程模型**:后台任务(`std::thread` + channel)跑长任务(解析/刀路/仿真),以进度事件(枚举)推给 egui;取消用 `AtomicBool` 令牌。参考模式:kirimoto 浏览器版把 CAM 引擎放 Web Worker,同构思想(来源:GridSpace/grid-apps 源码结构,https://github.com/GridSpace/grid-apps )。

---

## 2. 输入模块:STL 解析(自研规格)

### 2.1 格式规范(一手来源:[Wikipedia STL](https://en.wikipedia.org/wiki/STL_(file_format)))

**二进制布局**:
- `UINT8[80]` 头:内容未定义,**不应以 "solid" 开头**(许多判别器据此识别 ASCII);
- `UINT32`(小端):三角形数 N;
- N × 50 字节记录:`REAL32[3]` 法线(12B)+ 3 × `REAL32[3]` 顶点(36B)+ `UINT16` 属性字节数(2B);全部小端 IEEE 754 单精度;
- 属性字节规范要求为 0,实践中被两种互不兼容的着色扩展占用:VisCAM/SolidView(15-bit RGB)与 Materialise Magics(位序相反)——通用读取器无法区分,**直接忽略**;
- 总长校验:文件大小 == 84 + 50×N。

**ASCII 语法**:`solid name` → 若干 `facet normal / outer loop / vertex ×3 / endloop / endfacet` → `endsolid`。数字为科学计数法;空白任意。注意 ASCII 文件也可能以 "solid" 开头的二进制变体 —— **可靠判别:读 84 字节做长度校验,而非看关键字**。

### 2.2 已知缺陷与预处理(同上 Wikipedia "Concepts")

- **无单位信息**:必须 UI 询问(mm/inch),默认 mm 并显著标示;
- 法线常为 (0,0,0) 或错误 → 一律按顶点右手定则(外看逆时针)重算,不信任文件法线;
- 顶点重复存储、浮点误差导致"相同"顶点不重合 → **顶点焊接**;
- 合法实体要求:闭合、连通、每边恰被两面共享、不自交 —— STL 不保证任何一条;
- 常见病态:非流形边/点、翻面(绕向不一致)、破洞、退化三角形、自相交。

### 2.3 自研解析器规格

1. 打开文件 → 读 84 字节头 → 校验 `84 + 50N == file_len`(失败或 N==0 转 ASCII 路径);
2. 二进制:`memmap2::Mmap` → `bytemuck` 零拷贝转 `&[[u8;50]]` → rayon 并行解码为 `Vec<[Vec3<f32>;3]>`(f32→f64 提升在解码时完成);
3. ASCII:整读字节流(UTF-8 lossy)→ 按行分块并行解析(词法:跳空白、匹配关键字、解析科学计数法;非法行记录行号并报错);
4. 解析后立即构建 `Mesh`:焊接(空间哈希网格,容差默认 `bbox 对角线 × 1e-6`)→ 去重面/零面积面 → 统计报告(洞数、非流形边数、翻转面数)。

### 2.4 网格修复管线(自研算法清单)

顺序(参照 MeshLab 清洗过滤器的推荐顺序,来源:[PyMeshLab 过滤器文档](https://pymeshlab.readthedocs.io) 与 [MeshLab cleanfilter.cpp](https://github.com/cnr-isti-vclab/meshlab/blob/main/src/meshlabplugins/filter_clean/cleanfilter.cpp)):

1. 重复顶点焊接 → 2. 重复/零面积面删除 → 3. 未引用顶点删除 → 4. 非流形边处理(分裂或删除)→ 5. 绕向一致化(BFS 沿邻接翻转,不可达处按体积符号定全局朝向)→ 6. **破洞填补(Liepa 2003)**:动态规划最小权重三角化孔洞边界 + 薄板能量 fairing,论文:M. Liepa, "Filling Holes in Meshes", *Eurographics Symposium on Geometry Processing*, 2003(条目页:[Semantic Scholar](https://www.semanticscholar.org/paper/Filling-Holes-in-Meshes-Liepa/d8a45bce2730f12924b9fc6db7fc013089fd3ce6));参考实现(C++):https://github.com/russelmann/hole-filling-liepa 。

### 2.5 学习参考(不链接)

- `stl_io`(MIT,2026-03 仍活跃,https://crates.io/crates/stl_io 、https://github.com/hmeyer/stl_io )与 `nom_stl`(MIT,https://github.com/fast-radius/nom_stl )—— 读其判别与容错逻辑;
- `tri-mesh`(MIT,半边结构,https://github.com/asny/tri-mesh)—— 拓扑操作设计参考;
- `meshx`(MIT OR Apache-2.0,https://github.com/elrnv/meshx)。

---

## 3. 输入模块:STEP 解析(自研分阶段路线)

### 3.1 标准体系(来源:[Wikipedia ISO 10303](https://en.wikipedia.org/wiki/ISO_10303)、[Wikipedia STEP-file](https://en.wikipedia.org/wiki/STEP-file))

- ISO 10303(STEP)约 700 项:Part 11 = EXPRESS 建模语言;**Part 21 = 交换结构明文编码(STEP-File,我们要解析的层)**;Part 42 = 几何拓扑资源;Part 28 = XML。
- 应用协议:AP203(机械设计)、AP214(汽车)、**AP242(2014 起合并前两者,新增 GD&T、运动学、曲面网格 tessellation;2020 二版增材制造等)**;AP238(STEP-NC)面向 CNC,远期相关。
- AP203 旧文件常用 `FACETED_BREP`(平面片);AP242 网格用 `TESSELLATED_SOLID`/`TESSELLATED_SHELL`。
- **教训:STEP 无法流式读取**(实体前向/后向引用,必须整读入实体图);同一几何有多种编码,导入端普遍需要几何修复。

### 3.2 ISO 10303-21 语法(自研解析器规格依据)

```
ISO-10303-21;
HEADER;
  FILE_DESCRIPTION(('...'),'2;1');   // implementation_level
  FILE_NAME('x.stp','2026-01-01T00:00:00',(''),(''),'','','');
  FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 2 1 1 }'));
ENDSEC;
DATA;
  #16=PRODUCT('A0001','Test Part 1','',(#18));   // 实体实例
  #30=CARTESIAN_POINT('',(1.0,2.0,3.0));         // 简单实体
  ...复杂实体(内/外映射)=(A|B) 形式
ENDSEC;
END-ISO-10303-21;
```

要点:实例名 `#n` 文件内有效;未置值 `$`;派生 `*`;枚举 `.TRUE.`;字符串带编码前缀;第 3 版(2016)新增 anchor/reference/signature 节与 ZIP 容器 —— 自研解析器先支持 2002 版语法 + 对新节跳过告警即可。行结构简单,适合手写递归下降:`文件 → 头/数据节 → 实体记录 → 参数表(嵌套类型/引用/枚举/字符串/二进制/数字)`。

### 3.3 B-rep 关键实体链(来源:[steptools.com AP schema 文档](https://www.steptools.com/stds/stp_aim/html/t_manifold_solid_brep.html))

```
ADVANCED_BREP_SHAPE_REPRESENTATION
 └ MANIFOLD_SOLID_BREP (outer: CLOSED_SHELL; 子型 brep_with_voids / faceted_brep)
    └ CLOSED_SHELL → ADVANCED_FACE(含几何曲面引用 + 同一面朝向标志)
       └ FACE_OUTER_BOUND/FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE
          └ EDGE_CURVE(曲线 + 始末顶点 + 朝向)→ VERTEX_POINT / LINE|CIRCLE|...|B_SPLINE_*
TESSELLATED_SOLID(属性 items: SET OF tessellated_structured_item,
                   可选 geometric_link: manifold_solid_brep  ← 网格与精确 B-rep 关联)
```

### 3.4 自研路线(四档,逐档验收)

| 档 | 内容 | 难度 | 说明 |
|---|---|---|---|
| **S1** | 10303-21 语法解析 + 实体图 + HEADER 元数据 | 低~中 | 纯语法,2~3 周量级;参考 [ruststep](https://github.com/ricosjp/ruststep)(Apache-2.0,读思想) |
| **S2** | `TESSELLATED_*` 与 `FACETED_BREP` → Mesh | 低 | 实体属性直接给三角形/面片,优先交付 |
| **S3** | 解析曲面(PLANE/CYLINDRICAL_SURFACE/CONICAL_SURFACE/SPHERICAL_SURFACE/TOROIDAL_SURFACE)的 ADVANCED_BREP → 网格化 | 中 | 自研曲面参数求值 + 裁剪环内网格化(环内参数域三角化) |
| **S4** | B_SPLINE_CURVE/SURFACE(NURBS)求值与裁剪 | 高 | 教材:《The NURBS Book》,Piegl & Tiller, Springer 1997(de Boor 求值、节点矢量、曲面裁剪);允许无限期后置 |

### 3.5 生态现状(2026-11 核验,作为对照而非依赖)

- [ruststep](https://crates.io/crates/ruststep)(Apache-2.0,0.4.0,2024-09;https://github.com/ricosjp/ruststep )—— 最完整的 Rust STEP 工具集(10303-21 + EXPRESS 编译器 espr),更新趋缓;
- [step-io](https://crates.io/crates/step-io)(2026-10-21 更新;https://github.com/elgar328/step-io)—— 面向 3D 内核的精简 STEP I/O,新但值得关注;
- [step-p21](https://github.com/virtualritz/step-p21)(ruststep 的 fork)、[iso-10303](https://github.com/J-F-Liu/iso-10303)(停滞);
- truck 纯 Rust CAD 内核([ricosjp/truck](https://github.com/ricosjp/truck),`truck-stepio` 存在但两年未发版);**Fornjot 已归档关停**(https://github.com/hannobraun/Fornjot,"goals were never reached");
- C++ 兜底(不采用,仅记录):OpenCascade(LGPL-2.1 + 例外文件,绑定 [bschwind/opencascade-rs](https://github.com/bschwind/opencascade-rs),静态链接商用合规需法务确认)、STEPcode(BSD)。
- "step-rs"/"ironworks step" 不存在(定向搜索无果)。

### 3.6 测试模型(NIST 官方)

NIST "MBE PMI Validation and Conformance Testing" 公开 FTC/CTC/STC 全套模型与 AP203/AP242 STEP 文件:入口 https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0 ;文件下载页 https://www.nist.gov/document/nist-pmi-step-files 。作为 CI 回归金标准。

---

## 4. 几何基础库(自研清单)

### 4.1 数学与查询基元

- `Vec2/Vec3/Vec4/Mat4/Quat`(f64 主型,f32 供渲染)、`AABB`、`Ray`、`Plane`;全部单元测试覆盖(对拍可用 glam 做测试参照,但产品代码不依赖)。
- **自研 BVH(AABB 树 + SAH)**:SAH 代价模型与构建流程照 PBRT 第 3 版 BVH 章(全文可读:https://pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies )——自顶向下,每结点对 3 轴 × 分桶(如 16 bin)取最小代价 `C = C_trav + (SA_L/SA_P)·C_L + (SA_R/SA_P)·C_R`;叶 4~8 三角形;查询:射线(slab test + 近远子树排序栈)与最近点(按距离下界剪枝的优先遍历)。起源文献:Goldsmith & Salmon, *IEEE CG&A*, 1987(https://authors.library.caltech.edu/records/12gr1-2f603 )。
- **射线-三角形**:Möller & Trumbore, "Fast, Minimum Storage Ray/Triangle Intersection", *JGT* 2(1), 1997, DOI 10.1080/10867651.1997.10487468(免费 PDF:https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf )。

### 4.2 2D 布尔与偏置(自研 Vatti)

- 算法来源:B. R. Vatti, "A generic solution to polygon clipping", *Communications of the ACM* 35(7):56–63, 1992, DOI https://doi.org/10.1145/129902.129906 。
- **最佳学习参照:[Clipper2](https://github.com/AngusJohnson/Clipper2)**(Angus Johnson,BSL-1.0 许可;文档 https://www.angusj.com/clipper2/Docs/Overview.htm ):布尔 + 偏置 + 三角化一体;实现文档详尽。**时效注意:上游主仓库 2026-10-28 起归档只读,作者退休**(仓库页横幅,已核验)——更说明应当自研、避免依赖。
- 自研范围:多边形布尔(交/并/差)+ 偏置(Round join 用圆弧离散、Miter/Square join);浮点实现必须带**整数网格化**(如 Clipper2 的 64 位定标整数路径)或 epsilon-鲁棒处理,否则 Sliver/自交会系统性出错。
- 备选范式(二期可评估):直线骨架内偏置(综述:F. Cacciola, "A Survey of Polygon Offsetting Strategies", http://fcacciola.50webs.com/Offseting%20Methods.htm ;Held 2016, *CAD&A*, https://www.tandfonline.com/doi/full/10.1080/16864360.2016.1150718 );Voronoi/medial-axis 偏置(Martin Held, *On the Computational Geometry of Pocket Machining*, LNCS 500, Springer 1991, https://www.cosy.sbg.ac.at/~held/projects/pocket_off/pocket.html )。
- 多边形三角化(网格化/填充用):耳切法(Mapbox earcut 为公开参照,https://github.com/mapbox/earcut )或单调多边形三角化(de Berg 等《计算几何:算法与应用》第 3 章)。

### 4.3 布尔运算策略(重要架构决策)

**自研鲁棒网格布尔代价极高**(Manifold 库的核心卖点就是"第一个对边界情形鲁棒的网格布尔",https://github.com/elalish/manifold ,Apache-2.0)。因此:

- 粗加工边界 = **切片法**绕行:按 Z 层切片 → 每层 2D 布尔(自研 Vatti)求"毛坯截面 − 模型截面",不做 3D 网格布尔;
- 毛坯/切削场 = **SDF 布尔**(max/min,见 §8.4),在连续场层面做减法,后置表面提取;
- 实体级布尔仅在必须时引入,二期评估自研(可学习 Manifold 的算法思想)或 SDF 替代。

---

## 5. 刀具路径生成算法(自研)

### 5.1 刀具模型与基础公式

- 刀具四类:平底(Cyl)/球头(Ball)/圆鼻(Bull = 柱 + 环面圆角)/锥刀(Cone);OpenCAMLib 以 `CylCutter/BallCutter/BullCutter/ConeCutter/CompositeCutter` 分类(https://opencamlib.readthedocs.io/ )。
- **残留高度(cusp)公式**(球头,来源已核验:[Machining Doctor 计算器页](https://www.machiningdoctor.com/calculators/ball-nose-surface-finish)):

  ```
  h = r − √(r² − p²/4)        (p=步距, r=球头半径)
  p = 2√(2hr − h²)            (由目标残留高度反解步距)
  近似: h ≈ p²/(8r)
  ```

- 顺铣/逆铣:本质决定偏置环绕向;顺铣(切屑厚→薄)表面更好、要求无间隙传动(参考:Harvey Performance "In The Loupe",https://www.harveyperformance.com/in-the-loupe/conventional-vs-climb-milling ;实现参照 Kiri:Moto `op-area.js` 的 `POLY.setWinding`:https://github.com/GridSpace/grid-apps/blob/master/src/kiri/mode/cam/work/op-area.js )。
- 切削参数链:材料/刀具 → 线速度 Vc → `n = Vc·1000/(π·D)` → 每齿进给 Fz 表 → `Vf = n·Z·Fz`(定义页:[Machining Doctor glossary](https://www.machiningdoctor.com/glossary/feed-per-tooth/));数值表建议内置可编辑刀具库。

### 5.2 2D 操作

| 操作 | 算法 | 参考/出处 |
|---|---|---|
| **轮廓 Profile** | 外/内/在线偏置(Vatti)→ 多层深度 → 切向圆弧 lead-in/out → 可选 tab(桥) | Kiri:Moto `op-area.js` 的 `cutTabs`(已核验源码);CamBam Holding Tabs 文档:http://www.cambam.info/doc/plus/cam/HoldingTabs.htm |
| **挖槽 Pocket(等距环切)** | 区域 = 边界并 − 岛屿 → 逐层 `−stepover` 内偏置 → 环间连接(tip2tip 最近邻) | Kiri:Moto `op-area.js` 的 `mode:'clear'`(已核验);理论:Held 1991(LNCS 500)、Held 1994 *CAD* 26(1) |
| **栅格 Zigzag** | 区域内平行线扫描 → 交线 → 相邻行端点连接;最优连接见 Arkin, Held, Smith, "Optimization problems related to zigzag pocket machining", *Algorithmica* 26(2), 2000 | https://uni-salzburg.elsevierpure.com/en/publications/optimization-problems-related-to-zigzag-pocket-machining-2 ;OpenCAMLib `src/algo/zigzag.hpp`(https://github.com/aewallin/opencamlib/tree/master/src/algo ) |
| **钻孔 Drill** | 点位排序:贪心最近邻 + 可选 2-opt(TSP);OpenCAMLib 有 `tsp.hpp`(已核验) | CuraEngine `src/path_ordering.cpp` 组织方式可借鉴(AGPL,只读思想)https://github.com/Ultimaker/CuraEngine |
| **面铣 Level** | 毛坯顶面外扩环切或栅格 + 层深 | Kiri:Moto Level op |
| **下刀 Plunge** | 优先级:螺旋插补 > 斜坡 ramp > 直插;斜坡角经验 10°(铝)~45°(木) | CNC Cookbook(含计算器):https://www.cnccookbook.com/helical-interpolation-ramp-angle-calculator-best-ways-to-enter-a-cut |

- **摆线/自适应清角(Trochoidal/Adaptive)→ 二期**:文献 Rauch, Duc, Hascoet (2009) *IJMTM* 49(5), DOI 10.1016/j.ijmachtools.2008.09.006(https://www.sciencedirect.com/science/article/abs/pii/S0890695508002332 );Wang 等 (2016) "Trochoidal machining for the high-speed milling of pockets"(https://www.sciencedirect.com/science/article/abs/pii/S0924013616300310 );OpenCAMLib `adaptivewaterline.cpp`。MVP 用"等距偏置 + 局部小圆清角"近似。
- **morphed spiral / 光滑螺旋 → 远期**:Pateloup, Duc, Ray (2004) *IJMTM* 44(12–13), DOI 10.1016/j.ijmachtools.2004.04.001;Held 等 "A smooth spiral tool path for high speed machining of 2D pockets", *CAD* 2009(https://www.sciencedirect.com/science/article/abs/pii/S0010448509001031 )。
- **G41/G42 刀补**:CAM 内部自行偏置的路径(挖槽/自适应)**不开**刀补避免双重偏置;仅轮廓精修可选输出 G41/42 由机床做磨耗补偿(参考:https://www.machinemetrics.com/blog/cutter-compensation )。

### 5.3 3D 精加工:Drop-Cutter(自研核心算法)

原理:(x,y) 已知,刀具沿 −Z 下落至三角网格,取各接触测试的最大 Z。对每个三角形做 **7 类接触测试:3 顶点 + 3 边 + 1 面**:

- 球头:顶点=球心距离;面=球心到平面投影;边=点到线段距离(球心到边所在直线的垂足 clamp);
- 平底:顶点/面为平推;边=柱面-线段相切(二维圆-线段在刀轴垂直平面内解);
- 圆鼻(环面):边测试最复杂(圆环与线段),推导见 OpenCAMLib 作者博客 "Drop-Cutter toroid edge test"(2014):https://www.anderswallin.net/2014/02/drop-cutter-toroid-edge-test/ 与 "Toroidal drop-cutter"(2010):https://www.anderswallin.net/2010/04/toroidal-drop-cutter/ ;
- 原始论文(OpenCAMLib References 页收录,https://github.com/aewallin/opencamlib/wiki/References ):Yau, Chuang & Lee (2004) 广义 APT 刀具 + kd-tree, *Int. J. Adv. Manuf. Tech.*, DOI 10.1080/00207540410001671651;Hwang & Chang (1998) *CAD*, DOI 10.1016/S0010-4485(98)00021-9;Chuang, Chen & Yau (2002), DOI 10.1007/PL00003965;
- 工程要点:网格桶/kd-tree 空间索引只取刀位邻域三角形;**自适应采样**(路径曲率大处加密,作者博客 "Adaptive sampling drop-cutter":https://www.anderswallin.net/2010/10/adaptive-sampling-drop-cutter/ );rayon 按刀位点并行。

### 5.4 等高线精加工(Waterline)

- OpenCAMLib 完整方案 = push-cutter(刀具沿 X/Y 径向推入产生常 z 接触区间 fiber)→ weave 拓扑缝合,复杂度高(https://opencamlib.readthedocs.io/ ;源码 `waterline.cpp/fiberpushcutter.cpp/batchpushcutter.cpp/weave.cpp`:https://github.com/aewallin/opencamlib/tree/master/src/algo );原始文献:Chuang & Yau (2005) *CAD*, DOI 10.1016/j.cad.2004.10.005。
- **我们的简化路线(推荐)**:平面-三角形求交 → 2D 交线段拼接闭合环 → 交给 5.2 的 2D 偏置引擎做左右偏置 → 输出;复杂度大幅降低,精度足够三轴。

### 5.5 粗加工(切片式)

1. 按 `ap`(轴向切深)分层;2. 每层:模型切片 → 与毛坯截面做 2D 差集 → 按刀具半径 + 留量偏置 → 环切/栅格填充;3. 层内连接 + 斜坡/螺旋下刀;4. 平坦区(flat)单独补一刀(Level op 思路,Kiri:Moto `op-rough.js` 的 flats pass 已核验)。此路与 3D 打印切片同构,CuraEngine 的流水线组织(切片→墙→填充→排序)可作思想参考(**AGPL-3.0,严禁抄码**,https://github.com/Ultimaker/CuraEngine )。

### 5.6 直线/圆弧拟合(输出前压缩)

- 折线 → 圆弧串拟合,容差取弦高;biarc 理论:Bolton (1975) *CAD* 7(2);Meek & Walton, "Approximation of discrete data by G1 arc splines", *CAD* 24(6), 1992(https://www.sciencedirect.com/science/article/pii/001044859290047E );
- 工程简化:MVP 用"共圆判定"(连续点在同一圆且弦高 < 容差则合并为 G2/G3,注意起终点半径一致性——Kiri:Moto 曾因此出 bug:https://github.com/GridSpace/grid-apps/issues/411 );
- 参考:FlatCAM `camlib.py`(MIT,https://github.com/Denvi/FlatCAM/blob/master/camlib.py )的区域清除与连接优化。
- **多轴(3+2/五轴)策略**见 §24;机床运动学与 RTCP 输出见 §23。

### 5.7 全自动流水线(一键 CAM,2026-11-07 补查)

**商业对标解剖**:

| 产品 | 自动化程度 | 关键机制 | 来源 |
|---|---|---|---|
| MeshCAM($250/$500) | 最强一键:7 步出码、所有参数单对话框 | **自动进给 = 机型配置 × 材料 CSV 静态查表**(carbide-wizard.lua + materials.csv,用户可编辑);Auto Toolpath 向导会判定"本零件无需粗加工"、改 waterline 层深;自动 tabs;自动找孔;Quick Flip 双面;**无自适应粗加工、无物理进给计算** | https://www.grzsoftware.com 、官方 STL 教程 https://www.grzsoftware.com/tut/stl 、向导脚本 https://community.carbide3d.com/t/how-to-add-tools-to-carbide-wizard-in-meshcam/1469 、向导行为实例 https://community.carbide3d.com/t/trying-to-get-started/14359 |
| Kiri:Moto | 无向导,靠"默认值记忆"(改设置不自动传播) | 4 步:拖入 STL(自动贴床)→ Rough+Contour(记住的默认值)→ Slice → Export;**defaults.js 源码级验证**:Rough stepdown 2mm / stepover 40% 刀径 / 进给 1000 / 快移 6000 | https://docs.grid.space/kiri-moto/CAM/ 、https://raw.githubusercontent.com/GridSpace/grid-apps/master/src/kiri/app/conf/defaults.js |
| FreeCAD CAM | Job = 模板机 | New Job 自动创建毛坯/刀具控制器/SetupSheet;Job Template 可设为默认 | https://wiki.freecad.org/CAM_Job |
| MeshMill/pngcam(开源) | 极简全自动 | STL → 高度场 → G-code;斜坡下刀、跳过平顶、行程重排、**输出余料高度场供下一工序接力(免切空气)**;作者自述短板:走空优化=TSP、会蹭面、缺 tabs/多面 | https://incoherency.co.uk/blog/stories/meshmill.html 、https://github.com/jes/meshmill |
| OpenCAMStudio(Rust,GPL-3.0) | 2.5D 端到端 | DXF 2.5D;Profile/Pocket/Drill/Face/V-carve;中性 IR + 6 种后处理器;高度场仿真+过切检测;3 轴曲面在路线图 | https://github.com/pr0m1th3as/OpenCAMStudio |

**MeshCAM Unified Finishing 机制**(纯几何自动分流的完整规范,可直接复刻):**Surface angle limit / Min surface angle 双阈值**——比前者平缓的区域归平行行切、比后者陡的区域归 waterline(等高),两带**略微重叠防硬接边**(https://www.grzsoftware.com/blog/surface-angle-limits );Quick Flip 双面:Op1 顶面中心对零+面铣+外轮廓轻切 1–2mm 作基准 → 翻面 → Op2 毛坯角对零(https://www.grzsoftware.com/blog/quick-flip-toolpath )。

**纯几何全自动路线图(对任意 STL,每步都有先例)**:①预处理(单位确认/Z 归零/流形检查)→ ②毛坯(包围盒+边距自动生成)→ ③粗加工 = 恒 Z 切片 + 层内偏置/行切(opencamlib 双核:drop-cutter + waterline/z-slice)→ ④精加工 = drop-cutter 行切 → ⑤**双角度阈值自动分流**(MeshCAM 方案)→ ⑥自动 pencil → ⑦自动 tabs / Quick Flip → ⑧余料高度场接力(rest/pencil 触发)。

**缺陷清单(纯几何路线固有盲区与缓解)**:

| 缺陷 | 缓解(有出处) |
|---|---|
| 不知道哪些面要加工(装夹面/螺纹孔口都会被啃) | 用户点选装夹面/排除面;自动 tabs(MeshCAM);Clip To 裁剪(Kiri:Moto) |
| 公差语义缺失(全域均匀标量) | 选定面提升公差档(混合交互);轻量特征标记配合面 |
| 深腔/窄槽刀具不可达 | MICR/MICW 分析(§5.9)+ 自动 rest 链(hyperMILL 先例) |
| 高度场单值(2.5D)限制 | 双面 Quick Flip 工作流;切 waterline/体素路线 |
| 走空与接刀次序(TSP) | 分区贪心 + 高度场避障抬刀(pngcam 的 omit-top/余料图思路) |
| 向导保守性(默认值对铝不安全) | 机型刚性库 + 材料表 + 快↔好偏好二选一;标注"起始值可覆盖" |

**必答输入收敛为 4 项**(全部对标产品共性):单位/毛坯、机型(刚性库)、材料(材料表)、质量偏好(快↔好);刀具可由系统自动选。

**我们的一键 CAM 设计建议**:流水线骨架用上述 8 步(CL 内核 = drop-cutter + z-slice 双核);4 项必答 + 可解释决策输出(仿 MeshCAM"无需粗加工"判定);自动化顺序:自动毛坯/tabs/找孔(成本低感知高)→ 余料接力 → 策略自动裁剪;**v1 不做特征识别**(点选装夹面/排除面 + 面公差覆盖吸收两大盲区)、不承诺物理进给计算(全部对标均为静态查表)。

### 5.8 自动参数决策规则表(规则引擎初始规则库)

| 类别 | 规则 | 数值 | 来源 |
|---|---|---|---|
| 层深 stepdown | 厂商表值适用上限 | 表值适用至切深 ≤ 0.5×D;切深 >2×D 或全槽开粗 → SFM ×0.5 | https://www.cnccookbook.com/feeds-and-speeds |
| 层深 | 传统粗加工经验 | 轴向 0.5–1×D、径向 40–50%D;大刀(>20mm)轴向可至 4×D | https://qfcncmachine.com/cnc-depth-of-cut (搜索级) |
| 层深 | HEM/自适应反直觉规则 | 径向 5–25%D、轴向深至 2–3×D(总效果优于浅轴向) | https://www.cnccookbook.com/high-speed-machining-speeds-and-feeds |
| 层深 | 机型刚性修正 | MeshCAM 实例:同几何向导给 waterline stepdown 0.0625in(桌面机);默认值对铝需人审 | https://community.carbide3d.com/t/trying-to-get-started/14359 |
| 步距 stepover(精) | 通用带 | 1/3–1/10 D;软材料 1/5–1/3,硬/保细节 1/5–1/10 | https://www.cnccookbook.com/cnc-stepover |
| 步距 | 收益拐点 | < ~1/8 D 后残留曲线趋平,再小纯浪费时间("sweet spot");减半步距≈工时翻倍 | 同上 |
| 步距 | 残留反解 | h ≈ s²/(8R) → s = 2√(2Rh−h²)(§5.1 公式;配合 ISO 序列 clamp) | 同上 + §5.1 |
| 步距(粗) | 经验 | 40–70% D(50% 中位);Kiri 默认 40%;径向 <50%D 触发**切屑变薄修正**(进给上调,实例近 4 倍) | https://www.cnccookbook.com/feeds-and-speeds ;grid-apps defaults.js |
| 刀具选择 | 主规则 | "最大能装进最小特征的刀"(MICR 最小圆角半径 / MICW 最小通道宽分析);MRR 优先 | Ho 1997 / Navaneethan 2024 综述(§5.9) |
| 刀具选择 | 曲率匹配 | 五轴按刀具半径与表面曲率匹配 | https://www.sciencedirect.com/science/article/abs/pii/S0010448501000860 (搜索级) |
| 粗精组合 | 标准链 | 粗 → 清残(rest)→ 半精(可选)→ 精 → 清角(pencil) | https://docs.encycam.com/ENCYrobot/1/en/727.html (搜索级) |
| rest 触发 | 自动化条件 | 残料模型与目标模型差 → 若残料最小通道宽 ≥ 次级刀径 × 安全系数(≈1.2)则自动追加 rest,否则并入 pencil;先例:hyperMILL 自动 3D rest、pngcam 余料高度场接力 | https://www.openmind-tech.com (新闻页未验证) ;https://incoherency.co.uk/blog/stories/meshmill.html |

**目标 Sa 模式**:精加工 stepover 可由 §9.11 B 层的理论粗糙度预测反解(用户给目标 Sa → 自动定步距并 clamp 到经验带),作为经验规则的补充。

### 5.9 特征驱动的自动工艺规划(CAPP,2026-11-07 补查)——"特征识别出来之后,怎么自动变成操作树"

**领域框架**:检索式(variant,成组技术改编)vs 生成式(generative)——Alting & Zhang 1989, *IJPR*(https://www.tandfonline.com/doi/abs/10.1080/00207548908942569 );现代综述:知识图谱(Xiao 2023, *J. Manufacturing Systems*, https://www.sciencedirect.com/science/article/abs/pii/S0278612523001577 )、CAPP-GPT(Azab 2024, https://www.sciencedirect.com/science/article/pii/S221384632400066X )。**结论:工业可用的自动工艺规划 = 生成式框架 + 可编辑知识库(策略模板);纯学术生成式系统从未大规模商用。**

**特征→操作映射规则表(直接落库)**:

| 特征 | 标准工序链(触发条件→追加) | 参数来源 |
|---|---|---|
| 孔-普通钻 | 点钻(斜面/防偏或 L/D 大)→ 钻 → 倒角 | 点钻径≈钻径×1.25–1.5;vc/f 查 §16 库;Sandvik:https://www.sandvik.coromant.com/en-us/knowledge/drilling ;点钻 vs 中心钻:https://www.mastercam.com/community/blog/spot-drill-vs-center-drill-what-is-the-difference |
| 孔-铰(H7) | 点钻 → 预钻(留铰量 0.1–0.4mm,≈径向 1–3%)→ 铰 | 铰削 vc 为钻的 1/2–2/3;Fictiv:https://www.fictiv.com/articles/drilling-boring-reaming-precision-holes ;公差计算器:https://www.machiningdoctor.com/calculators/tolerances |
| 孔-攻丝 | 点钻(可选)→ 螺纹底孔(查丝锥表)→ 攻丝/铣螺纹 | 进给=螺距(刚性攻丝同步) |
| 孔-精密(位置度) | 点钻 → 钻 → 镗(单刃精镗保位置度)→ 铰 | 镗余量/分配由 GD&T 触发 |
| 多径台阶孔 | 大径→小径分层重复上链(CAMWorks Multi-Step Hole 逻辑) | 每段独立查库 |
| 型腔 | 开粗(摆线/自适应,留壁 0.3–0.5、底 0.1–0.25)→ rest 二次开粗(换小刀)→ 底面精 → 侧壁精 → 清角 | rest 刀径 ≤ 腔角半径 |
| 平面/台阶 | 面铣(粗)→ 精面铣(按 Ra);台阶=底+侧壁 | 精铣 ap 由 Ra 反查 |
| 外轮廓 | 侧壁粗 → 底精 → 侧壁精(顺铣)→ 倒角/清角 | 同腔壁 |

核心逻辑(**公差定链长、尺寸定刀具、材料定参数**)即 CAMWorks TBM(https://camworks.com/module/tolerance-based-machining/ ):读 PMI(尺寸/公差/GD&T/粗糙度/DimXpert)→ 自动选刀具/转速/进给;其 TechDB = 企业知识库模板,即我们要仿的结构。SolidCAM AFRM(https://solidcam.com/cam-modules/afrm )按朝向成组加工。

**工序/工步排序**:
- 规则表:基准先行/先面后孔、粗精分开、公差最短链(DAG)、刀具聚集(软约束)、特征内链自带顺序、时效在精前;
- 算法:排序 = **带优先约束的 TSP(TSPPC)**——Moon et al. 2002, *EJOR*(GA, https://doi.org/10.1016/S0377-2217(01)00248-8 );Su 2018(https://ideas.repec.org/a/spr/joinma/v29y2018i2d10.1007_s10845-015-1109-6.html );Yun 2013(https://www.sciencedirect.com/science/article/abs/pii/S0360835211003421 );
- **两阶段解耦**:setup 间顺序由基准依赖+公差链决定(近唯一拓扑序),setup 内才是组合优化——文献与商用共同做法。

**Setup/装夹规划**:**TAD(刀具接近方向)聚类**——每特征算无干涉接近方向集,按共享 TAD 聚成方向簇 → 最少 setup(Wakhare 2026 AI expert system, https://link.springer.com/article/10.1007/s00170-026-17859-3 ;Ong & Nee 模糊集, https://www.sciencedirect.com/science/article/pii/S0007850607621829 ;公差驱动混合图 https://www.academia.edu/54640823/A_hybrid_graph_approach_for_automated_setup_planning_in_CAPP );DRF 3-2-1(与 §9.9 一致);装夹自动配置:Nelaturi 2014, *CAD*(https://www.sciencedirect.com/science/article/abs/pii/S0010448513001681 );综述 Kang & Peng(https://cad-journal.net/files/vol_5/CAD_5(1-4)_2008_424-433.pdf )。**商用现状:无主流 CAM 声称全自动 setup+夹具设计** → 我们保持"自动建议 + 人工确认"。

**刀具选择自动化**:综述 Navaneethan 2024, *IJAMT*(规则/优化/AI 三类,https://link.springer.com/article/10.1007/s00170-024-13823-1 );工程做法:几何可行集 → 参数库最短工时 → 跨操作**刀具合并贪心**(合并刀的工时增量 < 换刀时间×次数才合并)。

**STEP-NC(ISO 14649 / AP238)**:machining features + workingsteps = 机器可执行工艺计划(https://en.wikipedia.org/wiki/STEP-NC 、https://www.steptools.com/stds/stepnc 、AP238 Ed.2 http://ap238.org/ap238e2/ap238_stepnc_v2.pdf );STEPNCpp 基本停滞(https://github.com/johnmichaloski/STEPNCpp )。判断:**操作树对齐其 executables 命名、预留导出;不做控制器端 STEP-NC 依赖**(生态仍小众)。

**AI 方法判断**:KG 值得跟(Xiao 2023);ML 切削参数选择价值有限(§16 厂商库+车间修正更可控);**LLM 只做规则编纂/解释助手**(无几何正确性保证;CAPP-LLM 2026:https://www.sciencedirect.com/science/article/pii/S073658452600013X );RL 炒作>实用(DRL 工艺路线 https://www.sciencedirect.com/science/article/abs/pii/S0736584523001631 、RL 铣削 https://ieeexplore.ieee.org/iel7/6287639/10005208/10081313.pdf )。

**规则引擎设计(Rust)**:三层——`StrategyLibrary`(ron/json 特征类模板:`{feature_class, trigger:{tol_grade, ra, dims}, chain:[op_spec], tool_constraint}`,对齐 TechDB)→ `RuleEngine`(硬约束:基准/粗精/特征内链;软目标:换刀最少/空程短)→ `Planner`(setup 分组 → setup 排序 → setup 内排序 → 刀具合并 → 参数解析)。接口:§15 特征结构需补 **GD&T/基准引用 + TAD 候选集**(否则 setup/排序无输入;半自动阶段由用户标基准);§16 查询签名 `query(op_type, material_group, tool_class, dia)` + 公差/粗糙度档位修正系数表;刀路引擎消费 `OperationSpec`;需剩余毛坯模型服务(rest)与快速可达性/干涉查询(setup 校验)。**MVP**:3 轴;平面/台阶、孔+台阶孔、矩形/圆角腔、开放轮廓;映射表全量落库;TAD 聚类 + 3-2-1 基准建议(夹具只校验不设计);硬约束拓扑序 + 刀具聚集贪心(不上 GA);输出对齐 ISO 14649 executables 命名。

### 5.10 曲面精加工自动化策略族(2026-11-07 补查,全部 DOI 经 CrossRef 逐条核验)

- **iso-parametric**:Elber & Cohen 1994(https://doi.org/10.1016/0010-4485(94)90070-1 );叶片加工最流行;缺陷:行距随参数化/曲率变化 → 残留不均、必须按最小行距设定(Lin 2015 综述:https://www.sciencedirect.com/science/article/abs/pii/S0010448515001013 )。
- **iso-planar(截平面)**:Ding 2003 自适应面间距(https://doi.org/10.1016/s0010-4485(02)00048-9 );waterline = 水平面子集;商用 **Steep & Shallow** = raster + 等高的自动拼接(阈值 ~30°,https://www.autodesk.com/products/fusion-360/blog/fusion-360-extensions-steep-shallow )。
- **iso-scallop(等残留高度,重点)**:开山 **Suresh & Yang 1994, "Constant Scallop-height Machining of Free-form Surfaces", ASME J. Eng. Ind. 116(2):253-259, DOI 10.1115/1.2901938**(CrossRef 纠偏);Feng & Li 2002(https://doi.org/10.1016/s0010-4485(01)00136-1 );Tournier & Duc 2002(*IJAMT* 19(5), https://doi.org/10.1007/s001700200019 ——**纠偏:网传 "CAD 34:267-279" 是错误出处**)/ 2005 五轴(https://doi.org/10.1007/s00170-003-2054-7 );网格版 Liu 2012(https://link.springer.com/article/10.1007/s00170-011-3892-3 )、Kim & Yang 2006(https://doi.org/10.1007/s00170-004-2372-4 );几何化推广:Kim T. 2007 测地平行(https://doi.org/10.1016/j.cad.2007.01.003 )、Zou 2014(https://doi.org/10.1016/j.cad.2014.04.006 )。
- **行距公式(可实现)**:曲面修正 `h ≈ (κs + κc)·w²/8` → **`w ≈ √(8h/(κs+κc))`**;κc = 刀具廓形在法截面内曲率(球头 = 1/r);κs = 曲面**垂直于进给方向**的法曲率(凹面朝刀取正)→ 等效相对曲率 `κrel = 1/r ± κs`;**Euler 公式 `κs = κ1·cos²θ + κ2·sin²θ`**(θ = 行距方向与主曲率方向夹角)→ 进给方向场与行距耦合;相邻行夹角 φ 修正 `s⊥ ≈ s·sinφ`;平面退化即 §5.1 公式。
- **逐 CC 点推进算法**:①初始路径(边界/等参线)按误差采样 CC 点 → ②每点在切平面取垂直进给的行距方向 → ③局部 κrel(网格主曲率,§15.2 公式已有)解 s(P) → ④推进 s(P) 得下一条路径候选,投影回网格(复用 drop-cutter 求交)→ ⑤夹角 φ 修正逐点积分;Liu 2012 离散版在"残留点局部系"直接求下一条 CL(免点偏置)。**已知失效模式**(Tournier & Lartigue 2008 全文已读,https://www.cad-journal.net/files/vol_5/CAD_5(1-4)_2008_278-286.pdf ):路径自身曲率半径 < 刀具半径 → 包络自交成环;球面-圆环过渡刀轴无限打转 → 解法:环消除(引入切向不连续)或预设几何(平行平面布置 + 刀轴 yaw/tilt 保恒残留)。收益:路径长显著缩短(检索级 25–50%;建议用自家 cusp 公式仿真自验证)。
- **pencil 自动生成**:Park, Chung, Kim, Choi 1999 "Pencil Curve Tracing via Virtual Digitizing"(https://doi.org/10.1007/978-0-387-35392-0_30 )——**z-map 虚拟数字化法:残留高度场的谷部脊线 trace 成 pencil 曲线,与我们 drop-cutter 栅格天然兼容(最可实现)**;Kim/Jun/Park 2005 曲线法(https://doi.org/10.1016/j.cad.2004.09.023 );Ren/Zhu/Lee 2005 网格材料侧 tracing(https://doi.org/10.1016/j.cad.2004.10.003 ,与 STL 契合);Park 2005 可见性(https://doi.org/10.1016/j.cad.2005.03.004 )。自动触发 = 大刀栅格仿真 → 理想面−仿真高度 → 阈值分割 + 谷脊连通分量;切序 = 分组就近连接。
- **3D 螺旋**:参数域(Takasugi & Asakawa 2018, https://doi.org/10.1016/j.precisioneng.2018.01.013 )、等残留轮廓偏置(Lee 2003, https://doi.org/10.1016/s0010-4485(01)00185-3 )、网格径向引导(Xu 2018, https://doi.org/10.1115/1.4039918 );5 轴 = 平行平面布置 + 刀轴 yaw/tilt(Tournier & Lartigue 2008);工程量最大,仅叶盘/封闭凸台类投入。
- **策略自动选择**:无权威文献;商用启发式可编码:坡度一致 → 固定步距;变化大 → iso-scallop;陡浅混合 → steep & shallow;凹角残留 → pencil + rest;定量评价框架(Dunkov 2026, https://ijomam.com/wp-content/uploads/2026/09/pp.-239-248_QUANTITATIVE-EVALUATION-OF-CAM-FINISHING-STRATEGIES-FOR-COMPLEX-FREEFORM-SURFACES-USING-A-DIMENSIONLESS-EFFICIENCY-INDEX.pdf )。
- **增量实现排序(基于已有:平行栅格 drop-cutter + 简化 waterline + §15.2 网格主曲率)**:**P1 iso-scallop 行距调度**(只改栅格"行位置"逻辑,性价比最高;先限单连通开面片带,或退化为 Ding 2003 分段自适应面间距更稳)→ **P2 自动 pencil**(z-map 残留法,模具件必备,兼作清角自动触发器)→ **P3 steep & shallow 拼接**(两件套已有,只差坡度阈值分割与交界处理)→ P4 任意方向 iso-planar + 自适应面间距 → P5 3D 螺旋。

### 5.11 刀路质量补强(2026-11-07 补查):进给自适应调度 / 安全链接 / 走刀方向场 / 直纹面拟合

#### 5.11.1 进给自适应调度(feedrate scheduling)

**问题**:全程单一名义进给在切宽/啮合剧变处(内角、全宽槽底、余量台阶)负载骤增(让刀/振纹/断刀),轻载处浪费机时。两类目标:恒进给(简单)vs **恒切屑负载/恒力**(逐点调制)。综述锚点:啮合角 = 刀具-工件接触期间夹角,是 pocket 进给优化核心变量(https://www.researchgate.net/publication/332267083 );经典对比:Erdim & Lazoglu, "Feedrate scheduling strategies for free-form surfaces", *IJMTM* 2006(几何 MRR 法 vs 力学法,https://www.sciencedirect.com/science/article/abs/pii/S0890695505001914 )。

**调制公式(带符号定义)**:
```
啮合角:  θ_en(i) = 2·arccos(1 − ae(i)/R)          # ae=R→180°全槽,自检
切屑厚度: h(φ) = fz·sin(φ), φ∈[φ_st, φ_ex]
Kienzle:  kc(φ) = kc1.1·h(φ)^(−mc),  Ft(φ)=kc·ap·h(φ)   # mc≈0.15–0.35
恒峰值力: fz_max(i) = [F_lim/(kc1.1·ap(i)·s_i)]^(1/(1−mc)), s_i=max sin^(1−mc)(φ)
恒 MRR(简化,v1 起步): vf(i) = clamp(vf_ref·(ae_ref·ap_ref)/(ae(i)·ap(i)), vf_min, vf_nom)
```
(Kienzle 定义:https://www.machiningdoctor.com/glossary/specific-cutting-force-kc-kc1 ;Sandvik:https://www.sandvik.coromant.com/en-us/knowledge/materials/specific-cutting-force ;近年延续:啮合角分段优化 Zeng 2025 https://www.sciencedirect.com/science/article/pii/S1000934525001063 、ANN vs 常规调度 Jacsó 2023 https://link.springer.com/article/10.1007/s00170-023-11506-x )

**逐点调度算法(三遍)**:①逐点限载(啮合查询 `engagement_query(pos,tool,stock)→(ae,ap,φ_st,φ_ex)` → 公式)→ ②**滑窗前后包络平滑**(必须向前看:进拐角前先降;Fusion 参数语义:Reduced Feed Distance/Radius/Rate + Maximum Directional Change + Only Inner Corners,https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-control-feed-rate-on-internal-and-external-arcs-in-Fusion.html )→ ③斜率限幅(每段 |ΔF| ≤ 20%)→ 输出段→F 映射(等值合并省 F 字;F 为模态,G94 下持续到下个 F,无需逐行)。

**与运动规划的分工**:CAM 的 F 是**名义进给**(工艺上限);控制器侧 JD/加减速另会压速(§7 公式),两层不重复降——但循环时间估算要乘控制器折扣(拐角用 v_junction)。颤振约束(SLD 稳定叶瓣图,Altintas & Budak:https://ijme.us/cd_06/PDF/IT%20301-050.pdf )需实测频响函数,**v1 不做**,用全局进给安全系数 0.8 兜底。**开源现状:opencamlib 无此能力(https://github.com/aewallin/opencamlib 已核验),商业也只有局部拐角降速(Fusion/hyperMILL factor 式)——逐点啮合调制是自研差异点**。啮合查询复用仿真毛坯(刀路生成时顺带记录 CC 点局部切削包围盒,O(N) 查表)。

#### 5.11.2 链接/空移的安全规划(linking & retract)

**安全高度体系对比**:

| 体系 | 定义 | 适用 |
|---|---|---|
| 固定安全面 | 全局 Z 平面(毛坯顶+增量),长快移全抬到该面 | 2.5D/平板件;最简单可靠 |
| **区域安全面(skim)** | 两点间沿途毛坯/零件局部包络 z-max + 增量;短链只抬到 skim | 层间/环间链接,减少不必要抬刀 |
| 包络面法向抬刀 | 抬刀跟随毛坯包络(常数法向偏置) | 曲面精加工/五轴;实现成本高,v1 不做 |

(商业共识 = 五高度体系 Clearance/Retract/Feed/Top/Bottom + 短链 skim/长链 clearance 二分,对应 PowerMill Skim distance 与 Fusion Heights;Fusion Feed Optimization 官方页 403,参数语义经搜索确认)

**链接生成伪代码**:
```
link(A,B):  skim = z_envelope_max(A,B) + t_skim;  clear = 全局安全面
  for z in [skim, clear]:                    # 先低后高,失败升级
      path = 竖(A→z) - 平(A→B @z) - 竖(→B)   # 三段折线,首选不走斜线
      if sweep_collision_free(path, tool+holder, stock): return path
  兜底: 垂直退到 clear + 平移;clear 仍碰 → 报错要求改安全面,不静默
中途校验: 水平段按步长 ≤ D/4 采样,刀杆+夹头扫掠圆柱 vs 毛坯 SDF 距离查询(复用仿真毛坯,零新增内核)
```
- 快移限速:距毛坯 < 2mm 的 G0 在后处理可降 G1@快速进给(擦料风险 vs 时间);
- 链末端进刀自动衔接:净空 ≤1mm/预钻 → 垂直 G1@plunge;浅腔 → 斜线 ramp(**Sandvik:ramp 进给降至正常 75%,并保持约一个刀径距离**,https://www.sandvik.coromant.com/en-us/knowledge/milling/milling-holes-cavities-pockets/ramping );封闭深腔 → 螺旋(pitch 由 ramp 角与半径推出);折扣经验:2.5–5°→75%、5–15°→50%、15–30°→25%(zero-divide,未验证);
- 学术锚点:Plakhotnik & Vaneker 2019(*CAD*,刀路点采样+前瞻避碰,https://www.sciencedirect.com/science/article/pii/S0010448518305761 );Nishat 2023 凸包近似(https://www.mdpi.com/2076-3417/13/12/7385 );TrajOpt(通用序列凸优化,https://www.ri.cmu.edu/publications/finding-locally-optimal-collision-free-trajectories-with-sequential-convex-optimization )。
- **MVP**:两级安全高度 + 三段折线 + 采样校验 + 升级/兜底/报错;绕障 v1 只报冲突不自动绕(先靠"全部抬 clear"保守档);5 轴法向抬刀/夹具库/TSP 式全局最优链接不做。共享资产:把仿真毛坯抽象成 `engagement_query` 与 `sweep_clear` 两个 trait 方法,进给调度与链接校验都吃它。

#### 5.11.3 三轴走刀方向优化

**机理**:行距方向(垂直进给)的法向曲率由 Euler 公式 `κn(φ)=κ1cos²φ+κ2sin²φ` 给出,与 iso-scallop 行距公式唯一耦合。**Lin & Koren 1996**(*ASME J. Eng. Ind.* 118(1):20-28,约 410 引,全文 PDF:https://www-personal.umich.edu/~ykoren/uploads/Efficient_tool-path_planning_for_machining_free-form_surfaces.pdf )给出解析判据:凸面沿最小法曲率方向、凹面沿最大幅值方向走刀,行数最少。**统一符号规则**(凸正凹负,κ1≤κ2):行距方向取带符号 κmax → **进给方向取 κmin**,一条规则统一两情形。行距 `L(φ)=√(8h/(1/r−κn(φ)))`;**凹面守卫:`1/r−κn ≤ 0`(内圆角比刀小)→ 根切,标记不可精加工/换小刀**;斜面水平步距投影 `s_h = L·|n·z|`(陡壁区进给应沿壁面母线)。

**v1 全局方向投票(候选 0–180°,步长 5–10°)**:

| # | 信号 | 规则 | 权重 |
|---|---|---|---|
| 1 | 主曲率 | 行距方向 κn → L_i(d);得分 = P10 分位数(比均值稳) | 0.5 |
| 2 | 陡壁 | 罚"进给跨陡壁梯度"方向(鼓励沿等高线) | 0.25 |
| 3 | 区域长轴 | 转弯次数 ≈ W⊥(d)/s_h,鼓励平行长轴(少抬刀) | 0.15 |
| 4 | 环形区域 | 环宽 < 5–8×行距 → 建议 contour-parallel(二元提示) | 开关 |
| 5 | 顺逆铣 | 打破并列的次要项 | 0.05–0.1 |
| 6 | 根切 | 有根切点的方向硬禁用 | 硬约束 |

**v2 分区**:Kumazawa/Feng/Chen 2015 PFD 场(Preferred Feed Direction Field,*CAD* 67-68, https://www.sciencedirect.com/science/article/abs/pii/S0010448515000652 )每点取 κmin 主方向 → 半球 k-means/区域生长分区(角容差 10–15°+法向连续),区内用 v1 规则;分区上限 3–5。**PFD 与 iso-scallop 的全局折中已形式化为 Poisson 问题且支持网格输入**:Zou/Wang/Feng 2020(https://arxiv.org/abs/2009.02660 ;期刊版 https://www.sciencedirect.com/science/article/abs/pii/S001044852100083X )。方向平行规划经典:Park & Choi 2000(https://www.sciencedirect.com/science/article/abs/pii/S0010448599000809 );模式效率对比(KAIST 2002,平滑 zigzag 最优:https://koasas.kaist.ac.kr/handle/10203/83434 );综述 Sun 2022(*Chinese J. Aeronautics*, https://www.sciencedirect.com/science/article/pii/S1000936121002156 )。
**方向敏感性(为什么整区一个方向够用)**:κn 在主方向附近是二阶驻点,方向偏 10–15° 行距损失仅 3–8%。

#### 5.11.4 网格→直纹面拟合(五轴侧刃 swarf 前置)

**可展性判定(离散版,基于已有 cotan/角亏)**:可展 ⟺ 高斯曲率 K≡0。离散:①角亏 `K_i=(2π−Σθ_j)/A_i`(https://en.wikipedia.org/wiki/Gaussian_curvature );②**零主曲率判据(更强)**:|κmin| < εκ 且明显各向异性 → κmin 特征向量 = 候选母线方向。**Crane/Stein/Grinspun 2018 "Developability of Triangle Meshes"(*ACM TOG* 37(4), https://doi.org/10.1145/3197517.3201303 ;项目页 https://www.cs.cmu.edu/~kmcrane/Projects/DiscreteDevelopable/ )**:可展 = 可平展 + 法向在 Gauss 映射上成曲线两条;**文中明确给出侧刃铣削应用**(缝线+母线→五轴刀路),并证明角亏单独不够(只得到可平展但噪乱)——双条件权威背书。我们 MVP 只做**检测**(判据 1+2),不做能量流再造形。

**检测-拟合流水线(6 步)**:①壁面候选(法向区域生长 10–20° + 陡度 n·z<cos60° 过滤,剔除平面)→ ②母线方向场(|κmin|<εκ 点的主方向,流线追踪成条带;Peternell 2004 可展面拟合需先估母线初值:https://www.semanticscholar.org/paper/293e550c05f3db63febfa08f6831965aa73565f7 )→ ③逐母线 PCA 直线拟合(残差 RMS/max)→ ④全局直纹面 `S(t,u)=(1−u)A(t)+uB(t)`(双导向曲线 LSQ + 行参数化对应优化,单侧 Hausdorff:Cao & Dong 2015, https://www.sciencedirect.com/science/article/pii/S1000936115001740 )→ ⑤可侧铣度判定(阈值表)→ ⑥输出条带列表(母线族=刀轴、导向线=轨迹基准)或拒绝理由。

**阈值表(经验值,按公差体系标定)**:逐母线残差 max ≤ 0.02–0.05mm 且 RMS ≤ 0.01(≈公差 1/3);法向沿母线旋转 ≤ 2°;相邻母线扭转 ≤ 2–4°、整带 < 20–30°(**Tsay & Her 2001:扭曲角×母线长×刀径共同决定切底误差**,https://asmedigitalcollection.asme.org/manufacturingscience/article/123/4/731/449760 );角亏 ≤ 0.02–0.05 rad;条带规模:母线 ≥5、长 ≥0.5×侧刃长。准侧刃放宽:残差 ≤ 3–5×公差(刀轴跟随 κmin + 容差内让刀;DP 搜索路线见 Rajain 2025 博士论文 https://bird.bcamath.org/bitstream/handle/20.500.11824/1980/MultiPassG1CustomShapedC.pdf );再差放弃。可切削直纹条带逼近先例:Steenstrup 2016(https://core.ac.uk/download/pdf/84001537.pdf )。

**商业对照(差异化空档)**:PowerMill Swarf 要 BREP 侧面(**网格原生不支持**,官方建议回 PowerShape 重建曲面)、Fusion Advanced Swarf 要直边侧壁、FeatureCAM/Creo 同族、SprutCAM 5D Meshing 与 swarf 仍是两条路径、hyperMILL Mesh 需预处理——**"网格直接检测+拟合→swarf 候选"没有商业产品做**。
**MVP 边界**:v1 = **可侧铣度检测器**(条带提取+PCA+残差报告+3D 可视化,给可解释拒绝理由:扭转角/残差/角亏三项),不生成刀路;v2 = 单带柱刀 swarf(刀轴=母线、刀心=法向偏置 r)+ 圆柱包络自检;明确不做 Stein 能量流与 DP 母线搜索。

### 5.12 刀路光顺与过渡处理(2026-11-07 补查)——直接提高控制器执行质量

**拐角容差内圆滑(公差-半径关系式,推导与 Aerotech 官方定义一致:https://help.aerotech.com/automation1/Content/Concepts/Corner-Rounding-Functions.htm )**:

```
偏转角 β、公差 T(相对编程拐点的最大偏离):
  r_max = T·cos(β/2) / (1 − cos(β/2))
  β=90° → r ≈ 2.414T;β=30° → ≈11.5T;β=150° → ≈0.73T
短段约束:段太短容纳不下半径 → 半径自动收缩,圆弧交点 ≤ 段长/2
```

**关键洞察:与 grbl JD 同几何**——控制器拐角限速的虚拟圆弧 `r = δ·cos(β/2)/(1−cos(β/2))`(δ=$11,§7.2)与 CAM 光顺圆角是同一公式。**CAM 在容差内先插 r≈2.414T 的圆角 → 控制器 JD 判定变成两个浅角(β 减半、r 倍增)→ 限速点大幅减少**。量化证据:Xiao et al., "Real-time smoothing of G01 commands for five-axis machining", *MMT* 2021(三次贝塞尔实时平滑,**加工时间缩短 >26%**,https://www.sciencedirect.com/science/article/abs/pii/S0094114X21000653 );综述:Yan 2023(http://www.jamstjournal.com/en/article/doi/10.51393/j.jamst.2023001 );line-arc 混合过渡(https://cad-journal.net/files/vol_19/CAD_19(4)_2022_854-867.pdf )。

**双容差模型**(Fusion 语义:https://forums.autodesk.com/t5/fusion-manufacture-forum/understanding-smoothing/td-p/6636189 ):**Tolerance**(线性段几何偏差上限)与 **Smoothing**(弧拟合容差)分离,smoothing ≥ tolerance(官方建议 1:1,激进 1:4);NX Corner Smoothing、Siemens 840D G645 平滑运动同族(手册页未逐字验证)。

**样条输出(G5/G5.2/G6.2/LN)——仅作后置可选项**:控制器支持面碎片化:Fanuc 30i 用 G6.2 + G05.1 Q1(**选配**)、Okuma SuperNURBS(G137)、Siemens 840D 走 COMPCAD/COMPCURV 压缩器(社区反馈常优于原生 NURBS 输出)、Heidenhain LN 块(https://industrialmonitordirect.com/blogs/knowledgebase/nurbs-vs-supernurbs-fanuc-okuma-siemens-mitsubishi-compared );弦差硬约束下的样条拟合文献(能量最小+LSPIA:DOI 10.1016/j.jcde.2015.06.002;微线段弦差约束拟合:DOI 10.1007/s11431-018-9374-6)。**grbl/Marlin 类只有折线+JD,输出样条反而增加解析负担 → 内部永远保留折线+圆弧 IRP,样条只是后置可选**。

**段长规则(落库)**:共线合并(阈值=路径容差)、零长段删除、最短段长下限(经验 >2×JD/公差量级;Aerotech 短段压缩圆角)、段长与控制器插补能力的匹配。

### 5.13 薄壁/低刚度件策略层规则(2026-11-07 补查,不用 FEM 的工艺知识,直接落库)

**权威来源**:Harvey Performance《Your Guide to Thin Wall Milling》全文已抓(https://www.harveyperformance.com/in-the-loupe/thin-wall-milling-accuracy );学术:Zawada-Michałowska 等(MDPI *Materials*, DOI 10.3390/ma13214723 、10.3390/ma15249049)、Kuczmaszewski 2017(*Procedia Engineering*, DOI 10.1016/j.proeng.2017.03.117)、Elhendawy 2023(DOI 10.1504/ijmmm.2023.10061206)、Liu 框体变形控制(DOI 10.4028/www.scientific.net/amr.912-914.1423)。

**规则表(特征 → 策略,落库)**:

| 特征 | 触发条件(初值) | 策略 | 出处 |
|---|---|---|---|
| 高径比 ≥5 且壁厚 ≤1mm | 特征分类器 | 分层(≤2×D/层)+ **两侧交替走刀**(让刀互相抵消) | Harvey(全文验证) |
| 高径比 ≥8 | — | 渐进 RDOC 3–5 步 + 终精 ≤0.2mm | Harvey |
| 任何薄壁精加工 | — | 顺铣一致、小 ae(≤15%D)大 ap、让刀方向交替/对称回路 | Harvey + Forbes 指南 |
| 框体多壁 | — | 工序重排:先粗全部 → 释放应力/回弹等待 → 对称精 | Liu(DOI 验证) |
| 极低刚度(<0.8mm) | — | 提示填料/工装支撑(软件给建议不自动) | Harvey |
| 薄壁+大曲率面(五轴) | — | 可推荐 oval-form 桶刀轻切 | RG 402176743(摘要级,未验证全文) |

**边界**:让刀精确补偿需 FEM/实测——本规则层只做"策略选择与参数收缩",不做变形数值预测(与 §9.12.2 的刚性警示衔接:δ_est 超 → 自动叠加本表策略)。

**优先级**:P1 光顺过渡(收益立竿见影,Xiao 26% 量级,~2 人周)> P2 桶刀 3+2 原型(~4–6 人周,先出行距倍数验证页)> P3 薄壁规则表(纯规则,~1 人周)。**明确不做**:5 轴联动桶刀切触分配与刀姿连续优化、桶刀力/颤振模型、FEM 让刀补偿、NURBS 插补器(控制器职责)。

### 5.14 刀路质量技术全景图(总览与闭环架构,2026-11-07 收束)

**总原则:最大的杠杆是闭环。** 单点算法只能把刀路从 60 分提到 80 分;**生成 → 仿真 → 度量 → 定向修正**的闭环才能到 95+(CNCSIM 实测:同一叶轮靠闭环迭代,解析合格率 47% → 98.2%,过切仅 0.4%)。以下七层按"离闭环远近"组织,全部门类已在 §2~§24 有对应章节。

#### 5.14.1 七层技术全景

| 层 | 技术 | 提升什么 | 位置 | 状态 |
|---|---|---|---|---|
| **一、反馈闭环** | 残料驱动重切(rest/pencil 自动触发) | 凹角残料 | §5.8/P2 | MVP |
| | 偏差驱动重跑(Δ 图超阈区定向细化) | 定向修正 | §8.10/§9.8 | M3+ |
| | 啮合查询(每 CC 点 ae/ap/啮合角) | 喂进给/策略/链接三模块 | §5.11.1 | MVP |
| | 链接校验(快移 vs 毛坯 SDF) | 消擦料 | §5.11.2 | MVP |
| | dry-run 快迭代(只走刀路不切材料) | 迭代速度 | CNCSIM `--dry`(8.3×,建议吸收) | 吸收 |
| | 三口径度量(体素/解析/体积并排) | 验收可信 | §9.7-9.8 | 已定 |
| | 逐点归因(过切反查工序/刀/段) | 可调试性 | §24.5.5 | M3+ |
| **二、策略与几何** | iso-scallop / steep&shallow / 自动 pencil / 螺旋 | 曲面残留均匀、少抬刀 | §5.10 | M2+ |
| | 摆线/自适应开粗;等距环切+过渡 | 恒啮合 | §5.2 | 二期 |
| | 歪刀轴 `tip=p+R(n−a)` | 可达性(不改切出形状) | §24.2 | M6 |
| | 直纹面检测→swarf;桶刀 | 侧刃/大行距 | §5.11.4/§21.4 | M6-C |
| | drop-cutter 落刀(非法向偏置) | 凹角防啃刀 | §5.3 | M2 |
| **三、规划** | 特征识别→操作树;CAPP 排序;TAD 聚类;刀具合并 | 该切的都切、少换刀 | §15/§5.9 | M4/M6-A |
| | 可达性分析前置 | 提前暴露不可达面 | §24.1 | M6-A |
| **四、参数** | 目标 Sa 反解 stepover;恒 MRR→恒力调制 | 质量目标驱动 | §9.11/§5.11.1 | M2 |
| | 切削参数库+修正链;薄壁规则表 | 参数有据 | §16/§5.13 | M1/M4 |
| **五、执行** | 拐角光顺(与 JD 同几何);弧拟合;时序仿真 | 控制器执行质量 | §5.12/§5.6/§7 | M2 |
| **六、输入与知识** | 网格修复管线 | 脏 STL 是错误第一来源 | §2 | M0 |
| | 零点偏置标定;GD&T 语义 | 度量可信、公差定链长 | §9.12/§9.9 | M3 |
| **七、前沿(可选)** | PFD Poisson 全局折中(Zou 2020);5 轴桶刀切触分配;关节空间时间优化;ML/LLM 助手 | 学术顶配 | §5.11.3/§21.4 | 二期+ |

#### 5.14.2 底层资产:仿真毛坯作为刀路生成器的一等输入

七层里第一层(闭环)的三个模块——进给调制、链接校验、策略评分——依赖**同一个接口**:把仿真毛坯从"事后验证工具"升格为"生成过程的查询服务":

```rust
trait StockQuery {
    fn engagement_query(&self, pos: DVec3, tool: &ToolSpec) -> (ae, ap, φ_st, φ_ex); // 啮合几何
    fn sweep_clear(&self, seg: &Seg, tool: &ToolSpec) -> f64;                        // 扫掠最小距离
    fn residual_map(&self, design: &DesignModel) -> ResidualField;                   // 残料/偏差场
}
```

这一层是刀路质量的天花板所在;CNCSIM 已把该模式跑通(其 `--dry`/`link_ok`/`surface_dev` 都是它的具体化),正式化为接口后,§5.11 的两个模块与 §5.10 的自动触发共用同一套查询。

#### 5.14.3 ROI 排序的前五个动作

1. **搭 `StockQuery` 接口**(一喂三:进给调制/链接校验/策略评分);
2. **dry-run 快迭代**(先走刀路后切材料,迭代提速 ~8×);
3. **解析口径验收**(与 h 无关的基准,防止体素假阳性误导调参);
4. **rest/pencil 自动触发**(仿真残料 → 聚类 → 小刀补刀);
5. **iso-scallop 行距调度**(只改栅格行位置逻辑)。

之后进入策略族扩展(steep&shallow / 桶刀 / 摆线)与规划层自动化(CAPP/TAD)——顺序已在 §13 路线图落位。

### 5.15 闭环编排协议(2026-11-07 设计)——把 §5.14 的全景图变成可执行方案

#### 5.15.1 循环状态机

```
[Idle] → [Simulate·dry] → [Measure] → [Diagnose] → [PlanFix] → [Apply] → [Verify] ─┐
   ▲                                                                               │
   └───────────────────────────────────────────────────────────────────────────────┘
   出口(任一满足): ①合格率 ≥ 目标  ②本轮改进 < 0.5pp 且无 S0 动作剩余  ③迭代/时间预算耗尽
   → [Converged] 或 [Escalate·人工](输出:剩余问题清单 + 已试动作 + 建议方向)
```

- 每轮度量用 **dry-run + 解析口径**(快、与 h 无关);**收敛前的最后一轮**才跑全量仿真出体积/预览/色图;
- 预算:`iter_max = min(时间预算 / 单轮 dry 成本, N_max)`;单轮 dry 成本首轮实测后已知(桌腿件 51s 量级)。

#### 5.15.2 失败分类 → 修正决策表(闭环的"大脑",综合 CNCSIM 实证)

| 症状(度量侧) | 根因假设(按概率排序) | 修正动作(代价) | 安全级 |
|---|---|---|---|
| 残料·可达性不足(过切≈0) | ①刀径 > 通道 ②A 超程 ③方位缺失 ④需翻面 | 歪轴(免费)→ 小刀 rest(+切长)→ 加方位 3+2(换向成本)→ 翻面(装夹) | S0~S2 |
| 残料·凹角 | R(1/sin(θ/2)−1) 固有残料 | pencil/rest 小刀(切长 +) | S1 |
| 过切·集中(归因到段) | 落刀方式错(法向偏置啃角)/歪轴硬挤 | 改 drop-cutter / 拒绝该点留料(shank_skips 语义) | S1 |
| 过切·大面积 | 策略不适配(如侧向道啃倒扣) | 关/换策略(以 A/B 对拍定夺,勿信单口径) | S2 |
| 擦料(链接采样) | 安立面不足 | 升 clearance / 绕障 | S0 |
| 尺寸系统性偏差 | 零点偏置 / 让刀 | 标定-补偿(§9.12.1)/ 刚性警示 | S0 |
| 表面纹理超差 | 行距/进给/让刀 | 目标 Sa 反解 / 薄壁规则表(§5.13) | S1 |

每个动作必须登记 **(Δ合格率估计, Δ切长/换刀)** ——首轮用文献/经验先验,第 2 轮起从迭代台账实测学(见 5.15.6)。

#### 5.15.3 修正安全分级(振荡与冲突控制)

- **S0 单调安全**(只加不减、不可能引入新过切):rest 拒点留料、升安全面、挂细格 patch、零点标定补偿 → **自动应用**;
- **S1 局部重验证**(改几何但不破坏下游):进给调制(只改 F 不改几何,天然安全)、方向改变、pencil/rest 刀路 → 应用后**局部 dry 重验证**,恶化即回退;
- **S2 净室重跑**(改上游,下游全失效):换刀径、改 ap/ae、重排工序、翻面 → 触发依赖级联(5.15.5);
- **回退规则**:S1/S2 应用后关键指标恶化 → 从台账回退到上一轮配置(台账见 5.15.6);禁止同一症状连续两轮同类 S1 修正无效后继续(→ Escalate)。

#### 5.15.4 跨工序/跨装夹状态传递

- **程序内活毛坯**:逐段按刀号传递(语义=CNCSIM 已实现的"活毛坯逐段取母线");工序级:op_i 的出口毛坯 = op_{i+1} 的入口毛坯(场引用续用,零拷贝);
- **翻面(第二次装夹)**:残料图/设计网格绕 X 180° = **精确体素索引置换 `(i,j,k) → (i, ny−1−j, nz−1−k)`**(旋转中心取格心时无插值);cut_segs 用同一 Frame 变换;两套工序拼合后**解析口径重算一遍**与体素口径对账;
- **对账不变量(每轮必跑,防度量伪影)**:①体积对账——解析去除体积 vs 体素去除体积差 ≤ 2%(超出 = 度量伪影,禁止进入收敛判定;CNCSIM 教训:侧向啃刀曾被残料掩盖);②收敛判定**只用解析口径**,体素/体积口径仅旁证;③假阳性守卫——体素过切点做归因抽查,归因失败率 >50% → 提示升分辨率而非改刀路。

#### 5.15.5 失效级联与最小重算范围

- **依赖图**:op_j 依赖 op_i ⟺ op_i 的修改改变了 op_j 的入口毛坯(判定:AABB 膨胀 2×刀径相交,即"影响锥");
- **最小重仿真**:从最早被改的工序起级联全部下游;未受影响支线复用缓存;结合 §8.10 patch——只对被改工序的时空范围(段 AABB 并集)做细格重跑;
- UI 的"已过时=重算队列"(§20.6)即本机制的呈现层。

#### 5.15.6 迭代台账(可复现、可回退、可解释)

```rust
struct IterationLedger {
    round: u32,
    config_diff: Vec<ParamChange>,      // 本轮改了什么(动作、参数、S 级)
    metrics: Metrics { pass_rate, gouge, residual, cut_len, swaps, time },
    artifacts: { delta_map, patch_list, skipped_ops, attribution_summary },
    verdict: Continue | Converged | BudgetExhausted | Escalate,
}
```
- 持久化到工程文件(§20.3 的 `document.ron` 同级 `iterations/`),支持任意轮回退与"为什么现在是 95%"的解释;色图可按轮次切换对比;
- 台账同时是 5.15.2 中 (Δ合格率, Δ代价) 估计的数据来源——闭环第 2 轮起从"先验驱动"过渡到"实测驱动"。

#### 5.15.7 多目标决策

- 缺省目标函数:`maximize(合格率) s.t. 切长 ≤ L_budget ∧ 换刀 ≤ N_t`;用户滑条(快↔好,§5.7)映射为 L_budget 与 vf_nom 的松紧;
- 应用顺序即隐式权重:**先穷尽 S0(免费)→ 达标即停 → 不达标才花 S1 → S2 需确认**(CNCSIM 实例:rest +29% 切长换 +2.1pp 合格率,是否值得交给用户);
- 明示:闭环不做自动多目标帕累托搜索(范围外),滑条 + 台账对比已覆盖工程需求。

#### 5.15.8 MVP 边界

- **M3 起(验证档交付时)实现**:dry-run 迭代 + S0 修正自动应用 + 三遍算法进给调制 + 台账(内存态)+ 对账不变量;
- **M6-A 起**:跨装夹传递(翻面置换)+ 依赖级联 + S2 净室重跑 + 台账持久化;
- **明确不做**:自动多目标搜索、机器学习式修正策略、无人工确认的 S2 级重排。

---

## 6. G-code 方言、解析与生成(自研)

### 6.1 规范母本(LinuxCNC RS274/NGC,一手文档)

- 总表:https://linuxcnc.org/docs/html/gcode.html ;G 代码详情:https://linuxcnc.org/docs/html/gcode/g-code.html ;语言总览(行结构/模态组/执行序):https://linuxcnc.org/docs/html/gcode/overview.html

核心规则(已逐条核验):
- 行 = 可选 `/` 块删除 + 可选行号 N + 词(字母+数) + 注释;行长 ≤ 256 字符;大小写不敏感(注释除外);空白任意;
- **模态组**(每组同时只能一个生效):组1 运动 G0/1/2/3/38.x/80~89;组2 平面 G17/18/19;组3 距离 G90/91;组4 圆心模式 G90.1/91.1;组5 进给 G93/94/95;组6 单位 G20/21;组7 刀径补 G40/41/42;组8 刀长 G43/43.1/43.2/49;组10 回退 G98/99;组12 坐标系 G54–G59.3;组13 路径 G61/61.1/64;
- **执行顺序与书写顺序无关**(固定序):注释→进给模式→F→S→T→M6→冷却→ dwell→平面→单位→刀径补→刀长补→坐标系→路径控制→距离模式→G98/99→G10/52/92→**运动**→停止;
- **圆弧**:IJK 为起点到圆心偏移(G91.1 默认)或绝对(G90.1);整圆用 IJK 且省略平面内轴词;P 词 = 多圈数;**R 格式:正 R <180°、负 R >180°、不能整圆**;半径一致性校验阈值 0.05mm(inch 0.0005in);
- **固定循环**:G81(钻)、G82(P 停留)、G83(啄钻,Q 步进)、G85/89(镗);R 平面 G90 绝对/G91 相对;G98 退回初始面、G99 退回 R 面;L 重复;G80 取消;
- 注释 `( )` 与 `;` 两种;`%` 程序包裹规则;活跃注释 `(MSG,)` 等。

### 6.2 grbl v1.1(首个目标方言)

- 命令:https://github.com/gnea/grbl/wiki/Grbl-v1.1-Commands ;设置:https://github.com/gnea/grbl/wiki/Grbl-v1.1-Configuration ;协议/错误码:https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface
- 支持子集:G0/1/2/3、G17/18/19、G20/21、G38.2~38.5、G53/54~59、G90/91、G91.1(仅)、G93/94、G40(仅解析)、G43.1/G49、G4/G10 L2/G28/G30/G92;M0/1/2/30、M3/4/5、M7/8/9。**无固定循环、无 G41/42、无 G59.1-.3**;
- $ 系统:`$$/$#/$G/$I/$C 校验模式/$X/$H 回零/$J 点动`;实时字符 `~ ! ? 0x18` 与 0x80-0xFF 倍率族;
- 串口 RX 缓冲 128 字符 → **字符计数发送协议**;规划器 16 块;ERROR 1–38 / ALARM 1–10(圆弧几何错误 32–36)。
- grblHAL([grblHAL/core](https://github.com/grblHAL/core))扩展:固定循环 G73/G81–89、M6 换刀、NGC 表达式(可选)——作为"进阶方言档"。

### 6.3 方言特性矩阵

| 特性 | LinuxCNC | grbl v1.1 | Marlin | Fanuc/HAAS |
|---|---|---|---|---|
| 固定循环 | G81-G89 全 | **无**(grblHAL 有) | 无 | G73-G89 全,HAAS 归组 09 |
| 圆弧 IJK | G90.1/G91.1 双模式 | 仅增量 G91.1 | IJK+R | IJK+R,整圆必须 IJK |
| 圆弧 R | 支持(±180° 规则) | 支持 | 支持 | 支持 |
| 刀径补 | G40/41/42 完整 | 仅 G40 | 无 | 完整(HAAS 另有 G141) |
| 刀长补 | G43/.1/.2/G49 | G43.1/G49 | 无 | G43/G44/G49 |
| 坐标系 | G54–G59.3 | G54–G59 | G54–G59 | G54–G59(+HAAS G110-129) |
| 暂停单位 | P=秒 | P=秒 | P=秒 | **P=毫秒**,X=秒 |
| 注释 | `( )` 与 `;` | `( )` 与 `;` | `( )` 与 `;` | 仅 `( )` |
| 程序号 | 无(用 %) | 无 | 无 | **O1234** 必备 |
| 样条 | G5/G5.1/G5.2/3 | 无 | G5 贝塞尔(https://marlinfw.org/docs/gcode/G005.html ) | 部分 |
| 表达式/宏 | `[ ]` + O 词流程 | 无(grblHAL 可选) | 无 | Macro B |
| 行长 | 256 | 128(error:11) | 宽松 | 宽松 |

HAAS G 代码表:https://haascnc.com/service/online-operator-s-manuals/mill-operator-s-manual/mill---g-codes.html ;方言差异综述:https://www.cnccookbook.com/cnc-programming-g-code 、G04 陷阱:https://www.cnccookbook.com/g04-gcode-pause-dwell

### 6.4 自研解析器四层架构

```
L1 词法   行 → Token 流(词 letter+value、注释、行号、块删除);保留源位置(行/列)
L2 Block  Token → 词袋 + 同行多 G 组冲突检查(grbl error:21/25 语义)
L3 状态机  Block + 模态状态 → 语义事件流 Instruction:
          Motion{G0/G1.., from, to, feed} / Arc{center_mode, cw, turns, helix} /
          Dwell{sec} / Cycle{G81.., r_plane, retract_mode} / SetOffset / ToolChange ...
L4 执行   固定循环展开成 Motion 序列(见 §7.4);坐标链换算(§7.5);校验器消费同一 IR
```

- 词法可用 winnow/nom 辅助,但**建议手写**(几百行,零依赖,且要支持"数字内部空格"等历史行为时可自行取舍);
- **方言 = 数据驱动 profile**(YAML/RON):注释形式、小数位、圆弧格式、循环展开开关、M6 降级规则、暂停单位、程序头尾模板等(清单见 6.6);
- 参考解析器(读思想):grbl `gcode.c`(模态组逐组校验;https://github.com/gnea/grbl/blob/master/grbl/gcode.c );LinuxCNC `src/emc/rs274ngc/`(已拆分为 interp_read/interp_execute/interp_convert/interp_arc/interp_cycles 等 43 文件;https://github.com/LinuxCNC/linuxcnc/tree/master/src/emc/rs274ngc );Kiri:Moto `export.js`(https://github.com/GridSpace/grid-apps );joewalnes/gcode-viewer(MIT,极简参照,https://github.com/joewalnes/gcode-viewer )。

### 6.5 Rust G-code crate 现状(均不采用,仅记录;crates.io API 核验)

`g-code`(0.6.0,rust-peg,https://github.com/sameer/g-code )、`gcode`(0.7.0,no_std,https://github.com/Michael-F-Bryan/gcode-rs )、`ngc`(LinuxCNC 方言,4 年未更,https://github.com/birkenfeld/ngc )、`g-win`、`gcode-nom` —— 全部止步"块→词"抽象,无模态组/方言/循环展开,验证了自研的必要性。

### 6.6 生成器与后处理规则(默认输出)

- 启动安全块 `G17 G21 G90 G94 G40 G49 G80`;圆弧一律 IJK(G91.1),禁 R(除非 profile 允许且转角在 15°–165° 安全区);坐标 mm 3 位 / inch 4 位小数;每行一个组 1 运动指令;行尾注释携带操作元数据(便于回读/统计);
- 为 grbl 输出:展开固定循环、去 `%`、M6 → M0+注释、圆弧 $12 容差约束下检查半径一致性;
- **Fusion 360 CPS 后处理体系**作为 post 设计蓝图(314 页官方训练指南:https://cam.autodesk.com/posts/posts/guides/Post%20Processor%20Training%20Guide.pdf ):全局区(格式定义/模态组/属性表)+ 回调(onOpen/onSection/onLinear/onRapid/onCircular/onDrilling)——我们的 profile + 生成器即此结构的静态化。

### 6.7 校验器

行程限位(G53/G92/G54 换算后)、**G0 横移是否低于安全面**(高危)、圆弧半径一致性(照抄 LinuxCNC 阈值)、F 未定义、% 包裹完整性、M2/M30 缺失;错误带行号可跳转。grbl 语义对照:软限位超程是 ALARM:2 而非语法错误(https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface )。

---

## 7. 运动规划仿真(grbl 模型自研)

### 7.1 grbl 规划器机制(一手源码)

- 规划缓冲 16 块([planner.h](https://github.com/gnea/grbl/blob/master/grbl/planner.h));段缓冲 6 段 × 10ms 时间片([stepper.c](https://github.com/gnea/grbl/blob/master/grbl/stepper.c)、[config.h](https://github.com/gnea/grbl/blob/master/grbl/config.h));
- 规划 = **两遍传播**:反向算拐点速度 → 正向下压 entry 速度([planner.c](https://github.com/gnea/grbl/blob/master/grbl/planner.c),注释原文核验)。

### 7.2 Junction Deviation 转角限速公式(完整,自研照此实现)

符号:`u_in/u_out` 前后单位方向向量;`d` = $11(mm);`a_j` 转角允许加速度;`v_j` 转角通过速度。

```
cosθ = −(u_in · u_out)                        # grbl: junction_cos_theta
sinHalfθ = √(0.5·(1 − cosθ))                  # 半角恒等式,无三角函数
r = d · sinHalfθ / (1 − sinHalfθ)             # 虚拟内切圆半径
v_j² = a_j · r                                # 向心加速度约束 v²/r ≤ a
junction_unit_vec = normalize(u_out − u_in)
a_j = min_axis( accel[axis] / junction_unit_vec[axis] )   # 分轴限幅

特例(planner.c 原文):
cosθ >  0.999999  →  掉头,v_j² = MINIMUM_JUNCTION_SPEED² (=0)
cosθ < −0.999999  →  共线,不限速(由名义速度与前后传播钳制)
入口钳制:max_entry_speed² = min(v_j², min(v_nominal_prev², v_nominal²))
```

出处:planner.c 源码;几何解释:grbl 作者 Sonny Jeon, "Improving Grbl: Cornering Algorithm"(2011):https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm ;独立推导:RepRap 论坛 "Junction Deviation Explained and Visualized":https://reprap.org/forum/read.php?1,739819 。该偏差是**纯虚拟量**,刀具实际不偏离路径。

### 7.3 梯形速度规划(闭式方程)

块参数:长 L、入口 v_s、出口 v_e、名义 v_n、加速度 a。

```
可达峰值 v_p = √( (2aL + v_s² + v_e²) / 2 )      # 同时出现在 grbl stepper.c 与 LinuxCNC time_estimate.cc

梯形(v_p ≥ v_n): d_a=(v_n²−v_s²)/2a; d_d=(v_n²−v_e²)/2a; d_c=L−d_a−d_d
                 t = (v_n−v_s)/a + (v_n−v_e)/a + d_c/v_n
三角形(v_p < v_n): d_acc = 0.5·(L + (v_s²−v_e²)/(2a))      # grbl stepper.c 原文
                 t = (v_p−v_s)/a + (v_p−v_e)/a
```

### 7.4 块间传播(自研伪代码,对应 planner.c)

```
反向 pass(队尾→队首):
  b.entry² = min(b.max_entry², 2·a·L)
  if next.entry² + 2aL < b.entry²: b.entry² = next.entry² + 2aL
正向 pass(队首→队尾):
  if b.entry² < next.entry²: next.entry² = min(next.entry², b.entry² + 2aL)
断点(block_buffer_planned):entry 达到 max_entry 的块;新块入队从断点增量重算
倍率变化:当前实际速度回写为 entry²,触发重规划(st_update_plan_block_parameters)
```

### 7.5 圆弧细分与坐标链

- **弦误差分段**(grbl [motion_control.c](https://github.com/gnea/grbl/blob/master/grbl/motion_control.c) 原式):

  ```
  segments = floor( 0.5·|Δθ|·r / √(tol·(2r − tol)) )    # tol = $12 弦高容差
  # 推导:矢高 s = r − √(r² − (l/2)²) 令 s=tol ⇒ 弦长 l = 2√(tol(2r−tol))
  ```

- 坐标变换链(LinuxCNC [overview](https://linuxcnc.org/docs/stable/html/gcode/overview.html)):机床坐标(G53)→ G54–G59.3(参数 5221–5390)→ G52/G92(5211–5219)→ 刀长 TLO(5081–5089)= 编程位置;grbl 状态报告 `WPos = MPos − WCO`;
- **G81 展开**(LinuxCNC [g-code.html #g81](https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g81)):① 快速 XY 定位 → 快速 Z 至 R 平面(必要时先抬)→ ② 进给 Z 至孔底 → ③ 快速退回 clear Z;G98 退回循环前 Z(仅当高于 R),G99 退回 R;R 恒 sticky;L 重复;`R < Z` 报错。**G83**:每啄进给 δ → 快速退到 R → 快速下到"孔底 − 0.254mm" → 重复(每次全退到 R)。

### 7.6 路径模式与时间估算

- G61 精确路径(共线不停)/ G61.1 精确停 / G64(可带 P 容差、Q 合并)(https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g64 )。grbl 无此选择,恒为"JD 连续 + 前瞻",等价固定容差 G64;
- 时间估算参考实现:LinuxCNC [time_estimate.cc](https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/rs274ngc/time_estimate.cc)(前瞻队列 + 反向传播 + 梯形闭式 + 弧限速 `√(0.866·a·r)` + G0 分轴钳制);CAMotics `LinePlanner.cpp/SCurve`(S 曲线与 JD 混合,https://github.com/CauldronDevelopmentLLC/CAMotics );
- 经验数据:不考虑加减速的估算可偏差 5~10 倍(gnea/grbl#893:https://github.com/gnea/grbl/issues/893 )——故时间轴必须走规划器模型。

### 7.7 仿真器实现规格(L2 保真级)

1. 解析层(§6)输出 Motion 流(固定循环已展开、弧按 $12 分段或作为原生弧块);2. 块队列 `VecDeque<Block>`(容量 16):每块存 L、单位向量、v_nominal(受 F 与各轴 $110-112 钳制)、a_j(§7.2)、entry/exit;3. 双 pass 传播;4. 闭式剖面(§7.3)求 t_total 与段边界;5. 按 10ms(或可调)网格输出带时间戳刀位:直线 s(t) 线性、弧按角插值;G0 用分轴限速合成矢量速度;6. dwell/换刀/M 停直接加时;7. feed override 改名义速度并局部重规划。
- 校验对拍:[grbl-sim](https://github.com/grbl/grbl-sim)(GPL-3.0)编译真 grbl 源码输出 block/step 曲线 —— **仅作为外部对照工具运行**,不链接其代码。
- **五轴注意**:grbl 模型仅覆盖线性轴;回转轴运动学、RTCP 进给语义与奇异处理见 §23.4/§23.6。

---

## 8. 材料去除仿真(自研:Z-map → SDF)

### 8.1 方法对比

| 方法 | 精度 | 内存(100×100mm 毛坯) | 速度 | 实现难度 | 适用 |
|---|---|---|---|---|---|
| **Z-map 高度场** | 平顶刀好;陡壁/垂直面受限(2.5D) | 0.1mm 格 ≈ 4MB;0.02mm ≈ 100MB | 最快 | **低** | 三轴首选 |
| Tri-dexel | 各向均衡,可表达侧壁 | ~0.5mm:3MB;0.1mm:~70MB | 中 | 中高 | 二期备选 |
| Voxel/八叉树 | 分辨率受限(CAMotics 用 MC33 提升亚格精度) | 0.1mm 全格 1e8 点级 | 中 | 中 | 通用 |
| **SDF + MC** | 亚体素等值面,法线精确 | 稠密同 voxel;可窄带压缩 | 中 | 中 | 二期首选 |

理论谱系:Jerard & Drysdale, "Discrete Simulation of NC Machining", *Algorithmica* 4(1):33–60, 1989;S.K. Lee & S.L. Ko, "…enhanced Z-map model", *JMPT* 130–131:344–350, 2002, DOI 10.1016/S0924-0136(02)00761-6(https://www.sciencedirect.com/science/article/abs/pii/S0924013602007616 ,超采样提效);综述:Bouhadja & Bey 2014(https://www.iaeng.org/publication/WCE2014/WCE2014_pp992-997.pdf ,抓取失败未验证)。

### 8.2 Z-map 自研规格(MVP)

**数据结构**:`h: Vec<f32>`(W×H,每格当前顶面高)+ `z_min` 底面 + 可选 `last_touch: Vec<u32>`(最后接触时间,做纹理/溯源)。

**更新操作**(每条 G 代码移动):
1. 路径离散:直线按 `ds ≤ cell/2` 采样;圆弧按矢高 `ds = 2√(R²−(R−cell/2)²) ≈ 2√(R·cell)`;
2. 每个采样点:刀具覆盖 AABB 内每格 `(i,j)` 计算径距 `d`,求刀底高度 `z_tool(i,j)`,执行 `h[i,j] = min(h[i,j], z_tool)`;
3. **刀底闭式**(径距 d,刀尖 z0):
   - 平底(R):`d ≤ R → z = z0`
   - 球头(R):`d ≤ R → z = z_center − √(R²−d²)`
   - 圆鼻(R, rc):`d ≤ R−rc → z = z_flat;R−rc < d ≤ R → z = z_corner − √(rc²−(R−d)²)`,其中 `z_corner = z_flat + rc`
   - 锥刀:底/顶半径线性插值
4. 圆弧移动先离散为刀位点序列再逐点做 2(螺旋 = 圆弧 + Z 线性)。

**复杂度**:`O(L/ds × (2R/cell)²)`;Φ10 刀、0.1mm 格、1m 路径 ≈ 2×10⁸ 次 min,Rust + rayon(按行条带并行写不相交区)秒级;陡壁伪影用 Lee & Ko 超采样(每格 2×2 子样取 min)缓解;渲染法线 = 高度场中心差分 `n = normalize(−∂h/∂x, −∂h/∂y, 1)`。

### 8.3 CAMotics 真实架构(重要纠偏 + 借鉴)

**"CAMotics 用 tri-dexel"是误传**——源码 GitHub Code Search `dexel` 出现 0 次(2026-11-07 核验)。真实架构(源码逐文件核验,https://github.com/CauldronDevelopmentLLC/CAMotics ,GPL-2.0+):
- 毛坯 = 自适应网格采样场 `GridTree`(标量场 f(p),f≥0 在材料内);
- 每条 move 生成解析扫掠深度函数 `Sweep`(`ConicSweep/SpheroidSweep/CompositeSweep`,接口 `depth(p)`);
- `OctTree` 只做 **move 索引**(`insert(move,bbox)`),使每格只需查询与其 AABB 相交的少数 move —— 复杂度从 O(格数×move 总数) 降为 O(格数×相交 move 数);
- 表面提取:`MarchingCubes` + **CorrectedMC33**(还有 CubicalMarchingSquares/QEF 备选,`src/camotics/contour/`);
- 手册口径:毛坯限长方体、分辨率即仿真网格、结束做一次可视化平滑(https://camotics.org/manual.html )。

**可照抄的骨架(思想层面)**:`毛坯 = 场 + 移动索引 + 扫掠深度函数 + MC 抽面`,与我们 SDF 路线完全同构。

### 8.4 SDF 法自研规格(二期)

- 毛坯:长方体 = 解析 box SDF;STL 毛坯 = 体素化 + 两遍 3D 距离变换(重距离化);
- **扫掠体闭式 SDF**(公式全部出自 Inigo Quilez,已抓取核验:https://iquilezles.org/articles/distfunctions/ ):
  - 胶囊(直线扫掠):`sdCapsule(p,a,b,r)`(clamp 投影 + 距离 − r);
  - 圆鼻刀直线扫掠:`sdRoundedCylinder(p, ra, rb, h)` 直接可用;平底刀 = 胶囊 + box 修平底;球头 = 胶囊/球组合;
  - 圆弧扫掠:弧平面内投影 + 角度 clamp 后同 capsule;旋转对称刀 = 变半径 torus 片段;
  - 复合刀形:CSG `max/min`;`opSmoothUnion`(页面明确:smooth 结果是界不是精确 SDF);
- 布尔减:`S_new(p) = max(S_old(p), −S_sweep(p))`,只在扫掠 AABB 内更新;**注意:减法/交只是 bound,多次减后 SDF 退化为界 → 周期性重距离化**(iquilez 页面原文;学术支持:Sullivan, Erdim, Jerard, Drysdale, "High accuracy NC milling simulation using composite adaptively sampled distance fields", *CAD*);
- 表面提取:Marching Cubes(见 8.5),法线 = ∇SDF 中心差分。

### 8.5 Marching Cubes

- 原始论文:Lorensen & Cline, "Marching Cubes: A High Resolution 3D Surface Construction Algorithm", *SIGGRAPH '87*, 21(4):163–169(全文:https://www.cs.toronto.edu/~jacobson/seminar/lorenson-and-cline-1987.pdf );
- 查找表(8 位 cube index → edgeTable[256] → triTable[256][16],边插值):Paul Bourke, "Polygonising a scalar field":http://paulbourke.net/geometry/polygonise/ (已抓取核验);
- 歧义问题:Bourke 页注明面片拼接一致性难题;工业解 = **MC33**(Chernyaev;CAMotics `CorrectedMC33*.cpp` 可对照)或 Marching Tetrahedra(无歧义);
- **替代方案 Surface Nets(对偶法)**:免 LUT/免歧义表、代码量小、面过渡平滑(锐边保留弱于 MC33);CNCSIM 已用它替代 MC 并实测(200mm 毛坯提取 3200 万面 / 578ms @ 24 线程)——**MC33 与 Surface Nets 二选一即可**:锐边特征要求高选 MC33,通用仿真选 Surface Nets 更省;
- 三向 dexel 重建文献(若走 tri-dexel):Ren, Zhu 等, *Computer-Aided Design and Applications* 5(6):932–941, 2008,免费 PDF:https://www.cad-journal.net/files/vol_5/CAD_5(6)_2008_932-941.pdf ;Inui 2019 triple-dexel:https://cad-journal.net/files/vol_16/CAD_16(1)_2019_89-102.pdf 。

### 8.6 扫掠体理论与采样误差

- 经典理论:Blackmore & Leu 扫掠包络微分方程(1992/1997,NC verification 应用)——工程上不照抄;实用直觉:**纯平移扫掠 = 刀具实体 ⊕ 路径(Minkowski 和)**,球头直线扫掠 = 胶囊,平底 = rounded box;
- 采样并集近似误差:相邻刀位间漏掉的月牙深 ≈ 矢高 `ds²/(8·R_eff)`;平底刀垂直进给时 R_eff→0 → 必须 `ds ≤ cell/2` 兜底。

### 8.7 碰撞检测(自研)

G0 快移/下刀时刀柄(锥面)/夹头 vs 毛坯与夹具模型:自研 BVH 射线查询 + 三角形求交(§4.1);刀柄用圆柱+锥体组合的解析表示;快速路径:与 Z-map 高度场比较刀柄包络最低点。GPU voxel 加速先例:Schnös 等 2021, *IJAMT*, https://link.springer.com/article/10.1007/s00170-021-07001-w 。**机床部件/夹具凸体对(凸包近似)用 GJK(最小距离)+ EPA(穿透深度)**——CNCSIM collide crate 已实证(34 测试,含接触 episode 聚类);非凸机床部件先凸分解或保守包围体。

### 8.8 演进路线

**MVP = Z-map**(清单见 §13);**二期 = SDF + MC**(理由:闭式扫掠公式现成、陡壁/任意毛坯/五轴可扩展、与 CAMotics 已验证架构同构);tri-dexel 仅当需要精确切削啮合(CWE)/切削力耦合时立项。Rust 同生态参考:[gsim-rs](https://github.com/navrajkalsi/gsim-rs)(MIT,voxel + wgpu,验证过存在)。

**五轴需求生效后(2026-11-07)修订**:tri-dexel 优先级**上调** —— 3+2 需要"每方位 Z-map → 合并 tri-dexel"(Inui 2022),五轴连续必须三向 dexel/体素表示(§24.3);SDF/tri-dexel 路线从"二期可选"变为"五轴里程碑的依赖项";Z-map 仍保留为三轴快速预览与 3+2 单方位引擎。

### 8.9 体素分辨率定标(VDB 引擎,2026-11-07 补充)——h 取多少才够

**反推链:精度需求 → 体素上限。**

- **规则 1(表面定位误差由表示质量决定)**:二值体素(布尔占格)的表面误差 ≈ 1×h(阶梯);**SDF 窄带 + MC33 + 梯度插值 ≈ h/4 ~ h/2**(亚体素等值面)——即 SDF@0.05mm 的表面保真 ≈ 二值@0.0125mm;
- **规则 2(过切检测阈,§9.8 差分场语义)**:要可靠检出深度 `t_g` 的过切,需 `h ≤ t_g/2`(二值)或 `h ≤ 2·t_g`(SDF+亚体素);
- **规则 3(对比有效下限,§9.8)**:偏差对比的有效下限 = 仿真格宽 → 体素大小就是报告敢承诺的精度;
- 典型公差:模具级 0.02–0.05mm、通用件 0.1mm、精密件 0.01mm。

**可行域(内存与更新成本)**:VDB 省内存的两个来源 = 窄带(只存表面附近)+ tile 折叠(实体内部/外部为常值子树)——**毛坯实心没关系,存的只有表面壳**。窄带体素数 `N ≈ 6 × A_表面 / h²`(两侧各 3 体素),内存 ≈ `N × 5B × 1.5`(f32+位掩码+VDB 开销):

| 体素 h | 简单件(A≈3×10⁴mm²) | 复杂模具件(A≈1.5×10⁵mm²) | 更新成本(Ø10 刀,1m 刀路) |
|---|---|---|---|
| 0.2mm | ~5 MB | ~30 MB | 秒级 |
| 0.1mm | ~130 MB | ~650 MB | 5–15 s |
| **0.05mm** | **~550 MB** | **~2.7 GB** | **30–90 s(rayon 并行)** |
| 0.02mm | ~3.4 GB | ~45 GB ✗ | 10×↑ |
| 0.01mm | ✗ | ✗ | — |

- 更新成本公式:每 mm 路径触及体素 ≈ `π·r²/h³`(Ø10 @0.05mm ≈ 6.3×10⁵ 体素/mm;10m 刀路 ≈ 6×10⁹ 次,每次为解析胶囊/圆环 SDF 求值 + max 写)——符合 §22"典型件 ≤1 分钟可用"目标;
- **与 tri-dexel 交叉验证**:0.05mm tri-dexel ≈ 290MB/100mm 件(§8.1 公式)——同精度下两种表示内存同量级,互为印证;
- **结论:0.02mm 及以下的全局体素在 CPU 侧不可行**(内存与更新双双爆炸);0.01mm 公差档必须改局部细化或解析评定(§9.11 B)。

**分级建议**:

| 档位 | 体素 | 用途 | 精度承诺 |
|---|---|---|---|
| 预览档 | 0.2mm | 拖动回放、教学 | 仅可视化 |
| 标准档 | 0.1mm | hobby/通用验证(公差 ≥0.1mm) | 等效 ~0.025–0.05mm |
| **验证档(默认)** | **0.05mm + SDF/MC33** | 模具级(公差 0.05mm) | **等效 ~0.012–0.025mm** |
| 局部精修档 | 关注区自适应 0.025mm | 高公差区 | VDB 天然多分辨率:平坦区 0.1、公差关键区(用户标记或曲率/偏差场驱动)2× 细化 |
| 0.01mm 公差 | ✗ | 改走 dexel 局部对比或解析评定(§9.11 B) | 全局体素内存爆炸 |

**四条工程守则**:
1. **分辨率=精度必须显式化**(CAMotics 手册同款口径:分辨率即仿真网格,复杂度随分辨率快速增长并给出警告,https://camotics.org/manual.html )——启动前按包围盒+表面积预估内存/耗时,超阈值自动降档并提示;
2. **SDF 是前提**:同样 0.05mm,SDF+MC33 比二值体素精度高约 4 倍——二值体素做验证档需要 0.01mm(不可行);
3. **周期性重距离化**(§8.4):布尔减累积使 SDF 退化为界,每 N 万次更新或每工序在窄带内重距离化一次;
4. **树参数**:4³/8³/16³ 节点 + 叶 8³(§24.5.8 规格),背景值取 +3×窄带宽度,ValueAccessor 每线程一个、写共享不安全(OpenVDB overview:https://www.openvdb.org/documentation/doxygen/overview.html )。

### 8.10 多分辨率仿真(分区 patch 方案,2026-11-07 设计)

**三种"多分辨率"辨析**:①显示 LOD(仿真格恒定,渲染降采样——CNCSIM k=4 属此类,是渲染预算不是精度);②真自适应 ADF(Frisken 2000,单树内体素物理尺寸随节点变化——跨分辨率求值与 Surface Nets 跨分辨率拓扑缝合是研究级难题、确定性难保,**不做**);③**分区多网格 patch(推荐)**:主网格全局 h_coarse,关注区挂独立细格 VDB 子树。注意 VDB 的稀疏是**拓扑**多分辨率(哪里有数据),不是**几何**多分辨率(数据多细)——一棵树内体素尺寸恒定,真多分辨率 = 多棵树并存。

**结构**:

```rust
struct MultiResSim { main: SparseGrid, patches: Vec<Patch> }   // 主网格引擎零改动
struct Patch { id, bbox: BBoxI /*主格坐标系,对齐格心*/, grid: SparseGrid,
               origin: DVec3, creator: CreatorMap, state: Active|Sealed }  // h_fine = h_coarse/k, k∈{2,4}
```

**三条铁律**:①**patch 区主网格让位**——主格照常粗更新管"去了多少料"(体积口径),表面查询/提取/偏差/碰撞以细格为权威(表面口径),不做双写同步(复用三口径分离);②**格心对齐**——origin 对齐主格角点、h_fine=h_coarse/k,细格点落在主格网格线二分点上,缝合误差最小;③**逐位可复现**——每段更新顺序固定 patch(按 id 序)→ main;patch 内复用现有 cut 内核(max 交换律保证批量幂等)。

**更新流水线(每刀段)**:①扫掠 AABB(含 band 外扩)→ ②查 PatchRegistry(bbox 相交,patch<64 线性扫)→ ③不命中走现有主格路径(零改动)→ ④命中:AABB∩patch 变换到局部坐标细格切,AABB 残余部分仍走主格 → ⑤体积归账:Σpatch 细格体积 + Σ主格体积 − 重叠区只计一次(防重复计)。

**细格区从哪来(自动为主)**:①用户点选/框选(点选基建与待办"反查 creator"同源);②**上一轮 Δ 偏差图超阈值连通域 → 自动挂 patch 重跑受影响段**(粗扫→定向细扫迭代,与 rest 重切闭环同构);③几何先验(设计面 κrel 大处 / 刀径 < 4·h_coarse 的小刀通道)。

**边界缝合(两方案)**:方案 1(推荐)——patch bbox 四周**外扩 2 主格**也用细格,patch 边界 SDF 与主格在二分点上解析一致,提取时 patch 边框用主格角点补三角,过缝误差预期 ≤ h_coarse/2;方案 2——更新完全独立、提取期手工桥接(简单但缝上有折痕)。

**精度与口径承诺**:patch 区过切检测阈按 §8.9 规则 2 以 h_fine 计;**偏差报告必须带"分辨率覆盖图"**(哪些区域细格口径、哪些粗格)——即 §9.8 可信度图的第一次落地;Σpatch 内存(同 `6×A_patch/h_fine²` 估)超主网格 ~40% 或绝对阈值(2GB)→ 自动降档;碰撞距离取 `min(main_sdf, patch_sdf)`(细格更真,取 min 保守正确——`DistanceField` trait 多实现一个 `MultiResField` 即可)。

**显示层衔接**:patch 区细格 Surface Nets、区外主格,顶点色/creator 规则同构继承;细格面数涨 ~k² 倍,与显示 LOD k=4 **正交组合**(两级预算分开);patch 是天然"分块"单位,与分块增量重建同构,patch 边界格块标 dirty。

**落地路径(基于 CNCSIM 现状)**:①最小验证(~1–2 天):MultiResSim 骨架 + 段分发 + 单 patch 跑通;**验收 = 同 h 时 patch 区域与全细格单树逐体素一致**(最佳回归测试);②边界重叠带 + 提取缝合 + 顶点色继承(check_mesh 体检无可见折痕);③Δ 图自动挂 patch + 覆盖图进报告;验收 = 桌腿件主 0.6/patch 0.15,解析口径与全细格差 ≤ 细格量化半格(预计 76.5s → <20s);④暂不做 ADF,等 patch 数量爆炸再评估。

**何时不需要**:全件单一公差、或仅回放预览时,单分辨率 + 显示 LOD 更简单——价值随公差要求的空间不均匀性增长(桌腿件平面 0.028 vs 均值 0.36,差 13 倍,即教科书案例)。

---

## 9. 模型差异对比 / 验证(自研)

### 9.1 点-三角形精确距离(自研蓝本,已全文核验)

出处:Eberly, "Distance Between Point and Triangle in 3D", Geometric Tools(CC-BY 4.0,全文:https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf ):

1. `T(s,t) = B + s·E0 + t·E1`,定义域 `s≥0, t≥0, s+t≤1`;预计算点积 `a=E0·E0, b=E0·E1, c=E1·E1, d=E0·Dv, e=E1·Dv, f=Dv·Dv`(`Dv = B−P`);
2. 平方距离 `Q(s,t) = a·s² + 2b·st + c·t² + 2d·s + 2e·t + f`;
3. 无符号驻点 `s̄ = be − cd, t̄ = bd − ae`,判别式 `δ = |ac − b²|`(取绝对值防短边浮点抖动);延迟除法;
4. 7 个 Voronoi 区域分类:区域 0(内部,`s=s̄/δ, t=t̄/δ`)、3 条边带(一维二次极小 + clamp)、3 个角区(方向导数符号定邻边/顶点);
5. 同族参考:Ericson《Real-Time Collision Detection》§5.1.5(书籍页:https://realtimecollisiondetection.net/ )。

### 9.2 带符号(内/外)判定:三方法对比

| 方法 | 成本 | 鲁棒性 | 出处 |
|---|---|---|---|
| A 最近三角形法线点积 `sign((P−Q)·n)` | 一次 BVH 最近点 | 表面附近快而准;边缘歧义;依赖绕向一致 | CloudCompare C2M wiki(全文核验):https://www.cloudcompare.org/doc/wiki/index.php/Cloud-to-Mesh_Distance |
| B 射线奇偶性 | ray-BVH | 仅对水密+取向一致网格可靠 | 通用(Ericson 书覆盖) |
| **C 广义环绕数(推荐主判定)** | 朴素 O(n);BVH 聚合实心角 + 远场近似 → O(log n) | 对破洞/自交/多组件最鲁棒,分数值=可疑度 | Jacobson, Kavan, Sorkine-Hornung, "Robust Inside-Outside Segmentation using Generalized Winding Numbers", SIGGRAPH 2013(项目页核验):https://igl.ethz.ch/projects/winding-number/ ;加速:Barill 等, "Fast Winding Numbers for Soups and Clouds", SIGGRAPH 2018:https://www.dgp.toronto.edu/projects/fast-winding-numbers |

**策略**:C 为主(加工仿真常产非水密网格),A 作近表面快速通道;符号约定须统一(注意 trimesh 是"外负内正",https://trimesh.org/trimesh.proximity.html ,与我们"外侧=正=余量"相反,对拍时换号)。

### 9.3 Hausdorff 距离与 Metro 采样

- 定义 `h(A,B) = max_a min_b ‖a−b‖`,双向取 max;奠基:Cignoni, Rocchini, Scopigno, "Metro: Measuring Error on Simplified Surfaces", *CGF* 17(2):167–174, 1998, DOI 10.1111/1467-8659.00236(https://onlinelibrary.wiley.com/doi/10.1111/1467-8659.00236 );
- **自研采样估计流程**:① 在被测面生成样本(顶点/边/面积加权蒙特卡洛/细分精化)→ ② 每样本对参照网格 BVH 做精确最近点 → ③ 增量统计 max/mean/RMS + 直方图/分位数 → ④ 交换方向再算 → ⑤ **双向分开报告**(单向结果强依赖采样面,MeshLab 博客实证:http://meshlabstuff.blogspot.com/2010/01/measuring-difference-between-two-meshes.html );
- 工程要点:max-dist 截断防离群拖垮耗时(CloudCompare 参数);max 对采样密度敏感 → 报告 p95/p99 + 直方图为主;PyMeshLab `hausdorff_distance` 参数命名可参照(https://pymeshlab.readthedocs.io/en/0.1.8/filter_list.html )。

### 9.4 过切/欠切判定与色图

- 以设计面外法线 `n` 定义**有符号代数偏差** `δ = (P − Q)·n`:`δ < −t_g → 过切`;`δ > +t_s → 余量`;带内 OK;斜面处代数偏差 ≠ 欧氏距离(≈ δ/|cosθ|)→ **双轨**:欧氏距离做 Hausdorff 统计,代数偏差做分类上色;
- 色图三段式:中性=带内、蓝→黄=正余量梯度、红=过切;`t_g/t_s` 可调;统计面板 mean/RMS/p95/p99/max + 直方图(UI 工作流照 CloudCompare C2M:角色显式化、色带可编辑、上下限手动 clamp、样本可导出);
- 工业对标:Vericut AUTO-DIFF —— 设计模型 vs 仿真件,全程 gouge/excess 检查(https://vericut.com/products/auto-diff );学术:Huang 1994 "NC Milling Error Assessment and Tool Path Correction"(Iowa State,https://dr.lib.iastate.edu/bitstreams/2b2403f9-d2b7-4e8b-a3c2-4c4f12260bb9/download )。

### 9.5 与仿真器内部表示衔接

**L0 高度场逐列对比(先做)**:

```
for (i,j) in 栅格:
    x,y = O + (i·h, j·h)
    zd  = design_top_z(x,y)          # 设计模型 BVH 沿 +Z 射线取最高面;无命中 → UNDEFINED
    d   = Zm[i,j] − zd               # 正=余量, 负=过切
    cls = GOUGE if d < −g else EXCESS if d > s else OK
    vol_gouge += (zd − Zm)·cell² ; vol_excess += (Zm − zd)·cell²   # 体积对账
```

陷阱:垂直壁/倒扣是 2.5D 盲区 → 标 UNDEFINED 而非 0;"设计最高面"语义要与加工面一致(型腔底会被顶面遮蔽,需沿 −Z 从材料内取面);多 Z-map 拼缝。

**SDF 差的语义陷阱**(iquilez 全文核验):`E(a) − E(b)` 既不是 `a−b` 的 SDF 也不满足距离不等式 —— 只可做快速预览;精确值一律"采样点 + BVH 最近点 + 环绕数符号";剩余材料场 `E_stock(p)` 本身就是 p 到已加工面的带符号距离,可直接消费。

### 9.6 对齐(简述)

同一机床坐标系下无需对齐;在机测量/首件检验用 ICP best-fit:Besl & McKay, "A Method for Registration of 3-D Shapes", *IEEE TPAMI* 14(2):239–256, 1992, DOI 10.1109/34.121791(免费 PDF:https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf );点面变体 Chen & Medioni, ICRA 1991。报告须注明"坐标系直比"还是"best-fit 后"。

### 9.7 分级实现(2026-11-07 修订:验证次序 = 先找全 → 再量准 → 后标可信度)

- **L0 Z-map 逐列对比**(1–2 周,三轴预览):色图 + 体积对账;**必须标注盲区**(垂直壁/倒扣/窄沟槽,见 9.8);
- **L1 逐单元差分场(存在性,无盲区)——"验证"的核心**:在共享网格(体素/dexel/SDF)表面窄带逐单元比较内外符号,翻转连通域 = 差异区域候选(过切域/余量域);三轴可用 Z-map 退化实现,五轴为方向无关表示(§24.5);
- **L2 采样量化**:Eberly 点-三角形 + 自研 SAH-BVH + 环绕数符号 + 双面面积加权蒙特卡洛采样 → 有符号色图 + 统计 + 双向 Hausdorff(Metro 式细分精化 max)+ 公差带工作流 + 样本导出 + 可选 ICP;
- **L3 精确化与诚实化**:跨域二分细化(亚单元边界)、偏差场等值面(公差带边界面,MC 复用)、覆盖度/置信度图、原始/滤波双视图、last-touch 归因(五轴必需,§24.5.5);
- 对照测试:与 MeshLab/CloudCompare 结果对数值(仅测试对照,不链接其 GPL/LGPL 代码)。

### 9.8 精度边界与盲区分析(为什么"精确"必须组合三层)

**核心结论:采样式方法有系统性盲区,栅格式方法无盲区但有量化误差;验证 = 两者组合 + 可信度标注。**

| 方法 | 能精确反映 | 盲区(会漏报) | 误差量级 |
|---|---|---|---|
| Z-map 逐列(L0) | 顶面缓变区域的深度差 | 垂直壁/倒扣/侧刃加工面(一列一高度无法表示);被遮蔽面(取"最高面"语义错);窄于格宽的沟槽(整体漏掉);刀痕混叠 | 格宽/2 + 盲区=完全失明 |
| 采样点-网格(L2) | 采样点处的精确距离(原语本身精确) | **两个采样点之间的一切**——窄于采样间距的过切槽可整条从样本间漏过;Hausdorff 的 max 系统性低估(采样 max ≤ 真实 max),**对"存在性"无保证** | 采样密度决定 |
| 逐单元差分场(L1) | 差异区域本身(存在性完备:单元内符号必翻转) | 无漏报;边界量化到单元尺寸 | 单元/2,可细化到亚单元 |

**五层系统性误差源(对比精度 = 精度链最差一环)**:

1. **上游仿真保真度**:对比有效下限 = 仿真格宽(更小的差异是量化噪声,不是加工误差,必须显式声明);
2. **模型细分容差**:STEP→网格弦高容差(如 0.01mm)直接进入"设计面"位置;
3. **对应关系歧义**:陡峭面上欧氏最近点 vs 法向投影差 1/cosθ 倍;尖锐凸边处最近点跳到另一张面(映射不连续)→ 色图沿边缘出现条带伪影;
4. **符号判定歧义**:薄壁两侧、非水密网格缺陷附近——环绕数给分数值;正确做法是把符号不确定带标"存疑",不硬分类;
5. **刀痕 vs 形位误差语义混淆**:球头精加工的 cusp 梳状纹理是工艺固有纹理,逐点对比会显示满屏周期性"偏差"——必须提供**原始/滤波双视图**(原始=刀痕可见,低通/包络=形位误差),否则色图无工程意义。

**正确架构**:①存在性(栅格差分,找全)→ ②定界与定量(局部细化 + 采样)→ ③可信度(覆盖度/置信度图 + 有效下限声明)。**任何只做采样的"验证"输出都会给人"查过=没问题"的错误安全感。**五轴版完整方案见 §24.5。

### 9.9 形位公差评定(GD&T,2026-11-07 补查)——把"偏差色图"升级为符合标准的评定

**标准框架**:
- **ISO 1101:2017**(几何公差核心;面轮廓度归其轮廓度条款;https://www.iso.org/standard/66777.html );**ASME Y14.5-2018**(https://www.asme.org/codes-standards/find-codes-standards/y14-5-dimensioning-tolerancing ,数学定义在 **ASME Y14.5.1-2019**);面轮廓度 = 理论正确轮廓沿法向 ±t/2 双向偏置带(ISO 的 UZ / ASME 的 Ⓤ 修饰符支持不等分布;两体系差异对比:Humienny 2021, https://www.mdpi.com/2076-3417/11/17/8269 );
- **ISO 5459:2016**(基准与基准体系建立方法学,https://www.iso.org/standard/59391.html );ISO 14638(GPS 矩阵,https://www.iso.org/standard/62965.html );**ISO 15530-3**(CMM 不确定度的校准件替代法,https://www.iso.org/standard/53627.html ;NPL 实用指南 https://eprintspublications.npl.co.uk/5924/1/MGPG130.pdf );
- 工程定义参照(gdandtbasics):https://www.gdandtbasics.com/profile-of-a-surface 。

**最小区域拟合(L∞ / Chebyshev,判据级算法)**:
- 数学定义:求参数使 `max_i |d_i|` 最小(最窄包容带)——FARO 明确"最小区域法最接近 ASME Y14.5"(https://www.faro.com/en/Resource-Library/Article/how-to-evaluate-flatness-in-gd-t );
- **LP 表述(平面类,可直接实现)**:`min t, s.t. −t ≤ z_i − (a·x_i + b·y_i + c) ≤ t`(4 变量、2n 约束的线性规划);
- **一般名义面 → 序列线性规划(SLP)**:Carr & Ferreira 1995, "Verification of form tolerances part I: Basic issues, flatness, and straightness", *Precision Engineering* 17(2):131–143(https://www.sciencedirect.com/science/article/pii/014163599400017T ;Part II 圆柱度 …/014163599400018U )——线性化最小区域,单调收敛全局最优(全文 403,细节以摘要为准);
- **凸包表述(平面度)**:点集凸包的最窄"平板宽度",由上 2 点 + 下 2 点支撑决定,O(n log n) 建凸包后扫描 (面,点)/(边,边) 对;
- **Remez/Lawson 交换算法**:LP 最优解只由活跃点决定 → 小活跃集解等式组 + 极值点交换迭代(Remez:https://en.wikipedia.org/wiki/Remez_algorithm ;Lawson IRLS→minimax 综述:https://epubs.siam.org/doi/10.1137/S0036142993254971 )。

**最小二乘(L2)与工程取舍**:L2 快且数值稳定,但评定值**偏保守(更大)**——PC-DMIS 文档原文 LSQ 轮廓度 "yields a larger, more conservative measured value",DEFAULT(min-max/最小区域)数据稠密时最接近真实(https://docs.hexagonmi.com/pcdmis/2020.2/en/helpcenter/mergedProjects/core/geometric_tolerances/Profile_of_a_Surface.htm )。取舍:**L2 用于在线反馈与基准拟合,最小区域用于最终判定**;两者差值异常大 → 提示对齐/数据质量问题。

**基准参考系(DRF)对齐——带基准的轮廓度不允许自由 ICP**:
- 3-2-1 层级:主基准 3 点约束 3 自由度 → 次 2 → 第三 1,共 6;
- **受限最小二乘(材料侧约束 LS)是最佳折中**(数值上比受约束 min-max 稳定):NIST Shakarji 受限全最小二乘(https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=921834 );Shakarji & Srinivasan 2023, "Toward a New Mathematical Definition of Datums in Standards…", ASME JCISE 23(2)(https://pubmed.ncbi.nlm.nih.gov/39380595 );
- 可实现步骤:①基准 A 平面加"材料同侧"约束的 LS(锁 3 自由度)→ ②基准 B 在垂直子空间内同法 → ③基准 C 再锁 1 → ④构造 DRF 刚体变换 T,被测特征评定在固定系下进行 → ⑤仅当公差无基准引用时才做自由最小区域优化。

**输出量定义**:max 正偏差、max 负偏差、**PV = max − min**(带中心无关)、RMS、逐点有符号偏差(=色图数据)、合格判定、拟合方式与基准系——与商业报告字段一致(PC-DMIS/ZEISS/PolyWorks)。

**开源现状(空白确认)**:无成熟开源"最小区域 GD&T 评定"库;可组合部件:geomfitty(MIT,仅 LSQ 图元拟合,https://github.com/mark-boer/geomfitty )、Qhull/凸包法底座、STEPNCpp(STEP-NC 算子,https://github.com/johnmichaloski/STEPNCpp );商业对标:PC-DMIS(DEFAULT/LSQ 双选项)、ZEISS INSPECT(完整 ISO 1101 需 Professional 模块)、PolyWorks(原生 ISO 1101 / ASME 双标称切换,https://www.polyworks.com/en-us/products/polyworks-inspector )。

**极值统计置信度**:采样 max 系统性低估真实 max;Fisher–Tippett 定理 → GEV(**Gumbel**)拟合分块极值外推期望真实最大值 + (1−α) 置信上界(https://en.wikipedia.org/wiki/Gumbel_distribution ;表面计量同构应用 Pawlus 2023:https://pmc.ncbi.nlm.nih.gov/articles/PMC10672465/ );块数不足时标"极值欠采样"。报告并报 HD95。

**实现分级**:L1 最小二乘 + PV(2–3 周,复用现有距离场)→ L2 最小区域(LP/凸包,自校验:LS PV ≥ MZ PV)→ L3 基准系完整(受约束 LS + UZ/Ⓤ + ISO/ASME 双标称 + EVT + ISO 15530-3 字段)——到 L3 才可宣称"符合标准的轮廓度评定"。**风险:标准正文需购买**(ISO 1101/5459、ASME Y14.5/.1)。

### 9.10 刀痕/形位误差分离滤波(ISO 25178 / 16610 体系,2026-11-07 补查)——L3"双视图"的可实现规格

**标准体系**:
- **ISO 25178-2:2021**(面轮廓参数 Sq/Sa/Sz…,https://www.iso.org/standard/74591.html );**ISO 16610 滤波器工具箱**(21/61 高斯、22/62 样条、31/32/71 鲁棒、41/81 形态学、29/69 样条小波);**注意:ISO 4287 已撤销,由 ISO 21920-2 取代**(引用时写 21920-2 或 25178-2);
- S-filter(短波低通:去噪)/ L-filter(长波高通:去形位)/ nesting index(线性滤波=截止波长,形态学=结构元素尺寸)概念:Digital Surf 滤波指南(实抓,https://guide.digitalsurf.com/en/guide-filtration-techniques.html 、https://www.digitalsurf.com/blog/the-delicate-art-of-applying-filters-to-a-surface-in-order-to-prepare-it-for-analysis )。

**λc(截止)选择规则**:标准序列 0.08/0.25/0.8/2.5 mm(面轮廓另有序列);高斯在 λc 处 50% 传输(渐进非硬截止);**截止必须落在刀痕波长族与形位波长族的谱谷之间,不能取中间**。对刀痕:**球头 cusp 主导波长 ≈ stepover**(cusp 公式复用 §5.1)→ 形位视图取 **λc ≈ 4–5×stepover 并归入标准序列**;刀痕视图取 **λc < stepover**(检行距异常/漏切);采样距 < stepover/2(Nyquist)。佐证:Quinsat et al. 2011, "Characterization of 3D surface topography in 5-axis milling", *CIRP Annals* 60(1)(https://www.sciencedirect.com/science/article/abs/pii/S0007850611000647 );Raja, Muralikrishnan & Fu 2002, "Recent advances in separation of roughness, waviness and form", *Precision Engineering* 26(2)(https://www.sciencedirect.com/science/article/abs/pii/S0141635902001034 )。

**算法要点(逐个)**:
1. **高斯滤波**(16610-21/61):卷积/FFT 实现;边界端效应按 16610-28 截短;弱点:对离群/台阶敏感 → 鲁棒版;
2. **鲁棒高斯/鲁棒样条**(16610-31/32/71):迭代重加权抑制深谷离群;样条张力 β=0.6252 时行为≈高斯;指南结论:两者结果几乎相同,选更简单的鲁棒高斯;
3. **F 操作(形状移除)**:**直接用设计 CAD 面作形状基准,零拟合**——与 ISO F-operator 语义一致(这是 CAM 软件的天然优势);
4. **形态学滤波**(16610-41/81):**Closing = 先膨胀后侵蚀 = 上包络**(球沿表面滚过的轨迹),Opening = 下包络——"表面包络 vs 设计面"比较的数学化;级联多尺度(-49/-89)给出连续尺度带;
5. **小波**(16610-29/69,样条小波):尺度谱诊断视图;Zahouani 2008 制造 signature 分解(https://www.sciencedirect.com/science/article/abs/pii/S0043164807001962 )。

**L3 三视图规格(落地)**:①形位视图(S 滤波 + CAD 面 F 基准 + λc≈4–5×stepover 的 L 滤波);②刀痕视图(λc<stepover 或带通,检行距异常);③包络视图(形态学 closing,半径 ≥ 刀尖有效半径)。

### 9.11 表面质量评价(2026-11-07 补查判定:做 A/B 两层,C 层列为非目标)

**定位与判定**:表面质量评价在本产品中有两个入口——**加工前预测**(参数面板实时显示理论 Sa,与 §5.8 规则引擎联动:目标 Sa → 反解 stepover)与**加工后评定**(§9.10 三视图 + ISO 25178 参数,进验证报告)。**C 层(颤振纹/烧伤/毛刺检测、ML 粗糙度预测、功能性评价)列为非目标**:需实测信号与工艺数据,属工艺监控/计量软件领地(见 §14)。

#### 9.11.1 A 层:ISO 25178 面参数计算器(约 1 周,纯定义式数学)

- 作用对象:§9.10 三视图(形位/波纹/粗糙度)+ 仿真高度场 + 实测点云栅格化表面;**每个参数必须标注所属视图**(nesting 语义,否则数值无意义);
- 核心参数(ISO 25178-2,https://www.iso.org/standard/74591.html ):
  - 高度类:`Sq`(RMS 高度)、`Ssk`(偏斜)、`Sku`(峰度)、`Sp/Sv/Sz`(最大峰/谷/高低差,Sz = |Sp|+|Sv|);
  - 空间类:`Sdq`(根均方坡度 = 梯度积分)、`Sdr`(界面面积比 = (1 − A_展开/A_投影)×100%);
  - 功能类(二期):Sk 家族(Sk/Spk/Svk/Smr1/Smr2,Abbott–Firestone 曲线);
- 算法要点:全部定义式积分/统计——`Sdq = √(Σ((∂z/∂x)²+(∂z/∂y)²)·dA)/A`(梯度用高度场中心差分,**复用渲染法线计算**,§8.2);`Sdr` 由梯度积分出展开面积;每个参数几十行,无鲁棒性难点;
- 输出:验证报告"表面纹理"页:每视图一组参数 + Abbott 曲线 + (可选)参数空间分布图。

#### 9.11.2 B 层:加工前理论预测(约 2–4 周,差异化亮点)

- **解析模型(主路线)**:
  - 三轴球头/圆鼻:cusp 理论 `Sz = h = R − √(R² − s²/4)`(§5.1);**Sa 理论值 = cusp 剖面(圆弧)在步距内平方偏差的积分**(数值积分 ~10 行;球头平面件可闭式);
  - 进给方向纹理高度 ≈ 每齿进给量级(fz),通常远小于 cusp → v1 忽略并标注;
  - 五轴:**有效切削半径 R_eff**(球头不变;圆鼻/平底受 lead/lean 影响大)→ 公式挂 §23 运动学输出;
  - 曲面修正:κrel(§5.10 iso-scallop 公式反用);
  - **实时联动**:stepover/层深滑块拖动 → 预测 Sa 实时更新;**目标 Sa 模式**:用户给目标 Sa → 反解 stepover 并 clamp 到 §5.8 经验带;
- **局部小块仿真(可选增强)**:仅对 ~1cm² 代表区用 **1–5µm 格** Z-map/针式模型跑地形后算 Sa(文献主流 = 离散地形仿真叠加刀刃轨迹;进阶修正:让刀/跳动,二期);
- **关键分辨率边界(必须随结果声明)**:**绝不能用主仿真网格算 Sa**——主仿真格宽 0.02–0.1mm,精加工纹理是 µm 级,量化噪声完全淹没纹理;故主路径 = 解析模型(+可选局部高分辨小块);
- **诚实边界**:v1 = 几何理论值,不含机床动态(让刀/跳动/振动是文献加修正项的原因);仅对球头/圆鼻精加工有效;平底刀平面无 cusp(理论 Sa→0,实际由 fz 纹理主导);
- 文献锚点:Ni 2024 几何仿真系统(https://link.springer.com/article/10.1186/s10033-024-01033-5 );Wang 2025 让刀/跳动修正(https://www.sciencedirect.com/science/article/abs/pii/S0263224124016555 );五轴地形预测(Guo 2026, https://www.sciepublish.com/article/pii/1045 );Quinsat 2011(§9.10 已引)。

#### 9.11.3 C 层:非目标声明

颤振纹检测(频域/PSD 诊断)、烧伤/毛刺判定、ML 粗糙度预测(§5.9 已判断价值有限)、功能性评价(密封/摩擦)——需要实测信号与工艺数据,属工艺监控/计量软件领域;若未来接入实测数据,优先做 §9.6 点云管线 + 本节 A 层参数,而非缺陷检测。

### 9.12 系统误差校正(2026-11-07 补查):设计场零点偏置 + 让刀

#### 9.12.1 设计场零点偏置(体素化系统误差,含成因清单与校正规格)

**现象**:设计网格体素化成 SDF 后,零等值面系统性偏移(CNCSIM 实测 +0.030mm 外偏,49318 点;对应 h/2 级一阶偏置)→ "切到设计面"显示为薄层过切,污染相似度统计。

**成因清单(按嫌疑度,文献基础:Bærentzen & Aanæs 采样误差分析,https://www2.imm.dtu.dk/pubdb/edoc/imm1289.pdf )**:
1. **符号边界"半格"约定错误(一阶 ±h/2,头号嫌疑)**:零点在内外格心中点=无偏;在"最后内侧格心"→固体收缩 h/2;在"首外侧格心"→扩张 h/2;
2. 占据判定口径(格心在内 / 格全在内=腐蚀 / 格有相交=膨胀);
3. 采样位置(格心)与解码位置(格顶点)错位固定 h/2——OpenVDB 的 SDF 记于格心(https://www.openvdb.org/documentation/doxygen/MeshToVolume_8h.html );
4. 符号翻转线性插值的曲率偏差(二阶 h²/8R,亚微米,解释不了 30µm);
5. **非对称窄带截断**(对称截断无偏置但放大 σ;单侧截断一阶移零点);
6. 符号判定局部误差(法向法薄壁/锐边漏;raycast 仅水密可靠,https://docs.rs/mesh_to_sdf );
7. 重距离化种子取"首个内部格"自带 ±h/2(Eikonal 不纠正种子)。

**数值实验标度(判别试验,自建脚本验证)**:解析球 R=2mm 表面点读数,变格宽——精确距离@格心+插值解码:bias ∝ h²(h=0.1 时 +0.83µm,无害);EDT 零点在最后内侧格心:bias ∝ h ≈ h/2(h=0.06 时 +26µm,**与实测吻合**);与 h 无关 → 单位/坐标错位。**诊断:换 2~3 个格宽重测 bias,h 标度直接定位是哪类。**

**标定-补偿流程规格**:
```
阶段0 诊断: 3 个格宽 × 解析基准体(球+盒+台阶) → bias(h) 曲线定标度
阶段1 标定: 与生产完全相同的体素化管线跑解析形状 → bias_table{h→(offset,spread)}
            assert(spread < 0.15·|offset|)  # σ 过大 = 约定 bug,报错而非补偿
阶段2 补偿(二选一,推荐 a): (a) 查询侧 d' = d_raw − bias(h)(不动场,可与每段不同格宽共存)
                          (b) 场侧全场减 bias(等价 iso=+bias)
阶段3 回归: |bias| < 0.05·h 且 < 2µm;σ < 0.1·h(球/盒/锐边分别统计)
```
**注意:刀具侧体素化(扫掠 SDF)同样有零点偏置,同一套标定。**库佐证:OpenVDB MeshToVolume = 扫描转换最近点+符号泛洪+重归一化(半带宽 3 voxel;FastSweeping 重距离化推荐);mesh_to_sdf(Rust)= 精确点到三角形距离@格心(RtreeBvh 加速)→ A 类管线零点偏置二阶;MC 边中点放置的半格偏置 → 用线性插值 `t=(iso−v0)/(v1−v0)` 放置顶点(Rajon & Wachsmann 2003:https://pubmed.ncbi.nlm.nih.gov/12821034 )。

#### 9.12.2 静态让刀(轻量版:先警示,后补偿)

**公式链**:Kienzle 比切削力 `kc = kc1.1·h^(−mc)` → 每齿切向力 `Ft = kc·ap·h`,径向比 Fr/Ft ≈ 0.3–0.6(https://www.machiningdoctor.com/glossary/specific-cutting-force-kc-kc1 、https://www.machiningdoctor.com/classroom/endmiils-deflection/ )→ 悬臂梁挠曲 `δ = 64·F·L³/(3·E·π·d⁴)`(**按容屑槽底径 deq 算,不是切削径**——12mm 刀 10mm 核心径比按 12mm 算多 ~85%;δ∝L³、∝1/d⁴;硬质合金 E≈600–650GPa,HSS ≈1/3 挠曲;Harvey Performance 同口径:https://www.harveyperformance.com/in-the-loupe/tool-deflection-remedies/ )。

**量级表(典型工况)**:悬伸 L/D = 2 → ~1µm;4 → ~3µm;6 → ~10µm;8 → ~25µm;10 → ~45µm。**判定:正常精加工(L/D≤6)让刀 1–10µm,远小于仿真格宽 0.02–0.1mm,体素口径不可见 → 对相似度统计无影响**;且方向是"弹开欠切",与零点偏置方向相反,不会被误判为过切。

**v1/v2 决策**:**v1 不做补偿,只做刚性警示**——刀具表加 L/deq/E,输出 δ_est,超公差 1/3 时 UI 警告(建议缩短悬伸/降 fz);纯计算函数零风险。**v2(触发:L/D≥8 或 δ_est>公差)**:精加工刀路法向反向补偿(刀位点平移,不改仿真内核)+ 把 δ_est 写入验证器"预期欠切壳层"先验。文献:微铣削力预测补偿有效(Dow 2004, https://www.sciencedirect.com/science/article/abs/pii/S0141635903000722 );五轴让刀为尺寸误差主因之一(MERL TR2013-057:https://www.merl.com/publications/docs/TR2013-057.pdf );**商业 CAM 公开功能均无刀路级让刀补偿 → 自研做了是差异化,但优先级低**。

**范围外(明示)**:薄壁件工件变形(需 FEM/迭代,量级 0.05–0.1mm)、机床热变形(占精密加工误差 40–70%,Ramesh 2003:https://www.sciencedirect.com/science/article/abs/pii/S0890695502002638 )、反向间隙——验证报告统一注明"仿真为标称几何,不含热漂移/间隙/工件变形"。另:**刀具侧与设计侧体素化的弦差/量化误差都在 §9.8 精度链里,ASME 体素铣削误差分析(https://asmedigitalcollection.asme.org/manufacturingscience/article/145/1/011005/1146709 )可作方法论参照。**

---

## 10. UI 与渲染(基础设施选型)

### 10.1 选型

- **egui/eframe**(immediate-mode,MIT OR Apache-2.0,https://github.com/emilk/egui ):0.34 起 eframe 默认后端为 **egui-wgpu**(2026-08 调查文章佐证 wgpu 成为 Rust GUI 事实标准;https://crates.io/crates/egui-wgpu ,2026-10 仍在发版)→ 面板 UI 与 3D 视口同一 GPU 上下文,集成成本低;
- 备选:bevy(功能全但过重、版本迭代快)、iced/slint(3D 视口集成更费劲);
- 窗口:winit;文件对话框:rfd;GPU API:wgpu(WebGPU 标准,Windows/macOS/Linux 通吃)。

### 10.2 自研渲染器任务清单(均为常规图形编程,非算法库范畴)

1. 轨道相机(球坐标 + 惯性);2. 网格渲染(位置/法线顶点流,简单 Lambert + 边缘线框可选);3. **刀路线渲染**:百万级 `line list`(GPU 顶点缓冲,粗/快移双色,LOD 按缩放抽稀);4. **高度场动态网格**:Z-map → 索引网格,切削增量更新顶点高程与法线(中心差分)后部分上传;5. SDF/voxel → MC 网格静态上传;6. 偏差色图 = 顶点色;7. 学习资料:[learn-wgpu](https://github.com/sotrh/learn-wgpu)(教程)、[awesome-wgpu](https://github.com/rofrol/awesome-wgpu)(资源索引)、[voxel-rs](https://github.com/florianhartung/voxel-rs)(体素渲染示例)。

### 10.3 数学库取舍

自研 `math` crate(约 500 行:f64 主型 + f32 渲染型 + AABB/Ray/Plane)—— 贯彻自研原则、可控精度(f64 几何 + f32 渲染分离);glam(SIMD 快,Bevy 系,https://github.com/bitshifter/mathbench-rs 基准)与 nalgebra(泛型维数、机器人系)仅作为**测试对拍参照**保留,不进产品依赖(对比讨论:https://www.reddit.com/r/rust_gamedev/comments/nfhu18/nalgebraglm_vs_glam )。

### 10.4 基础设施 crate 清单(2026-11 状态,以 crates.io 为准)

| crate | 用途 | 许可证(以仓库 LICENSE 为准) |
|---|---|---|
| [winit](https://github.com/rust-windowing/winit) | 窗口/输入 | Apache-2.0 OR MIT |
| [wgpu](https://github.com/gfx-rs/wgpu) | GPU 渲染 | MIT OR Apache-2.0 |
| [egui/eframe/egui-wgpu](https://github.com/emilk/egui) | UI | MIT OR Apache-2.0 |
| [rayon](https://github.com/rayon-rs/rayon) | 数据并行 | MIT OR Apache-2.0 |
| [memmap2](https://github.com/RazrFalcon/memmap2-rs) | 大文件映射 | MIT OR Apache-2.0 |
| [bytemuck](https://github.com/Lokathor/bytemuck) | POD 零拷贝转型 | MIT OR Apache-2.0 / Zlib |
| serde + [ron](https://github.com/ron-rs/ron) | 配置/工程文件 | MIT OR Apache-2.0 |
| [thiserror](https://github.com/dtolnay/thiserror) | 错误派生 | MIT OR Apache-2.0 |
| [tracing](https://github.com/tokio-rs/tracing) | 日志 | MIT |
| [rfd](https://github.com/PolyMeilex/rfd) | 文件对话框 | MIT |
| [image](https://github.com/image-rs/image) | 截图导出 | MIT OR Apache-2.0 |
| criterion / proptest | 基准 / 性质测试(dev) | MIT OR Apache-2.0 |

---

## 11. 测试与工程化

1. **黄金文件**:病态 STL 语料(自交/非流形/负属性字节/ASCII 冒充二进制)→ 解析器回归;NIST STEP(AP203+AP242)→ STEP 回归(§3.6);Kiri:Moto/LinuxCNC 文档示例 → G-code 解析回归;
2. **规划器对拍**:同 $ 参数下与 grbl-sim 输出(block.out/step.out)比时序曲线(§7.7);
3. **性质测试**(proptest):偏置结果与原区域拓扑一致性、BVH 最近点 == 暴力最近点、SDF 距离满足 |S(p)| ≈ dist(表面附近)、循环展开序列符合 §7.5 语义;
4. **基准**(criterion):STL 解析 MB/s、drop-cutter 点/s、Z-map 更新 Mcell/s、Vatti 布尔 ops/s、G-code 解析行/s;
5. **渲染回归**:视口截图(image crate)与基线像素对比(容差模式);
6. **CI/打包**:GitHub Actions 三平台矩阵 + [cargo-dist](https://github.com/axodotdev/cargo-dist) 一键安装包(细节以官方文档为准)。

---

## 12. 开源项目参考对照表

| 项目 | 语言/许可 | 定位 | 我们借鉴什么 |
|---|---|---|---|
| [opencamlib](https://github.com/aewallin/opencamlib)(LGPL-2.1,2018 起) | C++/Python | CAM 算法库 | drop-cutter/push-cutter/waterline/weave/tsp 算法结构;[References 页](https://github.com/aewallin/opencamlib/wiki/References) 是论文索引;文档 https://opencamlib.readthedocs.io/ |
| [CAMotics](https://github.com/CauldronDevelopmentLLC/CAMotics)(GPL-2.0+) | C++ | 三轴 G-code 仿真器 | **场+移动索引+扫掠深度+MC33 架构**(§8.3);LinePlanner/SCurve 时间模型 |
| [Kiri:Moto / grid-apps](https://github.com/GridSpace/grid-apps)(MIT) | JS | 浏览器 CAM | `op-*.js` 操作管线、clear=等距偏置、tip2tip、cutTabs(源码已核验);文档 https://docs.grid.space/kiri-moto/CAM/ops |
| [PyCAM](https://github.com/SebKuzminsky/pycam)(GPLv3) | Python | 老 CAM | DropCutter/PushCutter × 平行/等高策略矩阵(仿真已移除) |
| [FreeCAD CAM](https://wiki.freecad.org/CAM_Workbench) | Python+OCCT | 全功能 | 操作对象模型、OCL/openvoronoi 接法 |
| [grbl](https://github.com/gnea/grbl)(GPLv3) | C | 控制器 | gcode.c 模态组校验、planner.c/motion_control.c 公式(§7)、wiki 文档 |
| [LinuxCNC](https://github.com/LinuxCNC/linuxcnc)(GPLv2+) | C/C++ | 控制器 | rs274ngc 解释器分层、time_estimate.cc、gcode 文档(规范母本) |
| [FlatCAM](https://github.com/Denvi/FlatCAM)(MIT) | Python | PCB CAM | camlib.py 的偏置/清除/连接优化,轻量可读 |
| [CuraEngine](https://github.com/Ultimaker/CuraEngine)(**AGPL-3.0,严禁抄码**) | C++ | 3D 打印切片 | 切片→墙→填充→排序流水线组织(粗加工同构) |
| [gsim-rs](https://github.com/navrajkalsi/gsim-rs)(MIT) | Rust | G-code 体素仿真 | 同生态先例:wgpu 体素去除、Fanuc 方言解析 |
| [Clipper2](https://github.com/AngusJohnson/Clipper2)(BSL-1.0,**上游已归档 2026-10**) | C++/C# | 多边形布尔/偏置 | Vatti 实现细节 + 文档;Rust 封装 clipper2 仅作对照 |
| [Manifold](https://github.com/elalish/manifold)(Apache-2.0) | C++ | 鲁棒网格布尔 | 二期若自研网格布尔的算法思想来源 |
| [CloudCompare / CCCoreLib](https://github.com/CloudCompare/CCCoreLib)(GPL / LGPL-2.1) | C++ | 点云/网格对比 | C2M 工作流与参数(max-dist、flip normals、采样) |
| [MeshLab/VCGlib](https://github.com/cnr-isti-vclab/meshlab)(GPL 系) | C++ | 网格处理 | 清洗过滤器顺序、Hausdorff 过滤器、closest_point.h |
| [PBRT](https://pbr-book.org/3ed-2018/) | — | 渲染教材 | BVH/SAH 规格书(免费全文) |

**清洁室纪律**:以上 GPL/AGPL 项目一律**只读思想不抄代码**;可放心参照算法描述的:论文(思想不受版权保护)、BSD/MIT/Apache/BSL 项目(仍建议重写而非复制)、CC-BY 4.0 文档(如 Eberly,GTOC 许可,署名即可)。

---

## 13. 路线图

| 里程碑 | 内容 | 验收标准 |
|---|---|---|
| **M0 骨架**(4–6 周) | workspace + math + STL 解析(含修复)+ egui/wgpu 视口 + 轨道相机 | 100MB STL < 3s 加载;病态语料全部报错/修复 |
| **M1 2D CAM**(6–8 周) | Vatti 布尔/偏置 + Profile/Pocket/Drill/Level + G-code 生成器(grbl 方言)+ Z-map 仿真(MVP 刀底闭式)+ 校验器 | 样例件全流程出码;Z-map 渲染可见刀具纹理;限位/G0 校验零误报 |
| **M2 精加工 + 时序**(6–8 周) | drop-cutter 平行精加工 + 简化 waterline(切片+偏置)+ grbl 式规划器仿真(L2)+ 时间轴回放 | 时间估算与 grbl-sim 对拍误差 < 10%;球头精加工残留高度符合公式 |
| **M3 验证 + STEP 网格档**(4–6 周) | 差异对比 L0+L1(色图/统计/体积对账)+ ISO 10303-21 解析器 + TESSELLATED/FACETED → Mesh | NIST 网格档文件全通过;偏差色图与 CloudCompare 数值对拍 |
| **M4 深水区**(按需) | SDF 仿真 v2(+MC33)+ 解析曲面档(S3)+ 双向 Hausdorff(L2)+ 摆线/自适应清角 + 多方言 post(Fanuc) | 陡壁案例侧壁精度达标;自适应刀路切削角稳定 |

> **补充排期**(基于第 15~22 章):DXF 导入并入 M1;特征识别 L1(选面高亮)并入 M2、L2(规则自动)并入 M4;发送器/探测/调平作为独立里程碑 M5(对接实机时启动);NURBS S3/S4 档与 Hausdorff L2 维持按需。
>
> **五轴排期**(2026-11-07 需求升级,第 23~24 章):
> - **M6-A(3+2 定位)**:kinematics crate(BC/AC IK-FK)+ 倾斜帧刀路复用 + 方位自动划分 + 每方位 Z-map→tri-dexel 合并 + 柄/夹具静态碰撞 + G68.2/CYCLE800/PLANE 后处理(双模式输出)。依赖既有三轴引擎,建议 M2 后即启动。
> - **M6-B(五轴连续)**:固定刀轴/法向刀轴策略 + slerp/奇异规避 + Ilushin/SDF 碰撞 + tri-dexel 五轴去除 + 关节空间采样扫掠 + 模型差异:设计 SDF + 逐单元差分场 + last-touch 归因(§24.5)。
> - **M6-C(高级)**:swarf/flowline、完整机床碰撞(轴树/全对枚举/超程/快移入料)、linking 规划、CWE + 验证:公差带等值面/二分细化 + 实测点云接入 ICP(§24.5)+ GD&T 评定(L1 最小二乘→L2 最小区域→L3 基准系,§9.9)+ 滤波三视图(§9.10)+ 窄带/层级差分场性能工程(§24.5.8)。
>
> - **自动生成流水线(§5.7~5.10,2026-11-07)**:自动参数规则引擎(§5.8)并入 M1-M2;iso-scallop 行距调度(P1)与 steep&shallow 拼接(P3)并入 M2 精加工;自动 pencil(P2,依赖仿真残留)并入 M3;一键 CAM 流水线(§5.7)与 CAPP 规则引擎(§5.9)并入 M4(3+2 自动方位直接复用 TAD 聚类);STEP-NC 导出预留;表面质量:解析预测联动(§9.11 B)并入 M2,ISO 25178 参数报告页(§9.11 A)并入 M4。
>
> - **刀路质量补强(§5.11,2026-11-07)**:进给调制(恒 MRR 起步,v1 不做力模型/SLD)+ 两级安全链接(v1 绕障只报不绕)+ 方向投票(v1 全局,v2 PFD 分区)并入 M2;可侧铣度检测器(§5.11.4,不生成刀路)并入 M6-C(五轴侧刃前置);**零点偏置标定-补偿(§9.12.1)并入 M3(验证档交付前必做,先诊断 h 标度再标定)**;让刀 v1 只做刚性警示(§9.12.2)。
>
> - **刀路质量补强 II(§5.12/§5.13/§21.4,2026-11-07)**:光顺过渡(拐角圆角公式 + 短段/共线规则,Fusion 双容差模型)并入 M2(P1,~2 人周,Xiao 26% 时间量级);薄壁规则表(纯规则)并入 M4 工艺知识库(P3,~1 人周);桶刀 3+2 原型(sd_profile 环面片段 + 有效半径行距)并入 M6-C(P2,~4–6 人周,先出"同残留下行距倍数"验证页);5 轴切触分配/样条插补器/桶刀力模型不做。
>
> - **闭环编排协议(§5.15,2026-11-07 设计)**:M3 起实现 MVP(dry-run 迭代 + S0 修正自动应用 + 对账不变量 + 内存台账);跨装夹传递/依赖级联/S2 净室重跑/台账持久化随 M6-A;多目标搜索不做。

---

## 14. 风险清单

1. **STEP 精确 B-rep / NURBS(S4)是最大深水区** → 对策:四档分阶段(§3.4),S2 网格档先交付,S4 无限期后置;必要时评估 LGPL 的 OCCT 兜底(需法务确认静态链接例外条款)。
2. **自研 Vatti 的鲁棒性**(Sliver/自交/退化)→ 对策:64 位定标整数网格化(Clipper2 路线)+ 大规模性质测试 + 失败时降级告警。
3. **仿真精度-性能权衡**(0.02mm 格的更新量)→ 对策:分辨率分档 + 条带并行 + 必要时 GPU compute(二期)。
4. **GPL 清洁室纪律**(grbl/CAMotics/CuraEngine 源码)→ 对策:只读文档/论文/伪代码;接触过 GPL 源码的实现者写设计文档而非代码评审互审;grbl-sim 仅作外部对照工具。
5. **时间估算保真度** → 对拍 grbl-sim 数据(§7.7),否则循环时间可信度低(issue #893 实证 5~10 倍偏差)。
6. **上游生态漂移**(egui/wgpu 破坏性更新频繁)→ 锁版本 + 渲染层薄封装。
7. **范围蔓延**(切削力、STEP-NC 控制器生态、表面缺陷检测/颤振诊断与 ML 粗糙度预测——§9.11 C 层)→ 仍列为非目标,单独立项;**五轴已于 2026-11-07 升级为正式目标**(第 23~24 章),新增风险:运动学标定依赖机床实测参数(枢轴/行程/符号)、奇异区刀轴抖动、任意姿态碰撞检测性能、tri-dexel 内存与更新吞吐需按 §22 口径重算、grbl 系控制器不可用(发送目标收窄为 LinuxCNC/工业系)。

---

## 15. 特征自动识别(自研路线)

### 15.1 方法谱系与取舍

| 路线 | 代表文献/实现 | 对我们的结论 |
|---|---|---|
| B-rep 图法(AAG) | Joshi & Chang 1988, *CAD* 20(2):58–66(https://www.sciencedirect.com/science/article/pii/0010448588900504 );综述 Han, Pratt & Regli 2000, *IEEE TRA* 16(6)(https://ieeexplore.ieee.org/document/897789 );分类总览(Wikipedia,已核验):https://en.wikipedia.org/wiki/Feature_recognition | 依赖面-边拓扑,STL 主输入下不适用 |
| 启示法 | Vandenbrande & Requicha 1993, *IEEE TPAMI*, DOI 10.1109/34.250845(https://dl.acm.org/doi/10.1109/34.250845 ) | "最小特征边界 + 体积推理"思想可借鉴(处理特征相交) |
| 体积分解 | Sakurai & Chin 1993 等(见 Wikipedia 条目) | 理论参考 |
| 深度学习 | BRepGAT/BRepNet/AAGNet(仓库 https://github.com/whjdark/AAGNet ,MIT,含 MFCAD/MFCAD++ 基准);综述 https://arxiv.org/html/2402.17695v1 | 输入需 B-rep 拓扑图 + Python 训练栈 → **首期不做**;特征数据结构保留"类型+参数+置信度"接口 |

### 15.2 网格分割 / 基元拟合(STL 主战场)

**Schnabel 高效 RANSAC**(Schnabel, Wahl, Klein, "Efficient RANSAC for Point-Cloud Shape Detection", *CGF* 26(2):214–226, 2007, DOI 10.1111/j.1467-8659.2007.01016.x;PDF 镜像已核验:https://www.hinkali.com/Education/PointCloud.pdf )——步骤级规格:
1. 输入:点 + 法线(STL 直接用**面片重心 + 面法线**);
2. 基元最小集:平面 = 3 点;**圆柱 = 2 点+法线**(轴 = n1×n2,两条点-法线直线沿轴投影求交);球 = 2 点+法线;锥 = 3 点+法线;环面 = 4 点;
3. 检出概率与停止:所需候选数 `T = ln(1−p_t)/ln(1−P(n))`,`P(n)=C(n,k)/C(N,k)`;
4. **八叉树局部化采样**(首点全局随机,其余在包含它的胞格内抽取)——原文数据:200 万点中找 1066 点圆柱,均匀采样 1.51 亿候选 vs 局部化 64,929,**约 3 个数量级加速**;
5. 参数域 bitmask 打分(最大连通分量为得分,抗串扰)→ 正交最小二乘 refit → 提取 ≤3ε 兼容点 → 剩余点递归;
6. 参数:ε(距离)、α(法线角)、cluster_epsilon、min_points、probability(与 CGAL 实现一致:https://doc.cgal.org/latest/Shape_detection/index.html ;**CGAL 该包为 GPL-3.0——只读手册学参数语义,不链接**)。

**区域生长**(PCL 官方教程,全文核验:https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html ):按曲率升序取种子 → K 近邻中法线夹角 < 阈值并入 → 并入点若低曲率则入种子队列;参考默认:K=50、邻域 30、平滑阈 3°、曲率阈 1.0、最小簇 50。区域生长为通用算法,自研无许可问题。

**网格曲率**(Meyer, Desbrun, Schröder, Barr, "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds", VisMath 2002,PDF 已核验:https://multires.caltech.edu/pubs/diffGeoOps.pdf ):
- 平均曲率(cotan 拉普拉斯):`Δp_i = (1/2A) Σ (cot α_ij + cot β_ij)(p_j − p_i)`,`H = ‖Δp‖/2`;
- 高斯曲率(角亏):`K_i = (2π − Σθ_j)/A_Mixed`;
- 工程简化:先用每边二面角的有符号低通滤波版作离散曲率,仅在需区分柱/球/锥时上 cotan 公式。

### 15.3 孔检测管线(自研 6 步)

1. 圆柱面分割(Schnabel 圆柱基元,或"同轴圆柱区域生长");
2. 轴向确定性求解:柱面所有法线 ⊥ 轴 → **法线集合协方差矩阵最小特征向量为轴**;轴上点/半径最小二乘;共轴片段按 (c,a,r) 聚类合并;
3. 孔 vs 外圆柱:内孔法线朝轴(`dot(n_i, p_i−c) < 0` 为主);
4. 分类:通孔/盲孔(沿轴射线穿透测试);平底 vs 锥底(孔底法线 ⊥ 轴 / 与轴成恒定 ~59°,麻花钻 118°);锥孔(壁面法线-轴夹角恒定);沉头/阶梯孔(同轴多段 r 递减 + 台阶);倒角(孔口 45° 锥段);
5. 锐边环辅助:二面角 > 阈值的锐边图闭合环 → 环点圆拟合(RANSAC/Taubin)→ 孔口定位与圆柱面关联;
6. 输出:位置/轴向/直径/深度/类型 + 置信度。

文献:Qu & Stucker, "Circular hole recognition for STL-based toolpath generation", *Rapid Prototyping Journal* 11(3):132–139, 2005, DOI 10.1108/13552540510601255;Hase 等 2018(免费 PDF 已核验:https://www.cctech.co.in/downloads/publications/2018/complex-hole-recognition-from-cad-mesh-models-ICETSET-2018.pdf );工程综述:https://rapidmade.com/robust-hole-detection-in-stl-meshes 。

### 15.4 挖槽/台阶识别规则(自研 5 步)

1. 平面片段中挑**法向 +Z 且在材料内**的底面;
2. 底面边界邻接面中收集近竖直侧壁;
3. **开放性测试**:底面采样点向上射线到包围盒顶无交 → 开放口袋;有交 → 盲腔/内部流道(交面即"岛"的反向线索);
4. 分类:底面连通到模型外边界 → 台阶/开槽;外环 + 内环 → 带岛口袋;底面朝 −Z 且顶部开放 → 提示翻面装夹;
5. 参数输出:底面 Z(切深基准)、外轮廓/内岛多边形(**直接喂 §5 的 2D 偏置引擎**)、深度、最小转角半径。

备选:**切片法**(Z 平面截网格 → 嵌套多边形环 → 逐层差分自动给出口袋/岛/深度)——与 §5.5 粗加工同构,实现成本低。

### 15.5 工业与开源参照

| 对象 | 要点 | 链接 |
|---|---|---|
| CAMWorks AFR | "自动识别 20+ 种棱柱特征" + 交互式 IFR 兜底(页面已抓取) | https://camworks.com/why-camworks |
| Fusion 360 | Automated Hole Recognition(含部分圆柱面识别) | https://www.autodesk.com/products/fusion-360/blog/automated-hole-recognition-autodesk-fusion-manufacturing |
| OpenMill | **Rust** 5 轴 CAM(早期):自动孔检测(竖直圆柱壁)+ 口袋(开放水平底面)+ Pick Face 射线拾取共面洪泛 | https://github.com/charlatanshost/OpenMill (MIT/Apache) |
| AnvilCAD | Rust CAD+CAM 库:通孔/盲孔/带岛口袋识别 | https://github.com/swperb/anvilcad (MIT/Apache) |
| AAGNet | DL 参考实现 + MFCAD 基准入口 | https://github.com/whjdark/AAGNet (MIT) |

OpenMill/AnvilCAD 证明"网格 → 圆柱面找孔 + 水平底面找口袋"的轻量规则路线在 Rust 完全可行(只读思想)。

### 15.6 三级路线与落地顺序

- **L1 选面半自动**(2–4 人周):拾取三角形 → 洪泛扩展(法线角阈值 5–15° 可调 + 曲率相似)→ 高亮 → 一键生成操作;永远是兜底(对应 CAMWorks IFR 的定位)。
- **L2 规则自动**(2–3 人月):RANSAC + 区域生长双引擎互验 → 孔/口袋/大平面清单 → 特征→操作映射表(带置信度;低置信回退 L1 交用户确认)。
- **L3 深度学习**:不做(理由见 15.1)。
- **落地顺序**:顶点焊接 + 邻接表 + 锐边图 → 选面高亮 → 大平面自动面铣 → **孔管线自动钻孔(性价比最高)** → 口袋/台阶 → RANSAC 补强自由曲面。

---

## 16. 切削参数库、刀具库、刀柄与装夹

### 16.1 切削参数数据(材料 × 刀具 × 操作)

- **厂商公开源**:Sandvik CoroPlus Tool Guide(ISO 材料组 P/M/K/N/S/H → Vc/fz/n;https://www.sandvik.coromant.com/en-us/calculators-apps/machining-calculator-app );Kennametal 计算器(RPM = SFM×3.82/D;http://www.kennametal.com/pt/en/resources/engineering-calculators/miscellaneous/speed-and-feed.html );Guhring 图表(https://guhring.com/Support/Speeds-Feeds-Charts );Machinery's Handbook(版权出版物,无稳定公开 URL)。
- **商业计算器方法论**(可借鉴模型):G-Wizard 以"材料组 → 基准 SFM + 基准每齿进给"组织,再按切深/切宽、刀具几何、机床刚性修正(https://www.cnccookbook.com/feeds-speeds );FSWizard/HSMAdvisor 为商业闭源。
- **开源参照**:brturn/feeds-and-speeds(每材料存 Unit Power/SFM/IPT 与 DOC/WOC 区间,随机优化最大化 MRR,含切屑变薄 RCTF×ACTF 修正;http://brturn.github.io/feeds-and-speeds/instructions.html );tiktuk/cncfeedspeed(内嵌数据表);CNC-ToolHub、FreeCAD-FeedsAndSpeeds。
- **自研 schema 要点**:每值 `rec/min/max` 三元区间;ap/ae 用**直径分数**表示;切屑变薄修正开关;比切削力 k_c 供功率/扭矩估算;`provenance` 字段标注来源(catalog/manual/measured)。

### 16.2 刀具库格式(均核验)

- **LinuxCNC .tbl**(官方文档:https://linuxcnc.org/docs/html/gcode/tool-compensation.html ):每行一条;`T`(刀号唯一)`P`(刀位 1–1000)`X/Y/Z…`(偏置)`D`(直径)`I/J/Q`(车床);示例 `T1 P17 X0 Y0 Z1.5 D0.25`;G10 L1/L10 更新;
- **grblHAL**:默认无带偏置工具表(issue #79),以 G43.1 动态刀长为主 → 自研 CAM 以 **LinuxCNC 兼容 .tbl 作最低公分母交换格式**,富数据留自有 JSON;
- **Fusion 工具库 JSON**(schema zip:https://cam.autodesk.com/hsmtools ;导入说明:https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-import-a-tool-library-in-Fusion-360.html ):顶层 `{"data":[tool…],"version":N}`;tool 含 `geometry`(DC 直径/NOF 齿数/OAL/LB/RE 圆角/TA 锥角…)、`post-process`(number/diameter-offset/length-offset)、`start-values.presets`(tool_spindleSpeed/tool_feedCutting/f_z/v_c…);
- **FreeCAD ToolBit**(每刀一个 .fctb JSON + .fctl 库,可导出 LinuxCNC .tbl;https://wiki.freecad.org/CAM_ToolBit );
- **ISO 13399**(刀具参数代码:DC=切削直径、LF=功能长度等 70+ 项;https://www.sandvik.coromant.com/en-us/knowledge/machining-formulas-definitions/cutting-tool-parameters )——**自研 schema 几何字段直接采用其双字母代码**,免费获得与厂商目录的语义对齐。

### 16.3 自研刀具库 schema(建议)

```json
{
  "schema": "cam.tool/1",
  "id": "uuid", "tool_no": 5, "pocket": 17,
  "type": "endmill_flat|endmill_ball|endmill_chamfer|drill|face_mill",
  "iso13399": {"DC": 10.0, "LF": 30.0, "OAL": 75.0, "RE": 0.0, "NOF": 3, "SIG": 30, "DCONMS": 10.0},
  "shank": {"diameter": 10.0, "shoulder_length": 35.0},
  "assembly": {"holder_id": "cat40-er32-70", "gauge_length": 110.0, "stickout": 45.0},
  "cutting_data": [
    {"material_group": "iso_p.steel.low_carbon", "operation": "milling",
     "vc": [120, 180, 280], "fz": [0.03, 0.08, 0.18],
     "ap_max_frac_d": 1.0, "ae_max_frac_d": 0.5, "source": "catalog:sandvik"}
  ],
  "post_map": {"linuxcnc_tbl": "T5 P17 D10.0"},
  "life": {"minutes_used": 0, "limit_minutes": 300}
}
```

设计要点:几何用 ISO 13399 代码;切削参数内嵌刀具(按材料组索引);`assembly` 关联刀柄 + 伸出量(碰撞检测必需);**刀号/刀位属"机床状态",运行时才绑定**。

### 16.4 刀柄建模

- 锥度标准:7/24 锥度(每英尺 3.5 英寸,锥角 16.5943°;CAT=ANSI/ASME B5.50 体系,BT=JIS B6339/MAS 403/ISO 7388-2;Wikipedia 已核验:https://en.wikipedia.org/wiki/Machine_taper );CAT40 测量径 Ø44.45、法兰 Ø63.55、拉钉 5/8″-16;ER 筒夹 DIN 6499(8° 锥角,ER32 夹持 1–21mm);
- 工业实践 = **回转段堆叠**(非 B-rep):Fusion holder segments(upper/lowerDiameter + length + shoulderLength);ENCY CAM 用锥体+圆柱安全区(https://encycam.com/tech/adaptive-tilting );MecSoft 碰撞修正实践(https://mecsoft.com/blog/how-to-detect-correct-tool-holder-collisions-in-mecsoft-cam );
- Rust 表示:

```rust
enum RevolvedSeg { Cylinder { r: f64, len: f64 }, Frustum { r0: f64, r1: f64, len: f64 } }
struct ToolAssembly { segments: Vec<RevolvedSeg>, gauge_to_tip: f64 }
// 与高度场/体素毛坯碰撞:按 z 采样包络半径(锥台线性插值);
// 与网格夹具:分段 OBB/AABB 粗测 + 圆柱/锥台-三角形解析细测。
```

分层:切削部分(精确,含 RE)→ 柄部圆柱 → 刀柄锥台段;伸出量变化只移动轴向起点(`projection_adjust` 语义)。

### 16.5 夹具/装夹

- 语义参照:Fusion 在 Setup 内把实体指定为 fixture,刀路与仿真避开(https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-use-Fixture-attachment-in-Fusion-360-CAM-and-HSM.html );Vericut 机床装配树(Stock/Fixture/Design)+ 可配碰撞对(https://help.vericut.com/4_vericut_help/3.1.10_configure_collision_menu );
- 自研:参数化虎钳(`pose/jaw_width/opening/钳口高/垫块顶面 jaw_insert_top_z`)+ 压板点集(`xy/head_h`)+ 自由网格兜底(角板/吸盘);**固定体永远禁止切削进入,移动体全程参与碰撞检查**;`jaw_insert_top_z` 自动推算零件抬升。

### 16.6 工件坐标系(WCS)管理

- Fusion:一个 Setup = 一个 WCS,多偏置 = 多 Setup(https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/Defining-Work-Coordinate-Systems-in-the-Fusion-360-CAM-Workspace.html );双面件 G54/G55;多件/墓碑 = 每面一偏置 + 分度轴配合;
- 联动:`Setup { wcs, fixture_instance, stock_instance }` → 仿真逐 Setup 实例化(对齐 Fusion"只有加载的实体参与碰撞"行为);后处理输出 `G10 L2` 占位符由机床侧探针程序回填。

### 16.7 数据模型汇总

```
ToolCatalog(静态): Tool{iso13399+cutting_data[]} / Holder{锥度标准+回转段} / MaterialGroup{区间参数}
Job(工件级): Stock → Setup[](wcs + fixture_instance + ops[]) → ToolAssembly(Tool+Holder+stickout)
导出: LinuxCNC .tbl(兼容)+ 富 JSON(自有)+ Fusion JSON(互通)
```

---

## 17. G-code 传输与探测(串口发送/探针/调平)

### 17.1 流式协议(来源:grbl 官方 interface.md 与 Interfacing wiki,均已核验)

- **send-response**(逐行等 ok)最稳但最慢(短行高频段饿死 planner → 走走停停);**字符计数**(主机记账"已发未确认字节",RX 满载,运动连续)是推荐协议;
- RX 环形缓冲 128 字节,但主机预算取 **127**(grbl #982 维护者建议留 1 字节余量);状态报告的 `Bf:` 字段官方明确**不作流控**(数值滞后);
- worked example:行 25/40/31(=96 在途),再发 58 → 129 超限,必须等 ok;
- **EEPROM 写命令例外**(G10 L2/L20、G28.1/G30.1、`$x=`、`$Nx=`、`$RST=`):执行时关 RX 中断会丢字节 → 必须单发、等 ok,不走字符计数;
- 实时字符(`0x18` `~` `!` `?` `0x85` `0x84` 与 0x80–0xFF 倍率族)不占缓冲、缓冲满仍有效;`?` 轮询 ≤5 Hz;XON/XOFF 不被支持(USB 串口芯片时序不可靠)。

### 17.2 状态报告字段

| 字段 | 语义 |
|---|---|
| State | `Idle / Run / Hold:0-1 / Jog / Alarm / Door:0-3 / Check / Home / Sleep` |
| `MPos:` / `WPos:` | 二选一;`WPos = MPos − WCO` |
| `WCO:` | G5x + G92 + G43.1 之和;周期性或变化即发 |
| `Bf:15,128` | 可用 planner 块 / RX 字节(仅显示) |
| `F:` / `FS:` | 实时进给(率/+主轴 RPM),滞后 30–50ms |
| `Pn:XYZPDHRS` | 已触发引脚(XYZ 限位、**P=探针**、D 门、H hold) |
| `Ov:` / `A:` | 倍率 / 附件状态(S 主轴 CW、F 冷却) |

### 17.3 错误与恢复

- `error:`(38 码)→ 该行不执行,官方"GUI 应立即停止流";字符计数下 RX 内还有未执行行,**无法精确停在出错行** → 可靠恢复 = `0x18` 软复位(清空 planner+RX,模态失效)→ `$X`/`$H` → 重放模态组 → 从锚点(出错行或其后安全行)重发;"跳过错误行继续"不可取(grbl #817 作者原话语义);
- `ALARM:`(1–9):1 硬限位、2 软限位越程(位置安全)、4 探针初始态已触发、5 行程内未触发(第三方文档编号冲突时**以 gnea 官方为准**)。

### 17.4 Sender 参考架构(共同模式,源码/目录核验)

- **UGS**(Java,GPL-3.0):`connection/`(串口)→ `communicator/`(`BufferedCommunicator` 字节记账:`sentBufferSize += len+1`,ok 出队减回;错误且队列非空则 pause)→ `IController`(固件语义 + 状态轮询定时器)→ listeners(https://github.com/winder/Universal-G-Code-Sender );
- **bCNC**(Python,GPL-2.0):`Sender.py` + `controllers/`(每固件一子类)+ `GCode.py`(文档模型/Probe/调平变换)(https://github.com/vlachoudis/bCNC );
- **CNC.js**(Node,MIT):server + app 双层,controller 抽象多固件(https://github.com/cncjs/cncjs );
- **共同模式**:连接器抽象 → 命令队列+可换流控策略 → 固件语义层 → 状态机 → 事件广播 UI;实时命令旁路独立接口。

### 17.5 Rust 串口生态(核验)

- `serialport` 4.10.1(2026-09,MPL-2.0,月下载 ~144 万;**README 自述缺 Windows 维护者**——风险登记;macOS 同时枚举 `cu.*`/`tty.*` 需过滤;热插拔无系统推送 API,需轮询 `available_ports()`;https://lib.rs/crates/serialport );
- `tokio-serial` 5.5.0(2026-06,MIT,含 codec framing;https://lib.rs/crates/tokio-serial );
- Windows COM10+ 需 `\\.\COM10` 前缀(库内部已处理,UI 显示注意);macOS 一律用 `/dev/cu.*`(callout 口);备选 serial2/serial2-tokio(状态未验证)。

### 17.6 探测循环

- 语义(LinuxCNC:https://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g38 ):G38.2 朝工件+失败报警 / G38.3 朝工件不报 / G38.4/.5 离开工件;错误条件:目标=当前点、无轴字、刀补开启、F=0、探针已处目标态;结果参数 #5061–5069(触发点)、#5070(1 成功/0 失败);
- grbl:完成返回 `[PRB:x,y,z:1]`(WPos),`$#` 可查;**PRB 与 G43.1 TLO 均非持久**(复位即失);ALARM:4=初始态错、ALARM:5=未触发;
- 刀长/设零流程:探已知块高 → `G10 L20 P1 Z…`(EEPROM 类,**单发等 ok**)或 `G43.1 Z…`;bCNC `toolChange()` 用粗探+回退精探+feed 递减(源码核验)。

### 17.7 自动调平(bCNC 算法,源码级核验 https://github.com/vlachoudis/bCNC )

1. `Probe.scan()`:**蛇形网格**——提刀 zmax → G0 至格点 → G38.3 至 zmin → `%wait`;
2. `add(x,y,z)`:由 `[PRB]` 坐标算网格索引(0.1 步距容差)入矩阵;
3. `interpolate(x,y)`:**双线性四角插值**;`splitLine(...)`:**DDA 网格遍历**逐格切分 G0–G3 线段(弧先离散),每段 Z 加偏置;`setZero` 相对化;
4. OpenCNCPilot 同族实现(C#,MIT:https://github.com/martin2250/OpenCNCPilot );
5. 插值取舍:双线性 C0(快、不过冲探针点)默认;双三次 C1 可选(可能过冲);
6. **仿真侧**:探针命中 = 射线与毛坯/模型求交(BVH 一次 ray-cast),未命中按语义抛 ALARM:5;调平变换与发送共用同一代码路径,保证仿真重放与实机逐字节一致。

### 17.8 发送器状态机与伪代码

```
[Disconnected] → connect/握手($ 后收 [Grbl 1.1]) → [Idle] ⇄ [CheckMode]($C)
[Idle] → start job → [Running](逐行 ok 出队减字节)
[Running] --error:x--> [Error-Halted](停喂+记录行) --0x18--> [Idle*](须回零+重建模态+WCO 再续)
[Running] ⇄ [Hold](! / ~);[Jog]($J=,0x85 取消);[Alarm] ← ALARM:x;任何态 → 断开
```

```rust
const BUDGET: usize = 127;
let mut outstanding = 0;                    // 已发未确认字节(含 '\n')
loop {
    if let Some(rt) = realtime_rx.try_recv() { tx.write_all(&[rt])?; }   // 旁路
    while let Some(line) = rx.readline() {
        match line.as_str() {
            "ok" => { outstanding -= ack.pop_front(); }
            l if l.starts_with("error:") => { halt_feed(); record_line(); }
            l if l.starts_with("ALARM:") => { state = ALARM; }
            l if l.starts_with('<') => parse_status(l),
            l if l.starts_with("[PRB:") => prb = parse_prb(l),
            _ => {}
        }
    }
    if !paused && !errored {
        if let Some(cmd) = peek_next_job_line() {
            let n = cmd.len() + 1;
            if is_eeprom_cmd(&cmd) { drain_until_idle_then(cmd); }
            else if outstanding + n <= BUDGET {
                tx.write_all(cmd.as_bytes())?; tx.write_all(b"\n")?;
                outstanding += n; ack.push_back(n); advance_ptr();
            }
        }
    }
    if poll_due_5hz() { tx.write_all(b"?")?; }
}
```

### 17.9 安全分级

Hold(`!`,受控减速可恢复)→ Cancel(0x18+清队,模态失效须重建)→ 真急停(硬件断电);软限位越程 = ALARM:2;探测网格生成前校验 zmin/xmax 等不超 $130–132 行程。

---

## 18. 2D 输入:DXF 与 SVG

### 18.1 DXF 结构(来源:ezdxf 文档 + Autodesk 官方 DXF PDF,均核验)

- ASCII DXF = **(组码, 值) 标签对**流(两行一组);节:HEADER($变量)→ CLASSES → TABLES → BLOCKS → ENTITIES → OBJECTS;(0,'SECTION'/'ENDSEC'/'EOF')(https://ezdxf.readthedocs.io/en/stable/dxfinternals/filestructure.html );
- **组码决定值类型**:0–9 串 / 10–39 double / 60–79 int16 / 90–99 int32 / 210–229 法向量 —— 解析器分派表的依据;
- CAM 常用实体:LINE(10/20, 11/21)、ARC/CIRCLE(中心 10/20、半径 40、角 50/51 单位度)、LWPOLYLINE(90 顶点数、70 闭合、43 常宽、每顶点 42 bulge)、老式 POLYLINE+VERTEX+SEQEND(R12 只有这种)、SPLINE(71 阶/72 节数/73 控制点数/40 节值/41 权重)、ELLIPSE(41/42 参数起止角)、INSERT+BLOCK(递归展开)、LAYER(组码 8 挂层);
- **$INSUNITS**:1=英寸/4=毫米/…/0=无单位(常见缺失)→ 必须允许用户覆写;
- R12 vs R2000+:LWPOLYLINE/SPLINE/ELLIPSE 均为 R13+ 引入;二进制 DXF(22 字节哨兵)很少用 → **MVP 不做,检测哨兵后提示另存 ASCII**;
- 官方参考 PDF(直链已验证可达):R12 — https://damassets.autodesk.net/content/dam/autodesk/www/developer-network/platform-technologies/autocad-dxf-archive/acad_r12_dxf.pdf ;2012 — https://images.autodesk.com/adsk/files/autocad_2012_pdf_dxf-reference_enu.pdf ;组码镜像站 — https://documentation.help/AutoCAD-DXF/WS1a9193826455f5ff18cb41610ec0a2e719-7a6f.htm

### 18.2 Bulge → 圆弧(CAM 关键几何)

`b = tan(θ/4)`(θ 为该顶点到下顶点的包含角,带方向;b>0 逆时针、b<0 顺时针、b=0 直线;|b|<1e-9 按直线;**b=±1 恰为半圆**):

```
θ = 4·atan(b)
r = d·(1 + b²) / (4|b|)          (d = 弦长)
矢高 s = b·d/2
圆心 = 弦中点沿垂线偏移 √(r² − (d/2)²)(侧向由 b 符号定,实现用 θ 代回验证)
```

### 18.3 自研 DXF 解析器(5 阶段)

```
① 分词:逐行 → (i32 组码, String 值);容错码页/CRLF/999 注释
② 分节:扫描 (0,SECTION)…(0,ENDSEC)
③ 建模:HEADER $变量 map / TABLES(LAYER) / BLOCKS(块名→实体表) / ENTITIES 收集
④ 几何化:实体 → 内部 Path{Line|Arc|Nurbs}(带方向)
⑤ INSERT 递归展开:变换合成(插入点·缩放·旋转)、嵌套深度上限 + 自引用检测、列阵展开
```

- SPLINE 两场景:有控制点+节 → 直接 NURBS 求值(de Boor,支持权重)按弦高离散;只有拟合点 → 反算控制点(ezdxf 做法,https://ezdxf.readthedocs.io/en/stable/dxfentities/spline.html );
- ELLIPSE:`P(t) = C + rot(φ)·(a·cos t, b·sin t)`,自适应步长;弧离散统一式:`Δθ = 2·acos(1 − tol/r)`;
- **负缩放(镜像)必须翻转绕向**(det<0;绕向决定顺逆铣与补偿方向,见 18.5);
- 参考解析器:ezdxf(MIT,文档最全)、libdxfrw(**GPL-2.0,只读不链接**;https://github.com/LibreCAD/libdxfrw )、Rust `dxf` crate(0.6.2,2026-10,MIT;https://github.com/ixmilia/dxf-rs )。

### 18.4 SVG(官方规范已核验)

- path 命令 M/L/H/V/C/S/Q/T/A/Z(https://www.w3.org/TR/SVG11/paths.html );
- **A 命令 → 中心参数化**(SVG 1.1 附录 F.6.5/F.6.6:https://www.w3.org/TR/SVG11/implnote.html ):① 半径校正(rx/ry=0 → 直线;弦不可达 → 同乘放大系数);② 端点旋 −φ 归位算 c′(符号规则 **fA≠fS 取 +,fA=fS 取 −**);③ 旋回加弦中点;④ θ1/Δθ(叉积定号;**fS=0 时 Δθ 必负,fS=1 时必正**);
- transform 栈:列表按书写顺序后乘(最右先作用),嵌套 g 逐级左乘;viewBox + preserveAspectRatio 映射(https://www.w3.org/TR/SVG11/coords.html );
- **SVG 无绝对物理单位**(px=用户单位)→ 用户指定"1 用户单位 = X mm";
- fill-rule(nonzero 反向绕向即孔 / evenodd 奇偶)可辅助内外轮廓判定;
- 参考实现:usvg/resvg(Apache-2.0/MIT;https://github.com/linebender/resvg )——读其 transform 归一化与 path 解析;我们只取描边轮廓,不做填充偏移。

### 18.5 通用问题与规则

单位推断(两个格式都让用户覆写,不静默猜测);**det<0(镜像)翻转绕向 → 展开完所有变换后统一按 signed area 重新规范化,绝不信文件"原方向"**;开/闭轮廓分类(闭合才可挖槽;首尾容差重合的"几何闭合"要提示);深度由用户指定(每轮廓/层:切深、每层 stepdown、抬刀高);按 LAYER/颜色分派不同深度是低成本高收益功能;岛/孔嵌套判定(绕向计数 + 包含测试)。

### 18.6 实现路线

阶段 A:DXF(LINE → ARC/CIRCLE → LWPOLYLINE bulge → POLYLINE → INSERT 递归 → SPLINE → ELLIPSE);阶段 B:SVG(transform 栈 → path 段 → A 转换)。两者共用内部 `Path{Line|Arc}` 与刀路后端。对拍:ezdxf 脚本生成参照输出 + `dxf` crate 交叉验证;容差(弦高/闭合判定/bulge 阈值)全部配置化。

---

## 19. 商业竞品与市场格局(价格 2026-11 核验)

### 19.1 竞品速览

| 软件 | 定位 | 价格(验证方式) | 强项 / 弱项 |
|---|---|---|---|
| Fusion 360 | 云 CAD+CAM 全能 | $85/月、$680/年;个人版免费(2026-10 起限 10 个活动文档);Machining 扩展 ~$1,465/年(官方页 403,数字为官方域名内搜索转述,**未直接核验**) | 全能+后处理库大 / 云订阅+政策多变、UI 受诟病(观点) |
| Vectric VCarve Pro | 2.5D/雕刻王者,买断 | Pro $699、Desktop $349、Aspire $1,995(一次性;官方购买页域内搜索确认) | V-carve/嵌套/教程生态 / 仅 Win、3D 铣弱 |
| EstlCam | 德系 hobby 极简 | 49€ 买断(官方订单页直接核验) | 摆线铣、可兼控制器 / 仅 Win、UI 老 |
| Carbide Create | Shapeoko 生态随附 | 免费锁机器;Pro $120/年或 $360 买断 | 免费档完整 2D+仿真 / **不支持 STEP**(官方建议改用 MeshCAM) |
| OpenBuilds CAM | 免费浏览器 2D/2.5D | 免费(开源 https://github.com/OpenBuilds/OpenBuilds-CAM ) | 零门槛 / 无 3D |
| MeshCAM | "最简 3D CAM" | Standard $250、Pro $500 买断(官方购买页直接核验:https://www.grzsoftware.com/buy.php ) | 自动进给、极简流程 / 刀路非效率优化、无探针 |
| Mastercam | 工业事实标准 | 询价制(社区流传 $6K–15K/座席,**未验证**) | 2–5 轴深度、人才存量 / 贵、陡 |
| SW CAM / CAMWorks | AFR 强项 | SW 订阅附带 Standard;Pro 询价(捆绑 $3K–20K 传闻,未验证) | 特征识别自动加工 / 锁生态 |
| SprutCAM X | 多轴+机器人 | $249/月起订阅(官方商店核验) | 机器人离线编程 / 订阅制 |
| FreeCAD CAM | 开源 | 免费 | 参数化一体 / 2.5D 为主、打磨弱、3D 弱 |
| Kiri:Moto | 开源浏览器 CAM | 免费+自托管(https://grid.space/kiri ) | 三合一(打印/CNC/激光) / 深度有限 |
| NC Viewer / CAMotics | 免费查看/仿真 | 免费 | 即开即用 / 老旧、无商业化迭代 |

### 19.2 格局

价格断层:免费 → $50–500 买断 → $700–2000 → $3K–20K → $6K–30K+ 询价。**"$500–2500 的真 3D + 现代 UI + 买断"档最稀薄**(MeshCAM 克制、Aspire 主攻浮雕)。人气排序(CNCCookbook 多年调查,观点性来源):Fusion > Vectric > Mastercam(https://www.cnccookbook.com/best-cad-cam-software-for-cnc-machining-beginners )。

### 19.3 差异化机会与风险

- 机会:① 本地优先/隐私(云订阅与许可政策不满长期积累);② 低价档普遍缺**高质量实时材料去除仿真**与现代 UI;③ 可编程性(脚本/插件)近乎空白;④ Rust 同时支撑 WASM(Web 试用)与原生桌面;
- 风险:Fusion 个人免费 + $680/年 价格锚压制 $50–700 档;Vectric 教程/渠道壁垒;工业档护城河(后处理库/人才)短期不可企及;市场盘子小、学习成本高;开源替代蚕食底部。

### 19.4 国内简况(搜索汇总,**未验证**)

中望 3D(CAD/CAM 一体)、CAXA 制造工程师(教育市场)、北京精雕 JDPaint/SurfMill(五轴精密、垂直一体);机会更可能在教育、轻工雕刻与出海。

---

## 20. UI/UX 信息架构与桌面工程化

### 20.1 参照拆解

- **Fusion Manufacture**:工作流 = Setup → Ops → Simulate → Post;Setup 对话框含 Operation Type/机床/WCS + 独立 Stock 标签;工序面板分组 **Tool / Geometry / Heights(安全面/退刀/Z 深)/ Passes(步距/参数)/ Linking(进退刀)**;浏览器树 = 行业事实标准(Setup→工序→刀路,状态图标,右键 Edit/Duplicate/Generate/Simulate)(帮助页:https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-BEC5DEA9-AC3E-4FA8-998E-4AE8CD0D0B1E ;AU 讲义 PDF:https://static.au-uw2-prd.autodesk.com/MFG226547-L_Class_Handout_MFG226547L_Fundamentals_of_the_CAM_Workspace_Using_Fusion360_Learn_2D_and_3D_Toolpaths_Kevin_Lee.pdf );
- **VCarve**(官方手册已抓取):菜单 + 2D/3D 双视图(可平铺)+ 底部标签;Toolpaths 标签 = Material Setup → 策略列表 → **Toolpath Tree 置底**(编辑/重算/模板/合并/预览/工时汇总)(https://docs.vectric.com/docs/V12.5/VCarvePro/ENU/Help/page/main-window/index.html );
- **CAMotics**(手册已抓取):可停靠面板 Project/Context/Position/Estimates/Machine Status/Bounds;**用位置滑杆拖动回放**而非实时切削;布局三套预设,退出保存(https://camotics.org/manual.html );
- **NC Viewer**:编辑器 + 视口 + DRO 数字读出 + Machine Options + Backplotter 显示开关(https://ncviewer.com );
- **EstlCam**:线性向导式流程(https://www.estlcam.de/manual.php )。

### 20.2 egui 实践要点(均核验)

- 停靠布局:`egui_dock`(DockState/DockArea,拖拽换位、拖出成窗;https://docs.rs/egui_dock );
- 3D 视口:`egui_wgpu::CallbackTrait`(prepare 每帧 uniforms / paint 在 egui pass 内 draw)+ `Callback::new_paint_callback`;官方示例 custom3d_wgpu.rs(https://github.com/emilk/egui/blob/master/crates/egui_demo_app/src/apps/custom3d_wgpu.rs ;https://docs.rs/egui-wgpu/latest/egui_wgpu/trait.CallbackTrait.html );
- 长任务:后台线程 + mpsc 进度事件;UI 每帧 `try_recv`;完成时 `ctx.request_repaint()`;空闲 `request_repaint_after(300ms)`;egui `Context` 是 Send+Sync(issue #4368/#5859);
- 撤销:egui 无全局命令栈;`egui::util::undoer::Undoer` 适合表单标量(https://docs.rs/egui/latest/egui/util/undoer/index.html );刀路级用**自研 Command 模式**(几何计算代价大不宜快照;接口参考 `undo` crate:https://docs.rs/undo );
- 主题/中文:`ctx.set_theme`;CJK 需 `FontDefinitions` 注入 TTF(建议**内嵌 Noto Sans SC**,不依赖系统字体;crate egui-cjk-font 可参考)。

### 20.3 工程文件格式

- 先例:Fusion `.f3d`/`.f3z` = ZIP;**FreeCAD `.fcstd` = ZIP**(`Document.xml` 必须为第一条目 + `GuiDocument.xml` + 逐对象 `.brep` + thumbnail;https://wiki.freecad.org/File_Format_FCStd );Kiri:Moto = base64 JSON;
- 建议:`project.cam` = ZIP:`document.ron`(场景/Setup/工序树/参数,serde+ron,可 diff)/ `toolpaths/`(逐工序二进制缓存,改动才重算)/ `gui.ron`(布局独立,免污染 undo)/ `thumbnail.png`;外部引用记相对路径 + 提供"打包引用"选项。

### 20.4 打包与分发

- **cargo-dist(现名 dist)**:`dist init` 生成 tag 触发的 GH Actions(Plan→Build→Publish→Host);产物带校验和与 shell/PowerShell/msi 安装器;`dist-manifest.json` 机器可读(https://github.com/axodotdev/cargo-dist );
- Windows SmartScreen 基于**哈希+证书信誉**:EV 证书即时获得信誉、OV 需数周积累(Microsoft Learn:https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/smartscreen-reputation );
- macOS:Developer ID 签名 + **Hardened Runtime** + 公证(Apple:https://developer.apple.com/documentation/security/notarizing-macos-software-before-distribution );
- wgpu 兼容性:`RequestAdapterOptions`/`enumerate_adapters` → 失败回退 `force_fallback_adapter` 软渲染 + UI 提示(https://docs.rs/wgpu/latest/wgpu/struct.RequestAdapterOptions.html )。

### 20.5 i18n 与帮助

rust-i18n(简单,宏 `t!`)或 fluent(复数语法强,多语言扩展选它);egui `on_hover_text` 原生 tooltip —— 参数面板每个加工字段旁挂术语解释。

### 20.6 信息架构建议(综合)

- **左:制造浏览器树**(工程 → Setup(毛坯/WCS)→ 工序),每工序状态点(未计算/计算中/警告/已过时),右键 = Fusion 式菜单;
- **中:egui_dock 多视口**:3D 视口默认最大;可挂 2D 图纸视口、G-code 文本视口;
- **右:上下文属性面板**:选中工序显示 Tool/Geometry/Heights/Passes/Linking 五分组(CAMotics "Context dock" 思想,常驻不弹窗);
- **底部:仿真时间轴**(位置滑杆 + 播放/倍速/方向 + 进度条 + 当前行号/主轴/进给 + 剩余时间);
- 长任务:`JobHandle { id, name, cancel: Arc<AtomicBool>, rx: mpsc }`;"已过时" = 标记重算队列而非阻塞。

---

## 21. NURBS 数学与刀具接触公式(深潜)

### 21.1 B 样条 / NURBS 核心(来源已核验:Wikipedia B-spline / De Boor's algorithm / NURBS 条目;《The NURBS Book》)

- **Cox-de Boor 基函数递推**(https://en.wikipedia.org/wiki/B-spline ):

```
B_{i,0}(t) = 1  if t_i ≤ t < t_{i+1}, else 0
B_{i,p}(t) = [(t−t_i)/(t_{i+p}−t_i)]·B_{i,p−1}(t) + [(t_{i+p+1}−t)/(t_{i+p+1}−t_{i+1})]·B_{i+1,p−1}(t)
(0/0 = 0;节点数 = 控制点数 + 次数 + 1)
```

- **de Boor 求值**(只有 i = k−p..k 项非零;https://en.wikipedia.org/wiki/De_Boor%27s_algorithm ):

```python
def deBoor(k, x, t, c, p):
    d = [c[j + k - p] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            alpha = (x - t[j + k - p]) / (t[j + 1 + k - r] - t[j + k - p])
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
    return d[p]
```

- 节点矢量:**clamped**(两端 p+1 重复,端点插值)/ uniform;**m 重节点处连续性降为 C^{p−m}**,重数上限 p(超过则曲线断开);
- **有理化(实现要点)**:控制点升为 4D 齐次 `(w·x, w·y, w·z, w)`,跑普通 de Boor,最后**透视除法**;要求 `w_i > 0`;圆/椭圆/圆锥曲线可精确表示(https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline );
- **Boehm 节点插入**(Shene 讲义,已核验:https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/single-insertion.html ):`Q_i = (1−a_i)P_{i−1} + a_i P_i`,`a_i = (t−u_i)/(u_{i+p}−u_i)`;重复插入同一节点 → 提高重数/切段/Bézier 化;
- **张量积曲面**:先 v 向逐行曲线求值、再 u 向(O(p·q));偏导用基函数导数(NURBS Book A2.3 DersBasisFuns);**有理曲面导数必须对齐次坐标求导后按商法则组合**,否则变权重处法线有误差;法线 = S_u × S_v 归一;
- **弦高自适应离散**:`s = R − √(R² − (L/2)²) ≈ L²/(8R)` → 段长上限 `L ≤ √(8εR)`(R 取局部主曲率半径 1/κ_max);均匀离散仅作 fallback;
- **裁剪曲面流程**:裁剪环(3D 边 + 参数域 pcurve)→ pcurve 按弦高离散成域内折线 → 域内网格与环求交约束化 → **带孔多边形三角化**(ear clipping/约束 Delaunay)→ 顶点抬升到 S(u,v);对照 OCCT BRepMesh(https://dev.opencascade.org/doc/overview/html/visualization__mesh.html );参数域三角化奠基:Piegl & Richard 1995(https://bojianwu.github.io/gsoc2016/week_6/NURBS_tess.pdf );
- 教材:**The NURBS Book**(Piegl & Tiller, Springer 1997;关键算法 A2.1 FindSpan / A2.2 BasisFuns / A2.3 DersBasisFuns / A3.5 SurfacePoint / A4.4 SurfaceDerivs / A5.1 CurveKnotIns;PDF:https://link.springer.com/content/pdf/10.1007/978-3-642-59223-2.pdf );
- 实现参考:**neco-nurbs**(Rust,MIT,2026-10 活跃;https://github.com/barineco/neco-cad );**纠偏:"tinynurbs"是 C++ 库不是 Rust**(crates.io 搜索 0 结果,已核验;https://github.com/caadxyz/TinyNurbs );黄金对照:geomdl/NURBS-Python(逐算法实现,https://nurbs-python.readthedocs.io );openNURBS(C++,可读:https://github.com/mcneel/opennurbs )。

### 21.2 drop-cutter 通用框架(opencamlib 源码逐条核验,LGPL-2.1 只读)

- 每三角形 **7 类测试:3 顶点 + 3 边 + 1 面**;刀具由 `height(r)/width(h)` 截面函数完全参数化(`millingcutter.cpp`:https://raw.githubusercontent.com/aewallin/opencamlib/master/src/cutters/millingcutter.cpp );
- **顶点测试**:`q ≤ R → z = p.z − height(q)`;圆鼻 height:`r ≤ R1 → 0;R1 < r ≤ R1+R2 → R2 − √(R2² − (r−R1)²)`;
- **面测试通用式**:接触矢量 = `xy_normal_length·(n_x,n_y,0)/|·| + normal_length·n`;圆鼻参数:xy_normal_length = R1、normal_length = R2、center_height = R2;接触点须在面内(`liftZ_if_inFacet`);
- **圆鼻刀边测试(offset-ellipse)完整推导**(Wallin 2014 博客已抓取:https://www.anderswallin.net/2014/02/drop-cutter-toroid-edge-test/ + `bullcutter.cpp/ellipse.cpp/ellipseposition.cpp`):
  1. 规范位:CL 平移到原点、边转到 x 轴;
  2. 水平边特例:直接 `cl_z = u1.z − height(u1.y)`;
  3. 核心:半径 R2 的斜圆柱被水平面截得椭圆——短轴 b = R2,**长轴 a = |R2/sinθ|**(θ 为边倾角);
  4. offset ellipse(椭圆点沿法线外移 R1 = 主半径)与"刀切到边"等价,无闭式解 → 数值求根;
  5. 免三角参数化:`e(s,t) = center + (a·s, b·t), s²+t²=1`;法向 `N = (b·s, a·t)/|·|`;offset 点 `oe = e + R1·N`;
  6. 目标函数 = oe 的 y 坐标(到边的有向距离);参数用 diangle∈[0,3] 括根,**Brent 求根**(容差 1e-10);
  7. 双解(环面底/顶各一)取**更高 z**(drop 自上而下);
  8. `cl_z = 椭圆中心 z − R2`;
- **锥刀**(conecutter.cpp):`height(r) = r/tanα`;边测试判别阈 `mu = (center_height/R)·xu/√(xu²+d²)`;`|m| ≤ |mu|` → 锥面双曲线接触(给出闭式 ccu),否则底圆接触;
- **复合刀统一**(compositecutter.cpp):按半径/高度带堆叠子刀(CylCone/BallCone/BullCone/ConeCone),每子刀带 zoffset,接触点径向校验 `ccValidRadius` —— 即"按径距 d 分段选用公式"的实现;学术母本:Yau, Chuang & Lee 2004(一般 APT 刀;付费墙,框架描述属二手转述);
- **自适应采样**(Wallin 2010,已抓取:https://www.anderswallin.net/2010/10/adaptive-sampling-drop-cutter/ ):均匀初值 → 相邻点中点再 drop → **平坦判据(归一化点积)**低于阈值即加密并递归,`min_sampling` 截断;理论源头:Graphics Gems V, "Adaptive Sampling of Parametric Curves";有解析曲率时直接用弦高式 `L ≤ √(8εR)` 预排步距。

### 21.3 实现顺序

曲线(4D de Boor)→ 曲线弦高离散 + 节点插入 → 曲面求值/法线 → 均匀栅格三角化 → 自适应细分 → 裁剪曲面(对拍 BRepMesh);drop-cutter:**球头(最简)→ 平底 → 圆鼻(canonical + Brent,对拍 opencamlib)→ 锥/复合**;自适应采样并行推进。

### 21.4 桶形刀(Barrel)与圆弧段刀路(2026-11-07 补查)——五轴精加工的现代质量杠杆

**几何**:Circle-Segment 刀族按母线分四种——barrel(桶)/oval(卵)/taper(锥)/lens(透镜),共性是切削刃含一段**大半径圆弧母线**(常规 50–100mm;锥形桶刀 ≥1000mm;透镜形 Ø10 刀配 20mm 切削半径);切触为**椭圆接触带**而非球头单点——这是"残留对行距不敏感"的几何根源(来源:MoldMaking Technology《Ten Things You Need to Know about Circle Segment Milling》,https://www.moldmakingtechnology.com/articles/ten-things-you-need-to-know-about-circle-segment-milling ;hyperMILL 动态切触点控制:https://www.openmind-tech.com/en-us/cam/hypermill-maxx-machining/finishing/conical-barrel-cutter )。

**行距增益公式(自研推导,与厂商数据互洽)**:同残留下行距 ∝ √有效半径 → **增益 = √(R_b/R_球)**:16mm 桶刀(R_eff=500)vs R8 球头 ≈ **7.9×**;hyperMILL 案例 350mm 母线 vs R3 球头 ≈ **10.8×**。实测:循环时间降 70–95%(MMT:9h36m→1h10m;-90% OPEN MIND、Kyocera、Emuge-Franken 同口径,链接见来源汇总)。风险侧(GWS):需 5 轴/精细 3+2;大切距带来**高于球头的切削力**;夹持/让位不稳时大行距无意义(https://www.gwstoolgroup.com/circle-segment-end-mills-vs-ball-nose-end-mills-when-to-use-each )。

**刀位计算与球头 drop-cutter 的差异**:桶刀母线 = 大 minor radius 的**环面片段**——与圆鼻刀同族:固定刀轴(3+2)下复用 BullCutter"滚圆"逻辑,滚圆半径换成母线半径 R_b,新增两项:①**切触点是否落在弧段有效区间**的边界检查;②行距按有效半径(非刀径)计算。5 轴联动下切触点沿母线的位置本身是自由度,需要**曲率匹配**(选母线上曲率≈曲面法曲率的区段,把残留"熨平")+ **刀姿连续性**(相邻切触点刀轴连续,否则椭圆带跳跃出棱纹;Suzuki 2021, DOI 10.20965/ijat.2021.p0885)。其他学术锚点:双切触点选刀(Meng 2014 叶盘,被引 ~55)、包络面逼近反求刀位(CAD'14, DOI 10.14733/cadconfp.2014.186-188)、桶刀设计+刀路一体(IJAMT, DOI 10.1007/s00170-015-8113-z)、力学/稳定性建模(Urbikain & López de Lacalle, DOI 10.1016/j.precisioneng.2018.11.007)。

**纠偏:查无实据的引用**——"Autodesk Generalized Machining: Using Curved Cutting Tools…(Engeli/Rasch)"经 WebSearch/Crossref/dblp 三渠道检索均无果,**不要引用**;可验证的 Autodesk 事实是产品能力:PowerMill barrel 工具支持与 Fusion Circle Segment 博客(官方页存在,403 未读正文)。

**商业支持**:PowerMill(barrel + Flowline)、hyperMILL MAXX(tangent/conical/lens)、Fusion(circle segment);自研现状:出现初期仅 2 家 CAM 支持,现已普及——**跟进不领先,但网格输入 + sd_profile 路线仍可差异化**。

**自研评估**:`sd_profile` 新增"圆弧母线环面片段"profile(参数 R_b/弧心距/弧段端点),切触复用圆鼻滚圆代码 + 弧段边界检查——**3+2 定向桶刀原型难度中低**;5 轴切触分配/刀姿连续为二期。**何时值得做:叶片/叶盘、大型模具型腔、车身覆盖件等大曲率半径+五轴可达精加工;3 轴小曲率件收益为负(球头不可替代场景:深腔/小圆角/受限让位)。**验证方式:先出"同残留下行距倍数"报告页,对照 7.9×/10.8× 量级。

---

## 22. 性能基准与目标

### 22.1 公开基准大表(数字均注明出处与条件)

| 组件 | 条件 | 数值 | 来源 |
|---|---|---|---|
| GPU voxel 去除 | 100μm voxel,13,200cm³ 毛坯 43% 去除,V100 级 GPU | <60 s/件(≈140× 实时加工) | https://link.springer.com/article/10.1007/s00170-021-07001-w |
| GPU voxel vs CPU | 粗加工件 | 18–25× | 同上 |
| GPU TriDexel vs 多边形仿真 | 12GB NVIDIA 卡,40–92 万动作/件 | 2–7×(12min→2min 等) | https://mecsoft.com/blog/revolutionizing-cam-simulation-nvidia-gpu-accelerated-tridexel-simulation-in-rhinocam-visualcad-cam |
| CAMotics | v1.0.6 CHANGELOG | 提速 4.5×(相对值) | https://raw.githubusercontent.com/CauldronDevelopmentLLC/CAMotics/master/CHANGELOG.md |
| drop-cutter(C# 原型,无空间索引) | 1894 三角,473.5 万调用,2007 单核 | ~5 s ≈ 95 万调用/s | https://www.anderswallin.net/2007/08/drop-cutter-in-c-v2/ |
| waterline(push-cutter) | Tux 模型 5 条等高线,2010 单核 | 96 s | https://www.anderswallin.net/2010/07/waterline-toolpaths-part-2/ |
| weave 构建优化 | 预因子计算 | 2ms → 0.25ms(10×) | https://www.anderswallin.net/2011/05/faster-waterline/ |
| 网格布尔(Manifold) | 1000 圆角盒并集;223 次 ~380 三角扫掠减 | 10.5 s;525 ms ≈ 2.4 ms/op(1.02 Mtris/s) | https://github.com/elalish/manifold/discussions/383 、https://solidean.com/blog/2026/terrain-carve-benchmark/ |
| Marching Cubes | MacBook M3,512³ 体 | 126.5 MVx/s | https://github.com/seung-lab/zmesh |
| BVH/GPU 软光线遍历(参考量级) | RTX 5080 Laptop,Sponza 26.2 万三角 | ~5 Grays/s | https://github.com/jbikker/tinybvh |
| STL 解析(二进制) | 95MB/200 万三角,R9 8945HS | 950 MB/s(1T)/1.86 GB/s(16T);ASCII 1.1 GB/s | https://github.com/pyvista/pyvista-stl |
| STEP 解析(OCCT) | 大文件 STEPCAFControl_Reader | ~10 s(7.8 版内存管理整体提速 40%) | https://occt3d.com/dev/content/load-big-file-stepcontrolreader-cost-too-much-time |
| 摆线 vs 传统(加工周期) | 铣削实验 2020 | 周期 −23%,进给 +35% | https://www.researchgate.net/publication/344028429 |
| Z-map 吞吐 / G-code 解析 / Clipper2 文本数值 / drop-cutter C++ 点/s | — | **未找到公开数据 → 自建基准** | — |

### 22.2 本项目性能目标(MVP,公开数据外推)

| 项 | 目标 | 依据 |
|---|---|---|
| STL 解析 | ≥1 GB/s(单核即可达) | pyvista-stl 基准 |
| G-code 解析 | ≥1M 行/s | 无公开对标,自建 |
| drop-cutter | ≥10–50 M 三角测试/s/核(BVH 剪枝后有效测试 <5%) | 原型 95 万调用/s + 15 年硬件/多核红利 |
| Z-map 更新 | ≥50–100 Mcell/s/核;典型件(100×100mm @0.1mm)整件 ≤10 s | GPU voxel 数据按面积折算留 10× 余量 |
| Marching Cubes | ≥100 MVx/s(多核) | zmesh 单机 126 MVx/s |
| 网格布尔 | MVP ≤10 ms/op;终期追平 Manifold ≈2.4 ms/op | Manifold/solidean |
| BVH 构建 | 1M 三角 ≤1 s;遍历 Mrays/s 量级 | Embree/TinyBVH 参照 |
| waterline | Tux 级 5 条等高线 ≤2 s | 2010 年 96 s × weave 10× × 多核 × 硬件 |

**方法论**:把 drop-cutter 95 万调用/s、布尔 2.4ms/op、STL 1GB/s、MC 126MVx/s 四个数字做成 CI 基准回归门槛;公开数据缺失的项(Z-map 吞吐、drop-cutter 点/s、摆线生成耗时、G-code 解析)以自建基准为准绳。

---

## 23. 五轴机床运动学与 RTCP

> 2026-11-07 需求升级新增。符号:**P**=(Px,Py,Pz) 刀心点 TCP(工件坐标系);**u**=(i,j,k) 刀轴单位矢量(由刀尖指向刀柄,立式默认 (0,0,1));**L** = 刀长(刀尖到主轴端);**d** = 摆头枢轴距(枢轴到主轴端);Rx/Ry/Rz 为右手系基本旋转。
> 经典方法:Lee & She, "Developing a postprocessor for three types of five-axis machine tools", *Int. J. Adv. Manuf. Tech.* 13(9):658–665, 1997, DOI 10.1007/BF01350824(https://link.springer.com/article/10.1007/BF01350824 );一般化扩展:She & Lee(https://www.sciencedirect.com/science/article/pii/S1526612500700211 )。
> **警告:旋转顺序与符号随具体机床运动链而变,以下公式为实现模板,出厂前必须按机床标定数据核对。**

### 23.1 构型分类

| 构型 | 运动链 | 特点 | 代表 |
|---|---|---|---|
| **转台-转台**(摇篮 BC/AC) | 两回转轴在工件侧,主轴竖直固定 | **IK 最简单,回转角与刀长无关**;五面加工/模具/叶轮常见 | Haas UMC-500(+120°~−35°/360° 连续;https://haascnc.com/machines/vertical-mills/universal-machine/models/umc-500 );Pocket NC V2(桌面,A±120°+B 连续;https://www.pentamachine.com/all-products/p/pocket-nc-v2-50chb-or-chk ) |
| **摆头-摆头**(AC/BC 双摆头) | 两回转轴在主轴侧,工作台只 XYZ | 适合大件(工件不动);**刀长 L、枢轴距 d 进入所有线性轴输出**;换刀后非 RTCP 程序全部失效 | 航空结构件机床 |
| **摆头-转台**(B 头 + C 台) | 一头一台 | 折中;XYZ 输出受 L 影响,回转角不受 | DMG MORI DMU 系列、Hurco 混合 |
| 三轴机 + 回转附件 | 主轴竖直 + 竖装回转台 | 刚性/精度受限,3+2 为主 | Tormach 1100MX + 回转台(https://tormach.com/machines/mills/1100mx.html ) |

**过摆(swing over)**:摇篮/转台绕水平轴摆动时,工件/夹具最大回转半径超出摆动包络净空 → 特定摆角干涉;选型与 CAM 检查均须考虑(术语口径较散,未验证统一来源)。

### 23.2 正逆运动学闭式公式(实现模板)

通用方法:机床分解为"工件侧链 + 刀具侧链",CL 数据满足两侧一致,齐次变换展开闭式求解(推导框架同 Lee & She;Missouri S&T 硕士论文按 table-tilting/spindle-tilting 演示:https://scholarsmine.mst.edu/cgi/viewcontent.cgi?article=8096&context=masters_theses )。

**(a) 转台-转台(摇篮,链:工件←C←B←基座)** —— 姿态约束 `Ry(B)·Rz(C)·u = (0,0,1)`:

```
FK:  u = ( −sinB·cosC,  sinB·sinC,  cosB )
IK:  B = atan2( √(i²+j²), k )            # 主支 B∈[0,π];对称支 B′=−B, C′=C+180°
     C = atan2( j, −i )                  # sinB≈0 → C 不定(奇异,§23.4)
点变换: P_base = Ry(B)·Rz(C)·(P − r0) + O_t     # O_t=回转轴交点(标定), r0=工件原点偏置
非 RTCP 输出: S = (P.x, P.y, P.z + L)            # 刀长只进 Z,不进 B/C
```

**(b) 摆头-摆头(链:基座←C←A←主轴)** —— 约束 `Rz(C)·Rx(A)·(0,0,1) = u`:

```
FK:  u = ( sinA·sinC,  sinA·cosC,  cosA )
IK:  A = atan2( √(i²+j²), k )            # ±双解,按行程/就近选支
     C = atan2( i, j )
刀心: P_base = O_h + (L − d)·u_base      # u_base = Rz(C)Rx(A)·(0,0,1);O_h=摆头枢轴(可动输出轴)
反解: O_h = P_base + (d − L)·u_base
     → 仅 Z 动时: Z_cmd = Pz + (d − L)·cosA
# L 与 d 直接进入所有线性轴输出 —— head-head 非 RTCP 程序绑定刀长的根源
```

**(c) 摆头-转台(B 头 + C 台)**:回转角公式同 (a);点方程为耦合式 `O_t + Rz(C)·(P − r0) = O_h + (L−d)·Ry(B)·(0,0,1)`(转台链 = 摆头链),线性轴受 L 影响、回转角不受。

**每构型必标定参数**:两回转轴的方向/顺序/符号、回转交点或摆头枢轴在机床系的位置向量(2~3 个非零分量)、工作台面到枢轴距离、刀长 L(运行时输入)。对应 Fanuc 侧为 19700–19705 类旋转轴标定参数(二手来源:https://industrialmonitordirect.com/blogs/knowledgebase/fanuc-g434-tcp-behavior-with-g54-work-offset-shifts-on-5-axis ,已抓取)。

### 23.3 控制器 RTCP 实现(逐家核验)

| 控制器 | RTCP(五轴联动) | 3+2 定位 | 备注 |
|---|---|---|---|
| **Fanuc** | **G43.4**(刀轴方向刀长补偿 Type 1,保刀心)/ **G43.5**(Type 2,刀姿控制);与 G43/G43.1/G43.3/G49 同组 08;F=刀心速度 | **G68.2**(倾斜工作平面,`G68.2 X_ Y_ Z_ I_ J_ K_`)+ **G53.1**(刀对准法向),G69 取消;G43.4 须在 G68.2 之后、配 H=全长 | 参数 6005#7 选工件系/表系实现;变换顺序 work offset → G68.2 → G43.4 逆解 → 轴运动(佐证:https://industrialmonitordirect.com/blogs/knowledgebase/fanuc-g434-tcp-behavior-with-g54-work-offset-shifts-on-5-axis 、样例 https://www.emastercam.com/forums/topic/94855-g682-program-sample 、https://zappettiniconsulting.com/wiki/fanuc-tilted-working-plane-g682 ) |
| **Siemens 840D sl** | **TRAORI** 激活 / **TRAFOOF** 取消;TOROT/TOOLOR/TRAAANG 刀轴重定向 | **CYCLE800**(静态摆平面;"CYCLE800 只做 3+2 定位,联动需 TRAORI") | 官方手册:Job Planning ID 109748381(https://support.industry.siemens.com/cs/mdm/109748381?c=71120275851&lc=cs-cz );3→5 轴变换手册 ID 109248664(https://support.industry.siemens.com/cs/document/109248664 ) |
| **Heidenhain** | **FUNCTION TCPM**(option 9,M128 改进版,配 M129):`FUNCTION TCPM F TCP AXIS POS PATHCTRL AXIS`,随后 `L A+ B+ C+ R0 F…`;控制按最慢轴**自动收缩进给** | **PLANE SPATIAL**(`PLANE SPATIAL SPA+ SPB+ SPC+ MOVE DIST… F…`)、CYCLE 19(需 kinematics 描述 option 8) | TNC7 手册 PDF:https://content.heidenhain.de/doku/tnc_guide/pdf_files/TNC7/81762x-17/bhb/1358773-21.pdf ;官方页 https://www.heidenhain.com/industries/machine-tool/5-axis-machining |
| **HAAS (NGC)** | **G234 TCPC**(4/5 轴联动轮廓,需工作偏置 + 同行 H;**快移回转不保刀尖**) | **G254 DWO**(动态工件偏置,3+1/3+2 定位,"not for simultaneous",需 MRZP 标定;G255 取消);**G234 与 G254 互斥** | 官方页(已抓取):https://haascnc.com/service/codes-settings.type=gcode.machine=mill.value=G234.html 、…G254.html ;总览 https://haascnc.com/productivity/control/dwo-tcpc |
| **Mitsubishi M800/M80** | G43.4=刀心点控制 / G43.5=刀切点控制 | — | 官方手册:https://www.mitsubishielectric.com/dl/fa/document/manual/cnc/ib1501278/ib1501278engd.pdf |
| **LinuxCNC** | **无通用原生 RTCP** → 自定义 kinematics 组件:内置 `xyzbc-trt-kins`/`xyzac-trt-kins`(摇篮转台)、`5axiskins`(桥式)、`maxkins`(摆头 B+转台 C);自写 = 复制 `userkins.comp` 用 `halcompile` 编译实现 `kinematicsForward/Inverse`;`switchkins` 支持运行时 TCP/非 TCP 切换(如 xyzbc-trt ↔ trivkins) | 同左(定位态可用 trivkins) | https://linuxcnc.org/docs/html/motion/kinematics.html 、https://linuxcnc.org/docs/html/man/man9/kins.9.html 、trivkins 源码 https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/kinematics/trivkins.c |
| **grbl / grblHAL** | **不支持**:grbl 仅 3 轴(https://github.com/grbl/grbl/issues/1001 );grblHAL 有 A/B/C 度制回转轴(https://github.com/grblHAL/core/issues/333 )但**无运动学变换/RTCP**(由无变换器架构推断,部分未验证) | 多轴只能由 CAM/post 预变换后输出轴坐标 | —— |

### 23.4 进给与路径处理

| 模式 | 代码 | F 含义 | 场景 |
|---|---|---|---|
| 每分进给 | G94 | 路径长度/min;**RTCP 开启时 = 刀心速度**(五轴首选) | 常规 |
| 逆时间 | G93 | F = 1/T(每块必须给 F) | 无 RTCP 的联动/回转主导段;有 RTCP 时基本不用 |
| 度/分 | DPM | 回转轴 deg/min | 纯回转/4 轴 |
| 每转 | G95 | mm/rev | 钻/攻 |
| TCPM | Heidenhain F TCP | 刀心进给,按最慢轴自动收缩 | TCPM 激活时 |

(G93 语义与取舍:https://www.haas.co.uk/resources/better-techniques/control-features/inverse-time-feed-mode-g93 、https://www.emastercam.com/forums/topic/103548-choosing-between-inverse-time-g93-and-feed-per-minute-g94 ;TCPM 收缩:https://www.ccsoftcz.com/blog/3-2-vs-5-axis-cnc-working-planes-tcp ,该文已抓取,是 3+2/TCP 对照的优质二手来源)

- **插补与误差**:控制器对 NC 块线性插补各轴 + RTCP 变换;**线性插补回转轴 ⇒ 刀轴方向非线性误差,RTCP 不能消除**(https://www.mdpi.com/2075-1702/14/11/1263 ;实时双通道插补:Tajima et al. 2019, https://www.sciencedirect.com/science/article/abs/pii/S0890695519302007 );
- **五轴双重弦差容差**(自研离散器):刀心弦差 ‖P_n − 插补点‖ 与 **刀轴角差**(相邻离散点夹角)分别与容差比较,取更密者;
- **回转轴 wrapping**:绝对编程按单圈最短路径(Siemens DCB 语义:"rotary axis is moved along the shortest path to the absolute position within one revolution",https://assets.new.siemens.com/siemens/assets/api/uuid:a3164c91-5adf-4f70-b5a8-bdabc13116d4/NCSIE-SP02-20.pdf );叶轮类 >360° 连续回转需 post **解缠**(累计角输出,Δ>180° 时 ±360° 修正);限位无连续回转时只能 unwind 整机回退,应报警而非静默(https://www.practicalmachinist.com/forum/threads/dealing-with-singularities-in-5-axis-machining-using-tcp-traori.263079 );
- **奇异区**:table-table 在 B=0(刀轴竖直)时 C 完全不定(gimbal)→ 离散时对 C 用"保持上一角/最小增量"策略并告警,或工艺上摆离奇异姿态;head-head 满摆角类似(gimbal 头验证方法:https://industrialmonitordirect.com/blogs/knowledgebase/rtcp-kinematics-configuring-5-axis-gimbal-cb-and-ab-heads )。

### 23.5 RTCP 开 / 关双模式输出

- **RTCP 关(post 预变换)**:post 内做 §23.2 IK,直接输出各轴坐标 `X.. Y.. Z.. B.. C..`;程序**绑定**枢轴距/刀长/装夹,任一变更需重算;进给用 G93 或按合成轴速度换算;head-head 换刀即全程序漂移;数值示例:刀尖 X100 Y50 Z20 B25 C40 ⇒ 输出 X93.284 Y47.521 Z31.846(https://www.ccsoftcz.com/blog/3-2-vs-5-axis-cnc-working-planes-tcp );
- **RTCP 开(控制器解算)**:post 输出 TCP 坐标 + 刀轴矢量/角度 + G43.4(TCPM/TRAORI),控制器用标定运动学 + 当前 H 实时求解;改刀长/工件偏置只改偏置值,程序不变;
- 对应关系:**CYCLE800 : TRAORI = G68.2 : G43.4 = PLANE : TCPM**;
- 样例(G68.2 + G43.4,结构经多源交叉):

```nc
N10 G0 G90 G54 B30. C45.        ( 预定位回转轴 )
N20 G68.2 X0 Y0 Z0 I0 J30 K0    ( 定义倾斜面 )
N30 G53.1                        ( 刀对准法向, 3+2 )
N40 G43.4 H10                    ( RTCP ON, H=全长 )
N50 G01 X10. Y5. Z-2. F1000.     ( TCP 联动, F=刀心速度 )
...
N90 G49                          ( RTCP OFF )
N100 G69                         ( 撤倾斜面 )
```

### 23.6 kinematics 模块设计(Rust)

```rust
trait MachineKinematics {
    fn ik(&self, tcp: Vec3, axis: Vec3, tool_len: f64) -> Result<MachineAxes, KinError>;
    fn fk(&self, axes: &MachineAxes, tool_len: f64) -> (Vec3 /*tcp*/, Vec3 /*axis*/);
}
struct RotaryJoint { letter: char, order: u8, center: Vec3, dir: i8,
                     range: (f64, f64), continuous: bool }
enum OutputMode { RtcpOn /*G43.4+矢量+TCP*/, RtcpOff /*本地 IK+轴坐标+G93*/ }
```

- 内置三个闭式实现:`BCTableTable` / `ACHeadHead` / `BHeadCTable`(§23.2 公式);预留通用串联链数值求解器(齐次变换迭代)覆盖非正交枢轴;
- `OutputMode` 与控制器档案(fanuc/heid/siemens/haas/linuxcnc 字典:RTCP 指令对/取消码/平面指令/进给模式)**解耦**;grblHAL/无 RTCP 控制器自动落到 RtcpOff;
- 奇异与解支:|sin(tilt)|<ε 冻结 C(最小增量)+告警;双解按"距上一角最近 + 行程内"选;回转角解缠器(累计连续角);
- 测试:FK∘IK 往返单元测试 + §23.5 样例程序做黄金对照;运动学总文档:https://linuxcnc.org/docs/html/motion/kinematics.html 。

---

## 24. 五轴与 3+2 刀路、碰撞与仿真

### 24.1 3+2(定位五轴)工作流

- **CAM 侧**:Tool Orientation/倾斜帧分配 + 既有 3 轴策略**复用**(Fusion Tool Orientation:https://help.autodesk.com/view/fusion360/ENU/?guid=MFG-TOOL-ORIENTATION-OVERVIEW ;3+2 教程:https://www.autodesk.com/learn/ondemand/tutorial/3-2-orientation-definition );hyperMILL 官方明示 "3+2, automatic indexing and 5-axis simultaneous machining" + 全自动碰撞规避,并支持给 Z-level/轮廓/等距/残料等既有策略附加 5 轴定位(https://www.openmind-tech.com/en-us/cam/5-axis-milling ;策略册 https://www.openmind-tech.com/fileadmin/user_upload/pdf/cam/strategies/bro-cam-strategies-en.pdf );
- **自动定方位算法本质**(自研推导,两家官方描述佐证):候选方位集(工件法向聚类 / 可视性锥)→ 每方位可达性 + 碰撞过滤(摆角限位、刀柄/夹具 vs 工件)→ 方位图分块(减少换向)→ 每块内复用三轴刀路 → 按帧输出;
- **仿真侧 —— 多方位 Z-map 合并 tri-dexel**(真实出处:**Inui, Chen, Umezu**, "Visualization of 3+2 Axis Machining Result by Combining Multiple Z-map Models", *Computer-Aided Design and Applications*(CAD&A)19(4):825–837, 2022, DOI 10.14733/cadaps.2022.825-837,**全文已读**:https://cad-journal.net/files/vol_19/CAD_19(4)_2022_825-837.pdf ;会议版 http://www.cad-conference.net/files/CAD21/CAD21_88-92.pdf ;后续:Matsukawa & Inui, Triple-Dexel Integration of Multiple Z-Map Models, *IJAT* 2026)——每方位独立 Z-map 去除后 dexel 化布尔并集成 tri-dexel;**注意:该文只做加工结果可视化,无与 CAD 设计面的偏差计算章节**(全文核验),偏差方案为本报告自研综合(§24.5/§24.5.8);
- **碰撞检查要点**:①静态:摆到位后刀柄/夹具(旋转体)vs 该方位剩余毛坯高度场;②动态:**换方位的回转行程**中刀具扫过工件(3+2 典型坑);③换方位快移路径;
- 参照:CAMotics 官方自述 "**No 5-axis simulation**"、无刀柄/夹具碰撞(https://camotics.org )——三轴引擎结论不可外推,柄碰撞必须单独做。

### 24.2 五轴连续刀路(面向 STL 输入)

| 策略 | 定义 | 参照 |
|---|---|---|
| 固定刀轴(tilted plane) | 刀轴在工件系恒定,仅平动 | hyperMILL tilt strategies(上) |
| 平行曲面/曲线 | 刀轴平行参考面/曲线族 | Mastercam Multiaxis(https://www.mastercam.com/about/media/mastercam-multiaxis-machining ) |
| 垂直曲面(法向)+ lead/lean | 刀轴跟随法向,前倾/侧倾角控制 | Mastercam 2027 multiaxis 博客(https://www.mastercam.com/community/blog/multiaxis-in-mastercam-2027-smarter-toolpaths-smoother-motion-greater-control ) |
| **Swarf(侧刃)** | 圆柱刀侧刃贴壁,刀轴=壁面切向 | FeatureCAM Swarf 文章(https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-create-a-four-axis-toolpath-using-Swarf-finish-in-FeatureCAM.html ) |
| Flowline | 沿曲面 UV 流线 | 同上 |
| Point-based(点铣) | 刀位=点集+刀轴,逐点规划 | Sun 2016 向量场法(下) |

- **STL 网格五轴文献**:Sun 等, "Smooth tool path generation for 5-axis machining of triangular mesh surface with nonzero genus", *CAD* 2016(83 引;https://www.sciencedirect.com/science/article/abs/pii/S0010448516300501 )——向量场 + 流线,与我们 STL 输入最贴合;Dang 2020 网格 pattern 检测;MDPI 2026 端到端(https://www.mdpi.com/2673-9909/6/3/35 );
- **刀轴优化/平滑/奇异规避**:Wu, "Accurate Detection and Smoothness-Oriented Avoidance of Singularities for 5-axis CNC Machining", *CAD* 2024(https://www.sciencedirect.com/science/article/abs/pii/S0010448523001847 );Wan 等, "Singularity Avoidance…Geometrical Constraints", *IJMTM* 2018(一点一映射投影法);SO(3)-Based Orientation Smoothing, *J. Manuf. Processes* 2026(https://www.sciencedirect.com/science/article/abs/pii/S1526612526003282 );C-Space 刀轴优化(https://link.springer.com/article/10.1007/s00170-016-8849-0 );四元数 slerp 为帧插值工程标准做法(无单一权威论文);
- **碰撞检测(任意姿态)**:核心参考 **Ilushin, Elber, Halperin, Wein, Kim, "Precise Global Collision Detection in Multi-Axis NC-Machining", *CAD* 37(9):909–920, 2005, DOI 10.1016/j.cad.2004.09.018**(PDF:https://gershon.cs.technion.ac.il/papers/nc_5axis_sim.pdf )——把刀杆/柄/夹具表示为**随刀轴扫掠的平面片(swept planes)**与工件多边形求交,实现全局碰撞检测与规避,自研首选思想;SDF 路线可行性由 Dambly 系列(下)与 Ma 2021 distance-field CWE 佐证;
- **过切 gouging**:**CC 点(刀触点,在工件面上)≠ CL 点(刀位点,在刀轴上)**;Rao & Sarma, "On local gouging in five-axis sculptured surface machining", *CAD* 2000(https://www.sciencedirect.com/science/article/abs/pii/S0010448599001050 );Kim & Elber, "Precise gouging-free tool orientations for 5-axis CNC machining"(https://gershon.cs.technion.ac.il/papers/two-contact-tool-path.pdf );
- **连接移动(linking)**:Chichell 等, "Collision-free Tool Motion Planning for 5-Axis CNC", *CAD* 2024(https://www.sciencedirect.com/science/article/abs/pii/S0010448524000526 );回转角最短路径优化(Munlin & Chok, Five-Axis Tool Path Optimization Using Rotations);工程语义:沿刀轴回退至安全、换向取最短角等价解 + 安全抬刀。

### 24.3 五轴材料去除仿真

- **Z-map 为何不够**:单向 Z-map 隐含"刀轴≈Z",侧刃(swarf)、大摆角、悬垂底面无法表达 → 五轴需 **tri-dexel 或体素**;
- **tri-dexel 为五轴基准表示**:Dambly 等, "Tri-dexel-based cutter-workpiece engagement: computation and validation for virtual machining operations", *IJAMT* 2023, DOI 10.1007/s00170-023-10950-z(全文页:https://orbi.umons.ac.be/handle/20.500.12907/45647 )——工件=tri-dexel、刀具=扫掠体三角网格,"relative closest triangle algorithm" 求 CWE 与切屑厚度;Procedia CIRP 版 https://www.sciencedirect.com/science/article/pii/S2212827122003924/pdf ;2025 年对比基准以 tri-dexel 为精度/效率最优参照(https://www.mdpi.com/2075-1702/13/1/69 );
- **扫掠体推广**:Weinert 等, "Swept volume generation for the simulation of machining"(https://www.sciencedirect.com/science/article/abs/pii/S0890695503003158 );Abdel-Malek 综述;侧刃包络解析(ASME JMSE 134(2):021010)。**自研要点(推论)**:采样 + 并集近似在五轴仍成立,但必须在**机床关节空间**插值(先 IK 到 XYZAC 再插值,而非对 CL 点任务空间 slerp),否则包络失真;误差来源 = 姿态插值空间不一致 + 大摆角快变段采样不足 + 单向深度更新对侧向切削失效(必须三向 dexel);
- **GPU voxel 五轴**:Schnös et al. 2021(*IJAMT* 115:275–289, DOI 10.1007/s00170-021-07001-w)**摘要明确覆盖五轴铣削**("five-axis milling simulator, combining polygonal and voxel representations");先例 Abecassis 2015 CUDA(https://www.sciencedirect.com/science/article/abs/pii/S016636151500041X );
- **商业标杆与机床碰撞需求清单**(VERICUT 官方指南:https://vericut.com/resources/guides/g-code-simulator ;NCSIMUL:https://hexagon.com/products/ncsimul ):①轴树(运动链)定义与正逆解;②部件几何集{主轴头、滑枕、工作台、夹具、刀柄、刀杆、刀、工件、毛坯};③碰撞对全枚举(排除相邻连接对);④行程/摆角限位与超程报告;⑤G0 快移入料检测;⑥near-miss 最小间距阈值;⑦逐段时间插值(考虑 F/G0)而非仅端点。

### 24.4 能力分级与工艺选择

| 级别 | 内容 | 关键算法 | 难度/依赖 |
|---|---|---|---|
| **A:3+2 定位** | 倾斜帧 + 三轴刀路复用;方位自动划分;每方位 Z-map→tri-dexel 合并;柄/夹具静态碰撞;G68.2/CYCLE800 后处理(双模式) | 法向聚类、旋转体-高度场求交、Z-map/dexel 转换 | **低-中**(三轴引擎增量) |
| **B:五轴连续** | 固定刀轴/法向(+lead/lean)策略;slerp/奇异规避;Ilushin 扫掠平面片或 SDF 碰撞;tri-dexel 五轴去除;关节空间采样扫掠;GPU voxel 加速 | 向量场流线、四元数插值、tri-dexel 更新 | **中-高** |
| **C:高级** | swarf/flowline;完整机床碰撞(轴树 DSL/全对/超程/near-miss/快移入料);linking 规划;CWE | 包络解析、C-space 规划、碰撞对管理 | **高**(依赖 B 全部) |

- **3+2 够用的情形**:平面/棱柱特征、中等复杂度多面、批量件经济性、**旋转轴慢的机床**(联动时旋转轴跟不上 → 劣质表面)(https://www.rapiddirect.com/blog/32-vs-5-axis-machining );**必须五轴联动**:叶轮/叶片/自由曲面/短刀刚性/消接痕(https://www.methodsmachine.com/blog/what-is-5-axis-simultaneous-contouring );
- **验证流程**(VERICUT 模式归纳):刀路 → 后处理 → 加载机床定义(轴树+行程)→ 材料去除仿真 → 全对碰撞/超程/near-miss 报告 → 放行。

### 24.5 五轴模型差异对比算法(2026-11-07 追加)

核心原则:**所有"依赖观察方向"的环节换成方向无关表示**——对比原语(点-三角形精确距离、距离场)本就方向无关,要换的是三样:毛坯表示(Z-map → tri-dexel/体素/SDF)、采样策略(Z 射线 → 表面参数化采样)、归因能力(last-touch 推广)。精度边界与"找全→量准→标可信"总原则见 §9.8。

#### 24.5.1 表示层(方向无关的基础)

- **设计模型 → 窄带 SDF**:网格化设计模型体素化,窄带内精确有符号距离,**符号用广义环绕数**(对仿真产出的脏网格鲁棒,§9.2);一次构建,任意方向 O(1)~O(log n) 查询——五轴对比的中枢;
- **加工结果表示**:tri-dexel(三族射线材料区间)或体素/SDF(§24.3 路线);3+2 场景由多方位 Z-map 合并成 tri-dexel(§24.1,Inui 2022);
- 文献佐证(五轴验证用 dexel/tri-dexel 为成熟路线;**全文级核验见 24.5.8**):BSP-dexel 五轴扫掠体(https://www.researchgate.net/publication/345463142_Swept_volume_evaluation_using_the_BSP-dexel_representation_for_the_5-axis_CNC_machining_simulation ,摘要级);Sun 等 2018《Geometric simulation of 5-axis hybrid additive-subtractive manufacturing》(DOI 页 https://link.springer.com/article/10.1007/s00170-018-2577-6 ,**未获全文**);五轴三重 dexel 仿真系统(*JJSPE* 76(3):361, 2010, https://www.jstage.jst.go.jp/article/jjspe/76/3/76_3_361/_article/-char/en );US5710709A "NC milling simulation and dimensional verification"(dexel 尺寸验证与五轴刀表示难点,https://patents.google.com/patent/US5710709A/en );方法论源头 = Jerard & Drysdale 的 dexel/Z-buffer NC 验证体系(§8.1);**偏差色图可引先例:Huang & Oliver, "NC milling error assessment and tool path correction", SIGGRAPH 1994(https://doi.org/10.1109/192161.192231 )、Saito & Takahashi, "NC machining with G-Buffer method", SIGGRAPH 1991(https://doi.org/10.1145/122718.122741 )**。

#### 24.5.2 存在性检测(找全,无采样盲区)

- **逐单元差分场**:共享网格表面窄带逐单元比较 `sign_stock(p)` vs `sign_design(p)`(或 E_stock 与设计 SDF 同点比较),符号翻转连通域 = 差异区域候选(§9.8 ①);
- **dexel 区间 vs 设计 SDF 穿越比较**(tri-dexel 专用,高效):每条 dexel 射线上取设计 SDF 的穿越区间,与材料区间做**区间代数差** → 差出的子区间即该射线上的过切/余量段;三族结果并集成区域——等价于把三轴"逐列比较"推广为"三向逐射线比较"。

#### 24.5.3 量化(量准)

- **表面参数化双面采样**(替代 Z 射线采样,方向无关、无偏):
  - 设计面侧:面积加权蒙特卡洛采点 → 每点查毛坯 SDF/dexel → "设计面各处留了多少";
  - 加工面侧:tri-dexel 重建网格(Ren/Zhu 2008 方法,§8.5)上采样 → 对设计网格 BVH 做 Eberly 最近点 + 环绕数符号 → "加工面上哪里过切"(能发现设计面采样看不到的过切);
  - 双向分开统计(结果不对称,§9.3);
- **双轨度量**:法向投影 `δ=(P−Q)·n` 做过切/余量分类;欧氏距离做 Hausdorff 统计——掠射面上两者差 1/cosθ 倍,五轴侧壁必须分清;
- **符号置信度**:环绕数 |w|≠1 的带标"存疑",不硬分类。

#### 24.5.4 区域精化(精确到亚单元)

- **公差带等值面**:对差分场 D(p) 提取 **D = −t_g 与 D = +t_s 的 iso-surface**(marching cubes,直接复用仿真引擎的 MC)——公差带边界面从"色块格子"变成连续曲面;
- **跨域二分细化**:对连通域边界跨域单元做二分/沿场梯度步进 → 亚单元边界定位;
- **原始/滤波双视图**:五轴刀痕各向异性更严重(lead/lean 使 cusp 沿进给与横向形状不同、swarf 侧壁为规则纹理)——低通/包络后的形位误差视图是色图有工程意义的前提。

#### 24.5.5 归因(五轴刚需)

- **last-touch 推广到 dexel/体素**:仿真时每条 dexel 区间端点、每个体素面记录"时间戳 + move id + op + 刀具"(Z-map last_touch 的推广);检出过切域后,对域内单元取 last-touch 直方图 → **直接回答"哪个操作/哪把刀/哪段刀路切坏的"**;
- **3+2 方位归因**:每方位 Z-map 记录方位 id,合并 tri-dexel 时继承 → 过切区域按装夹方位定位。

#### 24.5.6 特殊几何与配套

- **掠射角面**(刀轴与面夹角小):误差被拉长、分辨率要求提高 → 按 1/cos(入射角) 加权提高采样密度,报告标注"掠射区,置信度降级";
- **倒扣/凹陷**:只有距离场/三向 dexel 可见(任何单向 Z-map 均盲)——这正是五轴换表示的主要理由;
- **near-miss 与快移碰撞**:与材料偏差并行的独立验证轨(Ilushin 扫掠平面片,§24.2/§24.3 需求清单),最终报告合并输出;
- **实测数据对接**(在机测量/首件检验):探针/扫描点云 → 同一管线(点 → 设计 SDF/BVH 查询),先 ICP 对齐 + G54 偏置换算,报告注明"坐标系直比 / best-fit 后"(§9.6)。

#### 24.5.7 落地对应

| 里程碑 | 验证能力 |
|---|---|
| **M6-A(3+2)** | 方位内 Z-map 逐列对比 + 合并 tri-dexel 整体色图 + 静态柄碰撞——区域可见,垂直壁盲区仍在(需标注) |
| **M6-B(五轴连续)** | 设计 SDF + 逐单元差分场/dexel 区间比较(无盲区)+ 双面采样 + 环绕数置信度 + last-touch 归因——**找全 + 量准 + 可归因** |
| **M6-C(高级)** | 公差带等值面 + 二分细化 + 掠射自适应采样 + near-miss 并轨报告 + 实测点云接入(ICP)——亚单元边界 + 完整验证报告 |

能力边界(诚实声明):"精确"来自 **方向无关表示 + 逐单元存在性检测 + 等值面细化** 的三层组合,而非某个更聪明的采样算法;有效下限 = max(仿真分辨率, 网格细分容差),报告必须随结果声明(§9.8)。

#### 24.5.8 论文全文级修正与性能工程(2026-11-07 补查)

**Inui 2022 全文已读**(正确题目 "Visualization of 3+2 Axis Machining Result by Combining Multiple Z-map Models", DOI 10.14733/cadaps.2022.825-837,开放获取已下载):
- 合并算法:每主轴方向 i → 变换 T_i 使刀轴平行 Z → Z-map 仿真 → 转闭合多面体 → **ray 算法(Menon et al. 1994:射线-多边形求交、交点排序、奇偶连线成 dexel)转 triple-dexel** → 逆变换回原系 → 布尔交/差在 dexel 上退化为同线线段求交;
- 性能工程:5000×5000 Z-map → **>5000 万三角形**;二叉 AABB 层级(按网格交替 x/y 划分,叶 ≤4×4 格)加速射线-三角形求交;复杂件 3+2 仿真数分钟级(i7+RTX2080S),瓶颈在多面体→triple-dexel 转换;**其文献[14](Inui 2021, CAD&A 18(4):786-798, DOI 10.14733/cadaps.2021.786-798):GPU RT core dexelization,数百万三角形 → 1 s**;
- **明确声明:该文只做加工结果可视化,无与 CAD 设计面的偏差计算章节** → §24.5 的偏差方案是本报告的自研综合,由已验证组件拼装(ray 法定位设计面 + RCT/PinMesh 点包含 + 法向投影为主/欧氏 fallback + G-buffer 式逐像素色图);色图先例:Huang & Oliver 1994(https://doi.org/10.1109/192161.192231 )、Saito & Takahashi 1991 G-Buffer(https://doi.org/10.1145/122718.122741 );
- Inui 2019(全文已读,https://cad-journal.net/files/vol_16/CAD_16(1)_2019_89-102.pdf ):CEF 提取;GPU 两级并行(多刀位姿态相互独立 + 逐 dexel 减除相互独立)——并行划分依据。

**Dambly 组(UMONS)Procedia 两篇全文已读**(IJAMT 2023 未获全文,ORBi 请求制):
- Procedia CIRP 107:1059-1064 (2022)(开放镜像已抓:https://orbi.umons.ac.be/bitstream/20.500.12907/44389/1/1-s2.0-S2212827122003924-main.pdf ):刀具=沿轴切片堆叠;**平面扫掠法**(dexel×三角形直接求交,快:16–39 s / 3 s 加工)与**二次扫掠法**(Hermite 插值 + Newton 求根,准:2081–6145 s);**未变形切屑厚度与求交解耦**(Hermite 重建料前缘)→ 可容忍更粗 dexel 分辨率;
- Procedia CIRP (CMMO 2023)(开放镜像已抓:https://orbi.umons.ac.be/bitstream/20.500.12907/47947/1/PROCIR_CMMO_2023_Dambly.pdf ):dexel 更新三态(**缩短/分裂/删除**,先做点包含测试);点包含用 **RCT(判定三角形预处理,Liu et al., *CAD* 42(12):1143-1150, 2010)** 或 PinMesh(Magalhães et al., *Computers & Graphics* 58, 2016);**dexel 子集(subsets)包围盒布尔预筛 + 每子集一线程并行**(与八叉树粗扫同构);实验级偏差:CMM 点云 vs dexel 逐面对比,出入刀区 ~400μm @ ddex=100/100/50μm——**属实验对比,非 dexel vs CAD 设计面的射线级算法**。

**窄带与数据结构(自研依据)**:
- **Narrow band**:Adalsteinsson & Sethian 1995, "A Fast Level Set Method for Propagating Interfaces", *J. Comput. Phys.* 118(2)(https://dl.acm.org/doi/abs/10.1006/jcph.1995.1098 )——只在界面 ±k 格内更新,~10× 加速;窄带外移后 reinitialize;Sethian 综述页 https://math.berkeley.edu/~sethian/2006/History/chart.overview.narrowband.html ;
- **VDB(OpenVDB)**:Museth 2013,**正确题目 "VDB: High-Resolution Sparse Volumes with Dynamic Topology"**, *ACM TOG* 32(3)(https://dl.acm.org/doi/10.1145/2487228.2487235 ;overview 实抓 https://www.openvdb.org/documentation/doxygen/overview.html ):4 层 B+树(5/4/3),叶=8³ 稠密块 + 激活位掩码,常值子树折叠为 tile;窄带=表面两侧各约 3 体素(真实距离只存叶层);ValueAccessor 缓存 ≈3×;**每线程独立 accessor(写共享不安全)**;**注意 VDB 13.x 已移除 out-of-core delayed loading**(https://www.openvdb.org/documentation/doxygen/changes.html )→ 自研外存方案不指望该机制;
- **粗到细**:Frisken et al., "Adaptively Sampled Distance Fields", SIGGRAPH 2000(MERL TR2000-15:https://www.merl.com/publications/docs/TR2000-15.pdf )——八叉树按场变化率自适应加密,"粗扫 + 疑异区细化"的直接文献依据;先例:ERdim & Sullivan 复合 ASDF 五轴 CWE(*Procedia CIRP* 8:438-443, 2013, https://doi.org/10.1016/j.procir.2013.06.130 );
- **GPU**:JFA(jump flooding)距离传播(Rong 博士论文:https://www.comp.nus.edu.sg/~tants/jfa/rong-guodong-phd-thesis.pdf ;RTSDF:https://arxiv.org/abs/2210.06160 )+ RT-core dexelization(Inui 2021,1 s/百万三角形量级);CPU 确定性回退保留。

**自研落地规格**:①数据结构:4 层树(建议 4³/8³/16³)+ 叶 8³ 稠密块 + tile 折叠 + 线程本地访问器;窄带 ±3 体素;②对比流程:Frisken 式粗扫 → 包围盒布尔预筛(Dambly 式)→ rayon 每块一线程精算;③滤波链(§9.10)+ GD&T 评定(§9.9)消费同一偏差场;④GPU 两级(JFA + RT-core)为加速项,CPU 为主路径。

---

## 25. 来源汇总

**格式与标准**:STL — https://en.wikipedia.org/wiki/STL_(file_format) ;ISO 10303 — https://en.wikipedia.org/wiki/ISO_10303 ;STEP-File — https://en.wikipedia.org/wiki/STEP-file ;B-rep 实体 — https://www.steptools.com/stds/stp_aim/html/t_manifold_solid_brep.html 、https://www.steptools.com/stds/stp_aim/html/t_tessellated_solid.html ;NIST 测试模型 — https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0 、https://www.nist.gov/document/nist-pmi-step-files

**G-code 规范**:LinuxCNC — https://linuxcnc.org/docs/html/gcode.html 、https://linuxcnc.org/docs/html/gcode/g-code.html 、https://linuxcnc.org/docs/html/gcode/overview.html 、https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g81 、#gcode:g83 、#gcode:g64 、https://linuxcnc.org/docs/stable/html/config/ini-config.html ;grbl — https://github.com/gnea/grbl/wiki/Grbl-v1.1-Commands 、-Configuration 、-Interface ;grblHAL — https://github.com/grblHAL/core ;Marlin — https://marlinfw.org/docs/gcode/G005.html 、https://marlinfw.org/docs/gcode/G002-G003.html ;HAAS — https://haascnc.com/service/online-operator-s-manuals/mill-operator-s-manual/mill---g-codes.html ;CNCCookbook — https://www.cnccookbook.com/cnc-programming-g-code 、https://www.cnccookbook.com/g04-gcode-pause-dwell ;Fusion CPS 指南 — https://cam.autodesk.com/posts/posts/guides/Post%20Processor%20Training%20Guide.pdf

**刀路算法**:Vatti — https://doi.org/10.1145/129902.129906 ;Clipper2 — https://github.com/AngusJohnson/Clipper2 、https://www.angusj.com/clipper2/Docs/Overview.htm ;偏置综述 — http://fcacciola.50webs.com/Offseting%20Methods.htm 、https://www.tandfonline.com/doi/full/10.1080/16864360.2016.1150718 、Held 项目页 — https://www.cosy.sbg.ac.at/~held/projects/pocket_off/pocket.html ;zigzag — https://uni-salzburg.elsevierpure.com/en/publications/optimization-problems-related-to-zigzag-pocket-machining-2 ;trochoidal — https://www.sciencedirect.com/science/article/abs/pii/S0890695508002332 、https://www.sciencedirect.com/science/article/abs/pii/S0924013616300310 ;spiral — https://www.sciencedirect.com/science/article/abs/pii/S0010448509001031 ;cusp — https://www.machiningdoctor.com/calculators/ball-nose-surface-finish ;进给 — https://www.machiningdoctor.com/glossary/feed-per-tooth/ ;顺逆铣 — https://www.harveyperformance.com/in-the-loupe/conventional-vs-climb-milling ;刀补 — https://www.machinemetrics.com/blog/cutter-compensation ;下刀 — https://www.cnccookbook.com/helical-interpolation-ramp-angle-calculator-best-ways-to-enter-a-cut ;biarc — https://www.sciencedirect.com/science/article/pii/001044859290047E ;opencamlib — https://opencamlib.readthedocs.io/ 、https://github.com/aewallin/opencamlib (References wiki、src/algo)、https://www.anderswallin.net/2014/02/drop-cutter-toroid-edge-test/ 、https://www.anderswallin.net/2010/04/toroidal-drop-cutter/ 、https://www.anderswallin.net/2010/10/adaptive-sampling-drop-cutter/ ;Kiri:Moto — https://github.com/GridSpace/grid-apps 、https://docs.grid.space/kiri-moto/CAM/ops ;CamBam tabs — http://www.cambam.info/doc/plus/cam/HoldingTabs.htm ;CuraEngine — https://github.com/Ultimaker/CuraEngine

**运动规划**:grbl 源码 — https://github.com/gnea/grbl/blob/master/grbl/planner.c 、planner.h、stepper.c、motion_control.c、gcode.c、config.h ;JD 解释 — https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm 、https://reprap.org/forum/read.php?1,739819 ;LinuxCNC 时估 — https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/rs274ngc/time_estimate.cc ;grbl #893 — https://github.com/gnea/grbl/issues/893 ;grbl-sim — https://github.com/grbl/grbl-sim ;jerk — https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained 、https://arxiv.org/abs/2105.04830

**仿真**:Jerard & Drysdale 1989(*Algorithmica* 4(1));Lee & Ko 2002 — https://www.sciencedirect.com/science/article/abs/pii/S0924013602007616 ;CAMotics — https://camotics.org/manual.html 、https://github.com/CauldronDevelopmentLLC/CAMotics ;MC — https://www.cs.toronto.edu/~jacobson/seminar/lorenson-and-cline-1987.pdf 、http://paulbourke.net/geometry/polygonise/ ;SDF — https://iquilezles.org/articles/distfunctions/ ;tri-dexel — https://www.cad-journal.net/files/vol_5/CAD_5(6)_2008_932-941.pdf 、https://cad-journal.net/files/vol_16/CAD_16(1)_2019_89-102.pdf ;GPU voxel — https://link.springer.com/article/10.1007/s00170-021-07001-w ;gsim-rs — https://github.com/navrajkalsi/gsim-rs ;BVH — https://pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies 、https://authors.library.caltech.edu/records/12gr1-2f603 ;Möller–Trumbore — https://dl.acm.org/doi/10.1080/10867651.1997.10487468 、https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf

**差异对比**:Eberly — https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf ;环绕数 — https://igl.ethz.ch/projects/winding-number/ 、https://www.dgp.toronto.edu/projects/fast-winding-numbers ;Metro — https://onlinelibrary.wiley.com/doi/10.1111/1467-8659.00236 ;CloudCompare — https://www.cloudcompare.org/doc/wiki/index.php/Cloud-to-Mesh_Distance 、https://www.cloudcompare.org/doc/wiki/index.php/M3C2_(plugin) 、https://github.com/CloudCompare/CCCoreLib ;MeshLab — http://meshlabstuff.blogspot.com/2010/01/measuring-difference-between-two-meshes.html 、https://pymeshlab.readthedocs.io/en/0.1.8/filter_list.html 、https://github.com/cnr-isti-vclab/meshlab ;Vericut — https://vericut.com/products/auto-diff ;Huang 1994 — https://dr.lib.iastate.edu/bitstreams/2b2403f9-d2b7-4e8b-a3c2-4c4f12260bb9/download ;ICP — https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf ;trimesh — https://trimesh.org/trimesh.proximity.html ;综述 — https://www.researchgate.net/publication/312674708_Survey_on_Simulation_Methods_in_Multi-axis_Machining

**网格修复**:Liepa 2003 — https://www.semanticscholar.org/paper/Filling-Holes-in-Meshes-Liepa/d8a45bce2730f12924b9fc6db7fc013089fd3ce6 、https://github.com/russelmann/hole-filling-liepa ;MeshLab 清洗 — https://github.com/cnr-isti-vclab/meshlab/blob/main/src/meshlabplugins/filter_clean/cleanfilter.cpp

**Rust 生态(状态核验)**:ruststep — https://github.com/ricosjp/ruststep ;step-io — https://github.com/elgar328/step-io ;truck — https://github.com/ricosjp/truck ;Fornjot(已归档)— https://github.com/hannobraun/Fornjot ;opencascade-rs — https://github.com/bschwind/opencascade-rs ;stl_io — https://github.com/hmeyer/stl_io ;nom_stl — https://github.com/fast-radius/nom_stl ;tri-mesh — https://github.com/asny/tri-mesh ;Manifold — https://github.com/elalish/manifold ;csgrs — https://github.com/timschmidt/csgrs ;fidget — https://github.com/mkeeter/fidget ;wgpu — https://github.com/gfx-rs/wgpu ;egui — https://github.com/emilk/egui ;egui-wgpu — https://crates.io/crates/egui-wgpu ;learn-wgpu — https://github.com/sotrh/learn-wgpu ;awesome-wgpu — https://github.com/rofrol/awesome-wgpu ;glam/nalgebra 对比 — https://github.com/bitshifter/mathbench-rs 、https://www.reddit.com/r/rust_gamedev/comments/nfhu18/nalgebraglm_vs_glam ;cargo-dist — https://github.com/axodotdev/cargo-dist ;G-code crates — https://github.com/sameer/g-code 、https://github.com/Michael-F-Bryan/gcode-rs 、https://github.com/birkenfeld/ngc

**补遗来源(第 15~22 章,2026-11-07 核验)**:

- **特征识别**:Joshi & Chang 1988 — https://www.sciencedirect.com/science/article/pii/0010448588900504 ;Vandenbrande & Requicha 1993 — https://dl.acm.org/doi/10.1109/34.250845 ;Han/Pratt/Regli 2000 — https://ieeexplore.ieee.org/document/897789 ;Wikipedia — https://en.wikipedia.org/wiki/Feature_recognition ;Schnabel 2007 — https://www.hinkali.com/Education/PointCloud.pdf ;PCL 区域生长 — https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html ;CGAL Shape_detection(GPL,只读)— https://doc.cgal.org/latest/Shape_detection/index.html ;Meyer 2002 曲率 — https://multires.caltech.edu/pubs/diffGeoOps.pdf ;Qu & Stucker 2005 — DOI 10.1108/13552540510601255 ;Hase 2018 — https://www.cctech.co.in/downloads/publications/2018/complex-hole-recognition-from-cad-mesh-models-ICETSET-2018.pdf ;CAMWorks AFR — https://camworks.com/why-camworks ;Fusion AHR — https://www.autodesk.com/products/fusion-360/blog/automated-hole-recognition-autodesk-fusion-manufacturing ;OpenMill — https://github.com/charlatanshost/OpenMill ;AnvilCAD — https://github.com/swperb/anvilcad ;AAGNet — https://github.com/whjdark/AAGNet
- **切削参数/刀具/装夹**:Sandvik — https://www.sandvik.coromant.com/en-us/calculators-apps/machining-calculator-app ;Kennametal — http://www.kennametal.com/pt/en/resources/engineering-calculators/miscellaneous/speed-and-feed.html ;Guhring — https://guhring.com/Support/Speeds-Feeds-Charts ;G-Wizard 方法论 — https://www.cnccookbook.com/feeds-speeds ;brturn 优化器 — http://brturn.github.io/feeds-and-speeds/instructions.html ;LinuxCNC 工具表 — https://linuxcnc.org/docs/html/gcode/tool-compensation.html ;Fusion 工具库 — https://cam.autodesk.com/hsmtools 、https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-import-a-tool-library-in-Fusion-360.html ;FreeCAD ToolBit — https://wiki.freecad.org/CAM_ToolBit ;ISO 13399 — https://www.sandvik.coromant.com/en-us/knowledge/machining-formulas-definitions/cutting-tool-parameters ;Machine taper — https://en.wikipedia.org/wiki/Machine_taper ;ENCY 刀柄防撞 — https://encycam.com/tech/adaptive-tilting ;MecSoft — https://mecsoft.com/blog/how-to-detect-correct-tool-holder-collisions-in-mecsoft-cam ;Fusion fixture — https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-use-Fixture-attachment-in-Fusion-360-CAM-and-HSM.html ;Vericut 碰撞 — https://help.vericut.com/4_vericut_help/3.1.10_configure_collision_menu ;Fusion WCS — https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/Defining-Work-Coordinate-Systems-in-the-Fusion-360-CAM-Workspace.html
- **传输/探测**:grbl interface.md — https://github.com/gnea/grbl/blob/master/doc/markdown/interface.md ;Interfacing wiki — https://github.com/grbl/grbl/wiki/Interfacing-with-Grbl ;grbl issues #982/#740/#817/#1026/#697 — https://github.com/grbl/grbl/issues/ ;UGS — https://github.com/winder/Universal-G-Code-Sender ;bCNC — https://github.com/vlachoudis/bCNC ;CNC.js — https://github.com/cncjs/cncjs ;serialport — https://lib.rs/crates/serialport ;tokio-serial — https://lib.rs/crates/tokio-serial ;G38.x — https://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g38 ;OpenCNCPilot — https://github.com/martin2250/OpenCNCPilot
- **DXF/SVG**:ezdxf 结构 — https://ezdxf.readthedocs.io/en/stable/dxfinternals/filestructure.html ;DXF R12 官方 PDF — https://damassets.autodesk.net/content/dam/autodesk/www/developer-network/platform-technologies/autocad-dxf-archive/acad_r12_dxf.pdf ;DXF 2012 官方 PDF — https://images.autodesk.com/adsk/files/autocad_2012_pdf_dxf-reference_enu.pdf ;组码镜像 — https://documentation.help/AutoCAD-DXF/WS1a9193826455f5ff18cb41610ec0a2e719-7a6f.htm ;bulge — https://github.com/mozman/ezdxf/discussions/826 ;SVG 1.1 — https://www.w3.org/TR/SVG11/paths.html 、https://www.w3.org/TR/SVG11/implnote.html 、https://www.w3.org/TR/SVG11/coords.html ;libdxfrw(GPL,只读)— https://github.com/LibreCAD/libdxfrw ;dxf crate — https://github.com/ixmilia/dxf-rs ;resvg — https://github.com/linebender/resvg
- **竞品**:Fusion — https://www.autodesk.com/products/fusion-360/overview ;Vectric — https://www.vectric.com/purchase/vcarve 、https://www.vectric.com/purchase/aspire ;EstlCam — https://www.estlcam.de/estlcam.php ;Carbide — https://carbide3d.com/carbidecreate/pro ;MeshCAM — https://www.grzsoftware.com/buy.php ;SprutCAM — https://shop.sprutcam.com/ ;Mastercam — https://www.mastercam.com/how-to-buy/ ;观点类 — https://www.cnccookbook.com/best-cad-cam-software-for-cnc-machining-beginners 、https://www.reddit.com/r/hobbycnc/
- **UI/UX 与工程化**:Fusion Manufacture — https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-BEC5DEA9-AC3E-4FA8-998E-4AE8CD0D0B1E ;AU 讲义 — https://static.au-uw2-prd.autodesk.com/MFG226547-L_Class_Handout_MFG226547L_Fundamentals_of_the_CAM_Workspace_Using_Fusion360_Learn_2D_and_3D_Toolpaths_Kevin_Lee.pdf ;VCarve 手册 — https://docs.vectric.com/docs/V12.5/VCarvePro/ENU/Help/page/main-window/index.html ;EstlCam 手册 — https://www.estlcam.de/manual.php ;egui_dock — https://docs.rs/egui_dock ;custom3d_wgpu — https://github.com/emilk/egui/blob/master/crates/egui_demo_app/src/apps/custom3d_wgpu.rs ;CallbackTrait — https://docs.rs/egui-wgpu/latest/egui_wgpu/trait.CallbackTrait.html ;Undoer — https://docs.rs/egui/latest/egui/util/undoer/index.html ;undo crate — https://docs.rs/undo ;egui-cjk-font — https://crates.io/crates/egui-cjk-font ;FCStd — https://wiki.freecad.org/File_Format_FCStd ;cargo-dist — https://github.com/axodotdev/cargo-dist ;SmartScreen — https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/smartscreen-reputation ;macOS 公证 — https://developer.apple.com/documentation/security/notarizing-macos-software-before-distribution ;wgpu 适配 — https://docs.rs/wgpu/latest/wgpu/struct.RequestAdapterOptions.html ;rust-i18n/fluent — https://crates.io/crates/rust-i18n 、https://projectfluent.org
- **NURBS/刀具接触**:B-spline — https://en.wikipedia.org/wiki/B-spline ;de Boor — https://en.wikipedia.org/wiki/De_Boor%27s_algorithm ;NURBS — https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline ;NURBS Book PDF — https://link.springer.com/content/pdf/10.1007/978-3-642-59223-2.pdf ;Shene 讲义 — https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/single-insertion.html ;Wallin toroid — https://www.anderswallin.net/2014/02/drop-cutter-toroid-edge-test/ ;opencamlib cutters 源码 — https://raw.githubusercontent.com/aewallin/opencamlib/master/src/cutters/ ;geomdl — https://nurbs-python.readthedocs.io ;neco-nurbs — https://github.com/barineco/neco-cad ;TinyNurbs(C++)— https://github.com/caadxyz/TinyNurbs ;BRepMesh — https://dev.opencascade.org/doc/overview/html/visualization__mesh.html ;Piegl & Richard 1995 — https://bojianwu.github.io/gsoc2016/week_6/NURBS_tess.pdf
- **性能基准**:Schnös 2021 — https://link.springer.com/article/10.1007/s00170-021-07001-w ;MecSoft TriDexel — https://mecsoft.com/blog/revolutionizing-cam-simulation-nvidia-gpu-accelerated-tridexel-simulation-in-rhinocam-visualcad-cam ;zmesh — https://github.com/seung-lab/zmesh ;TinyBVH — https://github.com/jbikker/tinybvh ;pyvista-stl — https://github.com/pyvista/pyvista-stl ;OCCT 论坛 — https://occt3d.com/dev/content/load-big-file-stepcontrolreader-cost-too-much-time ;Manifold #383 — https://github.com/elalish/manifold/discussions/383 ;solidean — https://solidean.com/blog/2026/terrain-carve-benchmark/ ;Waszczuk 2020 — https://www.researchgate.net/publication/344028429 ;Anders Wallin 各 benchmark 帖(见第 22 章表内)

**五轴补遗来源(第 23~24 章,2026-11-07 核验)**:

- **运动学/RTCP**:Lee & She 1997 — https://link.springer.com/article/10.1007/BF01350824 ;She & Lee 扩展 — https://www.sciencedirect.com/science/article/pii/S1526612500700211 ;Missouri S&T 论文 — https://scholarsmine.mst.edu/cgi/viewcontent.cgi?article=8096&context=masters_theses ;ccsoft 3+2/TCP 对照 — https://www.ccsoftcz.com/blog/3-2-vs-5-axis-cnc-working-planes-tcp ;Fanuc TCP/G54 行为 — https://industrialmonitordirect.com/blogs/knowledgebase/fanuc-g434-tcp-behavior-with-g54-work-offset-shifts-on-5-axis ;G68.2 — https://zappettiniconsulting.com/wiki/fanuc-tilted-working-plane-g682 、样例 https://www.emastercam.com/forums/topic/94855-g682-program-sample ;Siemens 3→5 轴变换手册 — https://support.industry.siemens.com/cs/document/109248664 、Job Planning — https://support.industry.siemens.com/cs/mdm/109748381?c=71120275851&lc=cs-cz ;Heidenhain TNC7 手册 — https://content.heidenhain.de/doku/tnc_guide/pdf_files/TNC7/81762x-17/bhb/1358773-21.pdf 、5 轴页 https://www.heidenhain.com/industries/machine-tool/5-axis-machining ;HAAS G234 — https://haascnc.com/service/codes-settings.type=gcode.machine=mill.value=G234.html 、G254 — https://haascnc.com/service/codes-settings.type=gcode.machine=mill.value=G254.html 、DWO/TCPC 总览 https://haascnc.com/productivity/control/dwo-tcpc ;Mitsubishi — https://www.mitsubishielectric.com/dl/fa/document/manual/cnc/ib1501278/ib1501278engd.pdf ;LinuxCNC 运动学 — https://linuxcnc.org/docs/html/motion/kinematics.html 、kins(9) https://linuxcnc.org/docs/html/man/man9/kins.9.html 、trivkins 源码 https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/kinematics/trivkins.c ;grbl 3 轴限制 — https://github.com/grbl/grbl/issues/1001 ;grblHAL 回转轴 — https://github.com/grblHAL/core/issues/333 ;回转最短路径(DCB)— https://assets.new.siemens.com/siemens/assets/api/uuid:a3164c91-5adf-4f70-b5a8-bdabc13116d4/NCSIE-SP02-20.pdf ;奇异讨论 — https://www.practicalmachinist.com/forum/threads/dealing-with-singularities-in-5-axis-machining-using-tcp-traori.263079 、gimbal 验证 https://industrialmonitordirect.com/blogs/knowledgebase/rtcp-kinematics-configuring-5-axis-gimbal-cb-and-ab-heads ;5 轴插补误差 — https://www.mdpi.com/2075-1702/14/11/1263 、Tajima 2019 https://www.sciencedirect.com/science/article/abs/pii/S0890695519302007 ;G93 — https://www.haas.co.uk/resources/better-techniques/control-features/inverse-time-feed-mode-g93 、https://www.emastercam.com/forums/topic/103548-choosing-between-inverse-time-g93-and-feed-per-minute-g94 ;Haas UMC-500 — https://haascnc.com/machines/vertical-mills/universal-machine/models/umc-500 ;Pocket NC — https://www.pentamachine.com/all-products/p/pocket-nc-v2-50chb-or-chk ;Tormach — https://tormach.com/machines/mills/1100mx.html
- **刀路/碰撞/仿真**:hyperMILL — https://www.openmind-tech.com/en-us/cam/5-axis-milling 、策略册 https://www.openmind-tech.com/fileadmin/user_upload/pdf/cam/strategies/bro-cam-strategies-en.pdf ;Fusion Tool Orientation — https://help.autodesk.com/view/fusion360/ENU/?guid=MFG-TOOL-ORIENTATION-OVERVIEW 、3+2 教程 https://www.autodesk.com/learn/ondemand/tutorial/3-2-orientation-definition ;Inui 2022(Z-map→tri-dexel)— https://cad-journal.net/files/vol_19/CAD_19(4)_2022_825-837.pdf 、会议版 http://www.cad-conference.net/files/CAD21/CAD21_88-92.pdf ;Sun 2016 网格五轴 — https://www.sciencedirect.com/science/article/abs/pii/S0010448516300501 ;MDPI 2026 — https://www.mdpi.com/2673-9909/6/3/35 ;Wu 2024 奇异 — https://www.sciencedirect.com/science/article/abs/pii/S0010448523001847 ;SO(3) 平滑 2026 — https://www.sciencedirect.com/science/article/abs/pii/S1526612526003282 ;C-Space 刀轴 — https://link.springer.com/article/10.1007/s00170-016-8849-0 ;Ilushin 2005 — DOI 10.1016/j.cad.2004.09.018 、PDF https://gershon.cs.technion.ac.il/papers/nc_5axis_sim.pdf ;gouging — https://www.sciencedirect.com/science/article/abs/pii/S0010448599001050 、https://gershon.cs.technion.ac.il/papers/two-contact-tool-path.pdf ;linking 2024 — https://www.sciencedirect.com/science/article/abs/pii/S0010448524000526 ;Dambly tri-dexel CWE — DOI 10.1007/s00170-023-10950-z 、https://orbi.umons.ac.be/handle/20.500.12907/45647 、Procedia 版 https://www.sciencedirect.com/science/article/pii/S2212827122003924/pdf ;2025 对比基准 — https://www.mdpi.com/2075-1702/13/1/69 ;Weinert 扫掠 — https://www.sciencedirect.com/science/article/abs/pii/S0890695503003158 ;Schnös 2021(五轴 GPU voxel)— DOI 10.1007/s00170-021-07001-w ;Abecassis 2015 — https://www.sciencedirect.com/science/article/abs/pii/S016636151500041X ;CAMotics 限制 — https://camotics.org ;VERICUT — https://vericut.com/resources/guides/g-code-simulator 、https://vericut.com/en-gb/blog/machine-simulation ;NCSIMUL — https://hexagon.com/products/ncsimul ;Fusion 机床定义 — https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-setup-machine-simulation-with-rotary-axes-in-Fusion.html 、Rotary Table Setup Guide https://cam.autodesk.com/posts/posts/guides/Rotary%20Table%20Setup%20Guide.pdf 、Machine Builder https://www.autodesk.com/products/fusion-360/blog/learn-how-machine-builder-works-in-autodesk-fusion ;SprutCAM 运动学 — https://kb.sprutcam.com/SC16manual/en/10125.html ;工艺选择 — https://www.rapiddirect.com/blog/32-vs-5-axis-machining 、https://www.methodsmachine.com/blog/what-is-5-axis-simultaneous-contouring ;**模型差异对比(五轴)** — BSP-dexel 五轴扫掠体 https://www.researchgate.net/publication/345463142_Swept_volume_evaluation_using_the_BSP-dexel_representation_for_the_5-axis_CNC_machining_simulation ;五轴三重 dexel 仿真系统(JJSPE 2010)https://www.jstage.jst.go.jp/article/jjspe/76/3/76_3_361/_article/-char/en ;US5710709A dexel 尺寸验证 https://patents.google.com/patent/US5710709A/en ;Sun 2018 五轴混合制造 tri-dexel 仿真 — https://link.springer.com/article/10.1007/s00170-018-2577-6 (未获全文)

**验证补遗来源(GD&T/滤波/性能,第 9.9~9.10、24.5.8 章,2026-11-07 核验)**:

- **GD&T**:ISO 1101:2017 — https://www.iso.org/standard/66777.html ;ASME Y14.5 — https://www.asme.org/codes-standards/find-codes-standards/y14-5-dimensioning-tolerancing ;ISO 5459:2016 — https://www.iso.org/standard/59391.html ;ISO 15530-3 — https://www.iso.org/standard/53627.html ;NPL GPG130 — https://eprintspublications.npl.co.uk/5924/1/MGPG130.pdf ;ISO/ASME 差异(Humienny 2021)— https://www.mdpi.com/2076-3417/11/17/8269 ;Carr & Ferreira I/II — https://www.sciencedirect.com/science/article/pii/014163599400017T 、…/014163599400018U ;Remez — https://en.wikipedia.org/wiki/Remez_algorithm ;FARO 平面度 — https://www.faro.com/en/Resource-Library/Article/how-to-evaluate-flatness-in-gd-t ;NIST 受限 LS — https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=921834 ;Shakarji & Srinivasan 2023 — https://pubmed.ncbi.nlm.nih.gov/39380595 ;PC-DMIS 轮廓度 — https://docs.hexagonmi.com/pcdmis/2020.2/en/helpcenter/mergedProjects/core/geometric_tolerances/Profile_of_a_Surface.htm 、基准解算 https://nexus.hexagon.com/documentationcenter/en-US/bundle/pcdmis-2022.1-core/page/geometric_tolerances/How_PC-DMIS_Solves_Datums.htm ;geomfitty — https://github.com/mark-boer/geomfitty ;STEPNCpp — https://github.com/johnmichaloski/STEPNCpp ;PolyWorks — https://www.polyworks.com/en-us/products/polyworks-inspector ;Gumbel — https://en.wikipedia.org/wiki/Gumbel_distribution ;Pawlus 2023 — https://pmc.ncbi.nlm.nih.gov/articles/PMC10672465/ ;面轮廓度定义 — https://www.gdandtbasics.com/profile-of-a-surface
- **滤波/表面计量**:ISO 25178-2 — https://www.iso.org/standard/74591.html ;ISO 16610-41 — https://standards.iteh.ai/catalog/standards/cen/5ef95af4-5a8c-4a54-86d7-93071bff8b60/en-iso-16610-41-2015 ;Digital Surf 滤波指南 — https://guide.digitalsurf.com/en/guide-filtration-techniques.html 、https://www.digitalsurf.com/blog/the-delicate-art-of-applying-filters-to-a-surface-in-order-to-prepare-it-for-analysis ;Quinsat 2011 — https://www.sciencedirect.com/science/article/abs/pii/S0007850611000647 ;Raja 2002 分离综述 — https://www.sciencedirect.com/science/article/abs/pii/S0141635902001034 ;Zahouani 2008 — https://www.sciencedirect.com/science/article/abs/pii/S0043164807001962 ;cusp 计算器 — https://www.conceptsnrec.com/blog/calculating-surface-roughness-cusp-height-ball-end-mills
- **验证性能工程**:Inui 2022 全文 — https://cad-journal.net/files/vol_19/CAD_19(4)_2022_825-837.pdf (DOI 10.14733/cadaps.2022.825-837);Inui 2019 全文 — https://cad-journal.net/files/vol_16/CAD_16(1)_2019_89-102.pdf ;Dambly Procedia 2022 全文镜像 — https://orbi.umons.ac.be/bitstream/20.500.12907/44389/1/1-s2.0-S2212827122003924-main.pdf ;Dambly Procedia 2023 全文镜像 — https://orbi.umons.ac.be/bitstream/20.500.12907/47947/1/PROCIR_CMMO_2023_Dambly.pdf ;RCT(Liu 2010,*CAD* 42(12))、PinMesh(Magalhães 2016)——经 Dambly 参考文献表核验;Huang & Oliver 1994 — https://doi.org/10.1109/192161.192231 ;Saito & Takahashi 1991 — https://doi.org/10.1145/122718.122741 ;Adalsteinsson & Sethian 1995 — https://dl.acm.org/doi/abs/10.1006/jcph.1995.1098 ;VDB(Museth 2013)— https://dl.acm.org/doi/10.1145/2487228.2487235 、overview https://www.openvdb.org/documentation/doxygen/overview.html 、changes https://www.openvdb.org/documentation/doxygen/changes.html ;Frisken ADF — https://www.merl.com/publications/docs/TR2000-15.pdf ;ERdim & Sullivan 2013 — https://doi.org/10.1016/j.procir.2013.06.130 ;JFA — https://www.comp.nus.edu.sg/~tants/jfa/rong-guodong-phd-thesis.pdf 、RTSDF https://arxiv.org/abs/2210.06160

**刀路自动生成来源(§5.7~5.10,2026-11-07 核验)**:

- **一键 CAM**:MeshCAM — https://www.grzsoftware.com 、https://www.grzsoftware.com/tut/stl 、https://www.grzsoftware.com/blog/surface-angle-limits 、https://www.grzsoftware.com/blog/quick-flip-toolpath 、https://www.grzsoftware.com/blog/speeds-and-feeds 、https://www.grzsoftware.com/blog/version-7-build-25 ;向导脚本/行为 — https://community.carbide3d.com/t/how-to-add-tools-to-carbide-wizard-in-meshcam/1469 、https://community.carbide3d.com/t/trying-to-get-started/14359 、https://community.carbide3d.com/t/first-attempt-at-aluminum-failed-please-help/6902 ;第三方教程 — https://1c3d.com/howtos/meshcam-cnc-automatic-toolpath-generation-guide ;Kiri:Moto — https://docs.grid.space/kiri-moto/CAM/ 、https://docs.grid.space/kiri-moto/CAM/ops 、https://raw.githubusercontent.com/GridSpace/grid-apps/master/src/kiri/app/conf/defaults.js ;FreeCAM Job — https://wiki.freecad.org/CAM_Job ;MeshMill/pngcam — https://incoherency.co.uk/blog/stories/meshmill.html 、https://github.com/jes/meshmill ;OpenCAMStudio — https://github.com/pr0m1th3as/OpenCAMStudio ;PathCAM — https://github.com/xenovacivus/PathCAM
- **自动参数**:CNCCookbook — https://www.cnccookbook.com/feeds-and-speeds 、https://www.cnccookbook.com/cnc-stepover 、https://www.cnccookbook.com/2-tools-calculating-cut-depth-cut-widthstepover-milling 、https://www.cnccookbook.com/high-speed-machining-speeds-and-feeds ;cusp — https://www.cutterbest.com/pt/ball-nose-end-mill-stepover 、https://engicalchub.com/calculators/ball-nose-surface-finish-calculator ;切深 — https://qfcncmachine.com/cnc-depth-of-cut ;rest — https://docs.encycam.com/ENCYrobot/1/en/727.html 、https://hawkridgesys.com/blog/explaining-rest-machining-in-solidworks-cam
- **CAPP**:Alting & Zhang 1989 — https://www.tandfonline.com/doi/abs/10.1080/00207548908942569 ;Xiao 2023 KG — https://www.sciencedirect.com/science/article/abs/pii/S0278612523001577 ;CAPP-GPT — https://www.sciencedirect.com/science/article/pii/S221384632400066X ;Sandvik 钻削 — https://www.sandvik.coromant.com/en-us/knowledge/drilling ;点钻 vs 中心钻 — https://www.mastercam.com/community/blog/spot-drill-vs-center-drill-what-is-the-difference ;Fictiv 孔加工 — https://www.fictiv.com/articles/drilling-boring-reaming-precision-holes ;公差计算器 — https://www.machiningdoctor.com/calculators/tolerances ;CAMWorks TBM — https://camworks.com/module/tolerance-based-machining/ 、AFR https://camworks.com/blog/unlocking-the-power-of-cad-cam-automation-in-solidworks ;SolidCAM AFRM — https://solidcam.com/cam-modules/afrm ;TSPPC GA(Moon 2002)— https://doi.org/10.1016/S0377-2217(01)00248-8 ;Su 2018 — https://ideas.repec.org/a/spr/joinma/v29y2018i2d10.1007_s10845-015-1109-6.html ;Yun 2013 — https://www.sciencedirect.com/science/article/abs/pii/S0360835211003421 ;Wakhare 2026 setup — https://link.springer.com/article/10.1007/s00170-026-17859-3 ;Ong & Nee — https://www.sciencedirect.com/science/article/pii/S0007850607621829 ;公差驱动混合图 — https://www.academia.edu/54640823/A_hybrid_graph_approach_for_automated_setup_planning_in_CAPP ;Nelaturi 2014 夹具 — https://www.sciencedirect.com/science/article/abs/pii/S0010448513001681 ;Kang & Peng 综述 — https://cad-journal.net/files/vol_5/CAD_5(1-4)_2008_424-433.pdf ;刀具选择综述 — https://link.springer.com/article/10.1007/s00170-024-13823-1 ;STEP-NC — https://en.wikipedia.org/wiki/STEP-NC 、https://www.steptools.com/stds/stepnc 、http://ap238.org/ap238e2/ap238_stepnc_v2.pdf 、https://github.com/johnmichaloski/STEPNCpp ;LLM CAPP — https://www.sciencedirect.com/science/article/pii/S073658452600013X 、制造综述 https://arxiv.org/html/2410.21418v1 ;DRL 工艺路线 — https://www.sciencedirect.com/science/article/abs/pii/S0736584523001631 ;RL 铣削 — https://ieeexplore.ieee.org/iel7/6287639/10005208/10081313.pdf ;ML 参数 — https://asmedigitalcollection.asme.org/tribology/article/147/9/091117/1214624
- **曲面精加工策略(全部经 CrossRef 核验)**:Suresh & Yang 1994 — https://doi.org/10.1115/1.2901938 ;Feng & Li 2002 — https://doi.org/10.1016/s0010-4485(01)00136-1 ;Tournier & Duc 2002/2005 — https://doi.org/10.1007/s001700200019 、https://doi.org/10.1007/s00170-003-2054-7 ;Tournier & Lartigue 2008 全文 — https://www.cad-journal.net/files/vol_5/CAD_5(1-4)_2008_278-286.pdf ;Liu 2012 — https://link.springer.com/article/10.1007/s00170-011-3892-3 ;Kim & Yang 2006 — https://doi.org/10.1007/s00170-004-2372-4 ;Kim T. 2007 — https://doi.org/10.1016/j.cad.2007.01.003 ;Zou 2014 — https://doi.org/10.1016/j.cad.2014.04.006 ;Elber & Cohen 1994 — https://doi.org/10.1016/0010-4485(94)90070-1 ;Ding 2003 — https://doi.org/10.1016/s0010-4485(02)00048-9 ;Lin 2015 综述 — https://www.sciencedirect.com/science/article/abs/pii/S0010448515001013 ;pencil — https://doi.org/10.1007/978-0-387-35392-0_30 、https://doi.org/10.1016/j.cad.2004.09.023 、https://doi.org/10.1016/j.cad.2004.10.003 、https://doi.org/10.1016/j.cad.2005.03.004 ;螺旋 — https://doi.org/10.1016/j.precisioneng.2018.01.013 、https://doi.org/10.1016/s0010-4485(01)00185-3 、https://doi.org/10.1115/1.4039918 ;steep&shallow — https://www.autodesk.com/products/fusion-360/blog/fusion-360-extensions-steep-shallow ;stepover 公式 — https://www.cutterbest.com/pt/ball-nose-end-mill-stepover ;策略定量评价 — https://ijomam.com/wp-content/uploads/2026/09/pp.-239-248_QUANTITATIVE-EVALUATION-OF-CAM-FINISHING-STRATEGIES-FOR-COMPLEX-FREEFORM-SURFACES-USING-A-DIMENSIONLESS-EFFICIENCY-INDEX.pdf

**表面质量评价来源(§9.11,2026-11-07)**:ISO 25178-2 — https://www.iso.org/standard/74591.html (已见滤波补遗);Ni 2024 几何仿真系统 — https://link.springer.com/article/10.1186/s10033-024-01033-5 ;Wang 2025 让刀/跳动修正 — https://www.sciencedirect.com/science/article/abs/pii/S0263224124016555 ;Guo 2026 五轴地形预测 — https://www.sciepublish.com/article/pii/1045 ;(Quinsat 2011、Raja 2002 已见验证补遗来源)

**刀路质量补强来源(§5.11,2026-11-07)**:进给调度 — Erdim & Lazoglu 2006 https://www.sciencedirect.com/science/article/abs/pii/S0890695505001914 、啮合角综述 https://www.researchgate.net/publication/332267083 、Zeng 2025 https://www.sciencedirect.com/science/article/pii/S1000934525001063 、Jacsó 2023 https://link.springer.com/article/10.1007/s00170-023-11506-x 、Kienzle https://www.machiningdoctor.com/glossary/specific-cutting-force-kc-kc1 、Sandvik 内角 https://www.sandvik.coromant.com/en-us/knowledge/milling/milling-inside-corners 、Fusion 拐角降速 https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-control-feed-rate-on-internal-and-external-arcs-in-Fusion.html 、SLD https://ijme.us/cd_06/PDF/IT%20301-050.pdf ;链接 — Plakhotnik & Vaneker 2019 https://www.sciencedirect.com/science/article/pii/S0010448518305761 、Nishat 2023 https://www.mdpi.com/2076-3417/13/12/7385 、TrajOpt https://www.ri.cmu.edu/publications/finding-locally-optimal-collision-free-trajectories-with-sequential-convex-optimization 、Sandvik ramp https://www.sandvik.coromant.com/en-us/knowledge/milling/milling-holes-cavities-pockets/ramping ;方向场 — Lin & Koren 1996 全文 https://www-personal.umich.edu/~ykoren/uploads/Efficient_tool-path_planning_for_machining_free-form_surfaces.pdf 、Kumazawa 2015 PFD https://www.sciencedirect.com/science/article/abs/pii/S0010448515000652 、Zou 2020 https://arxiv.org/abs/2009.02660 (期刊 https://www.sciencedirect.com/science/article/abs/pii/S001044852100083X )、Park & Choi 2000 https://www.sciencedirect.com/science/article/abs/pii/S0010448599000809 、KAIST 2002 https://koasas.kaist.ac.kr/handle/10203/83434 、Hao 2022 分区 https://academic.oup.com/jcde/article/9/5/1585/6656372 、Sun 2022 综述 https://www.sciencedirect.com/science/article/pii/S1000936121002156 ;直纹面 — Crane/Stein 2018 https://www.cs.cmu.edu/~kmcrane/Projects/DiscreteDevelopable/ (DOI 10.1145/3197517.3201303)、Peternell 2004 https://www.semanticscholar.org/paper/293e550c05f3db63febfa08f6831965aa73565f7 、Tsay & Her 2001 https://asmedigitalcollection.asme.org/manufacturingscience/article/123/4/731/449760 、Cao & Dong 2015 https://www.sciencedirect.com/science/article/pii/S1000936115001740 、Steenstrup 2016 https://core.ac.uk/download/pdf/84001537.pdf 、Rajain 2025 https://bird.bcamath.org/bitstream/handle/20.500.11824/1980/MultiPassG1CustomShapedC.pdf 、商业 swarf 对照 PowerMill/Fusion/FeatureCAM/Creo/SprutCAM/hyperMILL(见正文)

**系统误差来源(§9.12,2026-11-07)**:Bærentzen & Aanæs — https://www2.imm.dtu.dk/pubdb/edoc/imm1289.pdf ;OpenVDB MeshToVolume — https://www.openvdb.org/documentation/doxygen/MeshToVolume_8h.html 、FastSweeping https://github.com/AcademySoftwareFoundation/openvdb/blob/master/openvdb/openvdb/tools/FastSweeping.h ;mesh_to_sdf — https://docs.rs/mesh_to_sdf ;TriangleMeshDistance — https://github.com/InteractiveComputerGraphics/TriangleMeshDistance ;亚体素等值面 — https://pubmed.ncbi.nlm.nih.gov/12821034 ;让刀 — https://www.machiningdoctor.com/classroom/endmiils-deflection/ 、https://www.harveyperformance.com/in-the-loupe/tool-deflection-remedies/ 、Dow 2004 https://www.sciencedirect.com/science/article/abs/pii/S0141635903000722 、MERL TR2013-057 https://www.merl.com/publications/docs/TR2013-057.pdf 、五轴补偿 https://link.springer.com/article/10.1007/s00170-015-7793-8 、ASME https://asmedigitalcollection.asme.org/manufacturingscience/article/138/8/081005/376223/ ;其他误差 — 热变形 https://www.sciencedirect.com/science/article/abs/pii/S0890695502002638 、ASME 体素铣削误差 https://asmedigitalcollection.asme.org/manufacturingscience/article/145/1/011005/1146709 、J.JSPE https://www.jstage.jst.go.jp/article/jjspe/84/2/84_175/_article/-char/en

**刀路质量补强 II 来源(§5.12/§5.13/§21.4,2026-11-07)**:桶刀 — MMT https://www.moldmakingtechnology.com/articles/ten-things-you-need-to-know-about-circle-segment-milling 、hyperMILL https://www.openmind-tech.com/en-us/cam/hypermill-maxx-machining/finishing/conical-barrel-cutter 、Kyocera https://www.kyocera-sgstool.com/learn/circle-segment-vs-ball-nose 、Emuge-Franken https://www.emuge-franken-group.com/us/en/circle-segment-cutters---5-axis-milling-tooling/a/Circle%20Segment%20Cutters%20-%205-Axis%20Milling%20Tooling 、GWS https://www.gwstoolgroup.com/circle-segment-end-mills-vs-ball-nose-end-mills-when-to-use-each 、Suzuki 2021 DOI 10.20965/ijat.2021.p0885 、Meng 2014 双切触、包络逼近 DOI 10.14733/cadconfp.2014.186-188 、一体规划 DOI 10.1007/s00170-015-8113-z 、Urbikain DOI 10.1016/j.precisioneng.2018.11.007 ;光顺 — Aerotech https://help.aerotech.com/automation1/Content/Concepts/Corner-Rounding-Functions.htm 、Xiao 2021 https://www.sciencedirect.com/science/article/abs/pii/S0094114X21000653 、Yan 2023 http://www.jamstjournal.com/en/article/doi/10.51393/j.jamst.2023001 、line-arc https://cad-journal.net/files/vol_19/CAD_19(4)_2022_854-867.pdf 、Fusion 双容差 https://forums.autodesk.com/t5/fusion-manufacture-forum/understanding-smoothing/td-p/6636189 、样条弦差 DOI 10.1016/j.jcde.2015.06.002 、DOI 10.1007/s11431-018-9374-6 、控制器对照 https://industrialmonitordirect.com/blogs/knowledgebase/nurbs-vs-supernurbs-fanuc-okuma-siemens-mitsubishi-compared 、Tormach G64 https://tormach.com/machine-codes/set-blended-path-control-mode-g64 ;薄壁 — Harvey 全文 https://www.harveyperformance.com/in-the-loupe/thin-wall-milling-accuracy 、DOI 10.3390/ma13214723 、10.3390/ma15249049 、10.1016/j.proeng.2017.03.117 、10.1504/ijmmm.2023.10061206 、10.4028/www.scientific.net/amr.912-914.1423 、Forbes 指南 https://forbesprecision.co.in/wp-content/uploads/2024/01/Thin-wall-machinng-Guide.pdf

---

*报告完。所有"未验证"标注条目在使用前应先行复核;crate/项目状态以调研日期(2026-11-06/07)为准。*
