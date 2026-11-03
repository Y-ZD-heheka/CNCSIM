//! 设计模型比对：过切（gouge）/ 残留（residual）。
//!
//! 口径（`docs/技术方案-五轴CNC仿真.md` §7.1 与 A6 验收）：
//!
//! | 指标 | 方法 | 精度 |
//! |---|---|---|
//! | 过切/残留**体积** | 体素积分（毛坯场 × 设计 SDF 符号组合） | ± 一个表面层 |
//! | 过切**最大深度** | 解析扫掠点测试（设计面采样点 × 每段 `sd_sweep`） | 浮点级，不经过体素 |
//! | 残留**最大厚度** | 设计气区内材料体素到设计面的距离 + h/2 | ± h/2（0.05 mm 级 @ h=0.1） |
//!
//! ## 为什么过切深度不走体素
//!
//! 体素场的量化档是 3.15 µm（i8），0.01 mm 的过切判定会被量化噪声淹没；
//! 解析测试直接问"设计面上的这个点是否落在刀具扫掠体内"，精度只受
//! 黄金分割收敛限制（典型段长 <1 µm）。体积仍走体素——积分天然是体素的活。
//!
//! ## 残留的口径
//!
//! **设计气区内的全部材料都算残留**，不限工具走廊——设计模型说"这里不该有料"，
//! 它就是多余的，无论程序有没有试图去切。残留最大厚度取"离设计面最远的材料
//! 体素"到设计面的距离：距离场在材料区内单调增，最大值必然落在某个表面上
//! （毛坯表面或切削面），所以不需要显式的表面体素判定；+h/2 把体素中心修正到
//! 表面位置，误差 ±h/2。
//!
//! ## 设计面采样
//!
//! 盒体 6 个面按 `step` 网格采样（面片中心）。步距内的窄过切可能漏检——
//! 加密步距可换，代价是采样点数平方增长。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::{Profile, ToolTable};
use cncsim_voxel::{SparseGrid, sd_sweep, sweep_aabb};

/// 过切判定阈值（mm）。小于它的"过切"与扫掠黄金分割的迭代噪声不可区分
/// （典型段长 <1 µm，200 mm 长段 <5 µm），报出来只会是狼来了。
pub const DEFAULT_GOUGE_TOL_MM: f64 = 0.005;

/// 设计面采样步距（mm）。0.2 → 30 mm 见方的顶面 150×150 点。
pub const DEFAULT_DESIGN_STEP_MM: f64 = 0.2;

/// 设计成品。设计模型必须表达**最终的零件**：程序把料切到模型内 = 过切，
/// 没切到模型面 = 残留。
#[derive(Clone, Debug)]
pub enum DesignModel {
    /// 轴对齐盒——最常见的解析基准（铣平的顶面 / 台阶 / 沉孔深度）。
    Box { min: DVec3, max: DVec3 },
    /// 网格化设计场（STL → 采样 SDF，见 `voxel::meshvol`）+ 设计面采样点。
    ///
    /// `surface` 由 [`field_design`] 按 `step` 预生成，解析过切测试直接用；
    /// 场查询走体素三线性（量化 ±一个量化档，体积口径不受影响——符号
    /// 只在设计面一个量化档内可能翻转）。
    Field {
        sdf: SparseGrid<i8>,
        surface: Vec<DVec3>,
    },
    /// **盒 − 球**（球面型腔 / 半球坑）。解析式，无体素化代价。
    ///
    /// `sdf = max(box_sdf, −sphere_sdf)`：盒内挖掉一个球。球心落在盒的顶面上
    /// 时，被挖掉的正好是**下半球**——那就是一个 R=radius 的球面型腔。
    /// 球心在盒内偏上也没问题，只要球顶超出盒顶，多出来的部分本来就在空气里。
    Cavity {
        min: DVec3,
        max: DVec3,
        center: DVec3,
        radius: f64,
    },
    /// **盒 ∪ 球**（半球凸台）。`sdf = min(box_sdf, sphere_sdf)`。
    ///
    /// 球心落在盒的顶面上时，凸出来的正好是**上半球**——底面是一圈环形平面，
    /// 中间鼓出一个 R 的球冠。和 [`Cavity`](Self::Cavity) 是一对：一个是挖进去，
    /// 一个是凸出来，两者共用同一套球面采样（方向相反）。
    Boss {
        min: DVec3,
        max: DVec3,
        center: DVec3,
        radius: f64,
    },
}

impl DesignModel {
    /// 设计气区距离的**钳制上限**：超过它一律报成它。`None` = 无上限。
    ///
    /// 解析盒是精确欧氏距离，没有上限；网格场在窄带外被钳到 `±band_mm`，
    /// 所以"离设计面多远"只能报到 `band`。**依赖这个距离的指标（比如残留厚度）
    /// 必须知道自己是不是已经饱和**，否则会把上限当成真值。
    pub fn distance_cap(&self) -> Option<f64> {
        match self {
            DesignModel::Box { .. } => None,
            DesignModel::Cavity { .. } | DesignModel::Boss { .. } => None,
            DesignModel::Field { sdf, .. } => Some(sdf.config.band_mm()),
        }
    }

    /// 设计 SDF：负 = 设计材料内，正 = 设计气区，0 = 设计表面。
    ///
    /// 盒体用解析式（各轴取"到远侧面的负距离"的最大值——盒内即到最近面的
    /// 距离，盒外至少一个分量为正）。**注意它只在"面区"是精确欧氏距离，
    /// 角/棱阴影区会低估**——但符号恒与真值一致，体积/深度的判定不受影响。
    /// 场体用三线性插值（带外钳制 ±band）。
    pub fn sdf(&self, p: DVec3) -> f64 {
        match self {
            DesignModel::Box { min, max } => box_sdf(p, *min, *max),
            DesignModel::Cavity { min, max, center, radius } => {
                // 盒 ∩ 球外：两块实体的**并集**，SDF 取 max；球内是气区，所以取 −球
                box_sdf(p, *min, *max).max(*radius - (p - *center).length())
            }
            DesignModel::Boss { min, max, center, radius } => {
                // 盒 ∪ 球：并集取 min，球是实体
                box_sdf(p, *min, *max).min((p - *center).length() - *radius)
            }
            DesignModel::Field { sdf, .. } => sdf.sdf(p),
        }
    }
}

/// 轴对齐盒的有符号距离：负 = 盒内，正 = 盒外。
///
/// 只在"面区"是精确欧氏距离，角/棱阴影区会低估——但**符号恒正确**，
/// 体积/深度判定不受影响。
fn box_sdf(p: DVec3, min: DVec3, max: DVec3) -> f64 {
    let dx = (min.x - p.x).max(p.x - max.x);
    let dy = (min.y - p.y).max(p.y - max.y);
    let dz = (min.z - p.z).max(p.z - max.z);
    dx.max(dy).max(dz)
}

/// 从三角形网格装配场设计（STL 解析后的产物）。
///
/// 体素化用与毛坯相同的网格配置/原点/域，保证比对时体素中心对齐。
pub fn field_design(
    tris: &[cncsim_voxel::stl::Tri],
    config: cncsim_voxel::GridConfig,
    origin: DVec3,
    domain: [i32; 3],
    step: f64,
) -> DesignModel {
    let sdf = cncsim_voxel::meshvol::mesh_to_field(tris, config, origin, domain);
    let surface = cncsim_voxel::meshvol::surface_points(tris, step);
    DesignModel::Field { sdf, surface }
}

/// 一段实际切削（供解析过切测试与报警定位）。
#[derive(Clone, Copy, Debug)]
pub struct CutSegment {
    pub from: Pose,
    pub to: Pose,
    /// 所属块（`MotionProgram::blocks` 下标）
    pub block: u32,
    /// 源文件行号
    pub line: u32,
    /// 段开始时的仿真时间（秒）
    pub t_start: f64,
    /// 这一段的**刀号**：过切判定必须用**这一段真正用的那把刀**的母线。
    /// 粗加工 φ6 和精加工 R0.5 的扫掠体差着数量级，混用会把过切算成 0。
    pub tool: u16,
}

/// 比对结果。所有位置字段是世界坐标。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DesignComparison {
    /// 过切体积（mm³）：已被切空、但设计要求材料保留的体素积分。
    pub gouge_volume_mm3: f64,
    /// 过切最大深度（mm，解析扫掠点测试）。0 = 没有超过阈值的过切。
    ///
    /// 语义：**设计面被刀具表面侵入的深度**（设计面点到扫掠体边界的最近距离）。
    /// 对掠切（刀面擦过设计面）它是精确的过切量；对孔/槽这类整体切除，
    /// 这个值受扫掠体局部半宽限制——全深看 [`Self::gouge_extent_mm`]。
    pub gouge_max_depth: f64,
    pub gouge_at: DVec3,
    /// 过切区域的包围盒（世界坐标，体素中心范围）。`None` = 没有过切。
    ///
    /// 比"最大深度"更有用：孔切到 z=3、设计顶 z=11 → 区域 z 从 3.05 到 11，
    /// 一眼看出是哪个特征切多了。深度本身看 [`Self::gouge_max_depth`]。
    pub gouge_region: Option<(DVec3, DVec3)>,
    /// 最深过切所在的块/行/时刻——报警要能跳回源文件。
    pub gouge_block: u32,
    pub gouge_line: u32,
    pub gouge_t: f64,

    /// 残留体积（mm³）：材料还在、设计要求切掉的体素积分。
    pub residual_volume_mm3: f64,
    /// 残留最大厚度（mm，±h/2）。0 = 设计气区里没有材料。
    pub residual_max_thickness: f64,
    pub residual_at: DVec3,

    /// 设计气区距离的钳制上限（`None` = 解析盒，无上限）。
    ///
    /// 报告要显示"上限是多少"时必须用这个，**不能用毛坯场的 band**——
    /// 设计模型和毛坯是两个独立的网格，band 可以不同（实测 `--design` 用 2、
    /// 毛坯用 4，拿错就会把 0.25 的上限印成 0.45）。
    pub residual_thickness_cap: Option<f64>,

    /// 残留最厚那一点是否**已经饱和**（设计气区距离被钳到 `band`）。
    ///
    /// `true` 时 `residual_max_thickness` 是**下限**，真实值更大。
    /// 只有 `DesignModel::Field` 会饱和（解析盒的距离无上限）。
    pub residual_thickness_capped: bool,

    /// 参与解析测试的设计面采样点数（诊断用）
    pub design_points: usize,
}

/// 毛坯场 × 设计模型全量比对。
///
/// `domain` 是毛坯体素数（`RunConfig::stock_size`）——比对外只看毛坯域内：
/// 过切只可能发生在有料的地方，残留只可能发生在料还在的地方。
pub fn compare_with_design(
    grid: &SparseGrid<i8>,
    domain: [i32; 3],
    segments: &[CutSegment],
    tools: &ToolTable,
    design: &DesignModel,
    step: f64,
    tol: f64,
) -> DesignComparison {
    let t0 = std::time::Instant::now();
    let (
        gouge_volume_mm3,
        residual_volume_mm3,
        residual_max_thickness,
        residual_at,
        gouge_region,
        residual_thickness_capped,
    ) = voxel_compare(grid, domain, design);
    let t_vox = t0.elapsed().as_secs_f64() * 1e3;

    let t1 = std::time::Instant::now();
    let (gouge_max_depth, gouge_at, gouge_block, gouge_line, gouge_t, design_points) =
        analytic_gouge(segments, tools, design, step, tol);
    if std::env::var_os("CNCSIM_PROF").is_some() {
        println!(
            "  [cmp] 体素积分 {t_vox:.1} ms | 解析过切 {:.1} ms（{} 段 / {} 设计点）",
            t1.elapsed().as_secs_f64() * 1e3,
            segments.len(),
            design_points
        );
    }

    DesignComparison {
        gouge_volume_mm3,
        gouge_max_depth,
        gouge_at,
        gouge_region,
        gouge_block,
        gouge_line,
        gouge_t,
        residual_volume_mm3,
        residual_max_thickness,
        residual_at,
        residual_thickness_capped,
        residual_thickness_cap: design.distance_cap(),
        design_points,
    }
}

/// 体素积分：过切体积与体积深度 / 残留体积与最大厚度。
///
/// 毛坯值取体素中心的原存储值（不走三线性——中心查询本来就是精确值）：
/// 负 = 材料（带外钳制到 −band，钳制方向对"这是材料"的判定无害），
/// 正 = 已切空。设计值是解析 SDF，处处精确。
#[allow(clippy::type_complexity)]
fn voxel_compare(
    grid: &SparseGrid<i8>,
    domain: [i32; 3],
    design: &DesignModel,
) -> (f64, f64, f64, DVec3, Option<(DVec3, DVec3)>, bool) {
    let h = grid.config.h;
    let h3 = h * h * h;

    // 按 z 分片并行（域是 10.8M 体素，串行要 120 ms）。**归约按片序**，
    // 每片内部 z 递增 → 累加顺序与串行完全一致，结果逐位可复现。
    struct Part {
        gouge: f64,
        resid_vol: f64,
        resid_max: f64,
        resid_at: DVec3,
        region: Option<(DVec3, DVec3)>,
        /// 最厚那一点的设计气区距离是否被钳制（`DesignModel::Field` 才会）
        resid_capped: bool,
    }

    // 设计气区距离的上限：`None` = 解析盒，无上限
    let cap = design.distance_cap();

    let nz = domain[2] as usize;
    let slices = (cncsim_voxel::par::worker_count() * 2).min(nz).max(1);
    let per = nz.div_ceil(slices);

    let parts = cncsim_voxel::par::map_indexed(slices, |si| {
        let k0 = (si * per) as i32;
        let k1 = (((si + 1) * per).min(nz)) as i32;
        let mut part = Part {
            gouge: 0.0,
            resid_vol: 0.0,
            resid_max: 0.0,
            resid_at: DVec3::ZERO,
            region: None,
            resid_capped: false,
        };
        for k in k0..k1 {
            for j in 0..domain[1] {
                for i in 0..domain[0] {
                    let ijk = [i, j, k];
                    let p = grid.voxel_center(ijk);
                    let ds = grid.to_distance(grid.get(ijk));
                    let dd = design.sdf(p);
                    if ds > 0.0 {
                        // 已切空。设计还要料 → 过切。
                        if dd < 0.0 {
                            part.gouge += h3;
                            part.region = Some(match part.region {
                                None => (p, p),
                                Some((lo, hi)) => (
                                    DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)),
                                    DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z)),
                                ),
                            });
                        }
                    } else if dd > 0.0 {
                        // 料还在。设计说是气区 → 残留。
                        part.resid_vol += h3;
                        // 厚度：到设计面的距离 + h/2（体素中心 → 表面的修正）。
                        // 距离场在材料区随远离设计面单调增，最大值自动落在表面上。
                        let thickness = dd + 0.5 * h;
                        if thickness > part.resid_max {
                            part.resid_max = thickness;
                            part.resid_at = p;
                            // `dd` 撞到上限 ⟹ 这一点只是下限，真实更远
                            part.resid_capped = cap.is_some_and(|c| dd >= c - 1e-9);
                        }
                    }
                }
            }
        }
        part
    });

    let mut gouge = 0.0f64;
    let mut resid_vol = 0.0f64;
    let mut resid_max = 0.0f64;
    let mut resid_at = DVec3::ZERO;
    let mut region: Option<(DVec3, DVec3)> = None;
    let mut resid_capped = false;
    for part in parts {
        gouge += part.gouge;
        resid_vol += part.resid_vol;
        if part.resid_max > resid_max {
            resid_max = part.resid_max;
            resid_at = part.resid_at;
            resid_capped = part.resid_capped;
        }
        region = match (region, part.region) {
            (None, b) => b,
            (a, None) => a,
            (Some((alo, ahi)), Some((blo, bhi))) => Some((
                DVec3::new(alo.x.min(blo.x), alo.y.min(blo.y), alo.z.min(blo.z)),
                DVec3::new(ahi.x.max(bhi.x), ahi.y.max(bhi.y), ahi.z.max(bhi.z)),
            )),
        };
    }
    (gouge, resid_vol, resid_max, resid_at, region, resid_capped)
}

/// 解析过切：设计面采样点 × 每段扫掠 SDF。
///
/// 对每个设计面点问一个问题：**它是否曾被刀具包住？** `sd_sweep(p) < 0` ⟺ 点 p
/// 在扫掠体内 ⟺ 刀从设计面之外切了过去。深度 = −sd_sweep（刀面越过设计点多少）。
///
/// 预筛：点在段扫掠 AABB（外扩 tol）之外 ⟹ 到扫掠体距离 ≥ 到 AABB 距离 > tol，
/// 不可能是过切，跳过昂贵的 `sd_sweep`。
#[allow(clippy::type_complexity)]
fn analytic_gouge(
    segments: &[CutSegment],
    tools: &ToolTable,
    design: &DesignModel,
    step: f64,
    tol: f64,
) -> (f64, DVec3, u32, u32, f64, usize) {
    // 段 AABB 预先算一遍（每点每段重算是 78k×110 次纯浪费）。**每段用自己那把刀**
    // 的母线：粗加工的 φ6 和精加工的 R0.5 扫掠体差着数量级。
    let cutters: Vec<&Profile> = segments
        .iter()
        .map(|s| &tools.by_number(s.tool).unwrap_or_else(|| tools.first()).cutter)
        .collect();
    let boxes: Vec<(DVec3, DVec3)> = segments
        .iter()
        .zip(&cutters)
        .map(|(s, cutter)| {
            let (lo, hi) = sweep_aabb(&s.from, &s.to, cutter);
            let t = DVec3::new(tol, tol, tol);
            (lo - t, hi + t)
        })
        .collect();

    // 全体段的合并包围盒：设计面上在外面一圈的点**不可能被任何一段碰到**，
    // 每个点先做一次 O(1) 判断就能跳掉。50×50×100 的毛坯有 4 个 50×100 的侧面，
    // 它们的采样点占绝大多数，而刀从来不去那里。
    let whole = boxes.iter().fold(None::<(DVec3, DVec3)>, |acc, (lo, hi)| {
        Some(match acc {
            None => (*lo, *hi),
            Some((alo, ahi)) => (
                DVec3::new(alo.x.min(lo.x), alo.y.min(lo.y), alo.z.min(lo.z)),
                DVec3::new(ahi.x.max(hi.x), ahi.y.max(hi.y), ahi.z.max(hi.z)),
            ),
        })
    });

    let mut max_depth = 0.0f64;
    let mut at = DVec3::ZERO;
    let mut block = 0u32;
    let mut line = 0u32;
    let mut t_seg = 0.0f64;
    let mut points = 0usize;

    for p in design_surface_points(design, step) {
        points += 1;
        if let Some((lo, hi)) = whole
            && (p.x < lo.x || p.x > hi.x || p.y < lo.y || p.y > hi.y || p.z < lo.z || p.z > hi.z)
        {
            continue;
        }
        for (si, s) in segments.iter().enumerate() {
            let (lo, hi) = boxes[si];
            if p.x < lo.x || p.x > hi.x || p.y < lo.y || p.y > hi.y || p.z < lo.z || p.z > hi.z {
                continue;
            }
            let sd = sd_sweep(p, &s.from, &s.to, cutters[si]);
            if sd < -tol && -sd > max_depth {
                max_depth = -sd;
                at = p;
                block = s.block;
                line = s.line;
                t_seg = s.t_start;
                if std::env::var_os("CNCSIM_PROF").is_some() {
                    println!(
                        "  [gouge] {:.3} @ ({:.3},{:.3},{:.3}) 段 {si} tool T{} 第 {} 行 from {:?} to {:?}",
                        max_depth, p.x, p.y, p.z, s.tool, s.line, s.from.tip, s.to.tip
                    );
                }
            }
        }
    }
    (max_depth, at, block, line, t_seg, points)
}

/// 设计面采样点（扁平迭代器）。
///
/// 盒体：6 个面按 `step` 铺面片中心网格——不收集成 Vec（30 mm 盒 @0.2 步距
/// 有 7.8 万个点）。场体：加载时按同样步距预采样好的表面点，直接给。
fn design_surface_points(
    design: &DesignModel,
    step: f64,
) -> Box<dyn Iterator<Item = DVec3> + '_> {
    let step = step.max(1e-6);
    match design {
        DesignModel::Box { min, max } => box_faces(*min, *max, step),
        DesignModel::Cavity { min, max, center, radius } => {
            // 盒面里**被球挖掉的那块不是设计面**（它是空气），必须剔掉——
            // 否则坑口那一圈点会被当成"设计要料"判成过切。
            //
            // 容差必须**是 0**：早先用"半个采样步"（0.2 mm）当容差，结果把
            // 坑口内侧 0.2 mm 的空气点当成了设计面，精加工球正好扫过它们，
            // 报出 1.2 mm³ / 0.25 mm 的**假过切**。半径上的判定没有"差不多"，
            // 球口就是 r = R 那一个圆。
            let (c, r) = (*center, *radius);
            Box::new(
                box_faces(*min, *max, step)
                    .filter(move |p| (*p - c).length() >= r)
                    .chain(hemisphere_points(c, r, step, false)),
            )
        }
        DesignModel::Boss { min, max, center, radius } => {
            // 反过来：**球内的盒面**（底面中间那一块）是内部，不是表面；
            // 球面只取上半球。
            let (c, r) = (*center, *radius);
            Box::new(
                box_faces(*min, *max, step)
                    .filter(move |p| (*p - c).length() >= r)
                    .chain(hemisphere_points(c, r, step, true)),
            )
        }
        DesignModel::Field { surface, .. } => Box::new(surface.iter().copied()),
    }
}

/// 盒体 6 个面的采样点（面片中心）。**不收集成 Vec**——30 mm 盒 @0.2 步距
/// 有 7.8 万个点，装成 Vec 只是白占内存。
fn box_faces(min: DVec3, max: DVec3, step: f64) -> Box<dyn Iterator<Item = DVec3>> {
    let size = max - min;
    // 每个面：两个张成轴 + 一个固定轴
    let nx = ((size.x / step).ceil() as usize).max(1);
    let ny = ((size.y / step).ceil() as usize).max(1);
    let nz = ((size.z / step).ceil() as usize).max(1);
    let sx = size.x / nx as f64;
    let sy = size.y / ny as f64;
    let sz = size.z / nz as f64;

    // 6 个面 = (张成轴对, 固定轴取值) 的组合
    let faces: Vec<Box<dyn Iterator<Item = DVec3>>> = vec![
        // ±Z 面（x-y 张成）
        Box::new((0..ny).flat_map(move |j| {
            (0..nx).map(move |i| DVec3::new(min.x + (i as f64 + 0.5) * sx, min.y + (j as f64 + 0.5) * sy, min.z))
        })),
        Box::new((0..ny).flat_map(move |j| {
            (0..nx).map(move |i| DVec3::new(min.x + (i as f64 + 0.5) * sx, min.y + (j as f64 + 0.5) * sy, max.z))
        })),
        // ±Y 面（x-z 张成）
        Box::new((0..nz).flat_map(move |k| {
            (0..nx).map(move |i| DVec3::new(min.x + (i as f64 + 0.5) * sx, min.y, min.z + (k as f64 + 0.5) * sz))
        })),
        Box::new((0..nz).flat_map(move |k| {
            (0..nx).map(move |i| DVec3::new(min.x + (i as f64 + 0.5) * sx, max.y, min.z + (k as f64 + 0.5) * sz))
        })),
        // ±X 面（y-z 张成）
        Box::new((0..nz).flat_map(move |k| {
            (0..ny).map(move |j| DVec3::new(min.x, min.y + (j as f64 + 0.5) * sy, min.z + (k as f64 + 0.5) * sz))
        })),
        Box::new((0..nz).flat_map(move |k| {
            (0..ny).map(move |j| DVec3::new(max.x, min.y + (j as f64 + 0.5) * sy, min.z + (k as f64 + 0.5) * sz))
        })),
    ];
    Box::new(faces.into_iter().flatten())
}

/// **半球**面采样点（弧长约 `step`）。`upper` 决定取上半球还是下半球。
///
/// 经线按 `Δθ = step/R` 走，每条纬线上按 `Δφ = step/(R sinθ)` 铺 ——
/// 纬线圈越靠近极点越短，用固定 φ 步长的话极点附近会挤成一堆、赤道附近又稀疏。
fn hemisphere_points(
    center: DVec3,
    radius: f64,
    step: f64,
    upper: bool,
) -> std::vec::IntoIter<DVec3> {
    use std::f64::consts::TAU;
    let step = step.max(1e-3);
    let r = radius.max(1e-6);
    let n_theta = ((std::f64::consts::FRAC_PI_2 * r / step).ceil() as usize).max(2);
    let mut out = Vec::new();
    for k in 0..=n_theta {
        let th = std::f64::consts::FRAC_PI_2 * (k as f64 / n_theta as f64);
        let ring = r * th.sin();
        // upper = true 取上半球（凸台），false 取下半球（型腔）
        let z = if upper { center.z + r * th.cos() } else { center.z - r * th.cos() };
        let n_phi = ((TAU * ring / step).ceil() as usize).max(6);
        for j in 0..n_phi {
            let phi = TAU * (j as f64) / n_phi as f64;
            out.push(DVec3::new(
                center.x + ring * phi.cos(),
                center.y + ring * phi.sin(),
                z,
            ));
        }
    }
    out.into_iter()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_tool::flat_end_mill;

    fn box_design(min: (f64, f64, f64), max: (f64, f64, f64)) -> DesignModel {
        DesignModel::Box {
            min: DVec3::new(min.0, min.1, min.2),
            max: DVec3::new(max.0, max.1, max.2),
        }
    }

    /// 单刀刀库：把母线包成 `T1`。解析过切按段查刀号，测试里只有一把。
    fn single_tool(cutter: &cncsim_tool::Profile) -> ToolTable {
        ToolTable::single(cncsim_tool::ToolDef {
            number: 1,
            kind: cncsim_tool::CutterKind::Flat,
            diameter: 2.0 * cutter.max_radius(),
            corner_radius: 0.0,
            flute_length: cutter.height(),
            shank: cutter.clone(),
            length_offset: cutter.height(),
            cutter: cutter.clone(),
        })
    }

    #[test]
    fn box_sdf_signs_and_exact_surface() {
        let d = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 2.5)), -2.5, "盒内：到最近面（顶）的距离");
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 5.0)), 0.0, "面上恰为 0");
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 7.0)), 2.0, "盒外：到面距离");
        assert_eq!(d.sdf(DVec3::new(12.0, 5.0, 2.5)), 2.0, "侧面外也是到面距离");
        assert_eq!(d.sdf(DVec3::new(12.0, 5.0, 7.0)), 2.0, "角区：各轴距离的最大值");
    }

    /// 盒 ∪ 球：**球心落在盒顶面上** ⇒ 凸出来的是上半球。
    #[test]
    fn boss_sdf_is_union_of_box_and_sphere() {
        let d = DesignModel::Boss {
            min: DVec3::new(0.0, 0.0, 0.0),
            max: DVec3::new(50.0, 50.0, 85.0),
            center: DVec3::new(25.0, 25.0, 85.0),
            radius: 15.0,
        };
        // 半球内部（比球顶低 1）：材料
        assert_eq!(d.sdf(DVec3::new(25.0, 25.0, 99.0)), -1.0, "球内 → 负，距离 = R − |p−c|");
        // 球顶正上方 1 mm：空气
        assert_eq!(d.sdf(DVec3::new(25.0, 25.0, 101.0)), 1.0);
        // 球外、盒顶上方 1 mm（半径 20）：空气
        assert_eq!(d.sdf(DVec3::new(45.0, 25.0, 86.0)), 1.0);
        // 盒顶平面上的环带（半径 20，z=85）：表面
        assert_eq!(d.sdf(DVec3::new(45.0, 25.0, 85.0)), 0.0);
        // 盒体深处：材料
        assert!(d.sdf(DVec3::new(25.0, 25.0, 40.0)) < 0.0);
        // 下半球那一块也是材料（并被盒盖住）——取并集才会这样
        assert!(d.sdf(DVec3::new(25.0, 25.0, 71.0)) < 0.0, "球心下方 14 仍在盒内");
    }

    /// 凸台的表面采样：**上半球 + 被球切开的那部分盒面**，下半球不许出现。
    #[test]
    fn boss_surface_points_are_the_dome_and_the_floor() {
        use std::f64::consts::FRAC_PI_2;
        let c = DVec3::new(25.0, 25.0, 85.0);
        let d = DesignModel::Boss {
            min: DVec3::ZERO,
            max: DVec3::new(50.0, 50.0, 85.0),
            center: c,
            radius: 15.0,
        };
        let pts: Vec<DVec3> = design_surface_points(&d, 0.5).collect();
        assert!(pts.len() > 1000, "采样点太少：{}", pts.len());
        for &p in &pts {
            let sd = d.sdf(p);
            assert!(sd.abs() < 1e-9, "采样点必须在设计面上：{p:?} sd={sd}");
        }
        // 球面上必须有点，且**全部在上半球**（下半球被盒吃掉，不是表面）
        let on_dome: Vec<DVec3> = pts
            .iter()
            .copied()
            .filter(|p| ((*p - c).length() - 15.0).abs() < 1e-9)
            .collect();
        assert!(on_dome.len() > 500, "球面点 {} 个", on_dome.len());
        assert!(
            on_dome.iter().all(|p| p.z >= c.z - 1e-9),
            "出现了下半球的采样点——那里在盒体内，不是表面"
        );
        // 顶点必须被采到（θ 从赤道走到极点）
        assert!(
            on_dome.iter().any(|p| (p.z - (c.z + 15.0)).abs() < 1e-6),
            "球顶没有采样点：最大 z = {}",
            on_dome.iter().map(|p| p.z).fold(f64::MIN, f64::max)
        );
        let _ = FRAC_PI_2;
        // 底面环带也要有（半径 ≥ R 的 z=85 平面）
        assert!(
            pts.iter().any(|p| (p.z - 85.0).abs() < 1e-9 && (p.x - 25.0).hypot(p.y - 25.0) > 16.0),
            "盒顶环带没被采样——精加工/过切都靠它"
        );
    }

    #[test]
    fn design_surface_points_cover_all_six_faces() {
        let d = box_design((0.0, 0.0, 0.0), (2.0, 2.0, 1.0));
        let pts: Vec<DVec3> = design_surface_points(&d, 1.0).collect();
        // 2+2 面各 4 点、4 个侧面各 2 点
        assert_eq!(pts.len(), 2 * (2 * 2) + 4 * 2);
        // 全部落在盒面上
        for &p in &pts {
            let sd = d.sdf(p);
            assert!(sd.abs() < 1e-9, "采样点应在设计面上，实际 sd={sd}");
        }
    }

    #[test]
    fn field_design_matches_box_design() {
        use cncsim_voxel::{GridConfig, ValueType};
        // 同一个盒：解析 Box vs 网格体素化的 Field——两套独立路径必须同判
        let box_d = box_design((0.5, 0.5, 0.5), (2.5, 2.5, 2.5));
        // 盒网格 12 条三角形
        let p = |x, y, z| DVec3::new(x, y, z);
        let mut tris: Vec<cncsim_voxel::stl::Tri> = Vec::new();
        let mut quad = |a: (f64, f64, f64),
                        b: (f64, f64, f64),
                        c: (f64, f64, f64),
                        d: (f64, f64, f64)| {
            tris.push([p(a.0, a.1, a.2), p(b.0, b.1, b.2), p(c.0, c.1, c.2)]);
            tris.push([p(a.0, a.1, a.2), p(c.0, c.1, c.2), p(d.0, d.1, d.2)]);
        };
        quad((0.5, 0.5, 0.5), (2.5, 0.5, 0.5), (2.5, 2.5, 0.5), (0.5, 2.5, 0.5));
        quad((0.5, 0.5, 2.5), (2.5, 0.5, 2.5), (2.5, 2.5, 2.5), (0.5, 2.5, 2.5));
        quad((0.5, 0.5, 0.5), (2.5, 0.5, 0.5), (2.5, 0.5, 2.5), (0.5, 0.5, 2.5));
        quad((0.5, 2.5, 0.5), (2.5, 2.5, 0.5), (2.5, 2.5, 2.5), (0.5, 2.5, 2.5));
        quad((0.5, 0.5, 0.5), (0.5, 2.5, 0.5), (0.5, 2.5, 2.5), (0.5, 0.5, 2.5));
        quad((2.5, 0.5, 0.5), (2.5, 2.5, 0.5), (2.5, 2.5, 2.5), (2.5, 0.5, 2.5));

        let cfg = GridConfig { h: 0.1, band_voxels: 4, value_type: ValueType::I8 };
        let field_d = field_design(&tris, cfg, DVec3::ZERO, [30, 30, 30], 0.2);

        for k in 0..30 {
            for j in 0..30 {
                for i in 0..30 {
                    let pt =
                        DVec3::new(0.05 + i as f64 * 0.1, 0.05 + j as f64 * 0.1, 0.05 + k as f64 * 0.1);
                    let a = box_d.sdf(pt);
                    let b = field_d.sdf(pt);
                    // 符号必须一致
                    assert!(
                        a.signum() == b.signum() || a == 0.0 || b == 0.0,
                        "({i},{j},{k}) 符号不一致：box {a} vs field {b}"
                    );
                    // 带内距离差 ≤ 一个量化档 + 三线性误差。
                    // 只比面区：角/棱阴影区（≥2 个分量在盒外）box 公式低估
                    // 距离，而场算的是真欧氏距离——那里只对符号。
                    let outside_axes = (pt.x < 0.5 || pt.x > 2.5) as u32
                        + (pt.y < 0.5 || pt.y > 2.5) as u32
                        + (pt.z < 0.5 || pt.z > 2.5) as u32;
                    if a.abs() < 0.35 && b.abs() < 0.35 && (a < 0.0 || outside_axes <= 1) {
                        assert!(
                            (a - b).abs() < 0.01,
                            "({i},{j},{k}) 带内差过大：box {a} vs field {b}"
                        );
                    }
                }
            }
        }
        // 表面采样点都落在盒面上
        let DesignModel::Field { surface, .. } = &field_d else {
            panic!("应为 Field");
        };
        assert!(surface.len() > 100, "surface={}", surface.len());
        for &q in surface.iter() {
            assert!(box_d.sdf(q).abs() < 1e-9, "表面点应在盒面上，实际 {}", box_d.sdf(q));
        }
    }

    #[test]
    fn analytic_gouge_finds_the_deep_point_and_its_segment() {
        // 设计顶 z=5，刀（Ø6 平底）在 z=4.6 从 x=0 走到 x=10：
        // 顶面点在扫掠体内 0.4 mm——除了刀没经过的地方。
        let cutter = flat_end_mill(6.0, 20.0);
        let tools = single_tool(&cutter);
        let segments = vec![CutSegment {
            from: Pose { tip: DVec3::new(0.0, 5.0, 4.6), axis: DVec3::Z },
            to: Pose { tip: DVec3::new(10.0, 5.0, 4.6), axis: DVec3::Z },
            block: 3,
            line: 7,
            t_start: 1.25,
            tool: 1,
        }];
        let design = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        let (depth, at, block, line, _t, n) =
            analytic_gouge(&segments, &tools, &design, 0.5, 0.005);

        assert!(n > 100, "设计面应有大量采样点：{n}");
        assert!((depth - 0.4).abs() < 0.002, "过切深度应为 0.4：{depth}");
        assert_eq!(block, 3);
        assert_eq!(line, 7);
        assert!((at.z - 5.0).abs() < 1e-9, "过切点在设计顶面上");
        // argmax 取第一个并列最深点——任何落在刀 footprint 内部（|y−5| ≤ 2.75，
        // 网格步距 0.5）的点都给精确 −0.4，约束到 footprint 内即可
        assert!((at.y - 5.0).abs() <= 2.75 + 1e-9, "y={}", at.y);
    }

    #[test]
    fn analytic_gouge_ignores_tangent_passes() {
        // 刀底平面恰好在设计顶面上（完美的面铣）→ sd_sweep = 0，不是过切。
        let cutter = flat_end_mill(6.0, 20.0);
        let tools = single_tool(&cutter);
        let segments = vec![CutSegment {
            from: Pose { tip: DVec3::new(0.0, 5.0, 5.0), axis: DVec3::Z },
            to: Pose { tip: DVec3::new(10.0, 5.0, 5.0), axis: DVec3::Z },
            block: 0,
            line: 0,
            t_start: 0.0,
            tool: 1,
        }];
        let design = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        let (depth, _at, _b, _l, _t, _n) =
            analytic_gouge(&segments, &tools, &design, 0.5, 0.005);
        assert_eq!(depth, 0.0, "恰好切到设计面不算过切");
    }

    /// 体素比对的独立小场景：不用整套 sim::run，手工填一个场。
    #[test]
    fn voxel_compare_counts_gouge_and_residual_layers() {
        use cncsim_voxel::{GridConfig, ValueType};
        let cfg = GridConfig { h: 0.1, band_voxels: 4, value_type: ValueType::I8 };
        let mut grid = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
        // 2×2×2 mm 毛坯（20×20×20 体素）
        let domain = [20, 20, 20];
        grid.fill_box([0, 0, 0], domain, i8::MIN);
        grid.prune();

        // 设计：0..1.0 z（下半）。上半（z 1..2）按设计该切掉。
        let design = box_design((0.0, 0.0, 0.0), (2.0, 2.0, 1.0));

        // 场景一：什么都没切。残留 = 上半 10 层 × 400 体素 × 0.001 = 4.0 mm³；
        // 厚度最大处在顶面体素（中心 z=1.95，dd=0.95，+h/2 = 1.0）。
        let (g, rv, rt, _at, region, _capped) = voxel_compare(&grid, domain, &design);
        assert_eq!(g, 0.0);
        assert_eq!(region, None);
        assert!((rv - 4.0).abs() < 1e-9, "残留体积 = 20×20×10×0.001 = 4.0，实际 {rv}");
        assert!((rt - 1.0).abs() < 1e-9, "顶面体素 dd=0.95 + h/2 = 1.0，实际 {rt}");

        // 场景二：恰好切到设计面（z ≥ 1.0 的体素全变空气）。
        // 共享边界上两个 SDF 符号精确一致 → 过切必须是 0。
        for k in 10..20 {
            for j in 0..domain[1] {
                for i in 0..domain[0] {
                    grid.set([i, j, k], i8::MAX);
                }
            }
        }
        let (g, rv, rt, _at, region, _capped) = voxel_compare(&grid, domain, &design);
        assert_eq!(g, 0.0, "恰好切到设计面不算过切");
        assert_eq!(region, None);
        assert_eq!(rv, 0.0, "切干净了不该有残留");
        assert_eq!(rt, 0.0);

        // 场景三：切过头半毫米（z ≥ 0.5 变空气）。
        // 空气且设计说有料的体素：中心 z=0.55..0.95 共 5 层 → 2000×0.001 = 2.0 mm³。
        for k in 5..20 {
            for j in 0..domain[1] {
                for i in 0..domain[0] {
                    grid.set([i, j, k], i8::MAX);
                }
            }
        }
        let (g, rv, rt, _at, region, _capped) = voxel_compare(&grid, domain, &design);
        assert!((g - 2.0).abs() < 1e-9, "过切 = 5 层 × 400 体素 × 0.001，实际 {g}");
        // 区域：空气体素中心 z=0.55..0.95，x/y 全域
        let (lo, hi) = region.expect("有过切必有区域");
        assert!((lo.z - 0.55).abs() < 1e-9 && (hi.z - 0.95).abs() < 1e-9, "z {}..{}", lo.z, hi.z);
        assert!((lo.x - 0.05).abs() < 1e-9 && (hi.x - 1.95).abs() < 1e-9);
        assert_eq!(rv, 0.0);
        assert_eq!(rt, 0.0);
    }
}
