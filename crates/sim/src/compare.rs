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
use cncsim_tool::Profile;
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
            DesignModel::Box { min, max } => {
                let dx = (min.x - p.x).max(p.x - max.x);
                let dy = (min.y - p.y).max(p.y - max.y);
                let dz = (min.z - p.z).max(p.z - max.z);
                dx.max(dy).max(dz)
            }
            DesignModel::Field { sdf, .. } => sdf.sdf(p),
        }
    }
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
    cutter: &Profile,
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
        analytic_gouge(segments, cutter, design, step, tol);
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
    cutter: &Profile,
    design: &DesignModel,
    step: f64,
    tol: f64,
) -> (f64, DVec3, u32, u32, f64, usize) {
    // 段 AABB 预先算一遍（每点每段重算是 78k×110 次纯浪费）
    let boxes: Vec<(DVec3, DVec3)> = segments
        .iter()
        .map(|s| {
            let (lo, hi) = sweep_aabb(&s.from, &s.to, cutter);
            let t = DVec3::new(tol, tol, tol);
            (lo - t, hi + t)
        })
        .collect();

    let mut max_depth = 0.0f64;
    let mut at = DVec3::ZERO;
    let mut block = 0u32;
    let mut line = 0u32;
    let mut t_seg = 0.0f64;
    let mut points = 0usize;

    for p in design_surface_points(design, step) {
        points += 1;
        for (si, s) in segments.iter().enumerate() {
            let (lo, hi) = boxes[si];
            if p.x < lo.x || p.x > hi.x || p.y < lo.y || p.y > hi.y || p.z < lo.z || p.z > hi.z {
                continue;
            }
            let sd = sd_sweep(p, &s.from, &s.to, cutter);
            if sd < -tol && -sd > max_depth {
                max_depth = -sd;
                at = p;
                block = s.block;
                line = s.line;
                t_seg = s.t_start;
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
        DesignModel::Box { min, max } => {
            let size = *max - *min;
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
        DesignModel::Field { surface, .. } => Box::new(surface.iter().copied()),
    }
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

    #[test]
    fn box_sdf_signs_and_exact_surface() {
        let d = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 2.5)), -2.5, "盒内：到最近面（顶）的距离");
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 5.0)), 0.0, "面上恰为 0");
        assert_eq!(d.sdf(DVec3::new(5.0, 5.0, 7.0)), 2.0, "盒外：到面距离");
        assert_eq!(d.sdf(DVec3::new(12.0, 5.0, 2.5)), 2.0, "侧面外也是到面距离");
        assert_eq!(d.sdf(DVec3::new(12.0, 5.0, 7.0)), 2.0, "角区：各轴距离的最大值");
    }

    #[test]
    fn design_surface_points_cover_all_six_faces() {
        let d = box_design((0.0, 0.0, 0.0), (2.0, 2.0, 1.0));
        let pts: Vec<DVec3> = design_surface_points(&d, 1.0).collect();
        // 2+2 面各 4 点、4 个侧面各 2 点
        assert_eq!(pts.len(), 2 * (2 * 2) + 4 * (2 * 1));
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
                    assert_eq!(
                        a.signum() == b.signum() || a == 0.0 || b == 0.0,
                        true,
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
        let segments = vec![CutSegment {
            from: Pose { tip: DVec3::new(0.0, 5.0, 4.6), axis: DVec3::Z },
            to: Pose { tip: DVec3::new(10.0, 5.0, 4.6), axis: DVec3::Z },
            block: 3,
            line: 7,
            t_start: 1.25,
        }];
        let design = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        let (depth, at, block, line, _t, n) =
            analytic_gouge(&segments, &cutter, &design, 0.5, 0.005);

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
        let segments = vec![CutSegment {
            from: Pose { tip: DVec3::new(0.0, 5.0, 5.0), axis: DVec3::Z },
            to: Pose { tip: DVec3::new(10.0, 5.0, 5.0), axis: DVec3::Z },
            block: 0,
            line: 0,
            t_start: 0.0,
        }];
        let design = box_design((0.0, 0.0, 0.0), (10.0, 10.0, 5.0));
        let (depth, _at, _b, _l, _t, _n) =
            analytic_gouge(&segments, &cutter, &design, 0.5, 0.005);
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
