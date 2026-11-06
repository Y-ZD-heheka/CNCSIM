//! 模型相似度：把"像不像"从一句话变成一组**互不替代**的数。
//!
//! 现有口径已经有三层（`Δz`/`Δr` 单向几何量、体积比对、名义面逐点有符号偏差），
//! 但它们有两个共同的盲区：
//!
//! 1. **只有单向**（设计面 → 实际面）。落在设计面之外很远的残料——比如没被任何
//!    工序碰过的一个方块——不会被设计面上的采样点看到。这正是 Hausdorff/Chamfer
//!    要双向的原因。
//! 2. **只有位置，没有朝向**。面被平移 0.05 mm 和面被磨成斜面，逐点距离可能一样。
//!
//! 本模块补齐一组指标：
//!
//! | 指标 | 抓什么 | 盲区 |
//! |---|---|---|
//! | Hausdorff（双向最大） | 最坏情况：干涉/装配过盈就死在这一处 | 单点极值，对离群敏感；不看面积 |
//! | Chamfer（双向均值） | 整体形状差异，可当"做像了多少"的总分 | 大偏差被面积摊薄 |
//! | F-score@τ（P/R） | 面吻合程度：P 从实际面出发（多出来的料会拉低它），R 从设计面出发（没切到的面会拉低它） | **不含方向**——过切和残料都会让两者一起掉，要看 `bias` |
//! | `bias`（有符号平均位移） | 正 = 整体偏过切、负 = 整体偏残料 | 会互相抵消（一处过切 + 一处残料 ≈ 0） |
//! | IoU / Dice（体素体积） | 全局体积吻合，采样密度无关 | 对薄壁/曲面位移不敏感 |
//! | 法向一致 | 位置对但朝向错（斜面、台阶圆角） | 边缘/尖角处偏噪 |
//!
//! 都不做配准（ICP/Go-ICP 那一类）：实际面与设计面共用同一坐标系，配准只会把
//! "装夹偏了 0.3 mm"这种真问题吸收掉。
//!
//! **读数注意**：`DesignModel::Field` 的 SDF 在窄带外被钳制（`distance_cap()`），
//! 所以场设计的距离是**下界**；`design_saturated` / `actual_saturated` 就是被钳住的
//! 点数，非零时 `hausdorff` 只能读作"至少这么大"。

use crate::compare::DesignModel;
use cncsim_math::DVec3;
use cncsim_voxel::MeshData;

/// 相似度报告。距离单位 mm，比例单位 0..1。
#[derive(Clone, Debug, Default)]
pub struct Similarity {
    pub tol: f64,
    /// 设计面采样点数（面积加权）。
    pub design_points: usize,
    /// 设计面 → 实际面：面积加权平均 |d|。
    pub design_mean_abs: f64,
    /// 设计面 → 实际面：最大 |d|（单向 Hausdorff）。
    pub design_max: f64,
    /// 有符号平均位移（正 = 过切；负 = 残料）。
    pub design_bias: f64,
    /// 距离被窄带钳制的点数（>0 时 `hausdorff` 只是下界）。
    pub design_saturated: usize,
    /// 实际面顶点数（提取网格的顶点，非面积加权）。
    pub actual_points: usize,
    pub actual_mean_abs: f64,
    pub actual_max: f64,
    pub actual_bias: f64,
    pub actual_saturated: usize,
    /// 双向 Hausdorff：`max(design_max, actual_max)`。
    pub hausdorff: f64,
    /// 双向 Chamfer：两侧平均 |d| 的均值。
    pub chamfer: f64,
    /// 整体有符号偏置：`(design_bias + actual_bias) / 2`。
    pub bias_mm: f64,
    /// 精度：实际面顶点里落在设计面 `±tol` 内的比例。
    pub precision: f64,
    /// 召回：设计面面积里落在实际面 `±tol` 内的比例。
    pub recall: f64,
    /// `2PR/(P+R)`。
    pub f_score: f64,
    /// 面积加权的 `|n_design · n_actual|` 平均。
    pub normal_consistency: f64,
    /// 体素体积交并比。
    pub iou: f64,
    /// `2|A∩B|/(|A|+|B|)`。
    pub dice: f64,
}

impl Similarity {
    /// 一句话结论（给人看的，不参与判定）。
    pub fn verdict(&self) -> &'static str {
        if self.hausdorff <= self.tol && self.f_score >= 0.95 {
            "形状吻合"
        } else if self.bias_mm > self.tol {
            "整体偏过切（切多了）"
        } else if self.bias_mm < -self.tol {
            "整体偏残料（没切到）"
        } else if self.f_score < 0.9 {
            "局部有面偏离（正负抵消，看分布）"
        } else {
            "大体吻合，个别点超差"
        }
    }
}

/// 算一遍相似度。
///
/// * `samples` —— 设计面采样 `(位置, 单位外法向, 面积)`，来自
///   [`cncsim_voxel::meshvol::surface_samples`]。
/// * `grid` —— 仿真后的实际毛坯场（`< 0` = 还有材料）。
/// * `domain` —— 体素域大小（`RunConfig::stock_size`）。
/// * `actual` —— 实际面网格（`SparseGrid::extract_surface_lod` 的结果），
///   反向距离与法向一致都用它的顶点/法向。
pub fn compare(
    design: &DesignModel,
    samples: &[(DVec3, DVec3, f64)],
    grid: &cncsim_voxel::tree::SparseGrid<i8>,
    domain: [i32; 3],
    actual: &MeshData,
    tol: f64,
) -> Similarity {
    let cap = design.distance_cap().unwrap_or(f64::INFINITY);
    let sat = (cap - 1e-6).max(0.0);
    let mut s = Similarity {
        tol,
        design_points: samples.len(),
        actual_points: actual.positions.len(),
        ..Default::default()
    };

    // ── ① 正向：设计面 → 实际面（面积加权）──
    let mut w_sum = 0.0;
    let mut acc = 0.0;
    let mut acc_signed = 0.0;
    let mut in_tol = 0.0;
    for (p, _, a) in samples {
        let d = grid.sdf(*p); // 正 = 空气 = 材料被切掉（过切方向）
        let m = d.abs();
        acc += m * a;
        acc_signed += d * a;
        w_sum += a;
        s.design_max = s.design_max.max(m);
        if m >= sat {
            s.design_saturated += 1;
        }
        if m <= tol {
            in_tol += a;
        }
    }
    if w_sum > 0.0 {
        s.design_mean_abs = acc / w_sum;
        s.design_bias = acc_signed / w_sum;
        s.recall = in_tol / w_sum;
    }

    // ── ② 反向：实际面 → 设计面 ──
    //
    // 这一趟**不能省**。只做正向时，一块从来没被刀具碰过的残料不会出现在任何
    // 设计面采样点上（那些点的实际面是好的），Hausdorff 依然很小。反向从实际面
    // 顶点出发，立刻就量到那块料离设计面有多远。
    let eps = grid.config.h.max(1e-3) * 0.5;
    let n = actual.positions.len().min(actual.normals.len());
    let mut acc2 = 0.0;
    let mut acc2_signed = 0.0;
    let mut n_tol = 0usize;
    let mut nc_acc = 0.0;
    let mut nc_n = 0usize;
    for i in 0..n {
        let q = DVec3::new(
            actual.positions[i][0] as f64,
            actual.positions[i][1] as f64,
            actual.positions[i][2] as f64,
        );
        let d = design.sdf(q); // 负 = 在实际材料里 = 实际面比设计面更靠内（过切）
        let m = d.abs();
        acc2 += m;
        acc2_signed += -d; // 统一成"正 = 过切"
        s.actual_max = s.actual_max.max(m);
        if m >= sat {
            s.actual_saturated += 1;
        }
        if m <= tol {
            n_tol += 1;
        }
        // 法向一致：设计面在该点的法向（SDF 梯度）vs 实际网格法向
        let g = DVec3::new(
            design.sdf(q + DVec3::new(eps, 0.0, 0.0)) - design.sdf(q - DVec3::new(eps, 0.0, 0.0)),
            design.sdf(q + DVec3::new(0.0, eps, 0.0)) - design.sdf(q - DVec3::new(0.0, eps, 0.0)),
            design.sdf(q + DVec3::new(0.0, 0.0, eps)) - design.sdf(q - DVec3::new(0.0, 0.0, eps)),
        );
        let gl = g.length();
        if gl > 1e-12 {
            let nd = g / gl;
            let na = DVec3::new(
                actual.normals[i][0] as f64,
                actual.normals[i][1] as f64,
                actual.normals[i][2] as f64,
            );
            let al = na.length();
            if al > 1e-12 {
                nc_acc += (nd.dot(na / al)).abs();
                nc_n += 1;
            }
        }
    }
    if n > 0 {
        s.actual_mean_abs = acc2 / n as f64;
        s.actual_bias = acc2_signed / n as f64;
        s.precision = n_tol as f64 / n as f64;
    }
    if nc_n > 0 {
        s.normal_consistency = nc_acc / nc_n as f64;
    }

    s.hausdorff = s.design_max.max(s.actual_max);
    s.chamfer = 0.5 * (s.design_mean_abs + s.actual_mean_abs);
    s.bias_mm = 0.5 * (s.design_bias + s.actual_bias);
    s.f_score = if s.precision + s.recall > 0.0 {
        2.0 * s.precision * s.recall / (s.precision + s.recall)
    } else {
        0.0
    };

    // ── ③ 体积：IoU / Dice ──
    //
    // 在毛坯体素上逐格判"设计实心 / 实际实心"。体积口径与采样密度无关，
    // 但**对薄壁和曲面位移不敏感**——它是给"整体像不像"用的，不是给公差用的。
    let (mut inter, mut union, mut only_a, mut only_b) = (0usize, 0usize, 0usize, 0usize);
    for k in 0..domain[2] {
        for j in 0..domain[1] {
            for i in 0..domain[0] {
                let c = grid.voxel_center([i, j, k]);
                let a = design.sdf(c) < 0.0;
                let b = grid.get([i, j, k]) < 0;
                match (a, b) {
                    (true, true) => inter += 1,
                    (true, false) => only_a += 1,
                    (false, true) => only_b += 1,
                    _ => {}
                }
                if a || b {
                    union += 1;
                }
            }
        }
    }
    if union > 0 {
        s.iou = inter as f64 / union as f64;
        let tot = 2 * inter + only_a + only_b;
        s.dice = if tot > 0 {
            2.0 * inter as f64 / tot as f64
        } else {
            0.0
        };
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_voxel::stl::Tri;
    use cncsim_voxel::{GridConfig, ValueType};

    fn box_mesh(lo: DVec3, hi: DVec3) -> Vec<Tri> {
        let p = |x, y, z| DVec3::new(x, y, z);
        let c = [
            p(lo.x, lo.y, lo.z),
            p(hi.x, lo.y, lo.z),
            p(hi.x, hi.y, lo.z),
            p(lo.x, hi.y, lo.z),
            p(lo.x, lo.y, hi.z),
            p(hi.x, lo.y, hi.z),
            p(hi.x, hi.y, hi.z),
            p(lo.x, hi.y, hi.z),
        ];
        let f = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        f.iter().map(|q| [c[q[0]], c[q[1]], c[q[2]]]).collect()
    }

    fn build(tris: &[Tri], h: f64, lo: DVec3, dom: [i32; 3]) -> cncsim_voxel::tree::SparseGrid<i8> {
        let cfg = GridConfig {
            h,
            band_voxels: 4,
            value_type: ValueType::I8,
        };
        cncsim_voxel::meshvol::mesh_to_field(tris, cfg, lo, dom)
    }

    const LO: DVec3 = DVec3::new(-2.0, -2.0, -2.0);
    const DOM: [i32; 3] = [64, 24, 20];
    const H: f64 = 0.5;

    /// 同一块料自己比自己：各项都接近完美。**度量本身的校准**。
    #[test]
    fn identical_parts_score_near_perfect() {
        let tris = box_mesh(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 8.0, 6.0));
        let grid = build(&tris, H, LO, DOM);
        let mesh = grid.extract_surface_lod(1);
        let design = DesignModel::Field {
            sdf: grid.clone(),
            surface: Vec::new(),
        };
        let samples = cncsim_voxel::meshvol::surface_samples(&tris, H);
        let s = compare(&design, &samples, &grid, DOM, &mesh, 0.15);
        assert!(s.design_points > 100 && s.actual_points > 100, "{s:?}");
        assert!(
            s.hausdorff < 2.0 * H,
            "自比 Hausdorff 应 <2h：{}",
            s.hausdorff
        );
        assert!(s.chamfer < H, "自比 Chamfer 应 <h：{}", s.chamfer);
        assert!(s.f_score > 0.99, "F-score 应 ≈1：{}", s.f_score);
        assert!(s.iou > 0.95, "自比 IoU 应 >0.95：{}", s.iou);
        assert!(
            s.normal_consistency > 0.95,
            "法向一致应 >0.95：{}",
            s.normal_consistency
        );
        // 自比的 Hausdorff **不会** ≤ tol：盒子的棱边处顶点最多偏离 h/√2，
        // 这是离散化而不是加工误差。所以只要求"没有方向性偏置"。
        assert!(s.bias_mm.abs() < 0.05, "自比偏置应≈0：{}", s.bias_mm);
        println!(
            "自比：H {:.3} / Chamfer {:.3} / F {:.4} / IoU {:.4} / NC {:.3}",
            s.hausdorff, s.chamfer, s.f_score, s.iou, s.normal_consistency
        );
    }

    /// **反向距离不可省**：设计件旁边多出一块从没被切过的料。
    ///
    /// 正向（设计面 → 实际面）只看到设计件自己的面，那块料完全不在视野里，
    /// `design_max` 仍是 0；只有反向（实际面 → 设计面）才发现它离设计面 14 mm。
    /// 这条测试就是为了钉住"必须双向"这个结论。
    #[test]
    fn a_leftover_block_is_only_visible_from_the_reverse_direction() {
        let part = box_mesh(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 8.0, 6.0));
        let mut both = part.clone();
        both.extend(box_mesh(
            DVec3::new(24.0, 0.0, 0.0),
            DVec3::new(28.0, 4.0, 4.0),
        ));
        let grid = build(&both, H, LO, DOM);
        let mesh = grid.extract_surface_lod(1);
        // 设计用**解析盒**：Field 的 SDF 在窄带外被钳到 ±2 mm，量不到 14 mm
        let design = DesignModel::Box {
            min: DVec3::new(0.0, 0.0, 0.0),
            max: DVec3::new(10.0, 8.0, 6.0),
        };
        let samples = cncsim_voxel::meshvol::surface_samples(&part, H);
        let s = compare(&design, &samples, &grid, DOM, &mesh, 0.15);
        assert!(
            s.design_max < 2.0 * H,
            "正向看不到那块残料，应该仍很小：{}",
            s.design_max
        );
        assert!(
            s.actual_max > 12.0,
            "反向必须量到那块料（>12 mm）：{}",
            s.actual_max
        );
        assert!(
            s.hausdorff > 12.0,
            "Hausdorff 应被那块料顶起来：{}",
            s.hausdorff
        );
        assert!(s.precision < 0.95, "精度应被拉低：{}", s.precision);
        assert!(s.recall > 0.99, "设计面本身都在：{}", s.recall);
        assert!(s.iou < 0.95, "IoU 应被多出的体积拉低：{}", s.iou);
        assert_eq!(s.verdict(), "整体偏残料（没切到）");
    }

    /// 顶面被多切 1 mm：`bias` 为正（过切），其余五面仍然吻合。
    #[test]
    fn a_gouged_face_shows_a_positive_bias() {
        let nominal = box_mesh(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 8.0, 6.0));
        // 实际：顶面降到 5.0（少了 1 mm）
        let cut = box_mesh(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 8.0, 5.0));
        let grid = build(&cut, H, LO, DOM);
        let mesh = grid.extract_surface_lod(1);
        let design = DesignModel::Box {
            min: DVec3::new(0.0, 0.0, 0.0),
            max: DVec3::new(10.0, 8.0, 6.0),
        };
        let samples = cncsim_voxel::meshvol::surface_samples(&nominal, H);
        let s = compare(&design, &samples, &grid, DOM, &mesh, 0.15);
        assert!(
            s.design_max > 0.5,
            "顶面应量到 ~1 mm 过切：{}",
            s.design_max
        );
        assert!(s.bias_mm > 0.1, "有符号偏置应为正（过切）：{}", s.bias_mm);
        assert!(
            s.recall > 0.6 && s.recall < 0.95,
            "五个面还吻合，召回应介于中间：{}",
            s.recall
        );
        assert!(s.iou < 1.0 && s.iou > 0.8, "体积略小：{}", s.iou);
        assert_eq!(s.verdict(), "整体偏过切（切多了）");
    }
}
