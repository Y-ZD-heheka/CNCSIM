//! 特征级形位公差：平面度 / 圆柱度，按 **ISO 1101 的最小区域法（minimax）** 评定。
//!
//! 为什么不能只报"最大偏差"：形位公差定义的是**包容区域的宽度**——两张平行平面
//! （或两个同轴圆柱）把实际表面全部夹住时的最小间距。所以
//!
//! ```text
//! 平面度 = max(n·q) − min(n·q)        （不是 max|n·q − d| 的两倍，也不是一半）
//! 圆柱度 = max(r_i) − min(r_i)        （R 被消掉：区域宽度与半径无关）
//! ```
//!
//! 而基准（法向 / 轴向）本身要**优化**：ISO 1101 以最小区域为准，最小二乘只作近似，
//! 且 LSQ 在数学上**不可能小于** MZ（MZ 是从 LSQ 出发做下降）。本模块两个都算，
//! 差值就是「LSQ 高估了多少」——规则网格/对称分布上常常是 0，真实测量数据上
//! 才会拉开（那正是 ISO 要求用最小区域的原因）。
//!
//! 表面点从哪来：设计面采样点沿**实际场的梯度**投影到实际表面
//! （`q = p + d·∇d/|∇d|`）。`d` 是到实际面的有符号距离、梯度是最近点方向，
//! 所以这是精确的一阶投影，不需要最近邻搜索。
//!
//! 已知边界：
//! - 特征分割是"空间连通 + 法向一致"的粗分，不是商业 CAM 的特征识别；
//!   分不出来的归 `Freeform`（只报偏差统计，不报形位公差）。
//! - 圆柱的轴向由 4 参数模式搜索优化，结果仍是**上界**（更细的搜索只会更小）。
//! - 拟合点数上限 4000（按面积等距抽样）——形位公差是全局量，抽样不影响结论。

use cncsim_math::DVec3;
use cncsim_voxel::tree::SparseGrid;

/// 特征类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// 平面：法向高度一致，报**平面度**。
    Plane,
    /// 圆柱：法向都与一根轴垂直，报**圆柱度**。
    Cylinder,
    /// 分不出类型的自由曲面：只报偏差统计。
    Freeform,
}

impl Kind {
    pub fn name(&self) -> &'static str {
        match self {
            Kind::Plane => "平面",
            Kind::Cylinder => "圆柱",
            Kind::Freeform => "自由曲面",
        }
    }

    /// 形位误差的名字（自由曲面没有）。
    pub fn form_name(&self) -> &'static str {
        match self {
            Kind::Plane => "平面度",
            Kind::Cylinder => "圆柱度",
            Kind::Freeform => "—",
        }
    }
}

/// 一个特征上的评定结果。距离单位 mm。
#[derive(Clone, Debug)]
pub struct FeatureFit {
    pub kind: Kind,
    pub points: usize,
    pub area_mm2: f64,
    /// 最小区域法形位误差（区域宽度）。`Freeform` 时为 `NaN`。
    pub form_mz: f64,
    /// 最小二乘基准下的同一误差。按定义 `≥ form_mz`。
    pub form_lsq: f64,
    /// 基准：平面=法向，圆柱=轴向。
    pub axis: DVec3,
    /// 基准上的一点（平面=面心，圆柱=轴上一点）。
    pub anchor: DVec3,
    /// 圆柱半径（平面为 `NaN`）。
    pub radius: f64,
    /// 偏差统计（有符号，正 = 过切）。
    pub mean_dev: f64,
    pub max_gouge: f64,
    pub max_residual: f64,
    /// 法向一致性（0..1）：越小越不像单一特征。
    pub normal_tightness: f64,
    pub lo: DVec3,
    pub hi: DVec3,
}

/// 全件报告。
#[derive(Clone, Debug, Default)]
pub struct FormReport {
    pub features: Vec<FeatureFit>,
    pub total_area_mm2: f64,
    /// 因面积过小被丢掉的分片数。
    pub skipped: usize,
    /// 归到 `Freeform` 的面积占比。
    pub freeform_frac: f64,
}

/// 参数。
#[derive(Clone, Copy, Debug)]
pub struct FormConfig {
    /// 小于该面积的分片不评定（噪声片没有统计意义）。
    pub min_area_mm2: f64,
    /// 判"平面"的法向最大夹角（度）。
    pub plane_deg: f64,
    /// 判"圆柱"时允许的法向与轴夹角偏差（`|n·axis|` 上限）。
    pub cylinder_axis_tol: f64,
    /// 每个特征参与拟合的最大点数（面积等距抽样）。
    pub max_fit_points: usize,
    /// 最多报几个特征（按面积降序）。
    pub max_features: usize,
}

impl Default for FormConfig {
    fn default() -> Self {
        Self {
            min_area_mm2: 20.0,
            plane_deg: 8.0,
            cylinder_axis_tol: 0.15,
            max_fit_points: 4000,
            max_features: 6,
        }
    }
}

/// 评一遍所有特征。
///
/// `samples` 是设计面采样 `(位置, 单位外法向, 面积)`；`grid` 是仿真后的实际毛坯场。
pub fn analyze(
    samples: &[(DVec3, DVec3, f64)],
    grid: &SparseGrid<i8>,
    cfg: &FormConfig,
) -> FormReport {
    let mut rep = FormReport::default();
    if samples.is_empty() {
        return rep;
    }
    // ① 把设计面采样点投影到**实际表面**：q = p + d·∇d/|∇d|
    let eps = grid.config.h.max(1e-3) * 0.5;
    let mut q: Vec<DVec3> = Vec::with_capacity(samples.len());
    let mut dev: Vec<f64> = Vec::with_capacity(samples.len());
    let mut total = 0.0;
    for (p, _, a) in samples {
        let d = grid.sdf(*p);
        let g = grad(grid, *p, eps);
        let gl = g.length();
        q.push(if gl > 1e-12 { *p + g * (d / gl) } else { *p });
        dev.push(d);
        total += a;
    }
    rep.total_area_mm2 = total;

    // ② 空间连通分片（格距跟采样间距走，否则一片会碎成几十片）
    let spacing = (total / samples.len() as f64).sqrt();
    let segs = segment(samples, spacing * 2.5, 1);

    let mut out: Vec<FeatureFit> = Vec::new();
    let mut free_area = 0.0;
    for seg in segs {
        let area: f64 = seg.iter().map(|&i| samples[i].2).sum();
        if area < cfg.min_area_mm2 {
            rep.skipped += 1;
            continue;
        }
        // 面积加权平均法向 + 一致性
        let mut nsum = DVec3::ZERO;
        for &i in &seg {
            nsum += samples[i].1 * samples[i].2;
        }
        let nl = nsum.length();
        let mean_n = if nl > 1e-12 { nsum / nl } else { DVec3::ZERO };
        let mut tight = 0.0;
        for &i in &seg {
            tight += samples[i].1.dot(mean_n).abs() * samples[i].2;
        }
        let normal_tightness = if area > 0.0 { tight / area } else { 0.0 };

        // 抽样（面积等距）：形位公差是全局量
        let stride = (seg.len() / cfg.max_fit_points.max(1)).max(1);
        let pick: Vec<usize> = seg.iter().copied().step_by(stride).collect();
        let pts: Vec<DVec3> = pick.iter().map(|&i| q[i]).collect();
        let nrm: Vec<DVec3> = pick.iter().map(|&i| samples[i].1).collect();

        let mut f = FeatureFit {
            kind: Kind::Freeform,
            points: seg.len(),
            area_mm2: area,
            form_mz: f64::NAN,
            form_lsq: f64::NAN,
            axis: mean_n,
            anchor: centroid(&pts),
            radius: f64::NAN,
            mean_dev: 0.0,
            max_gouge: 0.0,
            max_residual: 0.0,
            normal_tightness,
            lo: samples[seg[0]].0,
            hi: samples[seg[0]].0,
        };
        let mut dsum = 0.0;
        let mut wsum = 0.0;
        for &i in &seg {
            let d = dev[i];
            let a = samples[i].2;
            dsum += d * a;
            wsum += a;
            f.max_gouge = f.max_gouge.max(d.max(0.0));
            f.max_residual = f.max_residual.max((-d).max(0.0));
            let p = samples[i].0;
            f.lo = DVec3::new(f.lo.x.min(p.x), f.lo.y.min(p.y), f.lo.z.min(p.z));
            f.hi = DVec3::new(f.hi.x.max(p.x), f.hi.y.max(p.y), f.hi.z.max(p.z));
        }
        f.mean_dev = if wsum > 0.0 { dsum / wsum } else { 0.0 };

        // ③ 分类 → 评形位
        let plane_cos = cfg.plane_deg.to_radians().cos();
        if normal_tightness >= plane_cos {
            let (n, mz, lsq) = plane_min_zone(&pts);
            f.kind = Kind::Plane;
            f.axis = n;
            f.form_mz = mz;
            f.form_lsq = lsq;
        } else if let Some((axis, mz, lsq, r)) =
            cylinder_min_zone(&pts, &nrm, cfg.cylinder_axis_tol)
        {
            f.kind = Kind::Cylinder;
            f.axis = axis;
            f.form_mz = mz;
            f.form_lsq = lsq;
            f.radius = r;
        } else {
            free_area += area;
        }
        out.push(f);
    }

    out.sort_by(|a, b| b.area_mm2.total_cmp(&a.area_mm2));
    out.truncate(cfg.max_features);
    rep.freeform_frac = if total > 0.0 { free_area / total } else { 0.0 };
    rep.features = out;
    rep
}

/// **最小区域法平面度**：`min over n of (max n·q − min n·q)`。
///
/// 返回 `(法向, MZ 平面度, 同一批点在 LSQ 法向下的平面度)`。
/// 按定义右侧 >= 左侧（LSQ 高估）。
pub fn plane_min_zone(pts: &[DVec3]) -> (DVec3, f64, f64) {
    if pts.len() < 3 {
        return (DVec3::new(0.0, 0.0, 1.0), 0.0, 0.0);
    }
    let c = centroid(pts);
    let cov = cov_of(pts, c, None);
    let lsq_n = smallest_eigenvector(cov);
    let lsq = spread(pts, lsq_n);
    let (n, mz) = pattern_dir(lsq_n, |d| spread(pts, d));
    (n, mz, lsq)
}

/// **最小区域法圆柱度**：`min over (轴方向, 轴位置) of (max r_i − min r_i)`。
///
/// 轴向种子来自法向协方差的最小特征向量（圆柱的法向都与轴垂直）。
/// 返回 `(轴, MZ 圆柱度, 同一轴上的 LSQ 值, 半径)`；不像圆柱返回 `None`。
pub fn cylinder_min_zone(
    pts: &[DVec3],
    normals: &[DVec3],
    axis_tol: f64,
) -> Option<(DVec3, f64, f64, f64)> {
    if pts.len() < 20 || normals.len() != pts.len() {
        return None;
    }
    let c = centroid(pts);
    // 法向的外积和：圆柱面 → 轴是"法向最不分散"的方向
    let mut m = [[0.0f64; 3]; 3];
    for n in normals {
        let v = [n.x, n.y, n.z];
        for a in 0..3 {
            for b in 0..3 {
                m[a][b] += v[a] * v[b];
            }
        }
    }
    let axis0 = smallest_eigenvector(m);
    // 校验：法向确实与轴垂直
    let mut bad = 0usize;
    for n in normals {
        if n.dot(axis0).abs() > axis_tol {
            bad += 1;
        }
    }
    if bad * 20 > normals.len() {
        return None; // 超过 5% 的法向不与轴垂直 → 不是圆柱
    }
    // 4 参数模式搜索：(轴方向 2 dof) + (轴位置 2 dof)。两个目标各搜一次：
    // 最小区域（验收值）与最小二乘（对照值）。
    let (axis, anchor, mz) = search_cylinder(pts, axis0, c);
    // LSQ 搜索**从最小区域的解出发**：模式搜索的步长会衰减，从形心起步时
    // （轴穿在曲面片里，径向距离从 0 到 2R）会卡在半路——实测方差停在 0.40
    // 而真解是 0.004，报出来的"LSQ 圆柱度"2.98 mm 比最小区域大 16 倍。
    let (lax, lanchor, _) = search_cylinder_with(pts, axis, anchor, true);
    // LSQ 值 = **在最小二乘轴上**量的区域宽度（ISO 的报告惯例）。
    // 早先这里量的是"种子轴 + 形心"的区域宽度，标签写着 LSQ——那不是 LSQ，
    // 实测在 1311 mm² 的圆角片上给出 20.172 mm（同一片最小区域才 0.914）。
    let lsq = spread_radial(pts, lax, lanchor);
    let r = pts
        .iter()
        .map(|p| dist_to_line(*p, anchor, axis))
        .sum::<f64>()
        / pts.len() as f64;

    // 三条否决条件。少了任何一条，**平面会被判成半径无穷大的圆柱**：
    // 轴挪得越远，平面上一片的到轴距离就越接近常数（极差 ~ w²/2R → 0），
    // 形式上也"很像圆柱"。这也是"圆柱度/平面度"在退化情形下会混淆的根源。
    let (mut lo, mut hi) = (pts[0], pts[0]);
    for p in pts {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let extent = (hi - lo).length().max(1e-6);
    if (anchor - c).length() > 2.5 * extent {
        return None; // 轴跑到特征外面去了：不是圆柱
    }
    if !mz.is_finite() || mz > 0.05 * r {
        return None; // 形位误差相对半径太大：不是圆柱（平面/自由曲面）
    }
    if sectors(pts, axis, anchor, 12) < 4 {
        return None; // 角向只覆盖不到 90°：一条平面带，不是圆柱
    }
    Some((axis, mz, lsq, r))
}

// ─────────────────────────── 拟合内核 ───────────────────────────

fn centroid(pts: &[DVec3]) -> DVec3 {
    let mut s = DVec3::ZERO;
    for p in pts {
        s += *p;
    }
    s / pts.len() as f64
}

fn cov_of(pts: &[DVec3], c: DVec3, w: Option<&[f64]>) -> [[f64; 3]; 3] {
    let mut m = [[0.0f64; 3]; 3];
    let mut wsum = 0.0;
    for (i, p) in pts.iter().enumerate() {
        let a = w.map_or(1.0, |v| v[i]);
        let d = [p.x - c.x, p.y - c.y, p.z - c.z];
        for (r, row) in m.iter_mut().enumerate() {
            for (s, cell) in row.iter_mut().enumerate() {
                *cell += a * d[r] * d[s];
            }
        }
        wsum += a;
    }
    if wsum > 0.0 {
        for row in m.iter_mut() {
            for cell in row.iter_mut() {
                *cell /= wsum;
            }
        }
    }
    m
}

/// 给定法向，点在法向上的投影极差 = 两张平行平面的最小间距。
fn spread(pts: &[DVec3], n: DVec3) -> f64 {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for p in pts {
        let v = p.dot(n);
        lo = lo.min(v);
        hi = hi.max(v);
    }
    (hi - lo).max(0.0)
}

fn dist_to_line(p: DVec3, a: DVec3, d: DVec3) -> f64 {
    let v = p - a;
    (v - d * v.dot(d)).length()
}

/// 给定轴，径向距离的极差 = 两个同轴圆柱的最小壁厚差。
fn spread_radial(pts: &[DVec3], axis: DVec3, anchor: DVec3) -> f64 {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for p in pts {
        let r = dist_to_line(*p, anchor, axis);
        lo = lo.min(r);
        hi = hi.max(r);
    }
    (hi - lo).max(0.0)
}

/// 在单位球面上做模式搜索：`min f(d)`，`d` 用两个正交方向上的旋转生成邻居。
fn pattern_dir<F: Fn(DVec3) -> f64>(seed: DVec3, f: F) -> (DVec3, f64) {
    let mut best = seed.normalize();
    let mut best_v = f(best);
    let mut step: f64 = 0.15;
    for _ in 0..400 {
        if step < 1e-9 {
            break;
        }
        let prev_v = best_v;
        // 以 best 为极点的两个正交切向
        let helper = if best.z.abs() < 0.9 {
            DVec3::new(0.0, 0.0, 1.0)
        } else {
            DVec3::new(1.0, 0.0, 0.0)
        };
        let e1 = helper.cross(best).normalize();
        let e2 = best.cross(e1);
        let mut moved = false;
        // **八个方向，不只四个坐标轴方向**：`spread` 是分段线性的（由极值点决定），
        // 坐标下降会在"四个方向都不降、对角方向才降"的折点停住——实测差 0.36%，
        // 被随机采样抓出来。加对角方向即可。
        let dirs = [
            (e1, 1.0),
            (e1, -1.0),
            (e2, 1.0),
            (e2, -1.0),
            ((e1 + e2).normalize(), 1.0),
            ((e1 + e2).normalize(), -1.0),
            ((e1 - e2).normalize(), 1.0),
            ((e1 - e2).normalize(), -1.0),
        ];
        for (ax, sgn) in dirs {
            let cand = (best * step.cos() + ax * (sgn * step.sin())).normalize();
            let v = f(cand);
            if v < best_v - 1e-12 {
                best = cand;
                best_v = v;
                moved = true;
            }
        }
        // **不只是"走不动"才缩步长**：每次只改进千分之一也缩，否则会在最优点
        // 附近以 0.02 rad 的步长来回跳，停在一个比真解差 0.4% 的地方
        // （实测被随机采样抓出来过）。
        if !moved || (prev_v - best_v) <= 1e-3 * prev_v.abs() {
            step *= 0.5;
        }
    }
    (best, best_v)
}

/// 4 参数（轴方向 2 + 轴位置 2）模式搜索。
/// 径向偏差的**方差**（LSQ 目标）。最小二乘圆柱 = 让这个量为最小的轴。
///
/// 和 [`spread_radial`]（区域宽度，符合 ISO 1101 最小区域）是两个口径：
/// 最小区域量的是"夹住用的两层圆柱多厚"，最小二乘量的是"残差平方和最小"。
/// 验收用前者（它才是公差带），后者只作对照。
fn var_radial(pts: &[DVec3], axis: DVec3, anchor: DVec3) -> f64 {
    let r: Vec<f64> = pts.iter().map(|p| dist_to_line(*p, anchor, axis)).collect();
    let mean = r.iter().sum::<f64>() / r.len() as f64;
    r.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / r.len() as f64
}

fn search_cylinder(pts: &[DVec3], axis0: DVec3, c0: DVec3) -> (DVec3, DVec3, f64) {
    search_cylinder_with(pts, axis0, c0, false)
}

/// 同一次模式搜索，`var = true` 时目标换成径向**方差**（最小二乘轴）。
fn search_cylinder_with(
    pts: &[DVec3],
    axis0: DVec3,
    c0: DVec3,
    var: bool,
) -> (DVec3, DVec3, f64) {
    let cost = |a: DVec3, p: DVec3| {
        if var {
            var_radial(pts, a, p)
        } else {
            spread_radial(pts, a, p)
        }
    };
    let mut axis = axis0.normalize();
    let mut anchor = c0;
    let mut best = cost(axis, anchor);
    let mut step: f64 = 0.1;
    let mut step_pos: f64 = 0.1;
    for _ in 0..200 {
        if step < 1e-5 && step_pos < 1e-5 {
            break;
        }
        let helper = if axis.z.abs() < 0.9 {
            DVec3::new(0.0, 0.0, 1.0)
        } else {
            DVec3::new(1.0, 0.0, 0.0)
        };
        let e1 = helper.cross(axis).normalize();
        let e2 = axis.cross(e1);
        let mut moved = false;
        // 方向
        for (ax, sgn) in [(e1, 1.0), (e1, -1.0), (e2, 1.0), (e2, -1.0)] {
            let cand = (axis * step.cos() + ax * (sgn * step.sin())).normalize();
            let v = cost(cand, anchor);
            if v < best - 1e-12 {
                axis = cand;
                best = v;
                moved = true;
            }
        }
        // 位置（沿垂直轴的两个方向平移；沿轴平移不改变径向距离）
        for (ax, sgn) in [(e1, 1.0), (e1, -1.0), (e2, 1.0), (e2, -1.0)] {
            let cand = anchor + ax * (sgn * step_pos);
            let v = cost(axis, cand);
            if v < best - 1e-12 {
                anchor = cand;
                best = v;
                moved = true;
            }
        }
        if !moved {
            step *= 0.5;
            step_pos *= 0.5;
        }
    }
    (axis, anchor, best)
}

/// 点绕轴的角向覆盖：返回 12 个 30° 扇区里被占用的个数。
///
/// 平面带与圆柱的区别不只是"距离差不多"，还有**是否绕过去**：
/// 一条 10×8 的平面带只覆盖 2~3 个扇区，真圆柱覆盖全部。
fn sectors(pts: &[DVec3], axis: DVec3, anchor: DVec3, n: usize) -> usize {
    let helper = if axis.z.abs() < 0.9 {
        DVec3::new(0.0, 0.0, 1.0)
    } else {
        DVec3::new(1.0, 0.0, 0.0)
    };
    let e1 = helper.cross(axis).normalize();
    let e2 = axis.cross(e1);
    let mut seen = vec![false; n];
    for p in pts {
        let v = *p - anchor;
        let a = v.dot(e2).atan2(v.dot(e1));
        let k = ((a + std::f64::consts::PI) / std::f64::consts::TAU * n as f64) as usize;
        seen[k.min(n - 1)] = true;
    }
    seen.iter().filter(|b| **b).count()
}

fn grad(grid: &SparseGrid<i8>, p: DVec3, e: f64) -> DVec3 {
    let dx = grid.sdf(p + DVec3::new(e, 0.0, 0.0)) - grid.sdf(p - DVec3::new(e, 0.0, 0.0));
    let dy = grid.sdf(p + DVec3::new(0.0, e, 0.0)) - grid.sdf(p - DVec3::new(0.0, e, 0.0));
    let dz = grid.sdf(p + DVec3::new(0.0, 0.0, e)) - grid.sdf(p - DVec3::new(0.0, 0.0, e));
    DVec3::new(dx, dy, dz) / (2.0 * e)
}

/// 3×3 对称矩阵的**最小**特征向量（Jacobi 迭代，纯 f64，20 行）。
///
/// 下标循环是 Jacobi 旋转的本体（要同时改第 p/q 行列），不是"能用迭代器"的情形。
#[allow(clippy::needless_range_loop)]
fn smallest_eigenvector(m: [[f64; 3]; 3]) -> DVec3 {
    let mut a = m;
    let mut v = [[1.0f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        // 最大非对角元
        let (mut p, mut q, mut mx) = (0usize, 1usize, 0.0f64);
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > mx {
                    mx = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if mx < 1e-14 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        for k in 0..3 {
            let akp = a[k][p];
            let akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for k in 0..3 {
            let apk = a[p][k];
            let aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
        for k in 0..3 {
            let vkp = v[k][p];
            let vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let mut best = 0usize;
    for i in 1..3 {
        if a[i][i] < a[best][best] {
            best = i;
        }
    }
    let e = DVec3::new(v[0][best], v[1][best], v[2][best]);
    let l = e.length();
    if l > 1e-12 {
        e / l
    } else {
        DVec3::new(0.0, 0.0, 1.0)
    }
}

/// 空间连通分片（粗网格 6 邻域并查集）。格距必须**大于**采样间距，否则一片会碎。
fn segment(samples: &[(DVec3, DVec3, f64)], cell: f64, min_pts: usize) -> Vec<Vec<usize>> {
    let cell = cell.max(0.1);
    let (mut lo, mut hi) = (samples[0].0, samples[0].0);
    for (p, _, _) in samples {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let n = [
        ((hi.x - lo.x) / cell).ceil() as usize + 1,
        ((hi.y - lo.y) / cell).ceil() as usize + 1,
        ((hi.z - lo.z) / cell).ceil() as usize + 1,
    ];
    let cells = n[0] * n[1] * n[2];
    let key = |i: usize, j: usize, k: usize| (k * n[1] + j) * n[0] + i;
    let at = |v: f64, o: f64, ax: usize| {
        (((v - o) / cell).floor() as i64).clamp(0, n[ax] as i64 - 1) as usize
    };
    let mut cell_of: Vec<usize> = Vec::with_capacity(samples.len());
    let mut occupied = vec![false; cells];
    // 每格的法向（面积加权）：相邻面在棱边处法向突变，不能只按位置连通
    let mut cn = vec![DVec3::ZERO; cells];
    for (p, n, a) in samples {
        let c = key(at(p.x, lo.x, 0), at(p.y, lo.y, 1), at(p.z, lo.z, 2));
        cell_of.push(c);
        occupied[c] = true;
        cn[c] += *n * *a;
    }
    let unit = |v: DVec3| {
        let l = v.length();
        if l > 1e-12 { v / l } else { DVec3::ZERO }
    };
    // 12°：能分开盒子相邻面（棱边处法向经混合格过渡，差 90° 走不通）。
    //
    // **代价（实测）**：曲率大的圆柱会被拆成条带——格距 3.75 mm 在 r=10 上对应
    // 21.5° 的法向差，超过 12° 就连不起来，孔壁整片退化成"自由曲面"。
    // 24° 能连上圆柱但会把 90° 的相邻面并成一片（自由曲面占比从 39% 涨到 55%），
    // 两头都不划算。要同时成立得做"按曲率自适应"，目前没做——这是已知限制。
    let cos_link = 12.0f64.to_radians().cos();
    let mut parent: Vec<usize> = (0..cells).collect();
    fn find(p: &mut [usize], mut x: usize) -> usize {
        while p[x] != x {
            p[x] = p[p[x]];
            x = p[x];
        }
        x
    }
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let c = key(i, j, k);
                if !occupied[c] {
                    continue;
                }
                for (di, dj, dk) in [(1usize, 0usize, 0usize), (0, 1, 0), (0, 0, 1)] {
                    let (ii, jj, kk) = (i + di, j + dj, k + dk);
                    if ii >= n[0] || jj >= n[1] || kk >= n[2] {
                        continue;
                    }
                    let o = key(ii, jj, kk);
                    if !occupied[o] {
                        continue;
                    }
                    // 法向差太多的两格不连（棱边两侧属于不同特征）
                    if unit(cn[c]).dot(unit(cn[o])) < cos_link {
                        continue;
                    }
                    let (a, b) = (find(&mut parent, c), find(&mut parent, o));
                    if a != b {
                        parent[a] = b;
                    }
                }
            }
        }
    }
    // 按根聚合；用 cell 根做索引
    let mut slot: Vec<Option<usize>> = vec![None; cells];
    let mut out: Vec<Vec<usize>> = Vec::new();
    for (si, &c) in cell_of.iter().enumerate() {
        let r = find(&mut parent, c);
        let k = match slot[r] {
            Some(k) => k,
            None => {
                out.push(Vec::new());
                slot[r] = Some(out.len() - 1);
                out.len() - 1
            }
        };
        out[k].push(si);
    }
    out.retain(|v| v.len() >= min_pts);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid_plane(tilt: f64, bump: Option<(usize, f64)>) -> Vec<DVec3> {
        let mut pts = Vec::new();
        for i in 0..20 {
            for j in 0..20 {
                let x = i as f64 * 0.5;
                let y = j as f64 * 0.5;
                // 倾斜平面 z = tilt * x
                let mut z = tilt * x;
                if let Some((k, h)) = bump
                    && i * 20 + j == k
                {
                    z += h;
                }
                pts.push(DVec3::new(x, y, z));
            }
        }
        pts
    }

    /// **倾斜但共面 → 平面度 0**：拟合不能假定"平面是水平的"。
    #[test]
    fn a_tilted_plane_has_zero_flatness() {
        let pts = grid_plane(0.5, None); // 26.6° 斜面
        let (n, mz, lsq) = plane_min_zone(&pts);
        assert!(mz < 1e-9, "共面点平面度应为 0，实际 {mz}");
        assert!(lsq < 1e-9, "LSQ 同样应为 0：{lsq}");
        // 法向应是 (-0.5, 0, 1)/|·|
        let want = DVec3::new(-0.5, 0.0, 1.0).normalize();
        assert!(n.dot(want).abs() > 0.999, "法向错了：{n:?} 应为 {want:?}");
    }

    /// **平面度是"区域宽度"**：棋盘式上下两层（各占一半、交错分布），
    /// 任何倾斜都会让一层的极差变大，所以最小区域就是 h。
    #[test]
    fn flatness_matches_a_two_level_checkerboard() {
        let h = 0.05;
        let mut pts = Vec::new();
        for i in 0..20 {
            for j in 0..20 {
                let z = if (i + j) % 2 == 0 { 0.0 } else { h };
                pts.push(DVec3::new(i as f64 * 0.5, j as f64 * 0.5, z));
            }
        }
        let (_, mz, lsq) = plane_min_zone(&pts);
        assert!(
            (mz - h).abs() < 2e-3,
            "交错两层 {h} → 平面度应为 {h}，实际 {mz}"
        );
        assert!(lsq >= mz - 1e-9, "LSQ 不该小于最小区域：{lsq} vs {mz}");
    }

    /// **搜索真的逼近了最优**：在 LSQ 法向附近随机撒 400 个方向，
    /// 没有一个能比模式搜索给出的区域更窄。
    ///
    /// 这条比"LSQ 一定大于 MZ"更能证伪：如果搜索方向反了、步长不收敛、
    /// 或者把"区域宽度"算成了"最大偏差"，随机采样立刻就能找到更好的解。
    /// （`form_lsq >= form_mz` 是**构造上**成立的——MZ 是从 LSQ 出发做下降，
    /// 所以那一半不构成独立证据。）
    #[test]
    fn the_pattern_search_is_near_optimal() {
        // 0.1 的孤立凸起 + 一个 0.05 的凹坑（不对称，避免与 LSQ 重合）
        let mut pts = Vec::new();
        for i in 0..20 {
            for j in 0..20 {
                let mut z = 0.0;
                if i == 3 && j == 4 {
                    z = 0.1;
                }
                if i == 16 && j == 12 {
                    z = -0.05;
                }
                pts.push(DVec3::new(i as f64 * 0.5, j as f64 * 0.5, z));
            }
        }
        let (n, mz, lsq) = plane_min_zone(&pts);
        assert!(lsq >= mz - 1e-12, "LSQ 不该小于最小区域");
        // 返回的法向必须真的达到所报的区域宽度
        assert!(
            (spread(&pts, n) - mz).abs() < 1e-9,
            "报的 {mz} 与按返回法向重算的 {} 不一致",
            spread(&pts, n)
        );
        // 随机采样：不该有更好的方向
        let helper = if n.z.abs() < 0.9 {
            DVec3::new(0.0, 0.0, 1.0)
        } else {
            DVec3::new(1.0, 0.0, 0.0)
        };
        let e1 = helper.cross(n).normalize();
        let e2 = n.cross(e1);
        let mut seed = 12345u64;
        let mut better = None;
        for _ in 0..400 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (seed >> 11) as f64 / (1u64 << 53) as f64;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let v = (seed >> 11) as f64 / (1u64 << 53) as f64;
            let (a, b) = ((u - 0.5) * 0.4, (v - 0.5) * 0.4);
            let cand = (n + e1 * a + e2 * b).normalize();
            let sp = spread(&pts, cand);
            if sp < mz - 1e-9 {
                better = Some((a, b, sp));
                break;
            }
        }
        assert!(
            better.is_none(),
            "有方向比模式搜索更好：{better:?}（报的 mz={mz}）"
        );
    }

    /// 圆柱：**二阶**径向波动（椭圆）±0.03 → 圆柱度 0.06。
    ///
    /// 为什么不用一阶（`cosθ`）：一阶是**偏心**——把轴平移 a 就能完全消掉，
    /// 按 ISO 1101 它属于位置误差而不是形状误差。这正好说明"轴可自由平移"
    /// 的拟合是对的行为，见下一条测试。
    #[test]
    fn cylinder_form_error_is_the_radial_zone() {
        let (r, amp) = (10.0, 0.03);
        let mut pts = Vec::new();
        let mut nrm = Vec::new();
        for i in 0..96 {
            let th = i as f64 / 96.0 * std::f64::consts::TAU;
            for j in 0..20 {
                let z = j as f64 * 0.5;
                let rr = r + amp * (2.0 * th).cos();
                let p = DVec3::new(rr * th.cos(), rr * th.sin(), z);
                pts.push(p);
                nrm.push(DVec3::new(th.cos(), th.sin(), 0.0));
            }
        }
        let (axis, mz, lsq, radius) = cylinder_min_zone(&pts, &nrm, 0.15).expect("应识别为圆柱");
        assert!(axis.z.abs() > 0.99, "轴向应为 +z：{axis:?}");
        assert!(
            (mz - 2.0 * amp).abs() < 0.25 * 2.0 * amp,
            "圆柱度应≈2×振幅={:.3}，实际 {mz:.4}",
            2.0 * amp
        );
        assert!(lsq >= mz - 1e-9, "LSQ 不该小于最小区域");
        assert!((radius - r).abs() < 0.05, "半径应≈{r}：{radius}");
    }

    /// LSQ 值必须**在最小二乘轴上**量，不能拿种子轴凑数。
    ///
    /// 回归测试：桌腿件的 1311 mm² 圆角片曾被报成"圆柱度 0.914 / LSQ 20.172"——
    /// LSQ 量的是"种子轴 + 形心"的区域宽度，轴歪着穿过那段圆弧，径向距离
    /// 从 3 mm 变到 23 mm。LSQ 值只该比最小区域略大，不该差 20 倍。
    #[test]
    fn lsq_value_is_measured_on_the_lsq_axis() {
        let (r, amp) = (10.0, 0.4);
        let mut pts = Vec::new();
        let mut nrm = Vec::new();
        // 只铺 70° 的弧（圆角片的典型角向覆盖），再加一点轴向倾斜
        for i in 0..70 {
            let th = -0.6 + i as f64 / 69.0 * 1.2;
            for j in 0..25 {
                let z = j as f64 * 0.4;
                let rr = r + amp * (3.0 * th).sin();
                let p = DVec3::new(rr * th.cos(), rr * th.sin(), z + 0.05 * rr * th.sin());
                pts.push(p);
                nrm.push(DVec3::new(th.cos(), th.sin(), 0.0));
            }
        }
        let (_, mz, lsq, _) = cylinder_min_zone(&pts, &nrm, 0.3).expect("应识别为圆柱");
        assert!(lsq >= mz - 1e-9, "LSQ 不该小于最小区域：{lsq} vs {mz}");
        assert!(
            lsq < mz + 2.0,
            "LSQ 与最小区域必须同量级（早先种子轴版本会差 20 倍）：{lsq} vs {mz}"
        );
    }

    /// **一阶波动是偏心，不是形状误差**：轴平移 a 就能消掉，圆柱度应≈0。
    #[test]
    fn a_first_harmonic_wobble_is_eccentricity_not_form_error() {
        let (r, a) = (10.0, 0.03);
        let mut pts = Vec::new();
        let mut nrm = Vec::new();
        for i in 0..96 {
            let th = i as f64 / 96.0 * std::f64::consts::TAU;
            for j in 0..20 {
                let z = j as f64 * 0.5;
                let rr = r + a * th.cos();
                pts.push(DVec3::new(rr * th.cos(), rr * th.sin(), z));
                nrm.push(DVec3::new(th.cos(), th.sin(), 0.0));
            }
        }
        let (_, mz, _, _) = cylinder_min_zone(&pts, &nrm, 0.15).expect("应识别为圆柱");
        assert!(mz < 0.2 * a, "偏心不该算进形状误差（应≈0），实际 {mz:.4}");
    }

    /// **不是圆柱的东西不能被报成圆柱**：平面点集应当被拒。
    #[test]
    fn a_plane_is_not_accepted_as_a_cylinder() {
        let pts = grid_plane(0.0, None);
        let nrm = vec![DVec3::new(0.0, 0.0, 1.0); pts.len()];
        assert!(
            cylinder_min_zone(&pts, &nrm, 0.15).is_none(),
            "法向全平行，不该判成圆柱"
        );
    }

    /// 端到端：一块盒形料自比 → 至少 3 个平面特征，平面度接近 0。
    #[test]
    fn a_box_self_comparison_yields_flat_faces() {
        use cncsim_voxel::stl::Tri;
        use cncsim_voxel::{GridConfig, ValueType};
        let p = |x, y, z| DVec3::new(x, y, z);
        let c = [
            p(0.0, 0.0, 0.0),
            p(10.0, 0.0, 0.0),
            p(10.0, 8.0, 0.0),
            p(0.0, 8.0, 0.0),
            p(0.0, 0.0, 6.0),
            p(10.0, 0.0, 6.0),
            p(10.0, 8.0, 6.0),
            p(0.0, 8.0, 6.0),
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
        let tris: Vec<Tri> = f.iter().map(|q| [c[q[0]], c[q[1]], c[q[2]]]).collect();
        let h = 0.5;
        let lo = DVec3::new(-2.0, -2.0, -2.0);
        let dom = [28, 24, 20];
        let grid = cncsim_voxel::meshvol::mesh_to_field(
            &tris,
            GridConfig {
                h,
                band_voxels: 4,
                value_type: ValueType::I8,
            },
            lo,
            dom,
        );
        let samples = cncsim_voxel::meshvol::surface_samples(&tris, h);
        let rep = analyze(&samples, &grid, &FormConfig::default());
        let planes = rep
            .features
            .iter()
            .filter(|f| f.kind == Kind::Plane)
            .count();
        assert!(planes >= 3, "盒形料应识别出多个平面，实际 {planes}");
        for f in &rep.features {
            if f.kind == Kind::Plane {
                assert!(
                    f.form_mz < 0.2,
                    "自比平面度应很小，实际 {:.3}（{} mm²）",
                    f.form_mz,
                    f.area_mm2
                );
                assert!(
                    f.form_lsq >= f.form_mz - 1e-9,
                    "LSQ 不该小于 MZ：{} vs {}",
                    f.form_lsq,
                    f.form_mz
                );
            }
        }
    }
}

