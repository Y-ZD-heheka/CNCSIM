//! 可达性：在给定的工序方向集合下，哪些设计面**一次装夹也够不到**。
//!
//! 这是把「残料 1.35 mm」这类读数拆成两种完全不同的原因：
//!
//! | 读数 | 原因 | 处置 |
//! |---|---|---|
//! | 面朝着某个工序方向、射线不被挡，但仍有残料 | **刀路问题**（刀太大、步距太粗、drop-cutter 在凹角让位） | 换刀 / 收紧步距 / 加精加工道 |
//! | 面**背面**或**全向遮挡** | **工艺问题**（几何上够不到） | 加方向、翻面装夹，或走五轴联动 |
//!
//! 判据是几何可视性（射线）。它给出的是一份**乐观上界**：射线不被挡只说明
//! "这一点看得见"，实际还要球头放得进去、刀杆不撞。所以结论的方向是——
//! **"够不到"可信（可以据此改工艺），"够得到"只是必要条件**。
//!
//! 判定用设计 SDF 的**球体追踪**（sphere tracing）：每步前进 `0.8 · sdf`，
//! 因此在开阔空气里几步就能穿出包围盒，只在贴近材料时才细走。0.8 的安全系数
//! 是为三线性插值出来的场留余量（它不严格 1-Lipschitz）。

use crate::compare::DesignModel;
use cncsim_math::DVec3;

/// 可达性分析的参数。
#[derive(Clone, Copy, Debug)]
pub struct ReachConfig {
    /// 面法向与工序方向的**夹角上限**（度）。超过它，这个方向对这一面就是背对。
    ///
    /// 与 CAM 里 `Process::max_finish_slope_deg` 同义：3 轴高度场只能加工
    /// 「法向与刀轴夹角 ≤ 它」的面。设到 90° 就退化成"只要能看见就能加工"。
    pub slope_limit_deg: f64,
    /// 射线的最小步长（mm）。球体追踪的兜底，避免贴表面时步长趋零。
    pub sample_mm: f64,
    /// 判定「仍在料里」的阈值（mm）。小于它算擦边，不算遮挡。
    ///
    /// `DesignModel::Field` 的符号在一个量化档内可能翻转，这里要给足余量。
    pub inside_tol: f64,
}

impl Default for ReachConfig {
    fn default() -> Self {
        Self {
            slope_limit_deg: 45.0,
            sample_mm: 0.5,
            inside_tol: 0.6,
        }
    }
}

/// 一片**空间连通的够不到的面**（同一特征上的一整块，不是散点）。
///
/// 按法向分箱会把整个零件上"朝下的面"归成一组、bbox 铺满全件，没法照着改工艺。
/// 所以先按粗网格做连通块：同一片里的点互相挨着，`lo..hi` 才是"这个特征在哪"。
#[derive(Clone, Copy, Debug)]
pub struct BlockedGroup {
    /// 建议的刀轴方向（这一片法向的面积加权平均）。可直接喂给 `--axis`。
    pub dir: DVec3,
    pub points: usize,
    pub area_mm2: f64,
    /// 这一片的空间范围。
    pub lo: DVec3,
    pub hi: DVec3,
    /// 这一片里"没有方向面朝"的面积占比（1 = 纯背面，0 = 全是被挡）。
    pub backfacing_frac: f64,
    /// 法向的一致性（0..1）：1 = 整片朝同一个方向（补一道那个方向就行）；
    /// ≈0 = 法向互相抵消（**封闭内腔**：任何单一方向都进不去）。
    pub dir_conf: f64,
}

impl BlockedGroup {
    /// 有没有一个可信的"单一建议方向"。封闭内腔/多方向的面会返回 `false`。
    pub fn has_direction(&self) -> bool {
        self.dir_conf > 0.25
    }

    /// 建议方向对应的 A 角（度，Head 约定：`A = acos(axis.z)`）。
    pub fn a_deg(&self) -> f64 {
        self.dir.z.clamp(-1.0, 1.0).acos().to_degrees()
    }

    /// 建议方向对应的 C 角（度，Head 约定：`C = atan2(axis.x, −axis.y)`）。
    pub fn c_deg(&self) -> f64 {
        self.dir.x.atan2(-self.dir.y).to_degrees()
    }
}

/// 可达性报告。面积都按采样面积加权（不是点数——STL 的细分密度和真实面积无关）。
#[derive(Clone, Debug, Default)]
pub struct ReachReport {
    pub points: usize,
    pub total_area_mm2: f64,
    /// 至少有一个工序方向既面朝、射线又通。
    pub reachable: usize,
    pub reachable_area_mm2: f64,
    /// 有方向面朝，但沿每个面朝的方向都被料挡住（凹腔、内孔、封闭内腔）。
    pub occluded: usize,
    pub occluded_area_mm2: f64,
    /// 没有任何工序方向面朝它（背面）。只能翻面或五轴。
    pub backfacing: usize,
    pub backfacing_area_mm2: f64,
    /// 完全够不到的面，按空间连通片分组，面积降序。
    pub groups: Vec<BlockedGroup>,
}

impl ReachReport {
    /// 一次装夹能加工到的面积占比（0..1）。
    pub fn coverage(&self) -> f64 {
        if self.total_area_mm2 <= 0.0 {
            return 1.0;
        }
        self.reachable_area_mm2 / self.total_area_mm2
    }
}

/// 方向位图：最多 64 个候选方向（够用——3+2 的实际工序数远小于这个数）。
pub type DirMask = u64;

/// 「每个设计面样本能被哪些方向看见」的矩阵。
///
/// 这是编排工序的**唯一**昂贵计算（射线求交）。算一次之后：选方向（集合覆盖）、
/// 算任意方向子集的可达率、报遮挡分组，全都只是按位与。
#[derive(Clone, Debug, Default)]
pub struct Visibility {
    pub dirs: Vec<DVec3>,
    /// 第 i 个样本**既面朝又射线通**的方向位图；0 = 这套候选方向都够不到。
    pub masks: Vec<DirMask>,
    /// 第 i 个样本**只判面朝**（不看遮挡）的位图。
    ///
    /// 没有它就分不开两件完全不同的事：`masks == 0 && facing != 0` 是"被挡住"
    /// （加方向/换角度可能救得回来），`facing == 0` 是"背面"（只能翻面或五轴）。
    /// 早先把两者混成一个 0 位图，结果封闭内腔被报成了"背面"。
    pub facing: Vec<DirMask>,
    pub areas: Vec<f64>,
    pub normals: Vec<DVec3>,
    pub points: Vec<DVec3>,
    pub total_area_mm2: f64,
}

impl Visibility {
    pub fn coverage(&self, m: DirMask) -> f64 {
        if self.total_area_mm2 <= 0.0 {
            return 1.0;
        }
        self.area_with(m) / self.total_area_mm2
    }

    pub fn area_with(&self, m: DirMask) -> f64 {
        let mut a = 0.0;
        for (k, ma) in self.masks.iter().enumerate() {
            if ma & m != 0 {
                a += self.areas[k];
            }
        }
        a
    }

    /// 贪心集合覆盖：在 `base` 之外再挑至多 `max_extra` 个方向，每次挑
    /// **新覆盖面积最大**的那个；新增覆盖不足 `min_gain_frac × 总面积` 就停。
    ///
    /// 为什么是贪心而不是精确解：这是带权集合覆盖（NP-hard），而候选只有几十个、
    /// 每个方向值一次换刀加一段仿真时间。贪心的近似比 1−1/e 够用，而且它给出的
    /// 是"再加这一道值不值"的量化依据（新增覆盖面积），不是拍脑袋。
    ///
    /// 返回被选中的方向下标（不含 `base` 里的）。
    pub fn greedy(&self, base: DirMask, max_extra: usize, min_gain_frac: f64) -> Vec<usize> {
        let mut covered = base;
        let mut chosen = Vec::new();
        let floor = self.total_area_mm2 * min_gain_frac;
        for _ in 0..max_extra {
            let mut best = (usize::MAX, 0.0f64);
            for (k, _) in self.dirs.iter().enumerate() {
                if covered & (1u64 << k) != 0 {
                    continue;
                }
                let mut gain = 0.0;
                for (i, m) in self.masks.iter().enumerate() {
                    if m & (1u64 << k) != 0 && m & covered == 0 {
                        gain += self.areas[i];
                    }
                }
                if gain > best.1 {
                    best = (k, gain);
                }
            }
            if best.0 == usize::MAX || best.1 <= 0.0 || best.1 < floor {
                break;
            }
            covered |= 1u64 << best.0;
            chosen.push(best.0);
        }
        chosen
    }

    /// 一组方向下标对应的位图。
    pub fn mask_of(&self, idx: &[usize]) -> DirMask {
        idx.iter().fold(0u64, |m, k| m | (1u64 << k))
    }

    pub fn all_mask(&self) -> DirMask {
        if self.dirs.is_empty() {
            0
        } else {
            (1u64 << self.dirs.len()) - 1
        }
    }
}

/// 算一遍可见性矩阵：`samples` 按 `(位置, 外法向, 面积)`，`dirs` 是候选工序方向。
///
/// 判定 = **面朝**（`n·d ≥ cos(slope_limit)`）且**射线不被挡**。两者缺一不可：
/// 只判面朝会把凹腔里的面算成可达，只判射线会把背面算成可达。
pub fn visibility(
    design: &DesignModel,
    samples: &[(DVec3, DVec3, f64)],
    dirs: &[DVec3],
    cfg: &ReachConfig,
) -> Visibility {
    let dirs: Vec<DVec3> = dirs.iter().map(|d| unit(*d)).take(64).collect();
    let mut out = Visibility {
        dirs: dirs.clone(),
        ..Default::default()
    };
    if samples.is_empty() || dirs.is_empty() {
        return out;
    }
    let cos_lim = cfg.slope_limit_deg.to_radians().cos();
    // 射线预算 = 样本包围盒对角线 ×1.2。用样本自己的范围，不要求调用方传 bbox。
    let (mut lo, mut hi) = (samples[0].0, samples[0].0);
    for (p, _, _) in samples {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let reach = (hi - lo).length().max(1.0) * 1.2;
    out.points.reserve(samples.len());
    out.masks.reserve(samples.len());
    out.areas.reserve(samples.len());
    out.normals.reserve(samples.len());
    out.facing.reserve(samples.len());

    // ── 逐点求可见性：**并行** ──
    //
    // 每个样本只读 `design`/`dirs`，自己算自己的掩码；输出按样本序号归位
    // （`map_indexed` 保证顺序），面积也在同一顺序上累加——所以与串行**逐位
    // 一致**。这是 `工序编排` 那一段（叶轮 4.9 s）的全部成本：158264 点 ×
    // 25 个候选方向的球体追踪，原来一行 `par_iter` 都没有。
    let one = |i: usize| -> Option<(DVec3, DVec3, f64, u64, u64)> {
        let (p, n, area) = samples[i];
        let n = unit(n);
        if n.length() < 0.5 {
            return None; // 退化三角形
        }
        let mut mask = 0u64;
        let mut facing = 0u64;
        for (k, d) in dirs.iter().enumerate() {
            if n.dot(*d) < cos_lim {
                continue;
            }
            facing |= 1u64 << k;
            if clear_along(design, p, *d, cfg, reach) {
                mask |= 1u64 << k;
            }
        }
        Some((p, n, area, mask, facing))
    };
    let nthread = cncsim_voxel::par::worker_count();
    let chunks = (samples.len() / 2000).clamp(1, nthread * 2).max(1);
    let parts: Vec<Vec<Option<(DVec3, DVec3, f64, u64, u64)>>> = if chunks == 1 {
        vec![(0..samples.len()).map(one).collect()]
    } else {
        let per = samples.len().div_ceil(chunks);
        cncsim_voxel::par::map_indexed(chunks, |c| {
            let a = (c * per).min(samples.len());
            let b = ((c + 1) * per).min(samples.len());
            (a..b).map(one).collect()
        })
    };
    for part in parts {
        for rec in part.into_iter().flatten() {
            out.points.push(rec.0);
            out.normals.push(rec.1);
            out.areas.push(rec.2);
            out.masks.push(rec.3);
            out.facing.push(rec.4);
            out.total_area_mm2 += rec.2;
        }
    }
    out
}

/// 对一个设计模型做可达性分析（用**全部**候选方向）。
pub fn analyze(
    design: &DesignModel,
    samples: &[(DVec3, DVec3, f64)],
    dirs: &[DVec3],
    cfg: &ReachConfig,
) -> ReachReport {
    let vis = visibility(design, samples, dirs, cfg);
    let m = vis.all_mask();
    report_of(&vis, m)
}

/// 按**指定的方向子集**出报告。编排完之后用它报"最终这一套工序能覆盖多少"。
pub fn report_with(vis: &Visibility, mask: DirMask) -> ReachReport {
    report_of(vis, mask)
}

fn report_of(vis: &Visibility, mask: DirMask) -> ReachReport {
    let mut rep = ReachReport {
        points: vis.masks.len(),
        total_area_mm2: vis.total_area_mm2,
        ..Default::default()
    };
    if vis.masks.is_empty() {
        return rep;
    }
    let mut blocked: Vec<(DVec3, DVec3, f64, bool)> = Vec::new();
    let (mut lo, mut hi) = (vis.points[0], vis.points[0]);
    for p in &vis.points {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    for i in 0..vis.masks.len() {
        let a = vis.areas[i];
        if vis.masks[i] & mask != 0 {
            rep.reachable += 1;
            rep.reachable_area_mm2 += a;
            continue;
        }
        // 面朝过（facing != 0）却一个方向都不通 → 被遮挡（凹腔/内孔）；
        // 压根没有方向面朝它 → 背面（要翻面或五轴）。
        let any_facing = vis.facing[i] != 0;
        if any_facing {
            rep.occluded += 1;
            rep.occluded_area_mm2 += a;
        } else {
            rep.backfacing += 1;
            rep.backfacing_area_mm2 += a;
        }
        blocked.push((vis.points[i], vis.normals[i], a, !any_facing));
    }
    let spacing = (vis.total_area_mm2 / vis.masks.len().max(1) as f64).sqrt();
    rep.groups = cluster(&blocked, lo, hi, spacing);
    rep
}

/// 把够不到的点按**粗网格 6 邻域连通块**聚成"片"。
///
/// 网格边长取包围盒最长边的 1/24：既能把同一特征上的点连起来，又不会把相邻但
/// 无关的两个面并成一片。面积降序，最多留 6 片（再多也不是"改一处工艺"能解决的）。
fn cluster(
    blocked: &[(DVec3, DVec3, f64, bool)],
    lo: DVec3,
    hi: DVec3,
    spacing: f64,
) -> Vec<BlockedGroup> {
    if blocked.is_empty() {
        return Vec::new();
    }
    let ext = hi - lo;
    let longest = ext.x.max(ext.y).max(ext.z).max(1e-6);
    // 格距 = max(2.5 个采样间距, 最长边/48)，上限 最长边/6（再大就分不开特征了）
    let cell = (spacing * 2.5)
        .max(longest / 48.0)
        .min(longest / 6.0)
        .max(0.2);
    let n = [
        ((ext.x / cell).ceil() as usize + 1).max(1),
        ((ext.y / cell).ceil() as usize + 1).max(1),
        ((ext.z / cell).ceil() as usize + 1).max(1),
    ];
    let cells = n[0] * n[1] * n[2];
    let key = |i: usize, j: usize, k: usize| (k * n[1] + j) * n[0] + i;
    let idx = |v: f64, o: f64, ax: usize| -> usize {
        (((v - o) / cell).floor() as i64).clamp(0, n[ax] as i64 - 1) as usize
    };

    // 每格累加：面积、法向×面积、背面面积、点数
    let mut acc: Vec<(f64, DVec3, f64, usize)> = vec![(0.0, DVec3::ZERO, 0.0, 0); cells];
    for (p, nv, area, back) in blocked {
        let c = key(idx(p.x, lo.x, 0), idx(p.y, lo.y, 1), idx(p.z, lo.z, 2));
        let e = &mut acc[c];
        e.0 += area;
        e.1 += *nv * *area;
        e.2 += if *back { *area } else { 0.0 };
        e.3 += 1;
    }

    // 并查集：只往 +x/+y/+z 三个方向连，反向的那一半会被邻居连到
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
                if acc[c].3 == 0 {
                    continue;
                }
                for (di, dj, dk) in [(1usize, 0usize, 0usize), (0, 1, 0), (0, 0, 1)] {
                    let (ii, jj, kk) = (i + di, j + dj, k + dk);
                    if ii >= n[0] || jj >= n[1] || kk >= n[2] {
                        continue;
                    }
                    let o = key(ii, jj, kk);
                    if acc[o].3 == 0 {
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

    // 汇总到根
    let inf = f64::INFINITY;
    let mut sums: Vec<(f64, DVec3, f64, usize, DVec3, DVec3)> = vec![
        (
            0.0,
            DVec3::ZERO,
            0.0,
            0,
            DVec3::splat(inf),
            DVec3::splat(-inf)
        );
        cells
    ];
    for (c, a) in acc.iter().enumerate() {
        if a.3 == 0 {
            continue;
        }
        let r = find(&mut parent, c);
        let i = c % n[0];
        let j = (c / n[0]) % n[1];
        let k = c / (n[0] * n[1]);
        let (cx, cy, cz) = (
            lo.x + (i as f64 + 0.5) * cell,
            lo.y + (j as f64 + 0.5) * cell,
            lo.z + (k as f64 + 0.5) * cell,
        );
        let s = &mut sums[r];
        s.0 += a.0;
        s.1 += a.1;
        s.2 += a.2;
        s.3 += a.3;
        s.4 = DVec3::new(
            s.4.x.min(cx - cell * 0.5),
            s.4.y.min(cy - cell * 0.5),
            s.4.z.min(cz - cell * 0.5),
        );
        s.5 = DVec3::new(
            s.5.x.max(cx + cell * 0.5),
            s.5.y.max(cy + cell * 0.5),
            s.5.z.max(cz + cell * 0.5),
        );
    }

    // 注意：**不能**因为"平均法向是零向量"就把整片丢掉。封闭内腔的法向正好
    // 互相抵消（球面各向同性），那恰恰是最该报出来的一类——它的结论是
    // "没有单一方向能进去"，而不是"这一片不存在"。
    let mut out: Vec<BlockedGroup> = sums
        .into_iter()
        .filter(|s| s.3 > 0 && s.0 > 0.0)
        .map(|s| {
            let sum = s.1;
            let mag = sum.length() / s.0.max(1e-12);
            BlockedGroup {
                dir: if mag > 0.25 { unit(sum) } else { DVec3::ZERO },
                points: s.3,
                area_mm2: s.0,
                lo: s.4,
                hi: s.5,
                backfacing_frac: (s.2 / s.0).clamp(0.0, 1.0),
                dir_conf: mag.clamp(0.0, 1.0),
            }
        })
        .collect();
    out.sort_by(|a, b| b.area_mm2.total_cmp(&a.area_mm2));
    out.truncate(6);
    out
}

/// 单位化；零长度返回零向量（`DVec3::normalize` 对零向量有 `debug_assert`，
/// 退化三角形/退化法向在这些输入里是真会出现的，不能让它 panic）。
fn unit(v: DVec3) -> DVec3 {
    let l = v.length();
    if l > 1e-12 { v / l } else { DVec3::ZERO }
}

/// 从 `p` 沿 `d` 出去，路上会不会撞进设计材料。
///
/// 球体追踪：设计 SDF 在空气里是到设计面的距离，每步可以安全前进 `0.8 · sdf`。
/// 撞进材料即返回 `false`。
fn clear_along(design: &DesignModel, p: DVec3, d: DVec3, cfg: &ReachConfig, reach: f64) -> bool {
    let mut t = cfg.sample_mm.max(0.2);
    while t <= reach {
        let q = p + d * t;
        let sd = design.sdf(q);
        if sd < -cfg.inside_tol {
            return false;
        }
        // sdf 是"到最近设计面"的距离：开阔处能一次跳很远
        t += (sd * 0.8).max(cfg.sample_mm);
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boxed() -> DesignModel {
        DesignModel::Box {
            min: DVec3::new(-10.0, -10.0, 0.0),
            max: DVec3::new(10.0, 10.0, 10.0),
        }
    }

    /// 盒顶面（法向 +z）在 `+z` 方向下可达；盒底面（法向 −z）是**背面**。
    #[test]
    fn top_is_reachable_and_the_bottom_is_backfacing() {
        let d = boxed();
        let up = DVec3::new(0.0, 0.0, 1.0);
        let samples = vec![
            (DVec3::new(0.0, 0.0, 10.0), up, 1.0),
            (DVec3::new(5.0, -5.0, 10.0), up, 1.0),
            (DVec3::new(0.0, 0.0, 0.0), -up, 1.0),
        ];
        let rep = analyze(&d, &samples, &[up], &ReachConfig::default());
        assert_eq!(rep.reachable, 2, "顶面两点应可达");
        assert_eq!(rep.backfacing, 1, "底面在 +z 下是背面");
        assert_eq!(rep.occluded, 0);
        assert!((rep.coverage() - 2.0 / 3.0).abs() < 1e-9);
    }

    /// **封闭内腔从任何方向都够不到**：盒内挖一个球，球面就是"全向遮挡"。
    ///
    /// 这是最能说明问题的一类：刀路怎么调都没用，必须换工艺。
    #[test]
    fn a_sealed_internal_void_is_occluded_from_every_direction() {
        let d = DesignModel::Cavity {
            min: DVec3::new(-10.0, -10.0, 0.0),
            max: DVec3::new(10.0, 10.0, 10.0),
            center: DVec3::new(0.0, 0.0, 5.0),
            radius: 2.0,
        };
        // 球面采样：材料在球外，所以外法向指向球心
        let mut samples = Vec::new();
        for i in 0..24 {
            for j in 0..12 {
                let th = i as f64 / 24.0 * std::f64::consts::TAU;
                let ph = (j as f64 + 0.5) / 12.0 * std::f64::consts::PI;
                let n = unit(DVec3::new(
                    ph.sin() * th.cos(),
                    ph.sin() * th.sin(),
                    ph.cos(),
                ));
                samples.push((DVec3::new(0.0, 0.0, 5.0) + n * 2.0, -n, 1.0));
            }
        }
        // 26 个方向全给上
        let mut dirs = Vec::new();
        for x in [-1.0, 0.0, 1.0] {
            for y in [-1.0, 0.0, 1.0] {
                for z in [-1.0, 0.0, 1.0] {
                    let v = DVec3::new(x, y, z);
                    if v.length() > 0.5 {
                        dirs.push(v);
                    }
                }
            }
        }
        let rep = analyze(&d, &samples, &dirs, &ReachConfig::default());
        assert_eq!(rep.points, 288);
        assert_eq!(rep.reachable, 0, "封闭内腔不该有可达点");
        assert_eq!(rep.occluded, 288, "全部应判为「有方向面朝但被挡」");
        assert!(rep.coverage() < 1e-9);
        assert_eq!(
            rep.groups.len(),
            1,
            "内腔是一个连通片：{}",
            rep.groups.len()
        );
        let g = &rep.groups[0];
        assert!((g.area_mm2 - 288.0).abs() < 1e-6);
        assert!(g.backfacing_frac < 1e-9, "内腔是遮挡，不是背面");
        assert!(!g.has_direction(), "封闭内腔不该给出单一建议方向");
    }

    /// 盒的**顶面**在斜向方向下也可达（斜向精加工那道就是靠这个覆盖陡面）。
    #[test]
    fn an_oblique_direction_reaches_the_top() {
        let d = boxed();
        let up = DVec3::new(0.0, 0.0, 1.0);
        let samples = vec![(DVec3::new(0.0, 0.0, 10.0), up, 1.0)];
        let oblique = DVec3::new(0.5, 0.0, 0.8);
        let rep = analyze(&d, &samples, &[oblique], &ReachConfig::default());
        assert_eq!(rep.reachable, 1, "40° 斜向应能加工顶面");
        // 太斜（法向与刀轴 > 45°）就该判背面
        let flat = DVec3::new(1.0, 0.0, 0.1);
        let rep = analyze(&d, &samples, &[flat], &ReachConfig::default());
        assert_eq!(rep.backfacing, 1, "接近水平的刀轴够不到顶面");
    }

    /// 竖直壁（法向 +x）在 `+z` 下是背面；给一个 `+x` 方向就可达。
    #[test]
    fn a_vertical_wall_needs_a_matching_side_direction() {
        let d = boxed();
        let n = DVec3::new(1.0, 0.0, 0.0);
        let p = DVec3::new(10.0, 0.0, 5.0);
        let up = DVec3::new(0.0, 0.0, 1.0);
        let r1 = analyze(&d, &[(p, n, 4.0)], &[up], &ReachConfig::default());
        assert_eq!(r1.backfacing, 1);
        assert!((r1.backfacing_area_mm2 - 4.0).abs() < 1e-9);
        let r2 = analyze(&d, &[(p, n, 4.0)], &[up, n], &ReachConfig::default());
        assert_eq!(r2.reachable, 1, "+x 方向应能加工竖直壁");
    }

    /// 贪心覆盖：4 个朝向各异的面，就该选出 4 个方向并达到 100%。
    ///
    /// 这是"按几何自己编排工序"的核心：不是预先写死 ±X/±Y，而是让覆盖面积说话。
    #[test]
    fn greedy_covers_all_four_side_faces() {
        let d = boxed();
        let mut samples = Vec::new();
        for (p, n) in [
            (DVec3::new(10.0, 0.0, 5.0), DVec3::new(1.0, 0.0, 0.0)),
            (DVec3::new(-10.0, 0.0, 5.0), DVec3::new(-1.0, 0.0, 0.0)),
            (DVec3::new(0.0, 10.0, 5.0), DVec3::new(0.0, 1.0, 0.0)),
            (DVec3::new(0.0, -10.0, 5.0), DVec3::new(0.0, -1.0, 0.0)),
        ] {
            samples.push((p, n, 2.0));
        }
        let cands = [
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(-1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(0.0, -1.0, 0.0),
            DVec3::new(0.0, 0.0, 1.0), // 这个方向一个面都盖不到，不该被选
        ];
        let vis = visibility(&d, &samples, &cands, &ReachConfig::default());
        assert!((vis.coverage(vis.all_mask()) - 1.0).abs() < 1e-9);
        let chosen = vis.greedy(0, 8, 0.0);
        assert_eq!(chosen.len(), 4, "四个面各要一道：{chosen:?}");
        assert!(!chosen.contains(&4), "盖不到任何面的方向不该被选进来");
        let m = vis.mask_of(&chosen);
        assert!((vis.coverage(m) - 1.0).abs() < 1e-9);
        // 已经全覆盖时，再贪心不该加任何方向
        assert!(vis.greedy(m, 8, 0.0).is_empty());
    }

    /// 连通块要**分得开**：两片互不相邻的够不到的面不能并成一片。
    ///
    /// 用盒**底面**（法向 −z），只给 `+z` 方向 → 两片都是"背面"。
    #[test]
    fn distant_blocked_patches_are_separate_groups() {
        let d = boxed();
        let down = DVec3::new(0.0, 0.0, -1.0);
        let up = DVec3::new(0.0, 0.0, 1.0);
        let mut samples = Vec::new();
        for (cx, cy) in [(-7.0, -7.0), (7.0, 7.0)] {
            for di in 0..3 {
                for dj in 0..3 {
                    samples.push((DVec3::new(cx + di as f64, cy + dj as f64, 0.0), down, 1.0));
                }
            }
        }
        let rep = analyze(&d, &samples, &[up], &ReachConfig::default());
        assert_eq!(rep.backfacing, 18);
        assert_eq!(rep.groups.len(), 2, "两片应各自成组：{}", rep.groups.len());
        for g in &rep.groups {
            assert_eq!(g.points, 9);
            assert!((g.backfacing_frac - 1.0).abs() < 1e-9, "全是背面");
            assert!(g.has_direction(), "同一朝向的两片应给得出建议方向");
            assert!(
                (g.a_deg() - 180.0).abs() < 1e-6,
                "朝下 → A180，实际 {}",
                g.a_deg()
            );
        }
    }
}
