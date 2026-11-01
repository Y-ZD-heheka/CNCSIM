//! # cncsim-tool
//!
//! 刀具词汇：母线 profile、刀型、刀柄。
//!
//! ## 核心抽象：一切刀具都是旋转体
//!
//! 平底刀、球头刀、圆鼻刀，以及刀杆、刀柄、夹头——**都是绕刀轴的旋转体**。
//! 所以只要一条**母线折线** `(ρ, z)` 就能描述：
//!
//! ```text
//! 平底刀  φ6：[(0,0), (3,0), (3,20), (0,20)]
//! 球头刀  φ6：[(0,0), (0.6,0.06), ..., (3,3), (3,20), (0,20)]   ← 圆弧段采样
//! 圆鼻刀 R1：[(0,0), (2,0), (3,1), (3,20), (0,20)]             ← 圆角过渡
//! ```
//!
//! 这条母线同时服务三个消费者：
//! - `voxel` 扫掠 SDF（`sd_profile`）
//! - `collide` 的凸分解（按拐点切成圆台）
//! - `render` 的外形显示
//!
//! **三者必须用同一份 profile**，否则"仿真用的刀"和"碰撞用的刀"不是同一把。

use cncsim_math::{DVec2, DVec3, EPS};

/// 母线折线，点按 z 递增排列，`ρ ≥ 0`。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Profile {
    pts: Vec<DVec2>,
}

impl Profile {
    pub fn new(pts: Vec<DVec2>) -> Self {
        debug_assert!(pts.len() >= 2, "profile 至少两个点");
        debug_assert!(pts.iter().all(|p| p.x >= -EPS), "profile 的 ρ 必须非负");
        debug_assert!(
            pts.first().unwrap().x.abs() < EPS && pts.last().unwrap().x.abs() < EPS,
            "母线首尾必须落在轴上（ρ=0）——否则闭合边是真实表面，sd_profile 的距离会算错"
        );
        Self { pts }
    }

    pub fn points(&self) -> &[DVec2] {
        &self.pts
    }

    /// 从刀尖到刀顶的轴向长度
    pub fn height(&self) -> f64 {
        self.pts.last().map(|p| p.y).unwrap_or(0.0) - self.pts[0].y
    }

    /// 最大半径
    pub fn max_radius(&self) -> f64 {
        self.pts.iter().map(|p| p.x).fold(0.0, f64::max)
    }

    /// 若母线**恰好**是矩形 `(0,0)–(R,0)–(R,L)–(0,L)`，返回 `(R, L)`。
    ///
    /// 平底刀的纯平移扫掠体是一个**精确的笛卡尔积**（径向 2D 胶囊 × 轴向区间），
    /// 因此 `sd_sweep` 可以跳过黄金分割直接闭式求解——见
    /// `cncsim_voxel::sweep::sd_sweep` 的快路径。
    ///
    /// 判据故意严格（4 个点、逐点对齐）：带倒角或圆角的 profile 走黄金分割兜底，
    /// 宁慢勿错。`flat_end_mill` 构造的正是这个形状。
    pub fn as_flat_end_mill(&self) -> Option<(f64, f64)> {
        let p = &self.pts;
        if p.len() != 4 {
            return None;
        }
        // 比 Profile::new 的 EPS 更紧：这是"能不能用解析式"的判据，不是容差。
        const E: f64 = 1e-12;
        let (r, l) = (p[1].x, p[2].y);
        let ok = p[0].x.abs() < E
            && p[0].y.abs() < E
            && (p[1].x - r).abs() < E
            && p[1].y.abs() < E
            && (p[2].x - r).abs() < E
            && (p[2].y - l).abs() < E
            && p[3].x.abs() < E
            && (p[3].y - l).abs() < E
            && r > 0.0
            && l > 0.0;
        if ok { Some((r, l)) } else { None }
    }
}

/// 刀型。只影响 UI 显示与默认 profile 的生成，几何一律走 `Profile`。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutterKind {
    Flat,
    Ball,
    BullNose,
}

/// 完整刀具定义。**这一份同时给几何内核与碰撞用**。
#[derive(Clone, Debug)]
pub struct ToolDef {
    pub number: u16,
    pub kind: CutterKind,
    pub diameter: f64,
    /// 圆鼻刀的圆角半径
    pub corner_radius: f64,
    /// 刃长：此长度以下算"切削区"，与毛坯接触是合法的
    pub flute_length: f64,
    /// 刃长以上（刀杆 + 刀柄）参与碰撞检测
    pub shank: Profile,
    /// H 寄存器值（G43 用）
    pub length_offset: f64,
    /// 切削部分的母线
    pub cutter: Profile,
}

impl ToolDef {
    /// 切削区与刀杆的分界高度。碰撞检测只测这以上（ADR-004 §1.2）。
    pub fn cutting_top(&self) -> f64 {
        self.flute_length
    }

    /// 刀具全长（刀尖到刀柄顶）
    pub fn overall_length(&self) -> f64 {
        self.length_offset
    }
}

/// 生成平底刀的母线。
pub fn flat_end_mill(diameter: f64, length: f64) -> Profile {
    let r = diameter / 2.0;
    Profile::new(vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(r, 0.0),
        DVec2::new(r, length),
        DVec2::new(0.0, length),
    ])
}

/// 生成球头刀的母线（圆弧段用 `segments` 段折线逼近）。
pub fn ball_end_mill(diameter: f64, length: f64, segments: usize) -> Profile {
    let r = diameter / 2.0;
    let mut pts = Vec::with_capacity(segments + 3);
    // 球头：从刀尖 (0,0) 到 (r, r) 的四分之一圆弧
    for i in 0..=segments {
        let a = core::f64::consts::FRAC_PI_2 * (i as f64 / segments as f64);
        pts.push(DVec2::new(r * a.sin(), r * (1.0 - a.cos())));
    }
    pts.push(DVec2::new(r, length));
    pts.push(DVec2::new(0.0, length));
    Profile::new(pts)
}

/// 旋转体的解析有符号距离：`d(p) = 2D 距离(母线) 在子午面内`。
///
/// 这条性质让所有刀型共用一个求值函数——旋转不改变子午面内的距离。
pub fn sd_profile(p_local: DVec3, profile: &Profile) -> f64 {
    sd_profile_rz(p_local.x.hypot(p_local.y), p_local.z, profile)
}

/// 已经算好 `(r, z)` 的版本——**扫掠内层用它**。
///
/// 两个理由：① 内层每次求值都要 `(r,z)`，而 `r` 是上一段算出来的，不该再走一次
/// `hypot`（libm 调用，实测约 7 ns，占平移路径单次求值的四成）；
/// ② 避免构造 `DVec3`。旋转体的距离只依赖子午面坐标，`(r, z)` 就是全部输入。
#[inline]
pub fn sd_profile_rz(r: f64, z: f64, profile: &Profile) -> f64 {
    sd_polygon_2d(DVec2::new(r, z), profile.points())
}

/// 2D 子午面内的有符号距离。
///
/// **距离只算 n−1 条边，不含"闭合边"**——因为母线的首尾都在轴（ρ=0）上，
/// 那条闭合边是旋转轴本身，**不是真实表面**。
///
/// 踩过的坑：把闭合边算进去的话，轴线上的点会得到距离 0（"贴在表面"），
/// 而实际上它到侧面的距离是半径。平底刀 φ6 在轴线上应返回 −3，不是 −0。
fn sd_polygon_2d(p: DVec2, poly: &[DVec2]) -> f64 {
    let n = poly.len();

    // ── 距离：开折线（排除闭合边） ──
    let mut d2 = f64::INFINITY;
    for i in 0..n - 1 {
        d2 = d2.min(dist_to_segment_sq(p, poly[i], poly[i + 1]));
    }

    // ── 符号：闭合多边形的射线法 ──
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) {
            let t = (p.y - a.y) / (b.y - a.y);
            if p.x < a.x + t * (b.x - a.x) {
                inside = !inside;
            }
        }
        j = i;
    }

    let d = d2.sqrt();
    if inside { -d } else { d }
}

fn dist_to_segment_sq(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let e = b - a;
    let w = p - a;
    let len2 = e.x * e.x + e.y * e.y;
    let t = if len2 < EPS { 0.0 } else { ((w.x * e.x + w.y * e.y) / len2).clamp(0.0, 1.0) };
    let b = w - e * t;
    b.x * b.x + b.y * b.y
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_end_mill_profile_is_a_rectangle() {
        let p = flat_end_mill(6.0, 20.0);
        assert_eq!(p.max_radius(), 3.0);
        assert_eq!(p.height(), 20.0);
    }

    #[test]
    fn ball_end_mill_profile_starts_at_tip() {
        let p = ball_end_mill(6.0, 20.0, 16);
        assert_eq!(p.points()[0], DVec2::new(0.0, 0.0), "母线必须从刀尖开始");
        assert!((p.max_radius() - 3.0).abs() < 1e-12);
        assert!((p.height() - 20.0).abs() < 1e-12);
    }

    #[test]
    fn sd_flat_mill_inside_and_outside() {
        let prof = flat_end_mill(6.0, 20.0);
        // 刀轴上的点：内部，距离 = 到侧面 3mm
        let d = sd_profile(DVec3::new(0.0, 0.0, 10.0), &prof);
        assert!((d + 3.0).abs() < 1e-9, "轴线处应为 −3，实际 {d}");
        // 侧面外 2mm
        let d = sd_profile(DVec3::new(5.0, 0.0, 10.0), &prof);
        assert!((d - 2.0).abs() < 1e-9, "侧面外应为 +2，实际 {d}");
        // 刀尖下方 1mm
        let d = sd_profile(DVec3::new(0.0, 0.0, -1.0), &prof);
        assert!((d - 1.0).abs() < 1e-9, "刀尖下应为 +1，实际 {d}");
    }

    #[test]
    fn sd_ball_tip_distance() {
        let prof = ball_end_mill(6.0, 20.0, 32);
        // 球头刀尖正下方 1mm，距离应约等于 1（球心在 (0,3)）
        let d = sd_profile(DVec3::new(0.0, 0.0, -1.0), &prof);
        assert!((d - 1.0).abs() < 0.01, "球头刀尖下方距离 {d} 应≈1");
    }
}
