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
    /// 锥度球头（模具/石墨/首饰精加工常用）：小球头 + 长锥颈，
    /// 细长但不失刚性。见 [`tapered_ball_end_mill`]。
    TaperedBall,
}

/// **刀库**：刀号 → 刀具定义。
///
/// 一份程序里可以有 `T1`…`T2`——粗加工用平底刀、精加工换球头刀是标准流程，
/// 而 `run` 需要"第 n 块用哪把刀"的答案。查表而不是让调用方传回调：
/// 回调会让"仿真用的刀"和"报告里写的刀"又变成两份数据。
#[derive(Clone, Debug, Default)]
pub struct ToolTable {
    entries: Vec<ToolDef>,
}

impl ToolTable {
    /// 单刀库（老接口 `run(program, tool, cfg)` 走这条）。
    pub fn single(tool: ToolDef) -> Self {
        Self { entries: vec![tool] }
    }

    pub fn new(entries: Vec<ToolDef>) -> Self {
        debug_assert!(!entries.is_empty(), "刀库不能为空——至少要有一把刀才能切");
        Self { entries }
    }

    /// 按刀号查。**查不到返回 `None`**，由调用方决定怎么处理
    /// （`sim` 会记一条报警并退回第一把，不静默用错刀）。
    pub fn by_number(&self, number: u16) -> Option<&ToolDef> {
        self.entries.iter().find(|t| t.number == number)
    }

    /// 第一把刀。刀库非空（构造时断言过）。
    pub fn first(&self) -> &ToolDef {
        self.entries.first().expect("刀库非空")
    }

    pub fn numbers(&self) -> impl Iterator<Item = u16> + '_ {
        self.entries.iter().map(|t| t.number)
    }

    /// 全部刀具（按刀库顺序）。`sim` 用它预生成每把刀的碰撞采样点。
    pub fn entries(&self) -> &[ToolDef] {
        &self.entries
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
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

/// 锥度球头刀的**锥面半角**（度）：由「刀尖半径 → 刃长处的半径」反解。
///
/// 供应商规格表给的是这个角度（例如 `R0.5*30.5*D6*75 / 单边斜角 4.69°`），
/// 但它实际是从**刀尖**量到刃顶的弦角，不是球头切点处的锥面角。两个值差
/// 0.1° 量级（球头那 0.5 mm 被算进锥长里了），见
/// `tapered_ball_sheet_angle_is_the_chord_angle` 测试。
pub fn taper_half_angle_deg(tip_radius: f64, top_radius: f64, flute_length: f64) -> f64 {
    ((top_radius - tip_radius) / flute_length.max(1e-12)).atan().to_degrees()
}

/// 生成**锥度球头刀**的母线。
///
/// ```text
///        ρ
///        │      ╱ 锥面（半角 α，斜率 tan α）
///        │    ╱
///        │  ╱ ← 切点 (r·cos α, r(1−sin α))：球面与锥面**相切**，不是折角
///        │╱
///     ╭──┴──╮  ← 球头 R = r，球心在 (0, r)
///     ╰──●──╯  ← 刀尖 (0, 0)
/// ```
///
/// 三个参数来自规格表：`tip_radius` = "半径 R"、`half_angle_deg` = "单边斜角 a"、
/// `flute_length` = "刃长 D"。
///
/// **切点必须算出来，不能取 (r, r)**：直接取球心高度会让球面与锥面在 ρ=r 处
/// 形成一个 α 大小的折角（4.69° 的刀在刃口附近是一圈肉眼可见的棱），
/// 而且真实刀具是磨出来的相切过渡。判据见
/// `tapered_ball_cone_is_tangent_to_the_ball`。
///
/// 母线 z 只到 `flute_length`；刀杆/刀柄由调用方另给（`shank`）。
pub fn tapered_ball_end_mill(
    tip_radius: f64,
    half_angle_deg: f64,
    flute_length: f64,
    segments: usize,
) -> Profile {
    let r = tip_radius;
    let a = half_angle_deg.to_radians();
    let (sa, ca, ta) = (a.sin(), a.cos(), a.tan());

    // 球面弧从刀尖 (0,0) 走到切点，β 是"从刀尖算起的仰角"：
    //   P(β) = (r·sin β, r(1−cos β))，切点在 β = 90° − α
    let beta_max = core::f64::consts::FRAC_PI_2 - a;
    let n = segments.max(4);
    let mut pts = Vec::with_capacity(n + 3);
    for i in 0..=n {
        let b = beta_max * (i as f64 / n as f64);
        pts.push(DVec2::new(r * b.sin(), r * (1.0 - b.cos())));
    }

    // 锥面：从切点按斜率 tan α 升到刃长。**起点就是切点**（上面最后一点），
    // 再推一次会在母线上留一个零长段。
    let z_t = r * (1.0 - sa);
    let r_top = r * ca + (flute_length - z_t).max(0.0) * ta;
    if flute_length > z_t {
        pts.push(DVec2::new(r_top, flute_length));
    }
    pts.push(DVec2::new(0.0, flute_length));
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

    /// 规格表：`R0.5*30.5*D6*75 单边斜角 4.69°`。四组测试全用这套数。
    const TB_R: f64 = 0.5;
    const TB_ANG: f64 = 4.69;
    const TB_LEN: f64 = 30.5;

    /// 锥面把球面接上去的切点（解析）：球心 (0,r)，锥面法向 (cos α, −sin α)。
    fn tangency(r: f64, half_angle_deg: f64) -> DVec2 {
        let a = half_angle_deg.to_radians();
        DVec2::new(r * a.cos(), r * (1.0 - a.sin()))
    }

    #[test]
    fn tapered_ball_starts_at_tip_and_ends_on_axis() {
        let p = tapered_ball_end_mill(TB_R, TB_ANG, TB_LEN, 16);
        assert_eq!(p.points()[0], DVec2::new(0.0, 0.0), "母线必须从刀尖开始");
        assert_eq!(p.points().last().unwrap().x, 0.0, "母线顶端必须回到轴上");
        assert!((p.height() - TB_LEN).abs() < 1e-12, "高度应等于刃长，实际 {}", p.height());
        // 切点必须出现在折线上：后面的相切测试靠它
        let t = tangency(TB_R, TB_ANG);
        assert!(
            p.points().iter().any(|q| (*q - t).length() < 1e-9),
            "母线上缺切点 {t:?}"
        );
    }

    /// 刃口是**球**，不是锥尖。锥尖会把 R0.5 的头部整段削掉，
    /// 而在刀尖附近 0.5 mm 恰好是精加工全部精度所在。
    #[test]
    fn tapered_ball_tip_is_a_sphere_not_a_cone() {
        let p = tapered_ball_end_mill(TB_R, TB_ANG, TB_LEN, 32);
        // (0.25, 0)：到球心 (0,0.5) 的距离 − R = 0.0590，**在球外**
        let d = sd_profile(DVec3::new(0.25, 0.0, 0.0), &p);
        let want = (0.25f64 * 0.25 + 0.5 * 0.5).sqrt() - TB_R;
        assert!((d - want).abs() < 2e-3, "球外一点应给 {want:.4}，实际 {d:.4}");
        // 纯锥面在这里会给 ≈0.249（差 200 倍），所以这个断言能分辨"球"和"锥"
        assert!(d < 0.1, "刃口若是锥尖，这里会是 0.249 而不是 {d:.4}");

        // (0.3, 0.2)：到球心 0.424 < R → **在球内**（注意 sd_profile 吃的是
        // 刀坐标：ρ = hypot(x,y)，所以 ρ=0.3 要写成 x=0.3, y=0）
        let d = sd_profile(DVec3::new(0.3, 0.0, 0.2), &p);
        let want = (0.09f64 + 0.09).sqrt() - TB_R;
        assert!(d < -0.05, "球内一点应给负距离（{want:.4} 附近），实际 {d:.4}");
        assert!((d - want).abs() < 2e-3, "实际 {d:.4}，应为 {want:.4}");
    }

    /// 球面与锥面**相切**：最后一小段球面弧的方向必须与锥面段的方向一致。
    ///
    /// 取 (r, r) 当切点（而不是 `r(1−sin α)`）会留下 α 大小的折角，
    /// 几何上是个棱，仿真里是一道 0.5 mm 宽的假过切。
    #[test]
    fn tapered_ball_cone_is_tangent_to_the_ball() {
        let p = tapered_ball_end_mill(TB_R, TB_ANG, TB_LEN, 64);
        let pts = p.points();
        let t = tangency(TB_R, TB_ANG);
        let i = pts.iter().position(|q| (*q - t).length() < 1e-9).expect("应有切点");
        let ball_dir = pts[i] - pts[i - 1]; // 切点前一段：球面弧
        let cone_dir = pts[i + 1] - pts[i]; // 切点后一段：锥面
        let ang = (ball_dir.x * cone_dir.x + ball_dir.y * cone_dir.y)
            / (ball_dir.length() * cone_dir.length());
        let deg = ang.clamp(-1.0, 1.0).acos().to_degrees();
        // 64 段时弧的弦与切线的差约 0.67°；把切点取成 (r,r) 会得到 α+0.67 ≈ 5.4°
        assert!(deg < 1.5, "球面与锥面之间存在 {deg:.2}° 折角（应 < 1.5°，相切）");
    }

    /// 刃顶半径对得上规格表：R0.5 / 4.69° / 刃长 30.5 → Ø5.92（表里写 D6）。
    #[test]
    fn tapered_ball_top_radius_matches_the_sheet() {
        let p = tapered_ball_end_mill(TB_R, TB_ANG, TB_LEN, 32);
        let a = TB_ANG.to_radians();
        let want = TB_R * a.cos() + (TB_LEN - TB_R * (1.0 - a.sin())) * a.tan();
        assert!((p.max_radius() - want).abs() < 1e-12, "解析值 {want:.6}，实际 {}", p.max_radius());
        // 与"柄径 6"的差是表里的圆整：4.69° 是从刀尖量的弦角，不是切点处的锥面角。
        // 实测 Ø5.926，差 0.074（1.2%）——表头自己写着"数据均为手工测量"。
        assert!(
            (2.0 * p.max_radius() - 6.0).abs() < 0.08,
            "刃顶直径 {:.3} 与柄径 6 差 {}",
            2.0 * p.max_radius(),
            (2.0 * p.max_radius() - 6.0).abs()
        );
    }

    /// 表里的"单边斜角"= 从**刀尖**拉到刃顶的弦角（不是锥面角）。
    /// 这一条把两个口径的差钉死：0.08° 以内，所以按 4.69° 建模是忠实的。
    #[test]
    fn sheet_angle_is_the_chord_angle_from_the_tip() {
        let chord = taper_half_angle_deg(TB_R, 3.0, TB_LEN);
        assert!((chord - TB_ANG).abs() < 0.02, "弦角 {chord:.3}° 应≈表里的 4.69°");
        let p = tapered_ball_end_mill(TB_R, TB_ANG, TB_LEN, 32);
        let p2 = tapered_ball_end_mill(TB_R, chord, TB_LEN, 32);
        assert!(
            (p.max_radius() - p2.max_radius()).abs() < 0.05,
            "两种口径的刃顶半径应差不到 0.05 mm"
        );
    }

    #[test]
    fn tool_table_looks_up_by_number_not_by_index() {
        let a = ToolDef {
            number: 7,
            kind: CutterKind::Flat,
            diameter: 6.0,
            corner_radius: 0.0,
            flute_length: 20.0,
            shank: flat_end_mill(6.0, 60.0),
            length_offset: 60.0,
            cutter: flat_end_mill(6.0, 20.0),
        };
        let b = ToolDef { number: 2, ..a.clone() };
        let t = ToolTable::new(vec![a, b]);
        assert_eq!(t.by_number(2).map(|x| x.number), Some(2), "刀号不是下标");
        assert_eq!(t.by_number(7).map(|x| x.number), Some(7));
        assert!(t.by_number(3).is_none(), "不存在的刀号必须返回 None，不能就近取");
        assert_eq!(t.first().number, 7);
        assert_eq!(t.len(), 2);
    }

    #[test]
    fn sd_ball_tip_distance() {
        let prof = ball_end_mill(6.0, 20.0, 32);
        // 球头刀尖正下方 1mm，距离应约等于 1（球心在 (0,3)）
        let d = sd_profile(DVec3::new(0.0, 0.0, -1.0), &prof);
        assert!((d - 1.0).abs() < 0.01, "球头刀尖下方距离 {d} 应≈1");
    }
}
