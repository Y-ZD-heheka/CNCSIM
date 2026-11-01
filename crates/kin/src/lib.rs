//! # cncsim-kin
//!
//! 机床运动学：拓扑、正解、逆解、RTCP、奇异点、轴速校核。
//!
//! 这是**全项目风险最高的一块**——摆长错 0.1 mm，所有仿真结果整体偏移，
//! 而画面看起来完全正常。所以它的测试必须最严（见 `tests/`）。
//!
//! ## 拓扑是数据，不是代码
//!
//! 不为每种构型写一份硬编码公式。用一条**关节链**描述：
//!
//! ```text
//! chain: [C(rot Z), A(rot X), X(lin), Y(lin), Z(lin)]
//! ```
//!
//! ## 一个容易搞反的约定（踩过坑，见 docs/机床建模-AC双转台.md §12.7）
//!
//! `chain` 的**物理装配顺序**是「工件 → 刀具」，但矩阵乘积的方向**相反**——
//! 因为旋转轴的 `sign = −1`（工件转了 +A，要表达"工具在工件坐标系里"必须用逆变换）。
//!
//! 结果是：`forward()` 返回的 `tip` 就是链乘积的**平移部分**，
//! **不是** `T⁻¹·0 = −Rᵀt`。写错会在 A=90° 时把 Y 的符号搞反。

use cncsim_math::{DMat3, DMat4, DVec3};
use cncsim_motion::Pose;

/// 平移/旋转所绕的轴。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub fn unit(self) -> DVec3 {
        match self {
            Axis::X => DVec3::X,
            Axis::Y => DVec3::Y,
            Axis::Z => DVec3::Z,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Linear(Axis),
    Rotary(Axis),
}

/// 一个关节。
#[derive(Clone, Debug)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    /// 相对父关节的固定偏置（装配尺寸）
    pub offset: DVec3,
    /// 旋转关节的回转中心（在 offset 之后、旋转之前施加）
    pub pivot: DVec3,
    /// 方向符号。工件端旋转轴取 −1，机床端直线轴取 +1。
    pub sign: f64,
    /// (min, max)，直线轴单位 mm，旋转轴单位**度**
    pub limit: (f64, f64),
}

impl Joint {
    pub fn linear(name: &str, axis: Axis, limit: (f64, f64)) -> Self {
        Self {
            name: name.into(),
            kind: JointKind::Linear(axis),
            offset: DVec3::ZERO,
            pivot: DVec3::ZERO,
            sign: 1.0,
            limit,
        }
    }

    pub fn rotary(name: &str, axis: Axis, limit: (f64, f64)) -> Self {
        Self {
            name: name.into(),
            kind: JointKind::Rotary(axis),
            offset: DVec3::ZERO,
            pivot: DVec3::ZERO,
            sign: 1.0,
            limit,
        }
    }

    pub fn with_offset(mut self, o: DVec3) -> Self {
        self.offset = o;
        self
    }

    pub fn with_sign(mut self, s: f64) -> Self {
        self.sign = s;
        self
    }

    pub fn is_rotary(&self) -> bool {
        matches!(self.kind, JointKind::Rotary(_))
    }
}

/// 机床拓扑：一条从工件到刀具的关节链。
#[derive(Clone, Debug)]
pub struct MachineTopology {
    pub name: String,
    pub chain: Vec<Joint>,
}

impl MachineTopology {
    /// AC 双转台摇篮，L = 180 mm。与 `assets/machines/ac_trunnion.machine.ron` 一致。
    ///
    /// 链顺序 **C → A → X → Y → Z**：摇篮式的旋转轴在**工件端**。
    pub fn ac_trunnion() -> Self {
        const L: f64 = 180.0;
        Self {
            name: "AC 双转台摇篮（L=180）".into(),
            chain: vec![
                Joint::rotary("C", Axis::Z, (-99999.0, 99999.0)).with_sign(-1.0),
                Joint::rotary("A", Axis::X, (-110.0, 110.0))
                    .with_offset(DVec3::new(0.0, 0.0, -L))
                    .with_sign(-1.0),
                Joint::linear("X", Axis::X, (-400.0, 400.0)),
                Joint::linear("Y", Axis::Y, (-300.0, 300.0)),
                Joint::linear("Z", Axis::Z, (-120.0, 520.0)),
            ],
        }
    }

    /// 关节数量
    pub fn joint_count(&self) -> usize {
        self.chain.len()
    }

    /// 按轴名查下标。**调用方不该记链顺序**——摇篮式是 `[C,A,X,Y,Z]`，
    /// 摆头式是 `[X,Y,Z,C,A]`，记错一个就是全盘错位。
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.chain.iter().position(|j| j.name == name)
    }

    /// 按轴名构造关节值向量。
    ///
    /// ```ignore
    /// // 别这样（顺序是链顺序，不是 XYZAC）：
    /// forward(&m, &[0.0, 0.0, -50.0, 90.0, 0.0]);
    /// // 这样：
    /// forward(&m, &m.q(&[("X", 0.0), ("Y", 0.0), ("Z", -50.0), ("A", 90.0), ("C", 0.0)]));
    /// ```
    pub fn q(&self, pairs: &[(&str, f64)]) -> Vec<f64> {
        let mut q = vec![0.0; self.chain.len()];
        for (name, v) in pairs {
            match self.index_of(name) {
                Some(i) => q[i] = *v,
                None => panic!("机床没有名为 {name:?} 的轴"),
            }
        }
        q
    }

    /// 旋转轴的下标（按链顺序）
    pub fn rotary_indices(&self) -> Vec<usize> {
        self.chain
            .iter()
            .enumerate()
            .filter(|(_, j)| j.is_rotary())
            .map(|(i, _)| i)
            .collect()
    }

    /// 直线轴的下标（按链顺序）
    pub fn linear_indices(&self) -> Vec<usize> {
        self.chain
            .iter()
            .enumerate()
            .filter(|(_, j)| !j.is_rotary())
            .map(|(i, _)| i)
            .collect()
    }

    /// 哪些关节超出了行程/转角限位（返回下标）。
    ///
    /// **`inverse` / `rtcp_inverse` 不检查限位**——它们只回答"运动学上能不能解出来"。
    /// 超程是**诊断**而不是无解：调用方要能区分这两种情况。
    /// 见 [`KinFinding::AxisLimit`]。
    pub fn violated_joints(&self, q: &[f64]) -> Vec<usize> {
        let mut out = Vec::new();
        for (i, (j, &v)) in self.chain.iter().zip(q).enumerate() {
            if v < j.limit.0 || v > j.limit.1 {
                out.push(i);
            }
        }
        out
    }

    /// 全部关节是否都在限位内
    pub fn within_limits(&self, q: &[f64]) -> bool {
        self.violated_joints(q).is_empty()
    }
}

/// 链乘积（内部用，正解与包络计算共享）。
///
/// ```text
/// t = I
/// for joint in chain:
///     t = t · Translate(offset)
///     Linear(axis) => t = t · Translate(axis · sign · q)
///     Rotary(axis) => t = t · Translate(pivot) · Rotate(axis, sign·q) · Translate(−pivot)
/// ```
pub fn chain_transform(topo: &MachineTopology, q: &[f64]) -> DMat4 {
    assert_eq!(
        q.len(),
        topo.chain.len(),
        "关节值数量 {} 与链长 {} 不一致",
        q.len(),
        topo.chain.len()
    );
    let mut t = DMat4::IDENTITY;
    for (joint, &val) in topo.chain.iter().zip(q) {
        t = t * DMat4::from_translation(joint.offset);
        match joint.kind {
            JointKind::Linear(axis) => {
                t = t * DMat4::from_translation(axis.unit() * (joint.sign * val));
            }
            JointKind::Rotary(axis) => {
                t = t * DMat4::from_translation(joint.pivot);
                t = t * DMat4::from_axis_angle(axis.unit(), (joint.sign * val).to_radians());
                t = t * DMat4::from_translation(-joint.pivot);
            }
        }
    }
    t
}

/// 正解：关节值 → 刀尖位姿（工件坐标系）。
///
/// `q` 的顺序与 `chain` 一致；直线轴单位 mm，旋转轴单位**度**。
///
/// **`tip` 取链乘积的平移部分**，不是 `T⁻¹·0 = −Rᵀt`——
/// 因为旋转轴的 `sign = −1`，链乘积实际是「刀具 → 工件」的变换。
/// 见模块文档与 docs/机床建模-AC双转台.md §12.7。
pub fn forward(topo: &MachineTopology, q: &[f64]) -> Pose {
    let t = chain_transform(topo, q);
    Pose {
        tip: t.translation(),
        axis: t.rotation().mul_vec3(DVec3::Z),
    }
}

/// 运动学诊断。**各层自带 finding 类型，由 `sim` 聚合成 `Alarm`**——
/// 不在底层造共享的 `Diagnostic` 枚举（见 docs/架构设计.md §2.3）。
#[derive(Clone, Debug, PartialEq)]
pub enum KinFinding {
    /// **近奇异**：两个旋转轴对刀轴的影响趋于共线，C 轴需求转速急剧上升。
    ///
    /// `kappa` 是 ∂(刀轴)/∂(旋转轴) 的条件数，奇异时 → ∞。
    /// 摇篮式在 A=0 处 κ=∞（C 转多少刀轴都不变）。
    NearSingular { kappa: f64, a_deg: f64 },

    /// 轴超出行程/转角限位。**这不是"无解"**——见 [`MachineTopology::violated_joints`]。
    AxisLimit {
        joint: usize,
        value: f64,
        limit: (f64, f64),
    },
}

impl KinFinding {
    /// 稳定编号，供 `sim` 转成 `Alarm` 时用。测试断言编号，不断言消息。
    pub fn id(&self) -> &'static str {
        match self {
            KinFinding::NearSingular { .. } => "KIN-001",
            KinFinding::AxisLimit { .. } => "KIN-002",
        }
    }
}

/// 旋转轴的条件数 `κ = σ_max/σ_min`，衡量离奇异点有多近。
///
/// 对摇篮式，`κ = 1/|sin A|`——A=0 时无穷大。返回 `None` 表示不是「2 旋转」构型。
///
/// 用**数值雅可比**而不是解析式：这样换机床构型不用改代码。
pub fn rotary_condition(topo: &MachineTopology, q: &[f64]) -> Option<f64> {
    let rot = topo.rotary_indices();
    if rot.len() != 2 {
        return None;
    }
    const H: f64 = 1e-6;
    let n0 = forward(topo, q).axis;
    let mut col = [DVec3::ZERO; 2];
    for j in 0..2 {
        let mut q2 = q.to_vec();
        q2[rot[j]] += H;
        col[j] = (forward(topo, &q2).axis - n0) / H;
    }
    // JᵀJ 是 2×2：[[a,b],[b,c]]
    let a = col[0].dot(col[0]);
    let b = col[0].dot(col[1]);
    let c = col[1].dot(col[1]);
    let trace = a + c;
    let disc = ((a - c) * (a - c) + 4.0 * b * b).sqrt();
    let lam_max = (trace + disc) * 0.5;
    let lam_min = (trace - disc) * 0.5;
    if lam_min <= 1e-300 {
        return Some(f64::INFINITY);
    }
    Some((lam_max / lam_min).sqrt())
}

/// 检查一个位姿是否有运动学问题。
///
/// `kappa_limit` 是近奇异的判定阈值：`κ > kappa_limit` 即报警。
/// 取 100 意味着 `|sin A| < 0.01`（摇篮式 A < 0.57°）才报——更小的值会让
/// C 轴需求转速爆炸（主方案 §4.5）。
pub fn check(topo: &MachineTopology, q: &[f64], kappa_limit: f64) -> Vec<KinFinding> {
    let mut out = Vec::new();

    if let Some(kappa) = rotary_condition(topo, q)
        && kappa > kappa_limit
    {
        let rot = topo.rotary_indices();
        out.push(KinFinding::NearSingular {
            kappa,
            a_deg: q[rot[1]],
        });
    }

    for i in topo.violated_joints(q) {
        let j = &topo.chain[i];
        out.push(KinFinding::AxisLimit {
            joint: i,
            value: q[i],
            limit: j.limit,
        });
    }

    out
}

/// 只解三个直线轴：旋转轴的值由 `q` 给定，求使刀尖落在 `target_tip` 的直线轴值。
///
/// 旋转轴定下来后，刀尖对直线轴是**仿射**的（直线轴都是平移），所以 3×3 一次解出。
/// `q` 里的直线轴分量会被忽略并覆写。
fn solve_linear(topo: &MachineTopology, q: &[f64], target_tip: DVec3) -> Option<Vec<f64>> {
    let lin = topo.linear_indices();
    if lin.len() != 3 {
        return None;
    }
    let mut q = q.to_vec();
    for &i in &lin {
        q[i] = 0.0;
    }

    let tip0 = forward(topo, &q).tip;
    let mut basis = [DVec3::ZERO; 3];
    for (k, &i) in lin.iter().enumerate() {
        let mut q2 = q.clone();
        q2[i] = 1.0;
        basis[k] = forward(topo, &q2).tip - tip0;
    }
    let m = DMat3::from_cols(basis[0], basis[1], basis[2]);
    let sol = m.inverse()?.mul_vec3(target_tip - tip0);
    for (k, &i) in lin.iter().enumerate() {
        q[i] = sol.component(k);
    }
    Some(q)
}

/// RTCP（刀尖点跟随）逆解：**旋转角由程序给定，只反算直线轴**。
///
/// 程序里的 `X/Y/Z` 是**工件坐标系里的刀尖位置**，`A/C` 是角度——控制器要算的是
/// 机床的三个直线轴该到哪。这与 [`inverse`] 的区别是：**不求解旋转轴**。
///
/// `q` 只需填好旋转轴的分量（用 [`MachineTopology::q`] 按名构造），直线轴会被覆写。
///
/// ## 不变量（见 `tests/rtcp.rs`）
///
/// 固定刀尖位置、让 A 从 −60° 摆到 +60°，每一组解算出的机床轴都必须让刀尖
/// 回到同一个点。这是 RTCP 的定义，也是"摆长基准错"的唯一可靠检出手段。
pub fn rtcp_inverse(
    topo: &MachineTopology,
    target_tip: DVec3,
    q_rotaries: &[f64],
) -> Option<Vec<f64>> {
    solve_linear(topo, q_rotaries, target_tip)
}

/// 逆解：位姿 → 关节值。
///
/// **解耦两步**（见主方案 §4.3）：
/// 1. 由刀轴方向用 Newton（阻尼最小二乘）解**两个旋转轴**——只有 2 个未知数、2 个自由度约束。
/// 2. 旋转轴定下来后，直线轴到刀尖的映射是**仿射**的，3×3 一次解出。
///
/// `seed` 用于多解选择：Newton 从它出发，收敛到离上一周期最近的那个解。
/// 返回 `None` 表示不可达（轴奇异、超出结构、或迭代不收敛）。
pub fn inverse(topo: &MachineTopology, pose: &Pose, seed: &[f64]) -> Option<Vec<f64>> {
    let n = topo.chain.len();
    let rot = topo.rotary_indices();
    if rot.len() != 2 || topo.linear_indices().len() != 3 {
        return None;
    }

    let mut q = if seed.len() == n {
        seed.to_vec()
    } else {
        vec![0.0; n]
    };
    for i in topo.linear_indices() {
        q[i] = 0.0;
    }

    // ── 第一步：解两个旋转轴 ──
    //
    // 残差用**误差向量**（target − cur），不是 `cur × target`。
    //
    // 叉积在两种情况下都归零：平行（对）和**反平行**（错）；而且它指向"转轴"
    // 方向，与雅可比的列（轴的切向）**正交**——在 A=0 的奇异点上，切向在 y-z 平面、
    // 叉积沿 x，两者点积恒为 0，牛顿一步都迈不出去，48 次迭代后直接报"不可达"。
    // 实测：A=0 起步解 A=20 的刀轴，永远失败。
    //
    // `target − cur` 的方向是"轴往哪边偏"，与雅可比列同处切平面，点积不为零。
    // 靠近收敛时两者一阶等价，所以已有的 500 例随机往返测试不受影响。
    let target = pose.axis;
    let mut converged = false;
    for _ in 0..48 {
        let cur = forward(topo, &q).axis;
        let err = target - cur;
        if err.length() < 1e-13 {
            converged = true;
            break;
        }

        // 数值雅可比：3×2（3 个残差分量、2 个未知数）
        const H: f64 = 1e-6;
        let mut col = [DVec3::ZERO; 2];
        for j in 0..2 {
            let mut q2 = q.clone();
            q2[rot[j]] += H;
            col[j] = (forward(topo, &q2).axis - cur) / H;
        }

        // 阻尼最小二乘：(JᵀJ + λ²I) Δq = Jᵀ·err
        let a = col[0].dot(col[0]);
        let b = col[0].dot(col[1]);
        let c = col[1].dot(col[1]);
        let r0 = col[0].dot(err);
        let r1 = col[1].dot(err);
        let lambda = 1e-10 * (a + c).max(1e-12);
        let (a2, c2) = (a + lambda, c + lambda);
        let det = a2 * c2 - b * b;
        if det.abs() < 1e-30 {
            return None; // 雅可比退化：接近奇异点
        }
        let mut dq0 = (c2 * r0 - b * r1) / det;
        let mut dq1 = (a2 * r1 - b * r0) / det;

        // 步长限幅：防止在大残差时冲出可行域
        let step = dq0.abs().max(dq1.abs());
        if step > 15.0 {
            let s = 15.0 / step;
            dq0 *= s;
            dq1 *= s;
        }
        q[rot[0]] += dq0;
        q[rot[1]] += dq1;
    }

    if !converged {
        let cur = forward(topo, &q).axis;
        if cur.cross(target).length() > 1e-9 || cur.dot(target) <= 0.0 {
            return None;
        }
    }

    // ── 第二步：解三个直线轴 ──
    solve_linear(topo, &q, pose.tip)
}

/// 一段的非线性误差（`docs/技术方案` §4.6）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NonlinearError {
    /// 刀尖轨迹到弦（起讫点连线）的最大距离，mm
    pub max_deviation: f64,
    /// 最大偏差处的段内参数（0..1）
    pub at_param: f64,
}

/// 一段程序在**关节空间线性插补**下，刀尖轨迹偏离直线的最大量。
///
/// 五轴联动时，控制器若只做"每个插补周期把 5 个轴各走一小步"，刀尖走的是
/// 空间曲线——绕回转中心的圆弧与弦之间的拱高，闭式 ≈ `R·(1−cos(Δθ/2))`。
/// 这直接给出后处理的点位密度要求（Δθ_max = 2·acos(1−ε/R)）。
///
/// **口径**：默认模拟"无补偿裸插补"（最坏情况）。RTCP 开启时控制器在位姿空间
/// 插补、刀尖走直线，这一段误差为零——本函数算的正是"如果控制器不补偿会偏多少"，
/// 用来判断后处理给的点位密度够不够。
///
/// 只用正解（`forward`），不求解——关节值由调用方（装配层）从 `inverse` 拿到。
pub fn nonlinear_error(
    topo: &MachineTopology,
    q_start: &[f64],
    q_end: &[f64],
    tip_start: DVec3,
    tip_end: DVec3,
    samples: usize,
) -> NonlinearError {
    assert_eq!(q_start.len(), q_end.len(), "两端关节值数量必须一致");

    // 旋转轴全程不动 ⇒ 链变换 = R(常量)·(直线轴的仿射)，刀尖恰好落在弦上，
    // 误差恒为 0——不用采样。三轴程序全部走这条早退。
    let rotary = topo.rotary_indices();
    if rotary.iter().all(|&i| (q_end[i] - q_start[i]).abs() < 1e-9) {
        return NonlinearError { max_deviation: 0.0, at_param: 0.0 };
    }

    let dir = tip_end - tip_start;
    let len = dir.length();
    let n = samples.max(2);

    let mut max_dev = 0.0f64;
    let mut at = 0.0f64;
    for k in 0..=n {
        let t = k as f64 / n as f64;
        let q: Vec<f64> = q_start.iter().zip(q_end).map(|(a, b)| a + (b - a) * t).collect();
        let tip = forward(topo, &q).tip;

        // 到"直线"的距离。起讫点重合（纯回转、刀尖定点）时弦退化为一点，
        // 改量到点的距离——这正是"刀尖该不动却动了多少"。
        let dev = if len < 1e-9 {
            (tip - tip_start).length()
        } else {
            ((tip - tip_start).cross(dir)).length() / len
        };
        if dev > max_dev {
            max_dev = dev;
            at = t;
        }
    }
    NonlinearError { max_deviation: max_dev, at_param: at }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trunnion_topology_shape() {
        let m = MachineTopology::ac_trunnion();
        assert_eq!(m.joint_count(), 5);
        assert_eq!(m.rotary_indices(), vec![0, 1], "C、A 在链首（工件端）");
        assert_eq!(m.linear_indices(), vec![2, 3, 4], "XYZ 在链尾（机床端）");
    }

    #[test]
    fn trunnion_rotary_signs_are_negative() {
        // 工件端旋转轴必须取 −1（表达"工具在工件系"用逆变换）
        let m = MachineTopology::ac_trunnion();
        for i in m.rotary_indices() {
            assert_eq!(m.chain[i].sign, -1.0, "关节 {} 的 sign 应为 −1", m.chain[i].name);
        }
        for i in m.linear_indices() {
            assert_eq!(m.chain[i].sign, 1.0, "关节 {} 的 sign 应为 +1", m.chain[i].name);
        }
    }

    /// 单旋转轴测试机床：A 绕 X（无偏置）+ XYZ 直线。
    fn one_rotary() -> MachineTopology {
        MachineTopology {
            name: "单旋转轴".into(),
            chain: vec![
                Joint::rotary("A", Axis::X, (-360.0, 360.0)),
                Joint::linear("X", Axis::X, (-400.0, 400.0)),
                Joint::linear("Y", Axis::Y, (-300.0, 300.0)),
                Joint::linear("Z", Axis::Z, (-120.0, 520.0)),
            ],
        }
    }

    #[test]
    fn nonlinear_error_matches_sagitta_closed_form() {
        // 刀尖在 (0, 0, R)：A 转 Δθ，刀尖画半径 R 的圆弧，弦是起讫点连线。
        // 弧与弦的最大偏差 = 拱高 R(1−cos(Δθ/2))，出现在 t = 0.5。
        let m = one_rotary();
        let r = 180.0;
        let d_theta = 30.0f64.to_radians();

        let qs = m.q(&[("A", 0.0), ("X", 0.0), ("Y", 0.0), ("Z", r)]);
        let qe = m.q(&[("A", 30.0), ("X", 0.0), ("Y", 0.0), ("Z", r)]);
        let p0 = forward(&m, &qs).tip;
        let p1 = forward(&m, &qe).tip;

        let ne = nonlinear_error(&m, &qs, &qe, p0, p1, 64);
        let expected = r * (1.0 - (d_theta / 2.0).cos());
        assert!(
            (ne.max_deviation - expected).abs() < 1e-9,
            "拱高应精确等于闭式：{} vs {expected}",
            ne.max_deviation
        );
        assert!((ne.at_param - 0.5).abs() < 1.0 / 32.0, "峰值应在段中，实际 t={}", ne.at_param);
    }

    #[test]
    fn nonlinear_error_is_zero_when_rotaries_stand_still() {
        // 三轴块：只有直线轴动。直线轴仿射 ⇒ 刀尖恰在弦上，误差必须为 0——
        // 连采样都不该做（这条也守住了"三轴程序不被误报"）。
        let m = MachineTopology::ac_trunnion();
        let qs = m.q(&[("C", 12.0), ("A", -30.0), ("X", 1.0), ("Y", 2.0), ("Z", 3.0)]);
        let qe = m.q(&[("C", 12.0), ("A", -30.0), ("X", 40.0), ("Y", -5.0), ("Z", 80.0)]);
        let ne = nonlinear_error(&m, &qs, &qe, DVec3::new(1.0, 2.0, 3.0), DVec3::new(40.0, -5.0, 80.0), 32);
        assert_eq!(ne.max_deviation, 0.0);
    }

    #[test]
    fn pure_rotary_with_fixed_tip_reports_zero() {
        // 起讫点重合（dir 退化）：直线轴全 0 时旋转轴转刀尖，刀尖在轴上不动。
        // 到"点"的距离必须为 0——叉积分支在这里除零/归零都不能出 NaN。
        let m = one_rotary();
        let qs = m.q(&[("A", 0.0), ("X", 0.0), ("Y", 0.0), ("Z", 0.0)]);
        let qe = m.q(&[("A", 90.0), ("X", 0.0), ("Y", 0.0), ("Z", 0.0)]);
        let ne = nonlinear_error(&m, &qs, &qe, DVec3::ZERO, DVec3::ZERO, 16);
        assert!(ne.max_deviation.abs() < 1e-12, "定点回转不该有偏差：{}", ne.max_deviation);
    }
}
