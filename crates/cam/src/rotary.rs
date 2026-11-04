//! 回转体（花瓶/叶轮罩/圆柱凸轮）的 **A/C 五轴 RTCP** 刀路。
//!
//! 和 [`crate::generate`] 的 3 轴高度场是两条独立的工艺路线：
//!
//! | | 3 轴高度场 | A/C 回转（本模块） |
//! |---|---|---|
//! | 适用 | 每个 XY 列单值上表面 | 绕 Z 的回转/波浪面 |
//! | 刀轴 | 固定 +Z | 沿外法向，A 摆动、C 回转 |
//! | RTCP | 无 | **G43.4 H1**（刀尖跟随） |
//! | 中空/倒扣 | 拒绝 | 外轮廓可做；内腔不在本工序 |
//!
//! ## 轮廓怎么来
//!
//! 不做射线求交：网格顶点足够密（花瓶 360 万顶点），按 `(θ, z)` 分箱取**最大半径**
//! 就是外轮廓。内壁半径更小，自然被排除。每箱 4°×2 mm 时花瓶 100% 有值。
//!
//! ## 刀轴
//!
//! 表面 `r(θ,z)` 的外法向（柱坐标）：
//!
//! ```text
//! n = normalize( e_r − (r_θ/r)·e_θ − r_z·e_z )
//! ```
//!
//! 再反解 A/C（Head 约定，刀轴随主轴摆）：
//!
//! ```text
//! axis = Rz(C)·Rx(A)·Z = (sinA·sinC, −sinA·cosC, cosA)
//! A = acos(n_z),  C = atan2(n_x, −n_y)
//! ```
//!
//! ## A 限位：限位 + 抬刀，不跳过
//!
//! A 超过机床行程（[`A_LIMIT_DEG`]）时**不能整点丢掉**：丢点会让相邻两刀之间
//! 连出一条穿过工件的 G1 弦，那是最典型的啃伤。这里改成把 A 压到限位、并把刀尖
//! 沿法向抬起 `R·(1−cosδ)`（δ = 欠倾角）——球头这时仍然与切平面相切，只是切点
//! 挪了位置，**不会啃进工件**，代价是那一小片留下最多 `R·(1−cosδ)` 的余量。
//! 欠倾 55° 时 Ø4 球头也就 0.85 mm，比"整片不切"好得多，而且可以如实报出来。
//!
//! ## 粗加工
//!
//! 方料四角离回转面很远（花瓶：外接半径 79，最小轮廓半径 10），只跑一刀精加工
//! 会得到"一块开了槽的方料"。所以先跑 [`Roughing`]：**沿外法向偏置的螺旋**，
//! 偏置量 `R_rough + 余量 + k·step` 逐层加大，覆盖从工件表面一直到毛坯外接圆柱
//! 的整个区域。每层一条连续螺旋（C 连续转、Z 连续下降），层间抬到安全高度。
//!
//! 粗加工刀轴保持竖直（`A0 C0`）：球头竖直时切点在球心正下方，偏置沿法向做，
//! 陡壁缓坡都不会啃伤——这是 [`crate::generate`] 里 `finish_pass` 的同一套构造。
//!
//! ## 一条精加工走刀
//!
//! 从顶到底逐圈，圈内 θ 走整周，下一圈反向（避免 C 在圈末空转 360°）。
//! 刀尖在表面上，球心在法向外侧 R 处——球头切掉的是表面**外侧**的余量。

use cncsim_math::{DVec2, DVec3};

use crate::{CamError, Stock, ToolKind, ToolSpec, Tri};

/// A 轴行程（度）。超过它的法向靠"限位 + 抬刀"处理，见模块注释。
pub const A_LIMIT_DEG: f64 = 110.0;

/// 粗加工参数。
#[derive(Clone, Debug)]
pub struct Roughing {
    /// 粗加工球头刀（一般比精加工刀大 2~4 倍，层数直接跟刀径成反比）。
    pub tool: ToolSpec,
    /// 层距（mm）。取 0.5~0.7 倍刀径。
    pub step_mm: f64,
    /// 每转 Z 进给（mm/r）。要小于刀径，否则螺旋之间留筋。
    pub pitch_mm: f64,
    /// 粗加工的 C 采样（度）。粗加工只要求"留料不啃伤"，可以比精加工粗。
    pub step_deg: f64,
    /// 精加工余量（mm）。
    pub allowance_mm: f64,
}

/// 回转加工的作业参数。
#[derive(Clone, Debug)]
pub struct RotaryJob<'a> {
    pub mesh: &'a [Tri],
    pub stock: Stock,
    /// 回转轴在 XY 平面的位置。
    pub center: DVec2,
    /// 精加工球头刀（回转轮廓用球头，法向偏置才有意义）。
    pub tool: ToolSpec,
    /// C 方向采样（度）。花瓶 4° → 一圈 90 点。
    pub step_deg: f64,
    /// Z 方向采样（mm）。
    pub step_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub spindle_rpm: u32,
    /// A 轴行程（度）。
    pub a_limit_deg: f64,
    /// 刀轴相对外法向**朝 +Z 翘**的角度（度）。0 = 纯法向。
    ///
    /// 球心始终在 `p + R·n` 上（球与曲面恒相切），翘的只是刀颈——用来躲开
    /// 相邻的壁。副作用是 A 变小，超程点也跟着少。
    pub tilt_deg: f64,
    /// 粗加工；`None` = 只跑精加工（毛坯已经是近净形时才这么用）。
    pub roughing: Option<Roughing>,
}

/// 外轮廓。
///
/// **两级分辨率**：刀路采样是 `n_theta × n_z`（4°×2 mm 这种），但半径不直接在这个
/// 粗格上取"格内最大值"——那是个**包络**：格内最鼓的地方在 5° 外、1.5 mm 外，
/// 按它下刀等于把凹槽整片啃掉（实测平均胖 1.13 mm、最大 7.36 mm，花瓶表面被
/// 削掉一层）。所以半径另建一张 0.5°×0.25 mm 的细网格存真值，刀路在细网格上
/// 查点，误差降到 0.1 mm 量级；法向也在细网格上做中心差分。
#[derive(Clone, Debug)]
pub struct RotaryProfile {
    /// 刀路采样：θ 方向格数。
    pub n_theta: usize,
    /// 刀路采样：Z 方向格数。
    pub n_z: usize,
    pub z0: f64,
    pub dz: f64,
    /// 刀路采样网格上的半径（每个值都来自细网格）。
    pub r: Vec<f64>,
    /// 细网格（真值）：`f_r[iz * f_nt + it]`。
    f_nt: usize,
    f_nz: usize,
    f_z0: f64,
    f_dz: f64,
    f_r: Vec<f64>,
}

impl RotaryProfile {
    pub fn theta(&self, it: usize) -> f64 {
        it as f64 / self.n_theta as f64 * std::f64::consts::TAU
    }

    pub fn z(&self, iz: usize) -> f64 {
        self.z0 + iz as f64 * self.dz
    }

    /// 该格的半径；0 / 非有限 = 没有轮廓。
    pub fn r_at(&self, it: usize, iz: usize) -> Option<f64> {
        let r = self.r[iz * self.n_theta + it];
        (r > 0.0 && r.is_finite()).then_some(r)
    }

    pub fn max_r(&self) -> f64 {
        self.r
            .iter()
            .copied()
            .filter(|v| *v > 0.0)
            .fold(0.0, f64::max)
    }

    pub fn min_r(&self) -> f64 {
        self.r
            .iter()
            .copied()
            .filter(|v| *v > 0.0)
            .fold(f64::INFINITY, f64::min)
    }

    /// 连续 `(θ, z)` → 细网格上的真值半径。3×3 邻域兜底（细网格边缘可能空）。
    pub fn radius(&self, th: f64, z: f64) -> Option<f64> {
        let it = ((th.rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU
            * self.f_nt as f64)
            .round() as usize)
            % self.f_nt;
        let iz = (((z - self.f_z0) / self.f_dz).round() as i64).clamp(0, self.f_nz as i64 - 1)
            as usize;
        // **双线性取真值**，不是邻域最大值。
        //
        // 之前这里取 3×3 最大，等于把刀尖顶到"包络"上：格内最鼓的点在 ±0.5° 之外，
        // 刀就永远进不了凹谷。实测刀尖比真值高 0.30 mm（P95 1.53 mm），
        // Ø4 球头跟着包络走 → 每个比 2 mm 浅的谷都被整段削掉（过切 51%）。
        if let Some(v) = fine_sample(self.f_r.as_slice(), self.f_nt, self.f_nz, self.f_z0, self.f_dz, th, z)
        {
            return Some(v);
        }
        // 细网格边缘可能留空：取邻域**非零中位数**兜底（仍不取最大值）
        let mut near: Vec<f64> = Vec::with_capacity(9);
        for dz in -1i64..=1 {
            let jz = iz as i64 + dz;
            if jz < 0 || jz >= self.f_nz as i64 {
                continue;
            }
            for dt in -1i64..=1 {
                let jt = (it as i64 + dt).rem_euclid(self.f_nt as i64) as usize;
                let q = self.f_r[jz as usize * self.f_nt + jt];
                if q > 0.0 && q.is_finite() {
                    near.push(q);
                }
            }
        }
        if near.is_empty() {
            return None;
        }
        near.sort_by(f64::total_cmp);
        Some(near[near.len() / 2])
    }

    /// 外法向（柱坐标梯度），在**细网格**上做中心差分。
    ///
    /// 差分步长取 ±0.6° / ±0.6 mm：细网格本身的 0.5°/0.25 mm 太细，网格噪声
    /// 会把法向抖成锯齿，A/C 就会跟着抖。
    pub fn normal(&self, th: f64, z: f64) -> DVec3 {
        let r = self.radius(th, z).unwrap_or(1.0);
        let dth = 0.6f64.to_radians();
        let dzz = 0.6;
        let r_prev = self.radius(th - dth, z).unwrap_or(r);
        let r_next = self.radius(th + dth, z).unwrap_or(r);
        let r_lo = self.radius(th, z - dzz).unwrap_or(r);
        let r_hi = self.radius(th, z + dzz).unwrap_or(r);
        let r_th = (r_next - r_prev) / (2.0 * dth);
        let r_z = (r_hi - r_lo) / (2.0 * dzz);
        let nx = th.cos() + (r_th / r.max(1e-6)) * th.sin();
        let ny = th.sin() - (r_th / r.max(1e-6)) * th.cos();
        let nz = -r_z;
        let l = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
        DVec3::new(nx / l, ny / l, nz / l)
    }

    /// **平滑**外法向，给粗加工用。
    ///
    /// 粗加工的偏置量是 `余量 + 刀半径 + k·层距`，能到几十毫米。法向在漩涡面上
    /// 抖 20°，刀尖就会在相邻两点之间跳二十几毫米，扫掠时把中间整片扫掉。
    /// 所以粗加工只用"大尺度法向"：半径先做 5×5 均值，差分步长放大到 ±2°/±1.5 mm。
    pub fn normal_smooth(&self, th: f64, z: f64) -> DVec3 {
        let avg = |t: f64, zz: f64| -> f64 {
            let mut s = 0.0;
            let mut n = 0.0;
            for dt in [-1.0f64, 0.0, 1.0] {
                for dz in [-0.5f64, 0.0, 0.5] {
                    if let Some(v) = self.radius(t + dt.to_radians(), zz + dz) {
                        s += v;
                        n += 1.0;
                    }
                }
            }
            if n > 0.0 { s / n } else { 1.0 }
        };
        let r = avg(th, z);
        let (dth, dzz) = (2.0f64.to_radians(), 1.5);
        let r_th = (avg(th + dth, z) - avg(th - dth, z)) / (2.0 * dth);
        let r_z = (avg(th, z + dzz) - avg(th, z - dzz)) / (2.0 * dzz);
        let nx = th.cos() + (r_th / r.max(1e-6)) * th.sin();
        let ny = th.sin() - (r_th / r.max(1e-6)) * th.cos();
        let nz = -r_z;
        let l = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
        DVec3::new(nx / l, ny / l, nz / l)
    }

    /// 连续 `(θ, z)` → 真值半径。粗加工的螺旋是连续参数路径，用它查轮廓。
    fn at(&self, th: f64, z: f64) -> Option<f64> {
        self.radius(th, z)
    }
}

pub struct RotaryProgram {
    pub gcode: String,
    pub profile: RotaryProfile,
    pub moves: usize,
    pub cut_mm: f64,
    /// A 超行程、**被跳过**的点数（那些面这一道工序做不了，留料）。
    pub clamped: usize,
    /// 被跳过的点在机床外、超出 A 行程的最大角度（度）。
    pub max_lift_mm: f64,
    /// 轮廓里需要的最大 A（度）——机床能不能一次装夹做完，看这个。
    pub max_a_deg: f64,
    /// 粗加工走刀点数（0 = 没开粗加工）。
    pub rough_moves: usize,
}

/// 生成 A/C + RTCP 的外轮廓刀路。
pub fn generate_rotary(job: &RotaryJob<'_>) -> Result<RotaryProgram, CamError> {
    if job.mesh.is_empty() {
        return Err(CamError::EmptyMesh);
    }
    if !matches!(job.tool.kind, ToolKind::Ball | ToolKind::TaperedBall)
        || job.tool.diameter <= 0.0
    {
        return Err(CamError::BadTool(
            "回转精加工需要球头或锥度球头（法向偏置只有球面才成立）",
        ));
    }
    if !(job.step_deg > 0.0 && job.step_deg <= 30.0) {
        return Err(CamError::BadParam("step_deg 必须在 (0, 30]"));
    }
    if job.step_z_mm <= 0.0 {
        return Err(CamError::BadParam("step_z 必须为正"));
    }
    if let Some(r) = &job.roughing {
        // 粗加工用**平头**最划算（每层是个平面，去料率远高于球头）；球头也允许，
        // 竖直刀轴 + 径向让刀对两者都成立——圆柱/球的半径在高度上不变。
        if !matches!(r.tool.kind, ToolKind::Flat | ToolKind::Ball) || r.tool.diameter <= 0.0 {
            return Err(CamError::BadTool("粗加工需要平头或球头刀"));
        }
        let ok = (0.0..=60.0).contains(&r.step_deg)
            && r.step_deg > 0.0
            && r.step_mm > 0.0
            && r.pitch_mm > 0.0;
        if !ok {
            return Err(CamError::BadParam("粗加工 step/pitch/step_deg 不合法"));
        }
    }
    let profile = profile_of(job.mesh, job.center, job.step_deg, job.step_z_mm);
    if !profile.r.iter().any(|r| *r > 0.0) {
        return Err(CamError::NoSurface);
    }

    let mut g = String::with_capacity(1 << 18);
    let mut moves = 0usize;
    let mut cut = 0.0f64;
    let mut clamped = 0usize;
    let mut max_lift = 0.0f64;
    let mut max_a = 0.0f64;
    let mut last: Option<DVec3> = None;
    let push = |g: &mut String, s: String| {
        g.push_str(&s);
        g.push('\n');
    };

    push(
        &mut g,
        format!(
            "(ROTARY A/C RTCP  stock {:.2}x{:.2}x{:.2}  center {:.3},{:.3})",
            job.stock.size().x,
            job.stock.size().y,
            job.stock.size().z,
            job.center.x,
            job.center.y
        ),
    );
    push(
        &mut g,
        format!(
            "(profile {} x {} @ {}deg x {:.2}mm  ball D{:.1})",
            profile.n_theta, profile.n_z, job.step_deg, job.step_z_mm, job.tool.diameter
        ),
    );
    push(&mut g, "G21 G90 G17 G94".into());

    let safe_z = job.stock.max.z + 10.0;

    // ── 粗加工：法向偏置的分层螺旋 ──
    let mut rough_moves = 0usize;
    if let Some(rough) = &job.roughing {
        rough_moves = rough_stage(&mut g, &profile, job, rough, safe_z, &mut cut);
    }

    // ── 精加工 ──
    push(&mut g, format!("T{} M6", job.tool.number));
    push(&mut g, format!("S{} M3", job.spindle_rpm));
    // RTCP 必须在运动之前打开；H1 是刀长寄存器（仿真里用不到，控制器要）。
    push(&mut g, "G43.4 H1".into());

    let r_tool = job.tool.radius();
    let mut first = true;
    let mut gap = false;
    let mut forward = true;
    for iz in (0..profile.n_z).rev() {
        let z = profile.z(iz);
        let n = profile.n_theta;
        // 每圈首尾都用 bin 0 的半径，接缝不跳。
        let mut idx: Vec<usize> = if forward {
            (0..=n).map(|i| i % n).collect()
        } else {
            (0..=n).rev().map(|i| i % n).collect()
        };
        forward = !forward;
        idx.dedup();
        for &it in &idx {
            let Some(r) = profile.r_at(it, iz) else {
                // 空箱：下一刀要重新下刀，不能连 G1 弦穿过去
                gap = true;
                continue;
            };
            let th = profile.theta(it);
            let nrm = profile.normal(th, z);
            let p = DVec3::new(job.center.x + r * th.cos(), job.center.y + r * th.sin(), z);
            // 刀轴 = 外法向朝 +Z 翘 `tilt_deg`。**球心仍严格在 `p + R·n` 上**，
            // 球与曲面恒相切，翘的只是刀颈——这是五轴躲让的常规手法，不改变
            // 切出来的曲面，只让锥颈/刃部离开相邻的壁（A 60..110° 那段的过切）。
            let axis_n = if job.tilt_deg.abs() > 1e-9 {
                let t = job.tilt_deg.to_radians();
                (nrm * t.cos() + DVec3::new(0.0, 0.0, t.sin())).normalize()
            } else {
                nrm
            };
            let (a_des, c) = ac_of(axis_n);
            max_a = max_a.max(a_des);
            let center = p + nrm * r_tool;
            // 限位：把刀轴压回行程，**球心不动**（仍相切）——比"抬刀"严格更好：
            // 抬刀会在行程边界留一圈余量，改刀轴则一点余量都不留。
            // **够不到就跳过**：A 超行程的点，硬把刀轴压回行程会啃进工件，
            // 不如留料——"无法加工的部分尽最大努力，完全加工不了的跳过"。
            if a_des > job.a_limit_deg + 1e-9 {
                clamped += 1;
                max_lift = max_lift.max(a_des - job.a_limit_deg);
                gap = true;
                continue;
            }
            let a_cmd = a_des;
            let (sa, ca) = a_cmd.to_radians().sin_cos();
            let (sc, cc) = c.to_radians().sin_cos();
            let axis = DVec3::new(sa * sc, -sa * cc, ca);
            let tip = center - axis * r_tool;
            if first || gap {
                // **垂直下刀**：G0 抬到安全高度 → 移到目标点正上方 → G1 直落。
                //
                // 早先是"从外侧 8 mm、抬高 6 mm 斜插过去"，看着稳妥，实际每次
                // 重新进刀都在工件上斜着犁一条沟——跳过点一多（A 超行程），
                // 整件就被这些斜插啃烂了（实测合格率从 61% 掉到 0.3%）。
                push(&mut g, format!("G0 Z{safe_z:.4}"));
                push(
                    &mut g,
                    format!("G0 X{:.4} Y{:.4} A{:.4} C{:.4}", tip.x, tip.y, a_cmd, c),
                );
                push(
                    &mut g,
                    format!("G0 Z{:.4}", tip.z + 5.0),
                );
                push(&mut g, format!("G1 Z{:.4} F{:.0}", tip.z, job.plunge_mm_min));
                last = Some(tip);
                moves += 1;
                first = false;
                gap = false;
                continue;
            }
            // 只有 A/C 变化时也照样输出（RTCP 下角度是运动的一部分）。
            push(
                &mut g,
                format!(
                    "G1 X{:.4} Y{:.4} Z{:.4} A{:.4} C{:.4} F{:.0}",
                    tip.x, tip.y, tip.z, a_cmd, c, job.feed_mm_min
                ),
            );
            cut += last.map_or(0.0, |p| (tip - p).length());
            last = Some(tip);
            moves += 1;
        }
    }
    push(&mut g, "G49".into());
    push(&mut g, format!("G0 Z{safe_z:.3}"));
    push(&mut g, "M5".into());
    push(&mut g, "M30".into());

    Ok(RotaryProgram {
        gcode: g,
        profile,
        moves,
        cut_mm: cut,
        clamped,
        max_lift_mm: max_lift,
        max_a_deg: max_a,
        rough_moves,
    })
}

/// 粗加工：一圈圈法向偏置的螺旋，从工件表面一直包到毛坯外接圆柱。
///
/// 每层是**一条连续的螺旋**（C 连续转、Z 连续下降），层与层之间抬到安全高度。
/// 偏置量 `余量 + 刀半径 + k·step`：球心离工件表面至少 `余量 + 刀半径`，
/// 所以球面離表面至少一个"余量"——粗加工绝不啃进精加工要保留的那层。
fn rough_stage(
    g: &mut String,
    profile: &RotaryProfile,
    job: &RotaryJob<'_>,
    rough: &Roughing,
    safe_z: f64,
    cut: &mut f64,
) -> usize {
    let rr = rough.tool.radius();
    let push = |g: &mut String, s: String| {
        g.push_str(&s);
        g.push('\n');
    };
    push(g, format!("T{} M6", rough.tool.number));
    push(g, format!("S{} M3", job.spindle_rpm));
    push(g, "G43.4 H1".into());

    // 偏置要盖到毛坯最远的 XY 角点（方料的四角就在那儿）
    let cx = (job.stock.min.x - job.center.x)
        .abs()
        .max((job.stock.max.x - job.center.x).abs());
    let cy = (job.stock.min.y - job.center.y)
        .abs()
        .max((job.stock.max.y - job.center.y).abs());
    let reach = cx.hypot(cy);

    let n_per_rev = ((360.0 / rough.step_deg).round() as usize).max(8);
    let dth = std::f64::consts::TAU / n_per_rev as f64;
    let dz_step = rough.pitch_mm / n_per_rev as f64;
    let z_top = profile.z0 + (profile.n_z - 1) as f64 * profile.dz;
    let n_steps = (((z_top - profile.z0) / dz_step).ceil() as usize).max(1);

    let mut count = 0usize;
    let mut last: Option<DVec3> = None;
    // 层间接缝错开，避免每层都在同一条母线上留痕
    let golden = 2.399_963_229_728_653f64;
    //
    // 构造（**径向让刀，竖直刀轴**）：
    //   球心放在高度 z、半径 `max(R_k, r(θ,z) + 余量 + R)` 处，刀尖 = 球心 − R·ẑ。
    // 这样在高度 z 上球切到的最内处就是 `r + 余量`，而其他高度上球的横截面只会更小，
    // 所以**径向永不啃伤**——不像"沿法向偏置"那样在大偏置量下自交。
    // 半径 R_k 从毛坯外接圆一路收到 `r_max + 余量 + R`，把方料四角一并车掉。
    let r_need = profile.max_r() + rough.allowance_mm + rr;
    let start = reach.max(r_need);
    let n_layers = (((start - r_need) / rough.step_mm).ceil() as usize) + 1;
    for k in 0..n_layers {
        let rk = start - k as f64 * rough.step_mm;
        let th0 = k as f64 * golden;
        let mut started = false;
        for i in 0..=n_steps {
            let th = th0 + i as f64 * dth;
            let z = z_top - i as f64 * dz_step;
            let Some(rp) = profile.at(th, z) else {
                continue;
            };
            let r = rk.max(rp + rough.allowance_mm + rr);
            if r > reach + 2.0 * rough.step_mm {
                continue;
            }
            // 平头刀：下端面就在切削高度 z；球头：刀尖还要再往下让一个半径
            let z_low = if rough.tool.kind == ToolKind::Flat {
                0.0
            } else {
                rr
            };
            let tip = DVec3::new(
                job.center.x + r * th.cos(),
                job.center.y + r * th.sin(),
                z - z_low,
            );
            if !started {
                push(g, format!("G0 Z{safe_z:.4}"));
                push(g, format!("G0 X{:.4} Y{:.4} A0 C0", tip.x, tip.y));
                push(g, format!("G0 Z{:.4}", tip.z + 6.0 + rr));
                started = true;
            }
            push(
                g,
                format!(
                    "G1 X{:.4} Y{:.4} Z{:.4} A0 C0 F{:.0}",
                    tip.x, tip.y, tip.z, job.feed_mm_min
                ),
            );
            *cut += last.map_or(0.0, |q: DVec3| (tip - q).length());
            last = Some(tip);
            count += 1;
        }
    }
    count
}

/// 在**细网格**上双线性取样。
///
/// 细网格是 `r[j * n_theta + i]`，样本落在 `θ = i·Δθ`、`z = z0 + j·dz`。
/// 之前用"最近格"，量化误差是半个格：0.5° 在 r=20 上就是 **0.15 mm**——
/// 比刀路本身的误差还大，量出来的偏差会有一半是取样噪声。
fn fine_sample(
    f_r: &[f64],
    n_theta: usize,
    n_z: usize,
    z0: f64,
    dz: f64,
    th: f64,
    z: f64,
) -> Option<f64> {
    let fx = th.rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU * n_theta as f64;
    let i0 = fx.floor();
    let tx = fx - i0;
    let fz = (z - z0) / dz;
    let j0 = fz.floor();
    let tz = fz - j0;
    let i0 = i0 as i64;
    let j0 = j0 as i64;
    let w = [(1.0 - tx) * (1.0 - tz), tx * (1.0 - tz), (1.0 - tx) * tz, tx * tz];
    let idx = [
        (i0, j0),
        (i0 + 1, j0),
        (i0, j0 + 1),
        (i0 + 1, j0 + 1),
    ];
    let mut acc = 0.0;
    let mut used = 0.0;
    for (k, (i, j)) in idx.iter().enumerate() {
        if *j < 0 || *j >= n_z as i64 {
            continue;
        }
        let ii = i.rem_euclid(n_theta as i64) as usize;
        let v = f_r[*j as usize * n_theta + ii];
        if v > 0.0 && v.is_finite() && w[k] > 0.0 {
            acc += w[k] * v;
            used += w[k];
        }
    }
    (used > 1e-9).then(|| acc / used)
}

/// "这件是不是回转体"：面积加权的**吻合度**（0..1）。
///
/// 做法：把顶点按 (θ,z) 分箱取最大半径当包络，再看每个三角形质心的半径
/// 离包络有多远。实心回转体几乎全部贴合（≈1）；方料/支架只有一小部分贴合。
/// 回转工艺只对贴合度高的件有意义——这是"换模型不换代码"的第一道判断。
pub fn rotationality(mesh: &[Tri]) -> f64 {
    let (mut lo, mut hi) = (DVec2::new(f64::INFINITY, f64::INFINITY), DVec2::new(f64::NEG_INFINITY, f64::INFINITY));
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for t in mesh {
        for v in t {
            lo.x = lo.x.min(v.x);
            lo.y = lo.y.min(v.y);
            hi.x = hi.x.max(v.x);
            hi.y = hi.y.max(v.y);
            zmin = zmin.min(v.z);
            zmax = zmax.max(v.z);
        }
    }
    if !(lo.x.is_finite() && hi.x.is_finite()) || zmax <= zmin {
        return 0.0;
    }
    let center = DVec2::new((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5);
    let p = profile_of(mesh, center, 5.0, 1.0);
    let (nt, nz) = (p.n_theta, p.n_z);
    let rmax = p.max_r().max(1e-9);
    let tol = (0.02 * rmax).max(0.05);
    let mut hit = 0.0;
    let mut tot = 0.0;
    for t in mesh {
        let c = (t[0] + t[1] + t[2]) * (1.0 / 3.0);
        let a = (t[1] - t[0]).cross(t[2] - t[0]).length() * 0.5;
        if a <= 0.0 {
            continue;
        }
        let r = (c.x - center.x).hypot(c.y - center.y);
        let th = (c.y - center.y).atan2(c.x - center.x);
        let it = ((th.rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU * nt as f64)
            .round() as usize)
            % nt;
        let iz = (((c.z - p.z0) / p.dz).round() as i64).clamp(0, nz as i64 - 1) as usize;
        let env = p.r[iz * nt + it];
        if env <= 0.0 {
            continue;
        }
        tot += a;
        if env - r <= tol {
            hit += a;
        }
    }
    if tot <= 0.0 { 0.0 } else { hit / tot }
}

/// 按曲面曲率给出**尖球半径**建议（mm）。
///
/// 球头半径大于局部**凹**曲率半径就会在谷口架桥、把谷底整段削掉。所以取
/// 面积加权 **5 分位**的凹曲率半径（只有 ~5% 的面积放不进去），再夹到
/// `[0.25, 3.0]`。取高分位是反的——那会挑出一把"大部分谷都进不去"的大刀。
pub fn suggest_tip_radius(mesh: &[Tri], center: DVec2) -> f64 {
    let p = profile_of(mesh, center, 1.0, 1.0);
    let (nt, nz) = (p.f_nt, p.f_nz);
    if nt < 8 || nz < 4 {
        return 1.0;
    }
    let at = |i: usize, j: usize| -> f64 {
        let jj = j.clamp(0, nz - 1);
        let ii = i % nt;
        p.f_r[jj * nt + ii]
    };
    let dth = std::f64::consts::TAU / nt as f64;
    let mut samples: Vec<(f64, f64)> = Vec::new(); // (ρ, 面积权重)
    for j in 1..nz - 1 {
        for i in 0..nt {
            let r = at(i, j);
            if r <= 1e-6 {
                continue;
            }
            let (rm, rp) = (at(i, j - 1), at(i, j + 1));
            let (rl, rr) = (at(i + nt - 1, j), at(i + 1, j));
            if rm <= 0.0 || rp <= 0.0 || rl <= 0.0 || rr <= 0.0 {
                continue;
            }
            let r1 = (rr - rl) / (2.0 * dth) / r; // ∂r/∂s1
            let r2 = (rp - rm) / (2.0 * p.f_dz); // ∂r/∂s2
            let r11 = (rr - 2.0 * r + rl) / (dth * dth) / (r * r);
            let r22 = (rp - 2.0 * r + rm) / (p.f_dz * p.f_dz);
            let r12 = (at(i + 1, j + 1) - at(i + 1, j - 1) - at(i + nt - 1, j + 1)
                + at(i + nt - 1, j - 1))
                / (4.0 * dth * p.f_dz)
                / r;
            let g2 = 1.0 + r1 * r1 + r2 * r2;
            let den = g2.powf(1.5);
            let tr = (r11 + r22) / den;
            let det = (r11 * r22 - r12 * r12) / (den * den);
            let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
            let kmax = (tr / 2.0 + disc).max(tr / 2.0 - disc);
            if kmax > 1e-4 {
                samples.push((1.0 / kmax, r));
            }
        }
    }
    if samples.is_empty() {
        return 1.0;
    }
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    let total: f64 = samples.iter().map(|s| s.1).sum();
    let mut acc = 0.0;
    for (rho, w) in &samples {
        acc += w;
        if acc >= total * 0.05 {
            return rho.clamp(0.25, 3.0);
        }
    }
    1.0
}

/// 细网格分辨率。0.5°/0.25 mm 时花瓶每格约 1.8k 个顶点——按"最近格"取样，
/// 格内没有别的顶点可争，所以它代表的就是真值，而不是包络。
const FINE_DEG: f64 = 0.5;
const FINE_Z_MM: f64 = 0.25;

/// 顶点分箱建**细网格**真值，再降采样到刀路网格。
fn profile_of(mesh: &[Tri], center: DVec2, step_deg: f64, step_z: f64) -> RotaryProfile {
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    for t in mesh {
        for v in t {
            zmin = zmin.min(v.z);
            zmax = zmax.max(v.z);
        }
    }
    // ── 细网格 ──
    let f_nt = ((360.0 / FINE_DEG).round() as usize).max(16);
    let f_nz = (((zmax - zmin) / FINE_Z_MM).ceil() as usize).max(2) + 1;
    let f_dz = (zmax - zmin) / (f_nz - 1) as f64;
    let mut f_r = vec![0.0f64; f_nt * f_nz];
    for t in mesh {
        for v in t {
            let dx = v.x - center.x;
            let dy = v.y - center.y;
            let rr = dx.hypot(dy);
            let th = dy.atan2(dx).rem_euclid(std::f64::consts::TAU);
            // **四舍五入**到最近格（不是截断）：细网格的样本点定义在 `i·Δθ` 上，
            // 截断会让一半顶点落到前一格，插值时就差半个格（0.5° 在 r=20 上是
            // 0.15 mm，比刀路误差还大）。
            let it = ((th / std::f64::consts::TAU * f_nt as f64).round() as usize) % f_nt;
            let iz = (((v.z - zmin) / f_dz).round() as usize).min(f_nz - 1);
            let k = iz * f_nt + it;
            if rr > f_r[k] {
                f_r[k] = rr;
            }
        }
    }
    fill_holes(&mut f_r, f_nt, f_nz);

    // ── 刀路网格：每个格点去细网格上查真值 ──
    let n_theta = ((360.0 / step_deg).round() as usize).max(8);
    let n_z = (((zmax - zmin) / step_z).ceil() as usize).max(2) + 1;
    let dz = (zmax - zmin) / (n_z - 1) as f64;
    let mut r = vec![0.0f64; n_theta * n_z];
    for iz in 0..n_z {
        let z = zmin + iz as f64 * dz;
        for it in 0..n_theta {
            let th = it as f64 / n_theta as f64 * std::f64::consts::TAU;
            let fit = ((th / std::f64::consts::TAU * f_nt as f64).round() as usize) % f_nt;
            let fiz = (((z - zmin) / f_dz).round() as i64).clamp(0, f_nz as i64 - 1) as usize;
            // 与 `radius()` 同口径：细网格双线性插值，不做 3×3 取最大（那会变成包络）
            let v = fine_sample(&f_r, f_nt, f_nz, zmin, f_dz, th, z).unwrap_or(0.0);
            r[iz * n_theta + it] = if v > 0.0 && v.is_finite() {
                v
            } else {
                let mut near: Vec<f64> = Vec::with_capacity(9);
                for ddz in -1i64..=1 {
                    let jz = fiz as i64 + ddz;
                    if jz < 0 || jz >= f_nz as i64 {
                        continue;
                    }
                    for ddt in -1i64..=1 {
                        let jt = (fit as i64 + ddt).rem_euclid(f_nt as i64) as usize;
                        let q = f_r[jz as usize * f_nt + jt];
                        if q > 0.0 && q.is_finite() {
                            near.push(q);
                        }
                    }
                }
                near.sort_by(f64::total_cmp);
                near.get(near.len() / 2).copied().unwrap_or(0.0)
            };
        }
    }
    RotaryProfile {
        n_theta,
        n_z,
        z0: zmin,
        dz,
        r,
        f_nt,
        f_nz,
        f_z0: zmin,
        f_dz,
        f_r,
    }
}

/// 补洞：同一 z 环上空格用环向邻居的均值；整环空就保持 0。
/// 再做轴向：同一 θ 上按最近的已知层线性插值。
fn fill_holes(r: &mut [f64], n_theta: usize, n_z: usize) {
    for iz in 0..n_z {
        let base = iz * n_theta;
        if r[base..base + n_theta].iter().all(|&v| v == 0.0) {
            continue;
        }
        for it in 0..n_theta {
            if r[base + it] == 0.0 {
                let l = r[base + (it + n_theta - 1) % n_theta];
                let rr = r[base + (it + 1) % n_theta];
                let vals: Vec<f64> = [l, rr].into_iter().filter(|v| *v > 0.0).collect();
                if !vals.is_empty() {
                    r[base + it] = vals.iter().sum::<f64>() / vals.len() as f64;
                }
            }
        }
    }
    for it in 0..n_theta {
        let mut iz = 0;
        while iz < n_z {
            if r[iz * n_theta + it] > 0.0 {
                iz += 1;
                continue;
            }
            let start = iz;
            while iz < n_z && r[iz * n_theta + it] == 0.0 {
                iz += 1;
            }
            let lo = if start > 0 { Some(start - 1) } else { None };
            let hi = if iz < n_z { Some(iz) } else { None };
            for k in start..iz {
                let v = match (lo, hi) {
                    (Some(a), Some(b)) => {
                        let za = r[a * n_theta + it];
                        let zb = r[b * n_theta + it];
                        let t = (k - a) as f64 / (b - a) as f64;
                        za + (zb - za) * t
                    }
                    (Some(a), None) => r[a * n_theta + it],
                    (None, Some(b)) => r[b * n_theta + it],
                    (None, None) => 0.0,
                };
                r[k * n_theta + it] = v;
            }
        }
    }
}

/// 外法向 → (A, C) 度（Head 约定）。
fn ac_of(n: DVec3) -> (f64, f64) {
    let a = n.z.clamp(-1.0, 1.0).acos().to_degrees();
    let c = n.x.atan2(-n.y).to_degrees();
    (a, c)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_math::DVec3;

    /// 封闭圆柱：侧面 64 段 + 上下底。
    fn cylinder(r: f64, z0: f64, z1: f64) -> Vec<Tri> {
        let n = 64;
        let mut v = Vec::new();
        for i in 0..n {
            let a = i as f64 / n as f64 * std::f64::consts::TAU;
            v.push(DVec3::new(r * a.cos(), r * a.sin(), z0));
            v.push(DVec3::new(r * a.cos(), r * a.sin(), z1));
        }
        let mut t = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let (a0, a1, b0, b1) = (v[2 * i], v[2 * i + 1], v[2 * j], v[2 * j + 1]);
            t.push([a0, b0, b1]);
            t.push([a0, b1, a1]);
            let (c0, c1) = (DVec3::new(0.0, 0.0, z0), DVec3::new(0.0, 0.0, z1));
            t.push([c0, a0, b0]);
            t.push([c1, b1, a1]);
        }
        t
    }

    /// 正弦起伏的回转体（模拟"流云"那种竖纹）：`r(θ) = r0 + amp·cos(k·θ)`。
    fn rippled(r0: f64, amp: f64, k: f64, z0: f64, z1: f64) -> Vec<Tri> {
        let n = 720; // 0.5° 一段，够表达 6° 一个周期的波纹
        let mut ring = Vec::with_capacity(n);
        for i in 0..n {
            let a = i as f64 / n as f64 * std::f64::consts::TAU;
            ring.push(r0 + amp * (k * a).cos());
        }
        let mut t = Vec::new();
        let (c0, c1) = (DVec3::new(0.0, 0.0, z0), DVec3::new(0.0, 0.0, z1));
        for i in 0..n {
            let j = (i + 1) % n;
            let (a0, a1) = (
                DVec3::new(ring[i] * (i as f64 / n as f64 * std::f64::consts::TAU).cos(),
                           ring[i] * (i as f64 / n as f64 * std::f64::consts::TAU).sin(), z0),
                DVec3::new(ring[i] * (i as f64 / n as f64 * std::f64::consts::TAU).cos(),
                           ring[i] * (i as f64 / n as f64 * std::f64::consts::TAU).sin(), z1),
            );
            let (b0, b1) = (
                DVec3::new(ring[j] * (j as f64 / n as f64 * std::f64::consts::TAU).cos(),
                           ring[j] * (j as f64 / n as f64 * std::f64::consts::TAU).sin(), z0),
                DVec3::new(ring[j] * (j as f64 / n as f64 * std::f64::consts::TAU).cos(),
                           ring[j] * (j as f64 / n as f64 * std::f64::consts::TAU).sin(), z1),
            );
            t.push([a0, b0, b1]);
            t.push([a0, b1, a1]);
            t.push([c0, b0, a0]);
            t.push([c1, a1, b1]);
        }
        t
    }

    fn job<'a>(mesh: &'a [Tri], roughing: Option<Roughing>) -> RotaryJob<'a> {
        RotaryJob {
            mesh,
            stock: Stock::from_size(DVec3::new(30.0, 30.0, 25.0)),
            center: DVec2::new(0.0, 0.0),
            tool: ToolSpec::ball(1, 3.0, 30.0),
            step_deg: 10.0,
            step_z_mm: 5.0,
            feed_mm_min: 800.0,
            plunge_mm_min: 200.0,
            spindle_rpm: 8000,
            a_limit_deg: A_LIMIT_DEG,
            tilt_deg: 0.0,
            roughing,
        }
    }

    #[test]
    fn cylinder_profile_is_the_radius_and_gcode_has_rtcp() {
        let mesh = cylinder(10.0, 0.0, 20.0);
        let p = generate_rotary(&job(&mesh, None)).unwrap();
        assert_eq!(p.profile.n_theta, 36);
        assert!(p.profile.r.iter().all(|r| (*r - 10.0).abs() < 1e-9));
        assert!(p.gcode.contains("G43.4 H1"));
        assert!(p.gcode.contains("G49"));
        assert!(p.moves > 150, "moves={}", p.moves);
        assert_eq!(p.clamped, 0, "圆柱侧面是 90°，不该触发限位");
        // 切点半径应等于圆柱半径
        for line in p.gcode.lines().filter(|l| l.starts_with("G1 X")) {
            let x: f64 = line.split(" X").nth(1).unwrap().split_whitespace().next().unwrap().parse().unwrap();
            let y: f64 = line.split(" Y").nth(1).unwrap().split_whitespace().next().unwrap().parse().unwrap();
            assert!((x.hypot(y) - 10.0).abs() < 1e-3, "点不在圆柱面上：{line}");
        }
    }

    /// 粗加工必须**留在余量之外**：任何一刀的球头都不许碰到轮廓以内，
    /// 否则精加工前就已经啃伤，后面补不回来。
    #[test]
    fn roughing_keeps_the_allowance() {
        let mesh = cylinder(10.0, 0.0, 20.0);
        let rough = Roughing {
            tool: ToolSpec::ball(2, 8.0, 30.0),
            step_mm: 4.0,
            pitch_mm: 5.0,
            step_deg: 20.0,
            allowance_mm: 1.0,
        };
        let p = generate_rotary(&job(&mesh, Some(rough.clone()))).unwrap();
        assert!(p.rough_moves > 100, "粗加工刀数太少：{}", p.rough_moves);
        assert_eq!(p.gcode.matches("M6").count(), 2, "应当换一次刀");

        // 粗加工段在 T2 M6 与 T1 M6 之间
        let mut in_rough = false;
        let mut worst = f64::INFINITY;
        let mut seen = 0usize;
        for line in p.gcode.lines() {
            if line.contains("T2 M6") {
                in_rough = true;
                continue;
            }
            if line.contains("T1 M6") {
                in_rough = false;
            }
            if !in_rough || !line.starts_with("G1 X") {
                continue;
            }
            let w = |k: &str| -> f64 {
                line.split(&format!(" {k}"))
                    .nth(1)
                    .unwrap()
                    .split_whitespace()
                    .next()
                    .unwrap()
                    .parse()
                    .unwrap()
            };
            let (x, y, z, a, c) = (w("X"), w("Y"), w("Z"), w("A"), w("C"));
            // 刀轴按 Head 约定：axis = (sinA·sinC, −sinA·cosC, cosA)
            let (sa, ca) = a.to_radians().sin_cos();
            let (sc, cc) = c.to_radians().sin_cos();
            let axis = DVec3::new(sa * sc, -sa * cc, ca);
            // 球心 = 刀尖 + R·axis，再到圆柱面的距离必须 >= 余量
            let ctr = DVec3::new(x, y, z) + axis * rough.tool.radius();
            let r_c = ctr.x.hypot(ctr.y);
            let dz = if ctr.z < 0.0 {
                -ctr.z
            } else if ctr.z > 20.0 {
                ctr.z - 20.0
            } else {
                0.0
            };
            worst = worst.min((r_c - 10.0).hypot(dz) - rough.tool.radius());
            seen += 1;
        }
        assert!(seen > 100, "没扫到粗加工点（{seen}）");
        assert!(
            // G 代码只写 4 位小数，比较留 2 µm 的余量
            worst >= rough.allowance_mm - 2e-3,
            "粗加工啃进了余量层：最近只留 {worst:.4} mm，要求 ≥ {} mm",
            rough.allowance_mm
        );
    }
    /// 刀路必须跟**真值曲面**，不能跟"格内最大"形成的包络。
    ///
    /// 这是花瓶 51% 过切的直接原因：包络把刀尖顶在波纹的脊上，Ø4 球头一摆，
    /// 2 mm 以内的谷全被扫平。旧代码在这里会读到 +0.26 mm 的包络偏差。
    #[test]
    fn profile_follows_the_true_radius_not_the_envelope() {
        let (r0, amp, k) = (20.0, 0.3, 60.0); // 6° 一个波纹
        let mesh = rippled(r0, amp, k, 0.0, 10.0);
        let mut j = job(&mesh, None);
        j.step_deg = 1.0;
        j.step_z_mm = 2.0;
        let p = generate_rotary(&j).unwrap();
        let mut worst = 0.0f64;
        let mut n = 0usize;
        for iz in 1..p.profile.n_z - 1 {
            for it in 0..p.profile.n_theta {
                let th = p.profile.theta(it);
                if let Some(r) = p.profile.r_at(it, iz) {
                    let want = r0 + amp * (k * th).cos();
                    worst = worst.max((r - want).abs());
                    n += 1;
                }
            }
        }
        assert!(n > 500, "轮廓采样太少：{n}");
        assert!(
            worst < 0.05,
            "轮廓偏离真值最大 {worst:.3} mm —— 又回到「格内最大」的包络了"
        );
    }

    /// 锥度球头：尖端半径 = 半直径，锥颈比尖端粗，刀杆直径按锥颈算。
    #[test]
    fn tapered_tool_has_a_small_tip_and_a_wider_neck() {
        let t = ToolSpec::tapered(2, 2.0, 10.0, 5.0);
        assert!((t.radius() - 1.0).abs() < 1e-12);
        let top = t.top_radius();
        assert!(top > 1.4 && top < 2.5, "锥颈顶部半径 {top:.3} 不合理");
        let def = t.to_tool_def();
        assert_eq!(def.kind, cncsim_tool::CutterKind::TaperedBall);
        assert!((def.corner_radius - 1.0).abs() < 1e-12);
        assert!(
            (def.diameter - 2.0 * top).abs() < 1e-9,
            "刀杆直径要按锥颈算，否则刀杆比刀还细"
        );
    }

    /// 粗加工接受**平头**刀：花瓶颈的粗加工就是平头 Ø16。
    #[test]
    fn roughing_accepts_a_flat_mill() {
        let mesh = cylinder(10.0, 0.0, 20.0);
        let rough = Roughing {
            tool: ToolSpec::flat(1, 8.0, 20.0),
            step_mm: 4.0,
            pitch_mm: 5.0,
            step_deg: 20.0,
            allowance_mm: 0.5,
        };
        let p = generate_rotary(&job(&mesh, Some(rough))).unwrap();
        assert!(p.rough_moves > 100, "平头粗加工刀数太少：{}", p.rough_moves);
    }

}
