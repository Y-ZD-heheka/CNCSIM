//! **半球凸台场景**：50×50×40 毛坯，顶面铣掉一圈，中间留下一个 R15 的**凸**半球。
//!
//! 球心放在 `z = 24`（顶面下 16 mm）：球顶在 z=39，赤道落在 z=24 的
//! 底面平面上。要去掉的是「顶面下面 15 mm 里、球面以外」的 ~30 000 mm³。
//!
//! | 工序 | 刀 | 策略 | 留量 |
//! |---|---|---|---|
//! | 粗加工 | T1 φ6 平底（刃长 20） | Z 层切，层降 1，**从内缘往外清环带** + 一圈矩形清四角 | 法向 0.5（最后一层 0.02） |
//! | 精加工 | T2 R0.5 锥度球头（4.69°/30.5/Ø6×75） | 沿球面**外侧**等弧长螺旋，行距 0.4 | 0 |
//!
//! ## 刀心半径：凸台是「往外让」，型腔是「往里收」
//!
//! 平底刀（刀底内缘在 `r_c − r_tool`，刀底在 `z`）不许碰到 R 的球：
//!
//! ```text
//! √((r_c − r_tool)² + d²) ≥ R + allow      d = z − 球心z
//! ⟹ r_c ≥ r_tool + √((R + allow)² − d²)
//! ```
//!
//! 和型腔那条 `r_max = √((R−allow)² − d²) − r_tool` 是一对镜像。**刀底内缘是
//! 整把刀离球心最近的点**（往上 z 变大、离球心只会更远），所以只查这一圈就够。
//!
//! ## 精加工：刀心在球**外** 15.5
//!
//! 球头刀与凸球面相切 ⟺ 刀心落在半径 `R + r_b` 的球面上（型腔是 `R − r_b`）：
//!
//! ```text
//! 刀心 B(θ,φ) = C + (R + r_b)·(sinθ·cosφ, sinθ·sinφ, +cosθ)   ← 注意 +cos
//! 刀尖 T      = B − (0, 0, r_b)
//! ```
//!
//! θ 从 `acos(r_b/(R+r_b)) = 88.15°` 走到 0°。**起点不是随便取的**：再往外
//! 刀尖就低于底面 z=24 了（刀尖 z = 24 + (R+r_b)cosθ − r_b ≥ 24 ⟺ cosθ ≥ r_b/(R+r_b)）。
//! 于是赤道根部必然留一圈 R0.5 的清角残料——球头刀在「竖直面与水平面的内角」
//! 处本来就清不干净，报告里的残留会如实报出来。
//!
//! ## 锥颈在赤道附近仍会蹭球面（0.03 mm）
//!
//! 直觉上凸台比型腔友好（锥颈在球外面往上张开，越往上离材料越远），但**赤道
//! 那一段球面也是竖直的**：刀心在球外 15.5，锥颈在 1.2 mm 高处半径 0.555，
//! 最内点到轴 14.94，而球面在 z=86.2 处半径 14.95——差 0.03 mm，蹭进去。
//!
//! 实测 **0.030 mm / 3.4 mm³**，全在 z≈86 那一圈。型腔版是 0.05 mm（压顶面），
//! 凸台版是 0.03 mm（压球面），同一个根因：**4.69° 的锥颈 + 竖直面 = 必然干涉**。
//! 想真的消掉只有三条路：换直柄球头刀、把工件摆斜（五轴）、或者让 CAM 在这圈
//! 往外让 0.03（拿残留换过切）。演示选第四条：**如实报**。

use cncsim_math::{DVec2, DVec3};
use cncsim_tool::{CutterKind, Profile, ToolDef, flat_end_mill, tapered_ball_end_mill};

/// 毛坯尺寸（mm）：50 × 50 × 40
pub const STOCK_MM: [f64; 3] = [50.0, 50.0, 40.0];

/// 凸台球半径（mm）
pub const DOME_R: f64 = 15.0;
/// 球心。球顶 = `z + R = 39`，**比毛坯顶面低 1 mm**。
///
/// 这一毫米是刻意的：球顶若正好落在毛坯顶面（z=40）上，球顶周围半径
/// `√(2R·a) ≈ 3.9` 以内那层球冠**任何刀具都够不到**——要切它就得比它更靠近
/// 球顶，而一靠近就切进球。实测那样会在成品球顶留一块 6 mm 见方的**毛坯面**
/// （色索引 0，灰的），体积 12.8 mm³。
///
/// 降 1 mm 之后，第一刀**面铣**（平底刀，刀底正好在球顶平面）能切掉这一层：
/// 平的刀底与球顶相切不算切球，球头刀做不到这件事。
pub const DOME_CENTER: DVec3 = DVec3::new(25.0, 25.0, 24.0);

/// 粗加工法向留量（mm）
pub const ROUGH_ALLOW_MM: f64 = 0.5;
/// 最后一层的留量：几乎贴住赤道，把底面那一圈铣平（留给精加工的只有 0.02 的壳）
pub const ROUGH_FLOOR_ALLOW_MM: f64 = 0.02;
/// 粗加工层降（mm）
pub const ROUGH_STEPDOWN_MM: f64 = 1.0;
/// 粗加工行距（mm）：φ6 的 2/3
pub const ROUGH_STEPOVER_MM: f64 = 4.0;
/// **底层**同心圆的最大刀心半径（mm）：要盖住方块对角 `√2·25 = 35.4`。
///
/// 早先取 26（只盖到边、盖不到角），四角留下 3169 mm³ 的料柱——`残留最厚 15 mm`
/// 报的就是它。角落不是"差不多就行"的地方：方形毛坯的对角比边远 41%。
pub const ROUGH_RING_MAX_MM: f64 = 36.0;

/// 精加工行距（mm，沿球面量）
pub const FINISH_STEPOVER_MM: f64 = 0.4;
/// 精加工折线弦长上限（mm）
pub const FINISH_CHORD_MM: f64 = 1.2;

/// 锥度球头刀规格（供应商表：`R0.5*30.5*D6*75 单边斜角 4.69°`）
pub const FINISH_TIP_R: f64 = 0.5;
pub const FINISH_TAPER_DEG: f64 = 4.69;
pub const FINISH_FLUTE_MM: f64 = 30.5;
pub const FINISH_OVERALL_MM: f64 = 75.0;

pub const ROUGH_TOOL_NO: u16 = 1;
pub const FINISH_TOOL_NO: u16 = 2;

/// T1：φ6 平底刀，刃长 20、全长 60。
pub fn rough_tool() -> ToolDef {
    ToolDef {
        number: ROUGH_TOOL_NO,
        kind: CutterKind::Flat,
        diameter: 6.0,
        corner_radius: 0.0,
        flute_length: 20.0,
        shank: Profile::new(vec![
            DVec2::new(0.0, 20.0),
            DVec2::new(3.0, 20.0),
            DVec2::new(3.0, 60.0),
            DVec2::new(0.0, 60.0),
        ]),
        length_offset: 60.0,
        cutter: flat_end_mill(6.0, 20.0),
    }
}

/// T2：锥度球头刀（规格表那把）。
///
/// 刀柄按柄径 Ø6 画成直筒：锥面顶部实际是 Ø5.93（表里的 4.69° 是从刀尖量的
/// 弦角），差 0.037 mm——碰撞检测上保守，显示上看不出来。
pub fn finish_tool() -> ToolDef {
    ToolDef {
        number: FINISH_TOOL_NO,
        kind: CutterKind::TaperedBall,
        diameter: 2.0 * FINISH_TIP_R,
        corner_radius: FINISH_TIP_R,
        flute_length: FINISH_FLUTE_MM,
        shank: Profile::new(vec![
            DVec2::new(0.0, FINISH_FLUTE_MM),
            DVec2::new(3.0, FINISH_FLUTE_MM),
            DVec2::new(3.0, FINISH_OVERALL_MM),
            DVec2::new(0.0, FINISH_OVERALL_MM),
        ]),
        length_offset: FINISH_OVERALL_MM,
        cutter: tapered_ball_end_mill(FINISH_TIP_R, FINISH_TAPER_DEG, FINISH_FLUTE_MM, 24),
    }
}

/// 设计成品：盒 ∪ 球（球心在盒顶面上 ⇒ 上半球凸台）。
pub fn design() -> crate::compare::DesignModel {
    crate::compare::DesignModel::Boss {
        min: DVec3::ZERO,
        max: DVec3::new(STOCK_MM[0], STOCK_MM[1], DOME_CENTER.z),
        center: DOME_CENTER,
        radius: DOME_R,
    }
}

/// 某一层（刀尖 z）平底刀**刀心**能到的最近半径。
///
/// `r_tool + √((R+allow)² − d²)`：刀底内缘正好落在"离球面 allow"的等距球面上。
/// 最后一层用 `ROUGH_FLOOR_ALLOW_MM`，几乎贴住赤道，把底面铣平。
pub fn rough_min_center_radius(z_tip: f64) -> f64 {
    let d = (z_tip - DOME_CENTER.z).max(0.0);
    let allow = if d < 1e-9 { ROUGH_FLOOR_ALLOW_MM } else { ROUGH_ALLOW_MM };
    let r_in = ((DOME_R + allow).powi(2) - d * d).max(0.0).sqrt();
    r_in + rough_tool().cutter.max_radius()
}

/// 上层的圆走到这个半径就停：**再往外的料由底层那一刀一次切掉**。
///
/// 刀具刃长 20 > 加工深度 15，所以底层（z=24）那一刀的刀身覆盖 `24..44`，
/// 半径 ≥ 15.02 的部分在整个高度上都被它清了。上层只需要补"球面周围那一圈"。
/// 少了这一条，每层都要走到 36，路径长 4 倍（15.5 分钟 vs 4.3 分钟）。
pub fn rough_ring_cap() -> f64 {
    rough_min_center_radius(DOME_CENTER.z)
}

/// 球顶高度（= 毛坯顶面 − 1）。面铣层就切到这儿。
pub const DOME_TOP: f64 = DOME_CENTER.z + DOME_R;

/// 面铣层的**刀心**半径序列：刀径 6、步距 4，从中心铺到 `rough_ring_cap()`。
///
/// 只铺到 `cap`（18.02）就够：再往外的 `z ∈ [DOME_TOP, 40]` 那薄薄一层由
/// 下一层（z=38）的刀身切掉——刃长 20，刀身在 z 方向从 38 一直到 58。
pub fn face_mill_rings() -> Vec<f64> {
    let r_tool = rough_tool().cutter.max_radius();
    let cap = rough_ring_cap();
    let mut out = Vec::new();
    let mut r = r_tool; // 刀心离轴一个刀半径 ⇒ 刀正好扫到中心
    while r <= cap + 1e-9 {
        out.push(r);
        r += ROUGH_STEPOVER_MM;
    }
    out
}

/// 粗加工的层深（刀尖 z），**从上往下**：38, 37, …, 25，最后一层 24（底面）。
///
/// 第一层不是 39：那是**面铣层**（刀底正好在球顶平面），单独由
/// [`face_mill_rings`] 生成——它的内缘约束是 0，不是"离球面 allow"。
///
/// 先切上面：台阶朝下，精加工从上往下扫时是顺铣；反过来会在每层台阶的
/// 立面下留一圈薄壳，精加工得多切一刀。
pub fn rough_levels() -> Vec<f64> {
    let mut out = Vec::new();
    let mut z = DOME_TOP; // 球顶平面，已被面铣层覆盖
    loop {
        z -= ROUGH_STEPDOWN_MM;
        if z <= DOME_CENTER.z {
            break;
        }
        out.push(z);
    }
    out.push(DOME_CENTER.z); // 底面那一层：刀底正好落在 z=24
    out
}

/// 精加工的刀尖折线（含起点）。见模块文档的解析式。
pub fn finish_tip_path() -> Vec<DVec3> {
    use std::f64::consts::{FRAC_PI_2, TAU};
    let rb = FINISH_TIP_R;
    let rr = DOME_R + rb; // 凸面：刀心在球外 15.5
    // 刀尖不低过底面的最外一圈
    let th_max = (rb / rr).acos().min(FRAC_PI_2);
    let dtheta = FINISH_STEPOVER_MM / DOME_R;
    let n_rev = (th_max / dtheta).ceil() as usize;
    let point = |th: f64, ph: f64| {
        DVec3::new(
            DOME_CENTER.x + rr * th.sin() * ph.cos(),
            DOME_CENTER.y + rr * th.sin() * ph.sin(),
            DOME_CENTER.z + rr * th.cos(),
        ) - DVec3::new(0.0, 0.0, rb)
    };
    let mut out = vec![point(th_max, 0.0)];
    for i in 0..n_rev {
        let th0 = (th_max - i as f64 * dtheta).max(0.0);
        let th1 = (th0 - dtheta).max(0.0);
        // 这一圈拆几段：按起点（最外那一圈）的半径反解弦长。极点附近 ρ→0，
        // 用中点半径会算少，弦长会超标。
        //
        // **下限是 3，不是 1。** 要覆盖的是**整圈周长** `2πρ`，不是一条直径。
        // 早先写 `2ρ ≤ 弦长 → n = 1`（把直径当成最大弦长），结果球顶 θ<2.2°
        // 那几圈变成"一段直线跨过整圈"：φ 转 2π 但只走了一个弦，中间那一圈
        // 环带整片漏切。
        //
        // **球顶要再密一档。**
        //
        // 真正的判据不是弦长，是**折线多边形与下一圈之间的最大间隙**：
        //
        // ```text
        // gap = 行距 + ρ·(1 − cos(π/n))
        // ```
        // `ρ·(1−cos(π/n))` 是 n 边形相对外接圆的内缩量。要求 `gap < 刀半径 0.5`，
        // 否则两圈之间会刮出一条缝。
        //
        // 按弦长公式，`ρ(1−cos(π/n)) ≈ 0.18/ρ`：ρ=15.5 时只有 0.012，ρ=2 时
        // 0.09，**ρ=0.57 时 0.285**——加上行距就是 0.695，比刀半径大 39%。
        // 实测漏下的那一块就在 θ=2.18°、ρ=0.57：球冠上留下 2 个 creator=1
        // 的顶点（面铣层的色）和一块 0.09 mm 厚的残料。
        //
        // 所以只在 ρ<2 把小圈的弦长收到 0.45（ρ=0.57 → n=8，gap=0.467）。
        // 全局收弦长会把 3008 段涨到 7000 段，为球顶那 5 圈不值得。
        let rho = rr * th0.sin();
        let chord = if rho < 2.0 { 0.45 } else { FINISH_CHORD_MM };
        let n = (((TAU * rho) / chord).ceil() as usize).max(3);
        for k in 1..=n {
            let u = k as f64 / n as f64;
            out.push(point(th0 + (th1 - th0) * u, TAU * (i as f64 + u)));
        }
        if th0 <= 0.0 {
            break;
        }
    }
    out
}

/// 完整程序：T1 粗加工 → T2 精加工。
pub fn gcode() -> String {
    let (cx, cy) = (DOME_CENTER.x, DOME_CENTER.y);
    let mut s = String::new();
    s.push_str("(半球凸台 R15 / 50x50x40 · T1 粗加工 + T2 精加工)\n");
    s.push_str("G21 G90 G17 G94\n");

    // ── T1 面铣：把顶面切到球顶平面（z=39） ──
    //
    // **这一刀是必需的**，不是"顺带"。球顶若与毛坯顶面平齐，球顶周围半径
    // `√(2R·a)≈3.9` 的那层球冠没有任何刀具够得到（球头刀一靠近就切进球），
    // 成品上会留一块 6 mm 见方的毛坯面。平底刀的刀底是平的，能和球顶相切着
    // 扫过去——这是球头刀做不到的事。
    let safe_z = STOCK_MM[2] + 15.0;
    s.push_str(&format!("T1 M6 G0 Z{safe_z:.0} F800\n"));
    s.push_str(&format!(
        "G0 X{:.4} Y{:.4}\nG1 Z{:.4} F200\n",
        cx, cy, DOME_TOP
    ));
    for r in face_mill_rings() {
        s.push_str(&format!("G1 X{:.4} Y{:.4} F800\n", cx + r, cy));
        s.push_str(&format!("G2 X{:.4} Y{:.4} I{:.4} J0 F800\n", cx + r, cy, -r));
    }

    // ── T1 粗加工：Z 层切，每层从内缘往外清环带 ──
    //
    // 底层走到 36（盖住对角），其余层只走到 `rough_ring_cap()`：再往外的料
    // 底层的刀身（20 mm 刃长）一刀就清完了。
    let cap = rough_ring_cap();
    for z in rough_levels() {
        let r0 = rough_min_center_radius(z);
        let r_end = if z <= DOME_CENTER.z + 1e-9 { ROUGH_RING_MAX_MM } else { cap };
        if r0 > r_end + 1e-9 {
            continue; // 这一层要切的环带是空的（底层已经覆盖到）
        }
        s.push_str(&format!(
            "G0 Z{safe_z:.0}\nG0 X{:.4} Y{:.4}\nG1 Z{z:.4} F200\n",
            cx + r0,
            cy
        ));
        let mut r = r0;
        while r <= r_end + 1e-9 {
            // **每个整圆前必须先把刀移到该圆的起点**：`I/J` 是相对**起点**的，
            // 直接连写 `G2 I-r` 会让圆心落在"上一圈终点 − r"上（实测偏到 21 而不是 25），
            // 切出来的圆大小对、位置全错。
            s.push_str(&format!("G1 X{:.4} Y{:.4} F800\n", cx + r, cy));
            s.push_str(&format!("G2 X{:.4} Y{:.4} I{:.4} J0 F800\n", cx + r, cy, -r));
            r += ROUGH_STEPOVER_MM;
        }
    }

    // ── T2 精加工 ──
    let path = finish_tip_path();
    let first = path[0];
    s.push_str(&format!("T2 M6 G0 Z{safe_z:.0} F800\n"));
    // 4 位小数 = 0.1 µm：相切关系要能在这份文本里活下来（测试锁 2e-4）
    s.push_str(&format!("G0 X{:.4} Y{:.4} Z{:.0}\n", first.x, first.y, safe_z - 3.0));
    s.push_str(&format!("G1 Z{:.4} F300\n", first.z));
    for p in &path[1..] {
        s.push_str(&format!("G1 X{:.4} Y{:.4} Z{:.4} F600\n", p.x, p.y, p.z));
    }
    s.push_str(&format!("G0 Z{safe_z:.0}\nM30\n"));
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_gcode::{InterpConfig, Trust, interpret};
    use cncsim_motion::arc::{DEFAULT_ARC_SAGITTA, sample_arc};
    use cncsim_motion::{MotionKind, Pose};

    fn prog() -> cncsim_gcode::MotionProgram {
        interpret(
            &gcode(),
            &InterpConfig {
                tool_radii: vec![0.0, 3.0, FINISH_TIP_R],
                ..Default::default()
            },
        )
    }

    /// 把程序摊成"每一小段的目标位姿 + 刀号"，圆弧按仿真同一份弦高离散。
    /// **只取切削段**：快移不切料，也不该满足"刀在等距面外"。
    fn poses() -> Vec<(u16, Pose)> {
        let p = prog();
        let mut out = Vec::new();
        let mut prev = p.blocks.first().map(|b| b.target).unwrap_or(Pose {
            tip: DVec3::ZERO,
            axis: DVec3::Z,
        });
        for b in &p.blocks {
            match b.motion {
                MotionKind::Rapid => {}
                MotionKind::Linear => out.push((b.tool, b.target)),
                MotionKind::Arc { center, clockwise, plane } => {
                    for q in
                        sample_arc(prev.tip, b.target.tip, center, clockwise, plane, DEFAULT_ARC_SAGITTA)
                    {
                        out.push((b.tool, Pose { tip: q, axis: b.target.axis }));
                    }
                }
                MotionKind::Dwell { .. } => {}
            }
            prev = b.target;
        }
        out
    }

    #[test]
    fn program_parses_clean() {
        let p = prog();
        assert!(matches!(p.trust, Trust::Reliable), "解释器有意见：{:?}", p.issues);
        assert!(!p.blocks.is_empty());
    }

    /// **粗加工不许啃球**：每段位姿下，刀底内缘（离轴 `r_c − r_tool`、就在刀底）
    /// 到球心的距离必须 ≥ R + allow。这是整把刀离球心最近的地方。
    #[test]
    fn roughing_never_eats_the_dome() {
        let r_tool = rough_tool().cutter.max_radius();
        let mut n = 0usize;
        for (tool, pose) in poses() {
            if tool != ROUGH_TOOL_NO {
                continue;
            }
            n += 1;
            let rho_c = (pose.tip.x - DOME_CENTER.x).hypot(pose.tip.y - DOME_CENTER.y);
            // 刀底内缘那一圈里离球心最近的点：半径 |r_c − r_tool|，方位与刀心相同
            let r_in = (rho_c - r_tool).abs();
            let rim = DVec3::new(
                DOME_CENTER.x + r_in,
                DOME_CENTER.y,
                pose.tip.z,
            );
            let d = (rim - DOME_CENTER).length();
            // 面铣层（刀底正好在球顶平面）是**故意相切**的：刀底内缘落在球顶
            // 那一点上，sd=0 不算过切。其余层必须留出 ROUGH_FLOOR_ALLOW。
            let limit = if (pose.tip.z - DOME_TOP).abs() < 1e-9 {
                DOME_R - 1e-6
            } else {
                DOME_R + ROUGH_FLOOR_ALLOW_MM - 1e-3
            };
            assert!(
                d >= limit,
                "第 {n} 段 (x{:.2} y{:.2} z{:.2}) 刀底内缘离球心 {d:.4} < {limit:.4}",
                pose.tip.x,
                pose.tip.y,
                pose.tip.z
            );
        }
        assert!(n > 100, "粗加工段数太少：{n}");
    }

    /// **四个角必须削到**：方形毛坯的对角（35.4）比边（25）远 41%。同心圆只走到边，
    /// 会在四角留下一根 15 mm 高的料柱——第一版实测残留 3169 mm³ / 最厚 15.000 mm，
    /// 报的位置就是 (1.1, 1.1, 40)。
    #[test]
    fn roughing_clears_the_four_corners() {
        let r_tool = rough_tool().cutter.max_radius();
        let flute = rough_tool().flute_length;
        let poses = poses();
        for (x, y) in [(0.0, 0.0), (50.0, 0.0), (0.0, 50.0), (50.0, 50.0)] {
            let p = DVec3::new(x, y, STOCK_MM[2]); // 顶面那四个角
            let covered = poses.iter().any(|(tool, pose)| {
                *tool == ROUGH_TOOL_NO
                    && (pose.tip.x - p.x).hypot(pose.tip.y - p.y) <= r_tool + 1e-9
                    && pose.tip.z <= p.z + 1e-9
                    && pose.tip.z + flute >= p.z
            });
            if !covered {
                let mut best = f64::INFINITY;
                let mut bz = 0.0;
                for (tool, pose) in &poses {
                    if *tool != ROUGH_TOOL_NO { continue; }
                    let d = (pose.tip.x - p.x).hypot(pose.tip.y - p.y);
                    if d < best { best = d; bz = pose.tip.z; }
                }
                panic!("角落 ({x}, {y}) 没被覆盖：最近刀心距离 {best:.3}（z={bz:.2}）");
            }
        }
    }

    /// **球顶周围那圈毛坯面必须被面铣掉**。
    ///
    /// 球顶与毛坯顶面平齐时，那里会留 12.8 mm³ 的球冠谁都切不到：球头刀一靠近
    /// 就切进球，平的刀底才行。判据是"顶面上半径 5 以内的点都被某一段覆盖"，
    /// 而且覆盖那一段的刀必须是 T1（面铣），不是 T2。
    #[test]
    fn face_mill_clears_the_top_face_around_the_apex() {
        let r_tool = rough_tool().cutter.max_radius();
        let flute = rough_tool().flute_length;
        let poses = poses();
        for r in [0.5, 1.0, 2.0, 3.0, 4.0, 5.0] {
            let p = DVec3::new(DOME_CENTER.x + r, DOME_CENTER.y, STOCK_MM[2]);
            let covered = poses.iter().any(|(tool, pose)| {
                *tool == ROUGH_TOOL_NO
                    && (pose.tip.x - p.x).hypot(pose.tip.y - p.y) <= r_tool + 1e-9
                    && pose.tip.z <= p.z + 1e-9
                    && pose.tip.z + flute >= p.z
            });
            assert!(covered, "毛坯顶面半径 {r} 处没被面铣覆盖——球顶会留一圈灰的");
        }
    }

    /// 粗加工必须**把底面切出来**：最后一层刀底落在球心平面，且内缘贴到 R。
    #[test]
    fn roughing_finishes_the_floor() {
        let levels = rough_levels();
        assert_eq!(*levels.last().unwrap(), DOME_CENTER.z, "最后一层应落在底面 z=24");
        assert!(levels.windows(2).all(|w| w[0] > w[1]), "层序必须从上往下：{levels:?}");
        let r0 = rough_min_center_radius(DOME_CENTER.z);
        let inner = r0 - rough_tool().cutter.max_radius();
        assert!(
            (inner - (DOME_R + ROUGH_FLOOR_ALLOW_MM)).abs() < 1e-9,
            "底面那层的刀内缘应贴到 {:.2}，实际 {inner:.3}",
            DOME_R + ROUGH_FLOOR_ALLOW_MM
        );
    }

    /// **精加工必须相切**：刀心落在 `R + r_b` 的球面上（凸面在外侧）。
    #[test]
    fn finish_path_is_tangent_to_the_dome() {
        let rr = DOME_R + FINISH_TIP_R;
        let mut n = 0usize;
        for (tool, pose) in poses() {
            if tool != FINISH_TOOL_NO {
                continue;
            }
            n += 1;
            let b = pose.tip + DVec3::new(0.0, 0.0, FINISH_TIP_R);
            let d = (b - DOME_CENTER).length();
            assert!(
                (d - rr).abs() < 2e-4, // G 代码文本 4 位小数 → 单轴 ≤5e-5
                "第 {n} 点刀心半径 {d:.9}，应 {rr}（tip {:?}）",
                pose.tip
            );
        }
        assert!(n > 2000, "精加工段数太少：{n}");
    }

    /// 精加工从**底面**扫到**球顶**，且刀尖全程不扎进底面。
    #[test]
    fn finish_runs_from_floor_to_apex() {
        let path = finish_tip_path();
        let first = path[0];
        let last = *path.last().unwrap();
        assert!((first.z - DOME_CENTER.z).abs() < 1e-9, "起点刀尖应正好在底面上：{}", first.z);
        assert!(
            (last.z - (DOME_CENTER.z + DOME_R)).abs() < 1e-9,
            "终点应在球顶 z=39：{}",
            last.z
        );
        assert!((last.x - 25.0).powi(2) + (last.y - 25.0).powi(2) < 1e-18, "终点应在轴上");
        // 全程不得低于底面（刀尖 z ≥ 24）
        let lowest = path.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        assert!(lowest >= DOME_CENTER.z - 1e-9, "有刀尖落到球心平面以下：{lowest}");
        let mut worst = 0.0f64;
        for w in path.windows(2) {
            worst = worst.max((w[1] - w[0]).length());
        }
        assert!(worst <= FINISH_CHORD_MM + 1e-9, "最大弦长 {worst:.3} 超过 {FINISH_CHORD_MM}");
    }

    /// **覆盖度**：球面上每个点都要有一个"理想刀心"在 0.7 mm 以内。
    ///
    /// 只测**精加工负责的那一段**（θ ≤ θ_max）：再往赤道那 1.85° 刀尖就低于底面了，
    /// 归粗加工的最后一层管（留 0.02 的壳）。把那段算进来会测出一个几何上
    /// 不可能满足的数。
    /// **球面各处都不能刮出缝**：折线多边形与下一圈的最大间隙 < 刀半径。
    ///
    /// `gap = 行距 + ρ·(1 − cos(π/n))`，其中 n 是该圈的段数（从相邻两点的
    /// 角度差反推）。刀半径 0.5，所以要求 gap < 0.5，否则刀扫过这一圈和
    /// 下一圈时，两条折线之间会漏一条缝。
    ///
    /// 第一版在球顶把一整圈写成 `n = 1`（把"直径"当成最大弦长），
    /// ρ=0.57 时 gap = 0.41 + 0.285 = 0.695，球冠上漏了 2 个 creator=1
    /// 的顶点和一块 0.09 mm 厚的残料。
    #[test]
    fn finish_rings_never_leave_a_gap_wider_than_the_tool() {
        use std::f64::consts::TAU;
        let path = finish_tip_path();
        let rr = DOME_R + FINISH_TIP_R;
        let mut worst = 0.0f64;
        let mut worst_at = DVec3::ZERO;
        // 每一圈：同一 θ 的一段。按 θ 分组（相邻点的 θ 单调下降）。
        let mut i = 0;
        while i + 2 < path.len() {
            let a = path[i];
            let b = path[i + 1];
            let va = a + DVec3::new(0.0, 0.0, FINISH_TIP_R) - DOME_CENTER;
            let vb = b + DVec3::new(0.0, 0.0, FINISH_TIP_R) - DOME_CENTER;
            let rho = (va.x * va.x + va.y * va.y).sqrt();
            // 同一圈的两点：θ 几乎相同（下降量 << 一圈）
            if (va.z - vb.z).abs() < 0.4 * rr {
                // 两点夹角
                let ang = ((va.x * vb.x + va.y * vb.y) / (rho * rho).max(1e-12))
                    .clamp(-1.0, 1.0)
                    .acos();
                if ang > 1e-6 {
                    let n = TAU / ang;
                    let gap = FINISH_STEPOVER_MM + rho * (1.0 - (std::f64::consts::PI / n).cos());
                    if gap > worst {
                        worst = gap;
                        worst_at = a;
                    }
                }
            }
            i += 1;
        }
        assert!(
            worst < 0.5,
            "最大圈间间隙 {worst:.3} mm @ ({:.2},{:.2},{:.2})——应小于刀半径 0.5",
            worst_at.x,
            worst_at.y,
            worst_at.z
        );
    }

    #[test]
    fn finish_path_covers_the_dome() {
        use std::f64::consts::TAU;
        let path = finish_tip_path();
        let centers: Vec<DVec3> =
            path.iter().map(|t| *t + DVec3::new(0.0, 0.0, FINISH_TIP_R)).collect();
        let rr = DOME_R + FINISH_TIP_R;
        let th_max = (FINISH_TIP_R / rr).acos();
        let mut worst = 0.0f64;
        let (n_th, n_ph) = (24, 72);
        for i in 0..=n_th {
            let th = th_max * (i as f64 / n_th as f64);
            for j in 0..n_ph {
                let ph = TAU * (j as f64) / n_ph as f64;
                // 设计面点（上半球）→ 同一方向的理想刀心
                let p = DOME_CENTER
                    + DVec3::new(
                        DOME_R * th.sin() * ph.cos(),
                        DOME_R * th.sin() * ph.sin(),
                        DOME_R * th.cos(),
                    );
                let want = DOME_CENTER + (p - DOME_CENTER) * (rr / DOME_R);
                let d = centers.iter().map(|c| (*c - want).length()).fold(f64::INFINITY, f64::min);
                worst = worst.max(d);
            }
        }
        assert!(
            worst <= 0.7,
            "球面最差点离最近的刀位 {worst:.3} mm（应 ≤0.7；行距 0.4 / 弦长 1.2）"
        );
    }

    /// 换刀顺序：T1 全程在 T2 之前，且两把刀都真的用上了。
    #[test]
    fn both_tools_are_used_in_order() {
        let p = prog();
        let mut seen2 = false;
        let (mut n1, mut n2) = (0, 0);
        for b in &p.blocks {
            match b.tool {
                ROUGH_TOOL_NO => {
                    assert!(!seen2, "T1 出现在 T2 之后（第 {} 行）", b.line_no);
                    n1 += 1;
                }
                FINISH_TOOL_NO => {
                    seen2 = true;
                    n2 += 1;
                }
                other => panic!("出现未定义的刀号 T{other}（第 {} 行）", b.line_no),
            }
        }
        assert!(n1 > 40 && n2 > 2000, "块数不对：T1={n1} T2={n2}");
    }
}
