//! 刀径补偿（G41/G42）：把**编程路径**按 ±r 偏移成**刀心路径**。
//!
//! ## 为什么必须单独一遍
//!
//! 拐角的补偿点取决于**前后两段的方向**：本段偏移线的终点要和下一段偏移线
//! 求交（内角），或者绕顶点转一段圆弧（外角）。逐行生成时只看得见本段，
//! 所以补偿放在整份程序解析完之后，当作一次后处理。
//!
//! ## 三种顶点
//!
//! ```text
//! 共线（切向相同）     内角（偏移侧在转弯内侧）    外角（偏移侧在外侧）
//!      ──────              ┐                          ┐
//!        ↦ 直接过           └── 两条偏移线求交          └─ 绕顶点走一段 r 弧
//! ```
//!
//! **外角必须是圆弧，不能求交**：两条偏移线在外侧的交点比顶点远 `r·tan(θ/2)`，
//! 90° 拐角就多切 0.41r。机床就是这么走的——刀绕顶点滚过去。
//!
//! ## 边界条件（做不到就报，不猜）
//!
//! - **第一块必须是直线**（引入段）。圆弧引入要新建半径，控制器也是这么要求的。
//! - 补偿组之后那块也必须是直线（引出段），否则引出弧的起点半径不对。
//! - 只支持 G17，只支持 G1/G2/G3。
//!
//! 任何一条不满足就把整组**还原成编程路径**并报 `GCODE-005`——宁可说"没补偿"，
//! 也不要交出一条差一个刀半径的路径。

use crate::{Block, CutterComp, ParseIssue, Severity};
use cncsim_math::DVec3;
use cncsim_motion::{MotionKind, Plane};

/// 切向叉积小于它就算共线（相邻两段方向相同）。
const COLLINEAR_EPS: f64 = 1e-9;
/// 内角求交的钳制倍数：交点离顶点超过 `MITER_LIMIT × r` 就退回法向偏移点。
///
/// 只在近乎 180° 折返时触发。那种角上刀具根本进不去，几何本身已经退化，
/// 但也不能让一个 1e6 mm 外的点把整条路径拉飞。
const MITER_LIMIT: f64 = 10.0;

/// 对整份程序做刀径补偿。没有 `G41/G42` 时原样返回。
pub(crate) fn apply(blocks: &mut Vec<Block>, initial: DVec3, issues: &mut Vec<ParseIssue>) {
    let mut i = 0;
    while i < blocks.len() {
        let side = blocks[i].cutter_comp;
        if side == CutterComp::Off {
            i += 1;
            continue;
        }
        let radius = blocks[i].comp_radius;
        // 一组 = 同一侧、同一半径的连续块。侧别/半径一变就重新起一组
        // （中间没写 G40 也认，那是程序写得随意，不是我们该纠正的）
        let mut j = i + 1;
        while j < blocks.len()
            && blocks[j].cutter_comp == side
            && (blocks[j].comp_radius - radius).abs() < 1e-9
        {
            j += 1;
        }

        if let Err(why) = check_group(blocks, i, j) {
            issues.push(ParseIssue::new(
                "GCODE-005",
                Severity::Error,
                blocks[i].line_no,
                format!("刀径补偿未生效：{why}"),
                "补偿需要直线引入/引出；把 G41/G42 那一行单独放，并确保引入段是 G1",
            ));
            i = j;
            continue;
        }

        let start = if i == 0 {
            initial
        } else {
            blocks[i - 1].target.tip
        };
        let mut replacement = offset_group(&blocks[i..j], start, side, radius, issues);
        let new_len = replacement.len();
        blocks.splice(i..j, replacement.drain(..));
        i += new_len;
    }
}

/// 这一组能不能补偿。
fn check_group(blocks: &[Block], i: usize, j: usize) -> Result<(), String> {
    if blocks[i].comp_radius <= 1e-9 {
        return Err("补偿半径为 0".into());
    }
    if !matches!(blocks[i].motion, MotionKind::Linear) {
        return Err("第一块必须是直线（引入段）".into());
    }
    for b in &blocks[i..j] {
        match b.motion {
            MotionKind::Linear => {}
            MotionKind::Arc { plane: Plane::Xy, .. } => {}
            MotionKind::Arc { plane, .. } => {
                return Err(format!("圆弧必须在 G17 平面内，当前是 {plane:?}"));
            }
            _ => return Err("补偿组里只能是 G1/G2/G3".into()),
        }
    }
    if j < blocks.len() && !matches!(blocks[j].motion, MotionKind::Linear) {
        return Err("补偿组之后必须是直线（引出段）".into());
    }
    Ok(())
}

/// 把一组块偏移成刀心路径，外角插入拐角圆弧。
fn offset_group(
    group: &[Block],
    start: DVec3,
    side: CutterComp,
    radius: f64,
    issues: &mut Vec<ParseIssue>,
) -> Vec<Block> {
    let sign = if side == CutterComp::Left { 1.0 } else { -1.0 };
    let n = group.len();

    // 每个点的坐标 + 每段末端的切向
    let mut pts = Vec::with_capacity(n + 1);
    pts.push(start);
    for b in group {
        pts.push(b.target.tip);
    }
    let mut tans = Vec::with_capacity(n);
    for (k, b) in group.iter().enumerate() {
        tans.push(end_tangent(b, pts[k]));
    }

    let mut out: Vec<Block> = Vec::with_capacity(n + 2);
    for (k, block) in group.iter().enumerate() {
        let p = pts[k + 1];
        let t_in = tans[k];
        let a = p + left_perp(t_in) * (radius * sign);

        let mut nb = block.clone();
        if k + 1 == n {
            // 最后一段：法向偏移就是终点，引出段自己会压回编程路径
            nb.target.tip = a;
            out.push(nb);
            continue;
        }

        // 拐角的出向必须是**下一段起点**的切向。下一段是圆弧时，它的
        // 起点切向和终点切向不同——用终点切向会把拐角算到圆弧的另一头去。
        let t_out = start_tangent(&group[k + 1], pts[k + 1]);
        let cross = t_in.x * t_out.y - t_in.y * t_out.x;
        if cross.abs() < COLLINEAR_EPS {
            nb.target.tip = a;
            out.push(nb);
            continue;
        }

        let outside = cross * sign < 0.0;
        if outside {
            // 外角：绕顶点滚一段 r 弧
            nb.target.tip = a;
            let b = p + left_perp(t_out) * (radius * sign);
            if (b - a).length() > 1e-9 {
                let mut arc = block.clone();
                arc.motion = MotionKind::Arc {
                    center: p,
                    clockwise: cross < 0.0,
                    plane: Plane::Xy,
                };
                arc.target.tip = b;
                out.push(nb);
                out.push(arc);
                continue;
            }
            out.push(nb);
            continue;
        }

        // 内角：两条偏移线求交
        let b = p + left_perp(t_out) * (radius * sign);
        let q = intersect(a, t_in, b, t_out).unwrap_or(a);
        if (q - p).length() > MITER_LIMIT * radius {
            issues.push(ParseIssue::new(
                "GCODE-012",
                Severity::Warning,
                block.line_no,
                format!(
                    "刀径补偿在尖角处退化（交点离顶点 {:.1} mm，超过 {}×r）",
                    (q - p).length(),
                    MITER_LIMIT
                ),
                "刀具几何上进不去这么尖的角；已按法向偏移处理，检查该处是否过切",
            ));
            nb.target.tip = a;
        } else {
            nb.target.tip = q;
        }
        out.push(nb);
    }
    out
}

/// 段末的切向（单位，XY 平面内）。
///
/// 直线就是两端连线方向；圆弧取该点处的切线（半径逆时针转 90°）。
fn end_tangent(b: &Block, from: DVec3) -> DVec3 {
    if let MotionKind::Arc {
        center, clockwise, ..
    } = b.motion
    {
        let r = xyz0(b.target.tip - center);
        let t = left_perp(r);
        let t = if clockwise { -t } else { t };
        let l = t.length();
        if l > 1e-12 {
            return t / l;
        }
    }
    let d = xyz0(b.target.tip - from);
    let l = d.length();
    if l > 1e-12 { d / l } else { DVec3::X }
}

/// 段首的切向（单位，XY 平面内）。与 [`end_tangent`] 同一套规则，
/// 只是圆弧取起点处的半径。
fn start_tangent(b: &Block, from: DVec3) -> DVec3 {
    if let MotionKind::Arc {
        center, clockwise, ..
    } = b.motion
    {
        let r = xyz0(from - center);
        let t = left_perp(r);
        let t = if clockwise { -t } else { t };
        let l = t.length();
        if l > 1e-12 {
            return t / l;
        }
    }
    let d = xyz0(b.target.tip - from);
    let l = d.length();
    if l > 1e-12 { d / l } else { DVec3::X }
}

/// 行进方向的左法向。
fn left_perp(t: DVec3) -> DVec3 {
    DVec3::new(-t.y, t.x, 0.0)
}

/// 抹掉 Z 分量（补偿只在 XY 平面内做）。
fn xyz0(v: DVec3) -> DVec3 {
    DVec3::new(v.x, v.y, 0.0)
}

/// 两条直线求交：过 `a` 方向 `ta`，过 `b` 方向 `tb`。平行返回 `None`。
fn intersect(a: DVec3, ta: DVec3, b: DVec3, tb: DVec3) -> Option<DVec3> {
    let denom = ta.x * tb.y - ta.y * tb.x;
    if denom.abs() < 1e-12 {
        return None;
    }
    let d = xyz0(b - a);
    let s = (d.x * tb.y - d.y * tb.x) / denom;
    Some(a + ta * s)
}
