//! **多工序计划**：同一件毛坯，从多个方向、多把刀，反复加工。
//!
//! 现有能力只有两条单方向链路（3 轴沿 +Z、绕 Z 回转）。这里把"方向"变成参数：
//! 每道工序 `Pass` 指定一个进刀方向 + 自己的粗/精刀，内部用
//! [`Frame::from_dir`] 把模型转进该方向，跑同一条 3 轴链路，再把点变回世界。
//!
//! 输出是**一份连续 G 代码**：所有工序串起来，刀号全局唯一（第 i 道用
//! `2i+1`/`2i+2`），中间只有换刀和安全高度回退，没有 `M30`（否则控制器会在
//! 第一道结束时停机回卷）。
//!
//! 局限：每道仍要求该方向上"单值上表面"（3 轴高度场的前提）。多方向能覆盖
//! 方件/支架这类各个面，但真正的倒扣（内腔顶面）还得靠五轴或翻面装夹。

use crate::{CamError, Frame, Job, Process, Program, Stats, Stock, ToolDef, ToolSpec, Tri};
use cncsim_math::DVec3;

/// 一道工序。
#[derive(Clone, Debug)]
pub struct Pass {
    /// 该工序的 **+Z**：也是**刀轴指向**（从零件指向刀具）。`+Z` = 传统立式。
    ///
    /// 注意它不是"进给方向"：从 −X 面开粗时刀轴指向 `−X`（刀从 −X 侧伸过来），
    /// 刀尖沿 `+X` 走刀。侧向工序必须真的把刀轴摆过去（`G43.4` + A/C），否则
    /// 刀杆会沿世界 +Z 垂直插进工件——见 [`generate_plan`] 里的说明。
    pub dir: DVec3,
    /// 该方向的粗加工刀；`None` = 只精加工。
    pub rough: Option<ToolSpec>,
    pub finish: ToolSpec,
    /// **清根目标点**（世界坐标）：`Some` 时这一道只在这些点附近走刀。
    /// 用于"凹角小刀清根"——上一道在大刀留下的残料圈里补一遍。
    pub rest: Option<Vec<DVec3>>,
    /// 这一道用自己的工艺参数（`None` = 用计划级的那套）。
    /// 清根要换更小的行距，但**不能**动其它道的参数。
    pub process: Option<Process>,
}

#[derive(Clone, Debug)]
pub struct PlanOutput {
    pub program: Program,
    /// 全部刀具（刀号已重排，按工序顺序）。
    pub tools: Vec<ToolDef>,
    /// 实际生成的工序数。
    pub passes: usize,
    /// 被**跳过**的工序：`(下标, 原因)`。
    ///
    /// 单道做不了（该方向多层/倒扣、网格为空、模型出界）不该拖垮整份计划——
    /// "无法加工的部分尽最大努力，完全加工不了的跳过"。跳过的原因如实报出来，
    /// 不吞掉。
    pub skipped: Vec<(usize, String)>,
}

/// 把世界坐标的包围盒变换到局部坐标系（取 8 个角的包围盒）。
fn local_stock(s: Stock, f: &Frame) -> Stock {
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for i in 0..8 {
        let p = DVec3::new(
            if i & 1 == 0 { s.min.x } else { s.max.x },
            if i & 2 == 0 { s.min.y } else { s.max.y },
            if i & 4 == 0 { s.min.z } else { s.max.z },
        );
        let q = f.unapply(p);
        lo = DVec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
        hi = DVec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
    }
    Stock { min: lo, max: hi }
}

/// 去掉一段程序的结束指令（`M5` / `M30`），只保留运动与换刀。
fn strip_end(gcode: &str) -> String {
    let mut out = String::with_capacity(gcode.len());
    for line in gcode.lines() {
        let t = line.trim();
        if t == "M5" || t == "M30" {
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// 生成多工序程序。
pub fn generate_plan(
    mesh: &[Tri],
    stock: Stock,
    process: &Process,
    passes: &[Pass],
) -> Result<PlanOutput, CamError> {
    if passes.is_empty() {
        return Err(CamError::BadParam("工序计划为空"));
    }
    let safe = if process.safe_z_mm.is_nan() {
        stock.max.z + 5.0
    } else {
        process.safe_z_mm
    };
    // 工件坐标系：所有坐标都在 G54 里。G53 只用于最后的机床坐标回退。
    let mut gcode = String::with_capacity(1 << 18);
    gcode.push_str("(WORK OFFSET G54)\n");
    gcode.push_str("G54\n");
    let mut tools: Vec<ToolDef> = Vec::new();
    let mut stats = Stats::default();
    let mut done = 0usize;
    let mut skipped: Vec<(usize, String)> = Vec::new();

    for (i, pass) in passes.iter().enumerate() {
        let f = Frame::from_dir(pass.dir);
        let local_mesh: Vec<Tri> = mesh
            .iter()
            .map(|t| [f.unapply(t[0]), f.unapply(t[1]), f.unapply(t[2])])
            .collect();
        let lstock = local_stock(stock, &f);

        // 刀号全局唯一：第 i 道 = 2i+1（粗）/ 2i+2（精）
        let mut rough = pass.rough.clone();
        if let Some(t) = &mut rough {
            t.number = (2 * i + 1) as u16;
        }
        let mut finish = pass.finish.clone();
        finish.number = (2 * i + 2) as u16;

        let job = Job {
            mesh: &local_mesh,
            stock: lstock,
            rough: rough.clone(),
            finish: finish.clone(),
            process: pass.process.clone().unwrap_or_else(|| process.clone()),
            rest: pass.rest.as_deref(),
        };
        // 单道失败就跳过，不让整份计划作废
        let prog = match crate::generate_in(&job, Some(f)) {
            Ok(p) => p,
            Err(e) => {
                skipped.push((i, e.to_string()));
                continue;
            }
        };

        gcode.push_str(&format!(
            "(PASS {} dir {:.2},{:.2},{:.2})\n",
            i + 1,
            pass.dir.x,
            pass.dir.y,
            pass.dir.z
        ));
        // 上一道可能停在任意姿态：先取消 RTCP、竖直抬到世界安全高度，再摆新姿态。
        // 这里必须用 `G0 Z` 的世界高度（而不是本道坐标系里的 safe）：换姿态是在
        // 机床坐标系里发生的。
        gcode.push_str("G49\n");
        gcode.push_str(&format!("G0 Z{:.3}\n", safe));

        // **侧向工序必须真的摆刀轴**。
        //
        // 只把坐标转进局部系、输出的仍是纯 XYZ 三轴程序是错的：控制器会把刀轴
        // 当成世界 +Z，于是刀杆（半径 = 刀半径、长度 = 刃长）从上方垂直插进工件。
        // 实测 `±X/±Y` 每道稳定过切 105 mm³，过切带正好 1.5 mm 深 = Ø3 球头半径。
        // 这里用 `G43.4 H1`（刀尖跟随）+ A/C 把刀轴摆到 `pass.dir`，程序点仍是刀尖。
        let (a, c) = crate::rotary::ac_of(pass.dir.normalize());
        gcode.push_str("G43.4 H1\n");
        gcode.push_str(&format!("G0 A{a:.3} C{c:.3}\n"));
        gcode.push_str(&strip_end(&prog.gcode));
        stats.shank_skips += prog.stats.shank_skips;
        stats.ball_skips += prog.stats.ball_skips;
        stats.levels += prog.stats.levels;
        stats.rough_moves += prog.stats.rough_moves;
        stats.finish_moves += prog.stats.finish_moves;
        stats.cut_mm += prog.stats.cut_mm;
        stats.rapid_mm += prog.stats.rapid_mm;
        if let Some(t) = &rough {
            tools.push(t.to_tool_def());
        }
        tools.push(finish.to_tool_def());
        done += 1;
    }

    if done == 0 {
        return Err(CamError::NoSurface);
    }
    gcode.push_str("G49\n");
    // 回退到**机床坐标**再停机：工件坐标系在换装夹/换工序后会变，回参考点必须用 G53。
    // 数值按「G54 的 Z 偏置 = 0」给出；机床偏置不同时按机床改。
    gcode.push_str("(G53: 机床坐标回退，按 G54 Z 偏置 0 给出)\n");
    gcode.push_str(&format!("G53 Z{:.3}\n", stock.max.z + 30.0));
    gcode.push_str("M5\nM30\n");
    stats.lines = gcode.lines().count();
    Ok(PlanOutput {
        program: Program { gcode, stats },
        tools,
        passes: done,
        skipped,
    })
}

/// 默认方向集：+Z 先做（最常用），再四个侧面。
///
/// 不含 −Z：零件通常坐在毛坯底面上，底面就是零件底面，不需要从下往上开粗。
/// 要全六面（翻面装夹）自己往 `Pass` 里加 `(0,0,-1)` 即可。
pub fn default_directions() -> Vec<DVec3> {
    vec![
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(-1.0, 0.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, -1.0, 0.0),
        DVec3::new(0.0, 1.0, 0.0),
    ]
}

/// 斜向精加工方向：从 +Z 倾斜 `tilt_deg`，方位角均匀铺 `n` 个。
///
/// 轴向工序（±X/±Y/±Z）用 45° 坡度门槛时，覆盖的是"法向与某根轴夹角 ≤45°"的面；
/// 剩下够不到的是**体对角线**附近那几片。取 `tilt_deg = 54.7356°`（体对角线倾角）
/// 加 4 个方位，正好把这几个洞补上——这些道**只精加工不粗加工**，粗料已经被
/// 轴向工序去掉了。
pub fn oblique_directions(tilt_deg: f64, n: usize) -> Vec<DVec3> {
    let n = n.max(1);
    let t = tilt_deg.to_radians();
    let (st, ct) = t.sin_cos();
    (0..n)
        .map(|k| {
            let phi = (k as f64 + 0.5) / n as f64 * std::f64::consts::TAU;
            DVec3::new(st * phi.cos(), st * phi.sin(), ct).normalize()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 4 条体对角线：倾角 54.74°、方位 45/135/225/315°，两两对称。
    #[test]
    fn oblique_directions_are_the_body_diagonals() {
        let d = oblique_directions(54.735_610_229_728_65, 4);
        assert_eq!(d.len(), 4);
        for v in &d {
            assert!((v.length() - 1.0).abs() < 1e-12);
            assert!((v.z - 0.577_350_269_189_626).abs() < 1e-6, "{v:?}");
        }
        // 两两不重合，且 xy 覆盖四个象限
        let mut quads = [0usize; 4];
        for v in &d {
            let k = (v.x > 0.0) as usize | (((v.y > 0.0) as usize) << 1);
            quads[k] += 1;
        }
        assert_eq!(quads, [1, 1, 1, 1], "四个象限各一条：{quads:?}");
    }
}
