//! # cncsim-cam
//!
//! **三角网格 → 3 轴刀路 → G 代码**。刻意只做最简单、可验证的一条链：
//!
//! ```text
//! STL/网格 ──► 高度场 z(x,y), n(x,y) ──► 粗加工（平底刀，Z 层 + 行切）
//!                                      └► 精加工（球头刀，平行行切，法向偏置）
//!                                      └► G 代码（G0/G1/G2）
//! ```
//!
//! ## 输入是网格，不是 B-rep
//!
//! `.stl` 是三角网格，直接进；`.stp/.step` 是 B-rep（边界表示），要先三角化。
//! 本 crate 不解析 STEP——那是 OpenCascade 量级的工程，且会把零依赖策略打破。
//! 文件加载放在 `cncsim-app`（输入适配器），本 crate 只认 `[DVec3; 3]` 网格。
//!
//! ## 工艺假设（不满足就会切错，所以写成显式约束）
//!
//! - **3 轴、+Z 可加工**：每个 XY 列只有一片材料，没有倒扣/内腔。
//! - 毛坯是轴对齐盒；零件网格必须在毛坯内。
//! - 高度场覆盖毛坯整个 XY；**没有命中的列视为"不是零件"**，不加工（保留毛坯）。
//! - 球头精加工按面法向偏置；坡度 > `max_finish_slope_deg` 的列跳过，
//!   留给粗加工台阶（报告里的残留会如实显示）。
//!
//! ## 为什么先做高度场，而不是 Z 层轮廓
//!
//! 轮廓要切网格、串环、做多边形偏置（带自交清理），是另一个量级的代码；
//! 高度场一次射线求交就能同时得到粗加工的"可下刀区"和精加工的法向。
//! 代价是垂直壁只能靠粗加工台阶近似——对"简单 CAM"是合算的交换。
//!
//! ## 和仿真器的关系
//!
//! 这里只**生成文本**，不认识解释器/仿真器。验收在 `tests/e2e.rs`：
//! 生成 → `cncsim_gcode::interpret` → `cncsim_sim::run` → 设计比对，
//! 断言不过切、残留有界。生成器与验证器不互相依赖，避免"自己验自己"。

pub mod rotary;
use cncsim_math::{DVec2, DVec3};
use cncsim_tool::{
    CutterKind, Profile, ToolDef, ball_end_mill, flat_end_mill, tapered_ball_end_mill,
};

/// 一条三角形（与 `cncsim_voxel::stl::Tri` 同构；本 crate 不依赖 voxel）。
pub type Tri = [DVec3; 3];

/// 轴对齐毛坯。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stock {
    pub min: DVec3,
    pub max: DVec3,
}

impl Stock {
    /// 原点在 `(0,0,0)` 的毛坯。
    pub fn from_size(size: DVec3) -> Self {
        Self {
            min: DVec3::ZERO,
            max: size,
        }
    }

    pub fn size(&self) -> DVec3 {
        self.max - self.min
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolKind {
    Flat,
    Ball,
    /// 锥度球头：小尖球 + 锥颈（模具/石墨/首饰精加工常用）。
    /// 尖部半径决定**能进多窄的谷**，锥颈决定刚性和让位。
    TaperedBall,
}

/// 刀具规格。CAM 只关心半径/刀型/刃长；几何仍由 `cncsim-tool` 的母线定义，
/// 保证"生成用的刀"和"仿真碰撞用的刀"是同一把。
#[derive(Clone, Debug, PartialEq)]
pub struct ToolSpec {
    pub number: u16,
    pub kind: ToolKind,
    /// 平头/球头是名义直径；锥度球头是**尖部**直径。
    pub diameter: f64,
    pub flute_mm: f64,
    /// 锥度球头的半角（度）。其它刀型忽略。
    pub taper_half_deg: f64,
}

impl ToolSpec {
    pub fn flat(number: u16, diameter: f64, flute_mm: f64) -> Self {
        Self {
            number,
            kind: ToolKind::Flat,
            diameter,
            flute_mm,
            taper_half_deg: 0.0,
        }
    }

    pub fn ball(number: u16, diameter: f64, flute_mm: f64) -> Self {
        Self {
            number,
            kind: ToolKind::Ball,
            diameter,
            flute_mm,
            taper_half_deg: 0.0,
        }
    }

    /// 锥度球头：尖球半径 + 锥颈（`half_angle_deg` 为半角）。
    ///
    /// 尖部小 → 能进窄谷；锥颈粗 → 刚性好、还能让开对面的壁。
    pub fn tapered(number: u16, tip_diameter: f64, half_angle_deg: f64, flute_mm: f64) -> Self {
        Self {
            number,
            kind: ToolKind::TaperedBall,
            diameter: tip_diameter,
            flute_mm,
            taper_half_deg: half_angle_deg.max(0.0),
        }
    }

    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }

    /// 切削部分的最大半径（锥度刀要按锥颈算，否则刀杆会比刀还细）。
    pub fn top_radius(&self) -> f64 {
        let r = self.radius();
        if self.kind != ToolKind::TaperedBall {
            return r;
        }
        let a = self.taper_half_deg.to_radians();
        let z_t = r * (1.0 - a.sin());
        (r * a.cos() + (self.flute_mm - z_t).max(0.0) * a.tan()).max(r)
    }

    /// 给 `cncsim-sim` 用的母线。
    pub fn profile(&self) -> Profile {
        match self.kind {
            ToolKind::Flat => flat_end_mill(self.diameter, self.flute_mm),
            ToolKind::Ball => ball_end_mill(self.diameter, self.flute_mm, 24),
            ToolKind::TaperedBall => {
                tapered_ball_end_mill(self.radius(), self.taper_half_deg, self.flute_mm, 24)
            }
        }
    }

    /// 完整刀定义（含刀柄）。生成与仿真用**同一个** `ToolDef`，避免
    /// "CAM 按 φ6 算的刀路、仿真按 φ5 查的碰撞"。
    pub fn to_tool_def(&self) -> ToolDef {
        let r = self.radius();
        let r_shank = self.top_radius();
        let top = self.flute_mm + 20.0;
        ToolDef {
            number: self.number,
            kind: match self.kind {
                ToolKind::Flat => CutterKind::Flat,
                ToolKind::Ball => CutterKind::Ball,
                ToolKind::TaperedBall => CutterKind::TaperedBall,
            },
            diameter: 2.0 * r_shank,
            corner_radius: if self.kind == ToolKind::Flat { 0.0 } else { r },
            flute_length: self.flute_mm,
            // 刀杆按**切削段最大半径**走：锥度刀用尖端半径会得到"刀杆比刀还细"的怪形状
            shank: Profile::new(vec![
                DVec2::new(0.0, self.flute_mm),
                DVec2::new(r_shank, self.flute_mm),
                DVec2::new(r_shank, top),
                DVec2::new(0.0, top),
            ]),
            length_offset: top,
            cutter: self.profile(),
        }
    }
}

/// 工艺参数。默认值是"φ6 刀、铝件"的量级，改之前先看单位注释。
#[derive(Clone, Debug)]
pub struct Process {
    /// 粗加工在零件表面以上留的余量（mm）。球头精加工会把它削掉。
    pub stock_allowance_mm: f64,
    /// 粗加工 Z 层降（mm）。
    pub stepdown_mm: f64,
    /// 粗加工行距（mm，刀心间距）。
    pub rough_stepover_mm: f64,
    /// 精加工行距（mm，球头刀心间距）。球头残高 ≈ s²/(8R)。
    pub finish_stepover_mm: f64,
    /// 高度场采样间距（mm）。越小越贴合网格，代价是内存/射线数。
    pub sample_mm: f64,
    /// 精加工允许的最大坡度（度）。超过就跳过，交给粗加工台阶。
    pub max_finish_slope_deg: f64,
    /// 安全高度（mm，绝对 Z）。必须高于毛坯顶面。
    pub safe_z_mm: f64,
    /// 切削进给（mm/min）。
    pub feed_mm_min: f64,
    /// 下刀进给（mm/min）。
    pub plunge_mm_min: f64,
    /// 主轴转速（rpm）。
    pub spindle_rpm: u32,
}

impl Default for Process {
    fn default() -> Self {
        Self {
            stock_allowance_mm: 0.2,
            stepdown_mm: 1.0,
            rough_stepover_mm: 3.6,
            finish_stepover_mm: 0.4,
            sample_mm: 0.25,
            max_finish_slope_deg: 75.0,
            safe_z_mm: f64::NAN, // NaN = 毛坯顶 + 5
            feed_mm_min: 800.0,
            plunge_mm_min: 200.0,
            spindle_rpm: 8000,
        }
    }
}

/// 一次 CAM 作业。
pub struct Job<'a> {
    pub mesh: &'a [Tri],
    pub stock: Stock,
    /// `None` = 毛坯已经等于零件，只精加工。
    pub rough: Option<ToolSpec>,
    pub finish: ToolSpec,
    pub process: Process,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stats {
    /// 高度场列数。
    pub samples: usize,
    /// 粗加工层数。
    pub levels: usize,
    /// 粗加工切削移动数。
    pub rough_moves: usize,
    /// 精加工切削移动数。
    pub finish_moves: usize,
    /// 切削行程（mm）。
    pub cut_mm: f64,
    /// 快移行程（mm）。
    pub rapid_mm: f64,
    /// G 代码行数。
    pub lines: usize,
}

#[derive(Clone, Debug)]
pub struct Program {
    pub gcode: String,
    pub stats: Stats,
}

#[derive(Clone, Debug, PartialEq)]
pub enum CamError {
    EmptyMesh,
    BadStock,
    /// 零件超出毛坯（某轴）。
    OutsideStock { axis: usize, part: f64, stock_lo: f64, stock_hi: f64 },
    BadTool(&'static str),
    BadParam(&'static str),
    /// 高度场一个有效列都没有。
    NoSurface,
    /// 模型有多层表面（中空 / 倒扣），3 轴高度场加工不了。
    Undercut { columns: usize, total: usize },
    /// STEP 是 B-rep：本组件只吃三角网格。
    StepNeedsTessellation,
}

impl std::fmt::Display for CamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CamError::EmptyMesh => write!(f, "网格为空"),
            CamError::BadStock => write!(f, "毛坯尺寸必须为正"),
            CamError::OutsideStock {
                axis,
                part,
                stock_lo,
                stock_hi,
            } => write!(
                f,
                "零件超出毛坯：{} = {part:.3} 不在 [{stock_lo:.3}, {stock_hi:.3}]",
                ["X", "Y", "Z"][*axis]
            ),
            CamError::BadTool(m) => write!(f, "刀具参数非法：{m}"),
            CamError::BadParam(m) => write!(f, "工艺参数非法：{m}"),
            CamError::NoSurface => write!(
                f,
                "高度场没有命中任何三角形——网格可能不在毛坯 XY 范围内"
            ),
            CamError::Undercut { columns, total } => write!(
                f,
                "模型有 {columns}/{total} 列存在多层表面（中空或倒扣）；3 轴高度场 CAM 只能加工每列单一上表面的零件，需要 4/5 轴或先做实体简化"
            ),
            CamError::StepNeedsTessellation => write!(
                f,
                "STEP 是 B-rep，不能直接生成刀路；请先三角化为 STL（FreeCAD: 另存为 STL）"
            ),
        }
    }
}

impl std::error::Error for CamError {}

/// 生成刀路与 G 代码。
pub fn generate(job: &Job<'_>) -> Result<Program, CamError> {
    validate(job)?;
    let p = &job.process;
    let map = HeightMap::build(job.mesh, job.stock, p.sample_mm);

    let part_min = map
        .z
        .iter()
        .copied()
        .filter(|z| z.is_finite())
        .fold(f64::INFINITY, f64::min);
    if !part_min.is_finite() {
        return Err(CamError::NoSurface);
    }
    let part_max = map
        .z
        .iter()
        .copied()
        .filter(|z| z.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);

    // **中空/倒扣检测**：封闭实体的每列通常只命中顶、底两次；>2 说明
    // 中间还有一层看不见的面（薄壁内腔、鼓肚下面的倒扣）。这种件 3 轴
    // 高度场加工不了——继续生成只会把外面那层"影子"切出来。
    // 阈值 2%：允许少量重复三角形/小孔造成的偶发 3 次命中。
    let total = map.z.iter().filter(|z| z.is_finite()).count();
    let under = map.layers.iter().filter(|&&h| h > 2).count();
    if total > 0 && under * 50 > total {
        return Err(CamError::Undercut {
            columns: under,
            total,
        });
    }

    let safe = if p.safe_z_mm.is_nan() {
        job.stock.max.z + 5.0
    } else {
        p.safe_z_mm
    };

    let mut e = Emit::new();
    e.raw(&format!(
        "(CAM 3-axis height-map  stock {:.3}x{:.3}x{:.3})",
        job.stock.size().x,
        job.stock.size().y,
        job.stock.size().z
    ));
    e.raw(&format!(
        "(part z {part_min:.3}..{part_max:.3}  samples {} x {} @ {:.3} mm)",
        map.nx, map.ny, p.sample_mm
    ));
    e.raw("G21 G90 G17 G94");
    e.rapid_z(safe);

    let mut stats = Stats {
        samples: map.nx * map.ny,
        ..Default::default()
    };

    // ── 粗加工 ──
    if let Some(rough) = &job.rough {
        let stop = part_min + p.stock_allowance_mm;
        if job.stock.max.z > stop + 1e-9 {
            let levels = descend(job.stock.max.z, stop, p.stepdown_mm);
            stats.levels = levels.len();
            e.raw(&format!(
                "(ROUGH T{} D{:.1} FLAT  {} levels, stepdown {:.3})",
                rough.number, rough.diameter, levels.len(), p.stepdown_mm
            ));
            e.tool_change(rough, p.spindle_rpm);
            let r = rough.radius();
            let step = p.rough_stepover_mm.min(r * 1.9);
            for &z in &levels {
                e.raw(&format!("(level z={z:.3})"));
                rough_level(&mut e, &map, job.stock, z, r, p.stock_allowance_mm, step, safe, p, &mut stats);
            }
            e.rapid_z(safe);
        }
    }

    // ── 精加工 ──
    e.raw(&format!(
        "(FINISH T{} D{:.1} BALL  stepover {:.3})",
        job.finish.number, job.finish.diameter, p.finish_stepover_mm
    ));
    let finish_radius = job.finish.radius();
    e.tool_change(&job.finish, p.spindle_rpm);
    finish_pass(
        &mut e,
        &map,
        job.stock,
        finish_radius,
        p,
        safe,
        part_min,
        &mut stats,
    );
    e.rapid_z(safe);
    e.raw("M5");
    e.raw("M30");

    stats.lines = e.s.lines().count();
    stats.cut_mm = e.cut_mm;
    stats.rapid_mm = e.rapid_mm;
    Ok(Program {
        gcode: e.s,
        stats,
    })
}

fn validate(job: &Job<'_>) -> Result<(), CamError> {
    if job.mesh.is_empty() {
        return Err(CamError::EmptyMesh);
    }
    let s = job.stock.size();
    if !(s.x > 0.0 && s.y > 0.0 && s.z > 0.0) {
        return Err(CamError::BadStock);
    }
    for (axis, (lo, hi)) in [
        (job.stock.min.x, job.stock.max.x),
        (job.stock.min.y, job.stock.max.y),
        (job.stock.min.z, job.stock.max.z),
    ]
    .into_iter()
    .enumerate()
    {
        let (mut a, mut b) = (f64::INFINITY, f64::NEG_INFINITY);
        for t in job.mesh {
            for v in t {
                a = a.min(v.component(axis));
                b = b.max(v.component(axis));
            }
        }
        if a < lo - 1e-6 || b > hi + 1e-6 {
            return Err(CamError::OutsideStock {
                axis,
                part: if a < lo - 1e-6 { a } else { b },
                stock_lo: lo,
                stock_hi: hi,
            });
        }
    }
    let p = &job.process;
    if p.stepdown_mm <= 0.0 || p.sample_mm <= 0.0 {
        return Err(CamError::BadParam("stepdown/sample 必须为正"));
    }
    if p.finish_stepover_mm <= 0.0 {
        return Err(CamError::BadParam("finish_stepover 必须为正"));
    }
    if !(p.max_finish_slope_deg > 0.0 && p.max_finish_slope_deg < 89.0) {
        return Err(CamError::BadParam("max_finish_slope_deg 必须在 (0, 89)"));
    }
    if !p.safe_z_mm.is_nan() && p.safe_z_mm <= job.stock.max.z {
        return Err(CamError::BadParam("safe_z 必须高于毛坯顶面"));
    }
    let check_tool = |t: &ToolSpec, name: &'static str| -> Result<(), CamError> {
        if t.diameter <= 0.0 {
            return Err(CamError::BadTool(name));
        }
        if t.flute_mm <= 0.0 {
            return Err(CamError::BadTool("刃长必须为正"));
        }
        Ok(())
    };
    check_tool(&job.finish, "精加工刀")?;
    if job.finish.kind != ToolKind::Ball {
        return Err(CamError::BadTool("精加工必须是球头刀（高度场按法向偏置）"));
    }
    if p.finish_stepover_mm > job.finish.radius() {
        return Err(CamError::BadParam(
            "finish_stepover 超过球头半径（残高会大于半径）",
        ));
    }
    if let Some(t) = &job.rough {
        check_tool(t, "粗加工刀")?;
        if t.kind != ToolKind::Flat {
            return Err(CamError::BadTool("粗加工必须是平底刀（Z 层行切）"));
        }
        if p.rough_stepover_mm <= 0.0 {
            return Err(CamError::BadParam("rough_stepover 必须为正"));
        }
    }
    Ok(())
}

/// 从 `from` 往下到 `to` 的层深，最后一层恰好是 `to`。
fn descend(from: f64, to: f64, step: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut z = from - step;
    while z > to + 1e-9 {
        out.push(z);
        z -= step;
    }
    out.push(to);
    out
}

// ─────────────────────────── 高度场 ───────────────────────────

/// XY 网格上的最高表面 `z(x,y)` 与命中三角形的法向。
struct HeightMap {
    x0: f64,
    y0: f64,
    step: f64,
    nx: usize,
    ny: usize,
    z: Vec<f64>,
    n: Vec<[f64; 3]>,
    /// 每列的**独立表面层数**（命中 z 相差 > 0.01 mm 才算一层）。
    /// 封闭实体通常是 2（顶 + 底）；>2 说明中间还有看不见的面。
    layers: Vec<u16>,
}

impl HeightMap {
    fn build(mesh: &[Tri], stock: Stock, step: f64) -> Self {
        let nx = ((stock.size().x / step).ceil() as usize) + 1;
        let ny = ((stock.size().y / step).ceil() as usize) + 1;
        let mut map = Self {
            x0: stock.min.x,
            y0: stock.min.y,
            step,
            nx,
            ny,
            z: vec![f64::NEG_INFINITY; nx * ny],
            n: vec![[0.0; 3]; nx * ny],
            layers: vec![0; nx * ny],
        };
        // 三角形按 XY 包围盒分桶：一个 0.25 mm 的高度场不该对每个点扫全网格。
        let cell = (step * 4.0).max(1.0);
        let cw = ((stock.size().x / cell).ceil() as usize).max(1);
        let ch = ((stock.size().y / cell).ceil() as usize).max(1);
        let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); cw * ch];
        for (ti, t) in mesh.iter().enumerate() {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for v in t {
                lo[0] = lo[0].min(v.x);
                lo[1] = lo[1].min(v.y);
                hi[0] = hi[0].max(v.x);
                hi[1] = hi[1].max(v.y);
            }
            let i0 = (((lo[0] - stock.min.x) / cell).floor() as i64).clamp(0, cw as i64 - 1) as usize;
            let i1 = (((hi[0] - stock.min.x) / cell).floor() as i64).clamp(0, cw as i64 - 1) as usize;
            let j0 = (((lo[1] - stock.min.y) / cell).floor() as i64).clamp(0, ch as i64 - 1) as usize;
            let j1 = (((hi[1] - stock.min.y) / cell).floor() as i64).clamp(0, ch as i64 - 1) as usize;
            for j in j0..=j1 {
                for i in i0..=i1 {
                    buckets[j * cw + i].push(ti as u32);
                }
            }
        }
        // 法向取**高度场梯度**（中心差分），不用命中三角形的面法向：
        // STL 是 0.5 mm 的折面，按单片面法向偏置球头会在相邻面上啃出 0.03 mm
        // 的过切（实测）。梯度把折角抹平一档，球头切的是"采样曲面"而不是某一片。
        let mut normals = vec![[0.0f64; 3]; nx * ny];
        let mut zs: Vec<f64> = Vec::with_capacity(12);
        for j in 0..ny {
            let y = map.y0 + j as f64 * step;
            let bj = (((y - stock.min.y) / cell).floor() as usize).min(ch - 1);
            for i in 0..nx {
                let x = map.x0 + i as f64 * step;
                let bi = (((x - stock.min.x) / cell).floor() as usize).min(cw - 1);
                let mut best = f64::NEG_INFINITY;
                let mut best_n = [0.0; 3];
                zs.clear();
                for &ti in &buckets[bj * cw + bi] {
                    if let Some((z, n)) = ray_z(&mesh[ti as usize], x, y) {
                        zs.push(z);
                        if z > best {
                            best = z;
                            best_n = n;
                        }
                    }
                }
                // 共边三角形会让同一个面被命中两次：按 10 µm 合并成"一层"，
                // 否则封闭实体的顶面在网格线上会被数成 2 层，误判倒扣。
                zs.sort_by(|a, b| b.total_cmp(a));
                let mut layers = 0u16;
                let mut prev = f64::INFINITY;
                for &z in &zs {
                    if prev - z > 0.01 {
                        layers += 1;
                    }
                    prev = z;
                }
                map.z[j * nx + i] = best;
                map.n[j * nx + i] = best_n;
                map.layers[j * nx + i] = layers;
            }
        }
        let z_at = |i: isize, j: isize| -> Option<f64> {
            if i < 0 || j < 0 || i >= nx as isize || j >= ny as isize {
                return None;
            }
            let v = map.z[j as usize * nx + i as usize];
            v.is_finite().then_some(v)
        };
        for j in 0..ny {
            for i in 0..nx {
                let k = j * nx + i;
                let Some(z) = z_at(i as isize, j as isize) else {
                    continue;
                };
                let zx = match (z_at(i as isize - 1, j as isize), z_at(i as isize + 1, j as isize)) {
                    (Some(a), Some(b)) => (b - a) / (2.0 * step),
                    (None, Some(b)) => (b - z) / step,
                    (Some(a), None) => (z - a) / step,
                    (None, None) => 0.0,
                };
                let zy = match (z_at(i as isize, j as isize - 1), z_at(i as isize, j as isize + 1)) {
                    (Some(a), Some(b)) => (b - a) / (2.0 * step),
                    (None, Some(b)) => (b - z) / step,
                    (Some(a), None) => (z - a) / step,
                    (None, None) => 0.0,
                };
                let l = (zx * zx + zy * zy + 1.0).sqrt();
                normals[k] = [-zx / l, -zy / l, 1.0 / l];
            }
        }
        map.n = normals;
        map
    }

    fn index(&self, x: f64, y: f64) -> Option<usize> {
        let i = ((x - self.x0) / self.step).round();
        let j = ((y - self.y0) / self.step).round();
        if i < 0.0 || j < 0.0 {
            return None;
        }
        let (i, j) = (i as usize, j as usize);
        (i < self.nx && j < self.ny).then(|| j * self.nx + i)
    }

    fn at(&self, x: f64, y: f64) -> Option<(f64, [f64; 3])> {
        let k = self.index(x, y)?;
        let z = self.z[k];
        z.is_finite().then(|| (z, self.n[k]))
    }

    /// 半径 `r` 圆盘内的最高表面。无命中 = `-inf`。
    fn max_within(&self, x: f64, y: f64, r: f64) -> f64 {
        let k = (r / self.step).ceil() as i64;
        let ci = ((x - self.x0) / self.step).round() as i64;
        let cj = ((y - self.y0) / self.step).round() as i64;
        let mut best = f64::NEG_INFINITY;
        for dj in -k..=k {
            let j = cj + dj;
            if j < 0 || j >= self.ny as i64 {
                continue;
            }
            for di in -k..=k {
                if (di * di + dj * dj) as f64 * self.step * self.step > r * r + 1e-12 {
                    continue;
                }
                let i = ci + di;
                if i < 0 || i >= self.nx as i64 {
                    continue;
                }
                best = best.max(self.z[j as usize * self.nx + i as usize]);
            }
        }
        best
    }
}

/// 竖直射线（从 +Z 向下）与三角形的交点。返回最高交点的 z 与朝上的法向。
fn ray_z(t: &Tri, x: f64, y: f64) -> Option<(f64, [f64; 3])> {
    let (a, b, c) = (t[0], t[1], t[2]);
    // XY 平面上的重心坐标；退化（竖直三角形在 XY 上投影成线）直接跳过。
    let den = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if den.abs() < 1e-18 {
        return None;
    }
    let u = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / den;
    let v = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / den;
    let w = 1.0 - u - v;
    const E: f64 = -1e-9;
    if u < E || v < E || w < E {
        return None;
    }
    let z = u * a.z + v * b.z + w * c.z;
    let mut n = (b - a).cross(c - a);
    let len = n.length();
    if len < 1e-15 {
        return None;
    }
    n = n / len;
    if n.z < 0.0 {
        n = -n; // 法向统一朝上（STL 的绕向不保证）
    }
    Some((z, [n.x, n.y, n.z]))
}

// ─────────────────────────── 刀路 ───────────────────────────

/// 一层粗加工：在"刀盘完全在余量以上"的区域行切。
///
/// 可下刀判据是 `max(z within r) <= level - allowance`——这是**腐蚀**，
/// 不是简单看列本身：只看列本身会让刀盘边缘啃进旁边更高的材料。
#[allow(clippy::too_many_arguments)]
fn rough_level(
    e: &mut Emit,
    map: &HeightMap,
    stock: Stock,
    z: f64,
    r: f64,
    allowance: f64,
    stepover: f64,
    safe: f64,
    p: &Process,
    stats: &mut Stats,
) {
    // 判断余量 0.05 mm：零件最低面常比"理论平面"高一点点（高斯尾、网格舍入），
    // 卡 1e-9 会让最低层一个可下刀列都没有，粗加工整层消失。
    // 这只影响"哪些列算可下刀"，**不降低切削平面**——层深仍是 z，
    // 所以它不会造成过切；代价是实际留量可能从 0.20 掉到 0.15。
    let limit = z - allowance + 0.05;
    let n_lines = (((stock.size().y) / stepover).ceil() as usize).max(1);
    let mut flip = false;
    for j in 0..=n_lines {
        let y = stock.min.y + (j as f64 * stepover).min(stock.size().y);
        let mut runs: Vec<(f64, f64)> = Vec::new();
        let mut run_start: Option<f64> = None;
        let mut last_x = 0.0;
        for i in 0..map.nx {
            let x = map.x0 + i as f64 * map.step;
            let ok = map.max_within(x, y, r) <= limit;
            match (ok, run_start) {
                (true, None) => run_start = Some(x),
                (false, Some(s)) => {
                    if last_x - s > 1e-9 {
                        runs.push((s, last_x));
                    }
                    run_start = None;
                }
                _ => {}
            }
            if ok {
                last_x = x;
            }
        }
        if let Some(s) = run_start
            && last_x - s > 1e-9
        {
            runs.push((s, last_x));
        }
        if flip {
            runs.reverse();
        }
        flip = !flip;
        for (a, b) in runs {
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            e.rapid_xy(a, y, safe);
            // **必须 G1 下刀**：G0 在仿真里不算切削，直接扎下去会"看起来切了、
            // 实际没去料"。从安全高度下到层深，慢，但语义正确。
            e.feed_z(z, p.plunge_mm_min);
            e.feed_xy(b, y, p.feed_mm_min);
            stats.rough_moves += 1;
            e.rapid_z(safe);
        }
    }
}

/// 平行精加工：球头刀沿面法向偏置，行距由 `finish_stepover` 决定。
#[allow(clippy::too_many_arguments)]
fn finish_pass(
    e: &mut Emit,
    map: &HeightMap,
    stock: Stock,
    r: f64,
    p: &Process,
    safe: f64,
    part_min: f64,
    stats: &mut Stats,
) {
    let dz_min = (p.max_finish_slope_deg.to_radians().cos()).max(1e-3);
    let step = p.finish_stepover_mm;
    // 行距是 step，但**沿走刀方向的采样要更密**：球头在两点之间走直线，
    // 弦高 ≈ dx²/(8(R+ρ))。用行距当采样步长会在曲面上啃出 0.03 mm 的弦切过切
    // （实测），把 dx 收到 sample_mm 后降到 0.002 mm 量级。
    let dx = p.sample_mm.min(step);
    let n_lines = (((stock.size().y) / step).ceil() as usize).max(1);
    let mut flip = false;
    for j in 0..=n_lines {
        let y = stock.min.y + (j as f64 * step).min(stock.size().y);
        let mut pts: Vec<DVec3> = Vec::new();
        let mut runs: Vec<Vec<DVec3>> = Vec::new();
        for i in 0..=((stock.size().x / dx).ceil() as usize) {
            let x = stock.min.x + (i as f64 * dx).min(stock.size().x);
            let tip = map.at(x, y).and_then(|(z, n)| {
                if n[2] < dz_min {
                    return None; // 太陡：球头法向偏置会把刀尖甩到零件下方
                }
                let n = DVec3::new(n[0], n[1], n[2]);
                let tip = DVec3::new(x, y, z) + n * r - DVec3::new(0.0, 0.0, r);
                (tip.z > part_min - 3.0 * r).then_some(tip)
            });
            match tip {
                Some(t) => pts.push(t),
                None => {
                    if pts.len() >= 2 {
                        runs.push(std::mem::take(&mut pts));
                    } else {
                        pts.clear();
                    }
                }
            }
        }
        if pts.len() >= 2 {
            runs.push(pts);
        }
        if flip {
            runs.reverse();
            for r in &mut runs {
                r.reverse();
            }
        }
        flip = !flip;
        for run in runs {
            e.rapid_xy(run[0].x, run[0].y, safe);
            e.feed_z(run[0].z, p.plunge_mm_min);
            for t in &run[1..] {
                e.feed_xyz(*t, p.feed_mm_min);
            }
            stats.finish_moves += run.len() - 1;
            e.rapid_z(safe);
        }
    }
}

// ─────────────────────────── G 代码输出 ───────────────────────────

struct Emit {
    s: String,
    last: Option<DVec3>,
    cut_mm: f64,
    rapid_mm: f64,
    line_no: usize,
}

impl Emit {
    fn new() -> Self {
        Self {
            s: String::with_capacity(1 << 16),
            last: None,
            cut_mm: 0.0,
            rapid_mm: 0.0,
            line_no: 0,
        }
    }

    fn raw(&mut self, line: &str) {
        self.s.push_str(line);
        self.s.push('\n');
        self.line_no += 1;
    }

    fn tool_change(&mut self, t: &ToolSpec, rpm: u32) {
        self.raw(&format!("T{} M6", t.number));
        self.raw(&format!("S{rpm} M3"));
    }

    fn axis_words(&mut self, p: DVec3) -> (String, f64) {
        let mut w = String::new();
        let mut d = 0.0;
        if let Some(l) = self.last {
            if (p.x - l.x).abs() > 1e-7 {
                w.push_str(&format!(" X{:.4}", p.x));
            }
            if (p.y - l.y).abs() > 1e-7 {
                w.push_str(&format!(" Y{:.4}", p.y));
            }
            if (p.z - l.z).abs() > 1e-7 {
                w.push_str(&format!(" Z{:.4}", p.z));
            }
            d = (p - l).length();
        } else {
            w = format!(" X{:.4} Y{:.4} Z{:.4}", p.x, p.y, p.z);
        }
        self.last = Some(p);
        (w, d)
    }

    fn rapid_xy(&mut self, x: f64, y: f64, safe: f64) {
        // 先抬到安全高度，再横移：不会在零件里穿行。
        if self.last.is_none_or(|l| l.z < safe - 1e-9) {
            self.rapid_z(safe);
        }
        let z = self.last.map_or(safe, |l| l.z);
        let (w, d) = self.axis_words(DVec3::new(x, y, z));
        self.rapid_mm += d;
        if !w.is_empty() {
            self.raw(&format!("G0{w}"));
        }
    }

    fn rapid_z(&mut self, z: f64) {
        let last = self.last.unwrap_or(DVec3::ZERO);
        if (z - last.z).abs() < 1e-7 {
            return;
        }
        let p = DVec3::new(last.x, last.y, z);
        let (w, d) = self.axis_words(p);
        self.rapid_mm += d;
        self.raw(&format!("G0{w}"));
    }

    fn feed_z(&mut self, z: f64, f: f64) {
        let last = self.last.unwrap_or(DVec3::ZERO);
        let p = DVec3::new(last.x, last.y, z);
        let (w, d) = self.axis_words(p);
        self.cut_mm += d;
        if !w.is_empty() {
            self.raw(&format!("G1{w} F{f:.0}"));
        }
    }

    fn feed_xy(&mut self, x: f64, y: f64, f: f64) {
        let last = self.last.unwrap();
        let p = DVec3::new(x, y, last.z);
        let (w, d) = self.axis_words(p);
        self.cut_mm += d;
        if !w.is_empty() {
            self.raw(&format!("G1{w} F{f:.0}"));
        }
    }

    fn feed_xyz(&mut self, p: DVec3, f: f64) {
        let (w, d) = self.axis_words(p);
        self.cut_mm += d;
        if !w.is_empty() {
            self.raw(&format!("G1{w} F{f:.0}"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(z: f64, size: f64) -> Vec<Tri> {
        vec![
            [
                DVec3::new(0.0, 0.0, z),
                DVec3::new(size, 0.0, z),
                DVec3::new(size, size, z),
            ],
            [
                DVec3::new(0.0, 0.0, z),
                DVec3::new(size, size, z),
                DVec3::new(0.0, size, z),
            ],
        ]
    }

    fn job<'a>(mesh: &'a [Tri]) -> Job<'a> {
        Job {
            mesh,
            stock: Stock::from_size(DVec3::new(20.0, 20.0, 4.0)),
            rough: Some(ToolSpec::flat(1, 6.0, 20.0)),
            finish: ToolSpec::ball(2, 4.0, 20.0),
            process: Process {
                sample_mm: 0.5,
                rough_stepover_mm: 3.0,
                finish_stepover_mm: 0.5,
                stepdown_mm: 1.0,
                ..Default::default()
            },
        }
    }

    #[test]
    fn height_map_recovers_a_plane() {
        let m = plane(2.5, 20.0);
        let map = HeightMap::build(&m, Stock::from_size(DVec3::new(20.0, 20.0, 4.0)), 0.5);
        let (z, n) = map.at(10.0, 10.0).unwrap();
        assert!((z - 2.5).abs() < 1e-9, "z={z}");
        assert!(n[2] > 0.99, "法向应朝上：{n:?}");
    }

    #[test]
    fn flat_plane_generates_rough_and_finish_without_going_below_part() {
        let m = plane(2.0, 20.0);
        let prog = generate(&job(&m)).unwrap();
        assert!(prog.gcode.contains("G21 G90 G17 G94"));
        assert!(prog.gcode.contains("T1 M6"));
        assert!(prog.gcode.contains("T2 M6"));
        assert!(prog.gcode.contains("M30"));
        // 粗加工最低层 = 2.2（2.0 + 0.2 余量），精加工刀尖 = 2.0 + R*(nz-1) = 2.0
        // 文本里所有 Z 都不能低于 1.9（球头在平面上 nz=1）。
        for line in prog.gcode.lines() {
            if let Some(z) = line.split(" Z").nth(1) {
                let z: f64 = z.split_whitespace().next().unwrap().parse().unwrap();
                assert!(z >= 1.9, "Z 低于零件：{line}");
            }
        }
        assert!(prog.stats.levels >= 2);
        assert!(prog.stats.finish_moves > 10);
    }

    #[test]
    fn rejects_mesh_outside_stock() {
        let m = plane(2.0, 30.0);
        let e = generate(&job(&m)).unwrap_err();
        assert!(matches!(e, CamError::OutsideStock { .. }), "{e:?}");
    }

    #[test]
    fn rejects_non_ball_finish() {
        let m = plane(2.0, 20.0);
        let mut j = job(&m);
        j.finish = ToolSpec::flat(2, 4.0, 20.0);
        assert!(matches!(generate(&j).unwrap_err(), CamError::BadTool(_)));
    }

    #[test]
    fn descend_ends_exactly_at_target() {
        let l = descend(4.0, 2.2, 1.0);
        assert_eq!(l, vec![3.0, 2.2]);
        let l = descend(3.0, 2.2, 1.0);
        assert_eq!(l, vec![2.2]);
    }
}
