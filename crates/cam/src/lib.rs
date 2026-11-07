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

pub mod feed;
pub mod multi;
pub mod rotary;
pub mod surface;
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

/// **工序坐标系**：把"局部 +Z 向上"的刀路变换到世界。
///
/// 3 轴高度场只会沿 +Z 加工。要"从 −X 开粗"，不必再写一套 CAM：把模型转进
/// 该方向的局部坐标系，照常出刀路，再用这个 `Frame` 把点变回世界即可。
/// 这样多方向开粗 = 多次调用同一条 3 轴链路。
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    /// 行优先，`world = rot · local`（正交阵，列是局部的 x/y/z 在世界里的方向）。
    pub rot: [[f64; 3]; 3],
    pub t: DVec3,
}

impl Default for Frame {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Frame {
    pub const IDENTITY: Frame = Frame {
        rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        t: DVec3::ZERO,
    };

    /// 让**局部 +Z 指向世界方向 `dir`**。x/y 的取法只要求右手系：优先让局部 X
    /// 落在世界 XZ 平面里，避免绕 Z 自转（否则同一个方向会有无数个解）。
    pub fn from_dir(dir: DVec3) -> Self {
        let z = dir.normalize();
        let helper = if z.z.abs() < 0.9 {
            DVec3::new(0.0, 0.0, 1.0)
        } else {
            DVec3::new(0.0, 1.0, 0.0)
        };
        let x = helper.cross(z).normalize();
        let y = z.cross(x);
        Self {
            rot: [
                [x.x, y.x, z.x],
                [x.y, y.y, z.y],
                [x.z, y.z, z.z],
            ],
            t: DVec3::ZERO,
        }
    }

    pub fn apply(&self, p: DVec3) -> DVec3 {
        DVec3::new(
            self.rot[0][0] * p.x + self.rot[0][1] * p.y + self.rot[0][2] * p.z + self.t.x,
            self.rot[1][0] * p.x + self.rot[1][1] * p.y + self.rot[1][2] * p.z + self.t.y,
            self.rot[2][0] * p.x + self.rot[2][1] * p.y + self.rot[2][2] * p.z + self.t.z,
        )
    }

    /// 世界 → 局部（正交阵转置即逆）。
    pub fn unapply(&self, p: DVec3) -> DVec3 {
        let q = p - self.t;
        DVec3::new(
            self.rot[0][0] * q.x + self.rot[1][0] * q.y + self.rot[2][0] * q.z,
            self.rot[0][1] * q.x + self.rot[1][1] * q.y + self.rot[2][1] * q.z,
            self.rot[0][2] * q.x + self.rot[1][2] * q.y + self.rot[2][2] * q.z,
        )
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
    /// **避空颈长**（mm）：刃长以上、缩颈之前的**等径细颈**。
    ///
    /// 真实长颈刀 = 刃 + 细颈 + 缩颈 + Ø6 柄。这一段**不切削但要查碰撞**
    /// （半径 = 刃顶半径）。`flute_mm` 不能兼任它：把 flute 加长等于把碰撞
    /// 豁免区跟着加长，刀在细颈那段横着扫（实测 flute 16→25 时 Ø1 刀的
    /// "切入啃"从 3 暴增到 249、合格率 96.5 → 95.7%）。
    pub neck_mm: f64,
    /// **刀柄直径**（mm）。`0` = 与刃部同径的"等径柄"（旧行为，用于历史对拍）。
    ///
    /// 真实刀具统一 **Ø6 h6 夹持**：小刃径的刀经缩颈扩到 6，Ø6 段在刃长以上。
    /// 之前这里隐式等于刃径（Ø1.0 刀的"刀杆"只有 Ø1.0），碰撞检查因此偏乐观
    /// ——叶轮报"刀杆撞 1420 点"是按细杆算的，真实 Ø6 柄在叶片通道里会撞更多。
    pub shank_diameter: f64,
}

impl ToolSpec {
    pub fn flat(number: u16, diameter: f64, flute_mm: f64) -> Self {
        Self {
            number,
            kind: ToolKind::Flat,
            diameter,
            flute_mm,
            taper_half_deg: 0.0,
            neck_mm: 0.0,
            shank_diameter: 0.0,
        }
    }

    pub fn ball(number: u16, diameter: f64, flute_mm: f64) -> Self {
        Self {
            number,
            kind: ToolKind::Ball,
            diameter,
            flute_mm,
            taper_half_deg: 0.0,
            neck_mm: 0.0,
            shank_diameter: 0.0,
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
            neck_mm: 0.0,
            shank_diameter: 0.0,
        }
    }

    /// 设定刀柄直径（链式）。`d = 0` 保持"等径柄"。
    pub fn with_shank(mut self, d: f64) -> Self {
        self.shank_diameter = d.max(0.0);
        self
    }

    /// 设定避空颈长（链式）。`n <= 0` = 无颈。
    pub fn with_neck(mut self, n: f64) -> Self {
        self.neck_mm = n.max(0.0);
        self
    }

    /// 颈顶高度：刃长 + 避空颈。缩颈从这里开始。
    pub fn neck_top(&self) -> f64 {
        self.flute_mm.max(self.radius()) + self.neck_mm.max(0.0)
    }

    /// 缩颈长度：从刃顶按 **30° 半角**扩到柄径需要多高。
    /// 等径柄（`shank_diameter = 0`）时为 0。
    pub fn taper_len(&self) -> f64 {
        let r_cut = self.top_radius();
        let r_shank = self.shank_radius();
        if r_shank > r_cut + 1e-9 {
            (r_shank - r_cut) / 30f64.to_radians().tan()
        } else {
            0.0
        }
    }

    /// 刀尖以上 `z` 高度处的**最大半径**——碰撞检查的分段判据。
    ///
    /// 三段（按刀具实际形状，不是按"一刀切"）：
    ///
    /// * `z ≤ flute`：**切削刃**。这一段会切掉材料，**不做碰撞检查**
    ///   （只有过切检查，那是另一套判据）。
    /// * `flute < z ≤ flute + neck`：**避空细颈**。等径（= 刃顶半径），要查。
    /// * `flute + neck < z ≤ flute + neck + taper`：**缩颈**。30° 线性长到柄径，要查。
    /// * `z > flute + neck + taper`：**Ø6 刀柄**。要查。
    ///
    /// 早先拿 `shank_r` 一刀切地从刃顶查到顶，等于把避空段也按 Ø6 算——
    /// Ø1.0 刀的避空段本来只有 Ø1 到 Ø3 那么粗，被当成 Ø6 会误杀大量刀点。
    pub fn radius_at(&self, z: f64) -> f64 {
        let r_cut = self.top_radius();
        let flute = self.flute_mm.max(self.radius());
        if z <= flute {
            return r_cut;
        }
        let neck_top = self.neck_top();
        if z <= neck_top {
            return r_cut;
        }
        let t = self.taper_len();
        if z <= neck_top + t {
            r_cut + (z - neck_top) * 30f64.to_radians().tan()
        } else {
            self.shank_radius()
        }
    }

    /// 刀柄半径：有 `shank_diameter` 就用它，否则退回"刃顶半径"（等径柄）。
    ///
    /// **碰撞检查必须用这个**，不能用 `top_radius()`——后者是切削部的最大
    /// 半径，对 Ø1 长颈刀只有 0.5，会把"Ø6 柄撞到对面叶片"这件事漏掉。
    pub fn shank_radius(&self) -> f64 {
        let r_cut = self.top_radius();
        if self.shank_diameter > 0.0 {
            (self.shank_diameter * 0.5).max(r_cut)
        } else {
            r_cut
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
        let r_cut = self.top_radius();
        let r_shank = self.shank_radius();
        let neck_top = self.neck_top();
        let top = neck_top + 20.0;
        // 缩颈：从颈顶按 **30° 半角**扩到柄径（Ø6 平底刀退化为 0）
        let taper_len = self.taper_len();
        ToolDef {
            number: self.number,
            kind: match self.kind {
                ToolKind::Flat => CutterKind::Flat,
                ToolKind::Ball => CutterKind::Ball,
                ToolKind::TaperedBall => CutterKind::TaperedBall,
            },
            diameter: 2.0 * r_cut,
            corner_radius: if self.kind == ToolKind::Flat { 0.0 } else { r },
            flute_length: self.flute_mm,
            shank: Profile::new(vec![
                DVec2::new(0.0, self.flute_mm),
                DVec2::new(r_cut, self.flute_mm),
                // 避空细颈：等径（= 刃顶半径），参与碰撞检查
                DVec2::new(r_cut, neck_top),
                DVec2::new(r_shank, neck_top + taper_len),
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
    /// 规划时检查**刀杆**（刃长以上那一段）会不会撞到工件。
    ///
    /// 高度场只保证"刀刃"不侵入：球头/平底刀的 drop-cutter 取的是刀盘内的最高点，
    /// 刀刃以上同半径的那段柱体自然也在外面。真正会撞的是 ① 锥度刀的锥颈比尖端粗、
    /// ② 零件有倒扣（高度场根本没表示出来的料）。所以这里按**三角网格**逐点算
    /// 刀轴到工件的距离，撞到就把该点判为不可达、留料。
    pub check_shank: bool,
    /// 刀杆与工件的最小间隙（mm）。
    pub shank_clearance_mm: f64,
    /// 遇到多层（中空/孔洞/倒扣）时**不拒收**，只把够不到的部分留在料里。
    ///
    /// 3 轴高度场天然只能加工"每个 XY 列的上表面"：孔、内腔、倒扣的背面它切不到。
    /// 严格模式（默认）直接拒收，避免用户拿到一份"看起来做了、其实只切了外壳"
    /// 的程序；多方向/定轴模式（用户明确要求"尽最大努力、做不了的跳过"）打开它，
    /// 让这一道把它看得见的面都切了，够不到的留料并如实报告。
    pub allow_undercut: bool,
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
            check_shank: true,
            shank_clearance_mm: 0.5,
            allow_undercut: false,
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
    /// **残料清根的目标点**（世界坐标）。`Some` 时只在这些点附近走刀。
    ///
    /// 用来做"凹角小刀清根"：上一道精加工在凹角处必然留下 `R(1/sin(θ/2)−1)`
    /// 的残料，换一把小刀只在残料那一圈补一遍，切长只涨几个百分点。
    /// 掩膜会把目标点**按刀半径外扩**（刀心在旁边时球面才够得到）。
    pub rest: Option<&'a [DVec3]>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stats {
    /// 因为**刀杆会撞工件**而放弃的刀点数。
    pub shank_skips: usize,
    /// 因为**球面会啃进设计面**（悬伸/倒扣）而放弃的刀点数。
    pub ball_skips: usize,
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
    generate_in(job, None)
}

/// 带**工序坐标系**的生成：刀路在局部坐标系里算，输出的 G 代码是世界坐标。
///
/// 多方向开粗就是换不同 `Frame` 反复调用它（模型要先 `unapply` 到该坐标系）。
pub fn generate_in(job: &Job<'_>, frame: Option<Frame>) -> Result<Program, CamError> {
    validate(job)?;
    let p = &job.process;
    let map = HeightMap::build(job.mesh, job.stock, p.sample_mm);
    // 残料清根的掩膜：局部坐标系里，按**这把刀**的半径外扩
    let rest = job.rest.map(|pts| {
        RestMask::build(pts, frame, job.stock, p.sample_mm, job.finish.radius())
    });

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
    let undercut_frac = if total > 0 {
        under as f64 / total as f64
    } else {
        0.0
    };
    if total > 0 && under * 50 > total && !p.allow_undercut {
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

    // 刀杆检查只在这种情况才必要：零件**有倒扣**（高度场没表示的料），或者
    // 锥度刀的**锥颈比尖端粗**。否则 drop-cutter 已经保证整根刀柱在几何之外
    // （球心以上是半径 R 的柱体，而刀盘内没有任何点在球内 → 柱体也在外）。
    let neck_wider = |t: &ToolSpec| t.top_radius() > t.radius() + 1e-6;
    let need_shank = p.check_shank
        && (undercut_frac > 0.02
            || neck_wider(&job.finish)
            || job.rough.as_ref().is_some_and(neck_wider));
    // 清根道**一定**要建索引：`ball_hits` 靠它判"球有没有啃到悬伸"
    let index =
        (need_shank || job.rest.is_some()).then(|| MeshIndex::build(job.mesh, job.stock));

    let mut e = Emit::new(frame);
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
    if undercut_frac > 0.02 {
        e.raw(&format!(
            "(WARN undercut {:.1}% columns multi-layer: 这些面的背面本道够不到，留料)",
            undercut_frac * 100.0
        ));
    }
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
    e.tool_change(&job.finish, p.spindle_rpm);
    finish_pass(
        &mut e,
        &map,
        job.stock,
        &job.finish,
        p,
        safe,
        part_min,
        index.as_ref(),
        rest.as_ref(),
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
    // 平底刀走 drop-cutter（刀盘碰到最高点就停），球头走法向偏置；两种都允许。
    let max_stepover = if job.finish.kind == ToolKind::Flat {
        job.finish.diameter * 0.95
    } else {
        job.finish.radius()
    };
    if p.finish_stepover_mm > max_stepover {
        return Err(CamError::BadParam(if job.finish.kind == ToolKind::Flat {
            "finish_stepover 超过平底刀直径的 95%（会留大台阶）"
        } else {
            "finish_stepover 超过球头半径（残高会大于半径）"
        }));
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

// ─────────────────────────── 刀杆碰撞 ───────────────────────────

/// 三角网格的 XY 分桶索引：查"某点到工件表面的最近距离"。
///
/// 高度场只能保证**刀刃**不侵入（drop-cutter 的几何保证），刀刃以上那一段柱体
/// 在"零件有倒扣"时仍可能插进高度场没表示出来的料里。这里按真实三角形算距离。
pub(crate) struct MeshIndex {
    cell: f64,
    lo: DVec3,
    nx: usize,
    ny: usize,
    buckets: Vec<Vec<u32>>,
    tris: Vec<Tri>,
}

impl MeshIndex {
    pub(crate) fn build(mesh: &[Tri], stock: Stock) -> Self {
        let cell = (stock.size().x.max(stock.size().y) / 48.0).clamp(0.5, 5.0);
        let nx = ((stock.size().x / cell).ceil() as usize + 1).max(1);
        let ny = ((stock.size().y / cell).ceil() as usize + 1).max(1);
        let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny];
        for (ti, t) in mesh.iter().enumerate() {
            let (mut x0, mut x1) = (f64::INFINITY, f64::NEG_INFINITY);
            let (mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY);
            for v in t {
                x0 = x0.min(v.x);
                x1 = x1.max(v.x);
                y0 = y0.min(v.y);
                y1 = y1.max(v.y);
            }
            let i0 = (((x0 - stock.min.x) / cell).floor() as i64).clamp(0, nx as i64 - 1) as usize;
            let i1 = (((x1 - stock.min.x) / cell).floor() as i64).clamp(0, nx as i64 - 1) as usize;
            let j0 = (((y0 - stock.min.y) / cell).floor() as i64).clamp(0, ny as i64 - 1) as usize;
            let j1 = (((y1 - stock.min.y) / cell).floor() as i64).clamp(0, ny as i64 - 1) as usize;
            for j in j0..=j1 {
                for i in i0..=i1 {
                    buckets[j * nx + i].push(ti as u32);
                }
            }
        }
        Self {
            cell,
            lo: stock.min,
            nx,
            ny,
            buckets,
            tris: mesh.to_vec(),
        }
    }

    /// 到 `reach` 范围内最近三角形的距离（无符号）。附近没有三角形 → `∞`。
    pub(crate) fn min_dist(&self, p: DVec3, reach: f64) -> f64 {
        let i0 = (((p.x - reach - self.lo.x) / self.cell).floor() as i64)
            .clamp(0, self.nx as i64 - 1) as usize;
        let i1 = (((p.x + reach - self.lo.x) / self.cell).floor() as i64)
            .clamp(0, self.nx as i64 - 1) as usize;
        let j0 = (((p.y - reach - self.lo.y) / self.cell).floor() as i64)
            .clamp(0, self.ny as i64 - 1) as usize;
        let j1 = (((p.y + reach - self.lo.y) / self.cell).floor() as i64)
            .clamp(0, self.ny as i64 - 1) as usize;
        let mut best = f64::INFINITY;
        for j in j0..=j1 {
            for i in i0..=i1 {
                for &ti in &self.buckets[j * self.nx + i] {
                    let d2 = pt_tri_dist2(p, &self.tris[ti as usize]);
                    if d2 < best {
                        best = d2;
                    }
                }
            }
        }
        best.sqrt()
    }
}

/// 点到三角形的最近距离平方（Ericson《Real-Time Collision Detection》5.1.5）。
fn pt_tri_dist2(p: DVec3, t: &Tri) -> f64 {
    let (a, b, c) = (t[0], t[1], t[2]);
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (p - a).length_squared();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (p - b).length_squared();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).length_squared();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (p - c).length_squared();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).length_squared();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).length_squared();
    }
    let den = 1.0 / (va + vb + vc);
    let v = vb * den;
    let w = vc * den;
    (p - (a + ab * v + ac * w)).length_squared()
}

/// 刀杆（刃长以上、刀尖以上整段）会不会碰到工件。
///
/// 局部坐标系里刀轴恒为 +Z：沿轴取样，逐点问到三角网格的最近距离。
pub(crate) fn shank_hits(ix: &MeshIndex, tip: DVec3, tool: &ToolSpec, clearance: f64) -> bool {
    let r = tool.radius();
    let flute = tool.flute_mm.max(r);
    let shank_r = tool.shank_radius().max(r);
    let taper = tool.taper_len();
    let len = tool.neck_top() + 20.0;

    // **刀刃段（z ≤ flute）整段不查**：那是切削区，材料本来就要被切掉。
    // 拿"离工件 ≥ 间隙"去要求它，会把每个刀点都判成碰撞（实测误杀 42858 点）。
    //
    // 刃长以上分两段，各自按**该高度的实际半径**查：
    //   ① 避空缩颈（flute … flute+taper）：30° 锥，半径连续变化 → 细步长
    //   ② Ø6 刀柄（flute+taper … 顶）：半径恒定 → 粗步长
    // 拿 `shank_r` 一刀切地查全程，等于把避空段也当成 Ø6（偏保守，误杀刀点）。
    let mut z = flute;
    while z < tool.neck_top() + taper {
        let rr = tool.radius_at(z);
        let need = rr + clearance;
        if ix.min_dist(tip + DVec3::new(0.0, 0.0, z), need) < need {
            return true;
        }
        z += 0.3;
    }
    let step = (shank_r * 0.8).clamp(0.5, 2.0);
    let mut z = (tool.neck_top() + taper).max(flute);
    while z <= len {
        let need = shank_r + clearance;
        if ix.min_dist(tip + DVec3::new(0.0, 0.0, z), need) < need {
            return true;
        }
        z += step;
    }
    false
}

/// 球头刀在 `tip` 处时，**球面是否侵入设计面超过 `tol`**。
///
/// drop-cutter 只保证"球不穿**高度场**"，而高度场每列只认最上面那一层：
/// 悬伸下面的一块料它看不见。清根道用 Ø0.7 钻进去补残料时就会啃到悬伸的
/// 底面——实测把"不可达面"的过切从 0.1% 抬到 6.9%。这里按**真网格**再判一次：
/// 球心到设计面的距离必须 ≥ R − tol（球与设计面相切时正好等于 R）。
fn ball_hits(ix: &MeshIndex, tip: DVec3, r: f64, tol: f64) -> bool {
    let need = (r - tol).max(0.0);
    if need <= 0.0 {
        return false;
    }
    ix.min_dist(tip + DVec3::new(0.0, 0.0, r), need) < need
}

// ─────────────────────────── 高度场 ───────────────────────────

/// XY 网格上的最高表面 `z(x,y)` 与命中三角形的法向。
/// 残料清根的**局部掩膜**：只在这些高度场格子里出刀。
///
/// 世界坐标 → 本道工序的局部系 → 格子。比目标点**外扩一个刀半径**：
/// 刀心落在残料旁边一个 R 以内时，球面才真的扫得到那一点。
struct RestMask {
    x0: f64,
    y0: f64,
    step: f64,
    nx: usize,
    ny: usize,
    hit: Vec<bool>,
}

impl RestMask {
    fn build(pts: &[DVec3], frame: Option<Frame>, stock: Stock, step: f64, radius: f64) -> Self {
        let nx = ((stock.size().x / step).ceil() as usize) + 1;
        let ny = ((stock.size().y / step).ceil() as usize) + 1;
        let mut m = Self {
            x0: stock.min.x,
            y0: stock.min.y,
            step,
            nx,
            ny,
            hit: vec![false; nx * ny],
        };
        // 外扩半径（格数向上取整）
        let k = ((radius / step).ceil() as i64).max(1);
        for p in pts {
            let q = frame.map_or(*p, |f| f.unapply(*p));
            let ci = ((q.x - m.x0) / step).round() as i64;
            let cj = ((q.y - m.y0) / step).round() as i64;
            for j in (cj - k)..=(cj + k) {
                if j < 0 || j >= ny as i64 {
                    continue;
                }
                for i in (ci - k)..=(ci + k) {
                    if i < 0 || i >= nx as i64 {
                        continue;
                    }
                    m.hit[j as usize * nx + i as usize] = true;
                }
            }
        }
        m
    }

    fn has(&self, x: f64, y: f64) -> bool {
        let i = ((x - self.x0) / self.step).round() as i64;
        let j = ((y - self.y0) / self.step).round() as i64;
        if i < 0 || j < 0 || i >= self.nx as i64 || j >= self.ny as i64 {
            return false;
        }
        self.hit[j as usize * self.nx + i as usize]
    }

    /// 整行没有目标就整行跳过（省掉 nx 次判断，也让空行程不生成）。
    fn row_has(&self, y: f64) -> bool {
        let j = ((y - self.y0) / self.step).round() as i64;
        if j < 0 || j >= self.ny as i64 {
            return false;
        }
        let base = j as usize * self.nx;
        self.hit[base..base + self.nx].iter().any(|v| *v)
    }

    /// 被覆盖的格子数（只在测试里用：确认"外扩一个半径"没退化成整张图）。
    #[cfg(test)]
    fn points(&self) -> usize {
        self.hit.iter().filter(|v| **v).count()
    }
}

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

    /// 球头刀的**刀心高度**：刀盘范围内 `z + √(R²−d²)` 的最大值。
    ///
    /// 这就是球头刀的 drop-cutter：球不能与刀盘内的任何一点相交，因此刀心高度
    /// 取所有约束里最高的那个。它在**凹角**处会把球抬起来——`tip = p + R·n` 那种
    /// 局部法向偏置做不到这一点，球会贴着墙面啃进去（实测桌腿件过切 1.35 mm，
    /// 正好一个刀半径）。凸面/平面处两者等价。
    fn ball_center(&self, x: f64, y: f64, r: f64) -> Option<f64> {
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
                let i = ci + di;
                if i < 0 || i >= self.nx as i64 {
                    continue;
                }
                let d2 = ((di * di + dj * dj) as f64) * self.step * self.step;
                if d2 > r * r {
                    continue;
                }
                let z = self.z[j as usize * self.nx + i as usize];
                if !z.is_finite() {
                    continue;
                }
                let v = z + (r * r - d2).sqrt();
                if v > best {
                    best = v;
                }
            }
        }
        best.is_finite().then_some(best)
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
    tool: &ToolSpec,
    p: &Process,
    safe: f64,
    part_min: f64,
    index: Option<&MeshIndex>,
    rest: Option<&RestMask>,
    stats: &mut Stats,
) {
    let r = tool.radius();
    let flat = tool.kind == ToolKind::Flat;
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
        // 清根道：整行没有目标就不走（空行程连 G 代码都不生成）
        if rest.is_some_and(|m| !m.row_has(y)) {
            continue;
        }
        let mut pts: Vec<DVec3> = Vec::new();
        let mut runs: Vec<Vec<DVec3>> = Vec::new();
        for i in 0..=((stock.size().x / dx).ceil() as usize) {
            let x = stock.min.x + (i as f64 * dx).min(stock.size().x);
            if rest.is_some_and(|m| !m.has(x, y)) {
                // 不在残料区里：当成"这一刀不存在"，断开成两段
                if pts.len() >= 2 {
                    runs.push(std::mem::take(&mut pts));
                } else {
                    pts.clear();
                }
                continue;
            }
            let tip = map.at(x, y).and_then(|(z, n)| {
                if flat {
                    // **平底刀（立铣刀）的落刀高度 = 刀盘范围内最高点**。
                    //
                    // 平底刀没有球心可沿法向偏，正确的 CL 点是 drop-cutter：
                    // 刀具降到刀盘刚好碰到最高点为止。它天然不过切（刀底是平面，
                    // 只会切到 z 以上的材料），代价是陡壁/凹坑处留台阶——那些
                    // 面交给侧向工序去精加工。
                    let zm = map.max_within(x, y, r);
                    if !zm.is_finite() {
                        return None;
                    }
                    return (zm > part_min - 3.0 * r).then_some(DVec3::new(x, y, zm));
                }
                // 太陡的面本道不做：高度场表达不了竖直壁，drop-cutter 只会沿壁
                // 留一条 R 的圆角；交给侧向工序去精加工更靠谱。
                if n[2] < dz_min {
                    return None;
                }
                // 球头：drop-cutter。`z` 只用来判"这里有没有面"，高度由刀盘决定。
                let _ = z;
                let zc = map.ball_center(x, y, r)?;
                let tip = DVec3::new(x, y, zc - r);
                (tip.z > part_min - 3.0 * r).then_some(tip)
            });
            // 刀杆撞工件的点直接放弃：宁可留料，也不能撞。
            //
            // 清根道是**贴着墙**走的：小刀的刃与刀颈同半径，在凹角里本来就与两侧
            // 墙面相切（间隙 0）。拿"必须留 0.5 mm 间隙"去判它，会把每个凹角点都
            // 判成碰撞（实测误杀 1351 点）。清根道改判"刀颈侵入设计面不超过 0.15"，
            // 与球面的 `ball_hits` 同一口径。
            let clearance = if rest.is_some() {
                -0.15
            } else {
                p.shank_clearance_mm
            };
            let tip = match tip {
                Some(t) if index.is_some_and(|ix| shank_hits(ix, t, tool, clearance)) => {
                    stats.shank_skips += 1;
                    None
                }
                // 清根道额外查**球面**：小刀会钻到悬伸下面，那里高度场是瞎的
                Some(t) if rest.is_some() && index.is_some_and(|ix| ball_hits(ix, t, r, 0.03)) => {
                    stats.ball_skips += 1;
                    None
                }
                other => other,
            };
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
    /// 局部 → 世界。`None` = 恒等（99% 的调用）。
    frame: Option<Frame>,
    last: Option<DVec3>,
    cut_mm: f64,
    rapid_mm: f64,
    line_no: usize,
}

impl Emit {
    fn new(frame: Option<Frame>) -> Self {
        Self {
            s: String::with_capacity(1 << 16),
            frame,
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

    /// 出点。**`last` 一律存局部坐标**，只在写字符串时做一次 `frame` 变换。
    ///
    /// 这条很关键：`rapid_xy` / `feed_z` / `feed_xy` 都会拿 `last.z` / `last.x`
    /// 去拼下一个点。早版本把变换后的世界坐标存进 `last`，换坐标系后就成了
    /// "用世界 z 拼局部 x,y"——侧向工序会疯狂重复下刀，实测单方向过切 524 mm³
    /// （零件总共才 864 mm³）。
    fn axis_words(&mut self, p: DVec3) -> (String, f64) {
        let wp = match &self.frame {
            Some(f) => f.apply(p),
            None => p,
        };
        let mut w = String::new();
        let mut d = 0.0;
        if let Some(l) = self.last {
            // **判断写哪个轴，必须看世界坐标的变化**；拼点、量行程用局部坐标。
            //
            // 这两件事不能混：坐标系一转置，局部 z 的变化就落到世界 Y 上。早先
            // 用局部差值判断、却写世界值，`(0,-1,0)` 这种置换下"局部 y 没变 → 不写 Y"，
            // 于是那条 Y 指令被整个吞掉——±Y 侧向刀路的刀尖全停在安全高度，
            // 一个体素都没切到（G 代码里表现为 `G0 X-12` 紧跟 `G1 Z0.0000`，
            // 该有的 `Y-10.5000` 不见了）。
            let wl = match &self.frame {
                Some(f) => f.apply(l),
                None => l,
            };
            if (wp.x - wl.x).abs() > 1e-7 {
                w.push_str(&format!(" X{:.4}", wp.x));
            }
            if (wp.y - wl.y).abs() > 1e-7 {
                w.push_str(&format!(" Y{:.4}", wp.y));
            }
            if (wp.z - wl.z).abs() > 1e-7 {
                w.push_str(&format!(" Z{:.4}", wp.z));
            }
            d = (p - l).length();
        } else {
            w = format!(" X{:.4} Y{:.4} Z{:.4}", wp.x, wp.y, wp.z);
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

    /// 合成一个高度场：`z[j*nx+i]`，`x = i*step`、`y = j*step`。
    fn map_of(nx: usize, ny: usize, step: f64, z: Vec<f64>) -> HeightMap {
        HeightMap {
            x0: 0.0,
            y0: 0.0,
            step,
            nx,
            ny,
            n: vec![[0.0, 0.0, 1.0]; nx * ny],
            layers: vec![2; nx * ny],
            z,
        }
    }

    /// **球头 drop-cutter 的不变量**：刀心放到算出来的高度后，球不得与刀盘内
    /// 任何采样点相交。这条是"凹角不啃墙"的几何保证。
    #[test]
    fn ball_center_never_intersects_the_height_field() {
        // 台阶：y<2 一侧高 10，另一侧是 0 的地面
        let mut z = vec![0.0; 64];
        for j in 0..2 {
            for i in 0..8 {
                z[j * 8 + i] = 10.0;
            }
        }
        let map = map_of(8, 8, 1.0, z);
        let r = 2.5;
        for j in [0.0, 1.0, 2.0, 3.0, 4.5] {
            for i in [0.0, 1.0, 3.5, 6.0] {
                let Some(zc) = map.ball_center(i, j, r) else {
                    continue;
                };
                let k = (r / map.step).ceil() as i64;
                let (ci, cj) = (i.round() as i64, j.round() as i64);
                for dj in -k..=k {
                    for di in -k..=k {
                        let (ii, jj) = (ci + di, cj + dj);
                        if ii < 0 || jj < 0 || ii >= 8 || jj >= 8 {
                            continue;
                        }
                        let d2 = ((di * di + dj * dj) as f64) * map.step * map.step;
                        if d2 > r * r {
                            continue;
                        }
                        let surf = map.z[jj as usize * 8 + ii as usize];
                        let clear = zc - surf;
                        assert!(
                            clear >= (r * r - d2).sqrt() - 1e-9,
                            "({i},{j}) 处球心 {zc:.3} 与格点 ({ii},{jj}) 相交：                             间距 {clear:.3} < {:.3}",
                            (r * r - d2).sqrt()
                        );
                    }
                }
            }
        }
    }

    /// 平面上 drop-cutter 与"刀尖贴面"等价（不能因为改了算法把平的地方抬高）。
    #[test]
    fn ball_center_matches_the_surface_on_a_flat_plane() {
        let map = map_of(6, 6, 0.5, vec![7.0; 36]);
        for (i, j) in [(0.0, 0.0), (1.5, 1.0), (2.5, 2.0)] {
            let zc = map.ball_center(i, j, 1.5).unwrap();
            assert!(
                (zc - (7.0 + 1.5)).abs() < 1e-9,
                "平面上刀心应为 z+R = 8.5，实际 {zc:.4}"
            );
        }
    }

    /// 点—三角形距离：正上方、边上、顶点外三种情况。
    #[test]
    fn point_triangle_distance_is_exact() {
        let t = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(4.0, 0.0, 0.0),
            DVec3::new(0.0, 4.0, 0.0),
        ];
        // 正上方 3 mm
        let d = pt_tri_dist2(DVec3::new(1.0, 1.0, 3.0), &t).sqrt();
        assert!((d - 3.0).abs() < 1e-9, "{d}");
        // 边的外侧 2 mm
        let d = pt_tri_dist2(DVec3::new(2.0, -2.0, 0.0), &t).sqrt();
        assert!((d - 2.0).abs() < 1e-9, "{d}");
        // 顶点外 5 mm
        let d = pt_tri_dist2(DVec3::new(-3.0, -4.0, 0.0), &t).sqrt();
        assert!((d - 5.0).abs() < 1e-9, "{d}");
    }

    /// **刀杆碰撞**：顶上有一块"天花板"时，短刃刀在下面会被判碰撞；
    /// 抬高到天花板以上就没事。
    #[test]
    fn shank_hits_a_ceiling_and_clears_when_lifted() {
        // 一块 z = 10 的平板（当作倒扣面）
        let mesh = vec![[
            DVec3::new(-5.0, -5.0, 10.0),
            DVec3::new(5.0, -5.0, 10.0),
            DVec3::new(5.0, 5.0, 10.0),
        ]];
        let st = Stock {
            min: DVec3::new(-6.0, -6.0, 0.0),
            max: DVec3::new(6.0, 6.0, 12.0),
        };
        let ix = MeshIndex::build(&mesh, st);
        let tool = ToolSpec::ball(1, 3.0, 6.0); // 刃长 6、半径 1.5
        assert!(
            shank_hits(&ix, DVec3::new(0.0, 0.0, 0.0), &tool, 0.5),
            "刀杆会穿过 z=10 的板，应判碰撞"
        );
        assert!(
            !shank_hits(&ix, DVec3::new(0.0, 0.0, 20.0), &tool, 0.5),
            "抬到板上方 20 mm 不该判碰撞"
        );
    }

    /// `radius_at` 的四段：刀刃 / 等径避空颈 / 30° 缩颈 / Ø6 柄。
    #[test]
    fn radius_at_has_four_segments_with_a_neck() {
        let t = ToolSpec::ball(1, 1.0, 10.0).with_neck(5.0).with_shank(6.0);
        // 刀刃与避空颈都是刃顶半径 0.5
        assert!((t.radius_at(0.0) - 0.5).abs() < 1e-9);
        assert!((t.radius_at(15.0) - 0.5).abs() < 1e-9, "颈内也是 0.5");
        // 缩颈：15 → 19.33；z=17 处 Δz=2 → 0.5 + 2·tan30° = 1.655
        assert!((t.neck_top() - 15.0).abs() < 1e-9);
        assert!((t.radius_at(17.0) - 1.655).abs() < 0.02);
        assert!((t.radius_at(20.5) - 3.0).abs() < 1e-9);
    }

    /// `radius_at` 的三段：刀刃（恒为刃顶半径）/ 避空缩颈（30° 线性）/ Ø6 柄。
    #[test]
    fn radius_at_has_three_segments() {
        let t = ToolSpec::ball(1, 1.0, 10.0).with_shank(6.0);
        // 刀刃段：任意高度都是刃顶半径 0.5
        assert!((t.radius_at(0.0) - 0.5).abs() < 1e-9);
        assert!((t.radius_at(10.0) - 0.5).abs() < 1e-9);
        // 缩颈：Δr = 2.5，30° → 长 4.330，中点是 1.75
        let taper = t.taper_len();
        assert!((taper - 4.330).abs() < 0.01, "缩颈长度 {taper}");
        assert!((t.radius_at(10.0 + taper * 0.5) - 1.75).abs() < 0.02);
        // 柄：3.0
        assert!((t.radius_at(10.0 + taper + 1.0) - 3.0).abs() < 1e-9);
        // 等径柄（shank_diameter = 0）没有缩颈，全程都是刃顶半径
        let e = ToolSpec::ball(2, 1.0, 10.0);
        assert!(e.taper_len() < 1e-9);
        assert!((e.radius_at(30.0) - 0.5).abs() < 1e-9);
    }

    /// **避空段按该高度的实际半径判碰撞，不是全程按 Ø6。**
    ///
    /// 构造：一小段竖直墙在 x=2、只覆盖 z ∈ [20,21]。Ø1 刀的缩颈在 z=21 处
    /// 半径只有 `0.5 + 1·tan30° = 1.08`，加 0.3 间隙是 1.38 < 2.0 → **不该判碰撞**。
    /// 往上缩颈变粗，但离墙角的距离也按 `√(4+dz²)` 变远，始终够不到（z=24.3 处
    /// 半径 3.0、need 3.3，距离 3.89）。
    /// 若实现拿柄径一刀切（z=21 处就 need 3.3 > 2.0），这里会误报。
    #[test]
    fn clearance_segment_uses_local_radius_not_the_shank() {
        let wall = vec![
            [
                DVec3::new(2.0, -10.0, 20.0),
                DVec3::new(2.0, 10.0, 20.0),
                DVec3::new(2.0, 10.0, 21.0),
            ],
            [
                DVec3::new(2.0, -10.0, 20.0),
                DVec3::new(2.0, 10.0, 21.0),
                DVec3::new(2.0, -10.0, 21.0),
            ],
        ];
        let st = Stock {
            min: DVec3::new(-5.0, -10.0, 0.0),
            max: DVec3::new(2.0, 10.0, 30.0),
        };
        let ix = MeshIndex::build(&wall, st);
        let t = ToolSpec::ball(1, 1.0, 20.0).with_shank(6.0);
        assert!(
            !shank_hits(&ix, DVec3::ZERO, &t, 0.3),
            "避空段在该高度只有 Ø2.2，不该按 Ø6 判碰撞"
        );
        // 同一把刀的柄（z ≥ 24.3）如果伸到墙的高度上，就该判——用一块更高的墙验
        let tall = vec![
            [
                DVec3::new(2.0, -10.0, 20.0),
                DVec3::new(2.0, 10.0, 20.0),
                DVec3::new(2.0, 10.0, 30.0),
            ],
            [
                DVec3::new(2.0, -10.0, 20.0),
                DVec3::new(2.0, 10.0, 30.0),
                DVec3::new(2.0, -10.0, 30.0),
            ],
        ];
        let ix2 = MeshIndex::build(&tall, st);
        assert!(
            shank_hits(&ix2, DVec3::ZERO, &t, 0.3),
            "Ø6 柄在 z≈27 处离墙 2.0 < 3.3，必须判碰撞"
        );
    }

    /// **刀刃段整段不参与碰撞检查**：刀尖埋在料里也不判（那是要切掉的）。
    #[test]
    fn cutting_edge_is_exempt_from_collision() {
        // 一块 z ∈ [0,5] 的薄板，刀尖在 (2,0,2) 埋在板下
        let plate = vec![
            [
                DVec3::new(0.0, -5.0, 5.0),
                DVec3::new(5.0, -5.0, 5.0),
                DVec3::new(5.0, 5.0, 5.0),
            ],
            [
                DVec3::new(0.0, -5.0, 5.0),
                DVec3::new(5.0, 5.0, 5.0),
                DVec3::new(0.0, 5.0, 5.0),
            ],
        ];
        let st = Stock {
            min: DVec3::new(0.0, -5.0, 0.0),
            max: DVec3::new(5.0, 5.0, 5.0),
        };
        let ix = MeshIndex::build(&plate, st);
        // 刃长 20：整块板都在刀刃段（z ≤ 20）以内 → 不该判碰撞
        let t = ToolSpec::ball(1, 1.0, 20.0).with_shank(6.0);
        assert!(
            !shank_hits(&ix, DVec3::new(2.0, 0.0, 2.0), &t, 0.3),
            "刀刃段会直接切掉这块料，不能判成碰撞"
        );
    }

    /// 清根掩膜：**只盖目标点 + 一个刀半径**，远处不覆盖。
    #[test]
    fn rest_mask_covers_targets_plus_one_radius() {
        let st = Stock {
            min: DVec3::ZERO,
            max: DVec3::new(10.0, 10.0, 10.0),
        };
        let m = RestMask::build(&[DVec3::new(5.0, 5.0, 0.0)], None, st, 0.25, 0.5);
        assert!(m.has(5.0, 5.0), "目标点本身必须在掩膜里");
        assert!(m.has(5.45, 5.0), "外扩 R=0.5 内的邻居也要走（球面才够得到）");
        assert!(!m.has(7.0, 5.0), "远处不该覆盖");
        assert!(m.row_has(5.0));
        assert!(!m.row_has(9.0), "整行没目标就该跳过");
        assert!(m.points() >= 9, "外扩后至少 3×3 格：{}", m.points());
    }

    /// 球面闸门：球心离设计面不到 `R − tol` 就判"会啃"。
    ///
    /// 这是清根道的安全阀——高度场只认每列最上面那层，悬伸下面是瞎的。
    #[test]
    fn ball_gate_blocks_a_tool_that_would_eat_a_ceiling() {
        let mesh = vec![[
            DVec3::new(-5.0, -5.0, 10.0),
            DVec3::new(5.0, -5.0, 10.0),
            DVec3::new(5.0, 5.0, 10.0),
        ]];
        let st = Stock {
            min: DVec3::new(-6.0, -6.0, 0.0),
            max: DVec3::new(6.0, 6.0, 12.0),
        };
        let ix = MeshIndex::build(&mesh, st);
        let (r, tol) = (1.5, 0.03);
        // 刀尖 z=8.4 → 球心 9.9，离板 0.1 ≪ R−tol：判"会啃"
        assert!(ball_hits(&ix, DVec3::new(0.0, 0.0, 8.4), r, tol));
        // 刀尖 z=10.0 → 球心 11.5，正好相切（距离 = R）：放行
        assert!(!ball_hits(&ix, DVec3::new(0.0, 0.0, 10.0), r, tol));
    }

    /// `Job.rest` 必须真的把刀路收窄：同一块平板，带 rest 的切长应远小于全表面。
    #[test]
    fn rest_job_only_cuts_near_the_targets() {
        let z = 5.0;
        let p = |x: f64, y: f64| DVec3::new(x, y, z);
        let mesh = vec![
            [p(0.0, 0.0), p(20.0, 0.0), p(20.0, 20.0)],
            [p(0.0, 0.0), p(20.0, 20.0), p(0.0, 20.0)],
        ];
        let stock = Stock {
            min: DVec3::ZERO,
            max: DVec3::new(20.0, 20.0, 10.0),
        };
        let process = Process {
            finish_stepover_mm: 1.0,
            sample_mm: 0.5,
            ..Default::default()
        };
        let mut job = Job {
            mesh: &mesh,
            stock,
            rough: None,
            finish: ToolSpec::ball(1, 2.0, 10.0),
            process,
            rest: None,
        };
        let full = generate_in(&job, None).expect("全表面应能生成");
        let targets = [DVec3::new(5.0, 5.0, z)];
        job.rest = Some(&targets);
        let one = generate_in(&job, None).expect("清根应能生成");
        assert!(
            one.stats.cut_mm < 0.25 * full.stats.cut_mm,
            "清根只该走一小块：{} vs {} mm",
            one.stats.cut_mm,
            full.stats.cut_mm
        );
        assert!(one.stats.cut_mm > 0.0, "目标点附近总要走一点");
    }

    /// 平底刀（立铣刀）在墙边被抬到墙顶：drop-cutter 天然不过切。
    #[test]
    fn flat_tool_lifts_to_the_wall_top() {
        let mut z = vec![0.0; 36];
        for i in 0..6 {
            z[i] = 9.0; // j=0 那一排是墙顶
        }
        let map = map_of(6, 6, 1.0, z);
        // 离墙 1 mm、刀半径 2 → 刀盘压到墙，落刀高度必须是墙顶
        assert!((map.max_within(2.0, 1.0, 2.0) - 9.0).abs() < 1e-9);
        // 离墙 3 mm（> R）就够不着墙了，落刀高度回到地面
        assert!(map.max_within(2.0, 3.0, 2.0).abs() < 1e-9);
    }

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
            rest: None,
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

    /// 平底刀（立铣刀）也能精加工：走 drop-cutter，刀尖落在刀盘内最高点上。
    ///
    /// 这是"立铣刀 + 定轴三轴"的几何基础——球头那套法向偏置对平底刀不成立。
    #[test]
    fn flat_finish_uses_drop_cutter() {
        let m = plane(2.0, 20.0);
        let mut j = job(&m);
        j.finish = ToolSpec::flat(2, 4.0, 20.0);
        let prog = generate(&j).unwrap();
        assert!(prog.gcode.contains("(FINISH T2 D4.0"));
        // 平面上 drop-cutter 就是"刀尖贴面"：精加工段所有 XY 走刀的 Z 都等于 2.0
        // （Z 是模态的，行里不重复写，所以要自己跟）
        let mut z = f64::NAN;
        let mut in_finish = false;
        let mut n = 0usize;
        for line in prog.gcode.lines() {
            if line.contains("(FINISH") {
                in_finish = true;
                continue;
            }
            if !in_finish {
                continue;
            }
            if let Some(v) = line.split(" Z").nth(1) {
                z = v.split_whitespace().next().unwrap().parse().unwrap();
            }
            if line.starts_with("G1 X") {
                assert!(
                    (z - 2.0).abs() < 1e-6,
                    "平底刀精加工刀尖应贴面（z=2.0），实际 {z:.4}：{line}"
                );
                n += 1;
            }
        }
        assert!(n > 10, "没扫到平底刀精加工点：{n}");
    }

    #[test]
    fn descend_ends_exactly_at_target() {
        let l = descend(4.0, 2.2, 1.0);
        assert_eq!(l, vec![3.0, 2.2]);
        let l = descend(3.0, 2.2, 1.0);
        assert_eq!(l, vec![2.2]);
    }
}
