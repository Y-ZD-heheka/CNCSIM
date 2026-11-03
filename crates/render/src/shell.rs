//! Bevy 窗口外壳（`window` feature）。**只做三件事**：开窗、把 `GpuMesh` 交给
//! Bevy、用 egui 画报告与时间轴。所有几何与时间数据都是 app 层算好的——
//! 这里不做任何计算（架构设计 §crate 职责：`render` 不承担计算）。
//!
//! 坐标：模型是 **Z 向上**的机床坐标，Bevy 是 **Y 向上**。整棵场景树用
//! `R_x(-90°)` 把模型旋转进世界：模型 `(x, y, z)` → 世界 `(x, z, -y)`。
//!
//! 默认构建不编译本模块（`window` feature 关闭）——Bevy 的编译税只在
//! 需要窗口时付。

use bevy::asset::RenderAssetUsages;
use bevy::camera_controller::pan_orbit_camera::prelude::PanOrbitCamera;
use bevy::camera_controller::pan_orbit_camera::DefaultPanOrbitCameraPlugins;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::pbr::{Material, MaterialPlugin, StandardMaterial};
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use crate::{PALETTE, SHANK_INDEX};

/// 窗口需要的全部输入。**都是算好的成品**。
pub struct ShellScene {
    /// 可增量重建的"活"毛坯：窗口按回放时间一刀刀往上切。
    pub live: LiveCut,
    pub report: cncsim_sim::SimReport,
    pub playback: cncsim_sim::playback::Playback,
    pub title: String,
    /// HMI 状态。面板的横幅 / 色调 / 报警行都从它派生（经 `cncsim_ui`），
    /// 不在这里各自 match 一遍——否则 `cargo run` 的窗口和 `--export` 的
    /// 预览页会慢慢说两套话。
    pub hmi: cncsim_hmi::Hmi,
    /// 给了就**截一张图写盘然后退出**（无显示器环境下的可视化验证手段）。
    ///
    /// 没有这个开关时，`--window` 只能"跑起来不崩"，看不出画面对不对——
    /// 而"画面对不对"恰恰是渲染层唯一要验证的东西。
    pub screenshot: Option<std::path::PathBuf>,
    /// **刀库**。每把刀画成"刀刃 + 刀杆"两段旋转体，回放按当前块的刀号
    /// 只显示一把——粗加工和精加工换刀时画面上真的换刀。
    ///
    /// 刀刃与刀杆分开画是为了**区分两者**：材质不同色，而且碰撞检测只测
    /// 刃长以上那一段——画出来才对得上"报告说撞了"的位置。
    pub tools: Vec<cncsim_tool::ToolDef>,
}

/// MiSans（小米，个人/商用免费）：egui 自带字体**没有 CJK 字形**，不装它
/// 面板上的中文全是方框。8 MB 嵌进二进制，不依赖运行时路径。
const MI_SANS: &[u8] = include_bytes!("../../../assets/fonts/MiSans-Regular.ttf");

#[derive(Resource, Default)]
struct FontsReady(bool);

/// 首次 egui pass 时把 MiSans 挂到字体族前面；egui 自带字体留作回退
/// （▶ ⏸ 等符号 MiSans 没有，落到后面）。
fn install_fonts(mut contexts: EguiContexts, mut ready: ResMut<FontsReady>) -> Result {
    if ready.0 {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    let mut fonts = egui::FontDefinitions::default();
    fonts
        .font_data
        .insert("misans".to_owned(), egui::FontData::from_static(MI_SANS).into());
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts
            .families
            .entry(family)
            .or_default()
            .insert(0, "misans".to_owned());
    }
    ctx.set_fonts(fonts);
    ready.0 = true;
    Ok(())
}

#[derive(Resource)]
struct Scene(ShellScene);

/// 脏区域：`seek` 的返回值。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Dirty {
    /// 没变
    None,
    /// 只重建这个盒子（mm，世界坐标）
    Region(cncsim_math::DVec3, cncsim_math::DVec3),
    /// 回退了，整块重建
    Full,
}

/// **可增量重建的"活"毛坯**：从空毛坯开始，按回放时间一刀刀往上切。
///
/// 为什么窗口里要这个东西：原来窗口是把**切完的最终网格**提一次、之后只动刀——
/// 材料从头到尾是静态的，看不出"在切"。有了它，播放到哪一刀，材料就少到哪一刀，
/// 而且只重建被这一刀扫过的那几块（`TileCache`），不是每次全量重提。
///
/// 回退（拖时间轴往回）走 `Full`：把毛坯重新填满再切到目标刀。毛坯填充 + 几刀切削
/// 在 30 mm 演示里是几毫秒，比维护"每刀一份快照"划算得多（后者内存按刀数线性涨）。
pub struct LiveCut {
    /// 当前状态
    pub grid: cncsim_voxel::SparseGrid<i8>,
    config: cncsim_voxel::GridConfig,
    origin: cncsim_math::DVec3,
    size: [i32; 3],
    /// 刀库：刀号 → 切削部分母线。**按刀号查而不是按顺序取**——程序里
    /// `T1` 打完 `T2` 再回 `T1` 是合法的，按下标就会切错刀。
    cutters: Vec<(u16, cncsim_tool::Profile)>,
    /// 每一刀：`(起始时刻, 起点位姿, 终点位姿, 刀号)`，按起始时刻升序
    cuts: Vec<(f64, cncsim_motion::Pose, cncsim_motion::Pose, u16)>,
    /// 每一刀的 `(刀号, 工序内序号, 工序内总数)` —— 进度着色用。
    /// **在 `new` 里一次算好**：回放时每帧都要取，现算会和仿真侧算出不同的段。
    phase: Vec<(u16, u32, u32)>,
    /// 已经应用了几刀
    pub applied: usize,
    /// 分块网格缓存
    pub cache: crate::TileCache,
}

impl LiveCut {
    /// `cfg` 直接吃 `RunConfig`：毛坯尺寸、原点、网格配置、**溯源粒度**
    /// 全部从同一个来源取。
    ///
    /// 原来这里是 8 个位置参数，`creator_sub` 是最后加的一个——忘了传，
    /// 窗口就按"每叶一个标记"（0.8 mm）着色，而导出/报告按 4³ 子块（0.2 mm），
    /// 表现为"切色溢出 0.1 → 0.3 mm"。这类"两份配置各写一遍"的错误
    /// 只能靠**消灭第二个来源**来防，不能靠记性。
    pub fn new(
        cfg: &cncsim_sim::run::RunConfig,
        cutters: Vec<(u16, cncsim_tool::Profile)>,
        mut cuts: Vec<(f64, cncsim_motion::Pose, cncsim_motion::Pose, u16)>,
        tile_mm: f64,
        k: u32,
    ) -> Self {
        let (config, origin, size, creator_sub) =
            (cfg.grid, cfg.stock_origin, cfg.stock_size, cfg.creator_sub);
        debug_assert!(!cutters.is_empty(), "刀库不能为空");
        cuts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // 每个刀号（工序）各有多少刀：粗加工、精加工各自 0→100%
        let mut total_by_tool: std::collections::BTreeMap<u16, u32> = Default::default();
        for (_, _, _, tool) in &cuts {
            *total_by_tool.entry(*tool).or_insert(0) += 1;
        }
        let mut done_by_tool: std::collections::BTreeMap<u16, u32> = Default::default();
        let phase: Vec<(u16, u32, u32)> = cuts
            .iter()
            .map(|(_, _, _, tool)| {
                let d = done_by_tool.entry(*tool).or_insert(0);
                let r = (*tool, *d, total_by_tool[tool]);
                *d += 1;
                r
            })
            .collect();
        let mut grid = cncsim_voxel::SparseGrid::<i8>::new(config, origin);
        grid.set_creator_sub(creator_sub);
        grid.fill_box_exact([0, 0, 0], size, config.band_mm());
        if std::env::var_os("CNCSIM_DBG").is_some() {
            eprintln!("[live] cuts={} phase[0..3]={:?} phase[end-3..]={:?}",
                cuts.len(), &phase[..3.min(phase.len())],
                &phase[phase.len().saturating_sub(3)..]);
            let mut by_tool: std::collections::BTreeMap<u16, usize> = Default::default();
            for (_, _, _, t) in &cuts { *by_tool.entry(*t).or_insert(0) += 1; }
            eprintln!("[live] 每把刀的切段数 {by_tool:?}");
        }
        Self {
            grid,
            config,
            origin,
            size,
            cutters,
            cuts,
            phase,
            applied: 0,
            cache: crate::TileCache::new(tile_mm, k),
        }
    }

    /// 刀号 → 母线下标；查不到退回 0（`run` 对缺刀号也是这个口径）。
    fn cutter_index(&self, tool: u16) -> usize {
        self.cutters.iter().position(|(n, _)| *n == tool).unwrap_or(0)
    }

    /// 时刻 `t` 应该已经切完几刀。
    pub fn target_at(&self, t: f64) -> usize {
        self.cuts.partition_point(|(t0, ..)| *t0 <= t)
    }

    pub fn cut_count(&self) -> usize {
        self.cuts.len()
    }

    /// 调试用：读某个体素的 creator（对比仿真与回放的染色是否一致）。
    pub fn creator_at(&self, p: [i32; 3]) -> u8 {
        self.grid.creator_of_voxel(p)
    }

    /// 第 `cut` 刀的 `(刀号, 工序内序号, 工序内总数)`。着色和回放共用这一份。
    pub fn progress_at(&self, cut: u32) -> (u16, u32, u32) {
        self.phase.get(cut as usize).copied().unwrap_or((0, 0, 1))
    }

    /// 把状态推进到第 `target` 刀，返回脏区域。
    ///
    /// 切削顺序必须与仿真里**完全一致**（同一份 `cut_segs`），否则刀痕和
    /// 溯源色会对不上。
    pub fn seek(&mut self, target: usize) -> Dirty {
        let target = target.min(self.cuts.len());
        if target == self.applied {
            return Dirty::None;
        }
        let backward = target < self.applied;
        let from = if backward { 0 } else { self.applied };
        if backward {
            let mut g = cncsim_voxel::SparseGrid::<i8>::new(self.config, self.origin);
            g.fill_box_exact([0, 0, 0], self.size, self.config.band_mm());
            self.grid = g;
        }
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for i in from..target {
            let (_, a, b, tool) = &self.cuts[i];
            // 与仿真同一条索引规则：第 i 刀盖第 i 个颜色
            let (tool_no, done, total) = self.phase[i];
            self.grid.set_creator(cncsim_voxel::creator_of_tool_segment(tool_no, done, total));
            let ci = self.cutter_index(*tool);
            self.grid.cut_segment(a, b, &self.cutters[ci].1);
            for p in [a.tip, b.tip] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p.component(k));
                    hi[k] = hi[k].max(p.component(k));
                }
            }
        }
        self.applied = target;
        if backward {
            Dirty::Full
        } else {
            // 外扩：**最粗那把刀**的半径 + 窄带 + 1 mm 余量（圆弧离散的弦高
            // 落在里面）。取最大值而不是当前刀的：脏区变大只是多重建几块，
            // 取小了会漏重建——那是一条看得见的残影。
            let pad = self
                .cutters
                .iter()
                .map(|(_, p)| p.max_radius())
                .fold(0.0f64, f64::max)
                + self.config.band_mm()
                + 1.0;
            Dirty::Region(
                cncsim_math::DVec3::new(lo[0] - pad, lo[1] - pad, lo[2] - pad),
                cncsim_math::DVec3::new(hi[0] + pad, hi[1] + pad, hi[2] + pad),
            )
        }
    }

    /// 毛坯的模型空间包围盒（mm）——相机取景与"把模型搬到原点"都用它。
    fn bounds_mm(&self) -> (cncsim_math::DVec3, cncsim_math::DVec3) {
        let h = self.config.h;
        let lo = self.origin;
        let hi = self.origin
            + cncsim_math::DVec3::new(
                self.size[0] as f64 * h,
                self.size[1] as f64 * h,
                self.size[2] as f64 * h,
            );
        (lo, hi)
    }
}

/// 刀具实体（回放时跟着位姿走）。
///
/// 刀具是"一个骨架 + 两段网格"：`ToolRig` 只带变换，刀刃与刀柄是它的子实体，
/// 各自在局部原点建模（母线本来就是从刀尖往上量的）。这样每帧只改一个
/// `Transform`，加第三段（夹头）也不用再动回放代码。
#[derive(Component)]
struct ToolRig(u16);

/// 刀具材质句柄（每把刀一份）。刀刃的颜色**每帧跟着当前块变**，
/// 要能就地改 uniform。
#[derive(Resource)]
struct ToolMats {
    flutes: Vec<(u16, Handle<CutMaterial>)>,
}



/// 母线 → 旋转体网格（绕局部 **+Z** 轴）。
///
/// 母线首尾都落在轴上（`Profile::new` 断言过），所以两个端面自动收成一点，
/// **不需要额外的封盖**——但这也意味着母线**端点**所在的四边形会塌成三角形
/// （轴上那两个角点重合）。所以按端点是否在轴上分三种情况发三角形：
/// 否则会多出面积为 0 的退化三角形，`compute_normals` 给它们算出
/// **零法向**（实测：法向 `(0,0,0)`，光照下是一块黑斑）。
///
/// 法向**解析给出**，不走 `compute_normals`：每个四边形自带 4 个顶点，
/// 逐面法向会让圆柱看起来是 48 条棱的多面体。解析法向在 (r, z) 平面里垂直于
/// 母线切向（`(dr, dz)` 顺时针转 90° → `(dz, -dr)`），再绕轴旋到各角度，
/// 圆柱段就**完全光滑**；同时按折痕角（30°）决定要不要在母线拐点处平均，
/// 平底刀的底面与圆柱之间那圈 90° 硬边得以保留。
///
/// 绕序按"外表面朝外"排：`[b, b+1, b+2]` / `[b+1, b+3, b+2]`。
/// 反过来的话整个刀具会**消失**（背面剔除，不是变黑）。
fn profile_mesh(profile: &cncsim_tool::Profile, segs: usize) -> Mesh {
    use std::f64::consts::TAU;
    let pts = profile.points();
    let segs = segs.max(3);
    let nseg = pts.len() - 1;

    // 每段母线在 (r, z) 平面里的外向单位法向
    let mut seg_n = Vec::with_capacity(nseg);
    for w in pts.windows(2) {
        let (dr, dz) = (w[1].x - w[0].x, w[1].y - w[0].y);
        let len = (dr * dr + dz * dz).sqrt();
        if len < 1e-12 {
            seg_n.push((0.0, 0.0));
        } else {
            seg_n.push((dz / len, -dr / len));
        }
    }
    // 折痕判据：相邻两段法向夹角小于 30° 才平均（90° 的端面硬边不平均）
    const CREASE_COS: f64 = 0.866_025_403_784_438_6;
    let mut nrm: Vec<[f32; 3]> = Vec::with_capacity(segs * pts.len() * 4);
    let mut pos: Vec<[f32; 3]> = Vec::with_capacity(segs * pts.len() * 4);
    let mut idx: Vec<u32> = Vec::new();

    for i in 0..segs {
        let a0 = (i as f64) / (segs as f64) * TAU;
        let a1 = ((i + 1) as f64) / (segs as f64) * TAU;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        for (si, w) in pts.windows(2).enumerate() {
            let (r0, z0) = (w[0].x, w[0].y);
            let (r1, z1) = (w[1].x, w[1].y);
            let (nr, nz) = seg_n[si];
            // 段起点 / 终点的法向：与相邻段夹角小就平均（光滑过渡），否则保持本段
            let mut n_start = (nr, nz);
            if si > 0 && (seg_n[si - 1].0 * nr + seg_n[si - 1].1 * nz) > CREASE_COS {
                let (ar, az) = (seg_n[si - 1].0 + nr, seg_n[si - 1].1 + nz);
                let l = (ar * ar + az * az).sqrt();
                if l > 1e-12 {
                    n_start = (ar / l, az / l);
                }
            }
            let mut n_end = (nr, nz);
            if si + 1 < nseg && (seg_n[si + 1].0 * nr + seg_n[si + 1].1 * nz) > CREASE_COS {
                let (ar, az) = (seg_n[si + 1].0 + nr, seg_n[si + 1].1 + nz);
                let l = (ar * ar + az * az).sqrt();
                if l > 1e-12 {
                    n_end = (ar / l, az / l);
                }
            }

            let a = [(r0 * c0) as f32, (r0 * s0) as f32, z0 as f32];
            let b_ = [(r0 * c1) as f32, (r0 * s1) as f32, z0 as f32];
            let c = [(r1 * c0) as f32, (r1 * s0) as f32, z1 as f32];
            let d = [(r1 * c1) as f32, (r1 * s1) as f32, z1 as f32];
            let na = [(n_start.0 * c0) as f32, (n_start.0 * s0) as f32, n_start.1 as f32];
            let nb = [(n_start.0 * c1) as f32, (n_start.0 * s1) as f32, n_start.1 as f32];
            let nc = [(n_end.0 * c0) as f32, (n_end.0 * s0) as f32, n_end.1 as f32];
            let nd = [(n_end.0 * c1) as f32, (n_end.0 * s1) as f32, n_end.1 as f32];
            let base = pos.len() as u32;
            // 端点在轴上 → 该端两个角点重合，只发一个三角形。
            if r0 == 0.0 {
                pos.extend_from_slice(&[b_, c, d]);
                nrm.extend_from_slice(&[nb, nc, nd]);
                idx.extend_from_slice(&[base, base + 2, base + 1]);
            } else if r1 == 0.0 {
                pos.extend_from_slice(&[a, b_, c]);
                nrm.extend_from_slice(&[na, nb, nc]);
                idx.extend_from_slice(&[base, base + 1, base + 2]);
            } else {
                pos.extend_from_slice(&[a, b_, c, d]);
                nrm.extend_from_slice(&[na, nb, nc, nd]);
                idx.extend_from_slice(&[base, base + 1, base + 2, base + 1, base + 3, base + 2]);
            }
        }
    }
    // `CutMaterial` 的着色器固定读 `@location(5)`（顶点色），网格没有这个属性
    // 会导致**管线创建失败**（日志一条 ERROR，网格静默不渲染）。刀具的索引来自
    // 材质 uniform 而不是顶点色，这里只是把属性补齐。
    let color = vec![[0.0f32, 0.0, 0.0, 1.0]; pos.len()];
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, nrm)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, color)
        .with_inserted_indices(Indices::U32(idx))
}

/// 模型方向（Z-up）→ 世界方向（Y-up）。与 `z_up_to_y_up` 同一个旋转。
fn to_world_dir(d: cncsim_math::DVec3) -> Vec3 {
    Vec3::new(d.x as f32, d.z as f32, -(d.y as f32))
}

/// 相机距离倍率（环境变量 `CNCSIM_CAM_SCALE`，默认 1.0）。
///
/// 存在的理由很具体：φ6 的刀在 300 mm 的毛坯里只占屏幕高度的约 1%，
/// `--screenshot` 出来的整机视角里几乎认不出"刀画没画出来、朝向对不对"。
/// 拉近是唯一能看见它的办法，而相机距离原本是写死的。
fn cam_scale() -> f32 {
    std::env::var("CNCSIM_CAM_SCALE")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
        .filter(|s| *s > 0.0 && s.is_finite())
        .unwrap_or(1.0)
}

/// 是否隐藏刀具（`CNCSIM_HIDE_TOOL=1`）。
///
/// 看**切面本身**的时候刀会挡住最想看的区域，而且刀是场景里唯一会动的东西，
/// 会让两次截图没法逐像素比对。检查切面时就把它关掉。
fn hide_tool() -> bool {
    matches!(
        std::env::var("CNCSIM_HIDE_TOOL").as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

/// 固定成沿模型某个轴的**正视图**（`CNCSIM_VIEW=x|y|z|-x|-y|-z`）。
///
/// 返回 `(相机方向, 屏幕上方)`，都是**世界**坐标（Y-up）。
/// 给了这个就**不挂 `PanOrbitCamera`**——轨道相机会按自己的 yaw/pitch
/// 重算位置，把 `Transform` 覆盖掉。检查用途要的是精确的轴向视图，不是可拖动的。
///
/// 模型 Z-up → 世界 Y-up 的映射是 `(x, y, z) → (x, z, −y)`：
/// 模型 +X → 世界 +X，模型 +Y → 世界 −Z，模型 +Z（向上）→ 世界 +Y。
fn fixed_view() -> Option<(Vec3, Vec3)> {
    let s = std::env::var("CNCSIM_VIEW").ok()?;
    match s.trim().to_ascii_lowercase().as_str() {
        // 沿 +X 看：屏幕上方取模型 +Z（世界 +Y）
        "x" => Some((Vec3::X, Vec3::Y)),
        "-x" => Some((-Vec3::X, Vec3::Y)),
        // 沿 +Y 看（模型 +Y = 世界 −Z）
        "y" => Some((-Vec3::Z, Vec3::Y)),
        "-y" => Some((Vec3::Z, Vec3::Y)),
        // 俯视：模型 +Z = 世界 +Y；屏幕上方取模型 +Y（世界 −Z）
        "z" => Some((Vec3::Y, -Vec3::Z)),
        "-z" => Some((-Vec3::Y, -Vec3::Z)),
        _ => None,
    }
}

/// 截图计划：等 `WAIT` 帧让画面稳定（首次渲染要编译管线、传网格），
/// 截一张，再等 `GRACE` 帧让异步写盘完成，然后退出。
#[derive(Resource)]
struct ShotPlan {
    path: std::path::PathBuf,
    frame: u32,
    fired: bool,
    after: u32,
}

const SHOT_WAIT: u32 = 30;
const SHOT_GRACE: u32 = 120;

fn shoot(
    mut commands: Commands,
    plan: Option<ResMut<ShotPlan>>,
    mut exit: MessageWriter<bevy::app::AppExit>,
) {
    let Some(mut plan) = plan else {
        return;
    };
    plan.frame += 1;
    if !plan.fired && plan.frame >= SHOT_WAIT {
        plan.fired = true;
        if let Some(dir) = plan.path.parent()
            && !dir.as_os_str().is_empty()
        {
            let _ = std::fs::create_dir_all(dir);
        }
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(plan.path.clone()));
        println!("截图 → {}（第 {} 帧）", plan.path.display(), plan.frame);
    }
    if plan.fired {
        plan.after += 1;
        if plan.after > SHOT_GRACE {
            exit.write(bevy::app::AppExit::Success);
        }
    }
}

#[derive(Resource)]
struct UiState {
    t: f64,
    playing: bool,
    speed: f64,
}

impl Default for UiState {
    fn default() -> Self {
        Self { t: start_time(), playing: false, speed: 4.0 }
    }
}

/// 起始回放时刻（环境变量 `CNCSIM_T`，默认 0）。
///
/// 和 [`cam_scale`] 一样是为 `--screenshot` 服务的：截图永远停在 `t=0`
/// 的话，只能看到"程序开头刀具悬在原点上方"，看不到它进刀、也看不到
/// 五轴摆角下刀轴对不对。**不播放**是刻意的——播放时截到哪一帧取决于
/// 帧率，两次截图没法对比。
fn start_time() -> f64 {
    std::env::var("CNCSIM_T")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|t| t.is_finite() && *t >= 0.0)
        .unwrap_or(0.0)
}

/// 工具标记（跟着回放走的球）
#[derive(Component)]
struct ToolMarker;

/// 模型 Z-up → 世界 Y-up 的旋转。
fn z_up_to_y_up() -> Quat {
    Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
}

/// 模型坐标 → 世界坐标（与 [`z_up_to_y_up`] 一致）。
fn to_world_model(p: cncsim_math::DVec3) -> Vec3 {
    Vec3::new(p.x as f32, p.z as f32, -p.y as f32)
}

/// 打开窗口并跑事件循环。**会阻塞直到窗口关闭**。
pub fn run_window(scene: ShellScene) {
    let title = format!("{} — 五轴联动 CNC 切削仿真", scene.title);
    let shot = scene.screenshot.clone();
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(0.08, 0.09, 0.11)))
        .insert_resource(Scene(scene))
        .init_resource::<UiState>()
        .init_resource::<FontsReady>()
        // **`assets/` 在项目根，不在 crate 目录**。Bevy 的 `AssetPlugin` 默认按
        // 可执行文件所在目录找 `assets/`（`cargo run` 时是 `crates/app/assets/`），
        // 所以自定义着色器会报 `Path not found`——而且**只打一条 ERROR 日志，
        // 网格静默不渲染**，画面看起来像"材质全黑"（实测踩过）。
        .add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets")
                        .to_string(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window { title, ..default() }),
                    ..default()
                }),
        )
        .add_plugins(EguiPlugin::default())
        .add_plugins(MaterialPlugin::<CutMaterial>::default())
        // 0.20 的轨道相机是独立 crate/feature：插件组 + 组件（见 bevy 文档）
        // 0.20 的轨道相机是独立 crate/feature：插件组 + 组件（见 bevy 文档）。
        // picking 由 DefaultPlugins 自带——重复加会 panic。
        .add_plugins(DefaultPanOrbitCameraPlugins)
        .add_systems(Startup, setup)
        .add_systems(EguiPrimaryContextPass, (install_fonts, ui_panel).chain())
        .add_systems(Update, (advance_playback, apply_cuts).chain());
    if let Some(path) = shot {
        app.insert_resource(ShotPlan {
            path,
            frame: 0,
            fired: false,
            after: 0,
        })
        .add_systems(Update, shoot);
    }
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut cut_mats: ResMut<Assets<CutMaterial>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scene: ResMut<Scene>,
) {
    // ── 毛坯：**按块**挂载（`TileCache`），不是一整块 ──
    //
    // 一块一个实体有两个理由：① 增量重建时只换被切到的那几块；② 相机/背面剔除
    // 能按块剔，大模型上省不少。
    let material = cut_mats.add(CutMaterial::default());
    let (lo_m, hi_m) = scene.0.live.bounds_mm();
    let center_model = (lo_m + hi_m) * 0.5;
    let diag = ((hi_m - lo_m).length() as f32).max(1.0);
    // 模型中心转到世界（Z-up → Y-up）
    let center = Vec3::new(center_model.x as f32, center_model.z as f32, -center_model.y as f32);
    let root = commands
        .spawn((
            Transform::from_translation(-center),
            Visibility::default(),
            Name::new("model root"),
        ))
        .id();

    // 首次全量建块
    let mut map: std::collections::HashMap<u64, (Entity, Handle<Mesh>)> =
        std::collections::HashMap::new();
    let first = {
        let lc = &mut scene.0.live;
        let LiveCut { cache, grid, .. } = lc;
        cache.update(grid, None, usize::MAX)
    };
    for up in first.uploads {
        let handle = meshes.add(tile_mesh(&up.mesh));
        let e = commands
            .spawn((
                Mesh3d(handle.clone()),
                MeshMaterial3d(material.clone()),
                Transform::from_rotation(z_up_to_y_up()),
                ChildOf(root),
                Name::new("tile"),
            ))
            .id();
        map.insert(up.tile_id, (e, handle));
    }
    commands.insert_resource(TileMap { map, root, material });

    // ── 灯光：主光 + 补光（不引环境光 API，两个方向光够用）──
    commands.spawn((
        DirectionalLight { illuminance: 9_000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.85, -0.55, 0.0)),
        Name::new("key light"),
    ));
    commands.spawn((
        DirectionalLight { illuminance: 2_500.0, shadow_maps_enabled: false, ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.4, 2.6, 0.0)),
        Name::new("fill light"),
    ));

    // ── 相机：绕**原点**（模型已被 root 搬到原点）+ 轨道控制 ──
    let r = (diag * 1.5 + 10.0) * cam_scale();
    match fixed_view() {
        // 检查用：固定轴向视图，**不挂轨道相机**（它会按自己的 yaw/pitch 重算位置）
        Some((dir, up)) => {
            commands.spawn((
                Camera3d::default(),
                Transform::from_translation(dir.normalize() * r).looking_at(Vec3::ZERO, up),
                Name::new("camera (fixed view)"),
            ));
        }
        None => {
            commands.spawn((
                Camera3d::default(),
                Transform::from_translation(Vec3::new(0.9, 0.7, 0.9).normalize() * r)
                    .looking_at(Vec3::ZERO, Vec3::Y),
                PanOrbitCamera::default().with_initial_anchor_depth(-(r as f64)),
                Name::new("camera"),
            ));
        }
    }

    // ── 刀具：母线绕局部 +Z 旋成的旋转体，回放时跟着位姿走 ──
    //
    // 网格原点是**刀尖**（profile 首点 `(0,0)`），所以平移直接给刀尖、
    // 旋转只负责把局部 +Z 对到刀轴上——不需要额外的偏移量。
    // 刀柄的母线本来就是从刃长顶部（z = flute_length）开始量的，所以两段
    // 网格共用同一个局部原点，叠在一起正好接上。
    let show_tool = !hide_tool();
    // **每把刀一份刀刃材质**：颜色是逐帧改 material 的，共用一份就没法
    // 只给"现在这把"上色（换刀瞬间会两把同时变）。
    let flutes: Vec<(u16, Handle<CutMaterial>)> = scene
        .0
        .tools
        .iter()
        .map(|t| {
            let m = cut_mats.add(CutMaterial {
                // 0 = 用材质 `index` 直接查表（刀刃和刀柄都走这条）；
                // 1 = 用顶点色 R 通道的色号（毛坯网格）。
                use_vertex: 0,
                index: 1, // 第 1 把刀的第 1 段
                ..CutMaterial::default()
            });
            (t.number, m)
        })
        .collect();
    let shank_mat = cut_mats.add(CutMaterial { use_vertex: 0, index: SHANK_INDEX, ..default() });
    commands.insert_resource(ToolMats { flutes: flutes.clone() });

    if show_tool {
        for (i, t) in scene.0.tools.iter().enumerate() {
            // 每个刀号一个骨架；`advance_playback` 按当前块只让一个可见。
            let rig = commands
                .spawn((
                    Transform::IDENTITY,
                    if i == 0 { Visibility::Visible } else { Visibility::Hidden },
                    ChildOf(root),
                    ToolRig(t.number),
                    Name::new(format!("tool T{}", t.number)),
                ))
                .id();
            commands.spawn((
                Mesh3d(meshes.add(profile_mesh(&t.cutter, 48))),
                MeshMaterial3d(flutes[i].1.clone()),
                Transform::IDENTITY,
                ChildOf(rig),
                Name::new("flute"),
            ));
            commands.spawn((
                Mesh3d(meshes.add(profile_mesh(&t.shank, 48))),
                MeshMaterial3d(shank_mat.clone()),
                Transform::IDENTITY,
                ChildOf(rig),
                Name::new("shank"),
            ));
        }

        // ── 工具标记：刀尖那个亮点 ──
        //
        // 半径**固定** 0.6 mm，不随 `diag` 缩放：早先按 `diag * 0.012 + 0.3`
        // 算出来是 5.6 mm，比 φ6 的刀还大，正好把要看的刀尖整个盖住。
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.95, 0.55, 0.15),
                emissive: Color::srgb(0.6, 0.25, 0.0).into(),
                ..default()
            })),
            // 初始放在模型中心（root 的原点）；回放时 `advance_playback` 会改成
            // `to_world_model(tip)` —— 那是**世界坐标**，父实体再减 `center`，对得上。
            Transform::from_translation(Vec3::ZERO),
            ChildOf(root),
            ToolMarker,
            Name::new("tool marker"),
        ));
    }
}

/// 分块实体表：`tile_id → (实体, 网格句柄)`。
///
/// 存句柄是为了**就地换网格**（`meshes.get_mut`）而不是每次新增一个——
/// 后者每更新一次就泄漏一份旧的 `Mesh` 资产。
#[derive(Resource)]
struct TileMap {
    map: std::collections::HashMap<u64, (Entity, Handle<Mesh>)>,
    root: Entity,
    material: Handle<CutMaterial>,
}

/// `MeshData` → Bevy 网格（位置 + 法向 + 顶点色）。
///
/// 顶点色必须给：`CutMaterial` 的着色器无条件声明了 `@location(5)`，
/// 网格缺这个属性会让**管线创建失败**，而且只打一条 ERROR、网格静默不渲染。
fn tile_mesh(src: &cncsim_voxel::MeshData) -> Mesh {
    // **拆顶点只在这里做**，不在 `voxel` 的提取里：那是纯显示需求（把 90° 棱做成
    // 硬边，消掉棱上的暗线），而提取出来的网格还要给水密性测试、体积/面积统计用——
    // 拆完索引级就不共享边了，那些判据会失效。
    let mut owned;
    let m = {
        owned = src.clone();
        owned.split_crease_normals(0.707);
        // 拆完硬边再按**面法线面积加权**平滑：每个顶点只累加同一侧的面，
        // 棱角不会被抹圆。提取时的法线是 SDF 粗格差分，带格子尺度的噪声
        // （R15 球面 k=3 时 1.46°），光照放大成一圈圈波纹。
        //
        // 分块重建时每个 tile 各算各的：`extract_surface_lod_in` 的 region
        // 多取了一圈 halo，边界顶点看到的是相邻 tile 的面，接缝可以忽略。
        owned.recompute_smooth_normals();
        &owned
    };
    let n = m.positions.len();
    let colors: Vec<[f32; 4]> = if m.colors.len() == n {
        m.colors
            .iter()
            .map(|c| {
                [
                    c[0] as f32 / 255.0,
                    c[1] as f32 / 255.0,
                    c[2] as f32 / 255.0,
                    c[3] as f32 / 255.0,
                ]
            })
            .collect()
    } else {
        vec![[0.0, 0.0, 0.0, 1.0]; n]
    };
    let normals = if m.normals.len() == n {
        m.normals.clone()
    } else {
        vec![[0.0, 0.0, 1.0]; n]
    };
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, m.positions.clone())
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
        .with_inserted_indices(Indices::U32(m.indices.clone()))
}

/// 把毛坯状态推进到当前回放时刻：切到哪一刀，材料就少到哪一刀。
///
/// 只有"跨过某一刀"时才动网格——每帧都重建会让播放变成幻灯片。
/// 增量范围由 `LiveCut::seek` 给出（前进出脏盒，回退走全量）。
fn apply_cuts(
    mut scene: ResMut<Scene>,
    ui: Res<UiState>,
    mut tiles: ResMut<TileMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut commands: Commands,
) {
    let t_all = std::time::Instant::now();
    let target = scene.0.live.target_at(ui.t);
    if target == scene.0.live.applied {
        return;
    }
    let dirty = scene.0.live.seek(target);
    // `cache` 与 `grid` 是同一个结构体的两个字段，必须**分开借用**——
    // 直接写 `scene.0.live.cache.update(&scene.0.live.grid, ..)` 会被借用检查器拦
    let t_c = std::time::Instant::now();
    let res = {
        let lc = &mut scene.0.live;
        let LiveCut { cache, grid, .. } = lc;
        match dirty {
            Dirty::None => return,
            Dirty::Full => cache.update(grid, None, usize::MAX),
            Dirty::Region(lo, hi) => cache.update(grid, Some((lo, hi)), usize::MAX),
        }
    };
    let t_cache = t_c.elapsed().as_secs_f64() * 1e3;
    let n_up = res.uploads.len();
    for up in res.uploads {
        let existing = tiles.map.get(&up.tile_id).map(|(_, h)| h.clone());
        match existing {
            // 就地换网格：句柄不变，Bevy 只重传这一个资产的 GPU 缓冲
            Some(h) => {
                if let Some(mut slot) = meshes.get_mut(&h) {
                    *slot = tile_mesh(&up.mesh);
                }
            }
            None => {
                let h = meshes.add(tile_mesh(&up.mesh));
                let e = commands
                    .spawn((
                        Mesh3d(h.clone()),
                        MeshMaterial3d(tiles.material.clone()),
                        Transform::from_rotation(z_up_to_y_up()),
                        ChildOf(tiles.root),
                        Name::new("tile"),
                    ))
                    .id();
                tiles.map.insert(up.tile_id, (e, h));
            }
        }
    }
    if std::env::var_os("CNCSIM_PROF").is_some() {
        println!(
            "  [tiles] 第 {} 刀：重建 {} 块 / 上传 {} / 删除 {}；分块提取 {t_cache:.2} ms，合计 {:.2} ms",
            scene.0.live.applied,
            res.rebuilt,
            n_up,
            res.removed,
            t_all.elapsed().as_secs_f64() * 1e3
        );
    }
    if res.removed > 0 {
        let live: std::collections::HashSet<u64> = scene.0.live.cache.tile_ids().collect();
        let dead: Vec<u64> = tiles.map.keys().copied().filter(|id| !live.contains(id)).collect();
        for id in dead {
            if let Some((e, _)) = tiles.map.remove(&id) {
                commands.entity(e).despawn();
            }
        }
    }
}

/// 回放推进：播放时按倍率走时间，把刀具与刀尖标记放到插值出的位姿上，
/// 并把刀刃换成**当前这一块**的颜色。
///
/// 三个查询都带 `Without` 互斥过滤器：它们都写 `Transform`，没有过滤器时
/// Bevy 在系统初始化阶段就会因访问冲突 panic（不是运行到才炸）。
#[allow(clippy::too_many_arguments)]
fn advance_playback(
    time: Res<Time>,
    mut ui: ResMut<UiState>,
    scene: Res<Scene>,
    mats: Res<ToolMats>,
    mut cut_mats: ResMut<Assets<CutMaterial>>,
    mut marker: Query<&mut Transform, (With<ToolMarker>, Without<ToolRig>)>,
    mut rig: Query<(&mut Transform, &mut Visibility, &ToolRig), Without<ToolMarker>>,
) {
    let total = scene.0.playback.total_time();
    if ui.playing {
        ui.t += time.delta_secs() as f64 * ui.speed;
        if ui.t >= total {
            ui.t = total;
            ui.playing = false;
        }
    }
    let Some(frame) = scene.0.playback.at(ui.t) else {
        return;
    };
    let tip = to_world_model(frame.pose.tip);
    if let Ok(mut t) = marker.single_mut() {
        t.translation = tip;
    }
    for (mut t, mut vis, tool) in &mut rig {
        // 换刀：只显示"这一块用的那一把"。几何上不动，只切可见性——
        // 重建网格会卡帧，而换刀本来就是一瞬间的事。
        let active = tool.0 == frame.tool;
        *vis = if active { Visibility::Visible } else { Visibility::Hidden };
        if !active {
            continue;
        }
        t.translation = tip;
        // 局部 +Z 是"刀尖指向主轴"的方向，正是 `pose.axis` 的语义。
        // `from_rotation_arc` 给的是**最小旋转**；旋转体绕自身轴任意转都不变，
        // 所以不必在意绕轴的剩余自由度。
        if let Some(axis) = to_world_dir(frame.pose.axis).try_normalize() {
            t.rotation = Quat::from_rotation_arc(Vec3::Z, axis);
        }
    }
    // 刀刃颜色 = 当前进度档的色。**与切面用的是同一个函数**
    // （`cncsim_voxel::creator_of_progress`），所以刀切下去的那一刻，
    // 刀刃和它刚切出来的面是同一个颜色。
    let (tool_no, done, total) = scene.0.live.progress_at(frame.cut);
    let want = cncsim_voxel::creator_of_tool_segment(tool_no, done, total) as u32;
    for (n, h) in &mats.flutes {
        if *n != frame.tool {
            continue;
        }
        if let Some(mut m) = cut_mats.get_mut(h)
            && m.index != want
        {
            m.index = want;
        }
    }
}

/// `Tone` → egui 颜色。**映射只有这一处**，`cncsim_ui` 只给语义不给颜色。
fn tone_color(t: cncsim_ui::Tone) -> egui::Color32 {
    use cncsim_ui::Tone::*;
    match t {
        Neutral => egui::Color32::from_rgb(190, 190, 190),
        Active => egui::Color32::from_rgb(120, 200, 120),
        Attention => egui::Color32::from_rgb(230, 190, 90),
        Fatal => egui::Color32::from_rgb(220, 90, 70),
    }
}

fn ui_panel(mut contexts: EguiContexts, mut ui: ResMut<UiState>, scene: Res<Scene>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let r = &scene.0.report;
    let view = cncsim_ui::UiView { hmi: &scene.0.hmi };
    egui::Window::new("报告").default_pos([12.0, 12.0]).resizable(false).show(ctx, |u| {
        // 横幅：一行说清「现在能不能继续」
        u.colored_label(
            tone_color(view.status_tone()),
            format!("● {}", view.status_banner()),
        );
        u.separator();
        u.label(format!("程序时间 {:.2} s", r.programmed_time_s));
        u.label(format!("去料 {:.1} mm³", r.removed_volume_mm3));
        if r.gouge_max_depth_mm > 0.0 {
            u.colored_label(
                egui::Color32::from_rgb(220, 90, 70),
                format!("过切 {:.2} mm³ / 最深 {:.3} mm", r.gouge_volume_mm3, r.gouge_max_depth_mm),
            );
        } else {
            u.colored_label(egui::Color32::from_rgb(120, 200, 120), "无过切");
        }
        if r.residual_volume_mm3 > 0.0 {
            u.colored_label(
                egui::Color32::from_rgb(90, 160, 220),
                format!("残留 {:.2} mm³ / 最厚 {:.3} mm", r.residual_volume_mm3, r.residual_max_thickness),
            );
        }
        if r.max_nonlinear_error > 0.0 {
            u.label(format!("非线性误差 {:.3} mm", r.max_nonlinear_error));
        }

        u.separator();
        let total = scene.0.playback.total_time();
        let mut t = ui.t;
        if u.add(egui::Slider::new(&mut t, 0.0..=total.max(1e-9)).text("t / s")).changed() {
            ui.t = t;
        }
        u.horizontal(|u| {
            if u.button(if ui.playing { "⏸ 暂停" } else { "▶ 播放" }).clicked() {
                if ui.t >= total {
                    ui.t = 0.0;
                }
                ui.playing = !ui.playing;
            }
            u.add(egui::DragValue::new(&mut ui.speed).range(0.25..=32.0).prefix("×"));
        });
        if let Some(f) = scene.0.playback.at(ui.t) {
            u.label(format!(
                "块 {} · 行 {} · tip ({:.1}, {:.1}, {:.1})",
                f.block, f.line, f.pose.tip.x, f.pose.tip.y, f.pose.tip.z
            ));
        }

        let rows = view.alarm_rows();
        if !rows.is_empty() {
            u.separator();
            egui::ScrollArea::vertical().max_height(180.0).show(u, |u| {
                for row in &rows {
                    // 一行一条：徽标 编号 消息（×次数）——严重度已降序，最严重的在最上
                    let count = if row.count > 1 { format!(" ×{}", row.count) } else { String::new() };
                    u.colored_label(
                        tone_color(match row.badge {
                            "[!!]" | "[E]" => cncsim_ui::Tone::Fatal,
                            "[W]" => cncsim_ui::Tone::Attention,
                            _ => cncsim_ui::Tone::Neutral,
                        }),
                        format!("{} {} {}{}", row.badge, row.id, row.text, count),
                    );
                    // 位置三元组与 hint 用小字缩进，跟着报警走
                    u.label(egui::RichText::new(format!("      {}", row.location)).weak());
                    if let Some(h) = &row.hint {
                        u.label(egui::RichText::new(format!("      → {h}")).weak());
                    }
                }
            });
        }
        u.separator();
        u.label("拖动旋转 · 右键平移 · 滚轮缩放");
    });
    Ok(())
}

/// 切削面 / 刀具共用的材质：**调色板 + 索引**，不接 PBR。
///
/// ADR-005 的"索引 + 调色板"两段式：顶点色 R 通道是索引，颜色表是 uniform。
/// 换配色、单刀隔离、灰度模式都只改这张表，**不重算仿真、不重传网格**。
///
/// 刀具也用这个材质而不是 `StandardMaterial`，理由是**颜色必须能对上**：
/// 两边走同一张表、同一段光照代码，切面和刀刃才真是同一个颜色，
/// 而不是"看着差不多"。顺带省掉 `metallic` 没有环境贴图时渲染成白柱子那个坑。
#[derive(Asset, TypePath, AsBindGroup, Clone)]
pub struct CutMaterial {
    /// 256 色调色板的前 16 项（`cncsim_voxel::PALETTE_LEN`）。
    ///
    /// 只有 16 项而不是 256：ADR-005 预留了 256 是为了"按工序着色"这种
    /// 高基数模式，现在索引来源只有"第几刀"，16 项够用，uniform 也小。
    #[uniform(0)]
    pub palette: [Vec4; cncsim_voxel::PALETTE_LEN],
    /// 1 = 索引取自顶点色 R 通道（毛坯网格）；0 = 用下面的 `index`（刀具）。
    #[uniform(0)]
    pub use_vertex: u32,
    /// `use_vertex == 0` 时用的调色板索引。
    #[uniform(0)]
    pub index: u32,
}

impl Default for CutMaterial {
    fn default() -> Self {
        // `[[f32; 4]; 16]` → `[Vec4; 16]`：**uniform 里必须用 `Vec4`**，
        // `[f32; 4]` 在 WGSL 里是 `array<f32, 4>`（步长 4），
        // 放进 uniform 数组会直接报 "array stride must be a multiple of 16"。
        Self { palette: PALETTE.map(Vec4::from_array), use_vertex: 1, index: 0 }
    }
}

impl Material for CutMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cut.wesl".into()
    }
    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Opaque
    }
    // 切削面可能有单侧三角形（Surface Nets 的绕序在某些退化情形下不一致），
    // 两面都画。`cull_mode: None` 在自定义材质里靠 `alpha_mode` + 这里的默认值，
    // 真正的双面由 WESL 里的 `front_facing()` 翻转法向承担。
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::VertexAttributeValues;

    fn pos_of(m: &Mesh) -> Vec<[f32; 3]> {
        match m.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(v)) => v.clone(),
            other => panic!("POSITION 属性类型不对：{other:?}"),
        }
    }

    fn nrm_of(m: &Mesh) -> Vec<[f32; 3]> {
        match m.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(v)) => v.clone(),
            other => panic!("NORMAL 属性类型不对：{other:?}"),
        }
    }

    /// 每个顶点都必须落在母线绕 +Z 旋出来的曲面上：半径属于母线的 ρ 集合、
    /// 高度属于母线的 z 集合。**这是"旋转体"的定义本身**，不是近似检查。
    #[test]
    fn profile_mesh_vertices_lie_on_surface_of_revolution() {
        let p = cncsim_tool::flat_end_mill(6.0, 6.0); // ρ ∈ {0, 3}，z ∈ {0, 6}
        let m = profile_mesh(&p, 12);
        let pos = pos_of(&m);
        // 每段母线每个角度扇区：端点在轴上（底面、顶面）发 3 个顶点，
        // 中间那圈圆柱发 4 个 → (3 + 4 + 3) × 12 = 120
        assert_eq!(pos.len(), 12 * (3 + 4 + 3), "端面收点后不该再有 4 顶点四边形");
        for v in &pos {
            let r = (v[0] * v[0] + v[1] * v[1]).sqrt();
            let ok_r = (r - 0.0).abs() < 1e-4 || (r - 3.0).abs() < 1e-4;
            assert!(ok_r, "顶点半径 {r} 不在母线的 ρ 集合 {{0,3}} 里：{v:?}");
            let ok_z = (v[2] - 0.0).abs() < 1e-4 || (v[2] - 6.0).abs() < 1e-4;
            assert!(ok_z, "顶点高度 {} 不在母线的 z 集合 {{0,6}} 里", v[2]);
        }
    }

    /// 绕序必须让**外表面朝外**：圆柱段的法向与该点径向同向，端面法向沿 ±Z。
    ///
    /// 这条测试的证伪方式是反着写绕序——那时法向整体取反，
    /// 点积变成 −0.99 而不是 +0.99。这正是"刀具整个消失"（背面剔除）
    /// 那个坑的几何根因，值得钉住。
    ///
    /// 注意**不能只按半径筛圆柱段**：平底刀的底面是 `(0,0)→(3,0)` 的环，
    /// 它的外缘顶点半径也是 3，但法向是 `(0,0,-1)`。第一版就是这么写错的，
    /// 于是把朝外的正确端面判成了"朝内"——判据本身要能区分这两类顶点。
    #[test]
    fn profile_mesh_normals_point_outward() {
        let p = cncsim_tool::flat_end_mill(6.0, 6.0);
        let m = profile_mesh(&p, 12);
        let pos = pos_of(&m);
        let nrm = nrm_of(&m);
        assert_eq!(pos.len(), nrm.len());
        // 退化三角形会算出零法向（光照下是一块黑斑）。这条断言就是那个坑的守卫。
        for n in &nrm {
            assert!(Vec3::from_array(*n).length() > 0.9, "法向不是单位向量：{n:?}");
        }
        let (mut cyl, mut caps) = (0, 0);
        for (v, n) in pos.iter().zip(&nrm) {
            let n = Vec3::from_array(*n);
            if n.z.abs() > 1e-3 {
                // 端面：z=0 的朝下、z=6 的朝上
                let want = if v[2] < 3.0 { -1.0 } else { 1.0 };
                assert!(n.z * want > 0.95, "端面法向 {n:?} 朝向反了（顶点 {v:?}）");
                caps += 1;
                continue;
            }
            let r = (v[0] * v[0] + v[1] * v[1]).sqrt();
            assert!(r > 1e-3, "法向水平但半径为零：{v:?}");
            let radial = Vec3::new(v[0], v[1], 0.0) / r;
            let d = radial.dot(n);
            // 12 段时半角 15°，cos = 0.966 —— 门槛 0.95 能容下离散误差，
            // 但容不下"整体反号"。
            assert!(d > 0.95, "圆柱面法向朝内：径向 {radial:?} · 法向 {n:?} = {d}");
            cyl += 1;
        }
        assert_eq!(cyl, 12 * 4, "圆柱段应有 12 段 × 4 顶点");
        assert_eq!(caps, 2 * 12 * 3, "上下端面各 12 段 × 3 顶点（轴上那点只有一个）");
    }

    /// 索引必须全部在顶点范围内（越界的索引在 GPU 上是未定义行为，
    /// 在 `compute_normals` 里则会直接 panic 或算出垃圾法向）。
    #[test]
    fn profile_mesh_indices_in_range() {
        let p = cncsim_tool::ball_end_mill(6.0, 6.0, 8);
        let m = profile_mesh(&p, 24);
        let n = pos_of(&m).len() as u32;
        let Some(Indices::U32(idx)) = m.indices() else {
            panic!("应为 U32 索引");
        };
        assert_eq!(idx.len() % 3, 0);
        assert!(idx.iter().all(|i| *i < n), "有索引越界（顶点数 {n}）");
    }

    /// `to_world_dir` 必须与 `z_up_to_y_up` **是同一个旋转**。
    ///
    /// 两条独立路径（一个是四元数、一个是手写的分量重排）算出同一个结果，
    /// 才说明"刀具朝向"和"毛坯朝向"在同一套坐标里——否则刀会绕到毛坯外去。
    #[test]
    fn to_world_dir_agrees_with_z_up_to_y_up() {
        let q = z_up_to_y_up();
        for d in [
            cncsim_math::DVec3::new(0.0, 0.0, 1.0),
            cncsim_math::DVec3::new(1.0, 0.0, 0.0),
            cncsim_math::DVec3::new(0.0, 1.0, 0.0),
            cncsim_math::DVec3::new(0.3, -0.5, 0.81),
        ] {
            let a = to_world_dir(d);
            let b = q * Vec3::new(d.x as f32, d.y as f32, d.z as f32);
            assert!(
                (a - b).length() < 1e-5,
                "手写重排 {a:?} ≠ 四元数 {b:?}（模型方向 {d:?}）"
            );
        }
    }
}

#[cfg(test)]
mod live_tests {
    use super::*;
    use cncsim_math::DVec3;
    use cncsim_motion::Pose;

    /// 逐刀增量更新的真实成本。
    ///
    /// 截图模式只做**一次** seek（0 → t），所有刀并成一个大脏区，量不出
    /// "每刀一次更新"的样子。这里直接驱动 `LiveCut`，模拟播放时逐刀推进。
    #[test]
    #[ignore = "性能基准，手动跑"]
    fn live_cut_per_cut_update_cost() {
        let cutters = vec![(1u16, cncsim_tool::flat_end_mill(6.0, 20.0))];
        // 两条对照：① 面铣（每刀横跨整个毛坯）② 局部短槽（只动一小片）
        let full_width = std::env::var("LOCAL").is_err();
        let mut cuts = Vec::new();
        for i in 0..10 {
            let y = 1.5 + i as f64 * 3.0;
            let (a, b) = if full_width {
                if i % 2 == 0 { (-10.0, 40.0) } else { (40.0, -10.0) }
            } else {
                // 6 mm 长的一小段，落在毛坯中间
                let x0 = 12.0 + i as f64;
                (x0, x0 + 6.0)
            };
            cuts.push((
                i as f64,
                Pose { tip: DVec3::new(a, y, 11.0), axis: DVec3::Z },
                Pose { tip: DVec3::new(b, y, 11.0), axis: DVec3::Z },
                1,
            ));
        }
        let tile_mm: f64 = std::env::var("TILE").ok().and_then(|v| v.parse().ok()).unwrap_or(8.0);
        let cfg_run = cncsim_sim::run::RunConfig {
            stock_size: [300, 300, 120],
            design: None,
            creator_sub: 4,
            ..Default::default()
        };
        let mut live = LiveCut::new(&cfg_run, cutters, cuts, tile_mm, 3);

        let t = std::time::Instant::now();
        let full = {
            let lc = &mut live;
            let LiveCut { cache, grid, .. } = lc;
            cache.update(grid, None, usize::MAX)
        };
        println!();
        println!("首次全量：{:.1} ms，{} 块", t.elapsed().as_secs_f64() * 1e3, full.uploads.len());

        let mut total = 0.0;
        for i in 1..=live.cut_count() {
            let t = std::time::Instant::now();
            let d = live.seek(i);
            let t_seek = t.elapsed().as_secs_f64() * 1e3;
            let res = {
                let lc = &mut live;
                let LiveCut { cache, grid, .. } = lc;
                match d {
                    Dirty::Full => cache.update(grid, None, usize::MAX),
                    Dirty::Region(lo, hi) => cache.update(grid, Some((lo, hi)), usize::MAX),
                    Dirty::None => continue,
                }
            };
            let ms = t.elapsed().as_secs_f64() * 1e3;
            total += ms;
            println!(
                "  第 {i:>2} 刀：{ms:>6.2} ms（切削 {t_seek:.2} + 重建 {:.2}），重建 {} 块",
                ms - t_seek,
                res.rebuilt
            );
        }
        println!(
            "逐刀合计 {total:.1} ms（平均 {:.2} ms/刀）；对照：全量提取一次约 10 ms",
            total / live.cut_count() as f64
        );

        // 窗口实际用的那份网格（分块合并后）的切色分布
        let merged = live.cache.merged();
        let bands = cncsim_voxel::color_bands(&merged);
        println!();
        println!("分块合并后的网格：{} 顶点", merged.positions.len());
        for b in bands.iter().filter(|b| b.index != 0) {
            println!(
                "  idx {:>2}  Z {:.3}..{:.3}  Y {:.2}..{:.2}  朝上 {:.1}/{} mm²",
                b.index, b.lo[2], b.hi[2], b.lo[1], b.hi[1], b.area_up_mm2, b.area_mm2 as u32
            );
        }
        let minz = merged
            .positions
            .iter()
            .zip(&merged.colors)
            .filter(|(_, c)| c[0] != 0)
            .map(|(p, _)| p[2] as f64)
            .fold(f64::INFINITY, f64::min);
        println!("  切色顶点最低 z = {minz:.3}（地板 11.00 → 溢出 {:.3} mm）", 11.0 - minz);
        // 对照：同一份网格走**整块**提取（不分块）
        let whole = live.grid.extract_surface_lod(3);
        let wz = whole
            .positions
            .iter()
            .zip(&whole.colors)
            .filter(|(_, c)| c[0] != 0)
            .map(|(p, _)| p[2] as f64)
            .fold(f64::INFINITY, f64::min);
        println!(
            "  对照：整块提取 {} 顶点，切色最低 z = {wz:.3}（溢出 {:.3} mm）",
            whole.positions.len(),
            11.0 - wz
        );

    }
}
