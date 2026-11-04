//! # cncsim-voxel
//!
//! 稀疏窄带 SDF：树结构、解析扫掠、CSG、Surface Nets 表面提取、显示 LOD。
//!
//! 结构借鉴 OpenVDB：三级树（root 哈希 → 32³ → 16³ → 8³ 叶）+ 位掩码 + tile 值。
//! 见 `docs/ADR-006-稀疏体素数据结构.md`。
//!
//! ## 存储层是地基
//!
//! 先树后算法，**不要先写扁平 `HashMap` 再改**——改一次所有上层要重测。
//!
//! ## 模块
//!
//! | 模块 | 职责 |
//! |---|---|
//! | [`tree`] | 三级树、tile 值、索引位运算、`get`/`set`/`fill_box`/`prune` |
//! | [`sdf`] | 量化换算、三线性插值查询 |
//! | [`sweep`] | 刀具扫掠体的解析 SDF（`min over t` + 黄金分割） |
//! | [`stl`] | STL 解析（二进制 + ASCII，零依赖） |
//! | [`cut`] | CSG 去除 `d_new = max(d_old, -sd_sweep)` |
//! | [`extract`] | 表面提取（Surface Nets，对偶等值面） |
//! | [`cellmap`] | 单元格 → 顶点索引（块 + 位掩码 + popcount） |
//! | [`lod`] | 显示 LOD：粗网格重采样（`k ≤ band_voxels`） |
//! | [`fxhash`] | 整数键哈希（替掉 SipHash） |
//! | [`par`] | 零依赖 fork-join（叶 / 体素 / 粗格级零共享） |
//! | [`leafset`] | 叶坐标集合（位图 / 哈希集回退） |
//! | [`components`] | 连通块清理（丢掉切下来还挂着的悬浮碎屑） |

pub mod cellmap;
pub mod components;
pub mod cut;
pub mod extract;
pub mod fxhash;
pub mod leafset;
pub mod lod;
pub mod meshvol;
pub mod par;
pub mod sdf;
pub mod stl;
pub mod sweep;
pub mod tree;

pub use cut::{CutStats, prof_take, prof_take_write, sweep_aabb, PROF_NEW_LEAF, PROF_N_BLOCK};
pub use tree::{prof_take_box, PROF_BOX_N};
pub use lod::choose_lod_factor;
pub use sweep::{MAX_GOLDEN_ITERS, MIN_GOLDEN_ITERS, golden_iters, lerp_pose, sd_sweep, sd_sweep_sampled, sd_tool};

pub use tree::{
    L1_SLOTS, L1_SPAN, L2_SLOTS, L2_SPAN, LEAF_DIM, LEAF_LOG2, LEAF_VOXELS, Leaf, LeafView, Level1, Level2,
    L1_LOG2, L2_LOG2, ROOT_SPAN, RootEntry, Scalar, SparseGrid, child_index, l1_coord, l1_slot,
    l2_slot, leaf_coord, leaf_index, root_coord,
};

/// 调色板长度（ADR-005）：0 = 原始毛坯面，1..=12 = 切割色，13 = 来源混杂，
/// 14 = 刀柄/非切削件，15 = 保留。
///
/// **索引与颜色分开**：网格里只存这个 `u8` 索引，颜色表在 `cncsim_render`。
/// 换配色、按刀隔离、单色模式都只改颜色表，**不重算仿真、不重传网格**。
pub const PALETTE_LEN: usize = 32;

/// 可用切割色数（索引 1..=29）。
pub const CUT_COLORS: u32 = 29;

/// 调色板索引 `MIXED`：一片叶被 8 把以上的刀切过（叶级溯源只留最后一把，
/// 所以实际不会产生这个值；保留它是为了与 ADR-005 的编码对齐）。
pub const CREATOR_MIXED: u8 = 30;

/// 调色板索引 `SHANK`：刀柄/夹头这类"非切削件"。
pub const CREATOR_SHANK: u8 = 31;

/// 每把刀按进度切成几段色（10 段 = 每 10% 换一种颜色）。
pub const TOOL_SEGMENTS: u8 = 10;

/// **一把刀内部的进度色**：按 10% 切段，返回该刀的第几段（0..=9）。
///
/// 参数 `done` / `total` 是**这一把刀内部**已切 / 总切削块数。
///
/// 早先按 254 级连续进度插值，视觉是平滑渐变；但用户要的是"一眼看出这块面
/// 是哪把刀、哪个阶段切的"，渐变反而不如分档直观。所以回到**离散**：
/// 每 10% 一段，段与段之间是硬边。
#[inline]
pub fn tool_segment(done: u32, total: u32) -> u8 {
    if total == 0 {
        return 0;
    }
    ((done as u64 * TOOL_SEGMENTS as u64) / total as u64).min(TOOL_SEGMENTS as u64 - 1) as u8
}

/// **调色板索引** = 刀具号 × 10 段 + 段号，从 1 起。
///
/// - `tool` 是 `T` 字（1 起）；0 表示"没指定"，按第 1 把算。
/// - 第 1 把刀占 1..=10，第 2 把占 11..=20，第 3 把占 21..=29（封顶）。
/// - 上限 29：30 是 `CREATOR_MIXED`、31 是 `CREATOR_SHANK`。
///
/// **按刀具分组**是有意的：粗加工和精加工各自从 0→100%，每 10% 换一个颜色，
/// 两族的色相/饱和度刻意拉开（粗加工饱和、精加工淡），一眼能分出"这片是谁切的"。
#[inline]
pub fn creator_of_tool_segment(tool: u16, done: u32, total: u32) -> u8 {
    let base = (tool.saturating_sub(1).min(2) as u8) * TOOL_SEGMENTS;
    (1 + base + tool_segment(done, total)).min(CUT_COLORS as u8)
}

/// 兼容旧名：单刀场景（`tool = 1`）下的 [`creator_of_tool_segment`]。
#[inline]
pub fn creator_of_progress(done: u32, total: u32) -> u8 {
    creator_of_tool_segment(1, done, total)
}


/// 体素值类型——**这是精度的真正旋钮，不是 float 宽度**。
///
/// `I8`：3.15 µm/档（band=±4 体素、h=0.1 → 满量程 0.8 mm / 254 档），内存 206 MB
/// `I16`：12.2 nm/档（满量程 0.8 mm / 65534 档），内存 412 MB
///
/// 比值 **258×**——2 倍内存换两个半数量级的精度。换 f64 零收益。
///
/// 见 docs/计算精度分析.md §2。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ValueType {
    #[default]
    I8,
    I16,
}

impl ValueType {
    /// 分档数（满量程被分成多少段）
    pub fn steps(self) -> f64 {
        match self {
            ValueType::I8 => 254.0,
            ValueType::I16 => 65534.0,
        }
    }

    /// 每档代表多少 mm
    pub fn step_mm(self, config: &GridConfig) -> f64 {
        // 满量程 = 2 × band（−band … +band）
        2.0 * config.band_mm() / self.steps()
    }
}

/// 加载期参数。改 `h` 要重建整个体素场（等于一次完整仿真）。
#[derive(Clone, Copy, Debug)]
pub struct GridConfig {
    /// 体素边长 mm
    pub h: f64,
    /// 窄带宽度（体素数，单侧）
    pub band_voxels: u8,
    pub value_type: ValueType,
}

impl Default for GridConfig {
    fn default() -> Self {
        Self { h: 0.1, band_voxels: 4, value_type: ValueType::I8 }
    }
}

impl GridConfig {
    /// 物理带宽（mm）
    pub fn band_mm(&self) -> f64 {
        self.band_voxels as f64 * self.h
    }
}

/// 提取出的网格。`render` 消费这个类型（不碰 SDF 内部）。
#[derive(Clone, Debug, Default)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// 顶点色：R = 调色板索引，G = 余量偏差编码（ADR-005）
    pub colors: Vec<[u8; 4]>,
    /// 因邻居单元格缺顶点而跳过的四边形数。
    ///
    /// **应当恒为 0**——非零表示网格有洞。保留这个计数是为了让"有洞"变成可观测的，
    /// 而不是悄悄少几个三角形。
    pub skipped_quads: usize,
}

impl MeshData {
    /// **用面积加权的面法线平均**重算全部顶点法线。
    ///
    /// 提取时每个顶点的法线是 SDF 的粗格中心差分（`vertex_of` 里 8 个角点），
    /// 它带着**格子尺度**的噪声：R15 球面 k=4 时与解析法线中位差 1.94°，
    /// 光照把这点角度放大成一圈圈「波纹」。面法线平均低通的是**网格本身**，
    /// 贴的是真实几何而不是体素格。
    ///
    /// **必须在 [`Self::split_crease_normals`] 之后调**：硬边已经按夹角拆成
    /// 独立顶点，每个顶点只累加同一侧的面，平均不会把棱角抹圆。
    ///
    /// 未归一化的叉积长度就是 2×面积，所以"直接累加不归一化"天然是面积权重，
    /// 不需要再乘一次。
    pub fn recompute_smooth_normals(&mut self) {
        let n = self.positions.len();
        if n == 0 || self.indices.len() < 3 || self.normals.len() != n {
            return;
        }
        let mut acc = vec![[0.0f32; 3]; n];
        for t in self.indices.as_chunks::<3>().0 {
            let (ia, ib, ic) = (t[0] as usize, t[1] as usize, t[2] as usize);
            let p0 = self.positions[ia];
            let p1 = self.positions[ib];
            let p2 = self.positions[ic];
            let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            let cr = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            for i in [ia, ib, ic] {
                acc[i][0] += cr[0];
                acc[i][1] += cr[1];
                acc[i][2] += cr[2];
            }
        }
        for (i, a) in acc.iter().enumerate() {
            let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
            if l > 1e-20 {
                self.normals[i] = [a[0] / l, a[1] / l, a[2] / l];
            }
        }
    }

    /// **按折痕角拆顶点**：把接近 90° 的棱做成硬边。
    ///
    /// Surface Nets 一格一个顶点，地板与立面**共用**同一个顶点，法向只能是折中的
    /// 45°。两个方向光在 45° 上都会变暗，于是棱上塌出一条暗线——实测机加工件的
    /// 顶棱 108 个顶点里 107 个法向被平均。未切削时两侧同色看不出来，一旦一侧
    /// 变成彩色，那条线就显眼了。
    ///
    /// 做法：法向与面法向夹角超过阈值的角，**为该三角形复制一份顶点**并用面法向。
    /// 平面区域一个顶点都不多（夹角 0），只有棱上才复制；曲面（球头刀铣出来的）
    /// 相邻面夹角小，仍然平滑。
    ///
    /// 顺序完全由三角形顺序决定，不引入任何不确定性。
    pub fn split_crease_normals(&mut self, cos_thresh: f32) {
        if self.indices.len() < 3 || self.normals.len() != self.positions.len() {
            return;
        }
        let n0 = self.positions.len();
        let mut pos = self.positions.clone();
        let mut nrm = self.normals.clone();
        let mut col = self.colors.clone();
        let mut idx = Vec::with_capacity(self.indices.len());
        for t in self.indices.as_chunks::<3>().0 {
            let (a, b, c) = (
                self.positions[t[0] as usize],
                self.positions[t[1] as usize],
                self.positions[t[2] as usize],
            );
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let cr = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let len = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
            if len < 1e-12 {
                idx.extend_from_slice(t);
                continue;
            }
            let f = [cr[0] / len, cr[1] / len, cr[2] / len];
            for &i in t {
                let vn = self.normals[i as usize];
                let d = vn[0] * f[0] + vn[1] * f[1] + vn[2] * f[2];
                if d >= cos_thresh {
                    idx.push(i); // 夹角小：继续共用，保持平滑
                } else {
                    // 硬边：复制一份，用面法向
                    idx.push(pos.len() as u32);
                    pos.push(self.positions[i as usize]);
                    nrm.push(f);
                    col.push(self.colors.get(i as usize).copied().unwrap_or([0, 128, 0, 255]));
                }
            }
        }
        let _ = n0;
        self.positions = pos;
        self.normals = nrm;
        self.colors = col;
        self.indices = idx;
    }
}

/// 一种调色板索引在网格上的分布（[`color_bands`] 的输出）。
///
/// 用途是**回答"切完之后面上到底有几种颜色、各自占多大一片"**——
/// 光看截图只能定性说"有渐变"，说不清"每刀一条带子、宽度等于步距"。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColorBand {
    /// 调色板索引（0 = 原始毛坯面）
    pub index: u8,
    pub vertices: usize,
    /// 归属到这一档的三角形数（按三顶点的**多数**表决）
    pub triangles: usize,
    /// 这些三角形的总面积（mm²）
    pub area_mm2: f64,
    /// 其中**朝上**（|nz| > 0.9）的那部分面积（mm²）。
    ///
    /// 面铣之后"这一档是不是一块平面"要看这个，不能看总面积：侧壁上
    /// 有一圈会被叶级溯源溢出染上同一档颜色（最多一个叶宽 8h），
    /// 那部分面积算进 `area_mm2` 会把"平面面积"撑大 6% 以上。
    pub area_up_mm2: f64,
    /// 该档顶点的包围盒
    pub lo: [f32; 3],
    pub hi: [f32; 3],
    /// 三顶点索引互不相同的三角形数——它们跨在两档之间。
    ///
    /// 拓扑上无法避免（顶点色是逐顶点的，一条边两侧可以不同档），
    /// 但**数量应该很小**：如果很大，说明着色边界比网格分辨率还碎，
    /// 那就不是"每一刀一条带子"而是噪点。
    pub mixed_triangles: usize,
}

/// 按调色板索引统计网格（`colors[i][0]` 是索引，见 ADR-005）。
///
/// 结果按索引升序，只包含**实际出现过**的索引。
pub fn color_bands(mesh: &MeshData) -> Vec<ColorBand> {
    let n = mesh.positions.len();
    if n == 0 {
        return Vec::new();
    }
    let idx_of = |i: usize| mesh.colors.get(i).map(|c| c[0]).unwrap_or(0);

    // 顶点：计数 + 包围盒（用 256 个桶，索引本来就只有 0..=255）
    let mut count = [0usize; 256];
    let mut lo = [[f32::INFINITY; 3]; 256];
    let mut hi = [[f32::NEG_INFINITY; 3]; 256];
    for i in 0..n {
        let k = idx_of(i) as usize;
        count[k] += 1;
        let p = mesh.positions[i];
        for a in 0..3 {
            lo[k][a] = lo[k][a].min(p[a]);
            hi[k][a] = hi[k][a].max(p[a]);
        }
    }

    // 三角形：多数表决 + 面积
    let mut tris = [0usize; 256];
    let mut area = [0.0f64; 256];
    let mut area_up = [0.0f64; 256];
    let mut mixed = [0usize; 256];
    for t in mesh.indices.as_chunks::<3>().0 {
        let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
        if i0 >= n || i1 >= n || i2 >= n {
            continue;
        }
        let (c0, c1, c2) = (idx_of(i0), idx_of(i1), idx_of(i2));
        let k = if c0 == c1 || c0 == c2 {
            c0
        } else if c1 == c2 {
            c1
        } else {
            // 三个都不同：记一次"混合"，归到最小的那档（稳定、可复现）
            c0.min(c1).min(c2)
        };
        let is_mixed = c0 != c1 && c1 != c2 && c0 != c2;
        let k = k as usize;
        tris[k] += 1;
        if is_mixed {
            mixed[k] += 1;
        }
        let p = |i: usize| {
            let v = mesh.positions[i];
            [v[0] as f64, v[1] as f64, v[2] as f64]
        };
        let (a, b, c) = (p(i0), p(i1), p(i2));
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cr = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let len = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
        let a2 = 0.5 * len;
        area[k] += a2;
        if len > 1e-12 && (cr[2] / len).abs() > 0.9 {
            area_up[k] += a2;
        }
    }

    (0..256)
        .filter(|k| count[*k] > 0)
        .map(|k| ColorBand {
            index: k as u8,
            vertices: count[k],
            triangles: tris[k],
            area_mm2: area[k],
            area_up_mm2: area_up[k],
            lo: lo[k],
            hi: hi[k],
            mixed_triangles: mixed[k],
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 面法线平均：两个成 90° 的面共边，共边顶点法线应是两者的角平分线。
    #[test]
    fn smooth_normals_average_adjacent_faces() {
        let mut m = MeshData {
            positions: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0], // 面 +Z
                [0.0, 0.0, 1.0], // 面 +Y
            ],
            // 法线故意写错（全 +X），看它会不会被重算
            normals: vec![[1.0, 0.0, 0.0]; 4],
            colors: vec![[1, 128, 0, 255]; 4],
            // 两个三角形共用 0-1 这条边
            indices: vec![0, 1, 2, 0, 3, 1],
            ..Default::default()
        };
        m.recompute_smooth_normals();
        // 顶点 0、1 同时属于两个面 → 法线应该是两个面法线的矢量和
        for i in [0usize, 1] {
            let n = m.normals[i];
            assert!(n[2] > 0.5, "顶点 {i} 的法线应带 +Z 分量：{n:?}");
            assert!(n[1].abs() > 0.5, "顶点 {i} 的法线应带 Y 分量：{n:?}");
        }
        // 顶点 2 只属于一个面 → 法线就是那个面的法线（±Z）
        assert!(m.normals[2][2].abs() > 0.99, "单面顶点的法线应贴合面：{:?}", m.normals[2]);
    }


    #[test]
    fn default_config_is_standard_preset() {
        let c = GridConfig::default();
        assert_eq!(c.h, 0.1);
        assert_eq!(c.band_voxels, 4);
        assert!((c.band_mm() - 0.4).abs() < 1e-12);
    }

    #[test]
    fn i16_is_much_more_precise_than_i8() {
        let c = GridConfig::default();
        let i8_step = ValueType::I8.step_mm(&c);
        let i16_step = ValueType::I16.step_mm(&c);

        // i8 每档应约 3.15 µm（满量程 0.8 mm / 254 档）
        assert!(
            (i8_step * 1000.0 - 3.15).abs() < 0.01,
            "i8 每档应约 3.15 µm，实际 {:.3} µm",
            i8_step * 1000.0
        );
        // i16 每档应约 12.2 nm
        assert!(
            (i16_step * 1e6 - 12.2).abs() < 0.1,
            "i16 每档应约 12.2 nm，实际 {:.3} nm",
            i16_step * 1e6
        );
        // 比值约 258×
        let ratio = i8_step / i16_step;
        assert!(
            (ratio - 258.0).abs() < 2.0,
            "i16 应比 i8 精细约 258 倍，实际 {ratio:.1}"
        );
    }
}
