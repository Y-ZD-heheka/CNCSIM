//! # cncsim-render
//!
//! 显示层：**网格 → GPU 缓冲**，以及一个自包含的 HTML 预览导出。
//!
//! ## 为什么这里还没有 Bevy
//!
//! Bevy 是 L1 呈现层的既定选择（ADR-003），但它只在"有窗口的那一层"需要。
//! 这一层真正难的不是窗口，是把体素网格变成能画的三角形：
//!
//! | 步骤 | 在哪 | 现在 |
//! |---|---|---|
//! | 表面提取 | `cncsim-voxel` | 已完成（Surface Nets + LOD） |
//! | 顶点交错 / 索引重映射 / 分块 | **本 crate** | 已完成 + 测试 |
//! | 上传 GPU、材质、相机 | Bevy | **未接** |
//!
//! 前两步可以脱离引擎验证，第三步只是"把 buffer 交给 `Mesh`"。所以先把前两步
//! 写实——Bevy 那一步不改变这里任何一个字节。
//!
//! ## 分块不是为了好看
//!
//! 一叶一实体 = 39 万 draw call。按空间分块（默认 3.2 mm 见方）之后，每块一个
//! 实体，**只重建被切到的那些块**——面铣一个来回只动几十块，不是几十万。

use cncsim_voxel::MeshData;

/// 一帧允许上传的网格字节数上限。超过就留到下一帧——否则面铣会单帧传几十 MB。
pub const MAX_UPLOAD_BYTES_PER_FRAME: usize = 8 * 1024 * 1024;

/// 一个显示块（默认 32³ 体素 = 3.2 mm @ h=0.1）。
///
/// **不是一叶一实体**——39 万个叶会产生 39 万 draw call。
pub struct TileUpload {
    pub tile_id: u64,
    pub mesh: MeshData,
    pub version: u64,
}

/// 顶点缓冲布局：位置 3 + 法向 3，**交错**。
///
/// 交错而不是分开两个 buffer：GPU 取一个顶点时两个属性通常在同一缓存行里，
/// 分开存会多一次随机访问。颜色走逐块材质，不进顶点。
pub const VERTEX_FLOATS: usize = 6;
/// 一个顶点占的字节数
pub const VERTEX_BYTES: usize = VERTEX_FLOATS * 4;

/// GPU 可以直接上传的网格：交错顶点 + 索引。
pub struct GpuMesh {
    /// `[x, y, z, nx, ny, nz] × n`
    pub vertices: Vec<f32>,
    /// 每顶点 4 字节：`[调色板索引, 余量偏差+128, 保留, 255]`（ADR-005 的编码）。
    ///
    /// **单独一个 buffer 而不是塞进交错顶点**：它是 u8，混进 f32 流要手写字节
    /// 拼装，而颜色更新频率远低于位置。分开存，改颜色时不用重传位置。
    pub colors: Vec<u8>,
    pub indices: Vec<u32>,
    /// 包围盒（世界坐标）
    pub lo: [f32; 3],
    pub hi: [f32; 3],
}

impl GpuMesh {
    pub fn vertex_count(&self) -> usize {
        self.vertices.len() / VERTEX_FLOATS
    }
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
    /// 上传要多少字节（顶点 + 颜色 + 索引）
    pub fn bytes(&self) -> usize {
        self.vertices.len() * 4 + self.colors.len() + self.indices.len() * 4
    }
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }
}

/// 把 `MeshData` 打成 GPU 缓冲。**顶点顺序不变**，索引原样保留。
///
/// 顺序不变是有意的：渲染不该改变几何，出了问题要能逐位对回提取结果。
pub fn interleave(mesh: &MeshData) -> GpuMesh {
    let n = mesh.positions.len();
    let mut vertices = Vec::with_capacity(n * VERTEX_FLOATS);
    let mut colors = Vec::with_capacity(n * 4);
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for i in 0..n {
        let p = mesh.positions[i];
        let nrm = mesh.normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]);
        vertices.extend_from_slice(&[p[0], p[1], p[2], nrm[0], nrm[1], nrm[2]]);
        let c = mesh.colors.get(i).copied().unwrap_or([0, 128, 0, 255]);
        colors.extend_from_slice(&c);
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    if n == 0 {
        lo = [0.0; 3];
        hi = [0.0; 3];
    }
    GpuMesh {
        vertices,
        colors,
        indices: mesh.indices.clone(),
        lo,
        hi,
    }
}

/// 按空间分块。三角形的归属看**质心**落在哪一块——这样每个三角形恰好属于一块，
/// 不会出现跨块重复绘制。
///
/// 分块之后每块单独重建：切削只影响它扫过的那几块，其余块的缓冲原样留着。
pub fn split_tiles(mesh: &MeshData, tile_mm: f32) -> Vec<TileUpload> {
    use std::collections::HashMap;
    let tile = tile_mm.max(1e-6) as f64;

    // 三角形 → 块号
    let mut groups: HashMap<u64, Vec<usize>> = HashMap::new();
    for t in 0..mesh.indices.len() / 3 {
        let ids = [
            mesh.indices[t * 3],
            mesh.indices[t * 3 + 1],
            mesh.indices[t * 3 + 2],
        ];
        let mut c = [0.0f64; 3];
        for &i in &ids {
            let p = mesh.positions[i as usize];
            for a in 0..3 {
                c[a] += p[a] as f64 / 3.0;
            }
        }
        groups.entry(tile_id(c, tile)).or_default().push(t);
    }

    let mut out = Vec::with_capacity(groups.len());
    for (id, tris) in groups {
        // 收集这块用到的顶点，重映射成 0..k
        let mut remap: HashMap<u32, u32> = HashMap::new();
        let mut sub = MeshData::default();
        for &t in &tris {
            for k in 0..3 {
                let old = mesh.indices[t * 3 + k];
                let new = *remap.entry(old).or_insert_with(|| {
                    let i = old as usize;
                    sub.positions.push(mesh.positions[i]);
                    sub.normals
                        .push(mesh.normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]));
                    sub.colors
                        .push(mesh.colors.get(i).copied().unwrap_or([0, 128, 0, 255]));
                    (sub.positions.len() - 1) as u32
                });
                sub.indices.push(new);
            }
        }
        out.push(TileUpload {
            tile_id: id,
            mesh: sub,
            version: 0,
        });
    }
    // 顺序必须确定：HashMap 的迭代顺序每次都不一样，直接拿去渲染会造成
    // "同一份数据两次运行 draw call 顺序不同"，逐位复现就断了
    out.sort_by_key(|t| t.tile_id);
    out
}

/// 把世界坐标压成块号（每轴 21 位，够 ±1 km @ 0.1 mm）。
fn tile_id(c: [f64; 3], tile_mm: f64) -> u64 {
    tile_id_of(
        (c[0] / tile_mm).floor() as i32,
        (c[1] / tile_mm).floor() as i32,
        (c[2] / tile_mm).floor() as i32,
    )
}

/// 块索引 → 块号。[`tile_id`] 的整数版本，增量重建按索引枚举时用。
pub(crate) fn tile_id_of(i: i32, j: i32, k: i32) -> u64 {
    let mut id = 0u64;
    for (a, v) in [i, j, k].iter().enumerate() {
        let v = *v as i64 + (1 << 20);
        id |= ((v as u64) & 0x1F_FFFF) << (a * 21);
    }
    id
}

/// 一帧能传多少块：按顺序贪心，累计超过 [`MAX_UPLOAD_BYTES_PER_FRAME`] 就留给下一帧。
pub fn tiles_per_frame(tiles: &[TileUpload]) -> usize {
    let mut bytes = 0usize;
    for (i, t) in tiles.iter().enumerate() {
        bytes += t.mesh.positions.len() * (VERTEX_BYTES + 4) + t.mesh.indices.len() * 4;
        if bytes > MAX_UPLOAD_BYTES_PER_FRAME {
            return i.max(1);
        }
    }
    tiles.len()
}

/// 调色板（ADR-005）：**索引由 `cncsim_voxel` 定义，颜色只在这里**。
///
/// 与顶点色一起构成"索引 + 调色板"两段式：网格里只存 `u8` 索引，换配色、
/// 单刀隔离、灰度模式都只改这张表，**不重算仿真、不重传网格**。
///
/// 放在 `render` 的根而不是 `shell`：窗口（`shell`）和自包含 HTML 预览
/// （`preview`）是同一份数据的两个出口，两张表一旦分家，"导出页和窗口不一致"
/// 就是迟早的事。**刀刃与切面也用同一张表**，所以"刀刃什么颜色、切出来的面
/// 就是什么颜色"不是靠约定对齐，而是构造上只有一份。
pub const PALETTE: [[f32; 4]; cncsim_voxel::PALETTE_LEN] = [
    // 0：原始毛坯面
    [0.70, 0.73, 0.78, 1.0],
    // 1..=10：**第 1 把刀（粗加工）10 段**——高饱和、明度 0.50，段间色相差 36°
    [0.890, 0.110, 0.110, 1.0], // T1 段0
    [0.890, 0.578, 0.110, 1.0], // T1 段1
    [0.734, 0.890, 0.110, 1.0], // T1 段2
    [0.266, 0.890, 0.110, 1.0], // T1 段3
    [0.110, 0.890, 0.422, 1.0], // T1 段4
    [0.110, 0.890, 0.890, 1.0], // T1 段5
    [0.110, 0.422, 0.890, 1.0], // T1 段6
    [0.266, 0.110, 0.890, 1.0], // T1 段7
    [0.734, 0.110, 0.890, 1.0], // T1 段8
    [0.890, 0.110, 0.578, 1.0], // T1 段9
    // 11..=20：**第 2 把刀（精加工）10 段**——色相错开 18°，明度 0.65（比粗加工亮）
    // 分族靠**明度**：粗加工 0.50 偏暗、精加工 0.65 偏亮；段内靠 36° 色相
    [0.912, 0.545, 0.388, 1.0], // T2 段0
    [0.912, 0.860, 0.388, 1.0], // T2 段1
    [0.650, 0.912, 0.388, 1.0], // T2 段2
    [0.388, 0.912, 0.440, 1.0], // T2 段3
    [0.388, 0.912, 0.755, 1.0], // T2 段4
    [0.388, 0.755, 0.912, 1.0], // T2 段5
    [0.388, 0.440, 0.912, 1.0], // T2 段6
    [0.650, 0.388, 0.912, 1.0], // T2 段7
    [0.912, 0.388, 0.860, 1.0], // T2 段8
    [0.912, 0.388, 0.545, 1.0], // T2 段9
    // 21..=29：第 3 把刀起（色相沿用第 1 族，明度 0.62 与 T1 分开）
    [0.878, 0.362, 0.362, 1.0], // T3 段0
    [0.878, 0.672, 0.362, 1.0], // T3 段1
    [0.775, 0.878, 0.362, 1.0], // T3 段2
    [0.465, 0.878, 0.362, 1.0], // T3 段3
    [0.362, 0.878, 0.568, 1.0], // T3 段4
    [0.362, 0.878, 0.878, 1.0], // T3 段5
    [0.362, 0.568, 0.878, 1.0], // T3 段6
    [0.465, 0.362, 0.878, 1.0], // T3 段7
    [0.775, 0.362, 0.878, 1.0], // T3 段8
    // 30：来源混杂（叶级溯源实际不会产生，保留以对齐 ADR-005 的编码）
    [0.55, 0.55, 0.55, 1.0],
    // 31：刀柄 / 夹头——刻意压暗，与浅灰毛坯和亮色刀刃都分得开
    [0.34, 0.37, 0.43, 1.0],
];

/// 刀柄用的固定索引（[`PALETTE`] 的 14 号）。
pub const SHANK_INDEX: u32 = cncsim_voxel::CREATOR_SHANK as u32;

pub mod preview;

/// Bevy 窗口外壳。默认构建不编译（`window` feature 关闭）。
#[cfg(feature = "window")]
pub mod shell;
mod tiles;
pub use tiles::{TileCache, UpdateResult};

#[cfg(test)]
mod tests {
    use super::*;

    fn quad() -> MeshData {
        let mut m = MeshData::default();
        m.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        m.normals = vec![[0.0, 0.0, 1.0]; 4];
        m.indices = vec![0, 1, 2, 0, 2, 3];
        m.colors = vec![[0, 128, 0, 255]; 4];
        m
    }

    #[test]
    fn interleave_keeps_order_and_bounds() {
        let g = interleave(&quad());
        assert_eq!(g.vertex_count(), 4);
        assert_eq!(g.triangle_count(), 2);
        assert_eq!(g.vertices[0..6], [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(g.lo, [0.0, 0.0, 0.0]);
        assert_eq!(g.hi, [1.0, 1.0, 0.0]);
        assert_eq!(g.indices, vec![0, 1, 2, 0, 2, 3], "索引不该被改");
    }

    #[test]
    fn colors_ride_along_separately() {
        let mut m = quad();
        m.colors = vec![[1, 200, 0, 255], [2, 128, 0, 255], [0, 0, 0, 255], [3, 255, 0, 255]];
        let g = interleave(&m);
        assert_eq!(g.colors.len(), 4 * 4, "每个顶点 4 个字节");
        assert_eq!(&g.colors[0..4], &[1, 200, 0, 255]);
        assert_eq!(&g.colors[12..16], &[3, 255, 0, 255]);
        // 颜色缺失时给默认值，不能 panic / 越界
        let mut bare = quad();
        bare.colors.clear();
        let g2 = interleave(&bare);
        assert_eq!(g2.colors.len(), 16);
        assert_eq!(&g2.colors[0..4], &[0, 128, 0, 255]);
        // 字节数把颜色算进去
        assert_eq!(g.bytes(), g.vertices.len() * 4 + g.colors.len() + g.indices.len() * 4);
    }

    #[test]
    fn empty_mesh_is_safe() {
        let g = interleave(&MeshData::default());
        assert!(g.is_empty());
        assert_eq!(g.vertex_count(), 0);
        assert!(g.lo.iter().all(|v| v.is_finite()));
        assert!(split_tiles(&MeshData::default(), 1.0).is_empty());
    }

    #[test]
    fn tiles_partition_every_triangle_exactly_once() {
        // 一个跨 4 个块的平面：三角形必须被完整分掉，不重不漏
        let mut m = MeshData::default();
        for i in 0..=4 {
            for j in 0..=4 {
                m.positions.push([i as f32, j as f32, 0.0]);
                m.normals.push([0.0, 0.0, 1.0]);
                m.colors.push([0, 128, 0, 255]);
            }
        }
        let w = 5u32;
        for i in 0..4u32 {
            for j in 0..4u32 {
                let a = j * w + i;
                m.indices
                    .extend_from_slice(&[a, a + 1, a + w + 1, a, a + w + 1, a + w]);
            }
        }
        let orig_tris = m.indices.len() / 3;
        let tiles = split_tiles(&m, 2.0);
        assert_eq!(tiles.len(), 4, "2×2 个块，实际 {}", tiles.len());
        let total: usize = tiles.iter().map(|t| t.mesh.indices.len() / 3).sum();
        assert_eq!(total, orig_tris, "三角形不能重复也不能丢");
        // 每块的索引都在自己的顶点范围内
        for t in &tiles {
            let n = t.mesh.positions.len() as u32;
            assert!(t.mesh.indices.iter().all(|&i| i < n), "索引越界");
            assert_eq!(t.mesh.positions.len(), t.mesh.normals.len());
            assert_eq!(t.mesh.positions.len(), t.mesh.colors.len());
        }
        // 块号必须唯一且有序（可复现）
        let ids: Vec<u64> = tiles.iter().map(|t| t.tile_id).collect();
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(ids, sorted, "块号必须升序且无重复");
    }

    #[test]
    fn split_is_deterministic() {
        let m = quad();
        let a = split_tiles(&m, 0.5);
        let b = split_tiles(&m, 0.5);
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.tile_id, y.tile_id);
            assert_eq!(x.mesh.indices, y.mesh.indices);
        }
    }

    #[test]
    fn upload_budget_never_returns_zero() {
        let mut m = MeshData::default();
        for i in 0..30_000 {
            m.positions.push([i as f32, 0.0, 0.0]);
            m.normals.push([0.0, 0.0, 1.0]);
        }
        // 3 万个顶点 → 1 万个三角形（索引进不了 3 万，否则是测试数据自己越界）
        m.indices = (0..90_000u32).map(|i| i % 30_000).collect();
        let tiles = split_tiles(&m, 1000.0);
        assert!(tiles_per_frame(&tiles) >= 1, "再大也得至少传一块，否则永远卡住");
        assert_eq!(tiles_per_frame(&[]), 0);
    }
}
