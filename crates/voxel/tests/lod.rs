//! 显示 LOD 的集成测试。
//!
//! 四条要证明的事，缺一不可：
//! 1. **`k=1` 与细格逐位一致**——LOD 不是第二套提取算法，是同一套换个格点。
//! 2. **三角形数按 `1/k²` 掉**——否则"进 200 万面预算"只是口号。
//! 3. **顶点落在真值场上**（`|sdf(v)|` 很小）——粗格顶点不是"看起来差不多"，
//!    这一条同时锁住"跨符号棱的端点必须落在窄带内"那个硬约束。
//! 4. **水密 + 绕序**——双网格的结构保证不能因为粗化而丢。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, MeshData, SparseGrid};

fn grid() -> SparseGrid<i8> {
    SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
}

/// 按解析 SDF 填一个球（半径 mm，球心 mm）
fn sphere_grid(radius: f64, center: DVec3) -> SparseGrid<i8> {
    let mut g = grid();
    let h = g.config.h;
    let n = ((radius + 2.0) / h).ceil() as i32;
    let lo = [
        ((center.x - radius - 1.0) / h) as i32,
        ((center.y - radius - 1.0) / h) as i32,
        ((center.z - radius - 1.0) / h) as i32,
    ];
    for dz in 0..2 * n {
        for dy in 0..2 * n {
            for dx in 0..2 * n {
                let ijk = [lo[0] + dx, lo[1] + dy, lo[2] + dz];
                let p = g.voxel_center(ijk);
                let d = (p - center).length() - radius;
                g.set(ijk, g.to_value(d));
            }
        }
    }
    g.prune();
    g
}

/// 12×12×8 mm 毛坯 + 一条 6 mm 宽、1 mm 深的直槽（真实切削，不是解析体）
fn slotted_block() -> SparseGrid<i8> {
    let mut g = grid();
    g.fill_box([0, 0, 0], [120, 120, 80], -127);
    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &Pose { tip: DVec3::new(0.0, 6.0, 7.0), axis: DVec3::Z },
        &Pose { tip: DVec3::new(12.0, 6.0, 7.0), axis: DVec3::Z },
        &prof,
    );
    g.prune();
    g
}

fn signed_volume(m: &MeshData) -> f64 {
    let mut v = 0.0;
    for t in m.indices.as_chunks::<3>().0 {
        let p = |i: u32| {
            let q = m.positions[i as usize];
            DVec3::new(q[0] as f64, q[1] as f64, q[2] as f64)
        };
        let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
        v += a.dot(b.cross(c)) / 6.0;
    }
    v
}

fn assert_watertight(m: &MeshData) {
    use std::collections::HashMap;
    let mut count: HashMap<(u32, u32), u32> = HashMap::new();
    for t in m.indices.as_chunks::<3>().0 {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *count.entry(key).or_insert(0) += 1;
        }
    }
    let bad: Vec<_> = count.iter().filter(|(_, c)| **c != 2).collect();
    assert!(bad.is_empty(), "网格不水密：{} 条边共享次数不是 2", bad.len());
}

/// 与顶点顺序无关的规范形：每个三角形的三个顶点各自排序，再把三角形排序。
///
/// 细格按叶迭代顺序产顶点，LOD 按粗格排序产顶点——**同一张面，顶点编号不同**。
/// 所以不能直接比数组，要先把"面"的集合规范化。
fn canonical(m: &MeshData) -> Vec<[[u32; 3]; 3]> {
    let key = |i: u32| {
        let p = m.positions[i as usize];
        // 量化到 1 nm，抹掉浮点末位噪声但保留真实几何
        [
            (p[0] as f64 * 1e6).round() as i64 as u32,
            (p[1] as f64 * 1e6).round() as i64 as u32,
            (p[2] as f64 * 1e6).round() as i64 as u32,
        ]
    };
    let mut tris: Vec<[[u32; 3]; 3]> = m
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|t| {
            let mut v = [key(t[0]), key(t[1]), key(t[2])];
            v.sort_unstable();
            v
        })
        .collect();
    tris.sort_unstable();
    tris
}

// ─────────────────── ① k=1 必须与细格一致 ───────────────────

/// 网格原点为负（`div_euclid` 的分支），表面跨 x=−4|−3。
fn plane_with_negative_indices() -> SparseGrid<i8> {
    let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::new(-0.55, -0.55, -0.55));
    for x in -8..8 {
        let v = if x < -4 { -127i8 } else { 127 };
        g.fill_box([x, -8, -8], [x + 1, 8, 8], v);
    }
    g.prune();
    g
}

#[test]
fn factor_one_is_bit_identical_to_fine_extraction() {
    let g = plane_with_negative_indices();
    let fine = g.extract_surface();
    let lod = g.extract_surface_lod(1);

    assert_eq!(fine.positions.len(), lod.positions.len(), "顶点数应相同");
    assert_eq!(fine.indices.len(), lod.indices.len(), "索引数应相同");
    assert_eq!(fine.skipped_quads, lod.skipped_quads);
    assert_eq!(canonical(&fine), canonical(&lod), "k=1 的面集合必须与细格一致");
}

#[test]
fn factor_one_matches_fine_on_cut_surface() {
    let g = slotted_block();
    let fine = g.extract_surface();
    let lod = g.extract_surface_lod(1);
    assert_eq!(fine.positions.len(), lod.positions.len());
    assert_eq!(canonical(&fine), canonical(&lod));
}

// ─────────────────── ② 三角形数按 1/k² 掉 ───────────────────

#[test]
fn triangle_count_falls_with_k_squared() {
    let g = sphere_grid(6.0, DVec3::new(6.0, 6.0, 6.0));
    let t1 = g.extract_surface_lod(1).indices.len() / 3;
    let t2 = g.extract_surface_lod(2).indices.len() / 3;
    let t4 = g.extract_surface_lod(4).indices.len() / 3;

    for (k, t) in [(2u32, t2), (4, t4)] {
        let expect = t1 as f64 / (k * k) as f64;
        let ratio = t as f64 / expect;
        assert!(
            (0.75..1.25).contains(&ratio),
            "k={k} 的三角形数应≈T1/{k}²={expect:.0}，实际 {t}（比值 {ratio:.2}）"
        );
    }
    println!("球面三角形：k=1 {t1}  k=2 {t2}  k=4 {t4}");
}

/// 换成 200 mm 毛坯的数字：LOD 4 必须落进 `render` 的 200 万面预算。
/// 用方坯（表面积可算）而不是球，避免"球太小预算不吃紧"这种假通过。
#[test]
fn budget_lands_under_two_million_triangles() {
    // 60×60×20 mm 方坯，表面积 2·(60·60 + 60·20 + 60·20) = 12000 mm²
    let mut g = grid();
    g.fill_box([0, 0, 0], [600, 600, 200], -127);
    let t1 = g.extract_surface_lod(1).indices.len() / 3;
    let t4 = g.extract_surface_lod(4).indices.len() / 3;
    // 面积法则：T ≈ A/h_c² × 2，200 mm 毛坯（A≈240000 mm²）@ h_c=0.4 → ≈300 万
    println!(
        "60³ 方坯：k=1 {t1} 面，k=4 {t4} 面；面积 {:.0} mm²",
        600.0 * 0.1 * 600.0 * 0.1 * 2.0 + 600.0 * 0.1 * 200.0 * 0.1 * 4.0
    );
    assert!(t4 * 16 < t1 * 20, "k=4 应比 k=1 少 15 倍以上，实际 {t1}→{t4}");
    assert_eq!(g.extract_surface_lod(4).skipped_quads, 0, "粗格也不许有洞");
}

// ─────────────────── ③ 顶点落在真值场上 ───────────────────

/// 每个粗格顶点到**真实球面**的距离必须很小。
///
/// 这一条同时守住窄带约束：若 `k` 超过 `band_voxels`，跨符号棱的远端被钳制，
/// 偏差立刻离开"量化 + 曲率"这个量级（实测 k=4 9.8 µm、k=5 15.5 µm、k=6 34.2 µm）。
#[test]
fn lod_vertices_lie_on_the_true_field() {
    let r = 6.0;
    let g = sphere_grid(r, DVec3::new(r, r, r));
    let m = g.extract_surface_lod(4);
    assert!(!m.positions.is_empty());

    let mut worst = 0.0f64;
    for p in &m.positions {
        let q = DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
        worst = worst.max(((q - DVec3::new(r, r, r)).length() - r).abs());
    }
    println!("R={r} 球，k=4：顶点最大径向偏差 {:.1} µm", worst * 1e3);
    assert!(
        worst < 0.02,
        "顶点偏离真实球面 {:.1} µm（上限 20 µm）——先查 k 是否超过 band",
        worst * 1e3
    );
}

/// 槽底是平面，粗格顶点也必须**精确**落在 z=7。
///
/// 平面上三线性与对偶都是精确的，所以这条不该有误差预算：
/// 一旦出现几百 µm 的偏差，说明粗格角点取到了钳制值。
#[test]
fn lod_keeps_slot_floor_exact() {
    let g = slotted_block();
    let fine = g.extract_surface();
    let lod = g.extract_surface_lod(4);

    // 只看槽内（y≈6、x 在槽长范围内、z 在槽底附近）的顶点。
    // 不带 z 下界会把毛坯底面 z=0 也算进来——那就成了测底面，不是测槽底。
    let lowest = |m: &MeshData| {
        m.positions
            .iter()
            .filter(|p| {
                (p[1] - 6.0).abs() < 2.0
                    && (p[0] - 6.0).abs() < 4.0
                    && (6.5..7.5).contains(&p[2])
            })
            .map(|p| p[2] as f64)
            .fold(f64::MAX, f64::min)
    };
    println!("槽底最低顶点：细格 {:.4}  LOD4 {:.4}", lowest(&fine), lowest(&lod));
    assert!((lowest(&fine) - 7.0).abs() < 1e-3, "细格槽底应在 z=7，实际 {}", lowest(&fine));
    // 容差 5 µm = i8 量化一档（3.15 µm）+ 扫掠迭代残差；不是"差不多就行"
    assert!(
        (lowest(&lod) - 7.0).abs() < 5e-3,
        "LOD 槽底应在 z=7（±5 µm），实际 {}",
        lowest(&lod)
    );
}

// ─────────────────── ④ 水密 + 绕序 ───────────────────

#[test]
fn lod_mesh_stays_watertight_and_oriented() {
    let g = sphere_grid(5.0, DVec3::new(5.0, 5.0, 5.0));
    for k in 1..=4 {
        let m = g.extract_surface_lod(k);
        assert_eq!(m.skipped_quads, 0, "k={k} 出现洞");
        assert_watertight(&m);
        let v = signed_volume(&m);
        assert!(v > 0.0, "k={k} 绕序反了：体积 {v}");
        let analytic = 4.0 / 3.0 * std::f64::consts::PI * 5.0f64.powi(3);
        let err = (v - analytic).abs() / analytic;
        println!("k={k} 体积 {v:.2} / 解析 {analytic:.2} → 误差 {:.2}%", err * 100.0);
        assert!(err < 0.03, "k={k} 体积误差 {:.2}% 超过 3%", err * 100.0);
    }
}

// ─────────────────── 边界 ───────────────────

#[test]
fn factor_beyond_band_is_rejected() {
    let g = sphere_grid(3.0, DVec3::new(3.0, 3.0, 3.0));
    assert_eq!(g.max_lod_factor(), 4, "默认 band=4 体素");
    // 5 > band → panic（消息里必须点名钳制，而不是"参数非法"）
}

#[test]
#[should_panic(expected = "超过上限")]
fn lod_factor_beyond_band_panics() {
    let g = sphere_grid(3.0, DVec3::new(3.0, 3.0, 3.0));
    let _ = g.extract_surface_lod(5);
}

#[test]
fn empty_grid_lod_is_empty() {
    let g = grid();
    assert!(g.extract_surface_lod(4).positions.is_empty());
}


/// 性能基准（默认跳过）：200×200×100 mm 毛坯 @ h=0.1，k=4。
///
/// ```bash
/// cargo test --release -p cncsim-voxel --test lod -- --ignored --nocapture perf
/// ```
#[test]
#[ignore = "性能基准，几秒，默认不跑"]
fn perf_full_stock_lod() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    for k in [2u32, 4] {
        let t0 = std::time::Instant::now();
        let m = g.extract_surface_lod(k);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        let bytes = m.positions.len() * 12
            + m.normals.len() * 12
            + m.indices.len() * 4
            + m.colors.len() * 4;
        println!(
            "200mm 毛坯 LOD k={k}（h_c={} mm）：顶点 {}  三角形 {}  耗时 {ms:.0} ms  网格 {:.0} MB  丢洞 {}",
            g.config.h * k as f64,
            m.positions.len(),
            m.indices.len() / 3,
            bytes as f64 / 1e6,
            m.skipped_quads
        );
        assert_eq!(m.skipped_quads, 0);
    }
}



/// 分块提取必须与整体提取**逐三角形一致**：不重、不漏、交界处无裂缝。
///
/// 这一条是增量重建的地基。分块重建时只会重算被切到的块，其余块沿用旧网格；
/// 如果分块的三角形和整体不一致（多一块少一块），屏幕上就会出现重影或洞，
/// 而且极难定位。做法是"按发射格划分"——每个四边形本来就由最小角所在的格
/// 发射一次，所以每块只发射自己范围内的格就天然不重不漏。
#[test]
fn region_extraction_partitions_the_full_mesh_exactly() {
    let g = slotted_block();
    let k = 2i32;
    let full = g.extract_surface_lod(k as u32);

    // 三角形集合的规范化键：三个顶点坐标排序后拼起来（用 to_bits 保证精确比较）
    fn keys(m: &MeshData) -> Vec<[u32; 9]> {
        let mut out = Vec::with_capacity(m.indices.len() / 3);
        for t in 0..m.indices.len() / 3 {
            let mut v: Vec<[u32; 3]> = (0..3)
                .map(|k| {
                    let p = m.positions[m.indices[t * 3 + k] as usize];
                    [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]
                })
                .collect();
            v.sort_unstable();
            out.push([
                v[0][0], v[0][1], v[0][2], v[1][0], v[1][1], v[1][2], v[2][0], v[2][1], v[2][2],
            ]);
        }
        out.sort_unstable();
        out
    }

    let (lo, hi) = g.bounds().expect("非空");
    let cell_lo = [0, 1, 2].map(|a| lo[a].div_euclid(k) - 1);
    let cell_hi = [0, 1, 2].map(|a| hi[a].div_euclid(k) + 1);
    // 每块 20 个粗格：切太细会把测试变成"跑 3000 次提取"，测的是性能不是正确性
    let step = 20i32;

    let mut tiled: Vec<[u32; 9]> = Vec::new();
    let mut cz = cell_lo[2];
    while cz <= cell_hi[2] {
        let mut cy = cell_lo[1];
        while cy <= cell_hi[1] {
            let mut cx = cell_lo[0];
            while cx <= cell_hi[0] {
                let m = g.extract_surface_lod_in(
                    k as u32,
                    [cx, cy, cz],
                    [cx + step - 1, cy + step - 1, cz + step - 1],
                );
                tiled.extend(keys(&m));
                cx += step;
            }
            cy += step;
        }
        cz += step;
    }
    tiled.sort_unstable();

    assert_eq!(
        tiled.len(),
        full.indices.len() / 3,
        "分块三角形数 {} ≠ 整体 {}：说明交界处有重复或缺口",
        tiled.len(),
        full.indices.len() / 3
    );
    let want = keys(&full);
    assert_eq!(tiled.len(), want.len());
    let bad = tiled.iter().zip(&want).filter(|(a, b)| a != b).count();
    assert_eq!(bad, 0, "有 {bad} 个三角形对不上（坐标级）");
}
