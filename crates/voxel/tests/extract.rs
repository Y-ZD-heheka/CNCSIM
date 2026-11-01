//! 表面提取的集成测试。
//!
//! 三条硬指标，缺一不可：
//! 1. **水密**——每条边恰好被两个三角形共享。有洞的网格在渲染时会漏背景。
//! 2. **有向体积为正**——绕序反了体积会变负，模型看起来是内翻的。
//! 3. **体积数值对**——位置错了体积就不对。

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
                let v = g.to_value(d);
                g.set(ijk, v);
            }
        }
    }
    // 把均匀区折叠成 tile，提取时会跳过它们
    g.prune();
    g
}

/// 每条无向边必须恰好出现两次。
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
    assert!(
        bad.is_empty(),
        "网格不水密：{} 条边的共享次数不是 2（例：{:?}）",
        bad.len(),
        bad.first()
    );
}

/// 有向体积（散度定理）
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

// ───────────────────────── 平面 ─────────────────────────

#[test]
fn plane_mesh_is_watertight_and_positioned() {
    // x<4 实心，x>=4 空 → x=3|4 之间的表面在 x=0.40。
    // 这是**方块不是半空间**，六个面都有表面，所以按法向筛。
    let mut g = grid();
    for x in 0..16 {
        let v = if x < 4 { -127i8 } else { 127 };
        g.fill_box([x, 0, 0], [x + 1, 16, 16], v);
    }
    let m = g.extract_surface();
    assert!(!m.positions.is_empty());
    assert_eq!(m.skipped_quads, 0, "平面不该有洞");
    assert_eq!(m.indices.len() % 3, 0);

    let mut checked = 0;
    for (p, n) in m.positions.iter().zip(&m.normals) {
        if n[0] > 0.9 {
            assert!((p[0] - 0.40).abs() < 1e-4, "+x 面顶点应在 x=0.40，实际 {}", p[0]);
            checked += 1;
        }
    }
    assert!(checked > 0, "应存在 +x 朝向的顶点");
    // 方块是闭合体 → 可以查水密与体积
    assert_watertight(&m);
    let v = signed_volume(&m);
    assert!(v > 0.0, "绕序反了：体积为 {v}");
}

/// 球面总面积应≈4πR²。
///
/// 用闭合球而不是平面测面积——平面的离散面积在边界上天然缺半格，
/// 断言会变成"测边界处理"而不是"测提取质量"。
#[test]
fn sphere_area_matches() {
    let r = 3.0;
    let g = sphere_grid(r, DVec3::new(r, r, r));
    let m = g.extract_surface();
    let p = |i: u32| {
        let q = m.positions[i as usize];
        DVec3::new(q[0] as f64, q[1] as f64, q[2] as f64)
    };
    let mut area = 0.0;
    for t in m.indices.as_chunks::<3>().0 {
        let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
        area += (b - a).cross(c - a).length() / 2.0;
    }
    let expect = 4.0 * std::f64::consts::PI * r * r;
    assert!(
        (area - expect).abs() < 0.08 * expect,
        "球面积应≈{expect:.2}，实际 {area:.2}"
    );
}

// ───────────────────────── 球 ─────────────────────────

#[test]
fn sphere_mesh_is_watertight() {
    let g = sphere_grid(3.0, DVec3::new(3.0, 3.0, 3.0));
    let m = g.extract_surface();
    assert!(!m.positions.is_empty(), "球应有表面");
    assert_eq!(m.skipped_quads, 0, "球面不该有洞");
    assert_watertight(&m);
}

/// 有向体积为正 + 数值接近解析值。
///
/// 这一条同时抓三件事：绕序（反了体积为负）、顶点位置（偏了体积不对）、
/// 以及"漏了三角形"（体积偏小）。
#[test]
fn signed_volume_is_positive_and_matches_sphere() {
    let r = 3.0;
    let g = sphere_grid(r, DVec3::new(r, r, r));
    let m = g.extract_surface();
    let v = signed_volume(&m);
    let exact = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;

    assert!(v > 0.0, "绕序反了：体积为 {v}（应为正）");
    let rel = (v - exact) / exact;
    // 对偶法的顶点是棱上零点的平均，凸面上会略微内缩；h=0.1、R=3 时约 −1%
    assert!(
        rel.abs() < 0.05,
        "体积应≈{exact:.2}，实际 {v:.2}（相对偏差 {:.1}%）",
        rel * 100.0
    );
}

#[test]
fn sphere_normals_point_outward() {
    let c = DVec3::new(3.0, 3.0, 3.0);
    let g = sphere_grid(3.0, c);
    let m = g.extract_surface();
    let mut bad = 0;
    for (p, n) in m.positions.iter().zip(&m.normals) {
        let radial = DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64) - c;
        let dot = radial.normalize().dot(DVec3::new(n[0] as f64, n[1] as f64, n[2] as f64));
        if dot < 0.5 {
            bad += 1;
        }
    }
    assert_eq!(bad, 0, "{bad} 个顶点的法向没有朝外");
}

#[test]
fn sphere_vertices_lie_on_the_surface() {
    let c = DVec3::new(3.0, 3.0, 3.0);
    let g = sphere_grid(3.0, c);
    let m = g.extract_surface();
    let mut worst: f64 = 0.0;
    for p in &m.positions {
        let r = (DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64) - c).length();
        worst = worst.max((r - 3.0).abs());
    }
    // 对偶法的顶点在格内，误差是 O(h)；h=0.1 时 0.05 以内
    assert!(worst < 0.05, "顶点离球面最远 {worst:.4} mm，超出 h/2");
}

// ───────────────────────── 切削后 ─────────────────────────

#[test]
fn cut_slot_surface_is_watertight() {
    let mut g = grid();
    // 12×12×8 mm 的实心块
    g.fill_box([0, 0, 0], [120, 120, 80], -127);
    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &Pose {
            tip: DVec3::new(0.0, 6.0, 7.0),
            axis: DVec3::Z,
        },
        &Pose {
            tip: DVec3::new(12.0, 6.0, 7.0),
            axis: DVec3::Z,
        },
        &prof,
    );
    let m = g.extract_surface();
    assert!(!m.positions.is_empty(), "切了槽应有表面");
    assert_eq!(m.skipped_quads, 0, "切槽后不该有洞");
    assert_watertight(&m);

    // 槽底应在 z=7
    let on_bottom: Vec<f64> = m
        .positions
        .iter()
        .filter(|p| (p[2] - 7.0).abs() < 0.05 && (p[1] - 6.0).abs() < 1.0)
        .map(|p| p[2] as f64)
        .collect();
    assert!(!on_bottom.is_empty(), "应提取出槽底");
    let mean = on_bottom.iter().sum::<f64>() / on_bottom.len() as f64;
    assert!((mean - 7.0).abs() < 0.02, "槽底平均高度 {mean:.4} 应≈7");
}

#[test]
fn extraction_is_deterministic() {
    let g = sphere_grid(2.5, DVec3::new(3.0, 3.0, 3.0));
    let a = g.extract_surface();
    let b = g.extract_surface();
    assert_eq!(a.positions, b.positions, "两次提取的顶点必须逐位相同");
    assert_eq!(a.indices, b.indices, "两次提取的索引必须逐位相同");
}

#[test]
fn empty_grid_produces_empty_mesh() {
    let g = grid();
    let m = g.extract_surface();
    assert!(m.positions.is_empty() && m.indices.is_empty());
}

/// 实心块在背景场里有外表面——应当是**闭合**的盒子。
#[test]
fn solid_block_produces_closed_box() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [32, 32, 32], -127);
    let m = g.extract_surface();
    assert!(!m.positions.is_empty(), "实心块应有外表面");
    assert_eq!(m.skipped_quads, 0, "闭合块不该有洞");
    assert_watertight(&m);
    let v = signed_volume(&m);
    let expect = 3.2_f64.powi(3);
    assert!(v > 0.0, "绕序反了：体积 {v}");
    assert!(
        (v - expect).abs() < 0.10 * expect,
        "体积应≈{expect:.2}，实际 {v:.2}"
    );
}

#[test]
fn colors_are_filled_for_every_vertex() {
    let g = sphere_grid(2.0, DVec3::new(3.0, 3.0, 3.0));
    let m = g.extract_surface();
    assert_eq!(m.colors.len(), m.positions.len(), "每个顶点都要有颜色");
    assert_eq!(m.normals.len(), m.positions.len());
}

/// 性能基准（默认跳过）：200×200×100 mm 毛坯 @ h=0.1。
///
/// ```bash
/// cargo test --release -p cncsim-voxel --test extract -- --ignored --nocapture perf
/// ```
#[test]
#[ignore = "性能基准，几十秒，默认不跑"]
fn perf_full_stock_extraction() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    let t0 = std::time::Instant::now();
    let m = g.extract_surface();
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    println!(
        "全量提取：顶点 {}  三角形 {}  丢洞 {}  耗时 {ms:.0} ms",
        m.positions.len(),
        m.indices.len() / 3,
        m.skipped_quads
    );
    assert_eq!(m.skipped_quads, 0);
}

/// **剖析专用**：重复 5 轮细格提取，给 VTune 足够长的采样窗口。
///
/// 单跑一次 200mm 毛坯只有 ~800 ms，user-mode 采样（默认 ~10 ms）只拿到几十个
/// 样本，热点表里全是噪声。
#[test]
#[ignore = "剖析专用（重复 5 轮），只给 VTune 用"]
fn perf_extract_loop() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    let rounds = 5;
    let t0 = std::time::Instant::now();
    let mut sink = 0usize;
    for _ in 0..rounds {
        let m = g.extract_surface();
        sink += m.positions.len() + m.indices.len() + m.skipped_quads;
    }
    let dt = t0.elapsed().as_secs_f64();
    println!(
        "提取剖析基准：{rounds} 轮，合计 {:.0} ms（每轮 {:.0} ms）；sink={sink}",
        dt * 1e3,
        dt * 1e3 / rounds as f64
    );
}
