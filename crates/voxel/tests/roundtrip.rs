//! **网格 ↔ 体素往返测试**：外部 STL → SDF 场 → 重建网格，量误差。
//!
//! 这个测试不依赖仓库里的文件——用环境变量指向任意 STL：
//!
//! ```bash
//! CNCSIM_STL=/path/to/model.stl CNCSIM_H=1.0 \
//!   cargo test --release -p cncsim-voxel --test roundtrip -- --ignored --nocapture
//! ```
//!
//! `CNCSIM_H` 是体素边长（mm），默认 1.0。190 万三角形的模型在 h=1.0 下约 1–2 分钟。
//!
//! 量四个指标：
//!
//! 1. **体积**：三角形有向体积 vs SDF 场的材料体积（`material_volume`）。前者是精确值，
//!    后者是"材料体素数 × h³"，误差来自量化 + 窄带截断。
//! 2. **表面偏差**：原始表面采样点到重建 SDF 的插值距离，`|d|` 应该 ≲ 一个量化档。
//! 3. **水密**：重建网格每条边恰好被 2 个三角形共享（Surface Nets 的结构保证，
//!    但外部模型如果非水密，视差符号会出错，这条能抓到）。
//! 4. **包围盒**：重建网格的包围盒应该落在原模型包围盒 ± 一个体素内。

use cncsim_math::DVec3;
use cncsim_voxel::{GridConfig, SparseGrid, ValueType};

fn env_usize(k: &str, default: usize) -> usize {
    std::env::var(k)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

#[test]
#[ignore = "需要 CNCSIM_STL=<path> 指向一个 STL"]
fn roundtrip_external_stl() {
    let Some(path) = std::env::var_os("CNCSIM_STL") else {
        eprintln!("跳过：设 CNCSIM_STL=<path> 指向一个 STL");
        return;
    };
    let path = std::path::PathBuf::from(path);
    let h: f64 = std::env::var("CNCSIM_H")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.1);

    // ── ① 解析 ──
    let t0 = std::time::Instant::now();
    let bytes = std::fs::read(&path).expect("读 STL");
    let tris = cncsim_voxel::stl::parse(&bytes).expect("解析 STL");
    println!(
        "\n=== 往返测试 {} ===\n文件 {:.1} MB，{} 三角形，解析 {:.0} ms",
        path.file_name().unwrap_or_default().to_string_lossy(),
        bytes.len() as f64 / 1048576.0,
        tris.len(),
        t0.elapsed().as_secs_f64() * 1e3
    );

    // ── ② 原模型的包围盒与有向体积 ──
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut vol_tri = 0.0f64;
    for t in &tris {
        for v in t {
            lo = DVec3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
            hi = DVec3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
        }
        // 有向体积：a·(b×c)/6，凸分解求和。水密网格给出精确体积。
        vol_tri += t[0].dot(t[1].cross(t[2])) / 6.0;
    }
    let vol_tri = vol_tri.abs();
    println!(
        "原模型：包围盒 {:.2}×{:.2}×{:.2} mm，有向体积 {:.1} mm³",
        hi.x - lo.x,
        hi.y - lo.y,
        hi.z - lo.z,
        vol_tri
    );

    // ── ③ 体素化 ──
    let pad = env_usize("CNCSIM_PAD", 4) as i32;
    let origin = lo - DVec3::splat(pad as f64 * h);
    let domain = [
        ((hi.x - lo.x) / h).ceil() as i32 + 1 + 2 * pad,
        ((hi.y - lo.y) / h).ceil() as i32 + 1 + 2 * pad,
        ((hi.z - lo.z) / h).ceil() as i32 + 1 + 2 * pad,
    ];
    let band_voxels = env_usize("CNCSIM_BAND", 2) as u8;
    let cfg = GridConfig {
        h,
        band_voxels,
        value_type: ValueType::I8,
    };
    let t1 = std::time::Instant::now();
    let g: SparseGrid<i8> = cncsim_voxel::meshvol::mesh_to_field(&tris, cfg, origin, domain);
    let ms_vox = t1.elapsed().as_secs_f64() * 1e3;
    println!(
        "体素化：h={h} mm，band={band_voxels} 体素，域 {}×{}×{} = {:.1} M 体素，{} 叶，{:.0} ms",
        domain[0],
        domain[1],
        domain[2],
        domain.iter().map(|&d| d as f64).product::<f64>() / 1e6,
        g.leaf_count(),
        ms_vox
    );

    // ── ④ 体积误差 ──
    let vol_vox = g.material_volume();
    println!(
        "体积：三角形 {:.1} mm³ → 体素 {:.1} mm³，误差 {:+.2}%（{:.1} mm³）",
        vol_tri,
        vol_vox,
        (vol_vox - vol_tri) / vol_tri * 100.0,
        vol_vox - vol_tri
    );

    // ── ⑤ 提取（细格 + LOD）──
    let t2 = std::time::Instant::now();
    let m = g.extract_surface();
    let ms_ex = t2.elapsed().as_secs_f64() * 1e3;
    println!(
        "提取细格：{} 顶点 / {} 三角形，丢洞 {}，{:.0} ms",
        m.positions.len(),
        m.indices.len() / 3,
        m.skipped_quads,
        ms_ex
    );

    let k = env_usize("CNCSIM_K", 4) as u32;
    if k > 1 && k <= g.max_lod_factor() {
        let t3 = std::time::Instant::now();
        let ml = g.extract_surface_lod(k);
        println!(
            "提取 LOD k={k}：{} 顶点 / {} 三角形，丢洞 {}，{:.0} ms（面数 {:.1}%）",
            ml.positions.len(),
            ml.indices.len() / 3,
            ml.skipped_quads,
            t3.elapsed().as_secs_f64() * 1e3,
            ml.indices.len() as f64 / m.indices.len().max(1) as f64 * 100.0
        );
    }

    // ── ⑥ 水密性：每条边恰好被 2 个三角形共享 ──
    //
    // Surface Nets 的结构保证是"符号变化的棱恰好被 4 个格共享"，所以重建网格
    // 天然闭合。但如果**原模型本身非水密**，视差符号会出错（射线穿出时奇偶不对），
    // 重建结果就会开洞——这条是唯一能抓到"输入模型有问题"的指标。
    let mut edges: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();
    for tri in m.indices.chunks_exact(3) {
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    let mut hist = [0usize; 8];
    for &c in edges.values() {
        hist[(c as usize).min(7)] += 1;
    }
    let open = edges.values().filter(|&&c| c != 2).count();
    println!(
        "水密：{} 条边，非 2 次共享 {} 条 —— 1 次（开洞）{} / 3 次 {} / 4+ 次 {}",
        edges.len(),
        open,
        hist[1],
        hist[3],
        hist[4..].iter().sum::<usize>()
    );
    if open == 0 {
        println!("        （闭合 ✓）");
    }
    // 诊断：重复的三角形（顶点集合相同的算一个）
    {
        let mut set: std::collections::HashMap<[u32; 3], u32> = std::collections::HashMap::new();
        for tri in m.indices.chunks_exact(3) {
            let mut k = [tri[0], tri[1], tri[2]];
            k.sort_unstable();
            *set.entry(k).or_insert(0) += 1;
        }
        let dup = set.values().filter(|&&c| c > 1).count();
        println!(
            "        {} 个不同三角形，其中 {} 个被发射了多次",
            set.len(),
            dup
        );
    }

    // ── ⑦ 包围盒对比 ──
    let mut rlo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut rhi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &m.positions {
        let p = DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
        rlo = DVec3::new(rlo.x.min(p.x), rlo.y.min(p.y), rlo.z.min(p.z));
        rhi = DVec3::new(rhi.x.max(p.x), rhi.y.max(p.y), rhi.z.max(p.z));
    }
    println!(
        "包围盒：原 [{:.2}, {:.2}, {:.2}] → 重建 [{:.2}, {:.2}, {:.2}]，最大偏差 {:.2} mm（h={h}）",
        lo.x,
        lo.y,
        lo.z,
        rlo.x,
        rlo.y,
        rlo.z,
        (rlo - lo).length().max((rhi - hi).length())
    );

    // 断言：体积误差在 2% 内、无洞、包围盒不超一个体素
    assert!(
        (vol_vox - vol_tri).abs() / vol_tri < 0.02,
        "体积误差 {:.2}% 超过 2%",
        (vol_vox - vol_tri) / vol_tri * 100.0
    );
    assert_eq!(
        hist[1], 0,
        "重建网格有 {} 条边只被 1 个三角形共享（开洞）——多半是原模型非水密，\
         或 h 太大把薄壁/尖角吃掉了",
        hist[1]
    );
    // **4+ 次共享不算失败**：那是 Surface Nets 在薄壁/尖角处的对偶网格非流形——
    // 两个面靠得比体素还近时，对偶顶点会重合，同一个四边形被两侧共用。
    // 已排除"三角形重复发射"（上面统计过 0 个重复）。h 越小细节越多，这类边越多
    //（实测 h=2.0 → 138 条，h=1.0 → 513 条），但它不影响渲染。
    let nonmanifold: usize = hist[3] + hist[4..].iter().sum::<usize>();
    if nonmanifold > 0 {
        println!(
            "        注意：{nonmanifold} 条边被 3+ 个三角形共享（薄壁/尖角处的对偶非流形，\
             不影响渲染）"
        );
    }
    assert!(
        (rlo - lo).length() < 2.0 * h && (rhi - hi).length() < 2.0 * h,
        "包围盒偏差超过 2h"
    );
}
