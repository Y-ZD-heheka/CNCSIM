//! 外部 STL → 体素 SDF → 重建网格，导出自包含 HTML 预览。
//!
//! ```bash
//! cargo run --release -p cncsim-render --example stl_roundtrip -- model.stl 0.1 4
//! #                                                            路径      h   k
//! ```
//!
//! 默认 `h = 0.2 mm`、`k = 4`（LOD 上限 = 窄带宽度 4 体素）。
//!
//! 输出 `out/stl_roundtrip.html`：单文件、零外链、浏览器直接打开。
//! 顶点色按 ADR-005 的调色板索引编码写进页面（预览着色器会展开）。

use cncsim_math::DVec3;
use cncsim_voxel::{GridConfig, ValueType};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = match args.next() {
        Some(p) => p,
        None => {
            eprintln!("用法：stl_roundtrip <model.stl> [h_mm=0.2] [k=4]");
            std::process::exit(2);
        }
    };
    let h: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.2);
    let k: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(4);

    // ── 解析 ──
    let t0 = std::time::Instant::now();
    let bytes = std::fs::read(&path).expect("读 STL 失败");
    let tris = cncsim_voxel::stl::parse(&bytes).expect("解析 STL 失败");
    println!(
        "{}：{:.1} MB，{} 三角形，解析 {:.0} ms",
        path,
        bytes.len() as f64 / 1048576.0,
        tris.len(),
        t0.elapsed().as_secs_f64() * 1e3
    );

    // ── 包围盒 → 域 ──
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for t in &tris {
        for v in t {
            lo = DVec3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
            hi = DVec3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
        }
    }
    let pad = 4;
    let origin = lo - DVec3::splat(pad as f64 * h);
    let domain = [
        ((hi.x - lo.x) / h).ceil() as i32 + 1 + 2 * pad,
        ((hi.y - lo.y) / h).ceil() as i32 + 1 + 2 * pad,
        ((hi.z - lo.z) / h).ceil() as i32 + 1 + 2 * pad,
    ];
    println!(
        "模型 {:.2}×{:.2}×{:.2} mm → 域 {}×{}×{}（h={h} mm）",
        hi.x - lo.x,
        hi.y - lo.y,
        hi.z - lo.z,
        domain[0],
        domain[1],
        domain[2]
    );

    // ── 体素化 ──
    let t1 = std::time::Instant::now();
    let g = cncsim_voxel::meshvol::mesh_to_field(
        &tris,
        GridConfig {
            h,
            band_voxels: 4,
            value_type: ValueType::I8,
        },
        origin,
        domain,
    );
    println!(
        "体素化 {:.0} ms：{} 叶，材料 {:.1} mm³",
        t1.elapsed().as_secs_f64() * 1e3,
        g.leaf_count(),
        g.material_volume()
    );

    // ── 提取 + 导出 ──
    let t2 = std::time::Instant::now();
    let mesh = if k > 1 && k <= g.max_lod_factor() {
        g.extract_surface_lod(k)
    } else {
        g.extract_surface()
    };
    println!(
        "提取 {:.0} ms：{} 顶点 / {} 三角形（丢洞 {}）",
        t2.elapsed().as_secs_f64() * 1e3,
        mesh.positions.len(),
        mesh.indices.len() / 3,
        mesh.skipped_quads
    );

    std::fs::create_dir_all("out").ok();
    let out = std::path::Path::new("out/stl_roundtrip.html");
    match cncsim_render::preview::write_html(out, &mesh, "STL → 体素 → 网格") {
        Ok(stats) => println!(
            "已写出 {}（{:.1} MB，{} 三角形）",
            out.display(),
            stats.bytes as f64 / 1048576.0,
            stats.triangles
        ),
        Err(e) => {
            eprintln!("写 HTML 失败：{e}");
            std::process::exit(1);
        }
    }
}
