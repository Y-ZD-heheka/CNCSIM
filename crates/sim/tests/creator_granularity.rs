//! 溯源粒度的四档对照：每叶 / 2³ / 4³ / 每体素。
//!
//! 量四件事：
//!
//! | 指标 | 怎么量 |
//! |---|---|
//! | 溢出（**最坏情况**） | 六个不同切深各切一条槽，取"切色顶点最低 z 比切深面低多少"的最大值 |
//! | 内存 | `creator_bytes()` |
//! | 切削耗时 | 整段 `run()`（六个切深取平均） |
//! | 顶点数 | 确认四档的几何完全一样（只有颜色不同） |
//!
//! 为什么要测六个切深：溢出 = `cut_z − 8h·floor((cut_z − band)/8h)`，
//! 随切深**台阶式跳变**。只测一个深度会碰巧落在好位置，得出偏乐观的结论。
//!
//! ```bash
//! cargo test --release -p cncsim-sim --test creator_granularity -- --nocapture
//! ```

use cncsim_gcode::{InterpConfig, interpret};
use cncsim_sim::demo;
use cncsim_sim::run::{RunConfig, run};

const SUBS: [u8; 4] = [1, 2, 4, 8];
/// 六个切深（mm）。刀尖落在这里，量染色区域比它低多少。
const DEPTHS: [f64; 6] = [11.0, 10.9, 10.7, 10.5, 10.2, 9.9];

fn run_slot(cut_z: f64, sub: u8) -> (cncsim_voxel::MeshData, usize, usize, f64) {
    let src = format!("G0 Z20\nG0 X-10 Y10\nG0 Z{cut_z}\nG1 X40 F800\nM30\n");
    let ic = InterpConfig { tool_radii: vec![0.0, 3.0], ..Default::default() };
    let prog = interpret(&src, &ic);
    let cfg = RunConfig {
        stock_size: [300, 300, 120],
        design: None,
        creator_sub: sub,
        ..Default::default()
    };
    let t0 = std::time::Instant::now();
    let out = run(&prog, &demo::tool(), &cfg);
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    // k=1：不降采样，否则量化会把溢出的最低顶点抬到 0.3 mm 一档
    let mesh = out.grid.extract_surface();
    (mesh, out.grid.creator_bytes(), out.grid.leaf_count(), ms)
}

#[test]
fn granularity_tradeoff() {
    println!();
    println!(
        "{:>4}  {:>10}  {:>9}  {:>7}  {:>10}  {:>9}",
        "sub", "最坏溢出mm", "creator B", "B/叶", "run ms/次", "顶点"
    );
    let mut rows = Vec::new();
    for sub in SUBS {
        let mut worst = 0.0f64;
        let (mut bytes, mut leaves, mut ms, mut verts) = (0usize, 0usize, 0.0f64, 0usize);
        for cut_z in DEPTHS {
            let (mesh, b, l, t) = run_slot(cut_z, sub);
            let lowest = mesh
                .positions
                .iter()
                .zip(&mesh.colors)
                .filter(|(_, c)| c[0] != 0)
                .map(|(p, _)| p[2] as f64)
                .fold(f64::INFINITY, f64::min);
            assert!(
                lowest.is_finite(),
                "sub={sub} 切深 {cut_z}：一个切色顶点都没有，标记失效"
            );
            worst = worst.max(cut_z - lowest);
            bytes = b;
            leaves = l;
            ms += t / DEPTHS.len() as f64;
            verts = mesh.positions.len();
        }
        println!(
            "{sub:>4}  {worst:>10.3}  {bytes:>9}  {:>7}  {ms:>10.1}  {verts:>9}",
            bytes / leaves.max(1)
        );
        rows.push((sub, worst, bytes, leaves, verts));
    }

    // ① 粒度越细，溢出不能变大
    for w in rows.windows(2) {
        assert!(
            w[1].1 <= w[0].1 + 1e-9,
            "sub={} 的溢出 {:.3} 比 sub={} 的 {:.3} 还大",
            w[1].0,
            w[1].1,
            w[0].0,
            w[0].1
        );
    }
    // ② 四档的**几何**必须逐点相同——只有颜色不同
    for w in rows.windows(2) {
        assert_eq!(
            w[0].4, w[1].4,
            "sub={} 与 sub={} 的顶点数不同：粒度只该影响颜色，不该影响几何",
            w[0].0, w[1].0
        );
    }
    // ③ 最细那档必须明显优于最粗那档，否则这套参数化不值得留
    let (_, coarse, _, _, _) = rows[0];
    let (_, fine, _, _, _) = *rows.last().unwrap();
    assert!(
        fine <= coarse * 0.75,
        "最细粒度（{fine:.3}）相比最粗（{coarse:.3}）没有明显改善，参数化没意义"
    );
}
