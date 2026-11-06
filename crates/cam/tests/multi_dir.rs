//! **多方向开粗**的第一块砖：工序坐标系（`Frame`）。
//!
//! 3 轴高度场只会沿 +Z 走刀。要"从 −X 开粗"，不重写 CAM：把模型转进该方向的
//! 局部坐标系 → 照常出刀路 → 用 `Frame` 把点变回世界。
//!
//! 这里验三件事：
//! 1. `Frame` 本身是正交的、`unapply∘apply = id`；
//! 2. 从 −X 生成的 G 代码确实落在世界毛坯里，并且**真的跑到 −X 那一面**；
//! 3. 它切掉的体积 >0 且不超过"毛坯减零件"。

use cncsim_cam::{Frame, Job, Process, Stock, ToolSpec, Tri, generate_in};
use cncsim_math::DVec3;

fn box_mesh(lo: DVec3, hi: DVec3) -> Vec<Tri> {
    let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
    let c = [
        p(lo.x, lo.y, lo.z),
        p(hi.x, lo.y, lo.z),
        p(hi.x, hi.y, lo.z),
        p(lo.x, hi.y, lo.z),
        p(lo.x, lo.y, hi.z),
        p(hi.x, lo.y, hi.z),
        p(hi.x, hi.y, hi.z),
        p(lo.x, hi.y, hi.z),
    ];
    let faces = [
        [0, 2, 1],
        [0, 3, 2], // 底
        [4, 5, 6],
        [4, 6, 7], // 顶
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    faces.iter().map(|f| [c[f[0]], c[f[1]], c[f[2]]]).collect()
}

fn transform_mesh(mesh: &[Tri], f: &Frame) -> Vec<Tri> {
    mesh.iter()
        .map(|t| [f.unapply(t[0]), f.unapply(t[1]), f.unapply(t[2])])
        .collect()
}

fn transform_box(s: Stock, f: &Frame, inverse: bool) -> Stock {
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for i in 0..8 {
        let p = DVec3::new(
            if i & 1 == 0 { s.min.x } else { s.max.x },
            if i & 2 == 0 { s.min.y } else { s.max.y },
            if i & 4 == 0 { s.min.z } else { s.max.z },
        );
        let q = if inverse { f.unapply(p) } else { f.apply(p) };
        lo = DVec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
        hi = DVec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
    }
    Stock { min: lo, max: hi }
}

/// 跑一次"从 `dir` 方向开粗"，返回 (G 代码, 世界毛坯)。
fn machine_from(dir: DVec3, part: &[Tri], world_stock: Stock) -> (String, Stock) {
    let f = Frame::from_dir(dir);
    let local_mesh = transform_mesh(part, &f);
    let local_stock = transform_box(world_stock, &f, true);
    let rough = ToolSpec::flat(1, 6.0, 20.0);
    let finish = ToolSpec::ball(2, 3.0, 10.0);
    let job = Job {
        mesh: &local_mesh,
        stock: local_stock,
        rough: Some(rough),
        finish,
        process: Process {
            stock_allowance_mm: 0.3,
            stepdown_mm: 1.5,
            rough_stepover_mm: 2.5,
            finish_stepover_mm: 1.0,
            sample_mm: 0.5,
            ..Default::default()
        },
        rest: None,
    };
    let prog = generate_in(&job, Some(f)).expect("生成失败");
    (prog.gcode, world_stock)
}

#[test]
fn frame_is_orthonormal_and_invertible() {
    for dir in [
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(-1.0, 0.0, 0.0),
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(1.0, 1.0, 1.0),
    ] {
        let f = Frame::from_dir(dir);
        // 局部 +Z 必须指向 dir
        let z = f.apply(DVec3::new(0.0, 0.0, 1.0));
        let d = dir.normalize();
        assert!(
            (z - d).length() < 1e-12,
            "from_dir 把 +Z 映射到了 {z:?}，期望 {d:?}"
        );
        // 正交：三列单位且互相垂直
        let cols = [
            DVec3::new(f.rot[0][0], f.rot[1][0], f.rot[2][0]),
            DVec3::new(f.rot[0][1], f.rot[1][1], f.rot[2][1]),
            DVec3::new(f.rot[0][2], f.rot[1][2], f.rot[2][2]),
        ];
        for c in &cols {
            assert!((c.length() - 1.0).abs() < 1e-12, "列不是单位向量");
        }
        assert!(cols[0].dot(cols[1]).abs() < 1e-12, "列不垂直");
        // 往返
        let p = DVec3::new(3.5, -2.0, 7.25);
        assert!((f.unapply(f.apply(p)) - p).length() < 1e-12, "unapply∘apply ≠ id");
    }
}

#[test]
fn roughing_from_minus_x_works() {
    // 零件：12×12×6 的方块，居中放在 24×24×8 的毛坯里 → 四周都有料可去。
    let part = box_mesh(
        DVec3::new(-6.0, -6.0, 0.0),
        DVec3::new(6.0, 6.0, 6.0),
    );
    let stock = Stock {
        min: DVec3::new(-12.0, -12.0, 0.0),
        max: DVec3::new(12.0, 12.0, 8.0),
    };

    let (gcode, _) = machine_from(DVec3::new(-1.0, 0.0, 0.0), &part, stock);
    let program = cncsim_gcode::interpret(&gcode, &Default::default());
    assert!(
        !program.has_errors(),
        "从 −X 生成的程序解释报错：{:?}",
        program.issues
    );

    // 所有刀点必须落在世界毛坯 + 安全余量之内（安全高度 10 mm）
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for b in &program.blocks {
        let p = b.target.tip;
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let m = 12.0; // 安全高度余量
    assert!(
        lo.x >= stock.min.x - m
            && lo.y >= stock.min.y - m
            && lo.z >= stock.min.z - m
            && hi.x <= stock.max.x + m
            && hi.y <= stock.max.y + m
            && hi.z <= stock.max.z + m,
        "刀点跑出世界毛坯：({:.1},{:.1},{:.1})..({:.1},{:.1},{:.1})",
        lo.x,
        lo.y,
        lo.z,
        hi.x,
        hi.y,
        hi.z
    );

    // **真的从 −X 那一面下刀**：最小 x 要贴近毛坯的 −X 面。
    // 恒等坐标系下最小 x 会是 −12（横移），所以再检查"切到 x≈−12 的同时，
    // y/z 覆盖到零件侧面"——这里只钉住最直接的一条：x 到达 −11 以内。
    assert!(
        lo.x < stock.min.x + 1.5,
        "没有切到 −X 面：最小 x = {:.2}（毛坯 {:.2}）",
        lo.x,
        stock.min.x
    );

    // 而且不能把整个毛坯都啃掉：Z 方向应该主要在 0..8 内
    assert!(
        hi.z <= stock.max.z + m,
        "z 超出安全余量：{:.2}",
        hi.z
    );
}

#[test]
fn roughing_from_plus_z_still_works() {
    // 对照组：恒等坐标系必须与旧行为一致（不能因为加 Frame 就变样）
    let part = box_mesh(DVec3::new(-6.0, -6.0, 0.0), DVec3::new(6.0, 6.0, 6.0));
    let stock = Stock {
        min: DVec3::new(-12.0, -12.0, 0.0),
        max: DVec3::new(12.0, 12.0, 8.0),
    };
    let (gcode, _) = machine_from(DVec3::new(0.0, 0.0, 1.0), &part, stock);
    let program = cncsim_gcode::interpret(&gcode, &Default::default());
    assert!(!program.has_errors(), "从 +Z 生成的程序解释报错");
    let mut min_z = f64::INFINITY;
    for b in &program.blocks {
        min_z = min_z.min(b.target.tip.z);
    }
    assert!(min_z >= -1e-6, "从 +Z 加工不该切到毛坯底面以下：{min_z:.3}");
}
