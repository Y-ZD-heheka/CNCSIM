//! 并行结果的一致性测试。
//!
//! 项目硬门槛是"仿真结果可复现"，并行只能改**怎么算**，不能改**算出什么**。
//! 这几条测试把"块数 / 片数"当成旋钮：旋钮拧到哪一档，产出都必须逐位相同。
//!
//! > 线程数是进程级全局变量（`OnceLock`），测试里改不了；所以对拍方式改成
//! > **强制分块数**——分块数 1 就是串行路径，分块数 97 就是 97 块并行。
//! > 动态领号本身是不确定的（谁先拿到哪块），两条路径都能逐位相同，
//! > 才说明"合并顺序与调度无关"这件事真的成立。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, MeshData, SparseGrid};

fn grid() -> SparseGrid<i8> {
    SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
}

/// 12×12×8 mm 毛坯 + 一条 6 mm 宽、1 mm 深的槽（真实切削面，不是解析体）
fn slotted() -> SparseGrid<i8> {
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

fn assert_same_mesh(a: &MeshData, b: &MeshData, what: &str) {
    assert_eq!(a.positions, b.positions, "{what}：顶点位置不一致");
    assert_eq!(a.normals, b.normals, "{what}：法向不一致");
    assert_eq!(a.colors, b.colors, "{what}：顶点色不一致");
    assert_eq!(a.indices, b.indices, "{what}：索引不一致");
    assert_eq!(a.skipped_quads, b.skipped_quads, "{what}：丢洞数不一致");
}

/// 网格提取：分块数不影响任何一个字节。
#[test]
fn extract_chunk_count_does_not_change_the_mesh() {
    let g = slotted();
    let serial = g.extract_surface_chunks(1);
    assert!(!serial.positions.is_empty(), "样本必须有面，否则测了个空");
    for chunks in [2, 5, 17, 97] {
        let par = g.extract_surface_chunks(chunks);
        assert_same_mesh(&serial, &par, &format!("{chunks} 块"));
    }
    // 自动分块（按线程数）也必须一致
    assert_same_mesh(&serial, &g.extract_surface(), "自动分块");
}

/// 显示 LOD：同样与分块数无关。
#[test]
fn lod_chunk_count_does_not_change_the_mesh() {
    let g = slotted();
    for k in [1u32, 2, 4] {
        let serial = g.extract_surface_lod_chunks(k, 1);
        assert!(!serial.positions.is_empty());
        for chunks in [3, 11, 64] {
            let par = g.extract_surface_lod_chunks(k, chunks);
            assert_same_mesh(&serial, &par, &format!("k={k} {chunks} 块"));
        }
    }
}

/// 切削：片数不影响**体素值与改动数**。
///
/// 注意 `visited` **不是**不变量：整叶短路（切深 > 一个叶高时触发）会把
/// 一个完整叶记成 512，而切片边界把叶切开后就退化成逐体素——
/// 两种路径的计数口径不同，但结果必须逐位一致。
#[test]
fn cut_slice_count_does_not_change_the_grid() {
    let prof = flat_end_mill(6.0, 20.0);
    // 刀尖 z=6，毛坯顶面 z=8 → 切 2 mm 深，改动的体素足够多
    let from = Pose { tip: DVec3::new(0.0, 6.0, 6.0), axis: DVec3::Z };
    let to = Pose { tip: DVec3::new(12.0, 6.0, 6.0), axis: DVec3::Z };

    let mut a = grid();
    a.fill_box([0, 0, 0], [120, 120, 80], -127);
    let sa = a.cut_segment_with(&from, &to, &prof, 1);

    let mut b = grid();
    b.fill_box([0, 0, 0], [120, 120, 80], -127);
    let sb = b.cut_segment_with(&from, &to, &prof, 13);

    // visited 口径随短路命中数变化，不能比；changed 与体素值是硬不变量
    assert_eq!(sa.changed, sb.changed, "改动数不一致");
    assert_eq!(sa.box_size, sb.box_size, "AABB 不一致");
    assert!(sa.changed > 0, "样本必须真削掉材料");
    assert_eq!(a.leaf_count(), b.leaf_count(), "叶数不一致（prune 结果不同）");

    // 逐体素比：整个毛坯范围
    let (lo, hi) = a.bounds().unwrap();
    for z in lo[2]..hi[2] {
        for y in lo[1]..hi[1] {
            for x in lo[0]..hi[0] {
                let ijk = [x, y, z];
                assert_eq!(a.get(ijk), b.get(ijk), "体素 {ijk:?} 不一致");
            }
        }
    }
}

/// 同一份数据连提两次（都走自动分块）：调度不同，结果必须相同。
#[test]
fn repeated_parallel_runs_are_bit_identical() {
    let g = slotted();
    let a = g.extract_surface();
    let b = g.extract_surface();
    assert_same_mesh(&a, &b, "重复提取");
    let a = g.extract_surface_lod(4);
    let b = g.extract_surface_lod(4);
    assert_same_mesh(&a, &b, "重复 LOD");
}

/// 真实形状的微基准（默认跳过）：连续 200 段 0.05 mm 的槽，模拟仿真主循环。
///
/// ```bash
/// cargo test --release -p cncsim-voxel --test parallel -- --ignored --nocapture perf
/// CNCSIM_THREADS=1 cargo test --release -p cncsim-voxel --test parallel -- --ignored --nocapture perf
/// ```
#[test]
#[ignore = "性能基准，默认不跑"]
fn perf_many_small_segments() {
    let prof = flat_end_mill(6.0, 20.0);
    let mut g = grid();
    g.fill_box([0, 0, 0], [120, 120, 80], -127);

    let n = 200;
    let t0 = std::time::Instant::now();
    for i in 0..n {
        let x0 = i as f64 * 0.05;
        let x1 = x0 + 0.05;
        g.cut_segment(
            &Pose { tip: DVec3::new(x0, 6.0, 7.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(x1, 6.0, 7.0), axis: DVec3::Z },
            &prof,
        );
    }
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    println!(
        "{n} 段（每段 0.05 mm，{} 线程）：合计 {ms:.0} ms，每段 {:.2} ms，外推 10 万段 {:.1} 分钟",
        cncsim_voxel::par::worker_count(),
        ms / n as f64,
        ms / n as f64 * 1e5 / 6e4
    );
}
