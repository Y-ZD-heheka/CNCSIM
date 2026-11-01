//! 稀疏树的存储层测试。
//!
//! 这些测试盯的是**存储层的正确性**，不是性能。其中最要紧的一条是
//! `prune_restores_constant_after_full_cut`——它对应 ADR-004 §5.4 那条
//! "切空的砖仍是 Full 会让碰撞持续误报"。

use cncsim_math::DVec3;
use cncsim_voxel::tree::Scalar;
use cncsim_voxel::{
    GridConfig, L1_SPAN, L2_SPAN, Leaf, ROOT_SPAN, SparseGrid, child_index, l1_coord, l1_slot,
    l2_slot, leaf_coord, leaf_index, root_coord,
};
use std::collections::HashMap;

fn grid() -> SparseGrid<i8> {
    SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
}

// ───────────────────────── 索引位运算 ─────────────────────────

#[test]
fn index_math_is_consistent_at_level_boundaries() {
    // 叶内：x 最快
    assert_eq!(leaf_index([0, 0, 0]), 0);
    assert_eq!(leaf_index([7, 0, 0]), 7);
    assert_eq!(leaf_index([0, 1, 0]), 8);
    assert_eq!(leaf_index([0, 0, 1]), 64);
    // 叶边界：8 与 7 必须属于不同叶
    assert_ne!(leaf_coord([7, 0, 0]), leaf_coord([8, 0, 0]));
    assert_eq!(leaf_coord([8, 0, 0]), [1, 0, 0]);
    // level-1：16 个叶/轴 → 128 体素
    assert_eq!(l1_coord([0, 0, 0]), [0, 0, 0]);
    assert_eq!(l1_coord([L1_SPAN - 1, 0, 0]), [0, 0, 0]);
    assert_eq!(l1_coord([L1_SPAN, 0, 0]), [1, 0, 0]);
    // root：32 个 level-1/轴 → 4096 体素
    assert_eq!(root_coord([0, 0, 0]), [0, 0, 0]);
    assert_eq!(root_coord([ROOT_SPAN - 1, 0, 0]), [0, 0, 0]);
    assert_eq!(root_coord([ROOT_SPAN, 0, 0]), [1, 0, 0]);
    // 一致性：ROOT_SPAN == L2_SPAN，且 L2_SPAN == 32 × L1_SPAN
    assert_eq!(ROOT_SPAN, L2_SPAN);
    assert_eq!(L2_SPAN, 32 * L1_SPAN);
}

#[test]
fn negative_coordinates_use_floor_semantics() {
    // 负数必须向下取整：体素 -1 属于叶 -1 的局部索引 7，不是叶 0
    assert_eq!(leaf_coord([-1, 0, 0]), [-1, 0, 0]);
    assert_eq!(leaf_index([-1, 0, 0]), 7);
    assert_eq!(leaf_coord([-8, 0, 0]), [-1, 0, 0]);
    assert_eq!(leaf_coord([-9, 0, 0]), [-2, 0, 0]);
    // 若用 / 和 % 实现，-1 会落到叶 0、索引 -1 → 直接崩或串味
    assert_eq!(l1_slot(leaf_coord([-1, 0, 0])), l1_slot([15, 0, 0]));
    assert_eq!(l2_slot(l1_coord([-1, 0, 0])), l2_slot([31, 0, 0]));
}

#[test]
fn child_index_is_popcount_of_low_bits() {    // 空掩码：任何槽都映射到 0
    let empty = [0u64; 4];
    assert_eq!(child_index(&empty, 0), 0);
    assert_eq!(child_index(&empty, 200), 0);

    // 只有槽 0 和槽 2 活动
    let mut m = [0u64; 4];
    m[0] = 0b101;
    assert_eq!(child_index(&m, 0), 0, "槽 0 之前没有活动子节点");
    assert_eq!(child_index(&m, 2), 1, "槽 2 之前有 1 个（槽 0）");
    assert_eq!(child_index(&m, 5), 2, "槽 5 之前有 2 个");

    // 跨字：槽 64 属于第 2 个字
    let mut m2 = [0u64; 4];
    m2[0] = 0b11; // 2 个
    m2[1] = 0b1; // 第 64 槽活动
    assert_eq!(child_index(&m2, 64), 2, "跨字累加必须正确");
    assert_eq!(child_index(&m2, 65), 3);
}

// ───────────────────────── 树结构 ─────────────────────────

/// 200×200×100 mm 毛坯 @ h=0.1 → 2000×2000×1000 体素。
/// 一个 level-2 节点覆盖 4096 体素/轴，所以根只应有 **1** 条。
#[test]
fn root_has_single_entry_for_small_stock() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    assert_eq!(
        g.root_len(),
        1,
        "200mm 毛坯应只有 1 条 root 条目（扁平 HashMap 要 12.5 万条）"
    );
    assert_eq!(g.get([1000, 1000, 500]), -127);
    assert_eq!(g.get([0, 0, 0]), -127);
    assert_eq!(g.get([1999, 1999, 999]), -127);
    assert_eq!(g.get([2000, 0, 0]), g.background, "盒外应是背景值");
}

/// 实心内部**连叶都不该分配**——整块同值的区域由 tile 吃掉。
///
/// 这是树结构存在的主要理由：2000×2000×1000 体素若逐叶存，光值就要 4 GB。
#[test]
fn uniform_interior_is_covered_by_tiles_not_leaves() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);

    assert_eq!(
        g.leaf_at([1000, 1000, 500]),
        None,
        "深内部应由 tile 覆盖，不该有叶"
    );
    assert_eq!(g.get([1000, 1000, 500]), -127, "tile 值仍要能读出来");

    // 稠密存储需要 2000×2000×1000 = 4 GB；实际应在 20 MB 以内
    assert!(
        g.value_bytes() < 20 * 1024 * 1024,
        "内部不该逐叶分配，实际占用 {} 字节",
        g.value_bytes()
    );
}

/// 紧贴填充边界的那层叶槽必须是**常量叶**（16 字节），不能是密集叶（528 字节）。
///
/// 它们是必要的——表面提取要覆盖"最后一个实体体素 ↔ 盒外背景"这条符号变化，
/// 否则网格会在实体边界上漏洞。但 200mm 毛坯的边界层有 25 万个叶槽，
/// 建成密集叶要多 130 MB，建成常量叶只要 4 MB。
#[test]
fn boundary_layer_leaves_are_constant_not_dense() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);

    match g.leaf_at([0, 0, 0]) {
        Some(Leaf::Constant(v)) => assert_eq!(*v, -127),
        other => panic!("边界叶应是常量叶，实际 {other:?}"),
    }
    // 深内部仍然是 tile（连叶都没有）
    assert_eq!(g.leaf_at([1000, 1000, 500]), None);
    assert!(
        g.value_bytes() < 20 * 1024 * 1024,
        "边界层不该建成密集叶，实际 {} 字节",
        g.value_bytes()
    );
}

#[test]
fn outside_is_background() {
    let g = grid();
    assert_eq!(g.get([-1, -1, -1]), g.background);
    assert_eq!(g.get([100_000, 0, 0]), g.background);
    assert_eq!(g.leaf_at([100_000, 0, 0]), None);
}

#[test]
fn set_expands_constant_leaf_to_dense() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    let p = [1000, 1000, 500];
    g.set(p, 42);

    match g.leaf_at(p) {
        Some(Leaf::Dense(a)) => assert_eq!(a[leaf_index(p)], 42),
        other => panic!("写入后应展开为密集叶，实际 {other:?}"),
    }
    assert_eq!(g.get(p), 42);
    // 同叶的其他体素保持原值——展开时不能丢数据
    assert_eq!(g.get([1001, 1000, 500]), -127);
    assert_eq!(g.get([1000, 1001, 500]), -127);
    assert_eq!(g.get([1000, 1000, 501]), -127);
}

#[test]
fn set_into_empty_region_creates_nodes() {
    let mut g = grid();
    // 从完全空的网格写入（所有节点都要现建）
    g.set([5000, 5000, 5000], -7);
    assert_eq!(g.get([5000, 5000, 5000]), -7);
    assert_eq!(g.get([5001, 5000, 5000]), g.background);
    // 新节点覆盖的其余区域应保持背景值，不能变成 0
    assert_eq!(g.get([5000, 5000, 5001]), g.background);
    assert_eq!(g.get([0, 0, 0]), g.background);
}

#[test]
fn neighbor_leaves_do_not_interfere() {
    let mut g = grid();
    g.set([0, 0, 0], 11);
    g.set([8, 0, 0], 22);
    g.set([0, 8, 0], 33);
    assert_eq!(g.get([0, 0, 0]), 11);
    assert_eq!(g.get([8, 0, 0]), 22);
    assert_eq!(g.get([0, 8, 0]), 33);
    // 三者互不污染
    assert_eq!(g.get([1, 0, 0]), g.background);
    assert_eq!(g.get([9, 0, 0]), g.background);
}

/// **存储层的正确性要求**（ADR-004 §5.4）：切空的叶必须折叠回常量，
/// 否则碰撞检测会认为那里还有材料，沿壁拖动时持续误报。
#[test]
fn prune_restores_constant_after_full_cut() {
    let mut g = grid();
    // 恰好一个叶：8³
    g.fill_box([0, 0, 0], [8, 8, 8], -127);
    // 逐体素切成 +127（模拟完全切空）
    for i in 0..8 {
        for j in 0..8 {
            for k in 0..8 {
                g.set([i, j, k], 127);
            }
        }
    }
    assert!(
        matches!(g.leaf_at([0, 0, 0]), Some(Leaf::Dense(_))),
        "写满之后仍是密集叶"
    );
    g.prune();
    match g.leaf_at([0, 0, 0]) {
        Some(Leaf::Constant(v)) => assert_eq!(*v, 127, "切空的叶必须折叠成常量 +127"),
        other => panic!("prune 后应折叠为常量叶，实际 {other:?}"),
    }
    assert_eq!(g.get([3, 3, 3]), 127);
}

#[test]
fn prune_keeps_non_uniform_leaves_dense() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [8, 8, 8], -127);
    g.set([1, 2, 3], 50);
    g.prune();
    match g.leaf_at([0, 0, 0]) {
        Some(Leaf::Dense(a)) => assert_eq!(a[leaf_index([1, 2, 3])], 50),
        other => panic!("非均匀叶不能被折叠，实际 {other:?}"),
    }
    assert_eq!(g.get([0, 0, 0]), -127);
}

// ───────────────────────── 与影子模型对拍 ─────────────────────────

/// 树查询必须与一个独立实现的 `HashMap` 影子模型逐点一致。
///
/// 这是"不要只测自己"的那条原则：影子模型不共享任何索引代码。
#[test]
fn traversal_matches_shadow_model() {
    let mut g = grid();
    let mut model: HashMap<[i32; 3], i8> = HashMap::new();

    // 确定性伪随机（不用外部 crate，保证可复现）
    let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };

    for _ in 0..3000 {
        let r = next();
        // 跨多个 level-1 / level-2 边界，覆盖负坐标
        let p = [
            (r % 600) as i32 - 300,
            ((r >> 20) % 600) as i32 - 300,
            ((r >> 40) % 600) as i32 - 300,
        ];
        let v = ((r >> 12) % 255) as i32 - 127;
        g.set(p, v as i8);
        model.insert(p, v as i8);
    }

    // 已写点必须一致
    for (k, v) in &model {
        assert_eq!(g.get(*k), *v, "坐标 {k:?} 与影子模型不一致");
    }

    // 未写点必须是背景值
    let mut checked = 0;
    for _ in 0..2000 {
        let r = next();
        let p = [
            (r % 600) as i32 - 300,
            ((r >> 20) % 600) as i32 - 300,
            ((r >> 40) % 600) as i32 - 300,
        ];
        if !model.contains_key(&p) {
            assert_eq!(g.get(p), g.background, "未写点 {p:?} 应是背景值");
            checked += 1;
        }
    }
    assert!(checked > 100, "随机点应大部分未写过，实际只检查了 {checked} 个");
}

#[test]
fn fill_box_matches_shadow_model() {
    let mut g = grid();
    let mut model: HashMap<[i32; 3], i8> = HashMap::new();
    let (lo, hi, v) = ([-20, -20, -20], [45, 33, 12], -60);

    g.fill_box(lo, hi, v);
    for i in lo[0]..hi[0] {
        for j in lo[1]..hi[1] {
            for k in lo[2]..hi[2] {
                model.insert([i, j, k], v);
            }
        }
    }

    for (k, val) in &model {
        assert_eq!(g.get(*k), *val, "盒内 {k:?} 应是 {val}");
    }
    // 盒外六个面各取一点
    for p in [
        [lo[0] - 1, lo[1], lo[2]],
        [hi[0], lo[1], lo[2]],
        [lo[0], lo[1] - 1, lo[2]],
        [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], lo[2] - 1],
        [lo[0], lo[1], hi[2]],
    ] {
        assert_eq!(g.get(p), g.background, "盒外 {p:?} 应是背景值");
    }
}

// ───────────────────────── 量化与插值 ─────────────────────────

#[test]
fn quantization_roundtrip() {
    let g = grid();
    let band = g.config.band_mm();
    assert!((band - 0.4).abs() < 1e-12);
    // 端点
    assert!((g.to_distance(127) - 0.4).abs() < 1e-12);
    assert!((g.to_distance(-127) + 0.4).abs() < 1e-12);
    assert!((g.to_distance(0)).abs() < 1e-12);
    // 往返
    for v in [-127i8, -64, -1, 0, 1, 63, 127] {
        assert_eq!(g.to_value(g.to_distance(v)), v, "值 {v} 往返失败");
    }
    // 每档 3.15 µm
    let step = g.to_distance(1) - g.to_distance(0);
    assert!((step * 1000.0 - 3.15).abs() < 0.01, "每档应约 3.15 µm，实际 {:.3}", step * 1000.0);
}

#[test]
fn sdf_interpolates_between_voxel_centers() {
    let mut g = grid();
    // 体素 (0,0,0) 中心 = 0.05，(1,0,0) 中心 = 0.15
    g.set([0, 0, 0], 0);
    g.set([1, 0, 0], 127);
    let a = g.voxel_center([0, 0, 0]);
    let b = g.voxel_center([1, 0, 0]);
    assert!((a.x - 0.05).abs() < 1e-12, "体素 0 中心应在 0.05，实际 {}", a.x);
    assert!((b.x - 0.15).abs() < 1e-12);

    // 中点：值 63.5 → 距离 0.2 mm
    let mid = DVec3::new(0.10, 0.05, 0.05);
    let d = g.sdf(mid);
    assert!((d - 0.2).abs() < 1e-6, "中点应插值出 0.2 mm，实际 {d}");

    // 端点必须精确等于体素值
    assert!((g.sdf(a) - 0.0).abs() < 1e-9);
    assert!((g.sdf(b) - 0.4).abs() < 1e-9);
}

#[test]
fn sdf_clamps_outside_band() {
    let g = grid();
    // 空网格：处处是背景值 +127 → +band
    assert!((g.sdf(DVec3::new(500.0, 500.0, 500.0)) - 0.4).abs() < 1e-12);
}

#[test]
fn sdf_sign_flips_across_surface() {
    let mut g = grid();
    // 沿 x 造一个"表面"：x<8 内部（负），x>=8 外部（正）
    g.fill_box([0, 0, 0], [8, 8, 8], -127);
    let inside = g.voxel_center([4, 4, 4]);
    let outside = g.voxel_center([12, 4, 4]);
    assert!(g.sdf(inside) < 0.0, "材料内应为负，实际 {}", g.sdf(inside));
    assert!(g.sdf(outside) > 0.0, "材料外应为正，实际 {}", g.sdf(outside));
}

// ───────────────────────── 盲元数据 ─────────────────────────

#[test]
fn blind_metadata_tracks_nearest_surface() {
    let mut g = grid();
    // 一个远离表面的点：|v| = 100 → 该 level-1 块的下界是 100
    g.set([10, 10, 10], 100);
    let l1 = g.level1_at([10, 10, 10]).expect("应有 level-1 节点");
    assert_eq!(l1.blind_min_abs(), Some(100), "全是远点，下界应是 100");

    // 同块内写入一个近表面点（|v| = 3）→ 下界必须跟着变小
    g.set([11, 11, 11], 3);
    let l1 = g.level1_at([10, 10, 10]).unwrap();
    assert_eq!(l1.blind_min_abs(), Some(3), "写入近表面点后下界应变 3");

    // prune 重算后仍应一致（不能把下界算丢）
    g.prune();
    let l1 = g.level1_at([10, 10, 10]).unwrap();
    assert_eq!(l1.blind_min_abs(), Some(3), "prune 后重算结果应一致");
}

#[test]
fn value_type_is_a_parameter_not_a_rewrite() {
    // 同一套代码换 S=i16 就得到 258 倍精度——存储层不该为此重写
    let g16 = SparseGrid::<i16>::new(GridConfig::default(), DVec3::ZERO);
    assert!((g16.to_distance(32767) - 0.4).abs() < 1e-12);
    let step = g16.to_distance(1);
    assert!((step * 1e6 - 12.2).abs() < 0.1, "i16 每档应约 12.2 nm，实际 {:.3}", step * 1e6);
    // 与 i8 的比值
    let g8 = grid();
    let ratio = g8.to_distance(1) / step;
    assert!((ratio - 258.0).abs() < 2.0, "比值应约 258，实际 {ratio:.1}");
}

#[test]
fn scalar_sentinels_are_symmetric() {
    // 注意用 `<i8 as Scalar>::MIN`——`i8::MIN` 会解析到标准库的固有常量 −128
    assert_eq!(<i8 as Scalar>::MIN, -127);
    assert_eq!(<i8 as Scalar>::MAX, 127);
    assert_eq!(<i16 as Scalar>::MIN, -32767);
    assert_eq!(<i16 as Scalar>::MAX, 32767);
    // 对称量程才让 |v| 直接当"离表面多远"
    assert_eq!(<i8 as Scalar>::MIN.unsigned_abs(), <i8 as Scalar>::MAX as u8);
}

// ─────────────── 整叶填值：tile 走原生表示，不造叶 ───────────────

/// `fill_leaf` 落到一个**还是 tile 的槽**上时，必须改 tile 本身。
///
/// 这是铣光里最大的一笔开销：整叶短路（叶被完全埋进刀体）在粗加工里每刀几百次，
/// 而落在 tile 上的槽**本来就是"整叶同值"的原生表示**。旧实现先 `ensure_leaf`
/// 造一个 `Dense`（520 字节分配 + 512 值写），再由 `prune` 发现全同折叠回
/// `Constant` —— 分配了又扔掉。
#[test]
fn fill_leaf_on_tile_keeps_it_a_tile() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [64, 64, 64], 40); // 整片 tile
    let leaf = [1, 1, 1];
    assert!(
        g.leaf_at([8, 8, 8]).is_none(),
        "前置条件：这一片应该是 tile，不该有叶"
    );

    let mut wc = cncsim_voxel::tree::WriteCache::default();
    let n = g.fill_leaf(leaf, -20, &mut wc);
    assert_eq!(n, 512, "整叶从 40 变成 −20，512 个体素全变");

    assert!(
        g.leaf_at([8, 8, 8]).is_none(),
        "改 tile 不该顺手造出一个叶（这正是要省的分配）"
    );
    assert_eq!(g.get([8, 8, 8]), -20, "叶内体素必须读到新值");
    assert_eq!(g.get([8, 8, 15]), -20, "整叶 512 个都变");
    assert_eq!(
        g.get([16, 8, 8]),
        40,
        "相邻叶（另一个槽）不受影响"
    );
}

/// tile 路径的幂等性：重复填同一个值返回 0，且不重复标脏。
#[test]
fn fill_leaf_on_tile_is_idempotent() {
    let mut g = grid();
    g.fill_box([0, 0, 0], [64, 64, 64], 40);
    let mut wc = cncsim_voxel::tree::WriteCache::default();
    assert_eq!(g.fill_leaf([1, 1, 1], -20, &mut wc), 512);
    assert_eq!(
        g.fill_leaf([1, 1, 1], -20, &mut wc),
        0,
        "已经是这个值：没有体素改变"
    );
}

/// 叶**已存在**时行为不能变：整叶换值返回真实改动数，并折叠成 `Constant`。
#[test]
fn fill_leaf_on_existing_dense_reports_real_change_count() {
    let mut g = grid();
    // 造一个 Dense 叶：512 个值里一半是 10、一半是 20
    for i in 0..8 {
        for j in 0..8 {
            for k in 0..8 {
                let v = if i < 4 { 10 } else { 20 };
                g.set([i, j, k], v);
            }
        }
    }
    assert!(matches!(g.leaf_at([0, 0, 0]), Some(Leaf::Dense(_))));

    let mut wc = cncsim_voxel::tree::WriteCache::default();
    let n = g.fill_leaf([0, 0, 0], 10, &mut wc);
    assert_eq!(n, 256, "只有原本是 20 的那一半变了");
    assert!(matches!(g.leaf_at([0, 0, 0]), Some(Leaf::Constant(10))));
    assert_eq!(g.get([7, 7, 7]), 10);
}

// ─────────────── fill_box_exact 的逐体素语义 ───────────────

/// `fill_box_exact` 的完整语义：**边界壳写精确盒 SDF，内部是饱和的 −band**。
///
/// 之前只有精度测试（`precision.rs` 看某个过零点的位置）和 OpenVDB 对拍
/// （`ovdb_parity.rs` 看三个过零点），**没有一个测试逐体素比对整场**。
/// 要重构写入路径（从逐点 `set` 改成按叶聚合 `blit`），先得有这条安全网——
/// 否则"某一片叶没被写进去"这种错只会表现为精度测试偶然通过。
#[test]
fn fill_box_exact_matches_pointwise_reference() {
    use cncsim_voxel::tree::Scalar;

    let cfg = GridConfig {
        h: 0.2,
        band_voxels: 4,
        value_type: cncsim_voxel::ValueType::I8,
    };
    let band = cfg.band_mm();
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);

    // 盒子故意不按叶对齐（lo/hi 都不是 8 的倍数），逼出边界叶的跨界情形
    let (lo, hi) = ([3, 5, 7], [29, 27, 21]);
    g.fill_box_exact(lo, hi, band);

    // 参考实现：直接按解析盒 SDF 逐点算
    let k = <i8 as Scalar>::MAX_MAG / band;
    let h = cfg.h;
    let mut checked = 0;
    // 覆盖盒子 + pad 一圈，也检查盒外的背景
    for z in lo[2] - 8..hi[2] + 8 {
        for y in lo[1] - 8..hi[1] + 8 {
            for x in lo[0] - 8..hi[0] + 8 {
                let p = DVec3::new(
                    (x as f64 + 0.5) * h,
                    (y as f64 + 0.5) * h,
                    (z as f64 + 0.5) * h,
                );
                let (x0, x1) = (lo[0] as f64 * h, hi[0] as f64 * h);
                let (y0, y1) = (lo[1] as f64 * h, hi[1] as f64 * h);
                let (z0, z1) = (lo[2] as f64 * h, hi[2] as f64 * h);
                let dx = (x0 - p.x).max(p.x - x1);
                let dy = (y0 - p.y).max(p.y - y1);
                let dz = (z0 - p.z).max(p.z - z1);
                let d = dx.max(dy).max(dz);

                let expect = if d.abs() < band {
                    <i8 as Scalar>::from_f64_round(d * k)
                } else if d < 0.0 {
                    // 带内的盒内：饱和材料 −MAX
                    <i8 as Scalar>::from_f64_round(-band * k)
                } else {
                    // 带外的盒外：**背景值就是饱和空气 +MAX**（`SparseGrid::new`
                    // 里 `background: S::MAX`）——SDF 语义里"没有数据"= 没有材料。
                    // 第一版这里写成 0，测试立刻报「盒外应为 0，实际 127」。
                    <i8 as Scalar>::MAX
                };
                let got = g.get([x, y, z]);
                assert_eq!(
                    got, expect,
                    "体素 {x},{y},{z}：d={d:.4}（band={band}）应为 {expect}，实际 {got}"
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 30_000, "覆盖的体素太少，测试没意义");
}

// ─────────────── blit_leaf：整块写不该绕道 Constant ───────────────

/// `blit_leaf` 到**不存在的叶**：直接建成 `Dense`，值逐个对上。
#[test]
fn blit_leaf_creates_dense_with_exact_values() {
    use cncsim_voxel::tree::WriteCache;
    let mut g = grid();
    g.fill_box([0, 0, 0], [32, 32, 32], 40); // 一片 tile

    let vals: [i8; 512] = std::array::from_fn(|i| (i % 100) as i8 - 50);
    let mut wc = WriteCache::default();
    g.blit_leaf([1, 1, 1], &vals, &mut wc);

    assert!(
        matches!(g.leaf_at([8, 8, 8]), Some(Leaf::Dense(_))),
        "整块写必须建 Dense 叶，不能留 Constant"
    );
    for z in 0..8 {
        for y in 0..8 {
            for x in 0..8 {
                let idx = (x | (y << 3) | (z << 6)) as usize;
                assert_eq!(
                    g.get([8 + x, 8 + y, 8 + z]),
                    vals[idx],
                    "体素 {x},{y},{z} 值不对"
                );
            }
        }
    }
}

/// `blit_leaf` 到**已存在的 Constant 叶**：展开成 `Dense` 并覆盖。
#[test]
fn blit_leaf_expands_constant_to_dense() {
    use cncsim_voxel::tree::WriteCache;
    let mut g = grid();
    // 用 set 造一个 Constant 叶（同一个值写满 512 个）
    for z in 0..8 {
        for y in 0..8 {
            for x in 0..8 {
                g.set([x, y, z], 7);
            }
        }
    }
    // prune 会把它折成 Constant
    g.prune();
    assert!(matches!(g.leaf_at([0, 0, 0]), Some(Leaf::Constant(7))));

    let vals = [9i8; 512];
    let mut wc = WriteCache::default();
    g.blit_leaf([0, 0, 0], &vals, &mut wc);

    assert!(matches!(g.leaf_at([0, 0, 0]), Some(Leaf::Dense(_))));
    assert_eq!(g.get([0, 0, 0]), 9);
    assert_eq!(g.get([7, 7, 7]), 9);
}

/// `blit_leaf` 到**已存在的 Dense 叶**：原地覆盖（cache 命中路径）。
#[test]
fn blit_leaf_overwrites_existing_dense() {
    use cncsim_voxel::tree::WriteCache;
    let mut g = grid();
    let a = [1i8; 512];
    let mut wc = WriteCache::default();
    g.blit_leaf([2, 2, 2], &a, &mut wc);
    assert_eq!(g.get([16, 16, 16]), 1);

    let b = [5i8; 512];
    g.blit_leaf([2, 2, 2], &b, &mut wc);
    assert_eq!(g.get([16, 16, 16]), 5, "第二次写必须覆盖");
    assert_eq!(g.get([23, 23, 23]), 5);
}
