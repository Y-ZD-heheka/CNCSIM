//! 切割面溯源（ADR-005 的叶级实现）：**切出来的面必须带着"是哪一刀切的"标记**。
//!
//! 这是"刀刃颜色 = 切面颜色"的地基：仿真在切这一刀之前把调色板索引盖到叶上
//! （`SparseGrid::set_creator`），提取时把叶的标记写进顶点色的 R 通道。
//! 渲染层再把这个索引查成颜色——**两边查的是同一张表**，所以颜色必然一致。
//!
//! 这里的测试全部是"能证伪"的：把 `set_creator` 的调用去掉、或把盖章搬到
//! 所有切削结束之后，`cut_surfaces_keep_their_own_creator` 就会失败。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{
    GridConfig, MeshData, SparseGrid, ValueType, creator_of_tool_segment, tool_segment,
};

const H: f64 = 0.5;
/// 毛坯边长（mm）。20 mm → 40 体素。
const S: f64 = 20.0;

fn n(mm: f64) -> i32 {
    (mm / H).round() as i32
}

fn stock() -> SparseGrid<i8> {
    stock_with_creator_sub(1)
}

fn stock_with_creator_sub(sub: u8) -> SparseGrid<i8> {
    let cfg = GridConfig { h: H, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_creator_sub(sub);
    g.fill_box_exact([0, 0, 0], [n(S), n(S), n(S)], cfg.band_mm());
    g
}

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose { tip: DVec3::new(x, y, z), axis: DVec3::Z }
}

/// 沿 +X 拉一条通槽，槽底在 `z = floor`，中心线 `y = y`。
fn slot(grid: &mut SparseGrid<i8>, y: f64, floor: f64, creator: u8) {
    grid.set_creator(creator);
    // 刃长 15 mm > 毛坯高，槽顶开在毛坯外面（否则是封闭内腔，测起来要多解释一堆面）
    let cutter = flat_end_mill(6.0, 15.0);
    grid.cut_segment(&pose(-5.0, y, floor), &pose(25.0, y, floor), &cutter);
}

/// φ2 窄槽：两条能放进同一片 4 mm 叶，且扫掠体互不相交。
fn narrow_slot(grid: &mut SparseGrid<i8>, y: f64, floor: f64, creator: u8) {
    grid.set_creator(creator);
    let cutter = flat_end_mill(2.0, 15.0);
    grid.cut_segment(&pose(-5.0, y, floor), &pose(25.0, y, floor), &cutter);
}

fn creators(mesh: &MeshData) -> Vec<(f64, u8)> {
    mesh.positions
        .iter()
        .zip(&mesh.colors)
        .map(|(p, c)| (p[2] as f64, c[0]))
        .collect()
}

/// 没切过的毛坯：所有顶点都是索引 0（原始毛坯面）。
#[test]
fn untouched_stock_is_index_zero() {
    let g = stock();
    let m = g.extract_surface();
    assert!(!m.positions.is_empty(), "毛坯应该有表面");
    let bad: Vec<_> = m.colors.iter().filter(|c| c[0] != 0).collect();
    assert!(bad.is_empty(), "未切削的毛坯出现了 {} 个非 0 索引：{:?}", bad.len(), &bad[..bad.len().min(4)]);
}

/// 槽底和槽壁必须带这一刀的颜色；毛坯顶面（没被切到）必须仍是 0。
#[test]
fn cut_surface_takes_the_creator_of_the_cut() {
    let mut g = stock();
    slot(&mut g, 10.0, 10.0, 5);
    let m = g.extract_surface();
    let cs = creators(&m);

    // 槽底：z ≈ 10，**且落在槽的横向范围内**
    //
    // 只按 z 筛是错的：毛坯四个侧壁在 z≈10 处也有顶点，它们本来就没被切过
    // （索引 0 是对的），混进来会把正确的实现判成错的。判据要能区分
    // "槽底"和"恰好同高的侧壁"。
    let floor: Vec<u8> = cs
        .iter()
        .zip(&m.positions)
        .filter(|((z, _), p)| {
            (9.2..10.8).contains(z)
                && (1.0..19.0).contains(&(p[0] as f64))
                && (7.5..12.5).contains(&(p[1] as f64))
        })
        .map(|((_, c), _)| *c)
        .collect();
    assert!(floor.len() > 20, "槽底顶点太少（{} 个），场景没切出来", floor.len());
    assert!(
        floor.iter().all(|c| *c == 5),
        "槽底有顶点不是这一刀的颜色 5：{:?}",
        floor.iter().filter(|c| **c != 5).take(6).collect::<Vec<_>>()
    );

    // 顶面：z ≈ 20，**且离槽足够远**
    //
    // 只看 z 是错的：槽把 z=10 以上的料全切掉了，槽壁一直延伸到 z=20，
    // 槽口边缘那几个顶点归属到被切过的叶是对的。
    //
    // 也不能只看"槽的横向范围"（y ∈ 7..13）：标记是**叶级**的，一片叶 8h 宽，
    // 只要叶里有一个体素变了，整片叶就盖上颜色。所以着色会向未切区域溢出
    // **最多一个叶宽**——这里 h=0.5 → 叶宽 4 mm，实测正好溢到 y=16。
    // h=0.1 的实际场景里这个溢出是 0.8 mm，看不见。
    let leaf_mm = 8.0 * H;
    let edge = 3.0 + leaf_mm + 0.5; // 刀半径 + 一个叶宽 + 半个体素的余量
    let far = |y: f64| (y - 10.0).abs() > edge;
    let top: Vec<u8> = cs
        .iter()
        .zip(&m.positions)
        .filter(|((z, _), p)| *z > 19.0 && far(p[1] as f64))
        .map(|((_, c), _)| *c)
        .collect();
    assert!(top.len() > 20, "顶面顶点太少（{} 个，叶宽 {leaf_mm} mm）", top.len());
    let bad: Vec<u8> = top.iter().copied().filter(|c| *c != 0).collect();
    assert!(bad.is_empty(), "没切到的顶面被盖了颜色：{bad:?}");
}

/// **两条互不重叠的槽各留各的颜色**。
///
/// 这条是整个特性的核心断言：如果盖章被搬到"全部切削结束之后"、或者用了
/// 一个全局变量只在最后写一次，那么先切的那条槽会被后切的覆盖成 7。
#[test]
fn cut_surfaces_keep_their_own_creator() {
    let mut g = stock();
    slot(&mut g, 5.0, 10.0, 3);
    slot(&mut g, 15.0, 10.0, 7);
    let m = g.extract_surface();

    let mut near_y5 = 0usize;
    let mut near_y15 = 0usize;
    for (p, c) in m.positions.iter().zip(&m.colors) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        if !(9.2..10.8).contains(&z) || !(1.0..19.0).contains(&x) {
            continue;
        }
        if (1.5..8.5).contains(&y) {
            assert_eq!(c[0], 3, "第一条槽（y≈5）的颜色被改成了 {}，位置 {:?}", c[0], (x, y, z));
            near_y5 += 1;
        } else if (11.5..18.5).contains(&y) {
            assert_eq!(c[0], 7, "第二条槽（y≈15）的颜色不对：{}，位置 {:?}", c[0], (x, y, z));
            near_y15 += 1;
        }
    }
    assert!(near_y5 > 10 && near_y15 > 10, "两条槽的采样点太少：{near_y5} / {near_y15}");
}

/// 两条不相交的窄槽落在同一片叶里。第二刀不能把第一刀的既有表面
/// 盖成自己的颜色——「这个子块有变化」不等于「这个旧面由本刀造成」。
#[test]
fn adjacent_cuts_in_one_leaf_do_not_recolor_each_other() {
    let mut g = stock_with_creator_sub(8);
    narrow_slot(&mut g, 8.5, 10.0, 3);
    narrow_slot(&mut g, 11.0, 10.0, 7);
    let m = g.extract_surface();

    let (mut first, mut second) = (0usize, 0usize);
    for (p, c) in m.positions.iter().zip(&m.colors) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        if !(9.2..10.8).contains(&z) || !(1.0..19.0).contains(&x) {
            continue;
        }
        if (7.6..9.4).contains(&y) {
            assert_eq!(c[0], 3, "第一刀的旧面被相邻第二刀侵染，位置 {p:?}");
            first += 1;
        } else if (10.1..11.9).contains(&y) {
            assert_eq!(c[0], 7, "第二刀自己的面颜色不对，位置 {p:?}");
            second += 1;
        }
    }
    assert!(first > 10 && second > 10, "两条槽底采样不足：{first} / {second}");
}

/// 刀旁的原始顶面必须保持 0。每体素粒度下仍染色，说明问题不是分辨率，
/// 而是把「旧值贴面」误当成了「当前扫掠在这里造面」。
#[test]
fn original_top_next_to_cut_stays_uncolored_at_voxel_granularity() {
    let mut g = stock_with_creator_sub(8);
    narrow_slot(&mut g, 10.0, 10.0, 5);
    let m = g.extract_surface();

    let mut top = 0usize;
    for (p, c) in m.positions.iter().zip(&m.colors) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        if z > 19.0
            && (1.0..19.0).contains(&x)
            && (8.0..12.0).contains(&y)
            && (y - 10.0).abs() > 1.5
        {
            assert_eq!(c[0], 0, "未切顶面被同叶刀痕染色，位置 {p:?}");
            top += 1;
        }
    }
    assert!(top > 10, "原始顶面采样不足：{top}");
}

/// 后一刀与既有切面完全重合，场值一个都没变——没有去除任何材料，
/// 旧面必须保留原 creator，不能因为刀擦过就改色。
#[test]
fn coincident_pass_keeps_the_surface_it_did_not_change() {
    let mut g = stock_with_creator_sub(8);
    slot(&mut g, 10.0, 10.0, 3);
    slot(&mut g, 10.0, 10.0, 7);
    let m = g.extract_surface();
    let mut floor = 0usize;
    for (p, c) in m.positions.iter().zip(&m.colors) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        if (9.2..10.8).contains(&z)
            && (1.0..19.0).contains(&x)
            && (7.5..12.5).contains(&y)
        {
            assert_eq!(c[0], 3, "未改值的旧面被擦过的刀改色，位置 {p:?}");
            floor += 1;
        }
    }
    assert!(floor > 20, "槽底采样不足：{floor}");
}

/// 逐体素检查：刀**真的搬动了这一格**（场值变了）且新面仍在一格内，才换 creator。
/// 值没变的体素（刀只是擦过、没去料）必须保留原色。
///
/// 注意不能要求"材料 → 空气"：精加工常常只把表面格从 −0.14 抬到 −0.01，
/// 格心仍在材料侧，但那一刀确实把面削出来了（实测 −14 → −1）。
#[test]
fn creator_changes_only_where_the_cut_moved_the_surface() {
    let mut g = stock_with_creator_sub(8);
    narrow_slot(&mut g, 10.0, 10.0, 3);
    let before = g.clone();
    narrow_slot(&mut g, 12.4, 10.0, 7);

    let nside = n(S);
    let mut stamped = 0usize;
    let mut changed = 0usize;
    for z in 0..nside {
        for y in 0..nside {
            for x in 0..nside {
                let ov = before.to_distance(before.get([x, y, z]));
                let nv = g.to_distance(g.get([x, y, z]));
                let c0 = before.creator_of_voxel([x, y, z]);
                let c1 = g.creator_of_voxel([x, y, z]);
                if ov == nv {
                    assert_eq!(
                        c1, c0,
                        "刀只擦过、场值没变，却换了色：({x},{y},{z}) v={ov:+.3} {c0}→{c1}"
                    );
                    continue;
                }
                changed += 1;
                if nv.abs() < H {
                    assert_eq!(
                        c1, 7,
                        "第二刀确实削出了这一格的面，却没换色：({x},{y},{z}) {ov:+.3}→{nv:+.3}"
                    );
                    stamped += 1;
                }
            }
        }
    }
    assert!(changed > 20, "第二刀改动的体素太少：{changed}");
    assert!(stamped > 10, "第二刀贴面改动的体素太少：{stamped}");
}

/// 第二刀从第一刀槽壁外 0.4 mm 擦过：没有去除该槽壁的材料，
/// 即使扫掠面进入一个体素范围内，也不能改色。
#[test]
fn grazing_pass_does_not_recolor_untouched_wall() {
    let mut g = stock_with_creator_sub(8);
    narrow_slot(&mut g, 10.0, 10.0, 3);
    // 半径 1，刀心 y=12.4 → 扫掠面在 y=11.4；第一刀槽壁在 y=11.0，
    // 中间还剩 0.4 mm 材料，场值不变。
    narrow_slot(&mut g, 12.4, 10.0, 7);
    let m = g.extract_surface();

    let (mut first, mut second) = (0usize, 0usize);
    for (p, c) in m.positions.iter().zip(&m.colors) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        if !(9.2..10.8).contains(&z) || !(1.0..19.0).contains(&x) {
            continue;
        }
        if (8.8..11.2).contains(&y) {
            assert_eq!(c[0], 3, "擦边刀改掉了未去除材料的槽壁颜色，位置 {p:?}");
            first += 1;
        } else if (12.4..13.6).contains(&y) {
            assert_eq!(c[0], 7, "第二刀真正切出的底面颜色不对，位置 {p:?}");
            second += 1;
        }
    }
    assert!(first > 10 && second > 10, "采样不足：{first} / {second}");
}

/// 顶点色不能依赖分块数——`fetch_min` 就是为了这个（并行写同一格时取最小，
/// 与写序无关）。把 `fetch_min` 换成 `store` 后这条会**间歇性**失败。
#[test]
fn creators_dont_depend_on_chunking() {
    let mut g = stock();
    slot(&mut g, 5.0, 10.0, 3);
    slot(&mut g, 15.0, 10.0, 7);
    let a = g.extract_surface_chunks(1);
    let b = g.extract_surface_chunks(7);
    assert_eq!(a.positions.len(), b.positions.len());
    assert_eq!(a.colors, b.colors, "分块数不同导致顶点色不同");
}

/// LOD 提取（窗口用的就是它）也必须带颜色，否则"低模看得见形状但全是灰"。
#[test]
fn lod_keeps_creators() {
    let mut g = stock();
    slot(&mut g, 10.0, 10.0, 5);
    let m = g.extract_surface_lod(2);
    assert!(!m.positions.is_empty());
    let hit = m.colors.iter().filter(|c| c[0] == 5).count();
    let zero = m.colors.iter().filter(|c| c[0] == 0).count();
    assert!(hit > 0, "LOD 网格里没有一个顶点带切割色 5");
    assert!(zero > 0, "LOD 网格里没有一个原始毛坯面的顶点");
    assert_eq!(hit + zero, m.colors.len(), "LOD 出现了第三种索引");
}

/// 色号 = **刀具号 × 10 段 + 进度段**，1 起。
///
/// 用户要的是"一眼看出这块面是哪把刀、哪个阶段切的"，所以颜色是**离散分档**
/// 而不是连续渐变：每把刀按 10% 切 10 段，每段一个调色板颜色。
/// 第 1 把刀占 1..=10，第 2 把占 11..=20，第 3 把封顶到 21..=29
/// （30 = MIXED，31 = SHANK）。
#[test]
fn creator_encodes_tool_and_segment() {
    // 第 1 把刀：1..=10，端点精确
    for d in 0..4096u32 {
        let c = creator_of_tool_segment(1, d, 4096);
        assert!((1..=10).contains(&c), "第 1 把刀的色号应在 1..=10，实际 {c}");
    }
    assert_eq!(creator_of_tool_segment(1, 0, 1000), 1, "0% 是第 1 段");
    assert_eq!(creator_of_tool_segment(1, 999, 1000), 10, "100% 是第 10 段");

    // 第 2 把刀：11..=20，与第 1 把**完全不重叠**（这是"按刀具换色"的核心）
    assert_eq!(creator_of_tool_segment(2, 0, 1000), 11);
    assert_eq!(creator_of_tool_segment(2, 999, 1000), 20);

    // 第 3 把：21..=29 封顶；再多刀也停在 29（30/31 是 MIXED/SHANK）
    assert_eq!(creator_of_tool_segment(3, 0, 1000), 21);
    assert_eq!(creator_of_tool_segment(3, 999, 1000), 29);
    assert_eq!(creator_of_tool_segment(9, 999, 1000), 29, "超出 3 把要封顶");
    assert!(creator_of_tool_segment(2, 999, 1000) < cncsim_voxel::CREATOR_MIXED);

    // 刀号 0（程序里没写 T 字）按第 1 把算
    assert_eq!(creator_of_tool_segment(0, 999, 1000), 10);

    // 同一把刀内段号单调不减，且最后一刀是第 10 段
    let mut prev = 0u8;
    for d in 0..=1000u32 {
        let s = tool_segment(d, 1000);
        assert!(s >= prev, "段号必须单调不减：{prev} → {s}");
        prev = s;
    }
    assert_eq!(prev, 9, "最后一刀是第 10 段（0 起 9）");

    // 退化输入
    assert_eq!(tool_segment(0, 0), 0);
    assert_eq!(creator_of_tool_segment(1, 0, 0), 1);
}

// ───────────────────────── 已知边界：颜色会溢出 ─────────────────────────
//
// 溯源标记按**叶**存（8×8×8 体素 = 8h），而 `cut` 还会把新切面周围 ±band 的
// 距离场重新精确化——那一片体素**真的变了**，所以所在的叶也要盖章。两者叠加，
// 染色范围会比真实切面多出 `band + 8h` 以内的一块。
//
// 下面这条测试把这个上界钉住，它同时是一份说明书：谁把粒度改细，这里就会失败，
// 那时该把上界往下调，而不是把测试删掉。
//
// 这里用**最粗**的一档（`creator_sub = 1`，每叶一个标记）来卡上界——它是
// 默认配置里唯一还留得住"大块溢出"的档。四档的完整对照在
// `crates/sim/tests/creator_granularity.rs`：
//
// | sub | 最坏溢出 | creator 内存 | 切削耗时 |
// |---|---|---|---|
// | 1 | 0.50 mm | 1 B/叶 | 基准 |
// | 2 | 0.20 mm | 8 B/叶 | 持平 |
// | 4 | 0.10 mm | 64 B/叶 | 持平（**默认**）|
// | 8 | 0.025 mm | 512 B/叶 | +30% |
//
// 上界 `band + 8h` 只对 sub = 1 成立；粒度越细，实际溢出越小。

/// 切一刀槽，量"切色区域最低点"比切深面低多少。
///
/// 用这个判据而不是"侧壁被染多高"：侧壁会不会被染取决于侧壁所在的那片叶
/// 有没有体素变化，h=0.5 时侧壁那片叶整个是空气、一个体素都没变，就完全
/// 染不到——按侧壁量会得到 inf。切色最低点与分辨率、几何都无关。
fn bleed_below(cut_z: f64) -> (f64, f64, f64) {
    use cncsim_motion::Pose;
    let mut g = stock();
    g.set_creator(1);
    let cutter = flat_end_mill(6.0, 15.0);
    g.cut_segment(
        &Pose { tip: DVec3::new(-5.0, 2.0, cut_z), axis: DVec3::Z },
        &Pose { tip: DVec3::new(25.0, 2.0, cut_z), axis: DVec3::Z },
        &cutter,
    );
    let m = g.extract_surface();
    let mut zmin = f64::INFINITY;
    for (p, c) in m.positions.iter().zip(&m.colors) {
        if c[0] != 0 && (p[2] as f64) < cut_z - 0.05 {
            zmin = zmin.min(p[2] as f64);
        }
    }
    let bleed = if zmin.is_finite() { cut_z - zmin } else { 0.0 };
    (bleed, g.config.band_mm(), 8.0 * g.config.h)
}

/// 溢出存在，但不超过 `band + 叶宽`。
#[test]
fn color_bleed_stays_within_band_plus_one_leaf() {
    let mut worst = 0.0f64;
    for cut_z in [11.0, 10.9, 10.7, 10.5, 10.2, 9.9] {
        let (bleed, band, leaf) = bleed_below(cut_z);
        assert!(
            bleed <= band + leaf + 1e-9,
            "切深 {cut_z}：溢出 {bleed:.2} mm 超过上界 {:.2}（band {band:.2} + 叶宽 {leaf:.2}）",
            band + leaf
        );
        worst = worst.max(bleed);
    }
    // 全部为 0 说明标记根本没盖到叶上——那是另一种坏法，单独卡一次
    assert!(worst > 0.0, "所有切深下都没有溢出：标记没生效或测试几何失效");
}
