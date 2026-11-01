//! 扫掠 SDF 与 CSG 去除的集成测试。
//!
//! 核心断言是**切完之后表面的位置**——不是"有没有变化"。
//! 切错方向（`min` 写成 `max`、符号反了）画面看起来也在动，只有量表面位置才抓得到。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::{ball_end_mill, flat_end_mill};
use cncsim_voxel::{
    GridConfig, Leaf, SparseGrid, lerp_pose, sd_sweep, sd_sweep_sampled, sd_tool,
};

fn grid() -> SparseGrid<i8> {
    SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
}

/// 10×10×5 mm 的实心毛坯，voxel 0..100 × 0..100 × 0..50
fn stock() -> SparseGrid<i8> {
    let mut g = grid();
    g.fill_box([0, 0, 0], [100, 100, 50], -127);
    g
}

fn up(tip: DVec3) -> Pose {
    Pose { tip, axis: DVec3::Z }
}

// ───────────────────── 扫掠 SDF 与暴力 oracle 对拍 ─────────────────────

/// 黄金分割的结果必须与"不假设凸性"的暴力采样一致，**且在窄带内**。
///
/// 采样版从上方逼近真值（min 只会被高估），所以 golden ≤ sampled 且差值是采样误差。
#[test]
fn golden_search_matches_sampled_oracle_near_surface() {
    let prof = flat_end_mill(6.0, 20.0);
    let from = up(DVec3::new(0.0, 0.0, 0.0));
    let to = up(DVec3::new(10.0, 0.0, 0.0));

    // 扫掠体是"圆柱沿 x 平移"形成的横向棱柱：侧面 y=±3、端面 x=−3 与 x=13
    let cases = [
        (DVec3::new(5.0, 3.0, 10.0), 0.0),    // 侧面正上方
        (DVec3::new(5.0, 3.5, 10.0), 0.5),    // 侧面外 0.5
        (DVec3::new(5.0, 2.5, 10.0), -0.5),   // 侧面内 0.5
        (DVec3::new(-3.0, 0.0, 10.0), 0.0),   // 起点端面
        (DVec3::new(-3.2, 0.0, 10.0), 0.2),   // 端面外 0.2
        (DVec3::new(13.0, 0.0, 10.0), 0.0),   // 终点端面
        (DVec3::new(13.1, 0.0, 10.0), 0.1),   // 端面外 0.1
        (DVec3::new(5.0, 0.0, 0.0), -0.0),    // 刀尖平面（z=0 是底面）
        (DVec3::new(5.0, 0.0, -0.3), 0.3),    // 底面下 0.3
    ];

    for (p, expected) in cases {
        let golden = sd_sweep(p, &from, &to, &prof);
        let sampled = sd_sweep_sampled(p, &from, &to, &prof, 400);
        // 两者都是"从上方逼近"的极小值：都不该低于解析真值
        assert!(
            golden >= expected - 1e-9,
            "{p:?}: 极小值不该低于真值 {expected}，实际 {golden}"
        );
        assert!(
            (golden - expected).abs() < 5e-4,
            "{p:?}: 解析值应为 {expected}，实际 {golden}"
        );
        assert!(
            (sampled - expected).abs() < 5e-4,
            "{p:?}: 暴力采样 {sampled} 与解析值 {expected} 差太多"
        );
    }
}

#[test]
fn sweep_is_symmetric_when_endpoints_swap() {
    let prof = flat_end_mill(6.0, 20.0);
    let a = up(DVec3::new(0.0, 0.0, 0.0));
    let b = up(DVec3::new(10.0, 3.0, -2.0));
    for p in [
        DVec3::new(5.0, 1.5, 5.0),
        DVec3::new(5.0, 5.0, 5.0),
        DVec3::new(-1.0, 0.0, 3.0),
    ] {
        let x = sd_sweep(p, &a, &b, &prof);
        let y = sd_sweep(p, &b, &a, &prof);
        assert!((x - y).abs() < 1e-12, "{p:?}: 交换端点后 {x} vs {y}");
    }
}

#[test]
fn sweep_of_ball_mill_has_round_end() {
    let prof = ball_end_mill(6.0, 20.0, 32);
    let from = up(DVec3::new(0.0, 0.0, 0.0));
    let to = up(DVec3::new(10.0, 0.0, 0.0));
    // 球头刀尖在 (0,0,0)，球心在 (0,0,3)。扫掠后起点端是"半圆柱 + 半球"。
    // 从起点往后 3mm 处沿 −x 方向，表面在 x=−3
    let on = sd_sweep(DVec3::new(-3.0, 0.0, 3.0), &from, &to, &prof);
    assert!(on.abs() < 0.02, "球头端点面应在 x=−3，实际 {on}");
    // 斜 45° 方向：球面半径 3
    let d = 3.0 / 2.0_f64.sqrt();
    let on = sd_sweep(DVec3::new(-d, d, 3.0), &from, &to, &prof);
    assert!(on.abs() < 0.03, "球面斜向点应≈0，实际 {on}");
}

/// 长行程必须靠自适应迭代次数保住精度。
///
/// 固定 8 次时 200 mm 行程的定位误差是 `200×0.236×0.618⁸ = 1.0 mm`，
/// 距离误差约 `1.0²/(2×3) = 0.17 mm`——表面位置直接错一个数量级。
#[test]
fn long_move_keeps_sweep_accuracy() {
    let prof = flat_end_mill(6.0, 20.0);
    let from = up(DVec3::new(0.0, 0.0, 0.0));
    let to = up(DVec3::new(200.0, 0.0, 0.0));

    // 侧面正上方（真实距离 0）
    let d = sd_sweep(DVec3::new(100.0, 3.0, 10.0), &from, &to, &prof);
    assert!(d.abs() < 0.005, "200mm 行程下侧面应在 0 附近，实际 {d}");

    // 侧面外 0.2 mm
    let d = sd_sweep(DVec3::new(100.0, 3.2, 10.0), &from, &to, &prof);
    assert!((d - 0.2).abs() < 0.005, "应为 0.2，实际 {d}");

    // 端面在 x = 200 + 3 = 203（刀具半径外扩），不是 x=200
    let d = sd_sweep(DVec3::new(203.0, 0.0, 10.0), &from, &to, &prof);
    assert!(d.abs() < 0.005, "终点端面应在 0 附近，实际 {d}");
    // 刀轴内部（x=200 是刀尖位置，径向 0）应是 −3
    let d = sd_sweep(DVec3::new(200.0, 0.0, 10.0), &from, &to, &prof);
    assert!((d + 3.0).abs() < 0.005, "刀轴内部应 −3，实际 {d}");
}

#[test]
fn iteration_count_scales_with_travel_and_is_bounded() {
    use cncsim_voxel::{MAX_GOLDEN_ITERS, MIN_GOLDEN_ITERS, golden_iters};

    assert_eq!(golden_iters(0.01, 0.0), MIN_GOLDEN_ITERS, "极短行程取下限");
    assert!(golden_iters(10.0, 0.0) >= 8, "10mm 行程至少要 8 次");
    assert!(
        golden_iters(200.0, 0.0) > golden_iters(10.0, 0.0),
        "行程越长迭代越多"
    );
    assert_eq!(golden_iters(1e6, 0.0), MAX_GOLDEN_ITERS, "上限必须封住");
    assert!(
        golden_iters(0.0, 50.0) > MIN_GOLDEN_ITERS,
        "刀轴转动的弧长也要计入行程"
    );
}

#[test]
fn lerp_pose_interpolates_axis_spherically() {    let a = Pose {
        tip: DVec3::ZERO,
        axis: DVec3::Z,
    };
    let b = Pose {
        tip: DVec3::new(10.0, 0.0, 0.0),
        axis: DVec3::X,
    };
    let m = lerp_pose(&a, &b, 0.5);
    assert!((m.tip.x - 5.0).abs() < 1e-12);
    // 中点刀轴应与两端各成 45°
    let c = m.axis.dot(DVec3::Z).clamp(-1.0, 1.0);
    assert!((c - 0.5_f64.sqrt()).abs() < 1e-12, "刀轴应是 45°，cos={c}");
}

// ───────────────────────── CSG 去除 ─────────────────────────

/// 在 10×10×5 毛坯上开一条 6mm 宽、2mm 深的槽，然后**量表面位置**。
#[test]
fn cut_makes_slot_of_the_right_width_and_depth() {
    let mut g = stock();
    let prof = flat_end_mill(6.0, 20.0);
    // 刀尖在 z=3（毛坯顶面 z=5 下方 2mm），沿 x 走完整个毛坯
    let stats = g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 3.0)),
        &up(DVec3::new(10.0, 5.0, 3.0)),
        &prof,
    );
    assert!(stats.changed > 0, "应当切掉材料");
    assert!(stats.visited > 0);

    // 槽底：z=3 处应是新表面（两侧材料 z<3、槽内空气 z>3）
    let d = g.sdf(DVec3::new(5.0, 5.0, 3.0));
    assert!(d.abs() < 0.005, "槽底应在 z=3，实际偏差 {d}");

    // 槽内（z=4，y=5）已无材料
    let d = g.sdf(DVec3::new(5.0, 5.0, 4.0));
    assert!(d > 0.0, "槽内应为正（无材料），实际 {d}");

    // 槽壁：y=2 处应是新表面
    let d = g.sdf(DVec3::new(5.0, 2.0, 4.0));
    assert!(d.abs() < 0.005, "槽壁应在 y=2，实际偏差 {d}");

    // 槽外（y=0.5）仍有材料
    let d = g.sdf(DVec3::new(5.0, 0.5, 4.0));
    assert!(d < 0.0, "槽外应有材料，实际 {d}");

    // 槽底以下（z=2）仍有材料
    let d = g.sdf(DVec3::new(5.0, 5.0, 2.0));
    assert!(d < 0.0, "槽底以下应有材料，实际 {d}");
}

/// 切削不能碰槽以外的材料——这是"只切该切的地方"。
#[test]
fn cut_leaves_material_outside_the_slot_untouched() {
    let mut g = stock();
    let probes = [
        DVec3::new(5.0, 0.5, 4.0),  // 槽壁外侧
        DVec3::new(5.0, 9.5, 4.0),  // 另一侧
        DVec3::new(5.0, 5.0, 1.0),  // 槽底以下
        DVec3::new(5.0, 5.0, 4.5),  // 槽内（会被切）
    ];
    let before: Vec<f64> = probes.iter().map(|p| g.sdf(*p)).collect();

    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 3.0)),
        &up(DVec3::new(10.0, 5.0, 3.0)),
        &prof,
    );

    for (i, p) in probes.iter().enumerate() {
        let after = g.sdf(*p);
        if i == 3 {
            assert!(after > before[i], "槽内应被切掉：{} → {after}", before[i]);
        } else {
            assert!(
                (after - before[i]).abs() < 1e-12,
                "{p:?} 不该被改动：{} → {after}",
                before[i]
            );
        }
    }
}

#[test]
fn cut_is_idempotent() {
    let prof = flat_end_mill(6.0, 20.0);
    let a = up(DVec3::new(0.0, 5.0, 3.0));
    let b = up(DVec3::new(10.0, 5.0, 3.0));

    let mut once = stock();
    once.cut_segment(&a, &b, &prof);

    let mut twice = stock();
    twice.cut_segment(&a, &b, &prof);
    let second = twice.cut_segment(&a, &b, &prof);
    assert_eq!(second.changed, 0, "同一段切两次不该再改动任何体素");

    for p in [
        DVec3::new(5.0, 5.0, 3.0),
        DVec3::new(5.0, 5.0, 4.0),
        DVec3::new(5.0, 2.0, 4.0),
        DVec3::new(5.0, 5.0, 2.0),
    ] {
        assert!((once.sdf(p) - twice.sdf(p)).abs() < 1e-12, "{p:?} 两次结果不同");
    }
}

/// 切空的区域必须被 `prune` 折叠成常量叶，否则碰撞会持续误报（ADR-004 §5.4）。
#[test]
fn cut_prunes_fully_removed_leaves() {
    let mut g = stock();
    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 3.0)),
        &up(DVec3::new(10.0, 5.0, 3.0)),
        &prof,
    );
    // 这个叶完全落在槽内（z 4.0–4.75，y 4.8–5.6），已被切空
    let leaf = g.leaf_at([50, 50, 45]);
    assert!(
        !matches!(leaf, Some(Leaf::Dense(_))),
        "切空的叶必须折叠成常量，实际 {leaf:?}"
    );
    if let Some(Leaf::Constant(v)) = leaf {
        assert_eq!(*v, 127, "切空后应是「远离材料」的值");
    }
}

#[test]
fn cut_does_not_touch_far_outside_voxels() {
    let mut g = stock();
    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 3.0)),
        &up(DVec3::new(10.0, 5.0, 3.0)),
        &prof,
    );
    // 毛坯上方远处
    assert_eq!(g.get([50, 50, 200]), g.background);
    // 毛坯侧面远处
    assert_eq!(g.get([300, 50, 25]), g.background);
}

#[test]
fn cut_aborts_when_sweep_is_outside_grid() {
    let mut g = stock();
    let prof = flat_end_mill(6.0, 20.0);
    // 刀完全在毛坯之外（z 远高于毛坯）
    let stats = g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 100.0)),
        &up(DVec3::new(10.0, 5.0, 100.0)),
        &prof,
    );
    assert_eq!(stats.visited, 0, "扫掠体在网格外时不该遍历任何体素");
    assert_eq!(stats.changed, 0);
}

#[test]
fn cut_on_empty_grid_is_a_noop() {
    let mut g = grid();
    let prof = flat_end_mill(6.0, 20.0);
    let stats = g.cut_segment(
        &up(DVec3::new(0.0, 0.0, 0.0)),
        &up(DVec3::new(10.0, 0.0, 0.0)),
        &prof,
    );
    assert_eq!(stats.changed, 0);
    assert_eq!(g.bounds(), None, "空网格不该被切削创建出范围");
}

/// 两步切削：先开槽，再沿 y 横切一刀，形成十字。
#[test]
fn two_cuts_form_a_cross() {
    let mut g = stock();
    let prof = flat_end_mill(6.0, 20.0);
    g.cut_segment(
        &up(DVec3::new(0.0, 5.0, 3.0)),
        &up(DVec3::new(10.0, 5.0, 3.0)),
        &prof,
    );
    g.cut_segment(
        &up(DVec3::new(5.0, 0.0, 2.5)),
        &up(DVec3::new(5.0, 10.0, 2.5)),
        &prof,
    );
    // 交叉处更深（z=2.5）
    let d = g.sdf(DVec3::new(5.0, 5.0, 2.5));
    assert!(d.abs() < 0.005, "十字交叉处底应在 z=2.5，实际偏差 {d}");
    // 第一刀的非交叉处仍是 z=3。
    // 探针取 x=1.0：第二刀的扫掠 AABB 是 x∈[2,8]，x=1 完全在它之外。
    // （取 x=2.0 会落在两刀表面的**交线**上，三线性插值在角点本来就不准，
    //   实测会读到 0.025 的偏差——那是探针选错，不是切削错。）
    let d = g.sdf(DVec3::new(1.0, 5.0, 3.0));
    assert!(d.abs() < 0.005, "第一刀槽底仍在 z=3，实际偏差 {d}");
}

/// 刀具姿态倾斜时，扫掠体也跟着倾斜——用 `sd_tool` 直接验证几何一致性。
#[test]
fn tilted_cut_matches_tool_geometry() {
    let prof = flat_end_mill(6.0, 20.0);
    // 刀轴指向 +X：刀身向 +X 伸出，刀尖在 (5,5,3)
    let pose = Pose {
        tip: DVec3::new(5.0, 5.0, 3.0),
        axis: DVec3::X,
    };
    // 沿刀轴 5mm 处的径向点：应距轴线 3mm
    let p = DVec3::new(10.0, 5.0, 6.0);
    let d = sd_tool(p, &pose, &prof);
    assert!(d.abs() < 1e-9, "径向 3mm 处应≈0，实际 {d}");
    // 刀尖正后方 2mm 是外部
    let d = sd_tool(DVec3::new(3.0, 5.0, 3.0), &pose, &prof);
    assert!((d - 2.0).abs() < 1e-9, "刀尖后方应 +2，实际 {d}");
}

/// **带旋转**的扫掠必须与暴力采样一致。
///
/// 这条是补漏的：把 `sin_cos` 的返回顺序写反过一次（`(sin, cos)` 不是 `(cos, sin)`），
/// 当时**所有测试都是绿的**——因为它们全部只用 +Z 刀轴，而平移路径已经不走三角函数了。
/// 现在这个用例专门让刀轴转 30°，把那条路径钉住。
#[test]
fn rotating_sweep_matches_sampled_oracle() {
    let prof = flat_end_mill(6.0, 20.0);
    let tilt = 30.0_f64.to_radians();
    let from = up(DVec3::new(0.0, 0.0, 0.0));
    let to = Pose {
        tip: DVec3::new(0.05, 0.0, 0.0),
        axis: DVec3::new(0.0, tilt.sin(), tilt.cos()),
    };

    // 围绕中点半径 3.2 / 4.0 的一圈点：扫掠面大致在半径 3 处
    let mid = (from.tip + to.tip) * 0.5;
    let mut worst = 0.0f64;
    for r in [2.4, 3.0, 3.2, 4.0] {
        for i in 0..16 {
            let a = std::f64::consts::TAU * i as f64 / 16.0;
            // 绕着倾斜后的轴转一圈
            let e1 = DVec3::new(1.0, 0.0, 0.0);
            let e2 = DVec3::new(0.0, tilt.cos(), -tilt.sin());
            let p = mid + (e1 * a.cos() + e2 * a.sin()) * r + DVec3::new(0.0, 0.0, 6.0) * 0.0;
            let golden = sd_sweep(p, &from, &to, &prof);
            let sampled = sd_sweep_sampled(p, &from, &to, &prof, 801);
            // 两者都是从上方逼近真极小值：黄金分割受迭代次数限制（这里 7 次，
            // 定位误差 ≈ 0.008 个参数区间），采样受网格限制。谁高一点都正常，
            // 只要求它们收敛到同一个数。
            worst = worst.max((golden - sampled).abs());
        }
    }
    println!("带旋转扫掠：最大偏差 {:.2e} mm", worst);
    assert!(worst < 1e-3, "与采样 oracle 的差 {worst:.2e} 超过迭代精度");
}

/// 沿单轴走刀的**截面共享路径**与**通用逐体素路径**必须逐位一致。
///
/// 构造：同一条扫掠线，一次切完 vs 拆成五段。
/// - 一次切完：叶大多整个落在走刀段内 → 走截面共享（sd 算 64 次而非 512 次）
/// - 拆成五段：每段变短，大量叶跨越段边界 → 退回通用逐体素路径
///
/// 两者几何等价（纯平移扫掠体的并集就是那条长扫掠体），所以体素值必须**逐位相同**。
/// 这条测试是"沿轴复用 SDF"这个优化的 correctness 锁。
#[test]
fn axis_sharing_matches_general_path() {
    let prof = flat_end_mill(6.0, 20.0);
    let mut a = stock();
    let mut b = stock();

    // 一次切完（25 mm 长，叶基本都在段内 → 共享路径）
    a.cut_segment(
        &up(DVec3::new(5.0, 5.0, 3.0)),
        &up(DVec3::new(25.0, 5.0, 3.0)),
        &prof,
    );
    // 拆成五段 4 mm（叶频繁跨段 → 通用路径）
    for i in 0..5 {
        let x0 = 5.0 + i as f64 * 4.0;
        b.cut_segment(
            &up(DVec3::new(x0, 5.0, 3.0)),
            &up(DVec3::new(x0 + 4.0, 5.0, 3.0)),
            &prof,
        );
    }

    // 抽查：步长 3，覆盖整个可能被影响的区域
    let mut checked = 0usize;
    for k in (15..60).step_by(3) {
        for j in (0..120).step_by(3) {
            for i in (0..280).step_by(3) {
                let ijk = [i, j, k];
                assert_eq!(
                    a.to_distance(a.get(ijk)),
                    b.to_distance(b.get(ijk)),
                    "体素 {ijk:?}：一次切完 vs 拆五段结果不同"
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 10_000, "抽查点太少：{checked}");
}

// ───────────────────────── 批量切削 ─────────────────────────

/// 一段混合刀路：x 向、y 向、斜向、沿 z 下刀、锥形刀都来一遍。
///
/// 批量路径的正确性不能只测"沿 x 直刀"——那正好落进单轴共享路径。
fn mixed_program() -> Vec<(Pose, Pose)> {
    vec![
        // x 向直刀（单轴共享路径）
        (up(DVec3::new(1.0, 4.0, 3.0)), up(DVec3::new(9.0, 4.0, 3.0))),
        // 返回时错开半步距（重叠，制造"同一个叶被多段碰到"）
        (up(DVec3::new(9.0, 4.4, 3.0)), up(DVec3::new(1.0, 4.4, 3.0))),
        // y 向直刀
        (up(DVec3::new(6.0, 1.0, 3.0)), up(DVec3::new(6.0, 9.0, 3.0))),
        // 斜向（通用路径）
        (up(DVec3::new(1.0, 8.0, 2.0)), up(DVec3::new(9.0, 9.5, 2.0))),
        // 沿 z 下刀（没有单轴共享性质）
        (up(DVec3::new(9.0, 2.0, 5.0)), up(DVec3::new(9.0, 2.0, 1.0))),
        // 再压一刀，切深不同
        (up(DVec3::new(2.0, 2.0, 1.5)), up(DVec3::new(8.0, 2.0, 1.5))),
    ]
}

fn assert_same_grid(a: &SparseGrid<i8>, b: &SparseGrid<i8>, what: &str) {
    let (lo, hi) = a.bounds().expect("应有范围");
    assert_eq!(b.bounds(), Some((lo, hi)), "{what}：范围不一致");
    for z in lo[2]..hi[2] {
        for y in lo[1]..hi[1] {
            for x in lo[0]..hi[0] {
                let ijk = [x, y, z];
                assert_eq!(a.get(ijk), b.get(ijk), "{what}：体素 {ijk:?} 不一致");
            }
        }
    }
    // 叶数是**结构细节**，不是契约——tile 与 `Constant` 叶在几何上完全等价
    // （上面已逐体素比对过 `bounds` 范围内的每一个点）。
    //
    // 为什么两边会不同：批量"看得见未来"，第一段就发现整叶被埋进刀体，
    // 于是把 tile 原地改值（tile 就是"整叶同值"的原生表示，零分配）；
    // 逐刀必须逐刀决策，可能先造一个 `Dense` 叶、后续刀才把它折叠成 `Constant`。
    // 实测这个样本 1028 vs 825，多出来的 203 个全是这种"先部分切、后整叶切空"的叶。
    //
    // 所以只要求**批量不多造叶**——批量反而更多才说明短路失效了。
    assert!(
        b.leaf_count() <= a.leaf_count(),
        "{what}：批量不该比逐刀多出叶（{} vs {}），多出来说明整叶短路失效",
        b.leaf_count(),
        a.leaf_count()
    );
}

/// **批量必须与逐刀逐位相同**。这是批量入口唯一的存在理由：
/// 它只是"少遍历几遍"，不是"另一种近似"。
#[test]
fn batch_matches_one_cut_at_a_time() {
    let prof = flat_end_mill(6.0, 20.0);
    let program = mixed_program();

    let mut seq = stock();
    for (from, to) in &program {
        seq.cut_segment(from, to, &prof);
    }

    let mut bat = stock();
    let stats = bat.cut_batch(&program, &prof);
    assert!(stats.changed > 0, "样本必须真削掉材料");

    assert_same_grid(&seq, &bat, "逐刀 vs 批量");
    assert!(
        (seq.material_volume() - bat.material_volume()).abs() < 1e-12,
        "材料体积不一致"
    );
}

/// 批量的片数也只是"怎么算"：切开之后结果仍必须逐位相同。
#[test]
fn batch_slice_count_does_not_change_the_grid() {
    let prof = flat_end_mill(6.0, 20.0);
    let program = mixed_program();

    let mut a = stock();
    a.cut_batch_with(&program, &prof, 1);
    let mut b = stock();
    b.cut_batch_with(&program, &prof, 17);
    assert_same_grid(&a, &b, "1 片 vs 17 片");
}

/// 单段批量 == `cut_segment`：两条入口共用一套内核，连统计都要一样。
#[test]
fn batch_of_one_equals_cut_segment() {
    let prof = flat_end_mill(6.0, 20.0);
    let from = up(DVec3::new(1.0, 5.0, 3.0));
    let to = up(DVec3::new(9.0, 5.0, 3.0));

    let mut a = stock();
    let sa = a.cut_segment(&from, &to, &prof);
    let mut b = stock();
    let sb = b.cut_batch(&[(from, to)], &prof);

    assert_eq!(sa, sb, "单段批量与 cut_segment 统计应完全一致");
    assert_same_grid(&a, &b, "单段批量 vs cut_segment");
}

/// 空批、整批在网格外：都不该改动任何东西，也不该创建范围。
#[test]
fn empty_or_outside_batch_is_a_noop() {
    let prof = flat_end_mill(6.0, 20.0);
    let mut g = grid();
    let empty = g.cut_batch(&[], &prof);
    assert_eq!((empty.visited, empty.changed), (0, 0));

    let outside = g.cut_batch(
        &[
            (up(DVec3::new(0.0, 0.0, 100.0)), up(DVec3::new(5.0, 0.0, 100.0))),
            (up(DVec3::new(-40.0, 0.0, -40.0)), up(DVec3::new(-30.0, 0.0, -40.0))),
        ],
        &prof,
    );
    assert_eq!((outside.visited, outside.changed), (0, 0));
    assert_eq!(g.bounds(), None, "空网格不该被切削创建出范围");
}

/// 批量里"重复切同一段"仍然幂等：第二遍一处都不改。
#[test]
fn batch_is_idempotent() {
    let prof = flat_end_mill(6.0, 20.0);
    let program = mixed_program();
    let mut g = stock();
    let first = g.cut_batch(&program, &prof);
    let second = g.cut_batch(&program, &prof);
    assert!(first.changed > 0);
    assert_eq!(second.changed, 0, "同一批切两次不该再改动任何体素");
}
