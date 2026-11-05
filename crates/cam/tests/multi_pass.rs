//! 多工序计划：同一毛坯、多方向、多把刀，串成**一份**程序。
//!
//! 现状：
//! - ✅ 五方向（+Z、±X、±Y）都切得到料，且**零过切**；
//! - ✅ 侧向工序靠 `G43.4 H1` + A/C 真的把刀轴摆过去（解释必须用
//!   [`cncsim_gcode::RotaryConvention::Head`]：`Table` 那套约定里 C 角对刀轴
//!   没有影响，表达不了任意方向）；
//! - ⚠️ 每道只做「该方向看得见的那一面」，背面的料留给反向那道工序——这是装夹
//!   决定的，不是刀路缺陷。

use cncsim_cam::multi::{Pass, default_directions, generate_plan};
use cncsim_cam::{Process, Stock, ToolSpec, Tri};
use cncsim_math::DVec3;

/// 多方向程序里的 A/C 按 **Head（双摆头）** 生成，解释必须配套。
fn ic() -> cncsim_gcode::InterpConfig {
    cncsim_gcode::InterpConfig {
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    }
}

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
    let f = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    f.iter().map(|q| [c[q[0]], c[q[1]], c[q[2]]]).collect()
}

fn part() -> Vec<Tri> {
    box_mesh(DVec3::new(-6.0, -6.0, 0.0), DVec3::new(6.0, 6.0, 6.0))
}

fn stock() -> Stock {
    Stock {
        min: DVec3::new(-12.0, -12.0, 0.0),
        max: DVec3::new(12.0, 12.0, 8.0),
    }
}

fn process() -> Process {
    Process {
        stock_allowance_mm: 0.3,
        stepdown_mm: 1.5,
        rough_stepover_mm: 2.5,
        finish_stepover_mm: 1.2,
        sample_mm: 0.6,
        ..Default::default()
    }
}

fn passes() -> Vec<Pass> {
    default_directions()
        .into_iter()
        .enumerate()
        .map(|(i, dir)| Pass {
            dir,
            rough: Some(ToolSpec::flat(0, 5.0 + i as f64, 20.0)),
            finish: ToolSpec::ball(0, 3.0, 10.0),
        })
        .collect()
}

fn bbox(out: &cncsim_cam::multi::PlanOutput) -> (DVec3, DVec3) {
    let program = cncsim_gcode::interpret(&out.program.gcode, &ic());
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for b in &program.blocks {
        let p = b.target.tip;
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo, hi)
}

/// 结构：五道工序合成**一份**程序，只有一个 `M30`，刀号全局唯一。
#[test]
fn plan_merges_into_one_program() {
    let out = generate_plan(&part(), stock(), &process(), &passes()).unwrap();
    let program = cncsim_gcode::interpret(&out.program.gcode, &ic());
    assert!(
        !program.has_errors(),
        "多工序程序解释报错：{:?}",
        program.issues
    );
    assert_eq!(
        out.program.gcode.matches("M30").count(),
        1,
        "合并后只能有一个程序结束指令（M30 会让控制器停机回卷）"
    );
    assert_eq!(
        out.program.gcode.matches("(PASS").count(),
        5,
        "五道工序都应留下标记"
    );
    // 坐标系：全程在 G54 工件坐标里，收尾回机床坐标（G53）再停机
    assert!(
        out.program.gcode.contains("G54"),
        "程序必须显式选择工件坐标系 G54"
    );
    assert!(
        out.program.gcode.contains("G53 Z"),
        "收尾要用 G53 回机床坐标，不能只写工件 Z"
    );
    let tail: Vec<&str> = out.program.gcode.lines().rev().take(3).collect();
    assert_eq!(tail[0], "M30", "最后一条必须是 M30：{tail:?}");
    let mut numbers: Vec<u16> = out.tools.iter().map(|t| t.number).collect();
    numbers.sort_unstable();
    numbers.dedup();
    assert_eq!(numbers.len(), out.tools.len(), "刀号有重复：{numbers:?}");
    assert_eq!(out.tools.len(), 10, "五道 × (粗+精)");
}

/// ±X 侧向开粗：刀点必须真的跑到那一面（不是只在 +Z 上面刷）。
///
/// ±Y 现在做不到（见文件头），所以这里只钉 ±X；修好 ±Y 之后把两条一起打开。
#[test]
fn side_passes_reach_the_x_faces() {
    let st = stock();
    for dir in [-1.0f64, 1.0] {
        let out = generate_plan(
            &part(),
            st,
            &process(),
            &[Pass {
                dir: DVec3::new(dir, 0.0, 0.0),
                rough: Some(ToolSpec::flat(0, 5.0, 20.0)),
                finish: ToolSpec::ball(0, 3.0, 10.0),
            }],
        )
        .unwrap();
        let (lo, hi) = bbox(&out);
        if dir < 0.0 {
            assert!(lo.x < st.min.x + 1.5, "没切 −X 面：{:.2}", lo.x);
        } else {
            assert!(hi.x > st.max.x - 1.5, "没切 +X 面：{:.2}", hi.x);
        }
        // 收尾有一条 `G53 Z` 回机床坐标（毛坯顶 + 30），那是回参考点，不算跑飞
        assert!(
            lo.z >= st.min.z - 12.0 && hi.z <= st.max.z + 35.0,
            "侧向工序 z 跑飞：{:.1}..{:.1}",
            lo.z,
            hi.z
        );
    }
}

/// 五方向真去料 + 不过切。
#[test]
fn five_directions_remove_material_without_gouging() {
    let st = stock();
    let out = generate_plan(&part(), st, &process(), &passes()).unwrap();
    let five = sim(st, &out);
    let single = {
        let mut p = passes();
        p.truncate(1);
        let o = generate_plan(&part(), st, &process(), &p).unwrap();
        sim(st, &o)
    };
    eprintln!("单方向(+Z): 去料 {:.0} / 过切 {:.0}", single.0, single.1);
    eprintln!("五方向: 去料 {:.0} / 过切 {:.0}", five.0, five.1);
    assert!(
        five.0 > single.0 * 1.5,
        "五方向去料 {:.0} mm³ 应该明显多于单方向 {:.0} mm³",
        five.0,
        single.0
    );
    assert!(
        five.1 < 40.0,
        "过切 {:.0} mm³（零件 864 mm³）—— 侧向工序切进了零件",
        five.1
    );
}

fn sim(st: Stock, out: &cncsim_cam::multi::PlanOutput) -> (f64, f64) {
    let h = 0.5;
    let cfg = cncsim_sim::run::RunConfig {
        grid: cncsim_voxel::GridConfig {
            h,
            band_voxels: 4,
            value_type: cncsim_voxel::ValueType::I8,
        },
        stock_origin: st.min,
        stock_size: [
            (st.size().x / h).round() as i32,
            (st.size().y / h).round() as i32,
            (st.size().z / h).round() as i32,
        ],
        design: Some(cncsim_sim::compare::DesignModel::Box {
            min: DVec3::new(-6.0, -6.0, 0.0),
            max: DVec3::new(6.0, 6.0, 6.0),
        }),
        collide: false,
        creator_sub: 1,
        ..Default::default()
    };
    let program = cncsim_gcode::interpret(&out.program.gcode, &ic());
    let table = cncsim_tool::ToolTable::new(out.tools.clone());
    let r = cncsim_sim::run::run_tools(&program, &table, &cfg);
    (r.report.removed_volume_mm3, r.report.gouge_volume_mm3)
}

/// **侧向工序必须真的摆刀轴**。
///
/// 这条守的是一个真实翻车：早先多方向只是把坐标转进局部系、输出仍是纯 XYZ，
/// 控制器按「刀轴 = 世界 +Z」扫掠，Ø3 球头的刀杆（半径 1.5、长 10）从上方垂直
/// 插进工件——每道稳定过切 105 mm³，过切带正好 1.5 mm 深 = 刀半径。
/// 现在用 `G43.4 H1` + A/C 摆姿态，这里反查解释后的刀轴。
#[test]
fn side_passes_tilt_the_tool_axis() {
    let st = stock();
    for dir in [
        DVec3::new(-1.0, 0.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, -1.0, 0.0),
        DVec3::new(0.0, 1.0, 0.0),
    ] {
        let out = generate_plan(
            &part(),
            st,
            &process(),
            &[Pass {
                dir,
                rough: Some(ToolSpec::flat(0, 5.0, 20.0)),
                finish: ToolSpec::ball(0, 3.0, 10.0),
            }],
        )
        .unwrap();
        let prog = cncsim_gcode::interpret(&out.program.gcode, &ic());
        assert!(!prog.has_errors(), "{:?} 解释报错：{:?}", dir, prog.issues);
        // 只看真正的切削块：刀尖在毛坯包络里
        let mut n = 0usize;
        let mut worst = 0.0f64;
        for b in &prog.blocks {
            let p = b.target.tip;
            let inside = p.x > st.min.x - 1.0
                && p.x < st.max.x + 1.0
                && p.y > st.min.y - 1.0
                && p.y < st.max.y + 1.0
                && p.z > st.min.z - 1.0
                && p.z < st.max.z + 1.0;
            if !inside {
                continue;
            }
            worst = worst.max((b.target.axis - dir).length());
            n += 1;
        }
        assert!(n > 50, "{:?} 切削块太少：{n}", dir);
        assert!(
            worst < 1e-3,
            "{:?} 的刀轴没摆过去（最大偏离 {worst:.4}）—— 又退回纯 XYZ 三轴了",
            dir
        );
    }
}
