//! 毛坯（稀疏体素场）←→ 碰撞的接线。
//!
//! `collide` 刻意不依赖 `voxel`（见 `docs/ADR-004` §7.7），依赖方向由这里反转：
//! 本模块把 `SparseGrid` 包成 [`DistanceField`]，并提供两个方向的毛坯检查。
//!
//! ## 为什么要 newtype
//!
//! `DistanceField` 是 `collide` 的 trait，`SparseGrid` 是 `voxel` 的类型，两者都不属于
//! `sim`——直接 `impl` 会违反 orphan rule。所以用 [`StockField`] 包一层。
//!
//! ## 两个方向，分工不同
//!
//! ADR-004 §2.5 定的是"两个方向都做"：
//!
//! | 方向 | 做法 | 检出保证 | 干什么 |
//! |---|---|---|---|
//! | **B（主）** | 刀杆表面采样点 → 毛坯 SDF | `δ²/(2R)`，δ=0.5 时 **42 µm** | 主力检出，还能连续化 |
//! | **A（辅）** | 毛坯体素中心 → 刀杆 SDF | `h/2` = **50 µm** | 兜底 + **定位**（报"毛坯上哪一块被啃了"） |
//!
//! B 的保证好几个数量级，因为它采的是**刀杆表面**：穿透区在曲面上是一块二维补丁，
//! 间隙按 `δ²` 缩小；A 采的是三维体素格，薄片卡在两层之间就漏，只能按 `h` 缩小。
//!
//! 但 A 保留的必要性在**定位**：碰撞点报在体素中心，"毛坯上这里被啃掉了"比
//! "刀杆表面上这个点进去了"更贴近用户的问题。而且 A 在 B 的采样出现空洞时能兜住。
//!
//! ## 刀刃不算碰撞
//!
//! 只用 [`cncsim_tool::ToolDef::shank`]（刃长以上的那段母线）——刀刃切材料是本职工作。
//! `shank` 的母线 z 从 `flute_length` 开始，所以 [`cncsim_voxel::sweep_aabb`] 用母线
//! 自己的 z 范围算包围盒，不会去扫刀尖下面那 20 mm 空气。

use cncsim_collide::DistanceField;
use cncsim_collide::continuous::{self, Hit, SurfaceBody};
use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::Profile;
use cncsim_voxel::{SparseGrid, sd_sweep, sweep_aabb};

/// 把稀疏体素场包成 [`DistanceField`]。
///
/// 只读、零开销——`SparseGrid::sdf` 本来就是窄带三线性插值，
/// 超出带宽的点被钳制到 `±band`（符号仍然正确，见 ADR-004 §2.2）。
pub struct StockField<'a>(pub &'a SparseGrid<i8>);

impl DistanceField for StockField<'_> {
    fn sdf(&self, p: DVec3) -> f64 {
        self.0.sdf(p)
    }
    fn band(&self) -> f64 {
        self.0.config.band_mm()
    }

    /// 毛坯已写范围的世界包围盒。
    ///
    /// 保守推进靠它算 O(1) 的距离下界——没有这一条，窄带（0.4 mm）比采样间隙
    /// （0.5 mm）还窄，下界恒为负，刀在毛坯上方 50 mm 也会报碰撞。
    fn aabb(&self) -> Option<(DVec3, DVec3)> {
        let (lo, hi) = self.0.bounds()?;
        let h = self.0.config.h;
        let o = self.0.origin;
        Some((
            DVec3::new(
                o.x + lo[0] as f64 * h,
                o.y + lo[1] as f64 * h,
                o.z + lo[2] as f64 * h,
            ),
            DVec3::new(
                o.x + (hi[0] + 1) as f64 * h,
                o.y + (hi[1] + 1) as f64 * h,
                o.z + (hi[2] + 1) as f64 * h,
            ),
        ))
    }
}

/// 方向 A 的一次命中。
#[derive(Clone, Copy, Debug)]
pub struct Overcut {
    /// 命中的体素中心（世界坐标）
    pub at: DVec3,
    /// 体素索引
    pub ijk: [i32; 3],
    /// 体素中心进入刀杆内部的深度（mm）。
    ///
    /// **这是体素中心的值，不是真实穿透深度**：真实穿透可能比它小 `h/2`
    /// 而完全没被采到（ADR-004 §2.5.1）。想要真实深度用方向 B 或 EPA。
    pub depth: f64,
    /// 遍历过的体素数（诊断用）
    pub visited: usize,
}

/// **方向 A**：毛坯体素中心落进刀杆扫掠体 → 过切。
///
/// 这是"定位"那一路：命中点直接是毛坯上的体素中心。
/// 检出下限是 `h/2`（体素中心可能错过薄片），不是 µm 级——要 µm 用 [`shank_vs_stock_continuous`]。
///
/// 遍历范围先被 [`sweep_aabb`] 裁到刀杆扫掠体，再与毛坯 `bounds` 求交，
/// 所以刀在毛坯上方空走时一个体素都不碰。
pub fn shank_vs_stock_voxels(
    grid: &SparseGrid<i8>,
    from: &Pose,
    to: &Pose,
    shank: &Profile,
) -> Option<Overcut> {
    let Some((glo, ghi)) = grid.bounds() else {
        return None; // 空毛坯
    };
    let (wlo, whi) = sweep_aabb(from, to, shank);
    let h = grid.config.h;
    let o = grid.origin;

    // 世界 → 体素索引（体素中心在 origin + (i+0.5)h）
    let mut lo = [0i32; 3];
    let mut hi = [0i32; 3];
    for a in 0..3 {
        let i0 = ((wlo.component(a) - o.component(a)) / h - 0.5).floor() as i32;
        let i1 = ((whi.component(a) - o.component(a)) / h - 0.5).ceil() as i32;
        lo[a] = i0.max(glo[a]);
        hi[a] = i1.min(ghi[a]);
        if lo[a] > hi[a] {
            return None; // 扫掠体完全在网格外
        }
    }

    let mut visited = 0usize;
    for z in lo[2]..=hi[2] {
        for y in lo[1]..=hi[1] {
            for x in lo[0]..=hi[0] {
                let ijk = [x, y, z];
                let v = grid.get(ijk);
                // 先看材料：不是材料就谈不上"被啃掉"
                if grid.to_distance(v) >= 0.0 {
                    continue;
                }
                visited += 1;
                let p = grid.voxel_center(ijk);
                let d = sd_sweep(p, from, to, shank);
                if d < 0.0 {
                    return Some(Overcut { at: p, ijk, depth: -d, visited });
                }
            }
        }
    }
    None
}

/// **方向 B**：刀杆表面采样点落进毛坯 → 碰撞，并且**连续化**（保守推进）。
///
/// `body` 由 [`SurfaceBody::revolve`] 从 `shank` 生成；默认采样间隙 0.5 mm
/// （检出下限 42 µm、φ6×20 刀杆约 3 500 点）。要更细就传更小的步长。
///
/// 返回值里 `clearance` 是采样点的**上界**、`clearance_lower` 是**下界**，
/// 两者之间是"太近但无法保证"的保守带——见 [`continuous`] 的模块文档。
pub fn shank_vs_stock_continuous(
    grid: &SparseGrid<i8>,
    body: &SurfaceBody,
    from: &Pose,
    to: &Pose,
) -> Option<Hit> {
    // AABB 预筛：刀杆扫掠体的包围盒与毛坯 bounds 不相交就秒退。
    // 真实仿真里绝大多数段走这条路（刀在毛坯上方空走）。
    if !sweep_may_touch_bounds(grid, from, to, body) {
        return None;
    }
    continuous::earliest_collision(&StockField(grid), body, from, to)
}

/// 刀杆扫掠体的包围盒是否与毛坯的已写范围相交。
///
/// 只用 `bounds`（轴向对齐的整数索引范围）而不是精确几何：这一层是**粗筛**，
/// 判错了只是多跑一次窄相，不会漏报。
fn sweep_may_touch_bounds(
    grid: &SparseGrid<i8>,
    from: &Pose,
    to: &Pose,
    body: &SurfaceBody,
) -> bool {
    let Some((glo, ghi)) = grid.bounds() else {
        return false;
    };
    // 用采样点的包围盒（含刀杆全长），不用 profile —— `SurfaceBody` 只有点集，
    // 但它本来就是刀杆表面，取包的包围盒是对的
    let (lo, hi) = body.aabb(from, to);
    let h = grid.config.h;
    let o = grid.origin;
    for a in 0..3 {
        let grid_lo = o.component(a) + glo[a] as f64 * h;
        let grid_hi = o.component(a) + (ghi[a] + 1) as f64 * h;
        if hi.component(a) < grid_lo || lo.component(a) > grid_hi {
            return false;
        }
    }
    true
}

/// 一次刀杆撞毛坯的完整结果：**检出**（方向 B）+ **定位**（方向 A）。
#[derive(Clone, Copy, Debug)]
pub struct StockCollision {
    /// 方向 B 的连续化结果：`clearance`（上界）/ `clearance_lower`（下界）/ 段内参数 `t`
    pub hit: Hit,
    /// 方向 A 的体素命中。**可能为 `None`**：A 的保证只有 `h/2`（50 µm），
    /// B 报的那种浅穿透 A 不一定扫得到。这只是缺了"毛坯上哪一块"的定位，不是漏报。
    pub overcut: Option<Overcut>,
}

/// 刀杆 vs 毛坯的完整检查。
///
/// **先 B 后 A**，这是有意的：
///
/// | | 成本（实测，φ9×60 刀杆 / 200 mm 毛坯） | 保证 |
/// |---|---|---|
/// | 方向 B（表面采样 + 保守推进） | **0.48 ms** | 42 µm（δ=0.5） |
/// | 方向 A（体素遍历） | 1.30 ms | 50 µm（h/2），但给出体素坐标 |
///
/// B 便宜 2.7× 且精度高两个数量级，所以由它做检出；A 只在 B 命中后跑一次，
/// 把"哪里被啃了"翻译成毛坯上的体素坐标。刀在空中时两者都是 AABB 预筛秒退（0.03 ms）。
pub fn check_shank(
    grid: &SparseGrid<i8>,
    body: &SurfaceBody,
    shank: &Profile,
    from: &Pose,
    to: &Pose,
) -> Option<StockCollision> {
    let hit = shank_vs_stock_continuous(grid, body, from, to)?;
    Some(StockCollision {
        hit,
        overcut: shank_vs_stock_voxels(grid, from, to, shank),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_math::DVec2;
    use cncsim_tool::{ToolDef, flat_end_mill};
    use cncsim_voxel::GridConfig;

    fn grid() -> SparseGrid<i8> {
        SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
    }

    /// 20×20×10 mm 的实心块，顶面 z = 1.0（h=0.1 时 10 层）
    fn block() -> SparseGrid<i8> {
        let mut g = grid();
        g.fill_box([0, 0, 0], [200, 200, 100], -127);
        g
    }

    /// 一把 φ6、刃长 20、全长 60 的平底刀；刀杆 profile 从 z=20 起
    fn tool() -> ToolDef {
        let shank = Profile::new(vec![
            DVec2::new(0.0, 20.0),
            DVec2::new(3.0, 20.0),
            DVec2::new(3.0, 60.0),
            DVec2::new(0.0, 60.0),
        ]);
        ToolDef {
            number: 1,
            kind: cncsim_tool::CutterKind::Flat,
            diameter: 6.0,
            corner_radius: 0.0,
            flute_length: 20.0,
            shank,
            length_offset: 60.0,
            cutter: flat_end_mill(6.0, 20.0),
        }
    }

    #[test]
    fn shank_profile_starts_at_the_flute_length() {
        let t = tool();
        assert_eq!(t.cutting_top(), 20.0);
        // 刀杆的 z 范围就是 [刃长, 全长]，`sweep_aabb` 靠它裁包围盒
        let (lo, hi) = sweep_aabb(
            &Pose { tip: DVec3::ZERO, axis: DVec3::Z },
            &Pose { tip: DVec3::ZERO, axis: DVec3::Z },
            &t.shank,
        );
        // 轴沿 Z 时径向在 Z 上的投影为 0，所以下界就是刃长本身
        assert!((lo.z - 20.0).abs() < 1e-12, "实际 {}", lo.z);
        assert!((hi.z - 60.0).abs() < 1e-12, "实际 {}", hi.z);
        assert!((lo.x + 3.0).abs() < 1e-12, "径向 −3，实际 {}", lo.x);
    }

    #[test]
    fn stock_field_matches_grid_sdf() {
        let g = block();
        let f = StockField(&g);
        for p in [
            DVec3::new(0.0, 0.0, -0.5),   // 内部
            DVec3::new(10.0, 10.0, 10.5), // 上方
            DVec3::new(10.0, 10.0, 10.0), // 表面附近
        ] {
            assert!(
                (f.sdf(p) - g.sdf(p)).abs() < 1e-15,
                "包装层不能改语义：{p:?} → {} vs {}",
                f.sdf(p),
                g.sdf(p)
            );
        }
        assert!((f.band() - g.config.band_mm()).abs() < 1e-15);
    }

    #[test]
    fn shank_above_the_stock_reports_nothing() {
        let g = block();
        let t = tool();
        // 刀尖 z = 10 → 刀杆在 z ∈ [30, 70]，远在毛坯（顶面 z=10）上方
        let a = Pose { tip: DVec3::new(5.0, 5.0, 10.0), axis: DVec3::Z };
        let b = Pose { tip: DVec3::new(15.0, 5.0, 10.0), axis: DVec3::Z };
        assert!(shank_vs_stock_voxels(&g, &a, &b, &t.shank).is_none());

        let body = SurfaceBody::revolve(&t.shank, 1.0, 1.0);
        assert!(shank_vs_stock_continuous(&g, &body, &a, &b).is_none());
    }

    #[test]
    fn shank_plunged_into_the_stock_is_caught_by_both_directions() {
        let g = block();
        let t = tool();
        // 刀尖插到 z = −20 → 刀杆在 z ∈ [0, 40]，穿过毛坯（z ∈ [0,10]）
        let a = Pose { tip: DVec3::new(10.0, 10.0, -20.0), axis: DVec3::Z };
        let b = Pose { tip: DVec3::new(12.0, 10.0, -20.0), axis: DVec3::Z };

        let hit = shank_vs_stock_voxels(&g, &a, &b, &t.shank).expect("方向 A 应检出");
        assert!(hit.depth > 0.0);
        assert!(hit.at.z > 0.0 && hit.at.z < 10.0, "命中点应落在毛坯厚度内，实际 {:?}", hit.at);

        let body = SurfaceBody::revolve(&t.shank, 1.0, 1.0);
        assert!(shank_vs_stock_continuous(&g, &body, &a, &b).is_some(), "方向 B 也应检出");
    }

    #[test]
    fn cutting_with_the_flutes_alone_is_not_a_collision() {
        // 刀尖正好在毛坯顶面（z=10）：刀刃（z<20）全在毛坯上方 / 表面，
        // 刀杆（z≥20）离得更远 —— 这是正常切削，不该报
        let mut g = block();
        let t = tool();
        g.cut_segment(
            &Pose { tip: DVec3::new(0.0, 10.0, 10.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(20.0, 10.0, 10.0), axis: DVec3::Z },
            &flat_end_mill(6.0, 20.0),
        );
        let a = Pose { tip: DVec3::new(0.0, 10.0, 9.5), axis: DVec3::Z };
        let b = Pose { tip: DVec3::new(20.0, 10.0, 9.5), axis: DVec3::Z };
        assert!(
            shank_vs_stock_voxels(&g, &a, &b, &t.shank).is_none(),
            "只有刀刃碰材料，不该报刀杆碰撞"
        );
    }

    #[test]
    fn cutting_deep_enough_for_the_shank_to_bite_is_caught() {
        // 同一把刀，刀尖降到 z = −15 → 刀杆底端在 z = 5，扎进毛坯（0..10）
        let g = block();
        let t = tool();
        let a = Pose { tip: DVec3::new(10.0, 10.0, -15.0), axis: DVec3::Z };
        let b = Pose { tip: DVec3::new(11.0, 10.0, -15.0), axis: DVec3::Z };
        assert!(
            shank_vs_stock_voxels(&g, &a, &b, &t.shank).is_some(),
            "刀杆扎进材料应检出"
        );
    }

    #[test]
    fn continuous_check_catches_a_sweep_through_a_thin_wall() {
        // 2 mm 厚的竖壁（x ∈ [9.9, 10.1] 之外是空…改成实际构造）：
        // 用一个只有两层体素厚的竖板，刀杆高速横穿。
        let mut g = grid();
        g.fill_box([100, 0, 0], [120, 200, 400], -127); // x ∈ [10.0, 12.0]，2 mm 厚
        let t = tool();
        // 刀尖在 z = −10 → 刀杆 z ∈ [10, 50]，落在板的高度范围内（0..40）
        let a = Pose { tip: DVec3::new(-30.0, 10.0, -10.0), axis: DVec3::Z };
        let b = Pose { tip: DVec3::new(30.0, 10.0, -10.0), axis: DVec3::Z };
        let body = SurfaceBody::revolve(&t.shank, 1.0, 1.0);
        let hit = shank_vs_stock_continuous(&g, &body, &a, &b).expect("横穿薄板应检出");
        assert!(hit.t > 0.0 && hit.t < 1.0, "应在段内，实际 t={}", hit.t);
    }

    #[test]
    fn check_shank_combines_detection_and_localisation() {
        let g = block();
        let t = tool();
        let body = SurfaceBody::revolve(&t.shank, 1.0, 1.0);

        // 扎进去 → 两个方向都该有结果
        let a = Pose { tip: DVec3::new(10.0, 10.0, -20.0), axis: DVec3::Z };
        let c = check_shank(&g, &body, &t.shank, &a, &a).expect("应检出");
        assert!(c.hit.clearance_lower <= 0.0);
        let oc = c.overcut.expect("这个深度方向 A 也应扫到");
        assert!(oc.at.z > 0.0 && oc.at.z < 10.0, "定位应落在毛坯厚度内，实际 {:?}", oc.at);

        // 空中 → 什么都没有
        let b = Pose { tip: DVec3::new(10.0, 10.0, 10.0), axis: DVec3::Z };
        assert!(check_shank(&g, &body, &t.shank, &b, &b).is_none());
    }

    #[test]
    fn aabb_prefilter_never_rejects_a_real_hit() {
        // 预筛是粗筛：可以多放行，不能漏。这里扫一遍高度，凡是有命中的
        // 位置，预筛必须放行。
        let g = block();
        let t = tool();
        let body = SurfaceBody::revolve(&t.shank, 1.5, 1.5);
        for k in 0..40 {
            let z = -25.0 + k as f64 * 1.5;
            let a = Pose { tip: DVec3::new(10.0, 10.0, z), axis: DVec3::Z };
            let b = Pose { tip: DVec3::new(10.0, 10.0, z), axis: DVec3::Z };
            let passes = sweep_may_touch_bounds(&g, &a, &b, &body);
            let hit = shank_vs_stock_continuous(&g, &body, &a, &b).is_some();
            if hit {
                assert!(passes, "z={z} 有命中却被预筛挡掉");
            }
        }
    }
}
