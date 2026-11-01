//! 演示程序：**面铣一层**（默认）与**型腔 + 钻孔**（`--demo pocket`）。
//!
//! 放在 `sim` 而不是 `app`：这两段 G 代码是**测试对象**，测试要能直接拿到，
//! 而不是把字符串在 `app/main.rs` 里抄一遍——抄一遍之后，测试通过的那个程序
//! 和窗口里跑的那个就可能是两份了。
//!
//! `app` 只负责把它们交给 `cncsim_gcode::interpret`，外加 `--design` 之类的 I/O。

use cncsim_math::DVec2;
use cncsim_tool::{CutterKind, Profile, ToolDef, flat_end_mill};

/// 演示用的 φ6 平底刀：刃长 20 mm，刀杆到 60 mm。
///
/// 刃长 20 > 毛坯高 12，所以刀杆永远在料上方——演示程序不会报碰撞。
pub fn tool() -> ToolDef {
    ToolDef {
        number: 1,
        kind: CutterKind::Flat,
        diameter: 6.0,
        corner_radius: 0.0,
        flute_length: 20.0,
        shank: Profile::new(vec![
            DVec2::new(0.0, 20.0),
            DVec2::new(3.0, 20.0),
            DVec2::new(3.0, 60.0),
            DVec2::new(0.0, 60.0),
        ]),
        length_offset: 60.0,
        cutter: flat_end_mill(6.0, 20.0),
    }
}

/// 毛坯尺寸（mm）。30×30×12，h=0.1 → 300×300×120 体素。
pub const STOCK_MM: [f64; 3] = [30.0, 30.0, 12.0];

/// 面铣切掉的一层厚度（mm）：顶面 12 → 11。
pub const FACING_DEPTH_MM: f64 = 1.0;

/// 面铣的步距（mm）= 刀半径，相邻两刀重叠一半，保证不留残料。
pub const FACING_STEP_MM: f64 = 3.0;

/// 面铣的刀数。`y` 从 1.5 走到 28.5，覆盖 −1.5..31.5，把 0..30 全部盖住。
pub const FACING_PASSES: usize = 10;

/// **面铣一层**：从 Z 方向均匀切掉 [`FACING_DEPTH_MM`]，
/// 每一刀是一条**直来直往**的直线，刀与刀之间抬刀、横移、再下刀。
///
/// 几个刻意的选择：
///
/// 1. **每刀只有一个 `G1`**。刀间用 `G0` 抬到 z=15、横移、再 `G0` 降回 z=11；
///    下刀点 `x = −10` / `x = 40` 在毛坯外，所以那几段快移**完全不碰料**
///    （不进切削统计、也不占颜色索引）。于是"第 n 刀"和"第 n 种颜色"一一对应，
///    着色梯度是干净的 `1,2,3,…`。
/// 2. **刀轨伸出毛坯两侧**（x 从 −10 到 40、y 从 1.5 到 28.5，刀半径 3），
///    保证顶面被完整切平，边上不留圆角残留。
/// 3. 下刀用 `G0` 而不是 `G1`：这一段在空气里（x 在料外），
///    真实机床的快速下刀也只允许在这种位置。**不要**把这套写法当成通用模板。
pub fn facing_gcode() -> String {
    let z_safe = STOCK_MM[2] + 3.0; // 15
    let z_cut = STOCK_MM[2] - FACING_DEPTH_MM; // 11
    let x_lo = -10.0;
    let x_hi = 40.0;
    let mut src = format!("G0 Z{z_safe}\n");
    for i in 0..FACING_PASSES {
        // 第一刀 y=1.5（刀半径的一半），之后每刀前进一个步距
        let y = FACING_STEP_MM / 2.0 + i as f64 * FACING_STEP_MM;
        let (from, to) = if i % 2 == 0 { (x_lo, x_hi) } else { (x_hi, x_lo) };
        src.push_str(&format!(
            "G0 X{from} Y{y}\nG0 Z{z_cut}\nG1 X{to} F800\nG0 Z{z_safe}\n"
        ));
    }
    src.push_str("M30\n");
    src
}

/// 型腔 + 钻孔：用来跑**刀径补偿**（G41）与**固定循环**（G83）的端到端路径。
///
/// 面铣之外的那一大堆 G 代码能力（G41/G83/圆弧）得有个程序能跑到，
/// 所以留着它当 `--demo pocket`。设计成品是「顶面 11 的盒」，
/// 方槽（z=8）和四个孔（z=3）都会切进设计体——报告会把它们点出来。
pub fn pocket_gcode() -> String {
    let mut src = String::from(
        "G0 Z15
G0 X3 Y3
G1 Z11 F200
",
    );
    // 面铣：来回走刀，步距 3 mm（半刀径）
    let mut dir = 1;
    for i in 0..9 {
        let y = 3.0 + i as f64 * 3.0;
        let (x0, x1) = if dir > 0 { (3.0, 27.0) } else { (27.0, 3.0) };
        src.push_str(&format!(
            "G1 X{x1:.1} F800
G1 Y{y:.1}
G1 X{x0:.1}
"
        ));
        dir = -dir;
    }
    src.push_str(
        "G0 Z15
G0 X8 Y8
G1 Z8 F200
         G1 G41 D1 X22 Y8 F500
G1 X22 Y22
G1 X8 Y22
G1 X8 Y8
G40
         G0 Z15
G83 X12 Y15 Z3 R12.5 Q2 F200
X18 Y15
X12 Y19
X18 Y19
M30
",
    );
    src
}

/// 设计成品的包围盒（缺省）：顶面被铣到 `z = 11` 的方盒。面铣恰好切到它。
///
/// 返回 `(min, max)`，`app` 拿去构 `DesignModel::Box`。
pub fn design_box() -> ([f64; 3], [f64; 3]) {
    ([0.0, 0.0, 0.0], [STOCK_MM[0], STOCK_MM[1], STOCK_MM[2] - FACING_DEPTH_MM])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 面铣程序必须**每刀一个 `G1`**——这是"一刀一色"的前提。
    /// 多一个 `G1`（比如把下刀写成进给）就会让颜色跳号。
    #[test]
    fn facing_has_exactly_one_cutting_block_per_pass() {
        let src = facing_gcode();
        let cutting = src
            .lines()
            .filter(|l| {
                let l = l.trim_start();
                l.starts_with("G1 ") || l.starts_with("G01 ")
            })
            .count();
        assert_eq!(cutting, FACING_PASSES, "切削块数应等于刀数：\n{src}");
        // 也不能有别的运动把刀带进料里：所有 G0 的 X 必须在毛坯外
        for l in src.lines() {
            let l = l.trim();
            if let Some(rest) = l.strip_prefix("G0 X")
                && let Some(x) = rest.split_whitespace().next().and_then(|t| t.parse::<f64>().ok())
            {
                assert!(
                    x < 0.0 || x > STOCK_MM[0],
                    "快移下刀点在毛坯内（x={x}），会在不切削的块里碰料：{l}"
                );
            }
        }
    }

    /// 十刀的 y 覆盖必须盖满 0..30（相邻刀重叠，不留残料带）。
    #[test]
    fn facing_passes_cover_the_whole_width() {
        let r = 3.0;
        let ys: Vec<f64> = (0..FACING_PASSES)
            .map(|i| FACING_STEP_MM / 2.0 + i as f64 * FACING_STEP_MM)
            .collect();
        assert!(ys[0] - r <= 0.0, "第一刀没盖住 y=0：{}", ys[0] - r);
        assert!(
            ys[ys.len() - 1] + r >= STOCK_MM[1],
            "最后一刀没盖住 y=30：{}",
            ys[ys.len() - 1] + r
        );
        for w in ys.windows(2) {
            assert!(w[1] - w[0] <= 2.0 * r, "步距 {} 超过刀径，中间会留残料带", w[1] - w[0]);
        }
    }
}
