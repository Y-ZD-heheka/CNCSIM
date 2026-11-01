//! # cncsim-config
//!
//! 机床配置文件（`.machine.ron`）→ [`MachineTopology`]。
//!
//! ## 为什么单独一个 crate
//!
//! 磁盘上的配置是**跨层的**：`chain` 归 `kin`，`parts`（外形/碰撞代理）归
//! `render` / `collide`，`motion`（插补周期/速度上限）归 `sim`。
//! 让 `kin` 去读整个文件，它就得知道 `Shape::Box` 和 `ColliderSpec::Derived`——耦合立刻变高。
//!
//! 所以配置格式单独成一个 crate，**只有它知道"文件长什么样"**，
//! 各领域 crate 只接收自己的类型。
//!
//! ## 当前实现范围
//!
//! v1 解析完整内容：**运动学 + 运动学之外的机床参数**。
//! - `chain` / `name` → [`MachineTopology`] (kin)
//! - `motion` → [`MachineMotion`](crate::MachineMotion) (sim)
//! - `parts` 与几何模型相关，v1 仍被 serde 静默忽略
//!
//! 文档见 docs/架构设计.md §2.4、docs/G 代码到轴运动.md §1.3。
//!
//! ## 一条锁住「硬编码 vs 配置」漂移的测试
//!
//! `kin::MachineTopology::ac_trunnion()` 是硬编码的测试夹具。
//! [`tests::hardcoded_matches_file`] 断言它与 `assets/machines/ac_trunnion.machine.ron`
//! 解析结果**完全一致**——否则两份数据会悄悄分家。

use cncsim_kin::{Axis, Joint, JointKind, MachineTopology};
use cncsim_math::DVec3;
use serde::Deserialize;
use std::path::Path;

/// 加载失败的原因。
#[derive(Debug)]
pub enum ConfigError {
    Io(std::io::Error),
    /// RON 语法错误或字段不匹配
    Parse(ron::error::SpannedError),
    /// 语义错误：链里没有旋转轴、限位反向等
    Invalid(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::Io(e) => write!(f, "读取配置文件失败：{e}"),
            ConfigError::Parse(e) => write!(f, "配置格式错误：{e}"),
            ConfigError::Invalid(m) => write!(f, "配置语义错误：{m}"),
        }
    }
}

impl std::error::Error for ConfigError {}

impl From<std::io::Error> for ConfigError {
    fn from(e: std::io::Error) -> Self {
        ConfigError::Io(e)
    }
}

impl From<ron::error::SpannedError> for ConfigError {
    fn from(e: ron::error::SpannedError) -> Self {
        ConfigError::Parse(e)
    }
}

/// 解析后的机床配置。
#[derive(Clone, Debug)]
pub struct MachineConfig {
    pub name: String,
    pub topology: MachineTopology,
    /// 刀具长度缺省值（mm）
    pub tool_length: f64,
    /// 回转中心到刀尖的偏置（mm）
    pub pivot_distance: DVec3,
}

// ────────────────────── 磁盘格式（只在这里出现） ──────────────────────

#[derive(Deserialize)]
struct MachineFile {
    name: String,
    chain: Vec<JointFile>,
    #[serde(default)]
    tool_length: f64,
    #[serde(default = "zero3")]
    pivot_distance: [f64; 3],
    // parts / motion 等字段故意不声明 → serde 静默忽略
}

#[derive(Deserialize)]
struct JointFile {
    name: String,
    kind: JointKindFile,
    #[serde(default = "zero3")]
    offset: [f64; 3],
    #[serde(default = "zero3")]
    pivot: [f64; 3],
    #[serde(default = "one")]
    sign: f64,
    limit: (f64, f64),
}

#[derive(Deserialize, Clone, Copy)]
enum JointKindFile {
    Linear(AxisFile),
    Rotary(AxisFile),
}

#[derive(Deserialize, Clone, Copy)]
enum AxisFile {
    X,
    Y,
    Z,
}

fn zero3() -> [f64; 3] {
    [0.0; 3]
}
fn one() -> f64 {
    1.0
}

impl From<AxisFile> for Axis {
    fn from(a: AxisFile) -> Self {
        match a {
            AxisFile::X => Axis::X,
            AxisFile::Y => Axis::Y,
            AxisFile::Z => Axis::Z,
        }
    }
}

impl From<JointFile> for Joint {
    fn from(j: JointFile) -> Self {
        let kind = match j.kind {
            JointKindFile::Linear(a) => JointKind::Linear(a.into()),
            JointKindFile::Rotary(a) => JointKind::Rotary(a.into()),
        };
        Joint {
            name: j.name,
            kind,
            offset: DVec3::new(j.offset[0], j.offset[1], j.offset[2]),
            pivot: DVec3::new(j.pivot[0], j.pivot[1], j.pivot[2]),
            sign: j.sign,
            limit: j.limit,
        }
    }
}

// ────────────────────────── 入口 ──────────────────────────

/// 从 RON 文本解析。
pub fn parse(src: &str) -> Result<MachineConfig, ConfigError> {
    let file: MachineFile = ron::from_str(src)?;

    if file.chain.is_empty() {
        return Err(ConfigError::Invalid("chain 为空".into()));
    }
    let n_rot = file
        .chain
        .iter()
        .filter(|j| matches!(j.kind, JointKindFile::Rotary(_)))
        .count();
    let n_lin = file.chain.len() - n_rot;
    if n_rot != 2 || n_lin != 3 {
        return Err(ConfigError::Invalid(format!(
            "只支持「2 旋转 + 3 直线」的五轴构型，实际 {n_rot} 旋转 + {n_lin} 直线"
        )));
    }
    for j in &file.chain {
        if j.limit.0 > j.limit.1 {
            return Err(ConfigError::Invalid(format!(
                "关节 {:?} 的限位反向：({}, {})",
                j.name, j.limit.0, j.limit.1
            )));
        }
        if j.sign != 1.0 && j.sign != -1.0 {
            return Err(ConfigError::Invalid(format!(
                "关节 {:?} 的 sign 只能是 ±1，实际 {}",
                j.name, j.sign
            )));
        }
    }

    let chain: Vec<Joint> = file.chain.into_iter().map(Joint::from).collect();
    Ok(MachineConfig {
        name: file.name.clone(),
        topology: MachineTopology { name: file.name, chain },
        tool_length: file.tool_length,
        pivot_distance: DVec3::new(
            file.pivot_distance[0],
            file.pivot_distance[1],
            file.pivot_distance[2],
        ),
    })
}

/// 从文件加载。
pub fn load(path: impl AsRef<Path>) -> Result<MachineConfig, ConfigError> {
    parse(&std::fs::read_to_string(path)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    const AC_TRUNNION: &str = include_str!("../../../assets/machines/ac_trunnion.machine.ron");

    /// **锁住「硬编码夹具 vs 配置文件」的漂移。**
    ///
    /// `MachineTopology::ac_trunnion()` 是测试用的硬编码夹具（测试不该依赖文件 I/O），
    /// 配置文件是产品的真实输入。两者必须完全一致，否则改了一处忘了另一处，
    /// 测试会继续绿而产品是错的。
    #[test]
    fn hardcoded_matches_file() {
        let cfg = parse(AC_TRUNNION).expect("配置文件应能解析");
        let hard = MachineTopology::ac_trunnion();

        assert_eq!(cfg.topology.chain.len(), hard.chain.len(), "关节数不一致");
        for (a, b) in cfg.topology.chain.iter().zip(&hard.chain) {
            assert_eq!(a.name, b.name, "关节名");
            assert_eq!(a.kind, b.kind, "关节 {} 的类型", a.name);
            assert_eq!(a.offset, b.offset, "关节 {} 的 offset", a.name);
            assert_eq!(a.pivot, b.pivot, "关节 {} 的 pivot", a.name);
            assert_eq!(a.sign, b.sign, "关节 {} 的 sign", a.name);
            assert_eq!(a.limit, b.limit, "关节 {} 的 limit", a.name);
        }
    }

    #[test]
    fn parses_expected_shape() {
        let cfg = parse(AC_TRUNNION).unwrap();
        assert!(cfg.name.contains("摇篮"));
        assert_eq!(cfg.topology.joint_count(), 5);
        assert_eq!(cfg.topology.rotary_indices(), vec![0, 1], "C、A 在链首");
        assert_eq!(cfg.topology.linear_indices(), vec![2, 3, 4]);
        assert_eq!(cfg.topology.index_of("A"), Some(1));
    }

    #[test]
    fn unknown_fields_are_ignored() {
        // parts / motion 等未来字段不该让解析失败
        let src = r#"(
            name: "测试机",
            chain: [
                (name: "C", kind: Rotary(Z), limit: (-360, 360)),
                (name: "A", kind: Rotary(X), limit: (-90, 90)),
                (name: "X", kind: Linear(X), limit: (-100, 100)),
                (name: "Y", kind: Linear(Y), limit: (-100, 100)),
                (name: "Z", kind: Linear(Z), limit: (-100, 100)),
            ],
            future_field: (whatever: [1, 2, 3]),
        )"#;
        let cfg = parse(src).expect("未知字段应被忽略");
        assert_eq!(cfg.topology.joint_count(), 5);
    }

    #[test]
    fn defaults_apply_when_omitted() {
        let src = r#"(
            name: "最小配置",
            chain: [
                (name: "C", kind: Rotary(Z), limit: (-1, 1)),
                (name: "A", kind: Rotary(X), limit: (-1, 1)),
                (name: "X", kind: Linear(X), limit: (-1, 1)),
                (name: "Y", kind: Linear(Y), limit: (-1, 1)),
                (name: "Z", kind: Linear(Z), limit: (-1, 1)),
            ],
        )"#;
        let cfg = parse(src).unwrap();
        assert_eq!(cfg.topology.chain[0].offset, DVec3::ZERO, "offset 缺省为 0");
        assert_eq!(cfg.topology.chain[0].sign, 1.0, "sign 缺省为 +1");
        assert_eq!(cfg.tool_length, 0.0);
    }

    #[test]
    fn rejects_wrong_axis_count() {
        let src = r#"(
            name: "三轴",
            chain: [
                (name: "X", kind: Linear(X), limit: (-1, 1)),
                (name: "Y", kind: Linear(Y), limit: (-1, 1)),
                (name: "Z", kind: Linear(Z), limit: (-1, 1)),
            ],
        )"#;
        let e = parse(src).unwrap_err();
        assert!(matches!(e, ConfigError::Invalid(_)), "应报语义错误，实际 {e:?}");
        assert!(format!("{e}").contains("2 旋转 + 3 直线"));
    }

    #[test]
    fn rejects_reversed_limit() {
        let src = r#"(
            name: "限位反向",
            chain: [
                (name: "C", kind: Rotary(Z), limit: (-1, 1)),
                (name: "A", kind: Rotary(X), limit: (90, -90)),
                (name: "X", kind: Linear(X), limit: (-1, 1)),
                (name: "Y", kind: Linear(Y), limit: (-1, 1)),
                (name: "Z", kind: Linear(Z), limit: (-1, 1)),
            ],
        )"#;
        let e = parse(src).unwrap_err();
        assert!(format!("{e}").contains("限位反向"), "实际 {e}");
    }

    #[test]
    fn rejects_bad_sign() {
        let src = r#"(
            name: "sign 非法",
            chain: [
                (name: "C", kind: Rotary(Z), sign: 2.0, limit: (-1, 1)),
                (name: "A", kind: Rotary(X), limit: (-1, 1)),
                (name: "X", kind: Linear(X), limit: (-1, 1)),
                (name: "Y", kind: Linear(Y), limit: (-1, 1)),
                (name: "Z", kind: Linear(Z), limit: (-1, 1)),
            ],
        )"#;
        let e = parse(src).unwrap_err();
        assert!(format!("{e}").contains("sign"), "实际 {e}");
    }

    #[test]
    fn syntax_error_is_reported() {
        let e = parse("(name: \"x\", chain: [").unwrap_err();
        assert!(matches!(e, ConfigError::Parse(_)), "应报解析错误，实际 {e:?}");
    }

    /// 解析出的拓扑必须能直接跑运动学——不是"解析成功"就算完。
    #[test]
    fn parsed_topology_is_usable() {
        let cfg = parse(AC_TRUNNION).unwrap();
        let m = &cfg.topology;
        let p = cncsim_kin::forward(m, &m.q(&[("X", 100.0), ("Y", 50.0), ("Z", -20.0), ("A", 30.0), ("C", 45.0)]));
        // 与 golden 锚点 3 一致
        assert!(p.tip.abs_diff(DVec3::new(94.258232091579, -47.163124145731, -222.320508075689)) < 1e-9);
    }
}
