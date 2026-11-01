# HMI 与控制语义

> 日期：2026-10-28
> 新增 crate：`cncsim-hmi`（依赖 `sim`）
> 关联：架构设计 §1、主方案 §3.5

---

## 0. 结论

**HMI 不是 UI 的一部分，是一个独立的状态机。**

三句话：

1. **仿真器也需要急停和倍率**——不是为了控制机床，而是为了**演练**（"如果在这儿按急停会怎样"）和**规划**（"降到 80% 节拍变多少"）。
2. **急停在仿真里是可撤销的事件，不是不可逆的破坏**——因为我们可以从头确定性重算。这是仿真器与控制器的根本差异。
3. **统一报错的核心是"稳定编号 + 位置三元组 + 怎么办"**，不是把错误消息集中打印。

工作量 **6 人日**。

---

## 1. 为什么仿真器需要 HMI

| 功能 | 在控制器里 | 在仿真器里 |
|---|---|---|
| 暂停 | 停止插补，保持位置 | 停止回放；**计算可以继续**（两个正交维度） |
| **急停** | 切伺服电源，不可逆 | **演练**：停下会留下什么痕迹？程序作废后能不能恢复？ |
| **倍率** | 实时调整进给 | **规划**：120% 跑节拍多少？降到 60% 表面质量会好吗？ |
| 报错 | 报警号 + 手册 | 同上，但要**能定位到 G 代码行** |

三条里，"暂停"我们已有（`sim` 的状态机），真正缺的是**急停、倍率、统一报错**。

---

## 2. 控制状态机

```rust
pub enum ControlState {
    Idle,                                    // 无程序
    Ready { program_len: usize },            // 已解析，可开始
    Running { since: f64 },
    Paused { at: f64, reason: PauseReason },
    /// 急停：必须复位才能离开
    EStopped { at: f64, block: u32 },
    /// 致命错误：必须复位
    Faulted { alarm: AlarmId },
}

pub enum PauseReason {
    Operator,        // 用户按暂停
    ProgramM0,       // 程序里的 M0/M1（可选：仿真默认忽略）
    BufferStarved,   // 计算跟不上回放（先算后播的背压）
}
```

### 2.1 转移表

| 从 | 事件 | 到 | 副作用 |
|---|---|---|---|
| Idle | 加载程序 | Ready | 解析、建 golden |
| Ready | 播放 | Running | 启动后台计算 |
| Running | 暂停 | Paused | 停止回放，**计算继续** |
| Paused | 播放 | Running | 恢复 |
| Running / Paused | **急停** | **EStopped** | 停止计算、记录痕迹、程序作废 |
| Running / Paused | 致命错误 | Faulted | 停止计算 |
| EStopped / Faulted | **复位** | Idle | 清空仿真、清空报警 |
| 任意 | 重算（改 h/改刀路） | Idle | 从头重算 |

**`Paused` 时计算继续**——这是"先算后播"架构的自然结果：暂停只是停止播放，后台该算还算。用户暂停时应该看到"计算进度仍在涨"。

**`EStopped` 与 `Faulted` 必须复位**——这是安全语义。不能从急停直接"继续"，必须走一遍复位流程（即使仿真里复位只是清状态）。

---

## 3. 急停的语义

### 3.1 仿真器 vs 控制器

| | 控制器 | 仿真器 |
|---|---|---|
| 物理后果 | 伺服断电、轴惰行、可能撞 | 无 |
| 程序状态 | 作废，需重新对刀 | 作废，但可重算 |
| 毛坯 | 留下刀痕 | **也应留下刀痕**（否则失去演练价值） |
| 可撤销 | **否** | **是**——从头确定性重算即可 |

### 3.2 我们应该做的三件事

1. **立即停止材料去除**——`voxel` 不再推进，毛坯停在急停那一刻
2. **记录急停位置**——`{ at: 12.34, block: 1240, tip: DVec3 }`，用于报告与画面标记
3. **程序作废**——`Trust::Unreliable`，报告里记一条 `HMI-001` Fatal

### 3.3 为什么"可撤销"很重要

控制器里急停是不可逆的物理事件。仿真器里如果也做成不可逆，用户就没法**反复试验**："我在第 30 秒急停会撞吗？第 45 秒呢？"

所以：**急停是一个带时间戳的事件，不是终态破坏**。用户可以：

- 拖回急停之前 → 事件自动清除（因为重算）
- 改急停时刻 → 重新演练
- 对比两次的毛坯差异

这条设计把"急停"从"一个按钮"变成了"一个可编程的仿真输入"。

```rust
pub struct EStopEvent {
    pub at: f64,          // 仿真时间
    pub block: u32,
    pub tip: DVec3,       // 急停时刀尖位置（画面上标个红叉）
}
```

---

## 4. 倍率

### 4.1 三种倍率

| 倍率 | 范围 | 影响 | 不影响 |
|---|---|---|---|
| **进给倍率** | 0–200% | 切削段时长 | 几何（v1 理想路径前提） |
| **快移倍率** | 25 / 50 / 100% | G0 段时长 | 几何 |
| **主轴倍率** | 50–200% | 转速（进报告） | 几何 |

**关键前提：v1 走"理想路径"，所以倍率只改时间不改几何。** 如果将来实现 G64 混合（ADR 见 G代码解释器设计 §12.4），倍率就会改变实际路径——那时这条要重写。

### 4.2 倍率改变的是"命令值"，实际还要过轴速限

```
实际进给 = min( 程序进给 × 倍率,  轴速限推出的最大进给 )
```

后半截就是主方案 §4.7 的轴速校核。所以：

- 倍率 120% 未必真的快 20%——可能被轴速卡住
- 报告要同时给"程序倍率"和"实际达成率"

这正是用户想知道的："我把倍率拉到 150%，为什么实际只快了 8%？"

### 4.3 倍率时间线（这是真正的价值）

倍率不该只是一个滑块，应该是一条**时间线**：

```rust
pub struct OverrideSchedule {
    /// (起始仿真时间, 进给倍率)
    pub feed: Vec<(f64, f64)>,
    pub rapid: Vec<(f64, f64)>,
    pub spindle: Vec<(f64, f64)>,
}
```

用户可以说"从第 30 秒起降到 60%"，然后看节拍从 12:34 变成 18:20。**这是节拍规划的核心工具**——比实时拖滑块有用得多。

默认实现：单点（全局恒定倍率）。进阶：分段。

---

## 5. 统一报错

### 5.1 结构

```rust
pub struct Alarm {
    pub id: AlarmId,          // 稳定编号，如 "GCODE-012"
    pub severity: Severity,
    pub source: Source,
    pub at: TimeRef,          // 位置三元组
    pub message: String,
    pub hint: Option<String>, // 怎么办
    pub count: u32,           // 聚合计数
}

pub enum Severity {
    Info,      // 提示，结果可信
    Warning,   // 能继续，结果可信
    Error,     // 能继续，但**结果不可信**
    Fatal,     // 无法继续
}

pub struct TimeRef {
    pub sim_time: f64,   // 仿真时间
    pub block: u32,      // 块号（跳转用）
    pub line: u32,       // 源文件行号（改代码用）
}
```

### 5.2 三条硬要求

**（1）编号是稳定契约，消息可以改**

```rust
pub const GCODE_UNKNOWN_WORD: AlarmId = "GCODE-012";
pub const KIN_SINGULARITY:    AlarmId = "KIN-001";
pub const COLL_HOLDER_STOCK:  AlarmId = "COLL-003";
pub const HMI_ESTOP:          AlarmId = "HMI-001";
```

**测试断言编号，不断言消息文本。** 否则改一句措辞就红一片测试，久了就没人维护断言。

编号也让用户能搜：文档里"KIN-001 是什么"能查到，而"检测到奇异点"搜出来一堆。

**（2）位置必须是三元组，缺一不可**

| 字段 | 缺了会怎样 |
|---|---|
| `sim_time` | 无法在时间轴上定位 |
| `block` | 无法跳转、无法做区间统计 |
| `line` | 无法回编辑器改代码 |

只给行号不够——G 代码里一行可能对应多个插补周期；只给时间不够——用户要改的是代码。

**（3）每条必须有 `hint`**

不合格：`未知 G 代码 G123`
合格：`未知 G 代码 G123（第 88 行）——不在 Fanuc 子集内，已跳过该块；若这是自定义循环，请检查后处理配置或改用宏程序`

**报错的价值在于"下一步做什么"，不在于"哪里错了"。** 只报现象不报办法的错误，用户看了还是不知道怎么办。

### 5.3 严重度的判定标准

| 级别 | 判据 | 例子 |
|---|---|---|
| Info | 结果完全可信 | "检测到 G64，未模拟混合（按理想路径计算）" |
| Warning | 结果可信，但有信息丢失 | "未知 M 代码 M123 已忽略" |
| **Error** | **结果不可信** | "G91 下跳过运动块 → 后续位置偏移" |
| Fatal | 无法继续 | "h=0.01 需要 13 GB 内存" / "急停" |

**`Error` 的定义要严格**：它意味着"跑完了，但别信这个结果"。这一级的存在是为了避免把"能跑但结果错"伪装成"成功"。

### 5.4 聚合与去重

同一个 alarm 在 1000 个块触发 → 聚合成一条：

```rust
Alarm {
    id: GCODE_UNKNOWN_WORD,
    count: 1000,
    at: TimeRef { sim_time: 0.0, block: 12, line: 88 },   // 首次位置
    // message 里注明 "另有 999 次，首次在第 88 行"
}
```

否则 10 万块的程序会刷出几千条一样的警告，用户直接关掉面板。

### 5.5 报警影响总体结论

```rust
pub fn overall_trust(alarms: &[Alarm]) -> Trust {
    if alarms.iter().any(|a| a.severity == Severity::Fatal) {
        Trust::Aborted
    } else if alarms.iter().any(|a| a.severity == Severity::Error) {
        Trust::Unreliable
    } else {
        Trust::Reliable
    }
}
```

UI 顶部横幅直接显示 `Trust`，不用用户去翻报警列表。

---

## 6. 报警从哪来

保持架构设计 §2.3 的原则：**各层自带 finding，不建底层共享枚举**。

```
gcode::ParseIssue ─┐
kin::KinFinding   ─┼→ sim 转换成 Alarm → hmi 维护队列 → ui 显示
collide::Episode  ─┤
voxel::VoxelError ─┘
```

`sim` 负责**转换**（finding → Alarm，补编号、severity、位置），`hmi` 负责**管理**（去重、计数、确认、按 severity 过滤）。

这样底层仍然互不相识，转换点集中在一处。

---

## 7. 架构位置

```
cncsim-ui ──▶ cncsim-hmi ──▶ cncsim-sim ──▶ kin / gcode / voxel / collide
                  │
                  └─ 控制状态机、倍率、报警队列
```

**新增第 12 个 crate `cncsim-hmi`**，依赖 `sim` 与 `math`。

三条理由：

1. **它有自己的状态机**，而且 E-stop 需要复位才能离开——这是安全语义，不该混进仿真调度。
2. **`ui` 和 headless CLI 共用它**——批处理模式也要能"设倍率、跑完、出报告"，不能只有 GUI 才有 HMI。
3. **报警队列的生命周期独立于单次仿真**——报警会跨多次重算累积、被确认、被过滤。

**`ui` 只依赖 `hmi`，不直接依赖 `sim`**——HMI 是 UI 的唯一后端。这样 UI 换实现（egui → 内置 Feathers）不影响仿真层。

---

## 8. 测试

```rust
#[test]
fn estop_requires_reset() {
    let mut h = Hmi::new();
    h.load(program); h.play();
    h.estop(12.34, 1240);
    assert!(matches!(h.state(), ControlState::EStopped { .. }));
    // 不能直接 play
    assert!(h.play().is_err(), "急停后必须复位才能继续");
    h.reset();
    assert!(matches!(h.state(), ControlState::Idle));
}

#[test]
fn pause_does_not_stop_computation() {
    // 暂停后 sim 的计算进度仍应增长
    let mut h = Hmi::new(); h.load(program); h.play();
    h.pause();
    let before = h.sim_progress();
    tick(&mut h, 100);
    assert!(h.sim_progress() > before, "暂停不该停止后台计算");
}

#[test]
fn override_clamped_by_axis_limit() {
    // 倍率 200% 但轴速限只允许 110% → 实际达成率应报 110%
    let r = h.set_feed_override(2.0);
    assert!((r.achieved_ratio - 1.10).abs() < 0.01);
}

#[test]
fn alarms_are_deduplicated() {
    // 同一个未知字在 1000 个块出现 → 1 条，count = 1000
    assert_eq!(h.alarms().len(), 1);
    assert_eq!(h.alarms()[0].count, 1000);
}

#[test]
fn every_alarm_has_hint() {
    // 质量门：不允许无 hint 的报警
    for a in all_alarms() { assert!(a.hint.is_some(), "{:?} 缺 hint", a.id); }
}
```

最后一条是**质量门测试**——它把"报错要告诉用户怎么办"从口头要求变成 CI 检查。

---

## 9. 工作量

| 模块 | 人日 |
|---|---|
| 控制状态机 + 转移 + 复位语义 | 2.0 |
| 急停事件与痕迹记录 | 0.5 |
| 倍率（三种 + 时间线 + 轴速限交互） | 1.5 |
| 报警系统（编号、聚合、severity、Trust 汇总） | 1.5 |
| 测试（含 hint 质量门） | 0.5 |
| **合计** | **6.0** |

---

## 10. 一句话

**HMI 让仿真器从"播放器"变成"可演练的控制台"。** 急停可以反复试、倍率可以排时间线、报警带编号和处置建议——这三件事合起来，才是"验证刀路"而不是"看动画"。
