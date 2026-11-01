"""
架构守卫：校验 crate 依赖是否违反白名单。

用法：
    python tools/check_deps.py            # 校验，违规退出码 1
    python tools/check_deps.py --verbose  # 打印每条边

架构约束写在文档里会腐化，只有写进 CI 才有效。
见 docs/架构设计.md §2。
"""
import sys, pathlib, tomllib

ROOT = pathlib.Path(__file__).resolve().parent.parent
CRATES = ROOT / "crates"

# ── 允许的依赖边（白名单）。不在表里的 cncsim-* 依赖一律违规 ──
ALLOWED = {
    "cncsim-math":    set(),
    "cncsim-motion":  {"cncsim-math"},
    "cncsim-tool":    {"cncsim-math"},
    "cncsim-kin":     {"cncsim-math", "cncsim-motion"},
    # 配置是**输入适配器**：只把磁盘格式翻译成各层自己的类型，
    # 不允许反过来（kin 不认识 .ron）。依赖面刻意收窄到 math + kin。
    "cncsim-config":  {"cncsim-math", "cncsim-kin"},
    "cncsim-gcode":   {"cncsim-math", "cncsim-motion", "cncsim-tool"},
    "cncsim-voxel":   {"cncsim-math", "cncsim-motion", "cncsim-tool"},
    "cncsim-collide": {"cncsim-math", "cncsim-motion", "cncsim-tool"},
    "cncsim-sim":     {"cncsim-math", "cncsim-motion", "cncsim-tool",
                       "cncsim-kin", "cncsim-gcode", "cncsim-voxel", "cncsim-collide"},
    "cncsim-hmi":     {"cncsim-math", "cncsim-sim"},
    # render 依赖 hmi+ui：外壳里的 egui 面板**只做适配**（Tone → Color32、
    # AlarmRow → label），判据全在 cncsim-ui 里。反过来（ui → render）会让
    # 面板逻辑分裂成窗口版和预览版两套。
    "cncsim-render":  {"cncsim-math", "cncsim-motion", "cncsim-tool", "cncsim-voxel",
                       "cncsim-sim", "cncsim-hmi", "cncsim-ui"},
    "cncsim-ui":      {"cncsim-math", "cncsim-hmi"},
    "cncsim-app":     {"cncsim-math", "cncsim-motion", "cncsim-tool", "cncsim-kin",
                       "cncsim-config", "cncsim-gcode", "cncsim-voxel", "cncsim-collide",
                       "cncsim-sim", "cncsim-hmi", "cncsim-render", "cncsim-ui"},
}

# ── 允许的外部依赖（按 crate）──
#
# 策略：**L2 领域 crate 原则上零外部依赖**，例外必须在这里登记，并在 ADR 写明理由。
# 登记 = 有人签字，不是"随便加"。想加先问：自研 150 行能不能顶掉？
EXTERNAL = {
    # 输入适配器：序列化只在输入边界出现
    "cncsim-config":  {"serde", "ron"},
    # 并行：常驻线程池（每次 fork-join 491 µs → ~5 µs）。见 ADR-009 §1.1
    "cncsim-voxel":   {"rayon"},
    # 窗口层 UI：egui 是 Bevy 生态里唯一成熟的即时模式 UI。只在 `window`
    # feature 下编译，默认构建零依赖。见 ADR-011（显示层）。
    "cncsim-render":  {"bevy-egui"},
}

# 只有这三层允许出现 bevy
BEVY_ALLOWED = {"cncsim-render", "cncsim-ui", "cncsim-app"}

# 禁止的名字：出现即视为"公共类型垃圾场"回潮
FORBIDDEN_NAMES = {"cncsim-core", "cncsim-common", "cncsim-utils", "cncsim-types", "cncsim-shared"}


def deps_of(cargo_toml: pathlib.Path):
    """返回 (普通依赖集合, dev 依赖集合)"""
    with cargo_toml.open("rb") as f:
        data = tomllib.load(f)
    norm = lambda t: {k.replace("_", "-") for k in t}
    return norm(data.get("dependencies", {})), norm(data.get("dev-dependencies", {}))


def main() -> int:
    verbose = "--verbose" in sys.argv
    if not CRATES.is_dir():
        print(f"[skip] 尚未搭建 workspace（{CRATES} 不存在）")
        return 0

    errors, checked = [], 0
    for crate_dir in sorted(p for p in CRATES.iterdir() if p.is_dir()):
        cargo = crate_dir / "Cargo.toml"
        if not cargo.exists():
            continue
        name = "cncsim-" + crate_dir.name
        if name in FORBIDDEN_NAMES:
            errors.append(f"{name}: 禁止的 crate 名（公共类型垃圾场）——按概念簇拆分")
            continue
        if name not in ALLOWED:
            errors.append(f"{name}: 未在白名单中。新增 crate 请先在 docs/架构设计.md §2.2 登记")
            continue

        normal, dev = deps_of(cargo)
        checked += 1

        for d in sorted(normal):
            if d.startswith("cncsim-"):
                if d not in ALLOWED[name]:
                    errors.append(f"{name} → {d}: 违规依赖（不在白名单内）")
                elif verbose:
                    print(f"  ok  {name} → {d}")
            elif d != "bevy":
                # 外部依赖：L2 领域 crate 原则上零依赖
                if d not in EXTERNAL.get(name, set()):
                    errors.append(
                        f"{name} → {d}: 未登记的外部依赖"
                        f"（先问：自研能不能顶掉？要引就在 EXTERNAL 里登记 + 补 ADR）"
                    )
                elif verbose:
                    print(f"  ok  {name} → {d}（已登记）")
        if "bevy" in normal and name not in BEVY_ALLOWED:
            errors.append(f"{name} → bevy: L0–L3 必须零 Bevy 依赖")
        for d in sorted(dev):
            if d in FORBIDDEN_NAMES:
                errors.append(f"{name} (dev) → {d}: 禁止的 crate 名")

    print(f"\n检查了 {checked} 个 crate")
    if errors:
        print(f"\n发现 {len(errors)} 处架构违规：")
        for e in errors:
            print("  ✗ " + e)
        return 1
    print("架构校验通过 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
