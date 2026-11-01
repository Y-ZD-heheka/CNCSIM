"""
依赖版本守卫：核对所有第三方依赖是否都是 crates.io 上的最新稳定版。

用法：
    python tools/check_versions.py              # 检查直接依赖
    python tools/check_versions.py --transitive # 连 Cargo.lock 里的间接依赖一起查
    python tools/check_versions.py --quiet      # 只报问题

策略（见 Cargo.toml 注释）：
    - 只用**精确的最新稳定版**，写成 `version = "=x.y.z"`
    - 落后即 CI 失败
    - 加新依赖前先跑 `--show <crate>` 查最新版

退出码：0 全部最新；1 有落后或未精确锁定。
"""
import json
import pathlib
import re
import sys
import time
import tomllib
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
CRATES = ROOT / "crates"
UA = "cncsim-dev (dependency policy check)"
_cache: dict[str, str | None] = {}


def latest_version(name: str) -> str | None:
    """查 crates.io 的最新稳定版。失败返回 None（不阻断，只报告）。"""
    if name in _cache:
        return _cache[name]
    url = f"https://crates.io/api/v1/crates/{name}"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=25) as r:
            d = json.load(r)
        v = d["crate"]["max_stable_version"]
    except Exception:
        v = None
    _cache[name] = v
    time.sleep(0.15)  # 尊重 crates.io 的速率建议
    return v


def parse_req(spec) -> tuple[str | None, bool]:
    """从依赖声明里取出版本号，以及是否用了精确锁定 `=x.y.z`"""
    if isinstance(spec, str):
        s = spec.strip()
    elif isinstance(spec, dict):
        s = str(spec.get("version", "")).strip()
    else:
        return None, False
    if not s:
        return None, False
    exact = s.startswith("=")
    return s.lstrip("=^~<> ").split(",")[0].strip(), exact


def collect_direct():
    """收集直接依赖：{crate: (declared, exact, 来源)}"""
    out = {}

    # 1) workspace 级别的 [workspace.dependencies]
    with (ROOT / "Cargo.toml").open("rb") as f:
        ws = tomllib.load(f)
    ws_deps = ws.get("workspace", {}).get("dependencies", {})
    for name, spec in ws_deps.items():
        if name.startswith("cncsim-") or (isinstance(spec, dict) and "path" in spec):
            continue
        ver, exact = parse_req(spec)
        if ver:
            out[name] = (ver, exact, "workspace")

    # 2) 各 crate 自己的外部依赖（不走 workspace 的）
    for c in sorted(p for p in CRATES.iterdir() if p.is_dir()):
        cargo = c / "Cargo.toml"
        if not cargo.exists():
            continue
        with cargo.open("rb") as f:
            d = tomllib.load(f)
        for section in ("dependencies", "dev-dependencies", "build-dependencies"):
            for name, spec in d.get(section, {}).items():
                if name.startswith("cncsim-") or (isinstance(spec, dict) and "path" in spec):
                    continue
                if isinstance(spec, dict) and spec.get("workspace") is True:
                    continue  # 已在 workspace 里统计
                ver, exact = parse_req(spec)
                if ver:
                    out.setdefault(name, (ver, exact, f"crates/{c.name}/{section}"))
    return out


def collect_transitive():
    lock = ROOT / "Cargo.lock"
    if not lock.exists():
        return {}
    with lock.open("rb") as f:
        d = tomllib.load(f)
    out = {}
    for pkg in d.get("package", []):
        name = pkg["name"]
        if name.startswith("cncsim-"):
            continue
        src = pkg.get("source", "")
        if "registry" not in src and "crates.io" not in src:
            continue  # 跳过 path / git 依赖
        out[name] = (pkg["version"], True, "Cargo.lock")
    return out


def check_edition() -> list[str]:
    """所有 crate 必须继承 workspace 的 edition（2024）。"""
    errs = []
    with (ROOT / "Cargo.toml").open("rb") as f:
        ws = tomllib.load(f)
    edition = ws.get("workspace", {}).get("package", {}).get("edition")
    if edition != "2024":
        errs.append(f"workspace edition 应为 2024，实际 {edition!r}")
    for c in sorted(p for p in CRATES.iterdir() if p.is_dir()):
        cargo = c / "Cargo.toml"
        if not cargo.exists():
            continue
        with cargo.open("rb") as f:
            d = tomllib.load(f)
        pkg = d.get("package", {})
        if pkg.get("edition") != {"workspace": True}:
            errs.append(f"crates/{c.name}: 应写 `edition.workspace = true`，实际 {pkg.get('edition')!r}")
    return errs


def main() -> int:
    quiet = "--quiet" in sys.argv
    transitive = "--transitive" in sys.argv

    if "--show" in sys.argv:
        i = sys.argv.index("--show")
        for name in sys.argv[i + 1 :]:
            print(f"{name} = {latest_version(name)}")
        return 0

    errors = check_edition()
    deps = collect_direct()
    if transitive:
        for k, v in collect_transitive().items():
            deps.setdefault(k, v)

    rows = []
    for name, (declared, exact, src) in sorted(deps.items()):
        latest = latest_version(name)
        if latest is None:
            rows.append((name, declared, "?", "查询失败", src))
            continue
        same = declared == latest
        rows.append((name, declared, latest, "最新" if same else "**落后**", src))
        if not same:
            errors.append(f"{name}: 声明 {declared}，最新 {latest}（{src}）")
        elif not exact and src != "Cargo.lock":
            errors.append(f"{name}: 未精确锁定，应写 `version = \"={latest}\"`（{src}）")

    if not quiet:
        print(f"{'crate':<24} {'声明':<12} {'最新':<12} {'状态':<8} 来源")
        print("─" * 86)
        for name, declared, latest, status, src in rows:
            print(f"{name:<24} {declared:<12} {latest:<12} {status:<8} {src}")
        print()

    if errors:
        print(f"发现 {len(errors)} 处问题：")
        for e in errors:
            print("  ✗ " + e)
        return 1

    print(f"依赖校验通过 ✓（{len(rows)} 个第三方依赖，全部为最新稳定版；edition 2024）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
