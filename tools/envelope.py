"""
加工包络计算器。

用法：
    python tools/envelope.py                                  # 默认读 assets/machines/ac_trunnion.machine.ron
    python tools/envelope.py --machine path.ron --radius 200
    python tools/envelope.py --csv out.csv                    # 导出包络边界点云

原理：**只用正解，不需要逆解**——对固定的旋转角 (A, C)，直线轴 X/Y/Z 在限位内的
可达集合，在工件坐标系里是一个"旋转过的长方体"（仿射映射）。于是每个 (A, C)
对应一个平行六面体，包络就是它们的并集。

判据：工件（半径 R、高 h 的圆柱）可加工 ⟺ 其表面每个点都落在某个平行六面体内。

见 docs/机床建模-AC双转台.md §12。
"""
import argparse, re, pathlib, sys
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_MACHINE = ROOT / "assets" / "machines" / "ac_trunnion.machine.ron"

AXES = {'X': np.array([1.0, 0, 0]), 'Y': np.array([0, 1.0, 0]), 'Z': np.array([0, 0, 1.0])}


# ────────────────────────── RON 解析 ──────────────────────────
JOINT_RE = re.compile(
    r'name:\s*"([^"]+)"\s*,\s*'
    r'kind:\s*(Linear|Rotary)\((\w)\)\s*,\s*'
    r'offset:\s*\(([^)]*)\)\s*,\s*'
    r'pivot:\s*\(([^)]*)\)\s*,\s*'
    r'sign:\s*(-?[\d.]+)\s*,\s*'
    r'limit:\s*\(([^)]*)\)',
    re.S,
)


def parse_machine(path: pathlib.Path):
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"//[^\n]*", "", text)          # 去注释
    joints = []
    for m in JOINT_RE.finditer(text):
        name, kind, axis, off, piv, sign, lim = m.groups()
        joints.append({
            "name": name, "kind": kind, "axis": axis,
            "offset": np.array([float(x) for x in off.split(",")]),
            "pivot": np.array([float(x) for x in piv.split(",")]),
            "sign": float(sign),
            "limit": tuple(float(x) for x in lim.split(",")),
        })
    if not joints:
        sys.exit(f"未能从 {path} 解析出任何关节")
    return joints


# ────────────────────────── 正解 ──────────────────────────
def rot(axis: str, deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    if axis == 'X':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'Y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def forward(joints, q: dict):
    """按主方案 §4.2 的 FK：t = t · Translate(offset) · Rotate/Translate(q)
    返回 (R, t)，tip_w = −Rᵀ·t"""
    R, t = np.eye(3), np.zeros(3)
    for j in joints:
        val = q.get(j["name"], 0.0)
        t = t + R @ j["offset"]
        if j["kind"] == "Rotary":
            t = t + R @ j["pivot"]
            R = R @ rot(j["axis"], j["sign"] * val)
            t = t - R @ j["pivot"]
        else:
            t = t + R @ (AXES[j["axis"]] * j["sign"] * val)
    return R, t


def tip_in_workpiece(joints, q) -> np.ndarray:
    """刀尖在工件坐标系中的位置。

    注意：链乘积的**平移部分**就是 tip_w（不是 T⁻¹·0）。
    因为 RON 里旋转轴 sign=−1，链乘积实际是「刀具 → 工件」的变换。
    与 tools/verify_trunnion.py 的 `tip = t[:3,3]` 一致，已被锚点测试确认。
    """
    _, t = forward(joints, q)
    return t


# ────────────────────────── 包络 ──────────────────────────
def parallelepipeds(joints, a_grid, c_grid):
    """对每个 (A,C)，把 X/Y/Z 的可达集合表示成仿射映射 p = M·(X,Y,Z) + b"""
    lin = [j for j in joints if j["kind"] == "Linear"]
    rot_names = [j["name"] for j in joints if j["kind"] == "Rotary"]
    out = []
    for A in a_grid:
        for C in c_grid:
            base = {n: 0.0 for n in rot_names}
            if len(rot_names) >= 1: base[rot_names[0]] = C      # 链里第一个旋转轴是 C
            if len(rot_names) >= 2: base[rot_names[1]] = A
            b = tip_in_workpiece(joints, base)                  # 线性轴全 0 时的 tip_w
            cols = []
            for j in lin:
                q = dict(base); q[j["name"]] = 1.0
                cols.append(tip_in_workpiece(joints, q) - b)
            M = np.column_stack(cols) if cols else np.zeros((3, 3))
            lo = np.array([j["limit"][0] for j in lin])
            hi = np.array([j["limit"][1] for j in lin])
            out.append({"A": A, "C": C, "M": M, "b": b, "lo": lo, "hi": hi})
    return out


def covered(p: np.ndarray, boxes, tol=1e-9) -> bool:
    for bx in boxes:
        try:
            local = np.linalg.solve(bx["M"], p - bx["b"])
        except np.linalg.LinAlgError:
            continue
        if np.all(local >= bx["lo"] - tol) and np.all(local <= bx["hi"] + tol):
            return True
    return False


def max_height(joints, radius: float, a_deg: float, boxes_for_a, h_step=1.0, h_cap=800.0,
               n_theta=24, n_z=6):
    """给定摆角，求半径为 radius 的圆柱能加工的最大高度"""
    def ok(h):
        if h <= 0:
            return True
        for th in np.linspace(0, 2 * np.pi, n_theta, endpoint=False):
            for z in np.linspace(0, h, n_z):
                p = np.array([radius * np.cos(th), radius * np.sin(th), z])
                if not covered(p, boxes_for_a):
                    return False
        return True

    lo, hi = 0.0, h_cap
    if ok(hi):
        return hi
    while hi - lo > h_step:
        mid = (lo + hi) / 2
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--machine", type=pathlib.Path, default=DEFAULT_MACHINE)
    ap.add_argument("--radius", type=float, default=200.0, help="工件半径 mm（默认转台半径）")
    ap.add_argument("--angles", type=str, default="0,15,30,45,60,75,90,105,110")
    ap.add_argument("--c-steps", type=int, default=72, help="C 轴采样数（越多包络越准；默认 72 已收敛）")
    ap.add_argument("--csv", type=pathlib.Path, default=None)
    args = ap.parse_args()

    joints = parse_machine(args.machine)
    print(f"机床：{args.machine.name}")
    print("关节链（工件 → 刀具）：")
    for j in joints:
        print(f"  {j['name']:>2} {j['kind']:<6}({j['axis']})  "
              f"offset={np.round(j['offset'], 1)}  sign={j['sign']:+.0f}  "
              f"limit=({j['limit'][0]:g}, {j['limit'][1]:g})")

    angles = [float(x) for x in args.angles.split(",")]
    c_grid = np.linspace(0, 360, args.c_steps, endpoint=False)
    all_boxes = parallelepipeds(joints, angles, c_grid)

    print(f"\n工件半径 R = {args.radius:g} mm，C 轴采样 {args.c_steps} 档")
    print(f"{'摆角 A':>8} {'最大高度':>10} {'受限于':>8}")
    rows = []
    for A in angles:
        boxes_a = [b for b in all_boxes if b["A"] == A]
        h = max_height(joints, args.radius, A, boxes_a)
        # 判定限制来源：放开 Y 或 Z 看哪个能提升
        lim = "—"
        for j in joints:
            if j["kind"] != "Linear":
                continue
            saved = j["limit"]
            j["limit"] = (saved[0] - 500, saved[1] + 500)
            boxes2 = parallelepipeds(joints, [A], c_grid)
            h2 = max_height(joints, args.radius, A, boxes2)
            j["limit"] = saved
            if h2 > h + 1.0:
                lim = j["name"]
                break
        print(f"{A:>7.0f}° {h:>9.0f}  {lim:>8}")
        rows.append((A, h))

    if args.csv:
        with args.csv.open("w", encoding="utf-8", newline="") as f:
            f.write("A_deg,h_max_mm\n")
            for A, h in rows:
                f.write(f"{A:g},{h:.1f}\n")
        print(f"\n已写出 {args.csv}")


if __name__ == "__main__":
    main()
