"""
验证 AC 双转台（摇篮）运动学：直接按 MachineTopology.chain 的 FK 规则算，
这样验的就是我们要发出去的那份配置，而不是另一个等价实现。
"""
import numpy as np
from numpy import sin, cos, radians as rad

H, L = 0.0, 180.0   # H 已被吸收进基准选择：Z=0 即 A 轴中心高度

def Rx(a):
    c, s = cos(a), sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Rz(a):
    c, s = cos(a), sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def T(v):
    m = np.eye(4); m[:3,3] = v; return m
def R4(R):
    m = np.eye(4); m[:3,:3] = R; return m

AX = {'X': np.array([1,0,0.]), 'Y': np.array([0,1,0.]), 'Z': np.array([0,0,1.])}

# chain: (name, kind, axis, offset, sign)   —— 与 .machine.ron 完全一致
CHAIN = [
    ('C', 'rot', 'Z', (0,0,0),   -1.0),
    ('A', 'rot', 'X', (0,0,-L),  -1.0),
    ('X', 'lin', 'X', (0,0,0),   +1.0),
    ('Y', 'lin', 'Y', (0,0,0),   +1.0),
    ('Z', 'lin', 'Z', (0,0,0),   +1.0),
]

def fk(X, Y, Z, A_deg, C_deg):
    """(X,Y,Z,A,C) -> (tip in workpiece frame, tool axis in workpiece frame)
    严格按主方案 §4.2 的 FK：t = t * Translate(offset) * Rotate/Translate(q)"""
    q = [C_deg, A_deg, X, Y, Z]
    t = np.eye(4)
    for (name, kind, axis, offset, sign), val in zip(CHAIN, q):
        t = t @ T(np.array(offset, float))
        if kind == 'rot':
            t = t @ R4((Rx if axis == 'X' else Rz)(rad(sign * val)))
        else:
            t = t @ T(AX[axis] * sign * val)
    tip = t[:3,3]
    axis_w = t[:3,:3] @ np.array([0,0,1.0])
    return tip, axis_w

def rtcp(p_w, A_deg, C_deg):
    """RTCP：给定工件系刀尖位置 + 角度，反算机床直线轴"""
    x, y, z = p_w
    A, C = rad(A_deg), rad(C_deg)
    u = x*cos(C) - y*sin(C)
    v = x*sin(C) + y*cos(C)
    return np.array([u,
                     v*cos(A) - (z + L)*sin(A),
                     H + v*sin(A) + (z + L)*cos(A)])

print("=== 锚点 1: 全零位 (0,0,0,0,0) ===")
p, n = fk(0,0,0,0,0); print("  p_W =", np.round(p,6), " n_W =", np.round(n,6))

print("=== 锚点 2: A=90, 机床(0,0,-50) ===")
p, n = fk(0,0,-50,90,0); print("  p_W =", np.round(p,6), " n_W =", np.round(n,6))

print("=== 锚点 3: A=30 C=45, 机床(100,50,-20) ===")
p, n = fk(100,50,-20,30,45)
print("  p_W =", np.round(p,6), " n_W =", np.round(n,6))
print("  |p_W| =", round(np.linalg.norm(p),4))

print("\n=== RTCP 与 FK 互逆（随机 500 组）===")
rng = np.random.default_rng(1); worst = 0.0
for _ in range(500):
    pw = rng.uniform([-150,-150,-200],[150,150,0])
    a = rng.uniform(-110,110); c = rng.uniform(-180,180)
    q = rtcp(pw, a, c)
    p2, _ = fk(q[0],q[1],q[2],a,c)
    worst = max(worst, np.linalg.norm(p2-pw))
print("  max |RTCP→FK 偏差| =", f"{worst:.3e}")

print("\n=== RTCP 不变量：A 从 -60 摆到 +60，刀尖固定 ===")
pw0 = np.array([30.0, 20.0, -5.0]); w = 0.0
for i in range(41):
    a = -60 + 3*i
    q = rtcp(pw0, a, 0.0); p2,_ = fk(q[0],q[1],q[2],a,0.0)
    w = max(w, np.linalg.norm(p2-pw0))
print("  最大漂移 =", f"{w:.3e}")

print("\n=== §6.1 摆动补偿量（刀尖在台面中心 p_W=(0,0,0)）===")
print(f"{'A(°)':>6} {'X':>8} {'Y':>9} {'Z':>9} {'ΔY':>8} {'ΔZ':>9}")
base = rtcp(np.zeros(3), 0, 0)
for a in [0,30,60,90,110]:
    q = rtcp(np.zeros(3), a, 0)
    print(f"{a:>6} {q[0]:>8.1f} {q[1]:>9.1f} {q[2]:>9.1f} {q[1]-base[1]:>8.1f} {q[2]-base[2]:>9.1f}")

print("\n=== §6.2 Y 轴补偿速度（A=0, L=180, A 轴 25 rpm）===")
omega = 25*2*np.pi/60
print(f"  |dY/dt| = L·ω = {L*omega:.1f} mm/s = {L*omega*60/1000:.1f} m/min")

print("\n=== 刀轴方向解析式校验 n_W = Rz(-C)Rx(-A)ẑ ===")
for a,c in [(0,0),(30,45),(90,0),(-40,120)]:
    _, n = fk(0,0,0,a,c)
    A,C = rad(a), rad(c)
    nm = np.array([ sin(A)*sin(C), sin(A)*cos(C), cos(A) ])
    print(f"  A={a:>4} C={c:>4}: fk={np.round(n,6)} 解析={np.round(nm,6)}  diff={np.linalg.norm(n-nm):.1e}")

# ─────────────────────────────────────────────────────────────
# golden 基准：生成 / 校验
# ─────────────────────────────────────────────────────────────
import csv, sys, pathlib

GOLDEN = pathlib.Path(r"C:/Users/15796/WorkBuddy AI/CNCSIM/testdata/golden/ac_trunnion_anchors.csv")

# 三组锚点由【手推 + 独立矩阵连乘】双重确认，是真正的 oracle；
# 网格点由本实现生成，只作回归锁（改动实现时若数值变了会立刻发现）。
ANCHORS = [
    (0.0,   0.0,   0.0,   0.0,  0.0),
    (0.0,   0.0, -50.0,  90.0,  0.0),
    (100.0, 50.0, -20.0, 30.0, 45.0),
]
GRID = [(x, y, z, a, c)
        for x in (-200.0, 0.0, 200.0)
        for y in (-150.0, 0.0, 150.0)
        for z in (-250.0, -100.0, 50.0)
        for a in (-90.0, -45.0, 0.0, 45.0, 90.0)
        for c in (0.0, 90.0, 180.0, 270.0)]

def build_rows():
    rows = []
    for i, (x, y, z, a, c) in enumerate(ANCHORS + GRID):
        p, n = fk(x, y, z, a, c)
        rows.append([f"{i:04d}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{a:.3f}", f"{c:.3f}",
                     f"{p[0]:.12f}", f"{p[1]:.12f}", f"{p[2]:.12f}",
                     f"{n[0]:.12f}", f"{n[1]:.12f}", f"{n[2]:.12f}"])
    return rows

HEADER = ["id", "X", "Y", "Z", "A", "C", "px", "py", "pz", "nx", "ny", "nz"]

if "--write" in sys.argv or not GOLDEN.exists():
    GOLDEN.parent.mkdir(parents=True, exist_ok=True)
    with GOLDEN.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(HEADER); w.writerows(build_rows())
    print(f"\n[golden] 已写入 {GOLDEN.name}（{len(ANCHORS)+len(GRID)} 行）")
else:
    with GOLDEN.open(encoding="utf-8") as f:
        old = list(csv.reader(f))[1:]
    new = build_rows()
    bad = 0
    if len(old) != len(new):
        print(f"\n[golden] 行数不一致：文件 {len(old)}，当前 {len(new)}"); bad += 1
    for o, n_ in zip(old, new):
        for j in range(6, 12):
            if abs(float(o[j]) - float(n_[j])) > 1e-9:
                print(f"[golden] 行 {o[0]} 第 {HEADER[j]} 列漂移：{o[j]} → {n_[j]}"); bad += 1
    print(f"\n[golden] 校验 {'通过' if bad == 0 else f'失败（{bad} 处）'}，共 {len(new)} 行")
    sys.exit(1 if bad else 0)
