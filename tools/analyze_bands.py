"""统计导出预览里 creator 的空间分布，区分“色带边界”和“噪点”。

用法：python tools/analyze_bands.py out/dome_removed_only.html
"""

import base64
import re
import sys
from collections import Counter, defaultdict

import numpy as np


def load(path):
    html = open(path, encoding="utf-8").read()
    vals = {}
    for key in ("POS", "NRM", "IDX", "COL"):
        m = re.search(key + r'_B64\s*=\s*"([^"]+)"', html)
        if not m:
            raise SystemExit(f"找不到 {key}_B64")
        vals[key] = base64.b64decode(m.group(1))
    pos = np.frombuffer(vals["POS"], dtype="<f4").reshape(-1, 3)
    col = np.frombuffer(vals["COL"], dtype=np.uint8).reshape(-1, 4)
    idx = np.frombuffer(vals["IDX"], dtype="<u4").reshape(-1, 3)
    return pos, col, idx


def main(path):
    pos, col, idx = load(path)
    creator = col[:, 0].astype(np.int32)
    print(f"顶点 {len(pos)}，三角形 {len(idx)}")
    cnt = Counter(creator.tolist())
    print("creator 计数（前 25）：", cnt.most_common(25))

    # 顶点邻接（按索引；split_crease 后同位置会有多个副本）
    nb = defaultdict(set)
    for a, b, c in idx:
        nb[a].update((b, c))
        nb[b].update((a, c))
        nb[c].update((a, b))

    # 三角形三顶点 creator 是否一致
    tri_creator = creator[idx]
    same = (tri_creator[:, 0] == tri_creator[:, 1]) & (tri_creator[:, 1] == tri_creator[:, 2])
    print(f"单色三角形 {same.sum()} / {len(idx)}（{100*same.mean():.1f}%）")

    # 顶点是否与所有邻居同色
    isolated = 0
    boundary = 0
    for i, ns in nb.items():
        if not ns:
            continue
        ci = creator[i]
        diff = sum(1 for j in ns if creator[j] != ci)
        if diff == len(ns):
            isolated += 1
        elif diff:
            boundary += 1
    print(f"与所有邻居异色（孤立噪点）{isolated}，位于边界 {boundary}")

    # 按高度看每个 z 分层的 creator 组成：统计相邻 z 处是否反复横跳
    z = pos[:, 2]
    # 半球顶在 z≈? 用范围归一
    zlo, zhi = np.percentile(z, 0.5), np.percentile(z, 99.5)
    print(f"z 范围 {z.min():.2f}..{z.max():.2f}（0.5–99.5 分位 {zlo:.2f}..{zhi:.2f}）")

    # 取朝上的面（法向 z 大的顶点不好拿，用三角形法向）
    p = pos[idx]
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-9
    n[ok] /= ln[ok, None]
    up = n[:, 2] > 0.6
    print(f"朝上三角形 {up.sum()}")

    cu = creator[idx][up]
    mixed = ~((cu[:, 0] == cu[:, 1]) & (cu[:, 1] == cu[:, 2]))
    print(f"朝上三角形里混色 {mixed.sum()}（{100*mixed.mean():.2f}%）")

    # 每个 creator 的 z 中位数 / 范围（只看顶点）
    by = defaultdict(list)
    for c, zz in zip(creator.tolist(), z.tolist()):
        by[c].append(zz)
    print("\ncreator -> z 中位数 / 5–95 分位 / 顶点数")
    for c in sorted(by):
        a = np.array(by[c])
        print(
            f"  {c:3d}  {np.median(a):7.2f}  {np.percentile(a,5):7.2f}..{np.percentile(a,95):7.2f}  n={len(a)}"
        )


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/dome_removed_only.html")
