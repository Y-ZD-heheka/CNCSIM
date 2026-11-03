#!/usr/bin/env python
"""网格缺陷体检：把导出页里的顶点数据当网格查一遍。

用法：

    python tools/check_mesh.py out/dome.html
    python tools/check_mesh.py out/dome.html --ref-sphere 25,25,84,15

为什么需要它：`check_preview.mjs` 只证明"页面能画出来"，画得对不对它不知道。
破面、翻面、退化三角形、孤立色块这些缺陷在截图上要么看不见、要么看见也说不清
位置，但在数据上都是**可枚举、可计数**的。

检查四类，对应三类不同的失败方式：

| 类别 | 查什么 | 失败意味着 |
|---|---|---|
| 几何 | NaN / 索引越界 / 退化三角形 | 顶点缓冲写坏，GPU 会画出噪点或整片消失 |
| 拓扑 | 边界边 / 非流形边 / 连通分量 | 破面、LOD 分块裂缝、悬空碎屑 |
| 法线 | 几何法线 vs 顶点法线 / 参考曲面偏差 | 翻面发黑、曲面"波纹"（光照放大了法线误差） |
| 颜色 | 相邻色差 / 孤立色块 | 溯源索引错位——就是"麻点" |

退出码：有硬错误（NaN / 越界 / 翻面 > 1%）返回 1，否则 0。
"""

import base64
import re
import sys

import numpy as np


def load(path):
    html = open(path, encoding="utf-8").read()
    raw = dict(re.findall(r'(POS|NRM|IDX|COL)_B64 = "([^"]+)"', html))
    if not raw:
        sys.exit(f"{path} 里没有 POS_B64 / NRM_B64 / IDX_B64 / COL_B64")
    pos = np.frombuffer(base64.b64decode(raw["POS"]), dtype=np.float32).reshape(-1, 3)
    nrm = np.frombuffer(base64.b64decode(raw["NRM"]), dtype=np.float32).reshape(-1, 3)
    idx = np.frombuffer(base64.b64decode(raw["IDX"]), dtype=np.uint32).reshape(-1, 3)
    col = np.frombuffer(base64.b64decode(raw["COL"]), dtype=np.uint8).reshape(-1, 4)
    return pos, nrm, idx, col


def section(title):
    print(f"\n=== {title} ===")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "out/preview.html"
    ref = None
    if "--ref-sphere" in sys.argv:
        ref = [float(v) for v in sys.argv[sys.argv.index("--ref-sphere") + 1].split(",")]

    pos, nrm, tri, col = load(path)
    nv, nf = len(pos), len(tri)
    hard = []

    print(f"文件 {path}")
    print(f"顶点 {nv} / 三角形 {nf}")

    # ── 几何 ────────────────────────────────────────────────
    section("几何")
    bad_nan = int(np.isnan(pos).any(1).sum() + np.isnan(nrm).any(1).sum())
    print(f"  NaN/Inf 顶点          {bad_nan}")
    if bad_nan:
        hard.append(f"{bad_nan} 个顶点带 NaN")

    oob = int((tri >= nv).sum())
    print(f"  索引越界              {oob}")
    if oob:
        hard.append(f"{oob} 个索引越界")

    v0, v1, v2 = pos[tri[:, 0]], pos[tri[:, 1]], pos[tri[:, 2]]
    cross = np.cross(v1 - v0, v2 - v0)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    degen = int((area < 1e-12).sum())
    print(f"  退化三角形(面积<1e-12) {degen}  ({degen / max(nf,1) * 100:.3f}%)")

    # ── 拓扑 ────────────────────────────────────────────────
    section("拓扑")
    # 每条边按 (小下标, 大下标) 归一，数它被几个面共用
    E = np.vstack([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]).astype(np.int64)
    E.sort(axis=1)
    key = E[:, 0] * nv + E[:, 1]
    _, cnt = np.unique(key, return_counts=True)
    boundary = int((cnt == 1).sum())
    nonman = int((cnt > 2).sum())
    print(f"  边界边(只被 1 面用)   {boundary}")
    print(f"  非流形边(被 >2 面用)  {nonman}")
    if nonman:
        hard.append(f"{nonman} 条非流形边")
    # 封闭件边界边应为 0；这里是开放面（毛坯外表面被裁掉），所以边界边是正常的，
    # 但**它们应该全部落在毛坯棱上**。这里只报数量，位置由调用者用 --ref-sphere 之外的
    # 方式核对（见 docs/项目记忆.md §渲染体检）。

    # 连通分量：并查集。**按位置判连通，不按索引**。
    #
    # `split_crease_normals` 会为硬边复制顶点，副本位置与原件完全相同、索引不同。
    # 按索引算会把棱上的"鳍"报成孤立碎屑（实测 R15 半球报出 4 个 3 顶点分量，
    # 全是这种副本，几何上它们和主网格是连着的）。
    q = np.round(pos.astype(np.float64) / 1e-4).astype(np.int64)
    _, vid = np.unique(q, axis=0, return_inverse=True)
    nq = int(vid.max()) + 1
    parent = list(range(nq))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in zip(vid[E[:, 0]], vid[E[:, 1]]):
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[ra] = rb
    # 每个**唯一位置**算一个顶点，不是每份副本
    root_of_pos = np.array([find(int(v)) for v in range(nq)])
    uniq, sizes = np.unique(root_of_pos, return_counts=True)
    order = np.argsort(-sizes)
    print(f"  连通分量              {len(sizes)}（按位置去重后 {nq} 个位置）")
    if len(sizes) > 1:
        small_i = [i for i in order if sizes[i] < 200][:8]
        print(f"    大分量 {int((sizes >= 200).sum())} 个，小分量 {int((sizes < 200).sum())} 个")
        for i in small_i:
            m = root_of_pos == uniq[i]
            c0 = pos[vid == np.flatnonzero(m)[0]][0] if m.any() else pos[0]
            # 用该分量里任一个位置反查一个顶点坐标
            vids = np.flatnonzero(m)
            j = int(np.flatnonzero(vid == vids[0])[0])
            c0 = pos[j]
            print(f"      · {int(sizes[i]):>3} 位置 @ ({c0[0]:.2f}, {c0[1]:.2f}, {c0[2]:.2f})")
        print("    → 小分量是**几何上真的断开**的料：悬空碎屑或分块没接上")
    else:
        print("    ✓ 单一连通体（棱上的顶点副本已按位置合并）")

    # ── 法线 ────────────────────────────────────────────────
    section("法线")
    gn = cross / (np.linalg.norm(cross, axis=1, keepdims=True) + 1e-20)
    vn = nrm[tri].mean(axis=1)
    vn /= np.linalg.norm(vn, axis=1, keepdims=True) + 1e-20
    dot = (gn * vn).sum(axis=1)
    flipped = int((dot < 0).sum())
    print(f"  几何法线与顶点法线相反 {flipped}  ({flipped / max(nf,1) * 100:.3f}%)")
    if flipped > nf * 0.01:
        hard.append(f"{flipped / nf * 100:.2f}% 的面法线翻转")
    cos = np.clip(dot, -1, 1)
    print(f"  两者夹角 中位 {np.degrees(np.arccos(np.median(cos))):.2f}°  "
          f"p99 {np.degrees(np.arccos(np.percentile(cos, 1))):.2f}°")

    if ref:
        cx, cy, cz, R = ref
        d = np.linalg.norm(pos - np.array([cx, cy, cz]), axis=1)
        # 只取**球心平面以上**的球面：底面那一圈到球心的距离也正好是 R，
        # 但法线朝上（与径向成 90°），混进来会把 p99 污染成 72°
        sel = (np.abs(d - R) < 0.3) & (pos[:, 2] > cz + 0.4)
        if sel.any():
            ideal = (pos[sel] - np.array([cx, cy, cz])) / d[sel, None]
            a = np.degrees(np.arccos(np.clip((nrm[sel] * ideal).sum(1), -1, 1)))
            print(f"  球面顶点 {int(sel.sum())} 的法线与解析法线偏差："
                  f"中位 {np.median(a):.2f}°  均值 {a.mean():.2f}°  p99 {np.percentile(a, 99):.2f}°  最大 {a.max():.2f}°")
            print("    → 这是「波纹」的量化：>2° 肉眼可见，>5° 就是明显的台阶")
        r = np.linalg.norm(pos - np.array([cx, cy, cz]), axis=1)
        ds = r[np.abs(r - R) < 0.3] - R
        if len(ds):
            print(f"  球面径向误差：均值 {ds.mean():+.4f} mm  标准差 {ds.std():.4f}  "
                  f"范围 [{ds.min():+.3f}, {ds.max():+.3f}]")

    # ── 颜色 ────────────────────────────────────────────────
    section("颜色")
    c = col[:, 0]
    hist = np.bincount(c, minlength=16)
    print("  creator 分布        " + " ".join(f"{i}:{n}" for i, n in enumerate(hist) if n))
    da = c[tri[:, 0]].astype(int) - c[tri[:, 1]].astype(int)
    db = c[tri[:, 1]].astype(int) - c[tri[:, 2]].astype(int)
    dc = c[tri[:, 2]].astype(int) - c[tri[:, 0]].astype(int)
    jump = int((np.abs(da) + np.abs(db) + np.abs(dc) > 0).sum())
    print(f"  三角形内色号不一致    {jump}  ({jump / max(nf,1) * 100:.2f}%)"
          f"  → 就是颜色「麻点」的面积占比")

    # 孤立色点：0.5 mm 格子里找邻居，8 个最近邻里 ≥7 个不同色
    cell = np.floor(pos / 0.5).astype(np.int64)
    order = np.argsort(cell[:, 0] * 1000003 + cell[:, 1] * 1009 + cell[:, 2])
    keys = cell[order, 0] * 1000003 + cell[order, 1] * 1009 + cell[order, 2]
    uq, st, cn = np.unique(keys, return_index=True, return_counts=True)
    bucket = {u: order[s:s + n] for u, s, n in zip(uq, st, cn)}
    iso = np.zeros(nv, bool)
    for i in range(nv):
        cx, cy, cz = cell[i]
        nb = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    j = bucket.get((cx + dx) * 1000003 + (cy + dy) * 1009 + (cz + dz))
                    if j is not None:
                        nb.append(j)
        nb = np.concatenate(nb) if nb else np.empty(0, int)
        nb = nb[nb != i]
        if len(nb) < 6:
            continue
        d = np.linalg.norm(pos[nb] - pos[i], axis=1)
        near = nb[np.argsort(d)[:8]]
        iso[i] = (c[near] != c[i]).sum() >= 7
    print(f"  孤立色点顶点          {int(iso.sum())}  ({iso.sum() / nv * 100:.3f}%)")
    if iso.any():
        print("    孤立点的 color 分布 " + " ".join(
            f"{i}:{n}" for i, n in enumerate(np.bincount(c[iso], minlength=16)) if n))

    # ── 结论 ────────────────────────────────────────────────
    section("结论")
    if hard:
        print("  ✗ 硬错误：")
        for h in hard:
            print(f"    - {h}")
        return 1
    print("  ✓ 无硬错误（几何/索引/法线朝向正常）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
