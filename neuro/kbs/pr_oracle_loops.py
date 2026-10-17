#!/usr/bin/env python3
"""pr_oracle_loops.py — 真闭环上限 (DL-NLM Phase 0, 事件级)

真闭环事件: 帧 i 与帧 j (j < i) 满足 dist3d(p_j, p_i) < R 且 arc[i] - arc[j] > D_arc
(离开过 D_arc 以上再回来; 停车原地不动不算)。连续帧合并为一个闭环事件。
"""
import os
import numpy as np

D = '../data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']


def load_gt(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        a = np.loadtxt(f, delimiter=',', usecols=(1, 2, 3))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])
    return a, arc


def loop_events(p, arc, R, D_arc, merge_gap=30):
    N = len(p)
    pairs = []
    BLK = 500
    for s in range(0, N, BLK):
        ii = np.arange(s, min(s + BLK, N))
        for j0 in range(0, s, BLK):
            jj = np.arange(j0, min(j0 + BLK, s))
            d = np.linalg.norm(p[ii, None, :] - p[None, jj, :], axis=2)
            da = arc[ii, None] - arc[None, jj]
            m = (d < R) & (da > D_arc)
            r, c = np.nonzero(m)
            pairs += list(zip(jj[c].tolist(), ii[r].tolist()))
    pairs.sort()
    ev = []
    for j, i in pairs:
        if ev and i - ev[-1][1] <= merge_gap and j - ev[-1][0] <= merge_gap:
            ev[-1] = (min(ev[-1][0], j), i)
        else:
            ev.append((j, i))
    return ev


hdr = f"{'set':24s} | 事件数 (R5, Darc): 30m   50m  100m | R3@50  R10@50"
print(hdr)
for name in SETS:
    p, arc = load_gt(os.path.join(D, name))
    e30 = loop_events(p, arc, 5, 30)
    e50 = loop_events(p, arc, 5, 50)
    e100 = loop_events(p, arc, 5, 100)
    e3 = loop_events(p, arc, 3, 50)
    e10 = loop_events(p, arc, 10, 50)
    seg = ' '.join(f"{arc[a]:.0f}->{arc[b]:.0f}m" for a, b in e50[:6])
    if len(e50) > 6:
        seg += f" ...({len(e50)}事件)"
    print(f"{name:24s} | {len(e30):4d}    {len(e50):3d}  {len(e100):3d} | {len(e3):3d} {len(e10):3d}")
    print(f"{'':24s}   R5@50: {seg or '无'}")
