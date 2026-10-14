#!/usr/bin/env python3
"""pr_oracle_revisit.py — oracle 回访上限 (DL-NLM Phase 0)

定义: 对帧 i, 若存在 j ∈ [i-max_lb, i-gap) 使 dist3d(p_j, p_i) < R, 记为一次真回访。
回答: 各集路线在 GT 层面到底有没有重访 (matcher 无关的上限), 以及重访发生在哪些区段。
"""
import os
import numpy as np

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']


def load_gt(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        return np.loadtxt(f, delimiter=',', usecols=(1, 2, 3))


print(f"{'set':24s} | R=5m: gap50/100/250  | R=3m/10m (gap100) | 重访区段 (R=5,gap100)")
for name in SETS:
    p = load_gt(os.path.join(D, name))
    N = len(p)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    cnt = {(g, r): 0 for g in (50, 100, 250) for r in (3, 5, 10)}
    hit_frames = []
    for i in range(251, N):
        lo = max(0, i - 3000)
        d = np.linalg.norm(p[i] - p[lo:i - 100], axis=1)
        for r in (3, 5, 10):
            cnt[(100, r)] += int((d < r).sum())
        d50 = np.linalg.norm(p[i] - p[max(0, i - 2000):i - 50], axis=1)
        cnt[(50, 5)] += int((d50 < 5.0).sum())
        d250 = np.linalg.norm(p[i] - p[lo:i - 250], axis=1)
        cnt[(250, 5)] += int((d250 < 5.0).sum())
        if (d < 5.0).any():
            hit_frames.append(i)
    # 重访区段 (连续帧合并, 弧长表示)
    segs = []
    for i in hit_frames:
        if segs and i - segs[-1][1] <= 5:
            segs[-1][1] = i
        else:
            segs.append([i, i])
    seg_str = ' '.join(f"{arc[a]:.0f}-{arc[b]:.0f}m" for a, b in segs[:8])
    if len(segs) > 8:
        seg_str += f" ...({len(segs)}段)"
    print(f"{name:24s} | {cnt[(50,5)]:5d}/{cnt[(100,5)]:5d}/{cnt[(250,5)]:5d} "
          f"| {cnt[(100,3)]:4d}/{cnt[(100,10)]:4d} | {seg_str or '无'}")
