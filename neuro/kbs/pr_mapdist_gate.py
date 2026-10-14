#!/usr/bin/env python3
"""pr_mapdist_gate.py — Phase 0.5c: 地图直线距离门 (正确的闭环判别器)

候选 (j,i): DINOv2 sim >= τ 且 arcΔ >= D_arc (离开过); 接受为闭环 iff |p_i - p_j| <= R_loop。
真闭环: 路径长但直线距离小 (回到原地); 假闭环 (直路自相似): 路径长且直线距离也长 (仍在前方)。
只依赖轨迹位置, 无特征匹配。对 R_loop ∈ {5,10,15} × τ ∈ {0.6,0.7,0.8,0.9} 报 P/R (对同 R oracle)。
"""
import os

import numpy as np

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']
DARC = 50.0
TAUS = [0.6, 0.7, 0.8, 0.9]
R_LOOPS = [5.0, 10.0, 15.0]
MERGE_GAP = 30


def load_all(sub):
    emb = np.load(os.path.join(sub, 'slam_results', 'pr_dinov2s.npy'))
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        a = np.loadtxt(f, delimiter=',', usecols=(1, 2, 3))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])
    return emb, a, arc


def oracle_events(p, arc, R, D_arc=DARC):
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
    return merge_pairs(pairs)


def merge_pairs(pairs):
    pairs = sorted(pairs)
    ev = []
    for j, i in pairs:
        if ev and i - ev[-1][1] <= MERGE_GAP and j - ev[-1][0] <= MERGE_GAP:
            ev[-1] = (min(ev[-1][0], j), i)
        else:
            ev.append((j, i))
    return ev


def match_events(cand, oracle, p, tol=10.0):
    tp, used = 0, set()
    for (j, i) in cand:
        for m_, (jo, io) in enumerate(oracle):
            if m_ in used:
                continue
            if np.linalg.norm(p[i] - p[io]) < tol and np.linalg.norm(p[j] - p[jo]) < tol:
                tp += 1
                used.add(m_)
                break
    return tp


for name in SETS:
    sub = os.path.join(D, name)
    emb, p, arc = load_all(sub)
    N = len(p)
    S = emb @ emb.T
    print(f"=== {name} (N={N}) ===")
    for R in R_LOOPS:
        oracle = oracle_events(p, arc, R)
        for tau in TAUS:
            cands = []
            for i in range(N):
                b = int(np.searchsorted(arc, arc[i] - DARC, side='right'))
                if b < 50:
                    continue
                row = S[i, :b]
                j = int(np.argmax(row))
                s = float(row[j])
                if s >= tau:
                    cands.append((j, i, s))
            evs = merge_pairs([(j, i) for (j, i, s) in cands])
            acc = [(j, i) for (j, i) in evs if np.linalg.norm(p[i] - p[j]) <= R]
            tp = match_events(acc, oracle, p)
            prec = tp / len(acc) * 100 if acc else float('nan')
            rec = tp / len(oracle) * 100 if oracle else float('nan')
            print(f"  R={R:4.1f} τ={tau}: oracle={len(oracle):2d} 候选事件={len(evs):3d} "
                  f"过门={len(acc):2d} TP={tp:2d}  P={prec:5.1f}%  R={rec:5.1f}%")
