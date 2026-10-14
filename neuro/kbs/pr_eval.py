#!/usr/bin/env python3
"""pr_eval.py — DL-NLM Phase 0 零样本闭环评估 (DINOv2 vs ORB vs oracle)

DINOv2 候选: 帧 i 取 arc 距离 > D_arc 的最相似 j (cos > τ), 事件合并; 对 oracle (GT R=5m, D_arc=50m) 算 P/R。
ORB 基线: experiences.mat 的 link (json 口径 1<=eid<=k), 逐边查 GT 距离/弧长 → TP/FP。
事件匹配: 候选 (j,i) 与 oracle (jo,io) 若两端空间距离 < 10m 记为匹配。
"""
import os

import numpy as np
import scipy.io as sio

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']
DARC = 50.0
TAUS = [0.5, 0.6, 0.7, 0.8, 0.85, 0.9]


def load_gt(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        a = np.loadtxt(f, delimiter=',', usecols=(1, 2, 3))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])
    return a, arc


def oracle_events(p, arc, R=5.0, D_arc=DARC, merge_gap=30):
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


def merge_pairs(pairs, merge_gap=30):
    pairs = sorted(pairs)
    ev = []
    for j, i in pairs:
        if ev and i - ev[-1][1] <= merge_gap and j - ev[-1][0] <= merge_gap:
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


def orb_edges(sub):
    m = sio.loadmat(os.path.join(sub, 'slam_results', 'experiences.mat'))
    E = m['EXPERIENCES']
    n = E.shape[1] if E.ndim == 2 else E.shape[0]
    edges = []
    for k in range(n):
        e = E[0, k] if E.ndim == 2 else E[k]
        try:
            nl = int(float(np.atleast_1d(e['numlinks']).ravel()[0]))
        except Exception:
            continue
        if nl <= 0:
            continue
        lnks = e['links']
        if lnks.shape != ():
            lnks = np.atleast_1d(lnks)
        for j in range(len(lnks)):
            lid = lnks[j]
            if lid.shape != ():
                lid = lid.ravel()[0]
            try:
                eid = int(float(np.atleast_1d(lid['exp_id']).ravel()[0]))
            except Exception:
                continue
            if 1 <= eid <= k:
                edges.append((k, eid - 1))
    return edges


for name in SETS:
    sub = os.path.join(D, name)
    emb_path = os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')
    if not os.path.exists(emb_path):
        print(f"=== {name} ===  (嵌入未就绪, 跳过 DINOv2)")
    p, arc = load_gt(sub)
    N = len(p)
    oracle = oracle_events(p, arc)
    # ORB
    orb = orb_edges(sub)
    orb_tp = sum(1 for (j, i) in orb if np.linalg.norm(p[i] - p[j]) < 5.0 and arc[i] - arc[j] > DARC)
    orb_det = ' '.join(f"({j},{i}) d={np.linalg.norm(p[i]-p[j]):.1f}m arcΔ={arc[i]-arc[j]:.0f}m"
                       for (j, i) in orb[:12])
    print(f"=== {name} ===  oracle事件(R5@50m)={len(oracle)}  "
          f"ORB边={len(orb)} (TP={orb_tp}, FP={len(orb)-orb_tp})")
    print(f"  ORB: {orb_det or '无'}")
    if not os.path.exists(emb_path):
        continue
    emb = np.load(emb_path)
    S = emb @ emb.T
    cands = []
    for i in range(N):
        b = int(np.searchsorted(arc, arc[i] - DARC, side='right'))
        if b < 50:
            continue
        row = S[i, :b]
        j = int(np.argmax(row))
        s = float(row[j])
        if s > 0.3:
            cands.append((j, i, s))
    for tau in TAUS:
        ev = merge_pairs([(j, i) for (j, i, s) in cands if s >= tau])
        tp = match_events(ev, oracle, p)
        prec = tp / len(ev) * 100 if ev else float('nan')
        rec = tp / len(oracle) * 100 if oracle else float('nan')
        print(f"  DINOv2 τ={tau}: 事件={len(ev):3d}  TP={tp}  P={prec:5.1f}%  R={rec:5.1f}%")
