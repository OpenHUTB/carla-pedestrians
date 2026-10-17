#!/usr/bin/env python3
"""pr_geom_gate.py — Phase 0.5: DINOv2 候选闭环的几何验证门 (SIFT + RANSAC 本质矩阵)

对每个 DINOv2 候选事件 (取事件内相似度最高的一对 (j,i)):
  SIFT 匹配 → findEssentialMat (fx=fy=320, 640x480, FOV90) → 内点数。
真闭环 → 几何一致 → 多内点; 直路自相似假闭环 → 少内点。
"""
import os

import cv2
import numpy as np

D = '../data/01_NeuroSLAM_Datasets'
SETS = {
    'Town05Data_IMU_Fusion': dict(pad=4, base=1),
    'Town03Data_IMU_Fusion': dict(pad=4, base=1),
    'Town07Data_IMU_Fusion': dict(pad=4, base=1),
    'Town10HDData_IMU_Fusion': dict(pad=4, base=1),
    'KITTI07Data_IMU_Fusion': dict(pad=6, base=0),
}
K = np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]])
DARC = 50.0
TAU = 0.8
MERGE_GAP = 30

sift = cv2.SIFT_create(nfeatures=1200)
FMAP = {}


def img_path(sub, i, cfg):
    return os.path.join(sub, f"{cfg['base'] + i:0{cfg['pad']}d}.png")


def feats(sub, i, cfg):
    key = (sub, i)
    if key not in FMAP:
        g = cv2.imread(img_path(sub, i, cfg), cv2.IMREAD_GRAYSCALE)
        if g is None:
            FMAP[key] = (None, None)
        else:
            kp, des = sift.detectAndCompute(g, None)
            FMAP[key] = (kp, des)
    return FMAP[key]


def inliers(sub, j, i, cfg):
    kpj, desj = feats(sub, j, cfg)
    kpi, desi = feats(sub, i, cfg)
    if desj is None or desi is None or len(desj) < 30 or len(desi) < 30:
        return 0, 0
    mm = cv2.BFMatcher(cv2.NORM_L2).match(desj, desi)
    mm = sorted(mm, key=lambda x: x.distance)[:80]
    pj = np.float32([kpj[x.queryIdx].pt for x in mm])
    pi = np.float32([kpi[x.trainIdx].pt for x in mm])
    E, mask = cv2.findEssentialMat(pj, pi, K, method=cv2.RANSAC, prob=0.999)
    if E is None or mask is None:
        return 0, len(mm)
    return int(mask.sum()), len(mm)


def load_gt(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        a = np.loadtxt(f, delimiter=',', usecols=(1, 2, 3))
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])
    return a, arc


def oracle_events(p, arc, R=5.0, D_arc=DARC):
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
        if ev and i - ev[-1][1] <= MERGE_GAP and j - ev[-1][0] <= MERGE_GAP:
            ev[-1] = (min(ev[-1][0], j), i)
        else:
            ev.append((j, i))
    return ev


def merge_pairs(pairs):
    pairs = sorted(pairs)
    ev = []
    for j, i in pairs:
        if ev and i - ev[-1][1] <= MERGE_GAP and j - ev[-1][0] <= MERGE_GAP:
            ev[-1] = (min(ev[-1][0], j), i)
        else:
            ev.append((j, i))
    return ev


for name, cfg in SETS.items():
    sub = os.path.join(D, name)
    emb = np.load(os.path.join(sub, 'slam_results', 'pr_dinov2s.npy'))
    p, arc = load_gt(sub)
    N = len(p)
    oracle = oracle_events(p, arc)
    oracle_set = set(oracle)
    S = emb @ emb.T
    cands = []
    for i in range(N):
        b = int(np.searchsorted(arc, arc[i] - DARC, side='right'))
        if b < 50:
            continue
        row = S[i, :b]
        j = int(np.argmax(row))
        s = float(row[j])
        if s >= TAU:
            cands.append((j, i, s))
    evs = merge_pairs([(j, i) for (j, i, s) in cands])
    # 每事件取最高 s 的代表对
    rep = []
    for (j0, i0) in evs:
        c = [(s, j, i) for (j, i, s) in cands if abs(i - i0) <= MERGE_GAP and abs(j - j0) <= MERGE_GAP]
        if c:
            s, j, i = max(c)
            rep.append((j, i, s, (j, i0)))
    print(f"=== {name}  τ={TAU} 事件={len(evs)} oracle={len(oracle)} ===", flush=True)
    res = []
    for k, (j, i, s, evkey) in enumerate(rep):
        n_in, n_tot = inliers(sub, j, i, cfg)
        is_orb = evkey in oracle_set
        res.append((n_in, n_tot, s, is_orb, j, i))
        tag = 'ORACLE' if is_orb else ''
        print(f"  [{k+1}/{len(rep)}] ({j},{i}) sim={s:.3f} inliers={n_in}/{n_tot} {tag}", flush=True)
    res.sort(reverse=True)
    ins = [r[0] for r in res]
    o_in = [r[0] for r in res if r[3]]
    f_in = [r[0] for r in res if not r[3]]
    import statistics as st
    def med(x): return st.median(x) if x else float('nan')
    print(f"  >>> 内点数: oracle={o_in or []}  假={med(f_in):.0f}中位 (max {max(f_in) if f_in else 0})  "
          f"分界: 真 min={min(o_in) if o_in else '-'} vs 假 max={max(f_in) if f_in else 0}", flush=True)
