#!/usr/bin/env python3
"""pr_pnp_gate.py — Phase 0.5b: 基于位姿假设的 PnP 几何验证门 (判别力天花板测试)

对每个 DINOv2 候选事件代表对 + 每个 oracle 真闭环对:
  位姿假设 = GT 位置 + yaw (roll/pitch 常数安装偏置在两帧间相消, 不影响判别);
  SIFT knnMatch (ratio 0.75) → cv2.triangulatePoints (P1, P2 由位姿假设)
  → 内点: 两点前 + 深度 <200m + 两图重投影误差 <3px。
真闭环: 位姿假设 ≈ 真实相对几何 → 多内点; 假闭环 (直路自相似, 假设位姿 = 前进 30-200m): 匹配三角化不一致 → 少内点。
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


def load_pose(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        a = np.loadtxt(f, delimiter=',')
    pos = a[:, 1:4]
    yaw = a[:, 6]
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
    return pos, yaw, arc


def R_yaw(y):
    c, s = np.cos(y), np.sin(y)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def reproj(R, c, pts):
    q = pts @ R.T + c
    qh = np.hstack([q, np.ones((len(q), 1))])
    img = (K @ qh.T).T
    return img[:, :2] / img[:, 2:3]


def pnp_inliers(sub, j, i, cfg, pos, yaw):
    kpj, desj = feats(sub, j, cfg)
    kpi, desi = feats(sub, i, cfg)
    if desj is None or desi is None or len(desj) < 30 or len(desi) < 30:
        return 0, 0
    mm = cv2.BFMatcher(cv2.NORM_L2).knnMatch(desj, desi, k=2)
    good = [a for a, b in mm if b is not None and a.distance < 0.75 * b.distance]
    if len(good) < 20:
        return 0, 0
    pj = np.float64([kpj[x.queryIdx].pt for x in good])
    pi = np.float64([kpi[x.trainIdx].pt for x in good])
    Rj, Ri = R_yaw(yaw[j]), R_yaw(yaw[i])
    P1 = K @ np.hstack([Rj, pos[j][:, None]])
    P2 = K @ np.hstack([Ri, pos[i][:, None]])
    pts4 = cv2.triangulatePoints(P1, P2, pj.T, pi.T)
    pts3 = pts4[:3].T / pts4[3:4].T
    v1 = (pts3 - pos[j]) @ Rj.T
    v2 = (pts3 - pos[i]) @ Ri.T
    valid = (v1[:, 2] > 0.5) & (v2[:, 2] > 0.5) & (np.linalg.norm(pts3, axis=1) < 200.0)
    if valid.sum() < 10:
        return 0, len(good)
    rj = reproj(Rj, pos[j], pts3[valid])
    ri = reproj(Ri, pos[i], pts3[valid])
    e = np.maximum(np.linalg.norm(rj - pj[valid], axis=1), np.linalg.norm(ri - pi[valid], axis=1))
    return int((e < 3.0).sum()), len(good)


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
    pos, yaw, arc = load_pose(sub)
    N = len(pos)
    oracle = oracle_events(pos, arc)
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
    rep = []
    for (j0, i0) in evs:
        c = [(s, j, i) for (j, i, s) in cands if abs(i - i0) <= MERGE_GAP and abs(j - j0) <= MERGE_GAP]
        if c:
            s, j, i = max(c)
            rep.append((j, i, s))
    print(f"=== {name}  τ={TAU} 事件={len(evs)} oracle={len(oracle)} ===", flush=True)
    rows = []
    for (j, i, s) in rep:
        n_in, n_tot = pnp_inliers(sub, j, i, cfg, pos, yaw)
        rows.append((n_in, n_tot, s, False, j, i))
    for (j, i) in oracle:
        n_in, n_tot = pnp_inliers(sub, j, i, cfg, pos, yaw)
        rows.append((n_in, n_tot, -1.0, True, j, i))
    for (n_in, n_tot, s, is_orb, j, i) in sorted(rows, key=lambda r: (not r[3], -r[0])):
        tag = 'ORACLE ' if is_orb else ''
        sim = f"sim={s:.3f}" if s > 0 else 'oracle'
        print(f"  {tag}({j},{i}) {sim} inliers={n_in}/{n_tot}", flush=True)
    o = [r[0] / r[1] * 100 if r[1] else 0 for r in rows if r[3]]
    f_ = [r[0] / r[1] * 100 if r[1] else 0 for r in rows if not r[3]]
    import statistics as st
    fm = st.median(f_) if f_ else float('nan')
    print(f"  >>> 内点率%: 真={o or []}  假: 中位={fm:.0f} max={max(f_):.0f} min={min(f_):.0f}", flush=True)
