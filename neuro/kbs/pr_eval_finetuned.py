#!/usr/bin/env python3
"""pr_eval_finetuned.py — P2: 微调头嵌入重算 (5 集) + Phase 0 协议 P/R 复评

对每集:
  1. 微调嵌入 = head(冻结 DINOv2 特征), L2 归一 → slam_results/pr_dinov2s_ft.npy
  2. 同 Phase 0 协议 (arcΔ>50m 最相似, 事件合并, vs oracle GT R=5m) 算 P/R
     —— 与零样本 (pr_dinov2s.npy) 并排对比, 验证 "假闭环下降 / 真闭环上升"。
  3. 导出 MATLAB 用 .mat (pr_embeddings_ft.mat, 变量 pr_embeddings)。
"""
import os

import numpy as np
import torch
import torch.nn as nn
import scipy.io as sio

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion', 'KITTI09Data_IMU_Fusion']
DARC = 50.0
TAUS = [0.5, 0.6, 0.7, 0.8, 0.85, 0.9]
HEAD = '/home/yangrb/openhutb/neuro/kbs/pr_head.pt'


class Head(nn.Module):
    def __init__(self, din=384, dmid=384, dout=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(din, dmid), nn.LayerNorm(dmid), nn.GELU(),
            nn.Linear(dmid, dout),
        )

    def forward(self, x):
        return nn.functional.normalize(self.net(x), dim=1)


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
            da = arc[ii][:, None] - arc[jj][None, :]
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


def pr_line(emb, p, arc, oracle, label):
    N = len(p)
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
    out = []
    for tau in TAUS:
        ev = merge_pairs([(j, i) for (j, i, s) in cands if s >= tau])
        tp = match_events(ev, oracle, p)
        prec = tp / len(ev) * 100 if ev else float('nan')
        rec = tp / len(oracle) * 100 if oracle else float('nan')
        out.append(f"τ={tau}: 事件={len(ev):3d} TP={tp} P={prec:5.1f}% R={rec:5.1f}%")
    print(f"  [{label}] " + ' | '.join(out), flush=True)


def main():
    ck = torch.load(HEAD, map_location='cpu')
    model = Head(ck['din'], 384, ck['dout']).cuda()
    model.load_state_dict(ck['state'])
    model.eval()
    for name in SETS:
        sub = os.path.join(D, name)
        emb = np.load(os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')).astype(np.float32)
        with torch.no_grad():
            ft = model(torch.from_numpy(emb).cuda()).cpu().numpy().astype(np.float32)
        np.save(os.path.join(sub, 'slam_results', 'pr_dinov2s_ft.npy'), ft)
        sio.savemat(os.path.join(sub, 'slam_results', 'pr_embeddings_ft.mat'),
                    {'pr_embeddings': ft})
        p, arc = load_gt(sub)
        oracle = oracle_events(p, arc)
        print(f"=== {name} ===  oracle(R5@50m)={len(oracle)}", flush=True)
        pr_line(emb, p, arc, oracle, '零样本')
        pr_line(ft, p, arc, oracle, '微调后')
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
