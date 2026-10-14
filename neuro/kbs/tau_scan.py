#!/usr/bin/env python3
"""tau_scan.py — PR 分支相似度阈值 τ 细扫 (0.90-0.98, 事件级明细)

背景 (2026-10-14): 重训后的 P2 头在 Town07 (PR 增益 -3.0m) / Town10HD (-1.17m)
净负, 根因 = 假闭环事件的 A-fix 修正叠加过冲 (EKF 实际漂移小的集上)。
本脚本逐 τ 给出事件数/TP/事件明细 (j,i,sim), 判断是否存在 per-sequence τ
标定能消除假事件且保住真事件 (与现有 VT/CE 逐集标定同方法论)。

用法: python tau_scan.py [Set ...]   (默认 Town05/Town07/Town10HD)
"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pr_eval_finetuned import (D, Head, load_gt, oracle_events,
                               merge_pairs, match_events)

TAUS = [0.90, 0.92, 0.94, 0.96, 0.98]
DARC = 50.0


def main():
    default = ['Town05Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
               'Town10HDData_IMU_Fusion']
    names = sys.argv[1:] or default
    ck = torch.load('/home/yangrb/openhutb/neuro/kbs/pr_head.pt',
                    map_location='cpu')
    model = Head(ck['din'], 384, ck['dout']).cuda().eval()
    for name in names:
        sub = os.path.join(D, name)
        emb = np.load(os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')).astype(np.float32)
        with torch.no_grad():
            ft = model(torch.from_numpy(emb).cuda()).cpu().numpy().astype(np.float32)
        p, arc = load_gt(sub)
        oracle = oracle_events(p, arc)
        N = len(p)
        S = ft @ ft.T
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
        print(f"=== {name} === oracle={len(oracle)}", flush=True)
        for tau in TAUS:
            ev = merge_pairs([(j, i) for (j, i, s) in cands if s >= tau])
            tp = match_events(ev, oracle, p)
            print(f"  τ={tau:.2f}: 事件={len(ev):3d} TP={tp}", flush=True)
            for (j, i) in ev:
                sim = float(S[i, j])
                print(f"    cur={i:5d} old={j:5d} sim={sim:.4f}", flush=True)


if __name__ == '__main__':
    main()
