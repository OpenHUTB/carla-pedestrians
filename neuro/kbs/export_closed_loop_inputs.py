#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kbs/export_closed_loop_inputs.py — 把 §4.5 闭环基准的输入导出为 kb_s2_se3_check.m 需要的 5 个 txt

背景: kb_s2_se3_check.m 读 /tmp/se3_*.txt; 早先这步是手工做的、没有脚本, /tmp 一清就断。
本脚本把它补成一条命令。

用法:
  python3 export_closed_loop_inputs.py                       # 用默认 <data>/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion
  python3 export_closed_loop_inputs.py --seq /path/to/Town01Turnaround_IMU_Fusion --out /tmp
  python3 export_closed_loop_inputs.py --csv                 # 另写 *_pred.csv 便于 Python 复算

输入 (相对 --seq):
  ground_truth.txt                 表头 pos_x,pos_y,pos_z
  fusion_pose.txt                  EKF 核轨迹 (表头 pos_x,pos_y,pos_z)
  a_final/exp_trajectory.txt       部署轨迹 A+S2
  a_final/exp_trajectory_preS2.txt 仅 A (无 S2)
  a_final/loop_constraint_events.mat  NLM_LOOP_EVENTS (N×>=1, MATLAB 1-based 帧号)
  或 loop_event_frames.txt         每行一个帧号 (二选一, mat 优先)

输出 (--out, 默认 /tmp):
  se3_gt.txt  se3_pres2.txt  se3_final.txt  se3_ekf.txt  se3_events.txt
全部为无表头三列 x,y,z (events 为一列帧号), 行数截断到四条轨迹的最小公共长度。
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

COLS = ['pos_x', 'pos_y', 'pos_z']
DEFAULT_SEQ = '../data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion'


def load_xyz(path, cols=COLS):
    """带表头的日志 (ground_truth.txt / fusion_pose.txt): 按列名取三维位置。"""
    if not os.path.exists(path):
        raise SystemExit(f'缺少输入文件: {path}')
    df = pd.read_csv(path)
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise SystemExit(f'{path} 缺少列 {missing} (实际列: {list(df.columns)})')
    return df[cols].values.astype(float)


def load_xyz3(path):
    """无表头的三列轨迹 (a_final/exp_trajectory*.txt): 直接当 x,y,z 读。"""
    if not os.path.exists(path):
        raise SystemExit(f'缺少输入文件: {path}')
    a = np.loadtxt(path, delimiter=',', ndmin=2).astype(float)
    if a.shape[1] < 3:
        raise SystemExit(f'{path} 列数不足 3 (实际 {a.shape[1]})')
    return a[:, :3]


def load_events(seq):
    mat = os.path.join(seq, 'a_final', 'loop_constraint_events.mat')
    txt = os.path.join(seq, 'loop_event_frames.txt')
    if os.path.exists(mat):
        import scipy.io as sio
        m = sio.loadmat(mat)
        ev = None
        for k in ('NLM_LOOP_EVENTS', 'LOOP_EVENTS', 'events'):
            if k in m:
                ev = np.asarray(m[k], dtype=float)
                break
        if ev is None:
            raise SystemExit(f'{mat} 中找不到 NLM_LOOP_EVENTS')
        idx = ev[:, 0]
    elif os.path.exists(txt):
        idx = np.loadtxt(txt, ndmin=1).astype(float)
    else:
        raise SystemExit(f'{seq} 下既无 a_final/loop_constraint_events.mat 也无 loop_event_frames.txt')
    idx = np.unique(idx.astype(int))
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seq', default=DEFAULT_SEQ, help='Town01 折返序列目录')
    ap.add_argument('--out', default='/tmp', help='输出目录 (kb_s2_se3_check.m 默认 /tmp)')
    ap.add_argument('--csv', action='store_true', help='额外写 se3_*_pred.csv 供 Python 复算')
    a = ap.parse_args()

    seq = a.seq
    gt = load_xyz(os.path.join(seq, 'ground_truth.txt'))
    ekf = load_xyz(os.path.join(seq, 'fusion_pose.txt'))
    fin = load_xyz3(os.path.join(seq, 'a_final', 'exp_trajectory.txt'))
    pre = load_xyz3(os.path.join(seq, 'a_final', 'exp_trajectory_preS2.txt'))
    evs = load_events(seq)

    n = min(len(gt), len(ekf), len(fin), len(pre))
    gt, ekf, fin, pre = gt[:n], ekf[:n], fin[:n], pre[:n]
    evs = evs[evs <= n]

    os.makedirs(a.out, exist_ok=True)
    for name, arr in [('se3_gt', gt), ('se3_pres2', pre), ('se3_final', fin), ('se3_ekf', ekf)]:
        np.savetxt(os.path.join(a.out, f'{name}.txt'), arr, fmt='%.6f', delimiter=' ')
    np.savetxt(os.path.join(a.out, 'se3_events.txt'), evs.reshape(-1, 1), fmt='%d')
    if a.csv:
        for name, arr in [('gt', gt), ('pres2', pre), ('final', fin), ('ekf', ekf)]:
            pd.DataFrame(arr, columns=COLS).to_csv(os.path.join(a.out, f'se3_{name}_pred.csv'), index=False)

    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    print(f'seq      : {seq}')
    print(f'frames   : {n}  (gt={len(gt)} ekf={len(ekf)} final={len(fin)} pres2={len(pre)})')
    print(f'events   : {len(evs)}  (first={evs[0] if len(evs) else None} last={evs[-1] if len(evs) else None})')
    print(f'gt length: {L:.1f} m')
    print(f'written  : {a.out}/se3_{{gt,pres2,final,ekf,events}}.txt')
    print('next     : 在 MATLAB 里跑 kb_s2_se3_check  (期望 terminal 55.92 / 842.54, RMSE 87.25 / 477.76)')


if __name__ == '__main__':
    sys.exit(main())
