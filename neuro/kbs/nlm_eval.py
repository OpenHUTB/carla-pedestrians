#!/usr/bin/env python3
"""NLM 回归评测工具 (2026-10-04)

用法:
  python3 nlm_eval.py <dataset_dir> [--pred slam_results/exp_trajectory.txt] [--tag BASE]

口径 (与 MATLAB align_trajectories 'simple' + run_ablation.py 一致):
  - simple 对齐: 两轨迹起点对齐到原点, pred 缩放 scale = len_gt/len_pred (无旋转)
  - ATE = RMSE(对齐后 3D, m)
  - RPE = mean(||diff(gt) - diff(pred_raw)||) (m/帧, 用未对齐原始轨迹, 尺度无关)
  - 路径长度对比 (GT vs pred)
  - 终点误差 (对齐后)

输出: 控制台表格 + 追加写入 nlm_regress_log.csv (同目录)
"""
import argparse
import csv
import os
import sys

import numpy as np


def load_pos(path):
    # GT 文件: 7列(timestamp,x,y,z,r,p,y)或3列(x,y,z); exp_trajectory.txt 无表头3列。
    # 与 MATLAB read_ground_truth 口径一致: 取 x,y,z 位置列。
    with open(path) as f:
        first = f.readline().strip()
    skip = 0
    if first and not first[0].isdigit() and first[0] != '-':
        skip = 1
    a = np.loadtxt(path, delimiter=',', skiprows=skip)
    ncols = a.shape[1]
    if ncols >= 7:      # timestamp 开头 → 位置列 = 2..4 (1-based)
        return a[:, 1:4]
    return a[:, :3]


def simple_align(pred, gt):
    """MATLAB 'simple': 起点对齐 + scale=len_gt/len_pred (pred -> gt 系)"""
    pc = pred - pred[0]
    gc = gt - gt[0]
    len_p = float(np.sum(np.linalg.norm(np.diff(pc, axis=0), axis=1)))
    len_g = float(np.sum(np.linalg.norm(np.diff(gc, axis=0), axis=1)))
    scale = len_g / len_p if len_p > 0 else 1.0
    return pc * scale, gc, scale, len_p, len_g


def compute_ate(pred_aligned, gt_aligned):
    n = min(len(pred_aligned), len(gt_aligned))
    d = gt_aligned[:n] - pred_aligned[:n]
    return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))


def compute_rpe(gt, pred):
    n = min(len(gt), len(pred))
    if n < 2:
        return 0.0
    return float(np.mean(np.linalg.norm(np.diff(gt[:n], axis=0) - np.diff(pred[:n], axis=0), axis=1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset_dir')
    ap.add_argument('--pred', default='slam_results/exp_trajectory.txt')
    ap.add_argument('--gt', default='ground_truth.txt')
    ap.add_argument('--tag', default='')
    args = ap.parse_args()

    d = args.dataset_dir
    gt = load_pos(os.path.join(d, args.gt))
    pred = load_pos(os.path.join(d, args.pred))

    n = min(len(gt), len(pred))
    gt, pred = gt[:n], pred[:n]

    pa, ga, scale, len_p, len_g = simple_align(pred, gt)
    ate = compute_ate(pa, ga)
    rpe = compute_rpe(gt, pred)
    ate_end = float(np.linalg.norm(pa[-1] - ga[-1]))

    print(f'[{args.tag}] {os.path.basename(d.rstrip("/"))}')
    print(f'  帧数={n}  路径: GT={len_g:.1f}m pred={len_p:.1f}m (scale={scale:.4f})')
    print(f'  ATE={ate:.2f}m  RPE={rpe:.4f}m/f  终点误差={ate_end:.2f}m')

    # 追加日志
    log = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nlm_regress_log.csv')
    row = {'tag': args.tag, 'dataset': os.path.basename(d.rstrip('/')),
           'n': n, 'len_gt': round(len_g, 1), 'len_pred': round(len_p, 1),
           'scale': round(scale, 4), 'ate': round(ate, 2),
           'rpe': round(rpe, 4), 'end_err': round(ate_end, 2)}
    exists = os.path.exists(log)
    with open(log, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            w.writeheader()
        w.writerow(row)


if __name__ == '__main__':
    main()
