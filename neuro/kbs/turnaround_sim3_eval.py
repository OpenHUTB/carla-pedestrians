#!/usr/bin/env python3
"""折返集 A/B 换算到论文口径(首100帧锚定 Sim(3) 7-DoF Procrustes, 3D)。
复用 metrics_aligned.py 的对齐函数; 数据源 = MATLAB core 脚本已对齐好的
trajectories.mat (exp_traj_aligned / fusion_pos_aligned / pure_visual_aligned /
gt_pos_aligned), 保证与 Table 1 其余行完全同口径。
用法: python turnaround_sim3_eval.py
"""
import os
import sys

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, '/home/yangrb/桌面/NeuroSLAM四图一键_脚本')
from metrics_aligned import align_metric  # noqa: E402

BASE = '/home/yangrb/openhutb/neuro/data/Town01Turnaround_IMU_Fusion'


def main():
    print('=== 折返集(10000帧) 论文口径 Sim(3) 对齐 ATE ===')
    for label, sub, traj_var in [
        ('NLM(dc基线)', 'dc_final', 'exp_traj_aligned'),
        ('NLM(+闭环约束)', 'a_final', 'exp_traj_aligned'),
        ('EKF', 'a_final', 'fusion_pos_aligned'),
        ('VO', 'a_final', 'pure_visual_aligned'),
    ]:
        mat = loadmat(os.path.join(BASE, sub, 'trajectories.mat'))
        traj = np.asarray(mat[traj_var], dtype=float)
        gt = np.asarray(mat['gt_pos_aligned'], dtype=float)
        # gt_pos_aligned 是GT自己对齐到自己=恒等, 直接用
        rmse, final, drift, L = align_metric(traj, gt)
        print(f'{label:14s} RMSE={rmse:8.2f}m  final={final:8.2f}m  '
              f'drift={drift:6.2f}%  (gt_len={L:.0f}m)')


if __name__ == '__main__':
    main()
