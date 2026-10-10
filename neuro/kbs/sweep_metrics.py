#!/usr/bin/env python3
"""kbs/sweep_metrics.py — 汇总 /tmp/nlm_sweep_newfoot 各 (VT,CE) 组合的 ATE
口径与 regen_ekf_curves.py 一致: 全轨迹 7-DoF Sim(3) Umeyama 对齐后逐帧 3D RMSE。
用法: python3 kbs/sweep_metrics.py
"""
import glob, os
import numpy as np
import pandas as pd

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/'

SETS = {
    'Town01Data_IMU_Fusion': 'pos_x',
    'Town10HDData_IMU_Fusion': 'pos_x',
}
EKF = {  # 恢复EKF地基后的 ATE 基准 (regen_ekf_curves.py 输出)
    'Town01Data_IMU_Fusion': 14.3,
    'Town10HDData_IMU_Fusion': 13.4,
}

def procrustes_full(est, gt):
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    t = gt.mean(0) - s * (est.mean(0) @ T)
    return s * (est @ T) + t

def ate(gt, est):
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]
    ea = procrustes_full(est, gt)
    err = np.linalg.norm(ea - gt, axis=1)
    return float(np.sqrt(np.mean(err ** 2)))

def main():
    for ds in SETS:
        gtp = f'{D}{ds}/ground_truth.txt'
        if not os.path.exists(gtp):
            continue
        gt = pd.read_csv(gtp)[['pos_x', 'pos_y', 'pos_z']].values
        print(f'\n══ {ds}  (EKF基线 ATE={EKF[ds]}m) ══')
        rows = []
        for d in sorted(glob.glob(f'/tmp/nlm_sweep_newfoot/{ds}/vt*')):
            f = os.path.join(d, 'exp_trajectory.txt')
            if not os.path.exists(f):
                continue
            a = np.asarray(pd.read_csv(f, header=None))
            est = a[:, 1:4] if a.shape[1] >= 4 else a[:, :3]
            try:
                m = ate(gt, est)
            except Exception as e:
                m = float('nan')
            rows.append((os.path.basename(d), m))
            print(f'  {os.path.basename(d):16s} ATE={m:8.3f}m')
        rows.sort(key=lambda x: x[1])
        if rows:
            print(f'  >> 最优: {rows[0][0]}  ATE={rows[0][1]:.3f}m  (EKF={EKF[ds]})')

if __name__ == '__main__':
    main()
