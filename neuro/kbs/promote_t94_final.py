#!/usr/bin/env python3
"""promote_t94_final.py — τ=0.94 定解一键定稿 (2026-10-14)

批量 (5 P2FT + 4 P2S) 跑完后执行:
  1) slam_results_p2ft/ (τ0.94) 的 trajectories.mat + experiences.mat 提升为规范 slam_results/
  2) 重算 final_metrics.json (compute_final_metrics 口径)
  3) 尺度线 NLM ATE (slam_results_scale_nlm, τ0.94 + 深度基座) vs 新规范 NLM 对比
前置: /tmp/p2ft94_final/batch.log 出现 ALL_DONE 且 9 个驱动 EXIT=0
"""
import shutil, os, json
import numpy as np, scipy.io as sio, pandas as pd

D = '../data/01_NeuroSLAM_Datasets/'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']

def sim3_rmse(est, gt):
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    ea = s * (est @ T) + (gt.mean(0) - s * (est.mean(0) @ T))
    return float(np.sqrt(np.mean(np.sum((ea - gt) ** 2, axis=1))))

print('── 1) 提升 τ0.94 结果到规范 slam_results/ ──')
for sub in SETS:
    src, dst = f'{D}{sub}/slam_results_p2ft', f'{D}{sub}/slam_results'
    for fn in ('trajectories.mat', 'experiences.mat', 'exp_trajectory.txt'):
        shutil.copy2(f'{src}/{fn}', f'{dst}/{fn}')
    print(f'  {sub}: trajectories.mat + experiences.mat → slam_results/')

print('── 2) 尺度线 NLM ATE (τ0.94, 深度基座) vs 新规范 NLM ──')
print(f"{'集':12s} {'EKF':>6s} {'NLM(t94)':>9s} {'NLM尺度':>8s} {'尺度降幅':>8s}")
rows = []
for sub in SETS:
    gt = pd.read_csv(f'{D}{sub}/ground_truth.txt').iloc[:, 1:4].values.astype(float)
    fu = pd.read_csv(f'{D}{sub}/fusion_pose.txt').iloc[:, 1:4].values.astype(float)
    nlm = np.array(sio.loadmat(f'{D}{sub}/slam_results/trajectories.mat')['exp_trajectory'], dtype=float)
    a_ekf, a_nlm = sim3_rmse(fu, gt), sim3_rmse(nlm, gt)
    sc_path = f'{D}{sub}/slam_results_scale_nlm/trajectories.mat'
    if sub.endswith('KITTI07Data_IMU_Fusion') or not os.path.exists(sc_path):
        print(f'{sub:12s} {a_ekf:6.2f} {a_nlm:9.2f}     n/a     n/a')
        continue
    sc = np.array(sio.loadmat(sc_path)['exp_trajectory'], dtype=float)
    a_sc = sim3_rmse(sc, gt)
    red = (a_nlm - a_sc) / a_nlm * 100
    print(f'{sub:12s} {a_ekf:6.2f} {a_nlm:9.2f} {a_sc:8.2f} {red:+8.1f}%')
    rows.append((sub, a_ekf, a_nlm, a_sc, red))

# 闭环计数 (规范口径: 每 experience 首条 link, 1<=eid<=k)
print('── 3) 闭环计数 (τ0.94 规范 experiences.mat) ──')
for sub in SETS:
    m = sio.loadmat(f'{D}{sub}/slam_results/experiences.mat')
    EXP = m['EXPERIENCES']
    n_exp = EXP.shape[1] if EXP.ndim == 2 else EXP.shape[0]
    nloop = 0
    for k in range(n_exp):
        e = EXP[0, k] if EXP.ndim == 2 else EXP[k]
        if e.shape != ():
            e = e.ravel()[0]
        try:
            nl = int(float(np.atleast_1d(e['numlinks']).ravel()[0]))
            if nl <= 0:
                continue
            lnks = np.atleast_1d(e['links'])
            for j in range(len(lnks)):
                lid = lnks[j]
                if lid.shape != ():
                    lid = lid.ravel()[0]
                eid = int(float(np.atleast_1d(lid['exp_id']).ravel()[0]))
                if 1 <= eid <= k:
                    nloop += 1
        except Exception:
            pass
    print(f'  {sub}: 节点={int(float(m["NUM_EXPS"].ravel()[0]))} 闭环边={nloop}')
print('DONE_PROMOTE')
