#!/usr/bin/env python3
"""kbs/regen_ekf_curves.py — 重绘 4 张 EKF 曲线图 (Town05/03/07/10HD, 2026-10-12 数据集换集)

数据集: Town05 / Town03 / Town07 / Town10HD (全 CARLA 重采数据; MH_03/KITTI07
曲线随 EuRoC 数据集退出论文而移除, KITTI07 改由 KITTI_07_input_output 图承担
现实数据验证)。
样式与旧图一致: 对齐后逐帧 3D 位置误差 vs GT 弧长, NLM红实线 / EKF蓝虚线 / VO黑点线, 图例带 ATE。
口径: 全轨迹 7-DoF Sim(3) Umeyama 对齐 (与论文 Table 1 / compute_final_metrics.py 一致)。
数据源 (各集同构):
  TownNN   GT/fusion/visual_odometry (有头 CSV) + slam_results/trajectories.mat:exp_trajectory
注: 2026-10-09 曾尝试把 CARLA #100 形态 (EKF_VIO 10维 log尺度状态 + 转弯运动学
    尺度观测) 移植进独立 EKF 生成器, 重生成实测均有害, 已回滚 (生成器回
    7f9bbb1 版本); CARLA 侧 10-09 恢复只动 IMU_Vision_Fusion_EKF.py。
"""
import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D = '../data/'
OUT = '../kbs/fig/'

def procrustes_full(est, gt, tag=''):
    est = np.asarray(est, dtype=float); gt = np.asarray(gt, dtype=float)
    if est.ndim != 2 or gt.ndim != 2 or est.shape[1] != 3 or gt.shape[1] != 3:
        raise ValueError(f'{tag}: 形状异常 est={est.shape} gt={gt.shape}')
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    t = gt.mean(0) - s * (est.mean(0) @ T)
    return s * (est @ T) + t

def curve(gt, est):
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]
    ea = procrustes_full(est, gt)
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
    err = np.linalg.norm(ea - gt, axis=1)
    ate = float(np.sqrt(np.mean(err ** 2)))
    return arc, err, ate

def load_csv(p, cols, header=0):
    a = pd.read_csv(p, header=header)
    c = [x for x in cols if x in a.columns]
    return a[c].values

def plot_set(name, title, gt, nlm, ekf, vo):
    fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=150)
    specs = [('NeuroLocMap', nlm, 'r-', 1.4),
             ('EKF fusion', ekf, 'b--', 1.4),
             ('Pure VO', vo, 'k:', 1.2)]
    labels = []
    for lab, tr, style, lw in specs:
        arc, err, ate = curve(gt, tr)
        ax.plot(arc, err, style, lw=lw, label=f'{lab} (ATE {ate:.1f}m)')
    ax.set_xlabel('Ground-truth arc length (m)')
    ax.set_ylabel('Position error (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.4)
    ax.legend(loc='upper left', framealpha=0.9)
    fig.tight_layout()
    out = f'{OUT}ekf_curve_{name}.pdf'
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)
    print(f'saved {out}')

def main():
    for town in ['Town05', 'Town03', 'Town07', 'Town10HD']:
        base = D + f'01_NeuroSLAM_Datasets/{town}Data_IMU_Fusion/'
        gt = load_csv(base + 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
        ekf = load_csv(base + 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'])
        vo = load_csv(base + 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z'])
        nlm = np.array(sio.loadmat(base + 'slam_results/trajectories.mat')['exp_trajectory'], dtype=float)
        plot_set(town, f'{town}  NeuroLocMap vs EKF vs VO', gt, nlm, ekf, vo)

    print('ALL DONE')

if __name__ == '__main__':
    main()
