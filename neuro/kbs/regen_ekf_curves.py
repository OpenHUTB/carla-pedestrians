#!/usr/bin/env python3
"""kbs/regen_ekf_curves.py — 重绘 4 张 EKF 曲线图 (01/03/07/10)

数据集换为 Town01 / MH_03 / KITTI07 / Town10HD。
样式与旧图一致: 对齐后逐帧 3D 位置误差 vs GT 弧长, NLM红实线 / EKF蓝虚线 / VO黑点线, 图例带 ATE。
口径: 全轨迹 7-DoF Sim(3) Umeyama 对齐 (与论文 Table 1 / compute_metrics_with_alignment 活跃版一致)。

数据源 (各集已核实 10-09/在盘):
  Town01   GT/fusion/visual_odometry + slam_results/trajectories.mat:exp_trajectory
  MH_03    GT(EuRoC无头格式) + fusion_pose_vo_ekf.txt(无头, col1:4=xyz)
           + euroc_trajectories.mat: pure_visual_traj / exp_trajectory
  KITTI07  GT/fusion_pose + ../KITTI_07/visual_odometry_ekf.txt(有头 vo_x/y/z)
           + slam_results/trajectories.mat:exp_trajectory
  Town10HD 同 Town01
注: MH_03/KITTI07 的 EKF 轨迹仍为恢复EKF前的旧地基 (10-03/09-30), 重跑后需重跑本脚本。
"""
import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D = '/home/yangrb/openhutb/neuro/data/'
OUT = '/home/yangrb/openhutb/neuro/kbs/fig/'

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
    # ── Town01 ──
    gt = load_csv(D + '01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/ground_truth.txt',
                  ['pos_x', 'pos_y', 'pos_z'])
    ekf = load_csv(D + '01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/fusion_pose.txt',
                   ['pos_x', 'pos_y', 'pos_z'])
    vo = load_csv(D + '01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/visual_odometry.txt',
                  ['vo_x', 'vo_y', 'vo_z'])
    nlm = np.array(sio.loadmat(D + '01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results/trajectories.mat')['exp_trajectory'], dtype=float)
    plot_set('Town01', 'Town01  NeuroLocMap vs EKF vs VO', gt, nlm, ekf, vo)

    # ── Town10HD ──
    gt = load_csv(D + '01_NeuroSLAM_Datasets/Town10HDData_IMU_Fusion/ground_truth.txt',
                  ['pos_x', 'pos_y', 'pos_z'])
    ekf = load_csv(D + '01_NeuroSLAM_Datasets/Town10HDData_IMU_Fusion/fusion_pose.txt',
                   ['pos_x', 'pos_y', 'pos_z'])
    vo = load_csv(D + '01_NeuroSLAM_Datasets/Town10HDData_IMU_Fusion/visual_odometry.txt',
                  ['vo_x', 'vo_y', 'vo_z'])
    nlm = np.array(sio.loadmat(D + '01_NeuroSLAM_Datasets/Town10HDData_IMU_Fusion/slam_results/trajectories.mat')['exp_trajectory'], dtype=float)
    plot_set('Town10HD', 'Town10HD  NeuroLocMap vs EKF vs VO', gt, nlm, ekf, vo)

    # ── MH_03 (EuRoC) ──
    gt = np.asarray(pd.read_csv(D + '02_EuRoc_Dataset/MH_03_medium/ground_truth.txt', header=None))[:, 1:4]
    ekf = pd.read_csv(D + '02_EuRoc_Dataset/MH_03_medium/fusion_pose_vo_ekf.txt', header=None).values
    ekf = ekf[:, 1:4]
    m = sio.loadmat(D + '02_EuRoc_Dataset/MH_03_medium/slam_results/euroc_trajectories.mat')
    vo = np.array(m['pure_visual_traj'], dtype=float)
    nlm = np.array(m['exp_trajectory'], dtype=float)
    plot_set('MH_03', 'EuRoC MH_03  NeuroLocMap vs EKF vs VO', gt, nlm, ekf, vo)

    # ── KITTI07 ──
    gt = load_csv(D + '01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion/ground_truth.txt',
                  ['pos_x', 'pos_y', 'pos_z'])
    ekf = load_csv(D + '01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion/fusion_pose.txt',
                   ['pos_x', 'pos_y', 'pos_z'])
    vo = load_csv(D + 'KITTI_07/visual_odometry_ekf.txt', ['vo_x', 'vo_y', 'vo_z'])
    nlm = np.array(sio.loadmat(D + '01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion/slam_results/trajectories.mat')['exp_trajectory'], dtype=float)
    plot_set('KITTI07', 'KITTI 07  NeuroLocMap vs EKF vs VO', gt, nlm, ekf, vo)

    print('ALL DONE')

if __name__ == '__main__':
    main()
