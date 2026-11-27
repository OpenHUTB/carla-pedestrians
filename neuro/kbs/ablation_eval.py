#!/usr/bin/env python3
"""ablation_eval.py — 脑启发模块单关消融评测 (GC/HD/腹侧流)
对 5 场景 × 3 模式(nogc/nohdc/novt)的 abl_<mode>/exp_trajectory.txt
与 ground_truth.txt 做 Sim(3) full-refit 与 SE(3) rigid ATE, 复用 se3_metrics 口径。
用法: python3 ablation_eval.py  (NLM_DATA_ROOT 可覆盖数据根)
"""
import os
import numpy as np

# 内联 se3_metrics 的两个对齐函数(该脚本导入即执行主表, 不能直接 import)
ROOT = os.environ.get('NLM_DATA_ROOT', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', '01_NeuroSLAM_Datasets'))


def se3_rigid(est, gt):
    """SE(3) 刚性对齐 (Kabsch, 无尺度)。返回 (rmse, drift%)。"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    ea = est @ R + (gt.mean(0) - (est @ R).mean(0))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100)


def sim3_full(est, gt):
    """full Sim(3) 对齐 (旋转+全局尺度+平移)。返回 (rmse, drift%, 拟合尺度 s)。"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    ea = s * (est @ T) + (gt.mean(0) - s * (est.mean(0) @ T))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100), float(s)
SCEN = ['Town05', 'Town03', 'Town07', 'Town10HD', 'KITTI07']
SHORT = {'Town05': 'Town05', 'Town03': 'Town03', 'Town07': 'Town07', 'Town10HD': 'Town10HD', 'KITTI07': 'KITTI07'}
MODES = ['nogc', 'nohdc']   # novt(腹侧流关闭)≡NLM-loopline 行, 见 Table 4
MODE_CN = {'nogc': 'w/o 网格细胞', 'nohdc': 'w/o 头方向细胞', 'novt': 'w/o 腹侧模板流'}
# 全系统基线: 论文 NLM-fullsystem 轨迹(slam_results_scale_nlm); 缺失时回退同批 abl_full
BASE = {s: 'slam_results_scale_nlm' for s in SCEN}


def loadtraj(p):
    """优先读 trajectories.mat 的 exp_trajectory(与 se3_metrics 口径一致), 回退 txt。"""
    import scipy.io as sio
    mp = os.path.join(os.path.dirname(p), 'trajectories.mat')
    if os.path.exists(mp):
        try:
            a = np.array(sio.loadmat(mp)['exp_trajectory'], dtype=float)
            if a.ndim == 2 and a.shape[1] >= 3:
                return a[:, :3]
        except Exception:
            pass
    if not os.path.exists(p):
        return None
    try:
        a = np.loadtxt(p, delimiter=',')
        if a.ndim != 2 or a.shape[1] < 3:
            a = np.loadtxt(p)
        return np.atleast_2d(a)[:, :3]
    except Exception:
        pass
    try:
        import pandas as pd
        df = pd.read_csv(p)
        cols = [c for c in ('pos_x', 'pos_y', 'pos_z') if c in df.columns]
        if len(cols) == 3:
            return df[cols].values.astype(float)
        a = np.genfromtxt(p, delimiter=',', skip_header=1)
        return np.atleast_2d(a)[:, :3]
    except Exception:
        return None


def main():
    print('=' * 72)
    print('脑启发模块单关消融 — Sim(3) full-refit / SE(3) rigid ATE (RMSE, m)')
    print('=' * 72)
    hdr = '%-10s %-16s %10s %10s' % ('场景', '配置', 'Sim(3)', 'SE(3)')
    print(hdr)
    print('-' * 72)
    for scen in SCEN:
        d = os.path.join(ROOT, scen + 'Data_IMU_Fusion')
        gt = loadtraj(os.path.join(d, 'ground_truth.txt'))
        # 全系统基线行
        base_dir = os.path.join(d, BASE.get(scen, 'abl_full'))
        est = loadtraj(os.path.join(base_dir, 'exp_trajectory.txt'))
        if est is None:
            est = loadtraj(os.path.join(d, 'abl_full', 'exp_trajectory.txt'))
        if est is not None and gt is not None:
            r3 = sim3_full(est, gt)[0]
            r4 = se3_rigid(est, gt)[0]
            print('%-10s %-16s %10.2f %10.2f' % (SHORT[scen], 'full system', r3, r4))
        else:
            print('%-10s %-16s %10s %10s' % (SHORT[scen], 'full system', '缺', '缺'))
        # 三个单关模式
        for mode in MODES:
            est = loadtraj(os.path.join(d, 'abl_' + mode, 'exp_trajectory.txt'))
            if est is None or gt is None:
                print('%-10s %-16s %10s %10s' % (SHORT[scen], MODE_CN[mode], 'MISS', 'MISS'))
                continue
            r3 = sim3_full(est, gt)[0]
            r4 = se3_rigid(est, gt)[0]
            print('%-10s %-16s %10.2f %10.2f' % (SHORT[scen], MODE_CN[mode], r3, r4))
        print('-' * 72)


if __name__ == '__main__':
    main()
