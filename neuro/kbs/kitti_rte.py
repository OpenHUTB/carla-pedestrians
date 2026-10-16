#!/usr/bin/env python3
"""kbs/kitti_rte.py — KITTI 官方相对平移误差 (RTE %) 侦察/计算 (缺口1<->缺口2 耦合证据)

目的:
  1) 在现有 KITTI 序列数据上算 EKF / NLM 的官方相对平移误差 (无全局对齐, 逐帧位移幅值
     误差按 GT 平均位移单一归一), 与官方排行榜直接可比;
  2) 对照 sim3_full 拟合尺度 s, 量化官方指标里全局尺度误差贡献的地板 (约 |s-1|) —
     "缺口1(外部对比)<->缺口2(全局尺度)" 耦合的核心证据: 不修全局尺度 (缺口2),
     单目系统在官方指标上会被公开方法碾压。

公式 (KITTI 官方相对平移误差, 相对量, 无对齐, 尺度敏感):
  RTE = mean_i | ||Δp_res,i|| - ||Δp_ref,i|| | / ( mean_i ||Δp_ref,i|| ) * 100%
  (按 GT 平均位移做单一归一, 避免近零位移帧把分母拖爆; 全局尺度偏置 s 贡献约 |s-1| 地板。)

用法:
  python kitti_rte.py [dataset_dir]
  默认 neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion

参考排行榜 (官方相对平移误差, 越低越好):
  SOFT2(st) 0.53%  SOFT-SLAM(st) 0.65%  VINS-Fusion(st) 1.09%  ORB-SLAM2(st) 1.15%
  MonoROCC 1.11%  MonoDepth2 12.59%   (激光 SOTA ~0.5%)
"""
import sys
import numpy as np, pandas as pd, scipy.io as sio, os


def loadcsv(base, fn, cols):
    p = base + fn
    if not os.path.exists(p):
        return None
    a = pd.read_csv(p)
    c = [x for x in cols if x in a.columns]
    return a[c].values.astype(float) if len(c) >= 3 else None


def loadmat(base, fn, var):
    p = base + fn
    if not os.path.exists(p):
        return None
    try:
        t = np.array(sio.loadmat(p)[var], dtype=float)
        return t[:, :3] if (t.ndim == 2 and t.shape[1] >= 3) else t
    except Exception:
        return None


def rte(res, ref):
    """官方相对平移误差 (无对齐, GT 平均位移单一归一)。返回 (RTE%, mean‖Δref‖, mean‖Δres‖)。"""
    n = min(len(res), len(ref)); res, ref = res[:n], ref[:n]
    dr = np.linalg.norm(np.diff(res, axis=0), axis=1)
    df = np.linalg.norm(np.diff(ref, axis=0), axis=1)
    denom = float(df.mean())
    if denom < 1e-9:
        return float('nan'), 0.0, 0.0
    return float(np.mean(np.abs(dr - df)) / denom * 100), float(df.mean()), float(dr.mean())


def sim3_scale(res, ref):
    """full Sim(3) 拟合的全局尺度因子 s。"""
    n = min(len(res), len(ref)); res, ref = res[:n], ref[:n]
    ec, gc = res - res.mean(0), ref - ref.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    return float((S @ np.array([1, 1, d])) / (ec ** 2).sum())


base = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion/'
if not base.endswith('/'):
    base += '/'
gt = loadcsv(base, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
assert gt is not None, f'GT missing: {base}ground_truth.txt'
L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
cfg = {
    'EKF': loadcsv(base, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z']),
    'NLM (规范)': loadmat(base, 'slam_results/trajectories.mat', 'exp_trajectory'),
    'NLM+尺度': loadmat(base, 'slam_results_scale_nlm/trajectories.mat', 'exp_trajectory'),
}
print(f'== {os.path.basename(base.rstrip("/"))}  GT {len(gt)} 帧, 长度 {L:.1f}m ==')
print(f"{'配置':14s} {'帧数':>7s} {'RTE(%)':>9s} {'|s-1|地板':>10s} {'sim3 s':>8s}   诊断(meanΔgt/meanΔres, m/帧)")
for name, traj in cfg.items():
    if traj is None:
        print(f'{name:14s}   (MISSING)')
        continue
    r, mref, mres = rte(traj, gt); s = sim3_scale(traj, gt)
    print(f'{name:14s} {len(traj):7d} {r:9.2f} {abs(s - 1) * 100:9.2f}% {s:8.4f}   '
          f'(gt {mref:.4f} / res {mres:.4f}, 比值 {mres / mref:.3f})')
print()
print('参考 (官方相对平移误差): SOFT2(st) 0.53% | SOFT-SLAM(st) 0.65% |')
print('  VINS-Fusion(st) 1.09% | ORB-SLAM2(st) 1.15% | MonoROCC 1.11% | MonoDepth2 12.59%')
print('  (激光 SOTA ~0.5%)')
