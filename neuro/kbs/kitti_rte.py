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


def procrustes_7dof(gt, est):
    """管线 7-DoF Procrustes (与 regen_table.py 同口径): 把 est 映射到 gt 坐标系。
    返回 (对齐后 est, 尺度 s, 逐帧误差)。ATE=RMSE(逐帧误差) 应与 final_metrics 主表一致。"""
    n = min(len(gt), len(est)); gt, est = gt[:n], est[:n]
    gs = gt - gt.mean(0); es = est - est.mean(0)
    U, S, Vt = np.linalg.svd(gs.T @ es)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = S.sum() / max(np.sum(es ** 2), 1e-12)
    t = gt.mean(0) - (s * R) @ est.mean(0)
    est_a = (s * (R @ est.T)).T + t
    return est_a, s, np.linalg.norm(est_a - gt, axis=1)


def per_frame_err(res, ref):
    """逐帧位移幅值误差 |‖Δres‖-‖Δref‖| 与 GT 逐帧位移 (官方 RTE 分子/分母)。"""
    n = min(len(res), len(ref))
    dr = np.linalg.norm(np.diff(res[:n], axis=0), axis=1)
    df = np.linalg.norm(np.diff(ref[:n], axis=0), axis=1)
    return np.abs(dr - df), df


def rte_excl(e, df, k):
    """剔除误差最大的 k 个帧后的 RTE%。"""
    m = np.ones(len(e), bool); m[np.argsort(e)[::-1][:k]] = False
    return float(e[m].mean() / df[m].mean() * 100)


base = sys.argv[1] if len(sys.argv) > 1 else \
    '../data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion/'
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
print(f"{'配置':14s} {'帧数':>7s} {'RTE%(无对齐)':>13s} {'RTE%(Sim3对齐)':>15s} {'ATE(Sim3)m':>11s} {'sim3 s':>8s}")
for name, traj in cfg.items():
    if traj is None:
        print(f'{name:14s}   (MISSING)')
        continue
    r, mref, mres = rte(traj, gt); s = sim3_scale(traj, gt)
    est_a, s7, d7 = procrustes_7dof(gt, traj)
    ate = float(np.sqrt((d7 ** 2).mean())); r7, _, _ = rte(est_a, gt)
    print(f'{name:14s} {len(traj):7d} {r:13.2f} {r7:15.2f} {ate:11.2f} {s7:8.4f}   '
          f'(meanΔgt {mref:.4f} / meanΔres {mres:.4f}, 比值 {mres / mref:.3f})')

print()
print('== 对齐后 RTE 离群帧诊断 (剔除误差最大 top-k 帧; 判定系统性逐帧漂移 vs 少量脏帧) ==')
for name, traj in cfg.items():
    if traj is None:
        continue
    est_a, _, _ = procrustes_7dof(gt, traj)
    e, df = per_frame_err(est_a, gt)
    full = float(e.mean() / df.mean() * 100)
    excl = '  '.join(f'剔top-{k}={rte_excl(e, df, k):6.2f}%' for k in (5, 10, 20, 50))
    print(f'{name:14s} 完整={full:7.2f}%  最大单帧={e.max():7.1f}m  {excl}')
print()
print('参考 (官方相对平移误差): SOFT2(st) 0.53% | SOFT-SLAM(st) 0.65% |')
print('  VINS-Fusion(st) 1.09% | ORB-SLAM2(st) 1.15% | MonoROCC 1.11% | MonoDepth2 12.59%')
print('  (激光 SOTA ~0.5%)')
