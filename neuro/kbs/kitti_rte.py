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


def compute(base):
    """Return the Table-6 numbers for one KITTI sequence directory."""
    if not base.endswith('/'):
        base += '/'
    gt = loadcsv(base, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
    assert gt is not None, f'GT missing: {base}ground_truth.txt'
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    cfg = {
        'EKF': loadcsv(base, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z']),
        'NLM': loadmat(base, 'slam_results/trajectories.mat', 'exp_trajectory'),
        'NLM+scale': loadmat(base, 'slam_results_scale_nlm/trajectories.mat', 'exp_trajectory'),
    }
    out = {'gt_len_m': round(L, 1), 'base': base}
    for name, traj in cfg.items():
        if traj is None:
            out[name] = None
            continue
        r, mref, mres = rte(traj, gt)
        est_a, s7, d7 = procrustes_7dof(gt, traj)
        ate = float(np.sqrt((d7 ** 2).mean())); r7, _, _ = rte(est_a, gt)
        e, df = per_frame_err(est_a, gt)
        out[name] = {'frames': int(len(traj)),
                     'rte_raw': round(r, 2), 'rte_aligned': round(r7, 2),
                     'ate': round(ate, 2), 's': round(s7, 4),
                     'rte_top50': round(rte_excl(e, df, 50), 2)}
    return out


HERE = os.path.dirname(os.path.abspath(__file__))


def default_root():
    return os.environ.get('NLM_DATA_ROOT',
                          os.path.join(HERE, '..', 'data', '01_NeuroSLAM_Datasets'))


def main(base=None):
    base = base or (os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else
                    os.path.join(default_root(), 'KITTI07Data_IMU_Fusion'))
    d = compute(base)
    print(f'== {os.path.basename(d["base"].rstrip("/"))}  GT {d["gt_len_m"]} m ==')
    print(f"{'config':10s} {'frames':>7s} {'RTE% raw':>10s} {'RTE% aligned':>13s} {'ATE m':>8s} {'sim3 s':>8s}")
    for name in ('EKF', 'NLM', 'NLM+scale'):
        v = d.get(name)
        if v is None:
            print(f'{name:10s}   (MISSING)')
        else:
            print(f'{name:10s} {v["frames"]:7d} {v["rte_raw"]:10.2f} {v["rte_aligned"]:13.2f} '
                  f'{v["ate"]:8.2f} {v["s"]:8.4f}   (RTE after dropping top-50 frames: {v["rte_top50"]:.2f}%)')
    print()
    print('Reference (official KITTI relative translation error, %): '
          'SOFT2 0.53 | SOFT-SLAM 0.65 | VINS-Fusion 1.09 | ORB-SLAM2 1.15 |'
          ' MonoDepth2 12.59')


if __name__ == '__main__':
    main()
