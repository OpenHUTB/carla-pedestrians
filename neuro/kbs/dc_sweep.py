#!/usr/bin/env python3
"""dc_sweep.py — 分布式闭环漂移场(DC v3)离线扫参, 7-DoF Procrustes 口径(论文Table口径).

方法(与 core DC v3 注入块逐行同构):
  输出 = base + (d - C_cum)
  C_cum[i] = Σ_{fi<=i} c_i                      重锚定阶梯
  d[i]     = Σ_i c_i * F_i(i-fi), F_i=0.5*(1-cos(pi*t/Wi)) (t<Wi), 之后=1
  Wi = clip(W0 + W_PER_M*|c_i|, W0, W_MAX)      自适应松弛时程
  Wi = min(Wi, n-fi+1)                           末端自然终止

用法:
  python3 dc_sweep.py Town01            # 单数据集扫 W0×W_PER_M
  python3 dc_sweep.py all               # 5 数据集(需 dc_pass1 事件齐全)
  python3 dc_sweep.py Town01 --W0 2400 --WP 30   # 单点评估(验证用)
"""
import sys, os
import numpy as np, scipy.io as sio
exec(open('/home/yangrb/openhutb/neuro/kbs/regen_table.py').read().split("# ---------- 口径验证")[0])
ROOT = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
DSS = [('Town01Data_IMU_Fusion', 'Town01', 5000), ('Town02Data_IMU_Fusion', 'Town02', 5000),
       ('Town05Data_IMU_Fusion', 'Town05', 5000), ('Town10HDData_IMU_Fusion', 'Town10HD', 5000),
       ('KITTI07Data_IMU_Fusion', 'KITTI07', 1101)]


def load_events(ds_dir):
    p = f'{ROOT}/{ds_dir}/dc_pass1/dc_events.mat'
    if not os.path.exists(p):
        return None
    a = sio.loadmat(p)
    k = 'NLM_DC_EVENTS' if 'NLM_DC_EVENTS' in a else 'dc_events'
    ev = a[k]
    return [(int(r[0]), np.asarray(r[1:4], float)) for r in ev]


def dc_inject(base, ev, n, W0, WP, WMAX, tail=True, cmax=np.inf):
    """cmax: 幅值门控 — |c|>cmax 的跳变不摊平(保持瞬时), 仅小跳变进漂移场."""
    f0 = np.arange(1, n + 1)
    C = np.zeros((n, 3)); d = np.zeros((n, 3))
    for fi, c in ev:
        if np.linalg.norm(c) > cmax:
            continue
        Wi = W0 + WP * np.linalg.norm(c)
        Wi = max(W0, min(Wi, WMAX))
        if tail:
            Wi = min(Wi, n - fi + 1)
        C[f0 >= fi] += c
        t = f0 - fi
        r = (t >= 0) & (t < Wi)
        if r.any():
            F = 0.5 * (1 - np.cos(np.pi * t[r] / Wi))
            d[r] += np.outer(F, c)
        h = f0 >= (fi + Wi)
        d[h] += c
    return base + (d - C)


def score(gt, est):
    r, t, s = metrics(gt, est)  # 7-DoF 全轨迹
    d = np.abs(np.diff(est, axis=0)).mean(1)
    rpe = float(d.mean())
    return r, t, rpe, s


def run_one(ds_dir, key, W0s, WPs):
    gt = load_pos(f'{ROOT}/{ds_dir}/ground_truth.txt')
    base = load_pos(f'{ROOT}/{ds_dir}/slam_results/exp_trajectory.txt')
    ekf = load_pos(f'{ROOT}/{ds_dir}/fusion_pose.txt')
    ev = load_events(ds_dir)
    n = len(base)
    rb, tb, rpeb, sb = score(gt, base)
    re, te, rpee, se = score(gt, ekf)
    print(f'=== {key}  n={n} 事件={0 if ev is None else len(ev)}  '
          f'基线ATE={rb:.2f} (终点{tb:.2f} RPE{rpeb:.3f})  EKF={re:.2f}')
    if ev is None:
        print('  ⚠ 无 dc_pass1/dc_events.mat, 跳过扫参')
        return None
    rows = []
    CMAXS = [np.inf, 6.0, 4.0, 3.0, 2.0]
    for W0 in W0s:
        for WP in WPs:
            for cm in CMAXS:
                out = dc_inject(base, ev, n, W0, WP, 4000, tail=True, cmax=cm)
                r, t, rpe, s = score(gt, out)
                rows.append((r, W0, WP, cm, t, rpe))
    rows.sort()
    print(f'  {"W0":>5} {"WP":>4} {"Cmax":>5}  {"ATE":>7} {"终点":>7} {"RPE":>7}  ΔATE%')
    for r, W0, WP, cm, t, rpe in rows[:10]:
        cms = 'inf' if cm == np.inf else f'{cm:g}'
        print(f'  {W0:>5} {WP:>4} {cms:>5}  {r:7.2f} {t:7.2f} {rpe:7.3f}  {(r-rb)/rb*100:+6.2f}%')
    best = rows[0]
    cms = 'inf' if best[3] == np.inf else f'{best[3]:g}'
    print(f'  BEST: W0={best[1]} WP={best[2]} Cmax={cms} ATE={best[0]:.2f} ({(best[0]-rb)/rb*100:+.2f}%) '
          f'vs EKF {re:.2f} → {"胜出" if best[0] < re else "未胜"}')
    # 最优配置逐帧误差曲线落盘(procrustes_7dof 返回逐帧误差数组)
    out = dc_inject(base, ev, n, best[1], best[2], 4000, tail=True, cmax=best[3])
    en, _ = procrustes_7dof(gt, out)
    ee, _ = procrustes_7dof(gt, ekf)
    nn = min(len(en), len(ee))
    np.savez(f'/tmp/dc_{key}.npz', frame=np.arange(nn), nlm=en[:nn], ekf=ee[:nn])
    return {'ds': key, 'base': rb, 'best_ate': best[0], 'W0': best[1], 'WP': best[2],
            'cm': 0 if best[3] == np.inf else best[3], 'ekf': re}


if __name__ == '__main__':
    args = sys.argv[1:]
    single_W0 = single_WP = None
    if '--W0' in args:
        i = args.index('--W0'); single_W0 = int(args[i+1]); args = args[:i] + args[i+2:]
    if '--WP' in args:
        i = args.index('--WP'); single_WP = int(args[i+1]); args = args[:i] + args[i+2:]
    which = args[0] if args else 'all'
    if single_W0 is not None:
        W0s, WPs = [single_W0], [single_WP or 0]
    else:
        W0s = [200, 400, 800, 1200, 1600, 2400]
        WPs = [0, 10, 20, 30, 50, 80]
    results = []
    if which == 'all':
        for ds_dir, key, _ in DSS:
            r = run_one(ds_dir, key, W0s, WPs)
            if r: results.append(r)
    else:
        for ds_dir, key, _ in DSS:
            if key == which:
                r = run_one(ds_dir, key, W0s, WPs)
                if r: results.append(r)
    if results:
        print('\n=== 汇总 ===')
        for r in results:
            print(f'{r["ds"]:8s} base={r["base"]:7.2f} → best={r["best_ate"]:7.2f} '
                  f'(W0={r["W0"]}, WP={r["WP"]})  EKF={r["ekf"]:7.2f}')
        np.savez('/tmp/dc_sweep_summary.npz',
                 base=np.array([r['base'] for r in results]),
                 best=np.array([r['best_ate'] for r in results]),
                 ekf=np.array([r['ekf'] for r in results]),
                 W0=np.array([r['W0'] for r in results], dtype=int),
                 WP=np.array([r['WP'] for r in results], dtype=int),
                 cm=np.array([r['cm'] for r in results]))
