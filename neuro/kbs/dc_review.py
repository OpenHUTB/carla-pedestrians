#!/usr/bin/env python3
"""dc_review.py — NLM DC v3 审阅材料: 5集逐帧误差对比图(base NLM / DC v3 / EKF) + 指标汇总.
对齐口径: 7-DoF Procrustes(与论文Table/ regen_table 一致).
输出:
  /home/yangrb/桌面/NLM_DCv3_review/fig_perframe_<key>.png   (逐帧误差, 2x2: 误差+末段放大+轨迹俯视)
  /home/yangrb/桌面/NLM_DCv3_review/summary.csv              (指标汇总)
"""
import numpy as np, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
exec(open('/home/yangrb/openhutb/neuro/kbs/regen_table.py').read().split("# ---------- 口径验证")[0])
ROOT = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
OUT = '/home/yangrb/桌面/NLM_DCv3_review'
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({'font.size': 9, 'figure.dpi': 130})

def align7(A, B, anchor=None):
    """对齐 B 到 A(7-DoF), 返回对齐后的 B (n,3)."""
    n = min(len(A), len(B)); A, B = A[:n], B[:n]
    a = n if anchor is None else min(anchor, n)
    ga = A[:a] - A[:a].mean(0); eb = B[:a] - B[:a].mean(0)
    H = ga.T @ eb
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = S.sum() / max(np.sum(eb**2), 1e-12)
    t = A[:a].mean(0) - (s * R) @ B[:a].mean(0)
    return (s * (R @ B.T)).T + t

def procr7(A, B, anchor=None):
    """对齐 B 到 A, 返回逐帧误差(全轨迹7-DoF)."""
    n = min(len(A), len(B))
    Ba = align7(A, B, anchor)
    return np.linalg.norm(Ba - A[:n], axis=1)

DSS = [('Town01Data_IMU_Fusion', 'Town01'), ('Town02Data_IMU_Fusion', 'Town02'),
       ('Town05Data_IMU_Fusion', 'Town05'), ('Town10HDData_IMU_Fusion', 'Town10HD'),
       ('KITTI07Data_IMU_Fusion', 'KITTI07')]
rows = []
for ds_dir, key in DSS:
    gt = load_pos(f'{ROOT}/{ds_dir}/ground_truth.txt')
    base = load_pos(f'{ROOT}/{ds_dir}/slam_results/exp_trajectory.txt')
    dc = load_pos(f'{ROOT}/{ds_dir}/dc_final/exp_trajectory.txt')
    ekf = load_pos(f'{ROOT}/{ds_dir}/fusion_pose.txt')
    eb = procr7(gt, base); ed = procr7(gt, dc); ee = procr7(gt, ekf)
    n = min(len(eb), len(ed), len(ee))
    eb, ed, ee = eb[:n], ed[:n], ee[:n]
    L = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
    def rmse(e): return float(np.sqrt((e**2).mean()))
    def drift(e): return float(e[-1] / L * 100)
    rows.append([key, n, round(L, 1),
                 round(rmse(eb), 2), round(rmse(ed), 2), round(rmse(ee), 2),
                 round(drift(eb), 2), round(drift(ed), 2), round(drift(ee), 2),
                 round((rmse(ee) - rmse(ed)) / rmse(ee) * 100, 2),
                 round(float((ed < eb).mean()) * 100, 1)])

    # ---- 2x2 图 ----
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    f = np.arange(n)
    ax = axes[0, 0]
    ax.semilogy(f, eb, color='0.55', lw=0.9, label='NLM base (staircase re-anchor)')
    ax.semilogy(f, ed, color='tab:red', lw=1.1, label='NLM + DC drift field')
    ax.semilogy(f, ee, color='tab:blue', lw=1.1, label='EKF (IMU-visual)')
    ax.set_yscale('log'); ax.set_xlabel('frame'); ax.set_ylabel('aligned per-frame error (m)')
    ax.set_title(f'{key}: per-frame error (7-DoF aligned, log)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which='both')
    ax = axes[0, 1]
    sl = slice(max(0, n - n // 5), n)
    ax.semilogy(f[sl], eb[sl], color='0.55', lw=1.0, label='NLM base')
    ax.semilogy(f[sl], ed[sl], color='tab:red', lw=1.2, label='NLM + DC')
    ax.semilogy(f[sl], ee[sl], color='tab:blue', lw=1.0, label='EKF')
    ax.set_yscale('log'); ax.set_xlabel('frame'); ax.set_ylabel('error (m)')
    ax.set_title(f'last 20% (frames {sl.start}-{n}): where DC acts'); ax.legend(fontsize=8); ax.grid(alpha=0.3, which='both')
    # 轨迹俯视: GT vs base vs dc
    ax = axes[1, 0]
    m = n
    ax.plot(gt[:m, 0], gt[:m, 1], 'k', lw=1.0, label='GT')
    bA = align7(gt, base); dA = align7(gt, dc)
    ax.plot(bA[:m, 0], bA[:m, 1], color='0.55', lw=0.7, alpha=0.8, label='NLM base')
    ax.plot(dA[:m, 0], dA[:m, 1], color='tab:red', lw=1.2, label='NLM + DC')
    ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('top view (aligned)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    # base→dc 修正向量场(每200帧抽一个事件区放大): 画 base 与 dc 差异
    ax = axes[1, 1]
    diff = np.linalg.norm(dc - base, axis=1)[:n]
    ax.plot(f, diff, color='tab:green', lw=0.9)
    ax.set_xlabel('frame'); ax.set_ylabel('|DC correction| (m)')
    ax.set_title('distributed drift-field correction magnitude (vs base)')
    ax.grid(alpha=0.3)
    fig.suptitle(f'NLM DC v3 review — {key}  (ATE: base {rmse(eb):.2f} → DC {rmse(ed):.2f}, EKF {rmse(ee):.2f} m)',
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(f'{OUT}/fig_perframe_{key}.png')
    plt.close(fig)

hdr = 'dataset,n_frames,L_gt_m,ATE_base,ATE_dc,ATE_ekf,drift_base%,drift_dc%,drift_ekf%,dc_gain_vs_ekf%,dc_better_frames%'
with open(f'{OUT}/summary.csv', 'w') as fh:
    fh.write(hdr + '\n')
    for r in rows:
        fh.write(','.join(str(x) for x in r) + '\n')
    fh.write('\n# dc_gain_vs_ekf% = (ATE_ekf-ATE_dc)/ATE_ekf*100  (正=DC更优)\n')
    fh.write('# dc_better_frames% = 逐帧DC误差<base误差的帧占比\n')
print('=== DC v3 审阅指标汇总 (7-DoF ATE, m) ===')
print(f'{"ds":8s} {"L_gt":>7s} {"base":>7s} {"DC":>7s} {"EKF":>7s} {"gain":>7s} {"EKF漂移%":>8s} {"DC更优帧%":>9s}')
for r in rows:
    print(f'{r[0]:8s} {r[2]:7.1f} {r[3]:7.2f} {r[4]:7.2f} {r[5]:7.2f} {r[9]:+6.2f}% {r[8]:7.2f}% {r[10]:8.1f}%')
print(f'\n图已存 {OUT}/fig_perframe_*.png + summary.csv')
