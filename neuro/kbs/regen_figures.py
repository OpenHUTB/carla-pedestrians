#!/usr/bin/env python3
"""regen_figures.py — 用 10-04 新 run (EKF-odo + DC v3, dc_final) 数据重绘论文 4 张图。
背景: MATLAB R2021b 本机不可用 (Settings 错误), 原图由 MATLAB 生成 (A4 页面+trim);
本脚本直接生成 trim 后的内容尺寸 PDF, tex 中对应 includegraphics 的 trim 参数改为 0。

输出 (neuro/kbs/fig/):
  representative_performance.pdf   6.6944 x 4.7917 in (482 x 345 pt, 2x2)
  performance_summary.pdf          6.4861 x 2.3889 in (467 x 172 pt, 1x3)
  ablation_unified.pdf             6.7083 x 3.3333 in (483 x 240 pt, 1x2)
  KITTI_07_input_output_visualization.pdf  7.7917 x 4.5417 in (561 x 327 pt, 2x2)

口径: 7-DoF (Procrustes, 旋转+平移+均匀尺度) 全轨迹对齐, 与 compute_metrics_with_alignment.m 一致。
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

ROOT = '/home/yangrb/openhutb/neuro/data'
FIG = '/home/yangrb/openhutb/neuro/kbs/fig'

# ---------------- 论文现值 (Table 1, EKF/VO 列不变) ----------------
EKF_RMSE = {'Town01': 15.94, 'Town02': 29.32, 'Town05': 23.02, 'Town10HD': 24.98, 'KITTI07': 22.07}
VO_RMSE  = {'Town01': 36.46, 'Town02': 55.41, 'Town05': 67.98, 'Town10HD': 24.44, 'KITTI07': 78.05}
EKF_DRIFT = {'Town01': 2.03, 'Town02': 5.41, 'Town05': 2.76, 'Town10HD': 7.15, 'KITTI07': 8.33}
MH = {  # EuRoC (mat 内已对齐数组, 数值与论文一致)
    'MH_01': {'NLM': 4.14, 'EKF': 4.14, 'VO': 3.74, 'NLM_drift': 7.84, 'EKF_drift': 7.66, 'L': 81.0, 'frames': 3681},
    'MH_03': {'NLM': 3.32, 'EKF': 3.32, 'VO': 3.43, 'NLM_drift': 3.01, 'EKF_drift': 2.99, 'L': 127.0, 'frames': 2699},
}

# 配色 (与原图一致: NLM 蓝 / EKF 浅橙 / VO 浅绿)
C_NLM, C_EKF, C_VO = '#4490C2', '#F4A582', '#92C59B'
C_GRAY = '#888888'


# ---------------- 数据加载 ----------------
def load_pos(path, poscols=None):
    with open(path) as f:
        first = f.readline().strip()
    try:
        [float(x) for x in first.split(',')]
        header = False
    except ValueError:
        header = True
    a = np.loadtxt(path, delimiter=',', skiprows=1 if header else 0)
    cols = poscols if poscols is not None else (1, 2, 3) if header else (0, 1, 2)
    return a[:, list(cols)]


def procrustes_7dof(gt, est):
    n = min(len(gt), len(est)); gt = gt[:n].astype(float); est = est[:n].astype(float)
    gs = gt - gt.mean(0); es = est - est.mean(0)
    H = gs.T @ es
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); R = U @ D @ Vt
    s = S.sum() / max(np.sum(es**2), 1e-12)
    t = gt.mean(0) - (s * R) @ est.mean(0)
    return (s * (R @ est.T)).T + t, s


def load_ds(ds):
    sub = f'{ROOT}/01_NeuroSLAM_Datasets/{ds}Data_IMU_Fusion'
    gt = load_pos(f'{sub}/ground_truth.txt')
    nlm = load_pos(f'{sub}/dc_final/exp_trajectory.txt')
    ekf = load_pos(f'{sub}/fusion_pose.txt')
    out = {'gt': gt, 'nlm': nlm, 'ekf': ekf, 'len_m': float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())}
    # VO 口径 = MATLAB 流水线内纯视觉分支 pure_visual_trajectory.txt (与论文 Table1 VO 列一致: 36.46/55.41/67.98/24.44/78.05)
    vo_p = f'{sub}/dc_final/pure_visual_trajectory.txt'
    out['vo'] = load_pos(vo_p) if os.path.exists(vo_p) else None
    for name in ('nlm', 'ekf', 'vo'):
        T = out[name]
        if T is not None:
            A, s = procrustes_7dof(out['gt'], T)
            n = min(len(out['gt']), len(T))
            A = A[:n]; g = out['gt'][:n]
            err = np.linalg.norm(A - g, axis=1)
            rpe = float(np.mean(np.linalg.norm(np.diff(A, axis=0) - np.diff(g, axis=0), axis=1)))
            out[name + '_A'] = A
            out[name + '_err'] = err
            out[name + '_rmse'] = float(np.sqrt((err**2).mean()))
            out[name + '_term'] = float(err[-1])
            out[name + '_drift'] = out[name + '_term'] / out['len_m'] * 100
            out[name + '_rpe'] = rpe
    return out


def load_euroc(mh_dir):
    m = loadmat(f'{ROOT}/02_EuRoc_Dataset/{mh_dir}/slam_results/euroc_trajectories.mat')
    out = {}
    g = np.asarray(m['gt_data'][0]['pos'][0], dtype=float).ravel().reshape(-1, 3)
    out['gt'] = g
    for name, key in [('nlm', 'exp_trajectory'), ('ekf', 'fusion_data'), ('vo', 'pure_visual_traj')]:
        v = m[key]
        e = np.asarray(v[0]['pos'][0] if v.dtype.names else v, dtype=float).ravel().reshape(-1, 3)
        A, s = procrustes_7dof(g, e)  # 论文口径: 7-DoF 再对齐
        n = min(len(g), len(A))
        A, g2 = A[:n], g[:n]
        err = np.linalg.norm(A - g2, axis=1)
        rpe = float(np.mean(np.linalg.norm(np.diff(A, axis=0) - np.diff(g2, axis=0), axis=1)))
        L = float(np.linalg.norm(np.diff(g2, axis=0), axis=1).sum())
        out[name] = A
        out[name + '_err'] = err
        out[name + '_rmse'] = float(np.sqrt((err**2).mean()))
        out[name + '_term'] = float(err[-1])
        out[name + '_drift'] = out[name + '_term'] / L * 100
        out[name + '_rpe'] = rpe
        out['L'] = L
    return out


def load_map(ds):
    m = loadmat(f'{ROOT}/01_NeuroSLAM_Datasets/{ds}Data_IMU_Fusion/dc_final/experiences.mat')
    E = m['EXPERIENCES']
    E = E[0] if E.ndim > 1 else E
    n = len(E)
    x = np.asarray(E['x_exp'], dtype=float).ravel()
    y = np.asarray(E['y_exp'], dtype=float).ravel()
    vt = np.asarray(E['vt_id'], dtype=float).ravel()
    vt_int = np.array([int(v) for v in vt])
    n_templates = int(len(np.unique(vt_int[vt_int > 0])))
    # 顺序创建曲线: 到第 i 个节点为止出现过的不同模板数
    seen = set(); growth = np.zeros(n)
    for i in range(n):
        if vt_int[i] > 0:
            seen.add(vt_int[i])
        growth[i] = len(seen)
    # 非相邻 (闭环/重锚定) 边, 无向
    loops = set()
    for i in range(n):
        l = E['links'][i]
        if l is None:
            continue
        for tup in np.asarray(l).ravel():
            t = int(np.asarray(tup[0]).ravel()[0]) - 1
            if 0 <= t < n and t not in (i - 1, i + 1):
                loops.add((min(i, t), max(i, t)))
    return {'n': n, 'x': x, 'y': y, 'n_templates': n_templates, 'growth': growth, 'loops': sorted(loops)}


def style_ax(ax, fs=9):
    for s in ax.spines.values():
        s.set_linewidth(0.8)
    ax.tick_params(labelsize=fs, length=3)
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.set_axisbelow(True)


# ---------------- 图1: representative_performance (482 x 345 pt) ----------------
def fig_representative():
    t01 = load_ds('Town01')
    mh03 = load_euroc('MH_03_medium')
    mp = load_map('Town01')

    fig, axes = plt.subplots(2, 2, figsize=(482 / 72, 345 / 72), dpi=300)
    fs = 8.5

    # (a) Town01 Metrics 柱状图
    ax = axes[0, 0]
    cats = ['RMSE/10', 'Drift (%)', 'RPE x10', 'VT/10', 'Loops']
    v_nlm = [t01['nlm_rmse'] / 10, t01['nlm_drift'], t01['nlm_rpe'] * 10, mp['n_templates'] / 10, len(mp['loops'])]
    v_ekf = [t01['ekf_rmse'] / 10, t01['ekf_drift'], t01['ekf_rpe'] * 10, 0, 0]
    v_vo = [t01['vo_rmse'] / 10, t01['vo_drift'] if t01['vo'] is not None else 0, t01['vo_rpe'] * 10 if t01['vo'] is not None else 0, 0, 0]
    x = np.arange(len(cats)); w = 0.26
    ax.bar(x - w, v_nlm, w, color=C_NLM, label='NeuroLocMap', edgecolor='k', linewidth=0.4)
    ax.bar(x, v_ekf, w, color=C_EKF, label='EKF Fusion', edgecolor='k', linewidth=0.4)
    ax.bar(x + w, v_vo, w, color=C_VO, label='Visual Odometry', edgecolor='k', linewidth=0.4)
    ax.set_xticks(x); ax.set_xticklabels(cats, fontsize=fs - 1.5)
    ax.set_ylabel('Value (scaled)', fontsize=fs)
    ax.set_title('(a) Town01 Metrics (517 m, 5000 frames)', fontsize=fs + 0.5)
    style_ax(ax, fs)

    # (b) Town01 Error Evolution (x = frame number)
    ax = axes[0, 1]
    n = min(len(t01['nlm_err']), len(t01['ekf_err']), len(t01['vo_err']))
    fr = np.arange(n)
    ax.plot(fr, t01['nlm_err'][:n], color=C_NLM, lw=1.2, label='NeuroLocMap')
    ax.plot(fr, t01['ekf_err'][:n], color=C_EKF, lw=1.2, ls='--', label='EKF Fusion')
    ax.plot(fr, t01['vo_err'][:n], color=C_VO, lw=0.9, ls=':', marker='.', ms=2.2, label='Visual Odometry')
    ax.set_xlabel('Frame Number', fontsize=fs)
    ax.set_ylabel('Position Error (m)', fontsize=fs)
    ax.set_title('(b) Town01 Error Evolution (aligned per frame)', fontsize=fs + 0.5)
    style_ax(ax, fs)
    ax.legend(fontsize=fs - 1.5, frameon=True)

    # (c) MH_03 Metrics 柱状图
    ax = axes[1, 0]
    cats = ['RMSE/10', 'Drift (%)', 'RPE x10']
    v_nlm = [mh03['nlm_rmse'] / 10, mh03['nlm_drift'], mh03['nlm_rpe'] * 10]
    v_ekf = [mh03['ekf_rmse'] / 10, mh03['ekf_drift'], mh03['ekf_rpe'] * 10]
    v_vo = [mh03['vo_rmse'] / 10, mh03['vo_drift'], mh03['vo_rpe'] * 10]
    x = np.arange(len(cats)); w = 0.26
    ax.bar(x - w, v_nlm, w, color=C_NLM, edgecolor='k', linewidth=0.4)
    ax.bar(x, v_ekf, w, color=C_EKF, edgecolor='k', linewidth=0.4)
    ax.bar(x + w, v_vo, w, color=C_VO, edgecolor='k', linewidth=0.4)
    ax.set_xticks(x); ax.set_xticklabels(cats, fontsize=fs)
    ax.set_ylabel('Value (scaled)', fontsize=fs)
    ax.set_title('(c) MH_03 Metrics (127 m, 2699 frames)', fontsize=fs + 0.5)
    style_ax(ax, fs)

    # (d) MH_03 Error Evolution
    ax = axes[1, 1]
    n = min(len(mh03['nlm_err']), len(mh03['ekf_err']), len(mh03['vo_err']))
    fr = np.arange(n)
    ax.plot(fr, mh03['nlm_err'], color=C_NLM, lw=1.0, label='NeuroLocMap')
    ax.plot(fr, mh03['ekf_err'], color=C_EKF, lw=1.0, ls='--', label='EKF Fusion')
    ax.plot(fr, mh03['vo_err'], color=C_VO, lw=0.8, ls=':', marker='.', ms=1.8, label='Visual Odometry')
    ax.set_xlabel('Frame Number', fontsize=fs)
    ax.set_ylabel('Position Error (m)', fontsize=fs)
    ax.set_title('(d) MH_03 Error Evolution (aligned per frame)', fontsize=fs + 0.5)
    style_ax(ax, fs)
    ax.legend(fontsize=fs - 1.5, frameon=True)

    fig.tight_layout(pad=1.0)
    fig.savefig(f'{FIG}/representative_performance.pdf', bbox_inches='tight')
    plt.close(fig)
    print('[1/4] representative_performance.pdf  OK  '
          f'Town01: NLM {t01["nlm_rmse"]:.2f}/{t01["nlm_drift"]:.2f}% EKF {t01["ekf_rmse"]:.2f}/{t01["ekf_drift"]:.2f}% '
          f'VO {t01["vo_rmse"]:.2f}/{t01["vo_drift"]:.2f}%  map n={mp["n"]} vt={mp["n_templates"]} loops={len(mp["loops"])}')
    print(f'      MH03: NLM {mh03["nlm_rmse"]:.2f} EKF {mh03["ekf_rmse"]:.2f} VO {mh03["vo_rmse"]:.2f} (论文 3.32/3.32/3.43)')
    return t01, mh03, mp


# ---------------- 图2: performance_summary (467 x 172 pt) ----------------
def fig_performance_summary(t01, mh03, mp01):
    ds_list = ['Town01', 'Town02', 'Town05', 'Town10HD', 'KITTI07']
    data = {}
    for ds in ds_list:
        data[ds] = load_ds(ds)
    t01 = data['Town01']

    fig, axes = plt.subplots(1, 3, figsize=(467 / 72, 172 / 72), dpi=300)
    fs = 8

    labels = ['Town01', 'Town02', 'Town05', 'Town10HD', 'KITTI~07', 'MH_01', 'MH_03']
    nlm_v = [data[d]['nlm_rmse'] for d in ds_list] + [MH['MH_01']['NLM'], MH['MH_03']['NLM']]
    ekf_v = [data[d]['ekf_rmse'] for d in ds_list] + [MH['MH_01']['EKF'], MH['MH_03']['EKF']]
    vo_v = [data[d]['vo_rmse'] if data[d]['vo'] is not None else np.nan for d in ds_list] + [MH['MH_01']['VO'], MH['MH_03']['VO']]

    # (a) RMSE Comparison
    ax = axes[0]
    x = np.arange(len(labels)); w = 0.27
    ax.bar(x - w, nlm_v, w, color=C_NLM, label='NeuroLocMap', edgecolor='k', linewidth=0.3)
    ax.bar(x, ekf_v, w, color=C_EKF, label='EKF', edgecolor='k', linewidth=0.3)
    vo_pad = [0.0 if np.isnan(v) else v for v in vo_v]
    ax.bar(x + w, vo_pad, w, color=C_VO, label='VO', edgecolor='k', linewidth=0.3)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=fs - 1.5, rotation=40, ha='right')
    ax.set_ylabel('RMSE (m)', fontsize=fs)
    ax.set_title('(a) RMSE Comparison', fontsize=fs + 0.5)
    style_ax(ax, fs - 0.5)
    ax.legend(fontsize=fs - 2, frameon=True, loc='upper left')

    # (b) Improvement vs EKF (bubble, size ~ length)
    ax = axes[1]
    lens = [data[d]['len_m'] / 1000 for d in ds_list] + [MH['MH_01']['L'] / 1000, MH['MH_03']['L'] / 1000]
    imp_e = [(e - r) / e * 100 for r, e in zip(nlm_v, ekf_v)]
    for i, (lab, v, L) in enumerate(zip(labels, imp_e, lens)):
        c = C_NLM if v >= 0 else '#D97B7B'
        ax.scatter(i, v, s=120 + L * 2600, c=c, alpha=0.85, edgecolor='k', linewidth=0.6)
        ax.annotate(f'{v:+.1f}%', (i, v), textcoords='offset points', xytext=(0, -14),
                    ha='center', fontsize=fs - 1.5, color='white' if v > 3 else ('black' if v >= 0 else '#8B1A1A'))
    ax.axhline(0, color='k', ls='--', lw=0.8)
    ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels, fontsize=fs - 1.5, rotation=40, ha='right')
    ax.set_ylabel('Improvement vs EKF (%)', fontsize=fs)
    ax.set_title('(b) Improvement vs EKF (bubble size ∝ length)', fontsize=fs + 0.5)
    style_ax(ax, fs - 0.5)
    ax.margins(y=0.25)

    # (c) Improvement vs VO
    ax = axes[2]
    vo_all = [data[d]['vo_rmse'] if data[d]['vo'] is not None else np.nan for d in ds_list] + [MH['MH_01']['VO'], MH['MH_03']['VO']]
    imp_v = [(v - r) / v * 100 if not np.isnan(v) else np.nan for r, v in zip(nlm_v, vo_all)]
    for i, (v, L) in enumerate(zip(imp_v, lens)):
        if np.isnan(v):
            ax.bar(i, 0, color='lightgray', alpha=0.3)
            ax.text(i, 2, 'n/a', ha='center', fontsize=fs - 2, color='gray')
            continue
        c = C_NLM if v >= 0 else '#D97B7B'
        ax.bar(i, v, color=c, edgecolor='k', linewidth=0.3)
        ax.text(i, v + (2.2 if v >= 0 else -4.2), f'{v:+.1f}%', ha='center', fontsize=fs - 1.5,
                color='#1A4A8B' if v >= 0 else '#8B1A1A')
    ax.axhline(0, color='k', lw=0.8)
    ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels, fontsize=fs - 1.5, rotation=40, ha='right')
    ax.set_ylabel('Improvement vs VO (%)', fontsize=fs)
    ax.set_title('(c) Improvement vs VO', fontsize=fs + 0.5)
    style_ax(ax, fs - 0.5)
    ax.margins(y=0.25)

    fig.tight_layout(pad=0.8, rect=(0, 0.09, 1, 1))
    fig.savefig(f'{FIG}/performance_summary.pdf', bbox_inches='tight')
    plt.close(fig)
    r_len = np.corrcoef([l * 1000 for l in lens], [abs(v) for v in imp_e])[0, 1]
    vo_ok = [v for v in imp_v if not np.isnan(v)]
    print(f'[2/4] performance_summary.pdf  OK  impE={["%+.1f" % v for v in imp_e]}  '
          f'impV={["%+.1f" % v if not np.isnan(v) else "na" for v in imp_v]}')
    print(f'      Pearson: gap-vs-len r={r_len:.2f}   mean impE={np.mean(imp_e):.2f}% mean impV(outdoor5)={np.mean(imp_v[:5]):.2f}%')


# ---------------- 图3: ablation_unified (483 x 240 pt) ----------------
def fig_ablation(t01, mp01):
    ds_list = ['Town01', 'Town02', 'Town05', 'Town10HD', 'KITTI07']
    maps = {ds: load_map(ds) for ds in ds_list}
    data_len = {ds: load_ds(ds)['len_m'] for ds in ds_list}

    fig, axes = plt.subplots(1, 2, figsize=(483 / 72, 240 / 72), dpi=300)
    fs = 8.5

    # (a) Ablation bars on Town01
    ax = axes[0]
    cats = ['Full', 'w/o IMU', 'w/o ExpMap']
    vals = [t01['nlm_rmse'], t01['vo_rmse'], t01['ekf_rmse']]
    cols = [C_NLM, C_VO, C_EKF]
    bars = ax.bar(cats, vals, color=cols, edgecolor='k', linewidth=0.5, width=0.55)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.6, f'{v:.1f} m', ha='center', fontsize=fs)
    ax.set_ylabel('RMSE (m)', fontsize=fs)
    ax.set_title('(a) Ablation RMSE on Town01 (517 m)', fontsize=fs + 0.5)
    ax.set_ylim(0, max(vals) * 1.15)
    style_ax(ax, fs)

    # (b) VT growth
    ax = axes[1]
    style_cfg = {
        'Town01': dict(color=C_NLM, ls='-'),
        'Town02': dict(color='#D97B7B', ls='--'),
        'Town05': dict(color=C_VO, ls='-.', ),
        'Town10HD': dict(color='#8B5FBF', ls=':', marker='.', ms=2),
        'KITTI07': dict(color='#E8A33D', ls='-'),
    }
    for ds in ds_list:
        mp = maps[ds]
        L = data_len[ds]
        s = np.linalg.norm(np.diff(np.column_stack([mp['x'], mp['y']]), axis=0), axis=1)
        arc = np.concatenate([[0], np.cumsum(s)])
        ax.plot(arc / L * 100, mp['growth'], **style_cfg[ds], lw=1.3,
                label=f'{ds} ({mp["n_templates"]})')
    ax.axhline(5, color='k', lw=1.2, ls='-', label='RatSLAM ($\\sim$5 templates)')
    ax.set_xlabel('Trajectory Progress (% of length)', fontsize=fs)
    ax.set_ylabel('Accumulated Visual Templates', fontsize=fs)
    ax.set_title('(b) Visual Template Growth (5 outdoor sequences)', fontsize=fs + 0.5)
    ax.set_xlim(0, 100)
    style_ax(ax, fs)
    ax.legend(fontsize=fs - 2, frameon=True, loc='upper left', ncol=1)

    fig.tight_layout(pad=1.0)
    fig.savefig(f'{FIG}/ablation_unified.pdf', bbox_inches='tight')
    plt.close(fig)
    print(f'[3/4] ablation_unified.pdf  OK  Full={t01["nlm_rmse"]:.2f} VO={t01["vo_rmse"]:.2f} EKF={t01["ekf_rmse"]:.2f}')


# ---------------- 图4: KITTI_07_input_output (561 x 327 pt) ----------------
def fig_kitti_io():
    sub = f'{ROOT}/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion'
    gt = load_pos(f'{sub}/ground_truth.txt')
    nlm = load_pos(f'{sub}/dc_final/exp_trajectory.txt')
    A, s = procrustes_7dof(gt, nlm)
    n = min(len(gt), len(A))
    gt, A = gt[:n], A[:n]
    imu = np.loadtxt(f'{sub}/aligned_imu.txt', delimiter=',', skiprows=1)
    mp = load_map('KITTI07')

    fig, axes = plt.subplots(2, 2, figsize=(561 / 72, 327 / 72), dpi=300)
    fs = 8.5

    # (a) Input images + heading  (单 axes 内 2x2 网格, extent 定位)
    ax = axes[0, 0]
    ax.set_xlim(0, 54); ax.set_ylim(0, 16); ax.set_aspect('auto'); ax.axis('off')
    ax.set_title('(a) Input: RGB Images + IMU Data', fontsize=fs + 0.5)
    import matplotlib.image as mimg
    frames = [110, 330, 551, 771]
    cw, ch, gap = 26, 6.8, 2.0
    for k, fr in enumerate(frames):
        p = f'{sub}/{fr:06d}.png'
        if not os.path.exists(p):
            p = f'{sub}/{fr:04d}.png'
        if not os.path.exists(p):
            continue
        im = mimg.imread(p)
        if im.ndim == 2:
            im = np.stack([im, im, im], axis=2)
        elif im.shape[2] == 4:
            im = im[:, :, :3]
        row, col = divmod(k, 2)
        x0 = col * (cw + gap)
        y0 = 8.4 - row * (ch + gap)
        ax.imshow(im, extent=(x0, x0 + cw, y0, y0 + ch), aspect='auto', zorder=2)
        ax.add_patch(plt.Rectangle((x0, y0), cw, ch, fill=False, ec='k', lw=0.7, zorder=3))
        ax.text(x0 + 0.5, y0 + ch - 0.9, f'Frame {fr}', fontsize=fs - 1, color='white', zorder=4,
                bbox=dict(boxstyle='round,pad=0.2', fc='#333333', alpha=0.75, ec='none'))
        # heading arrow (GT 航向)
        g0, g1 = gt[fr], gt[min(fr + 3, n - 1)]
        ang = np.arctan2(g1[1] - g0[1], g1[0] - g0[0])
        cx, cy = x0 + cw / 2, y0 + ch / 2
        ax.annotate('', xy=(cx + 1.7 * np.cos(ang), cy + 1.7 * np.sin(ang)),
                    xytext=(cx - 1.7 * np.cos(ang), cy - 1.7 * np.sin(ang)),
                    zorder=5,
                    arrowprops=dict(arrowstyle='->', color='#4490C2', lw=2.4,
                                    mutation_scale=14))

    # (b) top: trajectory aligned
    ax = axes[0, 1]
    ax.plot(gt[:, 0], gt[:, 1], color='k', lw=1.8, label='Ground Truth')
    ax.plot(A[:, 0], A[:, 1], color=C_NLM, lw=1.3, label='NeuroLocMap')
    ax.plot(gt[0, 0], gt[0, 1], 'o', color='#3C9A3C', ms=6, label='Start')
    ax.plot(gt[-1, 0], gt[-1, 1], 's', color='#C0392B', ms=6, label='End')
    e = np.linalg.norm(A - gt, axis=1)
    ax.text(0.98, 0.06, f'RMSE: NLM {np.sqrt((e**2).mean()):.2f} / EKF {EKF_RMSE["KITTI07"]:.2f} / VO {VO_RMSE["KITTI07"]:.2f} m',
            transform=ax.transAxes, ha='right', fontsize=fs - 1,
            bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='0.5', alpha=0.9))
    ax.set_xlabel('X (m)', fontsize=fs); ax.set_ylabel('Y (m)', fontsize=fs)
    ax.set_title('Trajectory (aligned, official 7-DoF metric)', fontsize=fs + 0.5)
    ax.legend(fontsize=fs - 1.5, loc='upper right', frameon=True)
    ax.set_aspect('equal'); style_ax(ax, fs)

    # (b) bottom: experience map topology
    ax = axes[1, 1]
    x, y = mp['x'], mp['y']
    ax.plot(x, y, '-', color=C_NLM, lw=1.0, alpha=0.9)
    for (i, j) in mp['loops']:
        if j < len(x):
            ax.plot([x[i], x[j]], [y[i], y[j]], '--', color='#3C9A3C', lw=1.4, alpha=0.9)
    ax.scatter(x, y, s=2.2, color=C_NLM, alpha=0.5, linewidths=0)
    if mp['loops']:
        li, lj = mp['loops'][0]
        ax.plot([x[li], x[lj]], [y[li], y[lj]], '--', color='#3C9A3C', lw=1.6)
        ax.scatter([x[li], x[lj]], [y[li], y[lj]], s=28, marker='*', color='#C0392B',
                   edgecolor='k', linewidths=0.5, zorder=5, label='Loop closure edge')
    ax.set_xlabel('X (m)', fontsize=fs); ax.set_ylabel('Y (m)', fontsize=fs)
    ax.set_title(f'Experience Map Topology ({mp["n"]} nodes, {len(mp["loops"])} loop edges)', fontsize=fs + 0.5)
    ax.legend(fontsize=fs - 1.5, loc='upper right', frameon=True)
    style_ax(ax, fs)

    # (b) bottom-left: IMU
    ax = axes[1, 0]
    acc = np.sqrt(imu[:, 1]**2 + imu[:, 2]**2 + imu[:, 3]**2)
    gyro = imu[:, 5]
    t = imu[:, 0]
    ax.plot(t, acc, color='#D97B7B', lw=0.5, label='Accel norm')
    ax.axhline(9.81, color='#D97B7B', ls='--', lw=0.8, alpha=0.6)
    ax.set_ylabel('Acceleration (m/s$^2$)', fontsize=fs)
    ax.set_xlabel('Time (s)', fontsize=fs)
    ax2 = ax.twinx()
    ax2.plot(t, gyro, color=C_NLM, lw=0.5, label='Gyro Z')
    ax2.set_ylabel('Angular Vel (rad/s)', fontsize=fs, color=C_NLM)
    ax2.tick_params(axis='y', labelcolor=C_NLM, labelsize=fs - 1)
    ax.set_title('IMU Sensor Data (10 Hz, 692 m highway)', fontsize=fs + 0.5)
    style_ax(ax, fs)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=fs - 2, loc='upper right', frameon=True)

    fig.tight_layout(pad=1.0)
    fig.savefig(f'{FIG}/KITTI_07_input_output_visualization.pdf', bbox_inches='tight')
    plt.close(fig)
    print(f'[4/4] KITTI_07_input_output_visualization.pdf  OK  nodes={mp["n"]} loops={len(mp["loops"])} '
          f'RMSE={np.sqrt((e**2).mean()):.2f}')


if __name__ == '__main__':
    t01, mh03, mp01 = fig_representative()
    fig_performance_summary(t01, mh03, mp01)
    fig_ablation(t01, mp01)
    fig_kitti_io()
    print('DONE')
