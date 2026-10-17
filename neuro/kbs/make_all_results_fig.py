# -*- coding: utf-8 -*-
"""
全部效果对比总览图:
  (a) Sim(3) 全重拟合 ATE: 5 集 x 4 配置
  (b) SE(3) 刚性 ATE:      5 集 x 4 配置
  (c) KITTI seq 07 外部基线 RTE (log)
ATE 数字直接取 se3_metrics.json (单一来源);
RTE 数字为 kitti_rte.py 实测 (复现: python3 kitti_rte.py)。
输出: fig/all_results_overview.pdf + .png
"""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

KBS = '/home/yangrb/openhutb/neuro/kbs'
FIG = f'{KBS}/fig'
with open(f'{KBS}/se3_metrics.json') as f:
    SE3 = json.load(f)

C_NLM, C_EKF, C_VO = '#4490C2', '#F4A582', '#92C59B'
C_EKFS, C_GRAY = '#9E7FB8', '#888888'
FS = 9


def style_ax(ax, fs=FS, grid_axis='y'):
    for s in ax.spines.values():
        s.set_linewidth(0.8)
    ax.tick_params(labelsize=fs, length=3)
    ax.grid(True, axis=grid_axis, linewidth=0.4, alpha=0.5)
    ax.set_axisbelow(True)


DS = [('Town05Data_IMU_Fusion', 'Town05'),
      ('Town03Data_IMU_Fusion', 'Town03'),
      ('Town07Data_IMU_Fusion', 'Town07'),
      ('Town10HDData_IMU_Fusion', 'Town10HD'),
      ('KITTI07Data_IMU_Fusion', 'KITTI 07')]


def row(ds_key, cfg, metric):
    d = SE3.get(ds_key, {}).get(cfg)
    return None if d is None else d[metric]


def bar_panel(ax, metric, title, ylim):
    x = np.arange(len(DS))
    w = 0.19
    order = [('VO', 'VO', C_VO),
             ('EKF', 'EKF', C_EKF),
             ('EKF+尺度', 'EKF+scale', C_EKFS),
             ('NLM-全系统', 'NLM full', C_NLM)]
    for off, (key, lab, col) in enumerate(order):
        for j, (k, _) in enumerate(DS):
            v = row(k, key, metric)
            if v is None and key == 'NLM-全系统':
                v = row(k, 'NLM-闭环线', metric)  # KITTI 无全系统, 闭环线承载
            cx = x[j] + (off - 1.5) * w
            if v is None:
                ax.text(cx, ylim * 0.03, 'n/a', ha='center', fontsize=6.5, color=C_GRAY)
                continue
            ax.bar(cx, v, w, color=col, edgecolor='k', linewidth=0.4,
                   label=lab if j == 0 else None)
            ax.text(cx, v + ylim * 0.008, f'{v:.1f}', rotation=90,
                    ha='left', va='bottom', fontsize=6.2)
    ax.set_xticks(x)
    ax.set_xticklabels([lab for _, lab in DS])
    ax.set_ylim(0, ylim)
    ax.set_ylabel('ATE RMSE (m)', fontsize=FS)
    ax.set_title(title, fontsize=FS + 0.5)
    style_ax(ax)
    ax.legend(fontsize=6.8, frameon=True, ncol=2, loc='upper left')


fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(14.2, 4.0), dpi=300)
bar_panel(axa, 'sim3_rmse', '(a) Sim(3) full refit (shape upper bound)', 64)
bar_panel(axb, 'se3_rmse', '(b) SE(3) rigid (metric-scale caliber)', 70)

# ---- (c) 外部基线 RTE (kitti_rte.py 实测, 2026-10-16) ----
items = [
    ('SOFT-SLAM (stereo)', 0.65, C_GRAY, 1.0),
    ('VINS-Fusion (st+IMU)', 1.09, C_GRAY, 1.0),
    ('ORB-SLAM2 (stereo)', 1.15, C_GRAY, 1.0),
    ('MonoDepth2 (mono)', 12.59, C_GRAY, 1.0),
    ('NLM EKF core: raw', 94.7, C_EKF, 1.0),
    ('NLM EKF core: Sim(3)-aligned', 72.1, C_NLM, 1.0),
    ('NLM EKF core: excl top-50', 38.2, C_NLM, 0.45),
    ('NLM full: raw', 101.1, C_EKF, 1.0),
    ('NLM full: Sim(3)-aligned', 75.2, C_NLM, 1.0),
    ('NLM full: excl top-50', 46.9, C_NLM, 0.45),
]
y = np.arange(len(items))[::-1]
for yi, (lab, v, col, alpha) in zip(y, items):
    axc.barh(yi, v, 0.62, color=col, alpha=alpha, edgecolor='k', linewidth=0.4)
    axc.text(v * 1.12, yi, f'{v:g}', va='center', fontsize=6.6)
axc.set_xscale('log')
axc.set_xlim(0.45, 300)
axc.set_ylim(-1.25, 9.7)
axc.set_yticks(y)
axc.set_yticklabels([i[0] for i in items], fontsize=6.8)
axc.axvline(1.2, color='k', ls=':', lw=0.8)
axc.text(1.45, -0.95, 'scale-consistent < 1.2%', fontsize=6.6, va='center')
axc.set_xlabel('RTE — relative trajectory error (%)', fontsize=FS)
axc.set_title('(c) KITTI seq 07 vs published baselines (RTE)', fontsize=FS + 0.5)
style_ax(axc, grid_axis='x')
axc.grid(True, axis='y', linewidth=0.4, alpha=0.25)

fig.suptitle('NeuroLocMap — all results overview: two alignment calibers on 5 scenarios '
             '+ KITTI external baselines', fontsize=10.5, y=1.02)
fig.tight_layout(pad=1.2)
fig.savefig(f'{FIG}/all_results_overview.pdf', bbox_inches='tight')
fig.savefig(f'{FIG}/all_results_overview.png', bbox_inches='tight')
plt.close(fig)
print('OK  fig/all_results_overview.pdf + .png')
for k, lab in DS:
    vals = []
    for cfg in ['VO', 'EKF', 'EKF+尺度', 'NLM-全系统', 'NLM-闭环线']:
        v = row(k, cfg, 'sim3_rmse')
        vals.append(f'{cfg[:4]}={v if v is not None else "n/a"}')
    print(f'  {lab:10s} ' + '  '.join(vals))
