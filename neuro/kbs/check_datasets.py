#!/usr/bin/env python3
"""kbs/check_datasets.py — 论文数据集结构预检 (Town05/03/07/10HD + KITTI07, 2026-10-12)

用法: python3 neuro/kbs/check_datasets.py
逐个检查 compute_final_metrics.py / REGEN_PAPER_FIGURES.m 所需的 5 个文件:
  ground_truth.txt           有头 CSV, 需 pos_x,pos_y,pos_z
  fusion_pose.txt            有头 CSV, 需 pos_x,pos_y,pos_z (EKF 基线)
  visual_odometry.txt        有头 CSV, 需 vo_x,vo_y,vo_z
                             (KITTI07 本地缺失时可回退 ../KITTI_07/visual_odometry_ekf.txt)
  slam_results/trajectories.mat   变量 exp_trajectory (NLM 轨迹)
  slam_results/experiences.mat    变量 NUM_EXPS, EXPERIENCES (模板/节点/闭环计数)
退出码: 0 = 全部就绪, 1 = 有缺失/错误
"""
import os, sys
import pandas as pd, scipy.io as sio

D = '../data/01_NeuroSLAM_Datasets/'
TOWNS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
         'Town10HDData_IMU_Fusion']
KITTI = 'KITTI07Data_IMU_Fusion'
KVO_FB = D.rstrip('/') + '/../KITTI_07/visual_odometry_ekf.txt'

def chk_csv(p, cols, label):
    if not os.path.exists(p):
        return f'{label}: MISSING'
    try:
        a = pd.read_csv(p, nrows=3)
        c = [x for x in cols if x in a.columns]
        if len(c) != 3:
            return f'{label}: header 缺 {sorted(set(cols) - set(a.columns))}'
        n = sum(1 for _ in open(p)) - 1
        return f'{label}: OK ({n} 行)'
    except Exception as e:
        return f'{label}: 读错 {e}'

def chk_mat(p, vars_, label):
    if not os.path.exists(p):
        return f'{label}: MISSING'
    try:
        m = sio.loadmat(p)
        miss = [v for v in vars_ if v not in m]
        if miss:
            have = sorted(k for k in m if not k.startswith('__'))[:8]
            return f'{label}: 变量缺 {miss} (现有 {have})'
        return f'{label}: OK'
    except Exception as e:
        return f'{label}: 读错 {e}'

n_missing = 0
for d in TOWNS + [KITTI]:
    print(f'══ {d} ══')
    base = D + d + '/'
    if not os.path.isdir(base):
        print(f'  目录: MISSING ({base})')
        n_missing += 1
        continue
    checks = [
        chk_csv(base + 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'], 'ground_truth.txt'),
        chk_csv(base + 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'], 'fusion_pose.txt'),
    ]
    vo = chk_csv(base + 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z'], 'visual_odometry.txt')
    if d == KITTI and 'MISSING' in vo and os.path.exists(KVO_FB):
        vo = chk_csv(KVO_FB, ['vo_x', 'vo_y', 'vo_z'], 'visual_odometry.txt (回退 KITTI_07/)')
    checks.append(vo)
    checks.append(chk_mat(base + 'slam_results/trajectories.mat', ['exp_trajectory'],
                          'slam_results/trajectories.mat'))
    checks.append(chk_mat(base + 'slam_results/experiences.mat', ['NUM_EXPS', 'EXPERIENCES'],
                          'slam_results/experiences.mat'))
    for c in checks:
        print('  ' + c)
        if 'MISSING' in c or '缺' in c or '读错' in c:
            n_missing += 1

print('═' * 44)
if n_missing == 0:
    print('✓ 全部 5 个数据集就绪, 可直接重算 final_metrics.json')
else:
    print(f'✗ 待补齐 {n_missing} 项')
sys.exit(0 if n_missing == 0 else 1)
