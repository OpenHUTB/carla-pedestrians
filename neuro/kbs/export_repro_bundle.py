#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kbs/export_repro_bundle.py — 把 Table 3 还缺的三类产物从原始运行目录打包出来

背景: 公开的复现资产包 (NLM_Repro_Assets_*) 只含每集的 slam_results/ (闭环线),
因此 Table 3 的 Ours 列 (NLM-fullsystem)、EKF+scale 列、以及 §4.4 的 ORB 消融无法从
该包复现。本脚本从作者本机的运行结果目录里把这三类产物收齐, 写成 se3_metrics.py
直接可吃的布局, 并生成 MANIFEST.json (含 sha256 / 行数 / GT 长度校验)。

用法:
  python3 export_repro_bundle.py --src /path/to/runs --out /path/to/NLM_Repro_Assets_full
  python3 export_repro_bundle.py --src ... --out ... --check-only   # 只体检, 不拷贝

--src 期望的布局 (与 se3_metrics.py 的 ../data 同构):
  <src>/01_NeuroSLAM_Datasets/<seq>/slam_results_scale_nlm/trajectories.mat
  <src>/01_NeuroSLAM_Datasets/<seq>/slam_results_orbonly/trajectories.mat
  <src>/_p11_backup_20261014/<Town05|Town03|Town07|Town10HD>/fp_head030.txt
  <src>/01_NeuroSLAM_Datasets/<seq>/ground_truth.txt          (用于长度/行数校验)

也接受扁平布局: <src>/<seq>/slam_results_scale_nlm/... 与 <src>/_p11_backup_20261014/...
"""
import argparse
import hashlib
import json
import os
import shutil
import sys

import numpy as np
import pandas as pd
import scipy.io as sio

SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']
SHORT = {'Town05Data_IMU_Fusion': 'Town05', 'Town03Data_IMU_Fusion': 'Town03',
         'Town07Data_IMU_Fusion': 'Town07', 'Town10HDData_IMU_Fusion': 'Town10HD'}
# KITTI 07 无尺度支路 (S.1 零样本失效), 因此不需要 EKF+scale 产物
NO_SCALE = {'KITTI07Data_IMU_Fusion'}


def sha256(path, cap=1 << 30):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def gt_len(path):
    if not os.path.exists(path):
        return None
    cols = ['pos_x', 'pos_y', 'pos_z']
    df = pd.read_csv(path)
    if not all(c in df.columns for c in cols):
        return None
    gt = df[cols].values.astype(float)
    return float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()), len(gt)


def probe_traj(path, var='exp_trajectory'):
    """返回 (行数, 首末点) 供体检使用; 打不开返回 (None, None)。"""
    try:
        a = np.asarray(sio.loadmat(path)[var], dtype=float)
    except Exception as e:
        return None, f'unreadable: {e}'
    if a.ndim != 2 or a.shape[1] < 3:
        return None, f'bad shape {a.shape}'
    return len(a), np.round(a[[0, -1]], 2).tolist()


def resolve_root(src):
    """把 --src 归一化到数据集根（容忍 01_NeuroSLAM_Datasets/ 前缀、datasets/ 前缀）。

    资产包与作者 run 树的层级不同：包是 <root>/datasets/<seq>/，run 树常是
    <root>/01_NeuroSLAM_Datasets/<seq>/。以前只认后者，直接对资产包跑会误报
    "找不到 ground_truth.txt"（文件其实在上一级）。
    """
    for pre in ('', '01_NeuroSLAM_Datasets', 'datasets', 'datasets/01_NeuroSLAM_Datasets'):
        cand = os.path.join(src, pre) if pre else src
        for probe in ('ground_truth.txt', 'slam_results'):
            if os.path.exists(os.path.join(cand, SETS[0], probe)):
                return cand, pre
    return src, ''


def find(root, rel_a, rel_b):
    for p in (os.path.join(root, rel_a), os.path.join(root, rel_b)):
        if os.path.exists(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='作者本机运行结果根目录')
    ap.add_argument('--out', default='./NLM_Repro_Assets_full', help='输出包目录')
    ap.add_argument('--check-only', action='store_true', help='只体检不拷贝')
    a = ap.parse_args()

    ds_rel = '01_NeuroSLAM_Datasets'
    base, pre = resolve_root(a.src)
    items, missing, problems = [], [], []

    for sd in SETS:
        gt = find(base, f'{sd}/ground_truth.txt', f'{sd}/ground_truth.txt')
        gl = gt_len(gt) if gt else None
        if gt is None:
            problems.append(f'{sd}: 找不到 ground_truth.txt, 无法校验行数与长度')

        # 评估还需要的参考输入: GT / EKF 核 / VO (缺失不算错, 但会记录)
        for fn in ('ground_truth.txt', 'fusion_pose.txt', 'visual_odometry.txt'):
            p = find(base, f'{sd}/{fn}', f'{sd}/{fn}')
            if p is None:
                if fn == 'visual_odometry.txt':
                    continue           # KITTI 的 VO 在别的目录, se3_metrics.py 有回退
                problems.append(f'{sd}: 找不到 {fn} (评估该集需要)')
                continue
            items.append(('input', sd, p, fn, None, gl))

        # 闭环线也一起收: 否则从零构建的 --out 里没有 slam_results/,
        # se3_metrics 会把 Table 1/3 的 NLM 列全部结转成 carried
        for cfg, sub in [('NLM-loopline', 'slam_results'),
                         ('NLM-fullsystem', 'slam_results_scale_nlm'),
                         ('ORB-only', 'slam_results_orbonly')]:
            p = find(base, f'{sd}/{sub}/trajectories.mat', f'{sd}/{sub}/trajectories.mat')
            if p is None:
                missing.append(f'{sd}/{cfg} ({sub}/trajectories.mat)')
                continue
            n, info = probe_traj(p)
            if n is None:
                problems.append(f'{sd}/{cfg}: {info}')
                continue
            if gl and n != gl[1]:
                problems.append(f'{sd}/{cfg}: 轨迹 {n} 行 != GT {gl[1]} 行')
            items.append((cfg, sd, p, 'trajectories.mat', n, gl))

        if sd in NO_SCALE:
            continue
        bk_rel = f'_p11_backup_20261014/{SHORT[sd]}/fp_head030.txt'
        p = find(base, bk_rel, f'../{bk_rel}')
        if p is None:
            missing.append(f'{sd}/EKF+scale (_p11_backup_20261014/{SHORT[sd]}/fp_head030.txt)')
            continue
        df = pd.read_csv(p)
        if not all(c in df.columns for c in ('pos_x', 'pos_y', 'pos_z')):
            problems.append(f'{sd}/EKF+scale: 缺少 pos_x/y/z 列')
            continue
        if gl and len(df) != gl[1]:
            problems.append(f'{sd}/EKF+scale: {len(df)} 行 != GT {gl[1]} 行')
        items.append(('EKF+scale', sd, p, 'fp_head030.txt', len(df), gl))

    print(f'src : {a.src}')
    if pre:
        print(f'      → 数据集根识别为 {base}')
    print(f'out : {a.out}')
    print(f'找到 {len(items)} 个产物, 缺 {len(missing)} 个')
    for cfg, sd, p, fn, n, gl in items:
        L = f'{gl[0]:.1f} m / {gl[1]} 帧' if gl else 'GT 未知'
        rows = f'{n:6d} 行' if n else '  (输入)'
        print(f'  [{cfg:10s}] {sd:26s} {rows}  {L}  {p}')
    if missing:
        print('\n缺失:')
        for m in missing:
            print(f'  - {m}')
    if problems:
        print('\n体检问题:')
        for m in problems:
            print(f'  ! {m}')

    if a.check_only:
        return 0 if not missing and not problems else 1

    if not items:
        print('\n没有可打包的产物; 退出。')
        return 1

    man = {'_meta': {'src': a.src, 'n_items': len(items), 'missing': missing, 'problems': problems},
           'files': {}}
    for cfg, sd, p, fn, n, gl in items:
        if cfg == 'EKF+scale':
            dst_dir = os.path.join(a.out, '_p11_backup_20261014', SHORT[sd])
        else:                                   # input / NLM-fullsystem / ORB-only
            sub = {'NLM-loopline': 'slam_results',
                   'NLM-fullsystem': 'slam_results_scale_nlm',
                   'ORB-only': 'slam_results_orbonly'}.get(cfg, '')
            dst_dir = os.path.join(a.out, ds_rel, sd, sub)
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, fn)
        shutil.copy2(p, dst)
        man['files'][f'{sd}/{cfg}'] = {
            'path': os.path.relpath(dst, a.out), 'bytes': os.path.getsize(dst),
            'sha256': sha256(dst), 'rows': n,
            'gt_len_m': round(gl[0], 1) if gl else None,
        }

    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'MANIFEST.json'), 'w') as f:
        json.dump(man, f, indent=2, ensure_ascii=False)

    total = sum(v['bytes'] for v in man['files'].values())
    print(f'\n打包完成: {len(items)} 个文件, {total / 1e6:.1f} MB -> {a.out}')
    print(f'清单: {os.path.join(a.out, "MANIFEST.json")}')
    print('校验方式: NLM_DATA_ROOT=<out>/01_NeuroSLAM_Datasets '
          'NLM_BACKUP_ROOT=<out>/_p11_backup_20261014 python3 se3_metrics.py')
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
