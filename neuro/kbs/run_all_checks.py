#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kbs/run_all_checks.py — 一条命令把论文全部数字对着数据重算一遍

它不做任何写入 (不覆盖 final_metrics.json / se3_metrics.json), 只把算出来的值
和 expected_values.json 里论文印刷值逐格对比, 打 OK / FAIL(+偏差) / SKIP(缺产物)。

用法:
  python3 neuro/kbs/run_all_checks.py                      # 用 ../data/01_NeuroSLAM_Datasets
  NLM_DATA_ROOT=/path/NLM_Repro_Assets_*/datasets \
  NLM_BACKUP_ROOT=/path/_p11_backup_20261014 \
  python3 neuro/kbs/run_all_checks.py

  python3 neuro/kbs/run_all_checks.py --data-root <dir> --backup-root <dir> --json report.json

分四段:
  A 输入清单   —— 每个集该有的文件在不在、行数对不对
  B Table 3    —— NLM/EKF/VO 的 Sim(3) RMSE 与 drift, 模板/节点/闭环计数, GT 长度
  C Table 4    —— 双口径四配置 8 格/集 (缺 slam_results_scale_nlm / orbonly / fp_head030 时记 SKIP)
  D 闭环基准   —— Town01 折返: GT 长度/帧数/事件数/terminal/ RMSE / 15x 比值

退出码: 0 = 没有 FAIL; 1 = 有 FAIL; 2 = 连 Table 3 都算不全 (缺基础输入)
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import scipy.io as sio

SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']
SHORT = {'Town05Data_IMU_Fusion': 'Town05', 'Town03Data_IMU_Fusion': 'Town03',
         'Town07Data_IMU_Fusion': 'Town07', 'Town10HDData_IMU_Fusion': 'Town10HD'}
HERE = os.path.dirname(os.path.abspath(__file__))
TOL = 0.011

res = {'ok': 0, 'fail': 0, 'skip': 0, 'fails': [], 'skips': []}


def mark(tag, ok, name, got=None, exp=None):
    if ok is None:
        res['skip'] += 1
        res['skips'].append(name)
        print(f'  {tag:6s} {name:44s}  SKIP')
    elif ok:
        res['ok'] += 1
        print(f'  {tag:6s} {name:44s}  OK    {got}')
    else:
        res['fail'] += 1
        res['fails'].append((name, got, exp))
        if isinstance(got, (int, float)) and isinstance(exp, (int, float)):
            print(f'  {tag:6s} {name:44s}  FAIL  got {got}  expected {exp}  (diff {got - exp:+.2f})')
        else:
            print(f'  {tag:6s} {name:44s}  FAIL  got {got}  expected {exp}')


def cmp(name, got, exp, tag='value'):
    if exp is None:
        return mark(tag, None, name)
    if got is None:
        return mark(tag, False, name, 0.0, exp)
    return mark(tag, abs(got - exp) <= TOL, name, got, exp)


def read_csv_dir(root, sd, fn, cols):
    p = os.path.join(root, sd, fn)
    if not os.path.exists(p):
        return None, p
    try:
        a = pd.read_csv(p)
    except Exception:
        return None, p
    c = [x for x in cols if x in a.columns]
    return (a[c].values.astype(float) if len(c) == 3 else None), p


def read_mat(root, sd, sub, var):
    p = os.path.join(root, sd, sub, 'trajectories.mat')
    if not os.path.exists(p):
        return None, p
    try:
        return np.asarray(sio.loadmat(p)[var], dtype=float), p
    except Exception:
        return None, p


def sim3(est, gt):
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    ea = s * (est @ T) + (gt.mean(0) - s * (est.mean(0) @ T))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100)


def se3(est, gt):
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    ea = est @ R + (gt.mean(0) - (est @ R).mean(0))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100)


def anchored(est, gt, k=100):
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    e0, g0 = est[:k], gt[:k]
    e0c, g0c = e0 - e0.mean(0), g0 - g0.mean(0)
    U, S, Vt = np.linalg.svd(g0c.T @ e0c)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (e0c ** 2).sum()
    ea = s * (est @ T) + (g0.mean(0) - s * (e0.mean(0) @ T))
    term = float(np.linalg.norm(ea[-1] - gt[-1]))
    rmse = float(np.sqrt(((ea - gt) ** 2).sum(1).mean()))
    return term, rmse


def counts(exp_path):
    m = sio.loadmat(exp_path)
    NUM = int(float(m['NUM_EXPS'].ravel()[0]))
    EXP = m['EXPERIENCES']
    nvt = nloop = 0
    n_exp = EXP.shape[1] if EXP.ndim == 2 else EXP.shape[0]
    for k in range(n_exp):
        e = EXP[0, k] if EXP.ndim == 2 else EXP[k]
        if e.shape != ():
            e = e.ravel()[0]
        try:
            nvt += 1 if int(float(np.atleast_1d(e['vt_id']).ravel()[0])) > 0 else 0
        except Exception:
            pass
        try:
            if int(float(np.atleast_1d(e['numlinks']).ravel()[0])) > 0:
                lnks = e['links']
                if lnks.shape != ():
                    lnks = np.atleast_1d(lnks)
                for j in range(len(lnks)):
                    lid = lnks[j]
                    if lid.shape != ():
                        lid = lid.ravel()[0]
                    if 1 <= int(float(np.atleast_1d(lid['exp_id']).ravel()[0])) <= k:
                        nloop += 1
        except Exception:
            pass
    return NUM, nvt, nloop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-root', default=os.environ.get('NLM_DATA_ROOT',
                                                          os.path.join(HERE, '..', 'data', '01_NeuroSLAM_Datasets')))
    ap.add_argument('--backup-root', default=os.environ.get('NLM_BACKUP_ROOT',
                                                            os.path.join(HERE, '..', 'data', '_p11_backup_20261014')))
    ap.add_argument('--json', default=None, help='把结果另存为 JSON')
    a = ap.parse_args()
    root, bk = os.path.abspath(a.data_root), os.path.abspath(a.backup_root)
    exp = json.load(open(os.path.join(HERE, 'expected_values.json')))

    print(f'data root   : {root}')
    print(f'backup root : {bk}\n')

    # ── A 输入清单 ──
    print('── A 输入清单 ' + '─' * 62)
    per_set = {}
    for sd in SETS:
        gt, _ = read_csv_dir(root, sd, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
        ek, _ = read_csv_dir(root, sd, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'])
        vo, _ = read_csv_dir(root, sd, 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z'])
        lo, _ = read_mat(root, sd, 'slam_results', 'exp_trajectory')
        full, _ = read_mat(root, sd, 'slam_results_scale_nlm', 'exp_trajectory')
        orb, _ = read_mat(root, sd, 'slam_results_orbonly', 'exp_trajectory')
        fph = None
        if sd in SHORT:
            p = os.path.join(bk, SHORT[sd], 'fp_head030.txt')
            if os.path.exists(p):
                d = pd.read_csv(p)
                c = [x for x in ('pos_x', 'pos_y', 'pos_z') if x in d.columns]
                fph = d[c].values.astype(float) if len(c) == 3 else None
        per_set[sd] = dict(gt=gt, ek=ek, vo=vo, lo=lo, full=full, orb=orb, fph=fph)
        e = exp['table1'][sd]
        for nm, arr in [('ground_truth.txt', gt), ('fusion_pose.txt', ek),
                        ('visual_odometry.txt', vo), ('slam_results/trajectories.mat', lo)]:
            mark('file', arr is not None, f'{sd}/{nm}',
                 f'{len(arr)} rows' if arr is not None else None)
        mark('rows', gt is not None and exp['table1'][sd]['frames'] == len(gt),
             f'{sd} GT frames', len(gt) if gt is not None else 0, e['frames'])
        mark('gt', gt is not None and abs(float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
                                          - e['gt_len_m']) <= 0.5,
             f'{sd} GT length (m)',
             round(float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()), 1) if gt is not None else 0,
             e['gt_len_m'])
        p = os.path.join(root, sd, 'slam_results', 'experiences.mat')
        if os.path.exists(p):
            n, vt, nl = counts(p)
            cmp(f'{sd} nodes', n, e['exp_nodes'])
            cmp(f'{sd} templates', vt, e['templates'])
            cmp(f'{sd} loop edges', nl, e['loop_edges'])
        else:
            for k in ('nodes', 'templates', 'loop edges'):
                mark('file', None, f'{sd} {k}')

    # ── B Table 3 ──
    print('\n── B Table 3 (Sim(3) RMSE / drift) ' + '─' * 40)
    for sd in SETS:
        s = per_set[sd]; e = exp['table1'][sd]
        if s['gt'] is None:
            mark('Table 3', None, sd); continue
        for nm, arr, kr, kd in [('NLM', s['lo'], 'nlm_rmse', 'nlm_drift'),
                                ('EKF', s['ek'], 'ekf_rmse', 'ekf_drift'),
                                ('VO', s['vo'], 'vo_rmse', 'vo_drift')]:
            if arr is None:
                mark('Table 3', None, f'{sd} {nm}'); continue
            r, dr = sim3(arr, s['gt'])
            cmp(f'{sd} {nm} sim3 RMSE', round(r, 2), e[kr])
            cmp(f'{sd} {nm} sim3 drift', round(dr, 2), e[kd])

    # ── C Table 4 ──
    print('\n── C Table 4 (Sim(3) upper bound | SE(3) rigid) ' + '─' * 28)
    for sd in SETS:
        s = per_set[sd]; e = exp['table3'][sd]
        if s['gt'] is None:
            mark('Table 4', None, sd); continue
        cfg = [('Ours', s['full'] if (s['full'] is not None or sd.startswith('KITTI'))
                else None, 'ours_sim3', 'ours_se3', s['full'] is not None),
               ('EKF+scale', s['fph'], 'ekfs_sim3', 'ekfs_se3', s['fph'] is not None),
               ('EKF', s['ek'], 'ekf_sim3', 'ekf_se3', True),
               ('VO', s['vo'], 'vo_sim3', 'vo_se3', True)]
        if sd.startswith('KITTI'):
            # KITTI 07 无尺度支路 (S.1), 论文的 Ours 格就是闭环线
            cfg[0] = ('Ours (=loop line)', s['lo'], 'ours_sim3', 'ours_se3', True)
        for nm, arr, ks, kse, present in cfg:
            if e[ks] is None and e[kse] is None:
                mark('Table 4', None, f'{sd} {nm} (paper n/a)'); continue
            if arr is None:
                mark('Table 4', None, f'{sd} {nm} (artifact missing)'); continue
            r, _ = sim3(arr, s['gt'])
            rr, _ = se3(arr, s['gt'])
            cmp(f'{sd} {nm} Sim(3)', round(r, 2), e[ks])
            cmp(f'{sd} {nm} SE(3)', round(rr, 2), e[kse])
        if e.get('orb_sim3') is not None:
            if s['orb'] is None:
                mark('Table 4', None, f'{sd} ORB-only Sim(3) (slam_results_orbonly missing)')
            else:
                r, _ = sim3(s['orb'], s['gt'])
                cmp(f'{sd} ORB-only Sim(3)', round(r, 2), e['orb_sim3'])
                cmp(f'{sd} ORB-only SE(3)', round(se3(s['orb'], s['gt'])[0], 2), e.get('orb_se3'))

    # ── D 闭环 ──
    print('\n── D Closed-loop benchmark (Town01 turnaround) ' + '─' * 26)
    cl = exp['closed_loop']
    t1 = os.path.join(root, cl['seq'])
    need = [os.path.join(t1, 'ground_truth.txt'), os.path.join(t1, 'fusion_pose.txt'),
            os.path.join(t1, 'a_final', 'exp_trajectory.txt'),
            os.path.join(t1, 'a_final', 'exp_trajectory_preS2.txt')]
    if not all(os.path.exists(p) for p in need):
        for k in ('GT length', 'frames', 'events', 'terminal final', 'terminal EKF',
                  'terminal preS2', 'RMSE final', 'RMSE EKF', 'ratio'):
            mark('closed', None, k)
    else:
        gt, _ = read_csv_dir(root, cl['seq'], 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
        ek, _ = read_csv_dir(root, cl['seq'], 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'])
        fin = np.loadtxt(os.path.join(t1, 'a_final', 'exp_trajectory.txt'), delimiter=',', ndmin=2)[:, :3]
        pre = np.loadtxt(os.path.join(t1, 'a_final', 'exp_trajectory_preS2.txt'), delimiter=',', ndmin=2)[:, :3]
        mat = os.path.join(t1, 'a_final', 'loop_constraint_events.mat')
        if os.path.exists(mat):
            ev = np.atleast_2d(np.asarray(sio.loadmat(mat)['NLM_LOOP_EVENTS'], dtype=float))
            nev = len({int(v) for v in ev[:, 0]})
        else:
            nev = len(np.unique(np.loadtxt(os.path.join(t1, 'loop_event_frames.txt'), ndmin=1).astype(int)))
        L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
        cmp('GT length (m)', round(L, 1), cl['gt_len_m'])
        cmp('frames', len(gt), cl['frames'], 'count')
        cmp('accepted events', nev, cl['events'], 'count')
        tf, rf = anchored(fin, gt)
        te, re_ = anchored(ek, gt)
        tp, _ = anchored(pre, gt)
        cmp('terminal final (m)', round(tf, 2), cl['terminal']['final'])
        cmp('terminal EKF (m)', round(te, 2), cl['terminal']['ekf'])
        cmp('terminal preS2 (m)', round(tp, 2), cl['terminal']['pres2'])
        cmp('RMSE final (m)', round(rf, 2), cl['rmse']['final'])
        cmp('RMSE EKF (m)', round(re_, 2), cl['rmse']['ekf'])
        cmp('ratio EKF/final', round(te / tf, 2), cl['ratio'])

    # ── 汇总 ──
    print('\n' + '═' * 74)
    print(f'PASS {res["ok"]}   FAIL {res["fail"]}   SKIP {res["skip"]}')
    if res['fails']:
        print('\n不一致项:')
        for nm, got, e in res['fails']:
            print(f'  ! {nm}: got {got}, expected {e}')
    if res['skips']:
        print('\n因缺产物跳过 (不含 Table 3 的 Ours/EKF+尺度/ORB-only):')
        for nm in res['skips']:
            print(f'  - {nm}')

    if a.json:
        json.dump({'ok': res['ok'], 'fail': res['fail'], 'skip': res['skip'],
                   'fails': res['fails'], 'skips': res['skips']}, open(a.json, 'w'),
                  indent=2, ensure_ascii=False)
        print(f'\n报告已写入 {a.json}')

    if per_set[SETS[0]]['gt'] is None:
        return 2
    return 0 if res['fail'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
