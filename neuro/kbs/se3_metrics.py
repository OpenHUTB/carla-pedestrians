#!/usr/bin/env python3
"""kbs/se3_metrics.py — SE(3) 刚性 ATE 全矩阵 (2026-10-15, 论文度量主张口径)

目的:
  1) SE(3) 刚性 ATE (旋转+平移对齐, 不重拟合全局尺度) — "无 RTK 度量 SLAM" 主张的诚实口径;
     sim3_full 会把全局尺度重新拟合掉, 尺度校正的全局分量在其中不可见。
  2) 同口径输出 sim3_full, 与 final_metrics.json 规范数字交叉校验 (必须逐位一致)。
  3) 输出 sim3_full 拟合的全局尺度因子 s — 量化 "评测吸收了多大全局尺度误差"。
配置 (每集, 存在即算):
  VO / EKF / EKF+尺度 (fp_head030, P2 尺度头 replay db=0.30, 无闭环) /
  NLM-闭环线 (slam_results, τ0.94 规范) / NLM-全系统 (slam_results_scale_nlm) /
  ORB-only (slam_results_orbonly, 消融)
输出: neuro/kbs/se3_metrics.json + 打印摘要
"""
import numpy as np, pandas as pd, scipy.io as sio, json, os

# 锚定到脚本所在目录: 本脚本含 ../ 相对路径, 需保证从任意 cwd 调用均正确
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# 数据根可覆盖: NLM_DATA_ROOT=/path/01_NeuroSLAM_Datasets (默认 ../data/01_NeuroSLAM_Datasets)
D  = os.environ.get('NLM_DATA_ROOT', '../data/01_NeuroSLAM_Datasets').rstrip('/') + '/'
BK = os.environ.get('NLM_BACKUP_ROOT', '../data/_p11_backup_20261014').rstrip('/') + '/'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion', 'KITTI09Data_IMU_Fusion']
SHORT = {'Town05Data_IMU_Fusion': 'Town05', 'Town03Data_IMU_Fusion': 'Town03',
         'Town07Data_IMU_Fusion': 'Town07', 'Town10HDData_IMU_Fusion': 'Town10HD'}

def se3_rigid(est, gt):
    """SE(3) 刚性对齐 (Kabsch, 无尺度)。返回 (rmse, drift%)。"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    ea = est @ R + (gt.mean(0) - (est @ R).mean(0))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100)

def sim3_full(est, gt):
    """full Sim(3) 对齐 (旋转+全局尺度+平移)。返回 (rmse, drift%, 拟合尺度 s)。"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    ea = s * (est @ T) + (gt.mean(0) - s * (est.mean(0) @ T))
    err = np.linalg.norm(ea - gt, axis=1)
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    return float(np.sqrt((err ** 2).mean())), float(err[-1] / L * 100), float(s)

def loadcsv(d, fn, cols):
    p = d + fn
    if not os.path.exists(p):
        return None
    a = pd.read_csv(p)
    c = [x for x in cols if x in a.columns]
    return a[c].values.astype(float) if len(c) == 3 else None

def loadmat(d, fn, var):
    p = d + fn
    if not os.path.exists(p):
        return None
    try:
        return np.array(sio.loadmat(p)[var], dtype=float)
    except Exception:
        return None

# ─────────────────── 全矩阵 ───────────────────
OUT = {}
for sd in SETS:
    base = D + sd + '/'
    gt = loadcsv(base, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
    if gt is None:
        print(f'MISSING GT: {sd}')
        continue
    cfg = {
        'VO': loadcsv(base, 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z']),
        'EKF': loadcsv(base, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z']),
        'EKF+尺度': None,
        'NLM-闭环线': loadmat(base, 'slam_results/trajectories.mat', 'exp_trajectory'),
        'NLM-全系统': loadmat(base, 'slam_results_scale_nlm/trajectories.mat', 'exp_trajectory'),
        'ORB-only': loadmat(base, 'slam_results_orbonly/trajectories.mat', 'exp_trajectory'),
    }
    if cfg['VO'] is None:   # KITTI VO 在 raw 转换目录 (有头 vo_x/y/z)
        cfg['VO'] = loadcsv(D + '../KITTI_07/', 'visual_odometry_ekf.txt', ['vo_x', 'vo_y', 'vo_z'])
    if sd in SHORT:         # EKF+尺度基座 (P2 尺度头 replay fp_head030, 无 NLM)
        cfg['EKF+尺度'] = loadcsv(BK + SHORT[sd] + '/', 'fp_head030.txt', ['pos_x', 'pos_y', 'pos_z'])
    row = {'gt_len_m': round(float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()), 1)}
    for name, traj in cfg.items():
        if traj is None or len(traj) < 50:
            continue
        r_s, dr_s = se3_rigid(traj, gt)
        r_f, dr_f, s_f = sim3_full(traj, gt)
        row[name] = {'se3_rmse': round(r_s, 2), 'se3_drift': round(dr_s, 2),
                     'sim3_rmse': round(r_f, 2), 'sim3_drift': round(dr_f, 2),
                     'sim3_fitted_scale': round(s_f, 4)}
    OUT[sd] = row

# ── 缺失产物的列: 从上一版 JSON 结转, 并打 carried=True 标记 (不静默丢行/丢集) ──
PREV = {}
JSN = '../kbs/se3_metrics.json'
if os.path.exists(JSN):
    try:
        PREV = json.load(open(JSN))
    except Exception:
        PREV = {}
SNAP = {k: v for k, v in PREV.items() if not k.startswith('_')}
for sd, prow in SNAP.items():
    row = OUT.get(sd)
    if row is None:                      # 整集没有数据 (如 KITTI09): 整行结转
        carried = {k: (dict(v, carried=True) if isinstance(v, dict) else v)
                   for k, v in prow.items() if k != 'gt_len_m'}
        OUT[sd] = {'gt_len_m': prow.get('gt_len_m'), **carried, 'carried_row': True}
        continue
    for cfg, v in prow.items():
        if cfg == 'gt_len_m' or not isinstance(v, dict):
            continue
        if cfg not in row:               # 该配置的产物不在数据根里
            row[cfg] = dict(v, carried=True)

OUT['_meta'] = {
    'calibers': {
        'sim3_rmse': 'full Sim(3) refit (shape-only upper bound)',
        'se3_rmse': 'SE(3) rigid Kabsch, no scale (metric-scale caliber)',
        'drift': 'terminal error / trajectory length (%)',
    },
    'data_root': D,
    'note': 'Keys computed from the data root are plain. Keys with "carried": true were '
            'computed in an earlier batch and their artifact is NOT under the data root '
            '(EKF+尺度 needs _p11_backup_20261014/<seq>/fp_head030.txt, NLM-全系统 needs '
            'slam_results_scale_nlm/, ORB-only needs slam_results_orbonly/); they are '
            'carried over from the previous JSON so that Table 3 stays complete, and are '
            'replaced the moment the artifact is present.',
}
present, carried = set(), set()
for sd, row in OUT.items():
    if sd.startswith('_') or not isinstance(row, dict):
        continue
    for k, v in row.items():
        if k in ('gt_len_m', 'carried_row') or not isinstance(v, dict):
            continue
        (carried if v.get('carried') or row.get('carried_row') else present).add(f'{sd}/{k}')
OUT['_meta']['configs_computed'] = sorted(present)
OUT['_meta']['configs_carried'] = sorted(carried)
OUT['_meta']['table3_notes'] = [
    'KITTI07 Ours cell (18.45 / 29.99) equals NLM-闭环线: the scale branch is inactive '
    'zero-shot on real imagery (Supplementary S.1), so the full system there IS the loop line.',
    'Town10HD Ours == EKF+尺度 (14.44): the single accepted loop link is inert on the open layout.',
    'Table 3 EKF+尺度 and ORB-only cells come from artifacts outside the released asset bundle '
    'unless export_repro_bundle.py has been run over the original run tree.',
]

with open('../kbs/se3_metrics.json', 'w') as f:
    json.dump(OUT, f, indent=2, ensure_ascii=False)

# ─────────────────── 打印 ───────────────────
CMPS = ['VO', 'EKF', 'EKF+尺度', 'NLM-闭环线', 'NLM-全系统', 'ORB-only']
print('=' * 96)
for sd, row in OUT.items():
    if sd.startswith('_'):
        continue
    print(f"\n### {sd}  (GT {row['gt_len_m']}m)")
    print(f"{'配置':12s} {'SE(3)刚性':>10s} {'drift%':>8s} | {'sim3_full':>10s} {'drift%':>8s} {'拟合s':>9s}")
    for c in CMPS:
        if c in row:
            m = row[c]
            print(f"{c:12s} {m['se3_rmse']:10.2f} {m['se3_drift']:8.2f} | "
                  f"{m['sim3_rmse']:10.2f} {m['sim3_drift']:8.2f} {m['sim3_fitted_scale']:9.4f}")
print('\n→ 已写入 neuro/kbs/se3_metrics.json')

# ─────────────────── 规范交叉校验 (sim3_full 必须与 final_metrics 逐位一致) ───────────────────
FM = json.load(open('../kbs/final_metrics.json'))
t1 = FM['table1']
print('\n── 与 final_metrics.json 交叉校验 (sim3_full 应一致) ──')
ok = True
for sd, row in OUT.items():
    if sd.startswith('_') or sd not in t1:
        continue  # 补充材料数据集 (如 KITTI09) 不在主表, 跳过交叉
    for name, key in [('NLM-闭环线', 'nlm'), ('EKF', 'ekf'), ('VO', 'vo')]:
        ref = t1[sd].get(key)
        if name in row and ref:
            a, b = row[name]['sim3_rmse'], ref['rmse']
            mark = 'OK' if abs(a - b) < 0.011 else 'MISMATCH'
            if mark != 'OK':
                ok = False
            print(f'  {sd:28s} {name:10s} {a:8.2f} vs {b:8.2f}  {mark}')
print('CROSS-CHECK', 'PASS' if ok else 'FAIL')
