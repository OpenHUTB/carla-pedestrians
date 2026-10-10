#!/usr/bin/env python3
"""kbs/compute_final_metrics.py — 论文定稿指标全量计算 (恢复EKF + S2 后, 2026-10-09)

产出:
  1) Table 1 五行 (CARLA 4 + KITTI07): NLM/EKF/VO 的 full-Sim(3) RMSE + drift rate
     + 模板/节点/闭环计数 (experiences.mat)
  2) Turnaround 三口径: fullSim(3) / anchored-Sim(3)(首100帧) / scale-locked 2D
     × 4 变体 (NLM+DC, +A-fix, +A-fix+S2, EKF)
  3) per-lap oracle (4 lap 独立对齐) 验证 S2 无局部畸变/无GT泄漏
  4) 闭环事件统计 (16 事件, gap/残差分布)
输出: neuro/kbs/final_metrics.json + 打印摘要
"""
import numpy as np, pandas as pd, scipy.io as sio, json, os, glob

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/'
OUT = {}

def sim3_full(est, gt):
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

def sim3_anchored100(est, gt):
    """首100帧锚定 Sim(3) (Turnaround 小节口径): 前100帧拟合, 全轨迹检验"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    e0, g0 = est[:100], gt[:100]
    e0c, g0c = e0 - e0.mean(0), g0 - g0.mean(0)
    U, S, Vt = np.linalg.svd(g0c.T @ e0c)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (e0c ** 2).sum()
    tvec = g0.mean(0) - s * (e0.mean(0) @ T)
    ea = s * (est @ T) + tvec
    err = np.linalg.norm(ea - gt, axis=1)
    return float(np.sqrt((err ** 2).mean()))

def scale_locked_2d(est, gt):
    """起点+总长匹配+无旋转 2D RMSE (Scale-locked 2D 口径)"""
    n = min(len(est), len(gt)); est, gt = est[:n], gt[:n]
    e, g = est - est[0], gt - gt[0]
    Le = np.linalg.norm(np.diff(e, axis=0), axis=1).sum()
    Lg = np.linalg.norm(np.diff(g, axis=0), axis=1).sum()
    s = Lg / Le if Le > 1e-9 else 1.0
    ea = e * s
    return float(np.sqrt((np.linalg.norm(ea[:, :2] - g[:, :2], axis=1) ** 2).mean())), s

def loadmat(d, var):
    m = sio.loadmat(D + d + '/slam_results/trajectories.mat')
    return np.array(m[var], dtype=float)

def loadcsv(d, fn, cols):
    p = D + d + '/' + fn
    if not os.path.exists(p):
        return None
    a = pd.read_csv(p)
    c = [x for x in cols if x in a.columns]
    return a[c].values if len(c) == 3 else None

# ─────────────────── 1) Table 1 五行 ───────────────────
t1 = {}
for d in ['Town01Data_IMU_Fusion', 'Town02Data_IMU_Fusion', 'Town05Data_IMU_Fusion',
          'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']:
    base = d + '/'
    gt = loadcsv(d, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
    nlm = loadmat(d, 'exp_trajectory')
    fu = loadcsv(d, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'])
    vo = loadcsv(d, 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z'])
    L = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    row = {'gt_len_m': round(L, 1), 'frames': int(min(len(gt), len(nlm)))}
    r, dr = sim3_full(nlm, gt); row['nlm'] = {'rmse': round(r, 2), 'drift': round(dr, 2)}
    if fu is not None and len(fu) > 50:
        r, dr = sim3_full(fu, gt); row['ekf'] = {'rmse': round(r, 2), 'drift': round(dr, 2)}
    else:
        row['ekf'] = None
    if vo is not None and len(vo) > 50:
        r, dr = sim3_full(vo, gt); row['vo'] = {'rmse': round(r, 2), 'drift': round(dr, 2)}
    else:
        row['vo'] = None
    # 模板/节点/闭环计数
    p = D + base + 'slam_results/experiences.mat'
    if os.path.exists(p):
        m = sio.loadmat(p)
        NUM = int(float(m['NUM_EXPS'].ravel()[0]))
        EXP = m['EXPERIENCES']  # (1, N) object-cell 数组, 每 cell 一个 struct
        nvt = 0; nloop = 0
        n_exp = EXP.shape[1] if EXP.ndim == 2 else EXP.shape[0]
        for k in range(n_exp):
            e = EXP[0, k] if EXP.ndim == 2 else EXP[k]
            if e.shape != ():
                e = e.ravel()[0]
            try:
                vtid = int(float(np.atleast_1d(e['vt_id']).ravel()[0]))
                nvt += 1 if vtid > 0 else 0
            except Exception:
                pass
            try:
                nl = int(float(np.atleast_1d(e['numlinks']).ravel()[0]))
                if nl > 0:
                    lnks = e['links']
                    if lnks.shape != ():
                        lnks = np.atleast_1d(lnks)
                    for j in range(len(lnks)):
                        lid = lnks[j]
                        if lid.shape != ():
                            lid = lid.ravel()[0]
                        eid = int(float(np.atleast_1d(lid['exp_id']).ravel()[0]))
                        if 1 <= eid <= k:
                            nloop += 1
            except Exception:
                pass
        row['exp_nodes'] = NUM; row['templates'] = nvt; row['loop_edges'] = nloop
    t1[d] = row

# ─────────────────── 2) Turnaround 三口径 × 4 变体 ───────────────────
td = 'Town01Turnaround_IMU_Fusion'
gt = loadcsv(td, 'ground_truth.txt', ['pos_x', 'pos_y', 'pos_z'])
fu = loadcsv(td, 'fusion_pose.txt', ['pos_x', 'pos_y', 'pos_z'])
vo = loadcsv(td, 'visual_odometry.txt', ['vo_x', 'vo_y', 'vo_z'])
n10k = min(len(gt), len(fu), len(vo))
gt, fu, vo = gt[:n10k], fu[:n10k], vo[:n10k]

def loadexp(sub, var='exp_trajectory'):
    p = D + td + f'/{sub}/trajectories.mat'
    if not os.path.exists(p):
        return None
    return np.array(sio.loadmat(p)[var], dtype=float)[:n10k]

def loadpreS2(sub):
    p = D + td + f'/{sub}/exp_trajectory_preS2.txt'
    if not os.path.exists(p):
        return None
    with open(p) as f:
        sep = ',' if ',' in f.readline() else None
    return np.loadtxt(p, delimiter=sep)[:n10k]

variants = {}
for name, tr in [('NLM_DC', loadexp('dc_final')),
                 ('NLM_AFIX', loadpreS2('a_final')),
                 ('NLM_AFIX_S2', loadexp('a_final')),
                 ('EKF', fu), ('VO', vo)]:
    if tr is None or len(tr) < 100:
        variants[name] = None; continue
    r3, dr3 = sim3_full(tr, gt)
    ra = sim3_anchored100(tr, gt)
    rs, sc = scale_locked_2d(tr, gt)
    variants[name] = {'fullSim3': round(r3, 2), 'drift': round(dr3, 2),
                      'anchoredSim3_100': round(ra, 2), 'scaleLocked2D': round(rs, 2),
                      's2_scale': round(sc, 3)}

# 闭环事件
evp = sorted(glob.glob(D + td + '/a_final/loop_constraint_events.mat'))
events = None
if evp:
    ev = sio.loadmat(evp[0])['NLM_LOOP_EVENTS']
    gaps = (ev[:, 0] - ev[:, 1]).astype(int)
    mags = np.linalg.norm(ev[:, 2:5], axis=1)
    events = {'n': int(ev.shape[0]), 'gap_min': int(gaps.min()), 'gap_max': int(gaps.max()),
              'resid_min_m': round(float(mags.min()), 2), 'resid_max_m': round(float(mags.max()), 2)}

# ─────────────────── 3) per-lap oracle ───────────────────
# lap 边界 = S2 事件帧 (16 事件 → 分段); 每段独立 Sim(3) 对齐后算 RMSE
def per_lap_oracle(tr, boundaries):
    segs = [0] + sorted(set(boundaries)) + [len(tr)]
    rms = []
    for a, b in zip(segs[:-1], segs[1:]):
        if b - a < 30:
            continue
        e, g = tr[a:b], gt[a:b]
        ec, gc = e - e.mean(0), g - g.mean(0)
        U, S, Vt = np.linalg.svd(gc.T @ ec)
        d = np.linalg.det(Vt.T @ U.T)
        T = Vt.T @ np.diag([1, 1, d]) @ U.T
        s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
        ea = s * (e @ T) + (g.mean(0) - s * (e.mean(0) @ T))
        rms.append(round(float(np.sqrt((np.linalg.norm(ea - g, axis=1) ** 2).mean())), 1))
    return rms

oracle = {}
_tr_nos2 = loadpreS2('a_final')
_tr_s2 = loadexp('a_final')
if events and _tr_nos2 is not None and _tr_s2 is not None:
    ev = sio.loadmat(evp[0])['NLM_LOOP_EVENTS']
    bounds = ev[:, 0].astype(int).tolist()
    oracle['noS2'] = per_lap_oracle(_tr_nos2, bounds)
    oracle['withS2'] = per_lap_oracle(_tr_s2, bounds)

OUT = {'table1': t1, 'turnaround': {'variants': variants, 'events': events, 'oracle': oracle}}
with open('/home/yangrb/openhutb/neuro/kbs/final_metrics.json', 'w') as f:
    json.dump(OUT, f, indent=2, ensure_ascii=False)

# ─────────────────── 打印 ───────────────────
print('════ Table 1 (full Sim(3), 恢复EKF + NLM重跑) ════')
print(f"{'set':24s} {'NLM':>7s} {'EKF':>7s} {'VO':>7s} | {'drift NLM/EKF':>14s} | nodes/tpl/loop | GTlen")
for d, r in t1.items():
    n = r['nlm']; e = r.get('ekf'); v = r.get('vo')
    ns = f"{n['rmse']:7.2f}"
    es = f"{e['rmse']:7.2f}" if e else '   ---'
    vs = f"{v['rmse']:7.2f}" if v else '   ---'
    ddr = f"{n['drift']}/{e['drift']}" if e else f"{n['drift']}/-"
    cnt = f"{r.get('exp_nodes','?')}/{r.get('templates','?')}/{r.get('loop_edges','?')}"
    print(f"{d:24s} {ns} {es} {vs} | {ddr:>14s} | {cnt:>14s} | {r['gt_len_m']}m")
print()
print('════ Turnaround 三口径 (10000帧) ═════')
print(f"{'variant':14s} {'fullSim3':>9s} {'anch100':>9s} {'scale2D':>9s} {'s2_scale':>9s}")
for name, v in variants.items():
    if v is None:
        print(f"{name:14s} (missing)"); continue
    print(f"{name:14s} {v['fullSim3']:9.2f} {v['anchoredSim3_100']:9.2f} {v['scaleLocked2D']:9.2f} {v['s2_scale']:9.3f}")
print()
print('════ 闭环事件 ═════', events)
print('════ per-lap oracle ═════', oracle)
print('\n→ 已写入 neuro/kbs/final_metrics.json')
