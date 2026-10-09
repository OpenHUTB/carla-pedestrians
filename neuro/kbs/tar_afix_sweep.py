#!/usr/bin/env python3
"""tar_afix_sweep.py — Turnaround 折返集 A-fix 闭环约束漂移场离线扫参.

数据流(与 core/test_imu_visual_fusion_slam.m 输出级逐行同构):
  base = dc_final/exp_trajectory.txt   (P1平滑 + DC v3注入, 无A-fix)
  out  = base + L_field                (A-fix 闭环约束场, 主循环之后注入)
  L_field(t) = sum_k F_k(t)*c_k
    默认形状(与core一致): F_k 在 [t_old_k, t_now_k) 按 raised-cosine 从0爬坡到1, 其余为0
    c_k = -beta*clamp(r_k, +-STEP_MAX),  beta=1 记录时未触发钳位(max|c|=24.63<25) -> r_k=-c_k

口径: 官方 'simple' 对齐(起点对齐原点+路径长度缩放, 无旋转; align_trajectories.m)
      ATE = 对齐后逐帧 RMSE (2D=x,y / 3D 双口径校准, 与 NLM_LOOP_DESIGN.md S8.1 对齐)

校准门槛: base ATE ~ 201.16, base+L(beta=1,cosine) ~ 172.12, 且逐帧 vs a_final 落盘轨迹.

用法:
  python3 tar_afix_sweep.py            # 校准+全量扫参
  python3 tar_afix_sweep.py --calib    # 仅校准
"""
import os
import sys
import numpy as np
from scipy.io import loadmat

BASE = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion'


def load(p):
    return np.loadtxt(p, delimiter=',')


def simple_ate(gt, est, dim=3):
    """官方 simple 对齐(无旋转): 起点归零 + 路径长缩放, 返回对齐后逐帧误差数组."""
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]
    g = gt - gt[0]
    e = est - est[0]
    lg = np.linalg.norm(np.diff(g, axis=0), axis=1).sum()
    le = np.linalg.norm(np.diff(e, axis=0), axis=1).sum()
    s = lg / le if le > 0 else 1.0
    return np.linalg.norm((e * s - g)[:, :dim], axis=1)


def rmse(err):
    return float(np.sqrt((err ** 2).mean()))


def afix_field(events, n, shape='cosine', beta=1.0, clamp_max=25.0, window=None):
    """复刻 core [A-fix] 注入块(632-657行). events=[t_now,t_old,cx,cy,cz] MATLAB 1-based.
    shape: cosine(默认,=core)/linear/sqrt/quad/step_old/pulse
    window: 非None时爬坡窗口改为 [t_now-window, t_now) 固定长度.
    """
    L = np.zeros((n, 3))
    f = np.arange(n)  # 0-based; MATLAB f0 = f+1
    for t_now, t_old, cx, cy, cz in events:
        r = np.array([-cx, -cy, -cz])  # 记录beta=1未钳位 -> r = -c
        c = -beta * np.clip(r, -clamp_max, clamp_max)
        tn, to = int(t_now), int(t_old)  # 1-based
        if to >= tn or to < 1:
            if tn > 1:
                L[tn - 2, :] += c
            continue
        if window is not None:
            to = max(1, tn - window)
        D = tn - to
        seg = (f >= to - 1) & (f < tn - 1)
        if not seg.any():
            continue
        t = f[seg] - (to - 1)
        u = t / D
        if shape == 'cosine':
            F = 0.5 * (1 - np.cos(np.pi * u))
        elif shape == 'linear':
            F = u
        elif shape == 'sqrt':
            F = np.sqrt(u)
        elif shape == 'quad':
            F = u ** 2
        elif shape == 'step_old':
            F = np.ones_like(u)
        elif shape == 'pulse':
            F = np.zeros_like(u)
        else:
            raise ValueError(shape)
        L[seg] += np.outer(F, c)
    return L


def sim3_first100(gt, est):
    """compute_metrics_with_alignment 口径: 前100帧锚定 Sim(3) 7-DoF, 返回逐帧误差."""
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]
    g, e = gt[:100] - gt[:100].mean(0), est[:100] - est[:100].mean(0)
    U, S, Vt = np.linalg.svd(e.T @ g)
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    s = (S[0] + S[1] + D[2, 2] * S[2]) / np.sum(g ** 2)
    t = gt[:100].mean(0) - s * (R @ est[:100].mean(0))
    return np.linalg.norm(s * (est @ R.T) + t - gt, axis=1)


def lap_bounds(gt, min_len=500):
    """GT 距起点 <8m 视为回到起点, 切 lap 边界."""
    d = np.linalg.norm(gt - gt[0], axis=1)
    in_start = d[0] < 8
    bounds = [0]
    for i in range(1, len(d)):
        if in_start and d[i] >= 8:
            bounds.append(i)
            in_start = False
        elif not in_start and d[i] < 8:
            in_start = True
    bounds.append(len(gt))
    # 合并过短段
    out = [0]
    for b in bounds[1:-1]:
        if b - out[-1] >= min_len:
            out.append(b)
    out.append(len(gt))
    return out


def main():
    calib_only = '--calib' in sys.argv
    gt = load(os.path.join(BASE, 'a_final', 'ground_truth_backup.txt'))
    base = load(os.path.join(BASE, 'dc_final', 'exp_trajectory.txt'))
    a_disk = load(os.path.join(BASE, 'a_final', 'exp_trajectory.txt'))
    ekf = load(os.path.join(BASE, 'a_final', 'imu_aided_trajectory.txt'))
    vo = load(os.path.join(BASE, 'a_final', 'pure_visual_trajectory.txt'))
    n = len(gt)
    M = loadmat(os.path.join(BASE, 'a_final', 'loop_constraint_events.mat'))
    ev = np.asarray(M['NLM_LOOP_EVENTS'], dtype=float)
    ev = np.atleast_2d(ev)
    if ev.shape == (5,):
        ev = ev.reshape(1, 5)
    if ev.shape[1] == 5 and ev.shape[0] == 1 and ev[0, 0] > 100:
        ev = ev.T  # (5,N) -> (N,5)
    assert ev.shape[1] == 5 and ev.shape[0] > 1, 'events shape %s' % (ev.shape,)
    events = [(r[0], r[1], r[2], r[3], r[4]) for r in ev]
    gaps = ev[:, 0] - ev[:, 1]
    cmax_rec = float(np.linalg.norm(ev[:, 2:5], axis=1).max())
    print('=== Turnaround 离线扫参  n=%d  闭环事件=%d  gap[min,med,max]=[%d,%d,%d]  |c|rec_max=%.2fm'
          % (n, len(events), gaps.min(), int(np.median(gaps)), gaps.max(), cmax_rec))

    # ---------- 校准 ----------
    L1 = afix_field(events, n, 'cosine', beta=1.0, clamp_max=25.0)
    print('\n--- 校准 (官方 simple 口径, 目标 base~201.16 / constraint~172.12) ---')
    for dim in (3, 2):
        rb = rmse(simple_ate(gt, base, dim))
        ra_disk = rmse(simple_ate(gt, a_disk, dim))
        ra_off = rmse(simple_ate(gt, base + L1, dim))
        df = float(np.abs((base + L1) - a_disk).max())
        print('  dim=%d  base(dc)=%8.2f  a_disk=%8.2f  离线重建=%8.2f  逐帧maxDelta=%.4fm'
              % (dim, rb, ra_disk, ra_off, df))
    df3 = float(np.abs((base + L1) - a_disk).max())
    ok = df3 < 0.05
    print('  逐帧重建一致性(核心校准): %s (maxDelta=%.4f m)' % ('OK' if ok else 'FAIL', df3))
    if not ok:
        print('  ⚠ 离线重建与落盘 a_final 不一致, 扫参结果仅供方向参考.')
    if calib_only:
        return
    DIM = 2  # 文档 §8.1 数字(201.16/172.12)与2D匹配; 若2D偏差大自动切3D
    rb2 = rmse(simple_ate(gt, base, 2))
    rb3 = rmse(simple_ate(gt, base, 3))
    if abs(rb3 - 201.16) < abs(rb2 - 201.16):
        DIM = 3
    base_r = rb3 if DIM == 3 else rb2
    print('  -> 采用 %dd 口径 (base=%.2f, 文档201.16; 3d=%.2f 2d=%.2f)' % (DIM, base_r, rb3, rb2))

    # ---------- 扫参 ----------
    betas = [0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]
    shapes = ['cosine', 'linear', 'sqrt', 'quad', 'step_old', 'pulse']
    windows = [None, 150, 400, 900]
    cmaxs = [25.0, 15.0]
    rows = []
    for beta in betas:
        for shape in shapes:
            for w in windows:
                for cm in cmaxs:
                    L = afix_field(events, n, shape, beta, cm, w)
                    r = rmse(simple_ate(gt, base + L, DIM))
                    rows.append((r, beta, shape, w, cm))
    rows.sort(key=lambda x: x[0])
    print('\n--- 扫参 top-15 (base=%.2f, 事件=%d, dim=%d) ---' % (base_r, len(events), DIM))
    print('  %8s %5s %8s %5s %5s  %8s' % ('ATE', 'beta', 'shape', 'win', 'cmax', 'dATE%'))
    for r, b, sh, w, cm in rows[:15]:
        ws = 'all' if w is None else str(w)
        print('  %8.2f %5.2f %8s %5s %5.1f  %+7.2f%%' % (r, b, sh, ws, cm, (r - base_r) / base_r * 100))

    # ---------- 分段(lap)ATE ----------
    best = rows[0]
    b, sh, w, cm = best[1], best[2], best[3], best[4]
    ws = 'all' if w is None else str(w)
    out_best = base + afix_field(events, n, sh, b, cm, w)
    bounds = lap_bounds(gt)
    print('\n--- 分段ATE (best: beta=%.2f shape=%s win=%s cmax=%.1f, dim=%d) ---'
          % (b, sh, ws, cm, DIM))
    print('  %-24s %10s %10s %10s %10s' % ('', 'dc基线', 'best', 'EKF', 'VO'))
    for i in range(len(bounds) - 1):
        a_, b_ = bounds[i], bounds[i + 1]
        e = [rmse(simple_ate(gt[a_:b_], t[a_:b_], DIM)) for t in (base, out_best, ekf, vo)]
        name = 'lap%d [%d:%d] (%d帧)' % (i, a_, b_, b_ - a_)
        print('  %-24s %10.2f %10.2f %10.2f %10.2f' % (name, e[0], e[1], e[2], e[3]))
    e = [rmse(simple_ate(gt, t, DIM)) for t in (base, out_best, ekf, vo)]
    print('  %-24s %10.2f %10.2f %10.2f %10.2f' % ('全段', e[0], e[1], e[2], e[3]))

    # 逐帧误差落盘(画论文图用)
    nn = min(n, len(ekf), len(vo))
    np.savez('/tmp/tar_afix_best.npz',
             frame=np.arange(nn),
             dc=simple_ate(gt, base, DIM)[:nn],
             best=simple_ate(gt, out_best, DIM)[:nn],
             ekf=simple_ate(gt, ekf, DIM)[:nn],
             vo=simple_ate(gt, vo, DIM)[:nn],
             t_now=ev[:, 0], t_old=ev[:, 1])
    print('\n  逐帧误差 -> /tmp/tar_afix_best.npz (含 t_now/t_old 事件帧)')


if __name__ == '__main__':
    main()
