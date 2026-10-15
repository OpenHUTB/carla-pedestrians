#!/usr/bin/env python3
"""diag_pr_gates.py — 离线复刻 MATLAB PR 检测器门链 + oracle 真环逐环归因

目的 (#24): 定位 Town07 召回/精度缺口 — 10 个真环哪些被哪个门杀掉, 还是模型没认出。
MATLAB 语义 (core L541-638, 1-based 帧号):
- PR_ODO_ARC = fusion_data.pos (EKF-odo) 累计距离; 窗 w = max{j<i : arc[j] <= arc[i]-50}
- 候选 j* = argmax_j sim(i,j), s* = sim(i,j*) (嵌入 L2 归一, 点积=余弦)
- 持续性门 (仅 s*>=τ 时更新): j*==last 或 |j*-last|<=10 → streak+=1 否则 1; s<τ 重置
- 事件: s*>=0.9 && streak>=15 && (i-last_ev>=30) && 航向门(|dyaw|<=60° 或反向) && i-j*>=300
- 命中 → last_ev = i
Oracle (pr_eval.py 口径): GT R=5m, GT弧Δ>50m, merge_gap=30
用法: python diag_pr_gates.py [Set1 Set2 ...]
"""
import sys

import numpy as np
import pandas as pd
import scipy.io as sio

D = '/home/yangrb/openhutb/neuro/data'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', '01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion']
TAU, STABLE_MIN, MIN_GAP, LOOP_MIN_GAP, ARC_GAP, YAW_GATE = 0.9, 15, 30, 300, 50.0, 60.0


def signed_delta_deg(a, b):
    return ((a - b + 180.0) % 360.0) - 180.0


def load(sub):
    d = f'{D}/{sub}'
    fp = pd.read_csv(f'{d}/fusion_pose.txt').values
    gt = pd.read_csv(f'{d}/ground_truth.txt').values
    m = sio.loadmat(f'{d}/slam_results/pr_embeddings_ft.mat')
    emb = np.asarray(m['pr_embeddings'], dtype=np.float32)
    if emb.shape[0] != len(gt) and emb.shape[1] == len(gt):
        emb = emb.T
    return fp[:, 1:4], fp[:, 6], gt[:, 1:4], emb


def simulate(emb, odo_pos, odo_yaw):
    N = len(odo_pos)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(odo_pos, axis=0), axis=1))])
    S = emb @ emb.T
    jstar = np.zeros(N, dtype=int)   # 0-based 候选 (-1 = 无窗)
    sstar = np.full(N, -1.0)
    streak_arr = np.zeros(N, dtype=int)
    events = []                       # (i, j) 1-based
    last = -1
    streak = 0
    last_ev = 0
    for i in range(1, N + 1):
        w0 = np.searchsorted(arc, arc[i - 1] - ARC_GAP, side='right') - 1
        if w0 >= 0:
            row = S[i - 1, :w0 + 1]
            j0 = int(np.argmax(row))
            jstar[i - 1], sstar[i - 1] = j0, float(row[j0])
            j = j0 + 1
            s = sstar[i - 1]
        else:
            j, s = -1, -1.0
        if s >= TAU:
            streak = streak + 1 if (j == last or abs(j - last) <= 10) else 1
            last = j
        else:
            last, streak = -1, 0
        streak_arr[i - 1] = streak
        if s >= TAU and streak >= STABLE_MIN and (last_ev == 0 or i - last_ev >= MIN_GAP):
            dyaw = signed_delta_deg(odo_yaw[i - 1], odo_yaw[j - 1])
            yaw_pass = abs(dyaw) <= YAW_GATE or abs(abs(dyaw) - 180.0) <= YAW_GATE
            if yaw_pass and i - j >= LOOP_MIN_GAP:
                events.append((i, j))
                last_ev = i
    return events, jstar, sstar, streak_arr, arc


def oracle_events(p, R=5.0, D_arc=ARC_GAP, merge_gap=30):
    N = len(p)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    pairs, BLK = [], 500
    for s_ in range(0, N, BLK):
        ii = np.arange(s_, min(s_ + BLK, N))
        for j0 in range(0, s_, BLK):
            jj = np.arange(j0, min(j0 + BLK, s_))
            d = np.linalg.norm(p[ii, None, :] - p[None, jj, :], axis=2)
            m = (d < R) & (arc[ii, None] - arc[None, jj] > D_arc)
            r, c = np.nonzero(m)
            pairs += list(zip(jj[c].tolist(), ii[r].tolist()))
    ev = []
    for j, i in sorted(pairs):
        if ev and i - ev[-1][1] <= merge_gap and j - ev[-1][0] <= merge_gap:
            ev[-1] = (min(ev[-1][0], j), i)
        else:
            ev.append((j, i))
    return ev


def match_tp(ev, oracle, p, tol=10.0):
    used = set()
    for (j, i) in ev:
        for m_, (jo, io) in enumerate(oracle):
            if m_ in used:
                continue
            if np.linalg.norm(p[i] - p[io]) < tol and np.linalg.norm(p[j] - p[jo]) < tol:
                used.add(m_)
                break
    return len(used)


def main():
    subs = sys.argv[1:] or SETS
    for sub in subs:
        odo_pos, odo_yaw, gt, emb = load(sub)
        ev, jstar, sstar, streak_arr, arc = simulate(emb, odo_pos, odo_yaw)
        oracle = oracle_events(gt)
        tp = match_tp(ev, oracle, gt)
        print(f'\n{"=" * 78}\n=== {sub} ===')
        print(f'模拟器事件={len(ev)}  TP={tp} FP={len(ev) - tp}  oracle环={len(oracle)}')
        for i, j in ev:
            print(f'  事件 (i={i}, j={j}) s*={sstar[i - 1]:.3f} dist_gt={np.linalg.norm(gt[i - 1] - gt[j - 1]):6.1f}m')
        # ---- 逐 oracle 环归因 ----
        print(f'-- oracle 真环归因 (io=回访帧, jo=旧帧) --')
        for m_, (jo, io) in enumerate(oracle):
            i, j = io, jstar[io - 1] + 1
            s = sstar[io - 1]
            in_ev = [e for e in ev if abs(e[0] - io) <= 30]
            if in_ev:
                e_i, e_j = in_ev[0]
                is_tp = np.linalg.norm(gt[io - 1] - gt[e_i - 1]) < 10 and np.linalg.norm(gt[jo - 1] - gt[e_j - 1]) < 10
                print(f'  #{m_ + 1} (io={io}, jo={jo}): [已检测{" TP" if is_tp else " FP"}] 事件(i={e_i},j={e_j})')
                continue
            d_j = np.linalg.norm(gt[i - 1] - gt[j - 1])
            why = []
            if s < TAU:
                why.append(f's*={s:.3f}<0.9')
            if streak_arr[io - 1] < STABLE_MIN:
                why.append(f'streak={streak_arr[io - 1]}<15')
            dyaw = signed_delta_deg(odo_yaw[io - 1], odo_yaw[j - 1])
            if abs(dyaw) > YAW_GATE and abs(abs(dyaw) - 180) > YAW_GATE:
                why.append(f'航向 {dyaw:.0f}°')
            if io - j < LOOP_MIN_GAP:
                why.append(f'帧差 {io - j}<300')
            S_row = (emb @ emb.T)[io - 1]
            w0 = np.searchsorted(arc, arc[io - 1] - ARC_GAP, side='right') - 1
            top3 = np.argsort(S_row[:w0 + 1])[::-1][:3] if w0 >= 0 else []
            t3 = '  '.join(f'f{t + 1}(s={S_row[t]:.2f},d={np.linalg.norm(gt[io - 1] - gt[t]):.0f}m,Δjo={np.linalg.norm(gt[t + 1] - gt[jo - 1]):.0f}m)' for t in top3)
            print(f'  #{m_ + 1} (io={io}, jo={jo}): [漏] j*={j}(s*={s:.3f},dist={d_j:.1f}m) 门杀: {",".join(why) or "无(去抖?)"}')
            print(f'       top3旧帧: {t3}')


if __name__ == '__main__':
    main()
