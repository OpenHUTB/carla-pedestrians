#!/usr/bin/env python3
"""afix_prototype.py — A-fix 字段形态原型 (Python, 不跑 MATLAB)

背景 (#23): 约束模式下去硬重锚定后, 内部轨迹在 t_now 处不再跳变 (零地图反馈),
但 A-fix 字段仍是旧形态: 只在 [t_old, t_now) 爬坡, t_now 之后归零 →
闭环残差在尾部 (漂移最大处) 从未被校正 = NLM 不提效的根因。
另外多事件对同一场所重复注入 → 叠加触顶 35m 过冲 (Town07/10HD 负值来源)。

方法: 从已提交的 p2ft 产物反解 raw 内部轨迹 (raw = final - L_old),
再在 raw 上试新形态, 直接算 sim3_full ATE:
  A  现形态:    爬坡 [t_old,t_now), 之后 0; 逐帧去重; cap35
  B  hold-c:    爬坡后保持 c 至轨迹末; 逐帧去重; cap35
  C  场所15:    现形态 + 场所去重 (旧帧位置<=15m 同场所, 留最早t_now); cap35
  D  hold+15:   hold-c + 场所15 (留最早)
  E  hold+15s:  hold-c + 场所15 (留|c|最大)
  F  15s:       现形态 + 场所15 (留|c|最大)

口径: MATLAB 1-based 帧号; 旧帧地图位置 = fusion_pose 行 t_old (EKF-odo 系)。
校验: raw=final-L_old 的 ATE 应≈EKF 基线 (raw 即内部 EKF-odo 轨迹)。
用法: python afix_prototype.py
"""
import numpy as np
import pandas as pd
import scipy.io as sio

ROOT = '/home/yangrb/openhutb/neuro/data'
SETS = [
    ('Town05Data_IMU_Fusion', 'slam_results_p2ft', 44.44),
    ('Town03Data_IMU_Fusion', 'slam_results_p2ft', 23.36),
    ('Town07Data_IMU_Fusion', 'slam_results_p2ft', 40.64),
    ('Town10HDData_IMU_Fusion', 'slam_results_p2ft', 18.61),
    ('01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion', 'slam_results_p2ft', 22.09),
]
CAP = 35.0
VARIANTS = [
    ('A   τ0.90', dict(hold=False, place_r=0.0, keep='first', pass_gap=0, prad=0.0, smin=0.90)),
    ('S94 τ0.94', dict(hold=False, place_r=0.0, keep='first', pass_gap=0, prad=0.0, smin=0.94)),
    ('S95 τ0.95', dict(hold=False, place_r=0.0, keep='first', pass_gap=0, prad=0.0, smin=0.95)),
    ('S96 τ0.96', dict(hold=False, place_r=0.0, keep='first', pass_gap=0, prad=0.0, smin=0.96)),
]


def sim3_full(est, gt):
    n = min(len(est), len(gt))
    est, gt = est[:n], gt[:n]
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    d = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, d]) @ U.T
    s = (S @ np.array([1, 1, d])) / (ec ** 2).sum()
    ea = s * (est @ T) + (gt.mean(0) - s * (est.mean(0) @ T))
    return float(np.sqrt(np.mean(np.sum((ea - gt) ** 2, axis=1))))


def field_from_events(n, evs, pos, emb=None, hold=False, place_r=0.0, cap=CAP,
                      keep='first', pass_gap=0, prad=0.0, smin=0.0):
    """evs: [t_now, t_old, c] 1-based。pos: fusion_pose (0-based 行)。emb: 嵌入。
    管线: 相似度门 (smin>0, 重算 emb 点积 >= smin) → 半径门 (prad>0)
    → 逐帧旧帧去重 → 场所去重 (place_r>0) → 通过级去重 (pass_gap>0) → 注入 (形态 + cap)。"""
    # ---- 相似度门 (τ 收紧): 用嵌入重算事件相似度 ----
    gated = []
    for t_now, t_old, c in evs:
        if smin > 0 and emb is not None and 1 <= t_old <= n and t_old < t_now:
            s = float(emb[t_now - 1] @ emb[t_old - 1])
            if s < smin:
                continue
        gated.append((t_now, t_old, c))
    evs = gated
    # ---- 半径门 (PR_LOOP_RADIUS): 当前帧 vs 旧帧 地图距离 ----
    if prad > 0:
        gated2 = []
        for t_now, t_old, c in evs:
            if 1 <= t_old <= n and t_old < t_now:
                d2d = float(np.linalg.norm(pos[t_now - 1, :2] - pos[t_old - 1, :2]))
                if d2d > prad:
                    continue
            gated2.append((t_now, t_old, c))
        evs = gated2
    # ---- 逐帧旧帧去重 (同一旧帧只留首个事件, 同 MATLAB 定解) ----
    seen_old = set()
    kept = []
    for t_now, t_old, c in evs:
        if t_old in seen_old:
            continue
        seen_old.add(t_old)
        kept.append((t_now, t_old, c))
    # ---- 场所级去重 ----
    if place_r > 0:
        clusters, passthru = [], []
        for t_now, t_old, c in kept:
            if t_old < 1 or t_old > n or t_old >= t_now:
                passthru.append((t_now, t_old, c))
                continue
            po = pos[t_old - 1, :3]
            hit = None
            for cl in clusters:
                if np.linalg.norm(po - cl[0]) <= place_r:
                    hit = cl
                    break
            if hit is not None:
                hit.append((t_now, t_old, c))
            else:
                clusters.append([(t_now, t_old, c)])
        kept = passthru
        for cl in clusters:
            if keep == 'strongest':
                best = max(cl, key=lambda e: np.linalg.norm(e[2]))
            else:
                best = min(cl, key=lambda e: e[0])
            kept.append(best)
    # ---- 通过级去重: 同一次重访通过里的多个闭环测量同一残差, 只留一个 ----
    if pass_gap > 0:
        kept = sorted(kept, key=lambda e: e[0])
        clusters = []
        for e_ in kept:
            if clusters and e_[0] - clusters[-1][-1][0] <= pass_gap:
                clusters[-1].append(e_)
            else:
                clusters.append([e_])
        new_kept = []
        for cl in clusters:
            if keep == 'strongest' and emb is not None:
                best = max(cl, key=lambda e_: float(emb[e_[0] - 1] @ emb[e_[1] - 1]))
            else:
                best = cl[0]
            new_kept.append(best)
        kept = new_kept
    # ---- 注入 (形态 + cap) ----
    L = np.zeros((n, 3))
    seen_old2 = set()
    for t_now, t_old, c in kept:
        if t_old in seen_old2:
            continue
        seen_old2.add(t_old)
        if t_old < 1 or t_old >= t_now:
            if t_now > 1:
                L[t_now - 2] += c
            continue
        a, b = t_old - 1, t_now - 1  # 0-based [a, b)
        t = np.arange(a, b) - a
        F = 0.5 * (1 - np.cos(np.pi * t / (t_now - t_old)))
        L[a:b] += F[:, None] * c
        if hold and b < n:
            L[b:] += c
    # ---- 上限 ----
    norm = np.linalg.norm(L, axis=1)
    over = norm > cap
    L[over] *= (cap / norm[over])[:, None]
    return L, len(evs), len(kept)


def main():
    names = [v[0] for v in VARIANTS]
    hdr = f"{'集':12s} {'raw':>6s} {'EKF':>6s} " + ' '.join(f'{nm:>10s}' for nm in names)
    print(hdr)
    print('-' * len(hdr))
    wins = {nm: 0 for nm in names}
    for sub, resdir, ekf_ref in SETS:
        d = f'{ROOT}/{sub}'
        gt = pd.read_csv(f'{d}/ground_truth.txt').iloc[:, 1:4].values.astype(float)
        fp = pd.read_csv(f'{d}/fusion_pose.txt').iloc[:, 1:4].values.astype(float)
        m = sio.loadmat(f'{d}/{resdir}/trajectories.mat')
        final = np.array(m['exp_trajectory'], dtype=float)
        e = sio.loadmat(f'{d}/{resdir}/loop_constraint_events.mat')
        evm = e['NLM_LOOP_EVENTS']
        evm = evm.reshape(-1, 5) if evm.ndim > 1 else np.atleast_2d(evm)
        evs = [(int(r[0]), int(r[1]), r[2:5]) for r in evm]
        em = sio.loadmat(f'{d}/slam_results/pr_embeddings_ft.mat')
        emb = np.asarray(em['pr_embeddings'], dtype=np.float32)
        if emb.shape[0] != len(gt) and emb.shape[1] == len(gt):
            emb = emb.T
        n = len(fp)
        a_raw = sim3_full(fp, gt)
        L_old, n_ev, _ = field_from_events(n, evs, fp, hold=False, place_r=0.0, cap=CAP)
        raw = final - L_old
        row = []
        for nm, kw in VARIANTS:
            L, _, n_kept = field_from_events(n, evs, fp, emb=emb, cap=CAP, **kw)
            a = sim3_full(raw + L, gt)
            row.append(f'{a:10.2f}')
            if a < ekf_ref - 0.05:
                wins[nm] += 1
        chk = 'OK' if abs(a_raw - ekf_ref) < 1.5 else f'!!({a_raw:.2f})'
        print(f'{sub[:12]:12s} {a_raw:6.2f} {ekf_ref:6.2f} ' + ' '.join(row) +
              f'  事件={len(evs)} raw≈EKF:{chk}')
    print('-' * len(hdr))
    print('胜 EKF 集数: ' + '  '.join(f'{nm}={wins[nm]}' for nm in names))


if __name__ == '__main__':
    main()
