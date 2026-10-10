#!/usr/bin/env python3
"""kbs/route2_dyaw_events.py — 路线2: 对 NLM 记录的全部闭环事件对测量视觉相对位姿 (dyaw)

与 /tmp/closedloop_decisive.m 完全同管线: ORB + 旋转不变BRIEF + F + E=K'FK + 四解(cheirality)。
数据源: a_final/loop_constraint_events.mat 的 [t_now, t_old] 事件对 (系统真实记录的闭环)。
输出: 逐事件 dyaw_est vs GT_dyaw + 匹配质量(inlier数, nn1 med) + 平移方向cos。
用途: ① 量化路线2 dyaw 测量通道在真实闭环点上的可用性 (S2 门控/权重信号的原料)
      ② 判断 dyaw 估计是否系统性可靠 (若 10/10 事件 |d_dyaw| 小 → 可作为闭环质量门)
"""
import numpy as np
import pandas as pd
import cv2
import scipy.io as sio
import glob, sys

BASE = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion/'
W, H = 160, 120
# 与 closedloop_decisive.m 一致的内参近似 (fx=W/2)
K = np.array([[W/2, 0, W/2], [0, W/2, H/2], [0, 0, 1.0]])
Kt = K.T

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def brief_pair(img1, img2, p1, p2, angle1, angle2, w):
    """旋转不变 BRIEF: 固定随机偏移 (同一组种子, 与 MATLAB 探针一致)"""
    rng = np.random.RandomState(20261009)
    N = 256
    ox  = np.round((rng.rand(N)-0.5)*w).astype(int).reshape(1,-1)
    oy  = np.round((rng.rand(N)-0.5)*w).astype(int).reshape(1,-1)
    ox2 = np.round((rng.rand(N)-0.5)*w).astype(int).reshape(1,-1)
    oy2 = np.round((rng.rand(N)-0.5)*w).astype(int).reshape(1,-1)
    def one(img, L, ang):
        c = np.cos(ang)[:, None]; s = np.sin(ang)[:, None]
        x1 = np.clip(np.round(L[:,0:1] + c*ox - s*oy).astype(int), 1, W-1)
        y1 = np.clip(np.round(L[:,1:2] + s*ox + c*oy).astype(int), 1, H-1)
        x2 = np.clip(np.round(L[:,0:1] + c*ox2 - s*oy2).astype(int), 1, W-1)
        y2 = np.clip(np.round(L[:,1:2] + s*ox2 + c*oy2).astype(int), 1, H-1)
        return (img[y1, x1] < img[y2, x2]).astype(np.uint8)
    return one(img1, p1, angle1), one(img2, p2, angle2)

def hamming_nn(D1, D2):
    """256-bit hamming 距离矩阵 + 每行最近邻"""
    n1, n2 = D1.shape[0], D2.shape[0]
    B1 = D1.reshape(n1, 32, 8)
    B2 = D2.reshape(n2, 32, 8)
    nn1 = np.full(n1, 257, dtype=int)
    ti  = np.zeros(n1, dtype=int)
    for c in range(32):
        for b in range(8):
            pass  # 逐位太慢; 改用 int32 打包
    # 打包成 8 个 32-bit 字
    w1 = np.zeros((n1, 8), dtype=np.uint32)
    w2 = np.zeros((n2, 8), dtype=np.uint32)
    for c in range(32):
        chunk = (D1[:, c*8:(c+1)*8] * (1 << np.arange(8)[::-1])).sum(1)
        # 每个 8-bit 字节展开到 32-bit: 用查表
    # 简单做法: 直接算 hamming
    lut = np.array([bin(i).count('1') for i in range(256)], dtype=np.uint8)
    # 把每行256bit分成32个字节
    d = np.zeros((n1, n2), dtype=np.uint16)
    for c in range(32):
        b1 = (D1[:, c*8:(c+1)*8] * (1 << np.arange(8)[::-1])).sum(1).astype(np.uint8)
        b2 = (D2[:, c*8:(c+1)*8] * (1 << np.arange(8)[::-1])).sum(1).astype(np.uint8)
        x = np.bitwise_xor(b1[:, None], b2[None, :])
        d += lut[x]
    nn1 = d.min(1)
    ti = d.argmin(1)
    return nn1, ti, d

def second_nn(d, nn1, ti):
    n = d.shape[0]
    d2 = d.copy()
    d2[np.arange(n), ti] = 255
    return d2.min(1)

def norm_pix(K, pts):
    h = np.hstack([pts, np.ones((len(pts), 1))])
    x = np.linalg.solve(K, h.T)
    return x[:2].T

def linear_tri(x1, x2, R, t):
    """由两个归一化射线 + (R,t) 求 3D 点: X0 = r1 * ss"""
    r0 = np.array([x1[0], x1[1], 1.0])
    d1 = -R.T @ t
    d2 = R.T @ np.array([x2[0], x2[1], 1.0])
    A = np.column_stack([r0, -d2])
    ss, *_ = np.linalg.lstsq(A, d1, rcond=None)
    return r0 * ss[0]

def four_solvers(E, x1, x2, n_test=80):
    U, S, Vt = np.linalg.svd(E)
    Wm = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    R1 = U @ Wm @ Vt
    R2 = U @ Wm.T @ Vt
    best = (-1, None, None)
    for Rc, tsign in [(R1, 1), (R2, 1)]:
        if abs(np.linalg.det(Rc) - 1) > 0.1:
            continue
        for sgn in (1, -1):
            tv = tsign * U[:, 2]
            nv = np.linalg.norm(tv)
            if nv < 1e-9:
                continue
            tv = tv / nv
            n_ok = 0
            for k in range(min(n_test, len(x1))):
                X0 = linear_tri(x1[k], x2[k], Rc, tv)
                X1 = Rc @ X0 + tv
                if X0[2] > 0.1 and X1[2] > 0.1:
                    n_ok += 1
            if n_ok > best[0]:
                best = (n_ok, Rc, tv)
    return best[1], best[2], best[0]

def main():
    gt = pd.read_csv(BASE + 'ground_truth.txt')
    yaw = gt['yaw'].values
    pos = gt[['pos_x','pos_y','pos_z']].values

    p = glob.glob(BASE + 'a_final/loop_constraint_events.mat')[0]
    ev = sio.loadmat(p)['NLM_LOOP_EVENTS']
    print(f"事件数: {ev.shape[0]}")
    print(f"{'ev':>2s} {'t_now':>5s} {'t_old':>5s} {'gap':>5s} | {'n_match':>7s} {'nn1med':>6s} {'F_inl':>5s} "
          f"{'E_svd3':>6s} | {'dyaw_gt':>8s} {'dyaw_est':>9s} {'|d|':>6s} | {'cos_t':>6s} | {'n_cheir':>7s}")

    results = []
    for i in range(ev.shape[0]):
        t_now = int(ev[i, 0]); t_old = int(ev[i, 1])
        f1, f2 = t_old, t_now  # 旧帧 → 新帧
        g1 = cv2.imread(BASE + f'{f1:04d}.png', cv2.IMREAD_GRAYSCALE)
        g2 = cv2.imread(BASE + f'{f2:04d}.png', cv2.IMREAD_GRAYSCALE)
        d1 = g1.astype(np.float32); d2 = g2.astype(np.float32)
        orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.1, nlevels=7)
        kp1, de1 = orb.detectAndCompute(g1, None)
        kp2, de2 = orb.detectAndCompute(g2, None)
        if de1 is None or de2 is None or len(kp1) < 20 or len(kp2) < 20:
            print(f"{i:2d} {f2:5d} {f1:5d} {t_now-t_old:5d} | 特征不足"); continue
        L1 = np.array([k.pt for k in kp1])
        L2 = np.array([k.pt for k in kp2])
        a1 = np.array([k.angle * np.pi/180 for k in kp1])
        a2 = np.array([k.angle * np.pi/180 for k in kp2])
        w = max(12, 30)
        D1, D2 = brief_pair(d1, d2, L1, L2, a1, a2, w)
        nn1, ti, dmat = hamming_nn(D1, D2)
        n2 = second_nn(dmat, nn1, ti)
        keep = nn1 < 0.9 * n2
        n_keep = int(keep.sum())
        if n_keep < 8:
            print(f"{i:2d} {f2:5d} {f1:5d} {t_now-t_old:5d} | {n_keep:7d} {np.median(nn1):6.1f}   匹配<8")
            continue
        L1m = L1[keep].astype(np.float64)
        L2m = L2[ti[keep]].astype(np.float64)
        F, mask = cv2.findFundamentalMat(L1m, L2m, cv2.FM_RANSAC, 0.8, 0.99, 100)
        if mask is None or F is None or F.size != 9:
            results.append((i, f1, f2, n_keep, float(np.median(nn1)), 0,
                            float('nan'), float(np.degrees(wrap_pi(yaw[f2-1]-yaw[f1-1]))),
                            float('nan'), float('nan'), float('nan'), 0,
                            float(np.linalg.norm(pos[f2-1]-pos[f1-1]))))
            print(f"{i:2d} {f2:5d} {f1:5d} {t_now-t_old:5d} | {n_keep:7d} {np.median(nn1):6.1f} F失败")
            continue
        E = Kt @ F @ K
        E = E / np.linalg.norm(E)
        Es = np.linalg.svd(E, compute_uv=False)
        inl = mask.reshape(-1).astype(bool)
        x1 = norm_pix(K, L1m[inl])
        x2 = norm_pix(K, L2m[inl])
        R, t, n_cheir = four_solvers(E, x1, x2)

        # GT: 帧 f1 → f2 的相对旋转 (绕 z 轴, CARLA 约定 yaw 顺时针正)
        dyaw_gt = wrap_pi(yaw[f2-1] - yaw[f1-1])
        ntr = float(np.linalg.norm(pos[f2-1] - pos[f1-1]))
        if R is None:
            results.append((i, f1, f2, n_keep, float(np.median(nn1)), int(inl.sum()),
                            float(Es[2]), float(np.degrees(dyaw_gt)), float('nan'),
                            float('nan'), float('nan'), 0, ntr))
            print(f"{i:2d} {f2:5d} {f1:5d} {t_now-t_old:5d} | {n_keep:7d} {np.median(nn1):6.1f} {int(inl.sum()):5d} "
                  f"{Es[2]:6.3f} | {np.degrees(dyaw_gt):8.1f} {'4解全拒':>9s}")
            continue
        dyaw_est = wrap_pi(np.arctan2(R[1, 0], R[0, 0]))
        d_dyaw = abs(wrap_pi(dyaw_est - dyaw_gt))
        # 平移方向 (GT)
        pa = pos[f1-1]; pb = pos[f2-1]
        tr = pa - pb  # 世界系: p1 = R*p2 + t_world 中 t 的方向
        ntr = np.linalg.norm(tr)
        t_dir_gt = tr / ntr if ntr > 1e-6 else np.array([1,0,0])
        cos_t = float(np.dot(t/np.linalg.norm(t), t_dir_gt)) if R is not None else float('nan')

        results.append((i, f1, f2, n_keep, float(np.median(nn1)), int(inl.sum()),
                        float(Es[2]), float(np.degrees(dyaw_gt)), float(np.degrees(dyaw_est)),
                        float(np.degrees(d_dyaw)), cos_t, n_cheir, float(ntr)))
        print(f"{i:2d} {f2:5d} {f1:5d} {t_now-t_old:5d} | {n_keep:7d} {np.median(nn1):6.1f} {int(inl.sum()):5d} "
              f"{Es[2]:6.3f} | {np.degrees(dyaw_gt):8.1f} {np.degrees(dyaw_est):9.1f} {np.degrees(d_dyaw):6.1f} | "
              f"{cos_t:6.3f} | {n_cheir:7d} (|t|={ntr:.2f}m)")

    print()
    ok = [r for r in results if r[9] < 30]
    print(f"══ 汇总: {len(results)}/{ev.shape[0]} 事件可估 | dyaw|d|<30° 的: {len(ok)} 个 | 中位 |d_dyaw| = {np.median([r[9] for r in results]) if results else float('nan'):.1f}°")

if __name__ == '__main__':
    main()
