#!/usr/bin/env python3
"""
EuRoC VO+EKF 融合生成器 (论文 EKF Fusion baseline) — 数据驱动版
从 euroc_bag_extract_v2.py 的解包目录生成 test_euroc_fusion_slam.m 所需文件:
    <seq_dir>/fusion_pose_vo_ekf.txt   13列, 无表头 (MATLAB dlmread 从第1行读!)
                                       ts,x,y,z,roll,pitch,yaw(弧度),vx,vy,vz,ux,uy,uz
    <seq_dir>/ground_truth.txt         7列, 无表头, ts,x,y,z,roll,pitch,yaw(度)
                                       (200Hz GT 插值到相机时间戳, 行数必须与 fusion 一致)

坐标系约定 (全部由数据确定, 零GT泄漏):
  * EKF 世界系 = body(t0) 系 (GT 首行四元数=identity => GT世界系=body(t0)轴,
    MATLAB 'simple' 对齐为平移+缩放无旋转, 故 EKF 输出必须已在 GT 世界系中)。
  * R_w_b(t) = 扣除 gyro 偏置(悬停段均值)后积分, 起点 R(t0)=I。
  * 重力 g_w = 悬停段 mean(R_w_b(t) @ (-acc(t)))。本 bag IMU 轴被重映射
    (x 朝上), g_w≈[-9.2,0.5,3.2] 而非 [0,0,-9.81]。
  * VO 累积在本文件内自实现(正确 SE(3)): 共享 visual_odometry_opencv.py 的
    t_total 累积式有误(平移/旋转组合不一致, KITTI 慢转向一阶可忽略,
    EuRoC 快偏航发散), 且该文件被 CARLA/KITTI 已发表基线共用, 不动它。
  * VO 位姿在 cam0(首帧) 系: R0[i]=R_{cam0<-cami}, C0[i]=相机中心位置(VO单位)。
  * 外参 Ex=R_{world<-cam0} (纯数据, 共轭轴角 Procrustes):
      R0[i] = Ex^T @ Rw[i] @ Ex  (VO/IMU 姿态共轭, E 精确消去,
      常外参 => 无漂移; 旧帧间公式漏了时变 R0 因子, MH_01 大幅旋转下 88° 残差)
      一阶线性化 b_i ≈ M a_i (a_i=axang(Rw[idx_i]), b_i=axang(R0[i])),
      M = Procrustes(A B^T) 再 SO(3) 正交化 => Ex
  * 尺度 s(米/VO单位) = 每窗(0.5s)零起点 IMU 双积分位移/VO位移 的中位数
      (短窗偏置漂移误差~1%可忽略; 旧全局累积积分发散 s≈1.6, 真值≈0.04)
  * 视觉观测: 位置 = Ex @ (C0[i]*s) (相机中心, 世界系; 相机-body0常偏移
             被 MATLAB simple 对齐的平移吸收),
             姿态 = rpy(Rw[idx_i]) (世界系=body(t0), 机体姿态即陀螺积分值, 全3轴)。

GT 仅在末尾做自检打印(路径长/尖峰/simple对齐RMSE), 不进入 EKF 任何环节。

用法:
    python euroc_ekf_fusion.py <seq_dir> [--fx 535.4 --fy 535.4 --cx 320.1 --cy 247.6]
"""
import os
import sys
import glob
import argparse
import numpy as np
import cv2


def euler_zyx(r, p, y):
    """ZYX 欧拉 -> R = Rz(y)Ry(p)Rx(r) (与 ekf_standalone 的 R_mat 同一约定)。"""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def rpy_of(R):
    """R -> (roll, pitch, yaw), ZYX, 与 euler_zyx 互逆。"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-8:
        return (np.arctan2(R[2, 1], R[2, 2]),
                np.arctan2(-R[2, 0], sy),
                np.arctan2(R[1, 0], R[0, 0]))
    return (np.arctan2(-R[1, 2], R[1, 1]), np.arctan2(-R[2, 0], sy), 0.0)


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def to_so3(X):
    """SO(3) 正交化 (SVD)。"""
    U, _, Vt = np.linalg.svd(X)
    return U @ Vt


def angle_of(R):
    """R 的旋转角(弧度)。"""
    return float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class EKF_VIO_MAV:
    """IMU 预测 + 视觉位置/姿态全观测的 9 维 EKF (pos, vel, rpy)。
    世界系 = body(t0), 重力 g_w 由数据标定。极信视觉(同 KITTI 链路)。"""

    def __init__(self, g_w, v_max=20.0):
        self.x = np.zeros(9)
        self.dt = 0.005
        self.g_w = np.asarray(g_w, dtype=np.float64)
        self.v_max = v_max
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001])
        self.Q = np.diag([10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 0.001, 0.001, 0.001])
        self.R = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

    def imu_prediction(self, acc, gyro):
        dt = self.dt
        self.x[6] = wrap_pi(self.x[6] + gyro[0] * dt)
        self.x[7] = wrap_pi(self.x[7] + gyro[1] * dt)
        self.x[8] = wrap_pi(self.x[8] + gyro[2] * dt)
        R_mat = euler_zyx(self.x[6], self.x[7], self.x[8])
        # 比力 f = a - g  =>  a_w = R@f + g_w
        accel_world = R_mat @ acc + self.g_w
        self.x[:3] += self.x[3:6] * dt
        self.x[3:6] = np.clip(self.x[3:6] + accel_world * dt, -self.v_max, self.v_max)
        self.P += self.Q

    def visual_update(self, pos_obs, rpy_obs):
        # 6观测: 位置(世界系) + 机体姿态(全3轴; KITTI遗留roll/pitch强制0已移除)
        H = np.zeros((6, 9))
        H[:3, :3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)
        y = np.zeros(6)
        y[:3] = pos_obs - self.x[:3]
        y[3] = wrap_pi(rpy_obs[0] - self.x[6])
        y[4] = wrap_pi(rpy_obs[1] - self.x[7])
        y[5] = wrap_pi(rpy_obs[2] - self.x[8])
        S = H @ self.P @ H.T + self.R + np.eye(6) * 1e-8
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)


def load_euroc_files(d):
    cam0 = sorted(glob.glob(os.path.join(d, 'cam0', 'data', '*.png')))
    if not cam0:
        sys.exit('未找到 cam0/data/*.png — 请先运行 euroc_bag_extract_v2.py 解包')
    ts_cam = np.array([int(os.path.basename(p)[:-4]) for p in cam0], dtype=float) * 1e-9

    imu_lines = [l for l in open(os.path.join(d, 'mav0', 'imu0', 'data.csv'))
                 if l.strip() and not l.startswith('#')]
    imu = np.array([l.split(',') for l in imu_lines], dtype=float)
    # 7列: ts,wx,wy,wz,ax,ay,az (MATLAB read_euroc_imu_data 同布局)
    ts_imu = imu[:, 0] * 1e-9
    gyro = imu[:, 1:4]
    acc = imu[:, 4:7]

    gt_lines = [l for l in open(os.path.join(d, 'mav0', 'state_groundtruth_estimate0', 'data.csv'))
                if l.strip() and not l.startswith('#')]
    gt = np.array([l.split(',') for l in gt_lines], dtype=float)
    ts_gt = gt[:, 0] * 1e-9
    return cam0, ts_cam, ts_imu, acc, gyro, ts_gt, gt


def calibrate_imu(ts_imu, gyro, acc):
    """悬停段(1s bin: |gyro|均值<0.2rad/s 且 9.4<|acc|<10.1)标定 gyro 偏置。"""
    n = len(ts_imu)
    hover = np.zeros(n, dtype=bool)
    step = 200  # 1s @200Hz
    for s in range(0, n - step, step):
        g = np.linalg.norm(gyro[s:s + step], axis=1).mean()
        an = np.linalg.norm(acc[s:s + step], axis=1).mean()
        if g < 0.2 and 9.4 < an < 10.1:
            hover[s:s + step] = True
    if hover.sum() < 2 * step:
        sys.exit(f'悬停段不足 ({hover.sum()} 样本), 无法标定 gyro 偏置')
    bias = gyro[hover].mean(0)
    return bias, hover


def run_vo(cam0, K):
    """单目 VO, 正确 SE(3) 累积, 位姿在 cam0(首帧) 系。
    返回 R0[i]=R_{cam0<-cami}, C0[i]=相机中心位置(VO单位), inl[i]=内点数。"""
    n = len(cam0)
    det = cv2.ORB_create(nfeatures=2000)
    mat = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    R0 = np.zeros((n, 3, 3))
    C0 = np.zeros((n, 3))
    R0[0] = np.eye(3)
    R_acc = np.eye(3)
    C_acc = np.zeros(3)
    inl = np.zeros(n)
    prev = pkp = pdes = None
    for i, p in enumerate(cam0):
        img = cv2.imread(p)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        kp, des = det.detectAndCompute(gray, None)
        if prev is None:
            prev, pkp, pdes = gray, kp, des
            continue
        if des is None or pdes is None or len(kp) < 10 or len(pkp) < 10:
            prev, pkp, pdes = gray, kp, des
            continue
        m = mat.knnMatch(pdes, des, k=2)
        good = [mm[0] for mm in m if len(mm) > 1 and mm[0].distance < 0.7 * mm[1].distance]
        if len(good) < 10:
            prev, pkp, pdes = gray, kp, des
            continue
        p1 = np.float32([pkp[x.queryIdx].pt for x in good])
        p2 = np.float32([kp[x.trainIdx].pt for x in good])
        E, mask = cv2.findEssentialMat(p1, p2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None or mask is None:
            prev, pkp, pdes = gray, kp, des
            continue
        _, R, t, pmask = cv2.recoverPose(E, p1, p2, K, mask=mask)
        ni = int(np.sum(pmask))
        inl[i] = ni
        if ni < 20:
            prev, pkp, pdes = gray, kp, des
            continue
        # 正确累积: C_i^{cam0} = R_{cam0<-i-1} C_i^{i-1} + C_{i-1}^{cam0},
        # C_i^{i-1} = -R^T t;  R_{cam0<-i} = R_{cam0<-i-1} R_{i-1<-i}
        C_acc = R_acc @ (-R.T @ t[:, 0]) + C_acc
        R_acc = R_acc @ R
        R0[i] = R_acc
        C0[i] = C_acc
        prev, pkp, pdes = gray, kp, des
    for i in range(1, n):  # 无效帧沿用上一个有效位姿
        if inl[i] < 20:
            R0[i] = R0[i - 1]
            C0[i] = C0[i - 1]
    return R0, C0, inl


def axang(R):
    """R -> 轴角向量 (幅值=旋转角, 方向=轴)。"""
    tr = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(tr)
    if th < 1e-8:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th * (ax / (np.linalg.norm(ax) + 1e-12))


def estimate_Ex(Rw, idx_cam, R0):
    """Ex = R_{world<-cam0} (世界系=body(t0)): R0[i] = Ex^T @ Rw[i] @ Ex。
    共轭关系一阶线性化 b_i ≈ M a_i (a_i=axang(Rw), b_i=axang(R0), 小角下
    共轭≈左乘), M = Procrustes(A B^T) 再 SO(3) 正交化得 Ex。
    纯数据, 常外参 => 无漂移。返回 (Ex, 残差均值°, 残差P90°, 旋转帧数)。"""
    n = len(idx_cam)
    A_cols, B_cols = [], []
    for i in range(1, n):
        a = axang(Rw[idx_cam[i]])
        b = axang(R0[i])
        if np.linalg.norm(a) > 1e-4 and np.linalg.norm(b) > 1e-4:
            A_cols.append(a)
            B_cols.append(b)
    A = np.array(A_cols).T  # 3xN
    B = np.array(B_cols).T
    if A.shape[1] < 100:
        sys.exit('有效旋转帧不足, 无法拟合 Ex')
    U, _, Vt = np.linalg.svd(A @ B.T)
    Ex = to_so3(U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt)
    # 残差: Ex^T Rw Ex R0^T 应≈I (抽 ~200 帧)
    step = max(1, n // 200)
    errs = np.array([np.degrees(angle_of(Ex.T @ Rw[idx_cam[i]] @ Ex @ R0[i].T))
                     for i in range(1, n, step)])
    return Ex, float(errs.mean()), float(np.percentile(errs, 90)), A.shape[1]


def estimate_scale(Rw, acc, g_w, ts_imu, C0, ts_cam, win_s=0.5):
    """尺度 s(米/VO单位) = 每窗(默认0.5s)零起点 IMU 双积分位移 / VO位移 的中位数。
    每窗独立从静止积分 (旧版全局累积积分被偏置漂移带发散: s≈1.6, 真值≈0.04);
    0.5s 短窗内偏置误差对位移影响 ~1%, 可忽略。"""
    W = max(int(win_s / (ts_cam[1] - ts_cam[0])), 1)
    scales = []
    i = 20
    while i + W < len(ts_cam):
        ia = int(np.searchsorted(ts_imu, ts_cam[i]))
        ib = int(np.searchsorted(ts_imu, ts_cam[i + W]))
        if ib - ia < 8:
            i += W
            continue
        a_w = np.einsum('tik,tk->ti', Rw[ia:ib], acc[ia:ib]) + g_w
        dtw = np.diff(ts_imu[ia:ib])
        dtw = np.concatenate([dtw, [dtw[-1]]])
        v = np.zeros_like(a_w)
        for k in range(1, len(v)):
            v[k] = v[k - 1] + a_w[k - 1] * dtw[k - 1]
        p = np.zeros_like(v)
        for k in range(1, len(p)):
            p[k] = p[k - 1] + v[k - 1] * dtw[k - 1]
        d_imu = np.linalg.norm(p[-1] - p[0])
        d_vo = np.linalg.norm(C0[i + W] - C0[i])
        if d_vo > 0.03 and d_imu > 0.002:
            scales.append(d_imu / d_vo)
        i += W
    if not scales:
        return 1.0, 0
    s_mp = float(np.median(scales))
    return s_mp, len(scales)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seq_dir')
    # EuRoC MH_01/MH_03 cam0 默认内参; 其余序列按 camera_info 覆盖
    ap.add_argument('--fx', type=float, default=535.4)
    ap.add_argument('--fy', type=float, default=535.4)
    ap.add_argument('--cx', type=float, default=320.1)
    ap.add_argument('--cy', type=float, default=247.6)
    a = ap.parse_args()
    d = os.path.abspath(a.seq_dir)

    cam0, ts_cam, ts_imu, acc, gyro, ts_gt, gt = load_euroc_files(d)
    n = len(cam0)
    print(f'序列: {d}\n相机帧: {n}  IMU行: {len(ts_imu)}  GT行: {len(ts_gt)}')

    # ---- IMU 标定 (纯数据) ----
    bias, hover = calibrate_imu(ts_imu, gyro, acc)
    a_hover = acc[hover].mean(0)
    print(f'gyro偏置(rad/s)={bias.round(5)}  悬停样本={hover.sum()}  悬停比力={a_hover.round(3)}')

    # ---- R_w_b(t): 偏置扣除后积分, 世界系=body(t0) ----
    gcb = gyro - bias
    dt_i = np.diff(ts_imu)
    dt_i = np.concatenate([dt_i, [dt_i[-1]]])
    Rw = np.zeros((len(ts_imu), 3, 3))
    Rw[0] = np.eye(3)

    def skew(w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    for k in range(len(ts_imu) - 1):
        w = gcb[k] * dt_i[k]
        S = skew(w)
        Rw[k + 1] = Rw[k] @ (np.eye(3) + S + 0.5 * S @ S)
    g_w = np.einsum('tik,tk->i', Rw[hover], -acc[hover]) / hover.sum()
    print(f'世界系重力 g_w={g_w.round(3)}  |g_w|={np.linalg.norm(g_w):.3f} (应≈9.81)')

    # ---- VO (正确 SE(3) 累积; 结果缓存, 免重复跑 ~100s) ----
    K = np.array([[a.fx, 0, a.cx], [0, a.fy, a.cy], [0, 0, 1]], dtype=np.float32)
    cache_path = os.path.join(d, '_vo_cache2.npz')
    R0 = C0 = inl = None
    if os.path.exists(cache_path):
        z = np.load(cache_path)
        if len(z['R0']) == n:
            R0, C0, inl = z['R0'], z['C0'], z['inl']
            print('VO: 使用缓存 _vo_cache2.npz')
    if R0 is None:
        R0, C0, inl = run_vo(cam0, K)
        np.savez(cache_path, R0=R0, C0=C0, inl=inl)
    n_up = int((inl >= 20).sum())
    print(f'VO: 有效帧 {n_up}/{n}  inlier均值(有效帧)={inl[inl >= 20].mean():.0f}')

    idx_cam = np.clip(np.searchsorted(ts_imu, ts_cam), 0, len(ts_imu) - 1)

    # ---- 外参 Ex (纯数据, 共轭轴角 Procrustes) ----
    Ex, fit_mean, fit_p90, n_rot = estimate_Ex(Rw, idx_cam, R0)
    print(f'Ex(共轭Procrustes, 旋转帧={n_rot}) 共轭残差: 均值={fit_mean:.2f}° P90={fit_p90:.2f}°')
    if fit_mean > 5.0:
        print('⚠️ Ex 拟合残差偏大, VO/IMU 姿态一致性差 (位置/尺度仍可用)')

    # ---- 尺度 (纯数据, 每窗0.5s零起点双积分) ----
    s_mp, n_win = estimate_scale(Rw, acc, g_w, ts_imu, C0, ts_cam)
    print(f'尺度: 有效窗={n_win}  s(米/VO单位)={s_mp:.4f}')

    # ---- EKF ----
    ekf = EKF_VIO_MAV(g_w)
    fus_f = open(os.path.join(d, 'fusion_pose_vo_ekf.txt'), 'w')
    gt_f = open(os.path.join(d, 'ground_truth.txt'), 'w')
    # IMU 事件指针: 样本 k 有效区间 [ts[k], ts[k+1])
    k_imu = int(min(np.searchsorted(ts_imu, ts_cam[0], side='right') - 1, len(ts_imu) - 2))
    k_imu = max(k_imu, 0)
    last_t = ts_cam[0]
    n_update = 0
    for i in range(n):
        t_cam = ts_cam[i]
        if i > 0:
            while k_imu < len(ts_imu) - 1 and ts_imu[k_imu + 1] <= t_cam:
                k_imu += 1
            dt_step = t_cam - last_t
            if dt_step > 1e-9:
                ekf.dt = dt_step
                ekf.imu_prediction(acc[k_imu], gcb[k_imu])
            last_t = t_cam
        # 视觉更新: 位置=世界系相机中心, 姿态=机体rpy(全3轴, 世界系=body(t0))
        if inl[i] >= 20:
            pos_obs = Ex @ C0[i] * s_mp
            rpy_obs = rpy_of(Rw[idx_cam[i]])
            ekf.visual_update(pos_obs, rpy_obs)
            n_update += 1

        pos, att, vel = ekf.x[:3], ekf.x[6:9], ekf.x[3:6]
        unc = np.sqrt(np.diag(ekf.P[:3, :3]))
        att = np.array([wrap_pi(att[0]), wrap_pi(att[1]), wrap_pi(att[2])])
        fus_f.write('%.9f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' % (
            ts_cam[i], *['%.6f' % v for v in pos], *['%.6f' % v for v in att],
            *['%.6f' % v for v in vel], *['%.6f' % v for v in unc]))

        # GT 重采样到相机帧 (仅输出文件用, 不进EKF; 四元数xyzw->ZYX欧拉, 度)
        jg = int(np.clip(np.searchsorted(ts_gt, ts_cam[i]), 0, len(gt) - 1))
        g = gt[jg]
        gt_r = np.degrees(np.arctan2(2 * (g[4] * g[5] + g[6] * g[7]), 1 - 2 * (g[5] ** 2 + g[6] ** 2)))
        gt_p = np.degrees(np.arcsin(np.clip(2 * (g[7] * g[5] - g[4] * g[6]), -1, 1)))
        gt_y = np.degrees(np.arctan2(2 * (g[5] * g[6] + g[4] * g[7]), 1 - 2 * (g[4] ** 2 + g[5] ** 2)))
        gt_f.write('%.9f,%s,%s,%s,%s,%s,%s\n' % (
            ts_cam[i], *['%.6f' % v for v in g[1:4]], *['%.4f' % v for v in (gt_r, gt_p, gt_y)]))
        if (i + 1) % 500 == 0:
            print(f'  进度: {i + 1}/{n} ({(i + 1) / n * 100:.1f}%) | VO更新: {n_update}')

    fus_f.close()
    gt_f.close()
    print(f'✅ 完成: VO有效更新 {n_update}/{n} | 已写 fusion_pose_vo_ekf.txt / ground_truth.txt')

    # ---- 自检 (GT 仅评估) ----
    fus = np.loadtxt(os.path.join(d, 'fusion_pose_vo_ekf.txt'), delimiter=',')
    pf = fus[:, 1:4]
    seg = np.linalg.norm(np.diff(pf, axis=0), axis=1)
    L_f = seg.sum()
    gt_pos = np.column_stack([np.interp(ts_cam, ts_gt, gt[:, c]) for c in (1, 2, 3)])
    L_g = np.sum(np.linalg.norm(np.diff(gt_pos, axis=0), axis=1))
    print(f'自检: EKF路径={L_f:.2f}m (GT={L_g:.2f}m)  最大单段={seg.max():.3f}m')
    # 复现 MATLAB 'simple' 对齐: 起点平移 + 路径长缩放, 无旋转
    c1 = pf - pf[0]
    c2 = gt_pos - gt_pos[0]
    sc_g = L_g / L_f if L_f > 0 else 1.0
    res = np.linalg.norm(c1 * sc_g - c2, axis=1)
    print(f"自检(simple对齐, 复现MATLAB): RMSE={np.sqrt((res ** 2).mean()):.3f}m  "
          f"终点误差={res[-1]:.3f}m  漂移率={res[-1] / L_g * 100:.2f}%")
    print(f'   {os.path.join(d, "fusion_pose_vo_ekf.txt")} (13列无表头)')
    print(f'   {os.path.join(d, "ground_truth.txt")} (7列无表头, 已重采样到相机帧)')


if __name__ == '__main__':
    main()
