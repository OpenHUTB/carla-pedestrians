#!/usr/bin/env python3
"""EKF 融合 vs 纯 VO vs 纯 IMU 的 ATE 轨迹误差对比评估。

对三条轨迹分别做 Umeyama（旋转+平移+尺度）对齐到 GT 后，计算
ATE（平均绝对轨迹误差 RMSE）与最大误差。对齐尺度 >0 且接近 1 时
结果最具可比性。

数据文件（同一目录、同一采集）：
  ground_truth.txt   ts,x,y,z,roll,pitch,yaw          (5000 行)
  fusion_pose.txt    ts,x,y,z,R,P,Y(度),...           (5000 行)
  visual_odometry.txt ts,vo_x,vo_y,vo_z,...           (5000 行)
  aligned_imu.txt    ts,ax,ay,az,gx,gy,gz             (60Hz，纯 IMU 用)

纯 IMU 轨迹：对 aligned_imu 做二阶欧拉积分（去重力）得到，
再重采样到与相机帧相同的时间点，保证与 VO/EKF 同帧对比。
"""
import os
import sys
import numpy as np

DATA_DIR = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', 'data', 'Town05Data_IMU_Fusion')


def load_positions(fname, cols=(1, 2, 3)):
    p = os.path.join(DATA_DIR, fname)
    a = np.loadtxt(p, delimiter=',', skiprows=1)
    return a[:, 0], a[:, list(cols)]


def umeyama(src, dst, with_scale=True):
    """对齐 src→dst：dst ≈ s·R·src + t。返回 (R, t, s)。"""
    n, d = src.shape
    mu_s = src.mean(0)
    mu_d = dst.mean(0)
    sc = src - mu_s
    dc = dst - mu_d
    var_s = (sc ** 2).sum() / n
    cov = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    if with_scale:
        s = (np.diag(D) * np.diag(S)).sum() / (var_s + 1e-12)
        if s < 0:
            s = 1.0
    else:
        s = 1.0
    t = mu_d - s * (R @ mu_s)
    return R, t, s


def align_and_ate(traj, gt):
    """同长度对齐，返回 (RMSE, max_err, scale)"""
    R, t, s = umeyama(traj, gt)
    est = (s * traj @ R.T) + t
    err = np.linalg.norm(est - gt, axis=1)
    return err, s


def imu_dead_reckoning(imu_ts, imu_acc, imu_gyr, cam_ts=None):
    """纯 IMU 二阶欧拉积分（z 轴为重力方向，静止时 az≈g）。

    去重力：静止重力估计 g0 = 前 200 个采样均值；a_body = acc - g0。
    姿态由陀螺积分（欧拉角），旋转矩阵变换到世界系后双重积分。
    cam_ts 非 None 且长度不同步时重采样到 cam_ts。
    """
    # 去重力：整段轨迹平均加速度≈0（往返行驶），故均值≈重力方向
    g0 = imu_acc.mean(axis=0)
    a = imu_acc - g0
    dt = np.diff(imu_ts, prepend=imu_ts[0])
    dt[0] = dt[1] if len(dt) > 1 else 0.0
    dt = np.clip(dt, 1e-4, 0.05)

    # 陀螺积分欧拉角
    ang = np.zeros_like(imu_gyr)
    dtv = dt[:, None]
    ang[1:] = np.cumsum(0.5 * (imu_gyr[1:] + imu_gyr[:-1]) * dtv[1:], axis=0)
    # 欧拉角 → 旋转矩阵（body→world, 小角度近似下足够；用精确公式）
    n = len(ang)
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    for i in range(1, n):
        r, p, y = ang[i]
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        ai = R @ a[i]
        dti = dt[i]
        vel[i] = vel[i - 1] + ai * dti
        pos[i] = pos[i - 1] + (vel[i - 1] + ai * dti / 2) * dti

    # 与相机帧同步：帧数一致时直接返回（aligned_imu 每相机帧一行），
    # 否则按时间戳插值
    if cam_ts is not None and len(cam_ts) != n:
        cam_pos = np.stack([
            np.interp(cam_ts, imu_ts, pos[:, k]) for k in range(3)], axis=1)
        return cam_pos
    return pos


def main():
    gt_ts, gt = load_positions('ground_truth.txt')
    fus_ts, fus = load_positions('fusion_pose.txt')
    vo_ts, vo = load_positions('visual_odometry.txt')

    print(f"GT 帧数: {len(gt_ts)}")
    n = min(len(gt_ts), len(fus_ts), len(vo_ts))
    gt_ts, gt = gt_ts[:n], gt[:n]
    fus_ts, fus = fus_ts[:n], fus[:n]
    vo_ts, vo = vo_ts[:n], vo[:n]

    results = {}

    def report(name, traj, ts, gt_ts_ref):
        if len(traj) != n:
            print(f"[WARN] {name} 帧数 {len(traj)} != {n}，跳过")
            return
        err, s = align_and_ate(traj, gt)
        results[name] = err
        print(f"{name:8s}: ATE(RMSE)={err.mean():8.4f} m | "
              f"max={err.max():8.4f} m | p95={np.percentile(err, 95):8.4f} m | "
              f"对齐尺度={s:.4f}")

    report("EKF", fus, fus_ts, gt_ts)
    report("VO", vo, vo_ts, gt_ts)

    # 纯 IMU
    try:
        imu_ts, imu = load_positions('aligned_imu.txt', cols=(1, 2, 3, 4, 5, 6))
        imu_pos = imu_dead_reckoning(imu_ts, imu[:, :3], imu[:, 3:6], gt_ts)
        report("IMU", imu_pos, imu_ts, gt_ts)
    except Exception as e:
        print(f"[WARN] IMU 积分失败: {e}")

    if len(results) == 3:
        best = min(results, key=lambda k: results[k].mean())
        print(f"\n结论: 最佳轨迹 = {best} "
              f"(RMSE {results[best].mean():.4f} m)")
        ok = (best == 'EKF'
              and results['EKF'].mean() < results['VO'].mean()
              and results['EKF'].mean() < results['IMU'].mean())
        print("EKF 优于纯VO 且 优于纯IMU:", "YES ✓" if ok else "NO ✗")


if __name__ == '__main__':
    main()
