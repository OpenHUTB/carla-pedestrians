#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
KITTI 转换管线: 原始 KITTI odometry 数据 → 标准 IMU_Fusion 格式 + 真视觉惯性 EKF

背景: 原 `prepare_kitti_fusion_data` 仅"读 GT poses + OXTS IMU"造合成 fusion_pose
(1.9x 尺度 + 130m 漂移), 精确降级过程在缺失代码里, 无法可靠复现 → 循环论证。
本脚本重建为**真·视觉惯性 EKF**: 从 KITTI 图像跑 VisualOdometry 得真 VO,
与 oxts IMU 融合, 用采集器同款 EKF_VIO (EKF_OFFLINE=1) 跑出 fusion_pose.txt。

用法:
    EKF_OFFLINE=1 python kitti_prepare_fusion.py \
        --raw <KITTI_raw_dir> --out <out_dir> \
        [--vo-scale 0.10] [--dt 0.104] [--img-dir image_0]

输入 (raw_dir):
    oxts/data/*.txt  每帧 oxts (30 列)
    times.txt        帧时间戳
    calib.txt        相机标定 (P2 行 = odometry 相机内参)
    poses.txt        GT 位姿 (3x4, 12 列/帧)
    <img-dir>/*.png  图像 (默认 image_0)

输出 (out_dir, 与 CARLA 标准格式一致, 供 replay/NLM 消费):
    ground_truth.txt    timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw   (弧度)
    aligned_imu.txt     timestamp,ax,ay,az,gx,gy,gz
    visual_odometry.txt timestamp,vo_x,vo_y,vo_z,roll,pitch,yaw      (弧度, 绝对位姿)
    fusion_pose.txt     真视觉惯性 EKF (与采集同格式, 角度=度)
    <img-dir>/          (图像拷贝, 供后续 NLM 闭环用)
"""
import os
import sys
import math
import glob
import shutil
import argparse

# ── 离线模式: stub 掉 carla 依赖, 只 import EKF_VIO + VisualOdometry + _ReplayImu ──
os.environ['EKF_OFFLINE'] = '1'
_COLLECT_DIR = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '00_collect_data'))
if _COLLECT_DIR not in sys.path:
    sys.path.insert(0, _COLLECT_DIR)
import IMU_Vision_Fusion_EKF as _collector  # noqa: E402
EKF_VIO = _collector.EKF_VIO
VisualOdometry = _collector.VisualOdometry
_ReplayImu = _collector._ReplayImu

import numpy as np
import cv2
import pandas as pd
from scipy.spatial.transform import Rotation as R


def load_raw_kitti(raw_dir):
    """读取 KITTI 原始数据: oxts, times, calib(P2 内参), poses."""
    oxts_files = sorted(glob.glob(os.path.join(raw_dir, 'oxts', 'data', '*.txt')))
    if not oxts_files:
        raise FileNotFoundError(f"未找到 oxts 文件: {raw_dir}/oxts/data/*.txt")
    oxts = np.array([np.loadtxt(f) for f in oxts_files], dtype=np.float64)
    times = np.loadtxt(os.path.join(raw_dir, 'times.txt'))
    # calib.txt 第 3 行是 P2 (odometry 相机), 形如 "P2: fx 0 cx 0 ..."
    calib_lines = open(os.path.join(raw_dir, 'calib.txt')).read().strip().split('\n')
    p2_line = next((ln for ln in calib_lines if ln.startswith('P2')), calib_lines[2])
    p2_nums = p2_line.split(':', 1)[1].split()
    P2 = np.array([float(x) for x in p2_nums]).reshape(3, 4)
    K = P2[:, :3].astype(np.float64)  # 3x3 内参
    poses = np.loadtxt(os.path.join(raw_dir, 'poses.txt'))
    return oxts, times, K, poses


def make_ground_truth(poses, times):
    """从 poses.txt (3x4) 生成 ground_truth.txt (re-origin, 欧拉角弧度)."""
    t = poses[:, [3, 7, 11]]  # tx, ty, tz (3x4 矩阵每行末列)
    t = t - t[0]              # re-origin 到首帧
    Rm = poses.reshape(-1, 3, 4)[:, :, :3]  # (N, 3, 3)
    eul = R.from_matrix(Rm).as_euler('xyz') * 180.0 / math.pi  # (N, 3) 度 (MATLAB read_ground_truth 期望度, 与 EKF/GT 旧口径一致)
    rows = []
    for i in range(len(times)):
        rows.append(
            f"{times[i]:.6f},{t[i,0]:.6f},{t[i,1]:.6f},{t[i,2]:.6f},"
            f"{eul[i,0]:.6f},{eul[i,1]:.6f},{eul[i,2]:.6f}")
    return rows


def make_aligned_imu(oxts, times):
    """从 oxts 生成 aligned_imu.txt (accel=0-based 11:14, gyro=0-based 17:20)."""
    rows = []
    for i in range(len(times)):
        ax, ay, az = oxts[i, 11], oxts[i, 12], oxts[i, 13]
        gx, gy, gz = oxts[i, 17], oxts[i, 18], oxts[i, 19]
        rows.append(f"{times[i]:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f}")
    return rows


def make_visual_odometry(raw_dir, times, K, img_dir='image_0', vo_scale=0.10):
    """从 KITTI 图像生成 visual_odometry.txt.

    用 VisualOdometry (ORB 单目 VO) 逐帧得相对位姿, 按主循环同口径累积成绝对位姿:
    相机系 → (固定相机 pitch=-20°) 车系 → (VO 自身偏航) 世界系, 乘固定尺度 vo_scale。
    与采集主循环 (IMU_Vision_Fusion_EKF.py L2726-2745) 完全一致。
    """
    img_files = sorted(glob.glob(os.path.join(raw_dir, img_dir, '*.png')))
    if not img_files:
        img_files = sorted(glob.glob(os.path.join(raw_dir, img_dir, '*.jpg')))
    if not img_files:
        raise FileNotFoundError(f"未找到图像: {raw_dir}/{img_dir}/*.png")
    vo = VisualOdometry(camera_matrix=K.astype(np.float32))
    vo_abs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    c20 = math.cos(math.radians(20.0))
    s20 = math.sin(math.radians(20.0))
    rows = []
    n = min(len(img_files), len(times))
    for i in range(n):
        img = cv2.imread(img_files[i], cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        if i == 0:
            vo.process_frame(img)  # 首帧初始化
            rows.append(f"{times[i]:.6f},0.0,0.0,0.0,0.0,0.0,0.0")
            continue
        vo_pose, _num = vo.process_frame(img)
        if vo_pose is None:
            rows.append(f"{times[i]:.6f}," + ",".join(f"{v:.6f}" for v in vo_abs))
            continue
        # 累积 VO 相对运动 → 绝对位姿 (相机系→车系→世界系, 固定尺度)
        cam_dx, cam_dy, cam_dz = float(vo_pose[0]), float(vo_pose[1]), float(vo_pose[2])
        veh_fwd = -(s20 * cam_dy + c20 * cam_dz)
        veh_left = cam_dx
        veh_up = s20 * cam_dz - c20 * cam_dy
        vo_yaw = vo_abs[5]
        cos_y = math.cos(vo_yaw)
        sin_y = math.sin(vo_yaw)
        vo_abs[0] += vo_scale * (veh_fwd * cos_y - veh_left * sin_y)
        vo_abs[1] += vo_scale * (veh_fwd * sin_y + veh_left * cos_y)
        vo_abs[2] += vo_scale * veh_up
        vo_abs[3] = (vo_abs[3] + vo_pose[3] + math.pi) % (2 * math.pi) - math.pi
        vo_abs[4] = (vo_abs[4] + vo_pose[4] + math.pi) % (2 * math.pi) - math.pi
        vo_abs[5] = (vo_abs[5] + vo_pose[5] + math.pi) % (2 * math.pi) - math.pi
        rows.append(f"{times[i]:.6f}," + ",".join(f"{v:.6f}" for v in vo_abs))
    return rows


def run_ekf(out_dir, dt=0.104):
    """在 out_dir 的 ground_truth/visual_odometry/aligned_imu 上跑真视觉惯性 EKF.

    与采集器 replay() 同口径, 但 dt 用 KITTI 实际帧间隔 (0.104s, 10Hz) 而非 CARLA 0.05s。
    输出 fusion_pose.txt (与采集同格式, 角度=度)。
    """
    gt = pd.read_csv(os.path.join(out_dir, 'ground_truth.txt'))
    vo = pd.read_csv(os.path.join(out_dir, 'visual_odometry.txt'))
    imu = pd.read_csv(os.path.join(out_dir, 'aligned_imu.txt'))
    n = min(len(gt), len(vo), len(imu))
    init_pose = [float(gt.iloc[0][c]) for c in
                 ('pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw')]
    ekf = EKF_VIO(init_pose, [0.0, 0.0, 0.0], dt=dt, depth_scale_file=None)
    vo_rows = vo[['vo_x', 'vo_y', 'vo_z', 'roll', 'pitch', 'yaw']].values[:n]
    imu_rows = imu.values[:n, 1:]  # 首列 timestamp, 取后 6 列
    ts_col = gt['timestamp'].values[:n]
    fusion_rows = []
    for i in range(n):
        z_row = vo_rows[i]
        ekf.imu_prediction(_ReplayImu(*(float(x) for x in imu_rows[i])))
        # 跳变检测 (>10m 跳过该帧更新, 与 replay 同口径)
        vo_jump_skip = False
        if ekf._last_vo_pose is not None:
            if float(np.linalg.norm(z_row[:3] - np.asarray(ekf._last_vo_pose[:3]))) > 10.0:
                vo_jump_skip = True
        ekf._last_vo_pose = [float(v) for v in z_row]
        if not vo_jump_skip:
            ekf._last_vo_inliers = int(ekf._vo_match_ref)
            ekf.visual_update(z_row, frame_idx=i + 1)
        fusion_pos, fusion_att = ekf.get_current_pose()
        imu_dr, _ = ekf.get_imu_dead_reckoning_pose()
        fusion_vel = ekf.get_current_velocity()
        pos_unc = ekf.get_position_uncertainty()
        fusion_rows.append([
            ts_col[i],
            fusion_pos[0], fusion_pos[1], fusion_pos[2],
            math.degrees(fusion_att[0]), math.degrees(fusion_att[1]),
            math.degrees(fusion_att[2]),
            imu_dr[0], imu_dr[1], imu_dr[2],
            fusion_vel[0], fusion_vel[1], fusion_vel[2],
            pos_unc[0], pos_unc[1], pos_unc[2],
        ])
    out_path = os.path.join(out_dir, 'fusion_pose.txt')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,"
                "imu_pos_x,imu_pos_y,imu_pos_z,vx,vy,vz,uncert_x,uncert_y,uncert_z\n")
        for row in fusion_rows:
            f.write(",".join(f"{v:.6f}" for v in row) + "\n")
    return fusion_rows


def main():
    ap = argparse.ArgumentParser(
        description="KITTI 转换管线: 原始 KITTI → 标准格式 + 真视觉惯性 EKF")
    ap.add_argument('--raw', required=True, help="KITTI 原始数据目录")
    ap.add_argument('--out', required=True, help="输出目录")
    ap.add_argument('--vo-scale', type=float, default=0.10,
                    help="VO 固定尺度 (默认 0.10, 与 ScaleEstimator 初值一致)")
    ap.add_argument('--dt', type=float, default=None, help="EKF 时间步 (默认从 times 推算)")
    ap.add_argument('--img-dir', default='image_0', help="图像子目录 (默认 image_0)")
    ap.add_argument('--no-ekf', action='store_true', help="只生成 GT/IMU/VO, 不跑 EKF")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"[KITTI] raw={args.raw}  out={args.out}")
    oxts, times, K, poses = load_raw_kitti(args.raw)
    dt = args.dt if args.dt is not None else float(np.mean(np.diff(times)))
    print(f"[KITTI] 帧数={len(times)}  dt_mean={dt:.4f}s  内参 fx={K[0,0]:.2f} cy={K[1,2]:.2f}")

    # 1. ground_truth.txt
    gt_rows = make_ground_truth(poses, times)
    with open(os.path.join(args.out, 'ground_truth.txt'), 'w') as f:
        f.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw\n")
        f.write("\n".join(gt_rows) + "\n")
    print(f"[KITTI] ground_truth.txt: {len(gt_rows)} 行")

    # 2. aligned_imu.txt
    imu_rows = make_aligned_imu(oxts, times)
    with open(os.path.join(args.out, 'aligned_imu.txt'), 'w') as f:
        f.write("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
        f.write("\n".join(imu_rows) + "\n")
    print(f"[KITTI] aligned_imu.txt: {len(imu_rows)} 行")

    # 3. visual_odometry.txt
    vo_rows = make_visual_odometry(
        args.raw, times, K, img_dir=args.img_dir, vo_scale=args.vo_scale)
    with open(os.path.join(args.out, 'visual_odometry.txt'), 'w') as f:
        f.write("timestamp,vo_x,vo_y,vo_z,roll,pitch,yaw\n")
        f.write("\n".join(vo_rows) + "\n")
    print(f"[KITTI] visual_odometry.txt: {len(vo_rows)} 行")

    # 4. fusion_pose.txt (真视觉惯性 EKF)
    if not args.no_ekf:
        fusion_rows = run_ekf(args.out, dt=dt)
        print(f"[KITTI] fusion_pose.txt: {len(fusion_rows)} 行 (dt={dt:.4f}s)")

    # 5. 拷贝图像 (供后续 NLM 闭环用)
    img_src = os.path.join(args.raw, args.img_dir)
    img_dst = os.path.join(args.out, args.img_dir)
    if os.path.isdir(img_src) and not os.path.isdir(img_dst):
        shutil.copytree(img_src, img_dst)
        print(f"[KITTI] 图像拷贝: {img_dst}")
    print("[KITTI] 完成")


if __name__ == '__main__':
    main()
