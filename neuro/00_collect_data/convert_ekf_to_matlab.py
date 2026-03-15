#!/usr/bin/env python3
"""
将CSV格式的EKF融合数据转换为MATLAB格式
同时从OXTS读取真实IMU数据
"""

import os
import numpy as np

# 路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(SCRIPT_DIR, '..', 'data', 'KITTI_07')
INPUT_FILE = os.path.join(DATA_PATH, 'fusion_pose_ekf.txt')
OUTPUT_FILE = os.path.join(DATA_PATH, 'fusion_pose.txt')
OXTS_PATH = os.path.join(DATA_PATH, 'oxts', 'data')

print("转换EKF数据为MATLAB格式...")
print(f"输入: {INPUT_FILE}")
print(f"输出: {OUTPUT_FILE}")

with open(INPUT_FILE, 'r') as f_in, open(OUTPUT_FILE, 'w') as f_out:
    # 写入MATLAB风格的header
    f_out.write("% KITTI 07 EKF Fusion Data\n")
    f_out.write("% timestamp(s) x(m) y(m) z(m) roll(deg) pitch(deg) yaw(deg) ")
    f_out.write("vx(m/s) vy(m/s) vz(m/s) ax(m/s2) ay(m/s2) az(m/s2) ")
    f_out.write("wx(rad/s) wy(rad/s) wz(rad/s)\n")
    
    # 跳过CSV header
    header = f_in.readline()
    
    # 转换数据行
    count = 0
    for line in f_in:
        line = line.strip()
        if not line:
            continue
        
        # 解析CSV
        parts = line.split(',')
        if len(parts) < 16:
            print(f"警告: 第{count+1}行数据不完整，跳过")
            continue
        
        # CSV格式: timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,
        #          imu_pos_x,imu_pos_y,imu_pos_z,vel_x,vel_y,vel_z,
        #          uncertainty_x,uncertainty_y,uncertainty_z
        # 我们需要额外的IMU加速度和角速度，用0填充
        timestamp = float(parts[0])
        pos_x, pos_y, pos_z = float(parts[1]), float(parts[2]), float(parts[3])
        roll, pitch, yaw = float(parts[4]), float(parts[5]), float(parts[6])
        vel_x, vel_y, vel_z = float(parts[10]), float(parts[11]), float(parts[12])
        
        # 转为度（EKF输出的是弧度）
        roll_deg = roll * 180.0 / 3.14159265359
        pitch_deg = pitch * 180.0 / 3.14159265359
        yaw_deg = yaw * 180.0 / 3.14159265359
        
        # 从OXTS读取真实IMU数据
        oxts_file = os.path.join(OXTS_PATH, f'{count:010d}.txt')
        try:
            with open(oxts_file, 'r') as f_oxts:
                oxts_data = list(map(float, f_oxts.readline().split()))
            # OXTS格式：lat,lon,alt,roll,pitch,yaw,vn,ve,vf,vl,vu,ax,ay,az,af,al,au,wx,wy,wz,...
            ax, ay, az = oxts_data[11], oxts_data[12], oxts_data[13]
            wx, wy, wz = oxts_data[17], oxts_data[18], oxts_data[19]
        except:
            # 如果读取失败，用0填充
            ax, ay, az = 0.0, 0.0, 0.0
            wx, wy, wz = 0.0, 0.0, 0.0
        
        # 写入MATLAB格式（空格分隔）
        f_out.write(f"{timestamp:.6f} {pos_x:.6f} {pos_y:.6f} {pos_z:.6f} ")
        f_out.write(f"{roll_deg:.6f} {pitch_deg:.6f} {yaw_deg:.6f} ")
        f_out.write(f"{vel_x:.6f} {vel_y:.6f} {vel_z:.6f} ")
        f_out.write(f"{ax:.6f} {ay:.6f} {az:.6f} ")
        f_out.write(f"{wx:.6f} {wy:.6f} {wz:.6f}\n")
        
        count += 1

print(f"✓ 转换完成！共{count}行数据")
