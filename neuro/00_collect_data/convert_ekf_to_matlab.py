#!/usr/bin/env python3
"""
将CSV格式的EKF融合数据转换为MATLAB格式
同时从OXTS读取真实IMU数据
"""

import os
import numpy as np

# -------------------------- 路径配置（清晰不变） --------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = r"D:\kitti\KITTI_07"
INPUT_FILE = os.path.join(DATA_PATH, 'fusion_pose_ekf.txt')
OUTPUT_FILE = os.path.join(DATA_PATH, 'fusion_pose.txt')
OXTS_PATH = os.path.join(DATA_PATH, 'oxts', 'data')

# -------------------------- 常量定义（新增，便于维护） --------------------------
PI = 3.141592653589793
OXTS_AX_INDEX = 11
OXTS_AY_INDEX = 12
OXTS_AZ_INDEX = 13
OXTS_WX_INDEX = 17
OXTS_WY_INDEX = 18
OXTS_WZ_INDEX = 19

# -------------------------- 主程序 --------------------------
def main():
    print("=" * 60)
    print("开始转换 EKF 数据为 MATLAB 格式".center(60))
    print("=" * 60)
    print(f"📥 输入文件: {INPUT_FILE}")
    print(f"📤 输出文件: {OUTPUT_FILE}")
    print(f"📊 OXTS 路径: {OXTS_PATH}")
    print("-" * 60)

    with open(INPUT_FILE, 'r', encoding='utf-8') as f_in, \
         open(OUTPUT_FILE, 'w', encoding='utf-8') as f_out:

        # MATLAB 头部注释（格式更整齐）
        header_text = (
            "% KITTI 07 EKF Fusion Data\n"
            "% timestamp(s) x(m) y(m) z(m) roll(deg) pitch(deg) yaw(deg) "
            "vx(m/s) vy(m/s) vz(m/s) ax(m/s2) ay(m/s2) az(m/s2) "
            "wx(rad/s) wy(rad/s) wz(rad/s)\n"
        )
        f_out.write(header_text)

        # 跳过 CSV 表头
        f_in.readline()
        count = 0

        # 逐行处理
        for line_num, line in enumerate(f_in, start=1):
            line = line.strip()
            if not line:
                continue

            # 分割数据
            parts = line.split(',')
            if len(parts) < 16:
                print(f"⚠️  警告: 第{line_num}行数据长度不足，跳过")
                continue

            # 解析字段（命名更清晰，注释更明确）
            timestamp = float(parts[0])
            pos_x, pos_y, pos_z = map(float, parts[1:4])
            roll, pitch, yaw = map(float, parts[4:7])
            vel_x, vel_y, vel_z = map(float, parts[10:13])

            # 弧度转角度
            roll_deg  = np.rad2deg(roll)
            pitch_deg = np.rad2deg(pitch)
            yaw_deg   = np.rad2deg(yaw)

            # 读取 OXTS IMU 数据（容错更强）
            oxts_file = os.path.join(OXTS_PATH, f'{count:010d}.txt')
            try:
                with open(oxts_file, 'r', encoding='utf-8') as f_oxts:
                    oxts_data = list(map(float, f_oxts.readline().split()))

                ax = oxts_data[OXTS_AX_INDEX]
                ay = oxts_data[OXTS_AY_INDEX]
                az = oxts_data[OXTS_AZ_INDEX]
                wx = oxts_data[OXTS_WX_INDEX]
                wy = oxts_data[OXTS_WY_INDEX]
                wz = oxts_data[OXTS_WZ_INDEX]

            except FileNotFoundError:
                ax = ay = az = wx = wy = wz = 0.0
                print(f"ℹ️  信息: 第{count}帧 OXTS 文件不存在，使用 0 填充")
            except Exception as e:
                ax = ay = az = wx = wy = wz = 0.0
                print(f"⚠️  警告: 第{count}帧 OXTS 读取异常: {str(e)}")

            # 写入输出文件（格式统一、可读性强）
            output_line = (
                f"{timestamp:<10.6f} {pos_x:<12.6f} {pos_y:<12.6f} {pos_z:<12.6f} "
                f"{roll_deg:<10.6f} {pitch_deg:<10.6f} {yaw_deg:<10.6f} "
                f"{vel_x:<8.6f} {vel_y:<8.6f} {vel_z:<8.6f} "
                f"{ax:<10.6f} {ay:<10.6f} {az:<10.6f} "
                f"{wx:<10.6f} {wy:<10.6f} {wz:<10.6f}\n"
            )
            f_out.write(output_line)
            count += 1

    print("-" * 60)
    print(f"✅ 转换完成！总计处理 {count} 帧数据")
    print(f"✅ 文件已保存至: {OUTPUT_FILE}")
    print("=" * 60)

if __name__ == "__main__":
    main()
