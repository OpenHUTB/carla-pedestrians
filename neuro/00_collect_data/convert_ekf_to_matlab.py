#!/usr/bin/env python3
"""
将CSV格式的EKF融合数据转换为MATLAB格式
同时从OXTS读取真实IMU数据
跨平台兼容：Windows / Linux / macOS
无绝对路径，使用相对路径运行
"""

import os
import numpy as np

# ===================== 路径配置（跨平台通用） =====================
# 获取当前脚本所在目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 数据目录：脚本所在目录下的 KITTI_07 文件夹
DATA_PATH = os.path.join(SCRIPT_DIR, "KITTI_07")

# 输入输出文件（自动跨平台拼接路径）
INPUT_FILE = os.path.join(DATA_PATH, "fusion_pose_ekf.txt")
OUTPUT_FILE = os.path.join(DATA_PATH, "fusion_pose.txt")
OXTS_PATH = os.path.join(DATA_PATH, "oxts", "data")

# ===================== 常量定义 =====================
PI = np.pi
OXTS_AX_INDEX = 11
OXTS_AY_INDEX = 12
OXTS_AZ_INDEX = 13
OXTS_WX_INDEX = 17
OXTS_WY_INDEX = 18
OXTS_WZ_INDEX = 19

# ===================== 主函数 =====================
def main():
    print("=" * 60)
    print("EKF 数据转 MATLAB 格式（跨平台版）".center(60))
    print("=" * 60)
    print(f"📂 脚本目录: {SCRIPT_DIR}")
    print(f"📥 输入文件: {INPUT_FILE}")
    print(f"📤 输出文件: {OUTPUT_FILE}")
    print(f"📊 OXTS 路径: {OXTS_PATH}")
    print("-" * 60)

    # 统一 UTF-8 编码，全平台兼容
    try:
        with open(INPUT_FILE, "r", encoding="utf-8") as f_in, \
             open(OUTPUT_FILE, "w", encoding="utf-8") as f_out:

            # 写入 MATLAB 头部
            header = (
                "% KITTI 07 EKF Fusion Data\n"
                "% timestamp x y z roll_deg pitch_deg yaw_deg "
                "vx vy vz ax ay az wx wy wz\n"
            )
            f_out.write(header)

            # 跳过 CSV 表头
            f_in.readline()
            count = 0

            for line_num, line in enumerate(f_in, start=1):
                line = line.strip()
                if not line:
                    continue

                parts = line.split(",")
                if len(parts) < 16:
                    print(f"⚠️  第 {line_num} 行数据不完整，跳过")
                    continue

                # 解析数据
                timestamp = float(parts[0])
                x, y, z = map(float, parts[1:4])
                roll, pitch, yaw = map(float, parts[4:7])
                vx, vy, vz = map(float, parts[10:13])

                # 弧度转角度
                roll_deg = np.rad2deg(roll)
                pitch_deg = np.rad2deg(pitch)
                yaw_deg = np.rad2deg(yaw)

                # 读取 OXTS 数据
                oxts_file = os.path.join(OXTS_PATH, f"{count:010d}.txt")
                ax = ay = az = 0.0
                wx = wy = wz = 0.0

                if os.path.exists(oxts_file):
                    try:
                        with open(oxts_file, "r", encoding="utf-8") as f_oxts:
                            data = list(map(float, f_oxts.readline().split()))
                        ax = data[OXTS_AX_INDEX]
                        ay = data[OXTS_AY_INDEX]
                        az = data[OXTS_AZ_INDEX]
                        wx = data[OXTS_WX_INDEX]
                        wy = data[OXTS_WY_INDEX]
                        wz = data[OXTS_WZ_INDEX]
                    except:
                        pass

                # 写入格式化数据
                f_out.write(
                    f"{timestamp:<10.6f} {x:<12.6f} {y:<12.6f} {z:<12.6f} "
                    f"{roll_deg:<10.6f} {pitch_deg:<10.6f} {yaw_deg:<10.6f} "
                    f"{vx:<8.6f} {vy:<8.6f} {vz:<8.6f} "
                    f"{ax:<10.6f} {ay:<10.6f} {az:<10.6f} "
                    f"{wx:<10.6f} {wy:<10.6f} {wz:<10.6f}\n"
                )
                count += 1

        print("-" * 60)
        print(f"✅ 转换完成！共处理 {count} 帧数据")
        print(f"✅ 文件已保存：{OUTPUT_FILE}")

    except FileNotFoundError:
        print("\n❌ 错误：未找到数据文件，请确保 KITTI_07 文件夹在脚本同目录下")
    except Exception as e:
        print(f"\n❌ 运行错误：{str(e)}")

    print("=" * 60)

if __name__ == "__main__":
    main()
