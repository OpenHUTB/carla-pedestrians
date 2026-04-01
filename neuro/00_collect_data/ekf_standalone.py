"""
独立的EKF类，用于离线数据处理
从IMU_Vision_Fusion_EKF.py提取，去除CARLA依赖
"""

import numpy as np

class EKF_VIO:
    """扩展卡尔曼滤波器 - 用于IMU和视觉里程计融合（全平台兼容版）"""

    def __init__(self, init_pose, init_vel, dt=0.05):
        """
        初始化EKF
        Args:
            init_pose: [x, y, z, roll, pitch, yaw]
            init_vel: [vx, vy, vz]
            dt: 时间步长
        """
        # 状态向量：位置(3) + 速度(3) + 姿态(3)
        self.x = np.array([
            init_pose[0], init_pose[1], init_pose[2],
            init_vel[0], init_vel[1], init_vel[2],
            init_pose[3], init_pose[4], init_pose[5]
        ], dtype=np.float64)

        self.dt = dt
        self.init_z = init_pose[2]

        # 协方差矩阵
        self.P = np.diag([
            0.01, 0.01, 0.01,
            0.1, 0.1, 0.1,
            0.001, 0.001, 0.001
        ])

        # 过程噪声（IMU）
        self.Q = np.diag([
            10.0, 10.0, 10.0,
            5.0, 5.0, 5.0,
            0.001, 0.001, 0.001
        ])

        # 观测噪声（视觉）
        self.R = np.diag([
            0.01, 0.01, 0.01,
            0.001, 0.001, 0.001
        ])

        # 历史记录
        self.innovation_history = []
        self.uncertainty_history = []

    def imu_prediction(self, imu_data):
        """IMU预测步骤"""
        accel = np.array([
            imu_data.accelerometer.x,
            imu_data.accelerometer.y,
            imu_data.accelerometer.z
        ])
        gyro = np.array([
            imu_data.gyroscope.x,
            imu_data.gyroscope.y,
            imu_data.gyroscope.z
        ])

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]

        # 姿态积分（安全角度归一化，跨平台稳定）
        new_roll = np.mod(roll + gyro[0] * self.dt + np.pi, 2 * np.pi) - np.pi
        new_pitch = np.mod(pitch + gyro[1] * self.dt + np.pi, 2 * np.pi) - np.pi
        new_yaw = np.mod(yaw + gyro[2] * self.dt + np.pi, 2 * np.pi) - np.pi

        # 旋转矩阵
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R_mat = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ], dtype=np.float64)

        # 重力补偿
        accel_world = R_mat @ accel - np.array([0, 0, -9.81], dtype=np.float64)

        # 运动积分
        new_vx = self.x[3] + accel_world[0] * self.dt
        new_vy = self.x[4] + accel_world[1] * self.dt
        new_vz = self.x[5] + accel_world[2] * self.dt

        new_x = self.x[0] + self.x[3] * self.dt
        new_y = self.x[1] + self.x[4] * self.dt
        new_z = self.x[2] + self.x[5] * self.dt

        # 更新状态
        self.x = np.array([
            new_x, new_y, new_z,
            new_vx, new_vy, new_vz,
            new_roll, new_pitch, new_yaw
        ], dtype=np.float64)

        self.P += self.Q

    def visual_update(self, visual_pose):
        """视觉观测更新步骤"""
        z = np.asarray(visual_pose, dtype=np.float64)
        H = np.eye(6, 9, dtype=np.float64)

        # 计算新息
        y = z - H @ self.x

        try:
            S = H @ self.P @ H.T + self.R + np.eye(6, dtype=np.float64) * 1e-8
            K = self.P @ H.T @ np.linalg.inv(S)

            self.innovation_history.append(float(np.linalg.norm(y)))
            self.uncertainty_history.append(float(np.trace(self.P[:3, :3])))
        except np.linalg.LinAlgError:
            K = np.eye(9, 6, dtype=np.float64) * 0.1

        # 状态更新
        self.x += K @ y

        # 协方差更新（Joseph 形式，数值稳定）
        I_KH = np.eye(9, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = (self.P + self.P.T) * 0.5  # 强制对称

        # 平地约束（固定高度与姿态）
        self.x[2] = self.init_z
        self.x[5] = 0.0
        self.x[6] = 0.0
        self.x[7] = 0.0

    def get_current_pose(self):
        """获取当前位姿"""
        return self.x[:3].copy(), self.x[6:9].copy()

    def get_current_velocity(self):
        """获取当前速度"""
        return self.x[3:6].copy()

    def get_position_uncertainty(self):
        """获取位置标准差"""
        return np.sqrt(np.diag(self.P[:3, :3]))

    def get_fusion_quality_metrics(self):
        """获取融合质量指标"""
        if len(self.innovation_history) == 0:
            return {'avg_innovation': 0.0, 'avg_uncertainty': 0.0}

        return {
            'avg_innovation': float(np.mean(self.innovation_history[-100:])),
            'avg_uncertainty': float(np.mean(self.uncertainty_history[-100:]))
        }
