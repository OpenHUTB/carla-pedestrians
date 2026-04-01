"""
独立的EKF类，用于离线数据处理
从IMU_Vision_Fusion_EKF.py提取，去除CARLA依赖
"""

import numpy as np

class EKF_VIO:
    """扩展卡尔曼滤波器 - 用于IMU和视觉里程计融合"""
    
    def __init__(self, init_pose, init_vel, dt=0.05):
        """
        初始化EKF
        
        Args:
            init_pose: [x, y, z, roll, pitch, yaw]
            init_vel: [vx, vy, vz]
            dt: 时间步长
        """
        self.x = np.array([
            init_pose[0], init_pose[1], init_pose[2],  # 位置
            init_vel[0], init_vel[1], init_vel[2],     # 速度
            init_pose[3], init_pose[4], init_pose[5]   # 姿态
        ], dtype=np.float64)
        self.dt = dt
        self.init_z = init_pose[2]  # 保存初始Z轴高度（平地车辆固定）
        
        # EKF协方差矩阵 - 极度信任视觉，IMU仅用于姿态和短期预测
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001])
        # 极大过程噪声（完全不信任IMU位置积分，只用IMU姿态）
        self.Q = np.diag([10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 0.001, 0.001, 0.001])
        # 极小视觉观测噪声（几乎完全信任VO）
        self.R = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        
        # 统计信息
        self.innovation_history = []
        self.uncertainty_history = []

    def imu_prediction(self, imu_data):
        """
        IMU预测步骤
        
        Args:
            imu_data: 包含 accelerometer 和 gyroscope 属性的对象
        """
        accel = np.array([imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z])
        gyro = np.array([imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z])

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        new_roll = np.mod(roll + gyro[0]*self.dt + np.pi, 2*np.pi) - np.pi
        new_pitch = np.mod(pitch + gyro[1]*self.dt + np.pi, 2*np.pi) - np.pi
        new_yaw = np.mod(yaw + gyro[2]*self.dt + np.pi, 2*np.pi) - np.pi

        # 旋转矩阵（从车体坐标系到世界坐标系）
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R_mat = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

        # 重力补偿（假设重力向下为-9.81）
        accel_world = R_mat @ accel - np.array([0, 0, -9.81])
        
        new_vx = self.x[3] + accel_world[0]*self.dt
        new_vy = self.x[4] + accel_world[1]*self.dt
        new_vz = self.x[5] + accel_world[2]*self.dt
        new_x = self.x[0] + self.x[3]*self.dt
        new_y = self.x[1] + self.x[4]*self.dt
        new_z = self.x[2] + self.x[5]*self.dt

        self.x = np.array([new_x, new_y, new_z, new_vx, new_vy, new_vz, new_roll, new_pitch, new_yaw], dtype=np.float64)
        self.P += self.Q

    def visual_update(self, visual_pose):
        """
        视觉观测更新步骤
        
        Args:
            visual_pose: numpy数组 [x, y, z, roll, pitch, yaw]
        """
        z = np.array(visual_pose, dtype=np.float64)
        H = np.zeros((6, 9))
        H[0,0] = H[1,1] = H[2,2] = 1
        H[3,6] = H[4,7] = H[5,8] = 1

        y = z - H @ self.x  # 新息(innovation)
        try:
            S = H @ self.P @ H.T + self.R + np.eye(6)*1e-8
            K = self.P @ H.T @ np.linalg.inv(S)
            
            # 记录新息和不确定性用于质量评估
            self.innovation_history.append(np.linalg.norm(y))
            self.uncertainty_history.append(np.trace(self.P[:3, :3]))  # 位置不确定性
        except Exception:
            K = np.eye(9,6)*0.1

        self.x += K @ y
        # Joseph形式协方差更新，保证数值稳定性
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = 0.5*(self.P + self.P.T)  # 保持对称性
        
        # 平地车辆：强制固定Z轴、roll、pitch
        self.x[2] = self.init_z  # Z轴固定
        self.x[5] = 0.0  # Z轴速度为0
        self.x[6] = 0.0  # roll = 0
        self.x[7] = 0.0  # pitch = 0

    def get_current_pose(self):
        """返回当前位姿 (位置, 姿态)"""
        return self.x[:3].copy(), self.x[6:9].copy()
    
    def get_current_velocity(self):
        """返回当前速度"""
        return self.x[3:6].copy()
    
    def get_position_uncertainty(self):
        """返回位置估计的不确定性(标准差)"""
        return np.sqrt(np.diag(self.P[:3, :3]))
    
    def get_fusion_quality_metrics(self):
        """返回融合质量指标"""
        if len(self.innovation_history) > 0:
            return {
                'avg_innovation': np.mean(self.innovation_history[-100:]),
                'avg_uncertainty': np.mean(self.uncertainty_history[-100:])
            }
        return {'avg_innovation': 0.0, 'avg_uncertainty': 0.0}
