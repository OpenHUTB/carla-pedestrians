import os.path
import shutil
import math
import random
import numpy as np
import cv2
import queue
import time
import carla
from scipy.spatial.transform import Rotation as R


# -------------------------- 配置参数（核心：修改此处切换地图名称） --------------------------
TARGET_MAP = "Town01"  # 可改为 "Town01" 或 "Town10HD"
MAX_SAVE_IMG = 5000     # 最大保存图像数
OUTPUT_DIR = '../data/01_NeuroSLAM_Datasets/Town10Data/'  # 数据输出目录


# -------------------------- 1. 全局工具函数 --------------------------
def clear_all_actors(world):
    """清理所有车辆、传感器、行人，避免资源残留"""
    for actor in world.get_actors().filter('vehicle.*.*'):
        try:
            actor.destroy()
        except Exception as e:
            print(f"清理车辆警告: {e}")
    for actor in world.get_actors().filter('sensor.*.*'):
        try:
            actor.stop()
            actor.destroy()
        except Exception as e:
            print(f"清理传感器警告: {e}")
    for actor in world.get_actors().filter('walker.*.*'):
        try:
            actor.destroy()
        except Exception as e:
            print(f"清理行人警告: {e}")
    time.sleep(1)


def safe_spawn_vehicle(world, bp_lib, max_attempts=10):
    """安全生成车辆，适配不同地图的生成点"""
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise ValueError(f"地图 {TARGET_MAP} 未找到生成点，请检查地图是否正确加载！")
    print(f"地图 {TARGET_MAP} 找到 {len(spawn_points)} 个生成点")

    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = None
    for attempt in range(max_attempts):
        chosen_spawn = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(vehicle_bp, chosen_spawn)
        if vehicle is not None:
            print(f"第{attempt+1}次尝试成功，生成车辆于: ({chosen_spawn.location.x:.1f}, {chosen_spawn.location.y:.1f})")
            return vehicle, spawn_points
        print(f"第{attempt+1}次生成失败，换生成点重试...")
        time.sleep(1)
    
    raise RuntimeError(f"连续{max_attempts}次生成车辆失败，可能生成点被占用！")


# -------------------------- 2. 初始化CARLA环境 --------------------------
def init_carla_environment():
    """初始化环境，根据TARGET_MAP加载对应地图"""
    # 连接服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    try:
        world = client.get_world()
    except Exception as e:
        raise ConnectionError(f"连接CARLA失败: {e}\n请先启动服务器：./CarlaUE4.sh -RenderOffScreen")
    
    # 清理残留资源
    clear_all_actors(world)
    
    # 加载目标地图
    print(f"加载地图 {TARGET_MAP}...")
    client.load_world(TARGET_MAP)
    world = client.get_world()  # 刷新世界对象
    
    # 交通灯全绿灯
    for tl in world.get_actors().filter('traffic.traffic_light*'):
        try:
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)
        except Exception as e:
            print(f"配置交通灯警告: {e}")
    
    # 生成车辆
    bp_lib = world.get_blueprint_library()
    vehicle, spawn_points = safe_spawn_vehicle(world, bp_lib)
    vehicle.set_autopilot(True)
    
    # 准备输出目录
    try:
        if os.path.exists(OUTPUT_DIR):
            shutil.rmtree(OUTPUT_DIR)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
    except PermissionError:
        raise PermissionError(f"无权限操作目录: {OUTPUT_DIR}")
    print(f"输出目录: {OUTPUT_DIR}")
    
    return world, bp_lib, vehicle, spawn_points, world.get_spectator()


# -------------------------- 3. 传感器配置 --------------------------
class SensorData:
    def __init__(self, data_type, timestamp, data):
        self.data_type = data_type  # 'image' 或 'imu'
        self.timestamp = timestamp
        self.data = data


def create_rgb_camera(world, bp_lib, vehicle, data_queue):
    rgb_bp = bp_lib.find('sensor.camera.rgb')
    rgb_bp.set_attribute("image_size_x", "640")
    rgb_bp.set_attribute("image_size_y", "480")
    rgb_bp.set_attribute("sensor_tick", "0.05")  # 20Hz
    transform = carla.Transform(carla.Location(x=0.2, y=0, z=4.2), carla.Rotation(pitch=-20))
    camera = world.spawn_actor(rgb_bp, transform, attach_to=vehicle)
    camera.listen(lambda img: data_queue.put(SensorData('image', img.timestamp, img)))
    print("RGB相机初始化完成")
    return camera, transform


def create_imu_sensor(world, bp_lib, vehicle, data_queue, transform):
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_bp.set_attribute('sensor_tick', str(1/60))  # 60Hz
    imu_bp.set_attribute('noise_accel_stddev_x', '0.1')
    imu_bp.set_attribute('noise_gyro_stddev_x', '0.001')
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    imu.listen(lambda data: data_queue.put(SensorData('imu', data.timestamp, data)))
    print("IMU传感器初始化完成")
    return imu


# -------------------------- 4. 时间戳对齐 --------------------------
class TimeAligner:
    def __init__(self, time_threshold=0.02):
        self.time_threshold = time_threshold
        self.imu_buffer = []
        self.image_buffer = []
        self.max_buffer = 100

    def add_data(self, data):
        if data.data_type == 'imu':
            self.imu_buffer.append(data)
            self.imu_buffer.sort(key=lambda x: x.timestamp)
            if len(self.imu_buffer) > self.max_buffer:
                self.imu_buffer.pop(0)
        elif data.data_type == 'image':
            self.image_buffer.append(data)
            self.image_buffer.sort(key=lambda x: x.timestamp)
            if len(self.image_buffer) > self.max_buffer//10:
                self.image_buffer.pop(0)

    def get_aligned_pairs(self):
        pairs = []
        if not self.image_buffer or not self.imu_buffer:
            return pairs
        for img in self.image_buffer:
            diffs = [abs(img.timestamp - imu.timestamp) for imu in self.imu_buffer]
            min_idx = np.argmin(diffs)
            if diffs[min_idx] <= self.time_threshold:
                pairs.append((img, self.imu_buffer[min_idx]))
                del self.imu_buffer[min_idx]
        self.image_buffer = []
        return pairs


# -------------------------- 5. EKF融合 --------------------------
class EKF_VIO:
    def __init__(self, init_pose, init_vel, dt=0.05):
        self.x = np.array([
            init_pose[0], init_pose[1], init_pose[2],
            init_vel[0], init_vel[1], init_vel[2],
            init_pose[3], init_pose[4], init_pose[5]
        ], dtype=np.float64)
        self.dt = dt
        self.P = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.01, 0.01, 0.01])
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001])
        self.R = np.diag([0.05, 0.05, 0.05, 0.005, 0.005, 0.005])

    def imu_prediction(self, imu_data):
        accel = np.array([imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z])
        gyro = np.array([imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z])

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        new_roll = (roll + gyro[0]*self.dt + np.pi) % (2*np.pi) - np.pi
        new_pitch = (pitch + gyro[1]*self.dt + np.pi) % (2*np.pi) - np.pi
        new_yaw = (yaw + gyro[2]*self.dt + np.pi) % (2*np.pi) - np.pi

        R_body2world = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        accel_world = R_body2world @ accel
        new_vx = self.x[3] + accel_world[0] * self.dt
        new_vy = self.x[4] + accel_world[1] * self.dt
        new_vz = self.x[5] + (accel_world[2] - 9.81) * self.dt

        new_x = self.x[0] + new_vx * self.dt
        new_y = self.x[1] + new_vy * self.dt
        new_z = self.x[2] + new_vz * self.dt

        self.x = np.array([new_x, new_y, new_z, new_vx, new_vy, new_vz, new_roll, new_pitch, new_yaw])
        self.P += self.Q

    def visual_update(self, visual_pose):
        z = np.array(visual_pose, dtype=np.float64)
        H = np.zeros((6, 9))
        H[0,0] = H[1,1] = H[2,2] = 1  # 位置观测
        H[3,6] = H[4,7] = H[5,8] = 1  # 姿态观测

        y = z - H @ self.x
        try:
            S = H @ self.P @ H.T + self.R + np.eye(6)*1e-8
            K = self.P @ H.T @ np.linalg.inv(S)
        except:
            K = np.eye(9,6)*0.1

        self.x += K @ y
        self.P = (np.eye(9) - K@H) @ self.P
        self.P = 0.5*(self.P + self.P.T)

    def get_current_pose(self):
        return self.x[:3].copy(), self.x[6:9].copy()


# -------------------------- 6. 主循环 --------------------------
def main():
    try:
        world, bp_lib, vehicle, spawn_points, spectator = init_carla_environment()
    except Exception as e:
        print(f"初始化失败: {e}")
        return

    # 启用同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 初始化传感器
    sensor_queue = queue.Queue(maxsize=1000)
    try:
        camera, cam_transform = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
        imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
    except Exception as e:
        print(f"传感器初始化失败: {e}")
        clear_all_actors(world)
        return

    # 初始化对齐器和EKF
    time_aligner = TimeAligner()
    init_pose = vehicle.get_transform()
    init_pos = [init_pose.location.x, init_pose.location.y, init_pose.location.z]
    init_att = [math.radians(init_pose.rotation.roll),
                math.radians(init_pose.rotation.pitch),
                math.radians(init_pose.rotation.yaw)]
    ekf = EKF_VIO(init_pos+init_att, [0,0,0])

    # 数据保存
    img_idx = 0
    fusion_log = open(os.path.join(OUTPUT_DIR, 'fusion_pose.txt'), 'w')
    fusion_log.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,imu_pos_x,imu_pos_y,imu_pos_z\n")

    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
    stagnant_count = 0  # 用于检测车辆停滞

    try:
        while True:
            world.tick()

            # 处理传感器数据
            while not sensor_queue.empty():
                try:
                    data = sensor_queue.get(block=False)
                    time_aligner.add_data(data)
                except:
                    break

            # 处理对齐数据
            for img_data, imu_data in time_aligner.get_aligned_pairs():
                # IMU预测
                ekf.imu_prediction(imu_data.data)
                imu_pos, _ = ekf.get_current_pose()

                # 视觉观测（CARLA真实位姿）
                carla_pose = vehicle.get_transform()
                visual_pose = [
                    carla_pose.location.x, carla_pose.location.y, carla_pose.location.z,
                    math.radians(carla_pose.rotation.roll),
                    math.radians(carla_pose.rotation.pitch),
                    math.radians(carla_pose.rotation.yaw)
                ]

                # EKF更新
                ekf.visual_update(visual_pose)
                fusion_pos, fusion_att = ekf.get_current_pose()

                # 保存数据
                img_idx += 1
                img_path = os.path.join(OUTPUT_DIR, f"{img_idx:04d}.png")
                img = np.reshape(np.copy(img_data.data.raw_data), (480, 640, 4))
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                cv2.imwrite(img_path, cv2.resize(img, (160, 120)))
                print(f"保存图像 {img_idx}/{MAX_SAVE_IMG}: {img_path}")

                # 保存IMU
                with open(os.path.join(OUTPUT_DIR, 'aligned_imu.txt'), 'a') as f:
                    f.write(f"{imu_data.timestamp:.6f},"
                            f"{imu_data.data.accelerometer.x:.6f},{imu_data.data.accelerometer.y:.6f},{imu_data.data.accelerometer.z:.6f},"
                            f"{imu_data.data.gyroscope.x:.6f},{imu_data.data.gyroscope.y:.6f},{imu_data.data.gyroscope.z:.6f}\n")

                # 保存融合结果
                fusion_log.write(f"{img_data.timestamp:.6f},"
                                f"{fusion_pos[0]:.6f},{fusion_pos[1]:.6f},{fusion_pos[2]:.6f},"
                                f"{math.degrees(fusion_att[0]):.6f},{math.degrees(fusion_att[1]):.6f},{math.degrees(fusion_att[2]):.6f},"
                                f"{imu_pos[0]:.6f},{imu_pos[1]:.6f},{imu_pos[2]:.6f}\n")

                # 显示图像
                cv2.putText(img, f"Pos: {fusion_pos[:2]}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                cv2.imshow('RGB Camera', img)

                if img_idx >= MAX_SAVE_IMG:
                    print("达到最大保存数量，退出")
                    return

            # 检测车辆是否停滞（避免碰撞后采集重复数据）
            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x**2 + vel.y**2)
            if speed < 0.1:
                stagnant_count += 1
                if stagnant_count > 100:  # 停滞5秒（100*0.05）
                    print("车辆停滞，重新生成...")
                    camera.stop()
                    imu.stop()
                    camera.destroy()
                    imu.destroy()
                    vehicle.destroy()
                    vehicle, _ = safe_spawn_vehicle(world, bp_lib)
                    vehicle.set_autopilot(True)
                    camera, _ = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
                    imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
                    stagnant_count = 0
            else:
                stagnant_count = 0

            # 跟随视角
            spec_transform = carla.Transform(
                vehicle.get_transform().transform(carla.Location(x=-4, z=50)),
                carla.Rotation(yaw=-180, pitch=-90)
            )
            spectator.set_transform(spec_transform)

            if cv2.waitKey(1) == ord('q'):
                print("用户退出")
                return

    except Exception as e:
        print(f"主循环错误: {e}")
    finally:
        fusion_log.close()
        camera.stop()
        imu.stop()
        clear_all_actors(world)
        world.apply_settings(carla.WorldSettings(synchronous_mode=False))
        cv2.destroyAllWindows()
        print("资源清理完成")


if __name__ == "__main__":
    main()
