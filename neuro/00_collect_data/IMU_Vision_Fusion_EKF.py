import os.path
import shutil
import math
import random
import numpy as np
import cv2
import queue
import time
import carla
import weakref
from scipy.spatial.transform import Rotation as R

# -------------------------- 路径配置 --------------------------
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
carla_api_path = os.path.join(current_dir, '../../../../carla-0.9.15/PythonAPI/carla')
sys.path.append(carla_api_path)
from agents.navigation.behavior_agent import BehaviorAgent

# -------------------------- 配置参数 --------------------------
TARGET_MAP = "Town10HD"
MAX_SAVE_IMG = 5000
OUTPUT_DIR = '../data/01_NeuroSLAM_Datasets/Town10Data/'

EXPOSURE_MODE = "manual"
EXPOSURE_COMPENSATION = "0.0" 
FSTOP = "4.0"
ISO = "250"  
GAMMA = "2.2"
AGENT_BEHAVIOR = "cautious"
AGENT_MAX_SPEED = 30  # km/h

# -------------------------- 官方碰撞传感器 --------------------------
class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = event.other_actor.type_id.split('.')[-1]
        print(f"⚠️  碰撞检测：与 {actor_type} 发生碰撞")

# -------------------------- 全局工具函数 --------------------------
def clear_all_actors(world):
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
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise ValueError(f"地图 {TARGET_MAP} 未找到生成点！")
    print(f"地图 {TARGET_MAP} 找到 {len(spawn_points)} 个生成点")

    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle_bp.set_attribute('role_name', 'hero')
    vehicle = None
    for attempt in range(max_attempts):
        chosen_spawn = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(vehicle_bp, chosen_spawn)
        if vehicle is not None:
            print(f"第{attempt+1}次尝试成功，生成车辆")
            return vehicle, spawn_points
        print(f"第{attempt+1}次生成失败，重试...")
        time.sleep(1)
    raise RuntimeError(f"连续{max_attempts}次生成失败！")

# -------------------------- 初始化CARLA环境 --------------------------
def init_carla_environment():
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    try:
        world = client.get_world()
        print("成功连接CARLA服务器")
    except Exception as e:
        raise ConnectionError(f"连接失败: {e}\n请先启动服务器：./CarlaUE4.sh")
    
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    
    clear_all_actors(world)
    
    print(f"加载地图 {TARGET_MAP}...")
    client.load_world(TARGET_MAP)
    time.sleep(3)
    world = client.get_world()
    print(f"地图加载完成: {world.get_map().name}")
    
    for tl in world.get_actors().filter('traffic.traffic_light*'):
        try:
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)
        except Exception as e:
            print(f"配置交通灯警告: {e}")
    
    bp_lib = world.get_blueprint_library()
    vehicle, spawn_points = safe_spawn_vehicle(world, bp_lib)
    try:
        physics_control = vehicle.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle.apply_physics_control(physics_control)
    except Exception as e:
        print(f"配置车辆物理参数警告: {e}")
    
    agent = BehaviorAgent(vehicle, behavior=AGENT_BEHAVIOR)
    agent.follow_speed_limits(True)
    destination = random.choice(spawn_points).location
    agent.set_destination(destination)
    print(f"避障智能体初始化完成，目标: ({destination.x:.1f}, {destination.y:.1f})")
    
    collision_sensor = CollisionSensor(vehicle)
    
    try:
        if os.path.exists(OUTPUT_DIR):
            shutil.rmtree(OUTPUT_DIR)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
    except PermissionError:
        raise PermissionError(f"无权限操作目录: {OUTPUT_DIR}")
    print(f"输出目录: {OUTPUT_DIR}")
    
    return world, bp_lib, vehicle, spawn_points, world.get_spectator(), agent, traffic_manager, collision_sensor

# -------------------------- 传感器配置 --------------------------
class SensorData:
    def __init__(self, data_type, timestamp, data):
        self.data_type = data_type
        self.timestamp = timestamp
        self.data = data

# -------------------------- 核心修改：同步参考代码的相机配置及图像处理 --------------------------
def create_rgb_camera(world, bp_lib, vehicle, data_queue):
    rgb_bp = bp_lib.find('sensor.camera.rgb')
    # 完全同步参考代码的相机参数
    rgb_bp.set_attribute("image_size_x", "640")
    rgb_bp.set_attribute("image_size_y", "480")
    rgb_bp.set_attribute("sensor_tick", "0.05")
    rgb_bp.set_attribute("exposure_mode", EXPOSURE_MODE)
    rgb_bp.set_attribute("exposure_compensation", EXPOSURE_COMPENSATION)
    rgb_bp.set_attribute("fstop", FSTOP)
    rgb_bp.set_attribute("iso", ISO)  
    rgb_bp.set_attribute("gamma", GAMMA)
    # 新增参考代码中的图像增强关闭参数，避免色彩失真
    rgb_bp.set_attribute("bloom_intensity", "0.0")
    rgb_bp.set_attribute("chromatic_aberration_intensity", "0.0")
    rgb_bp.set_attribute("lens_flare_intensity", "0.0")
    
    transform = carla.Transform(
        carla.Location(x=0.2, y=0, z=4.2),  
        carla.Rotation(pitch=-20)
    )
    
    # 同步参考代码的图像回调逻辑：直接保存原始RGB数据，避免格式转换失真
    def image_callback(image):
        # 参考代码逻辑：不做ColorConverter转换，直接保留原始数据
        data_queue.put(SensorData('image', image.timestamp, image))
    
    camera = world.spawn_actor(rgb_bp, transform, attach_to=vehicle)
    camera.listen(image_callback)
    print("RGB相机初始化完成（已同步参考代码色彩参数）")
    return camera, transform

def create_imu_sensor(world, bp_lib, vehicle, data_queue, transform):
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_bp.set_attribute('sensor_tick', str(1/60))
    imu_bp.set_attribute('noise_accel_stddev_x', '0.1')
    imu_bp.set_attribute('noise_gyro_stddev_x', '0.001')
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    imu.listen(lambda data: data_queue.put(SensorData('imu', data.timestamp, data)))
    print("IMU传感器初始化完成")
    return imu

# -------------------------- 时间戳对齐 --------------------------
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

# -------------------------- EKF融合--------------------------
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
        H[0,0] = H[1,1] = H[2,2] = 1
        H[3,6] = H[4,7] = H[5,8] = 1

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

# -------------------------- 图像后处理 --------------------------
def save_image_simple(img_array, output_dir, idx, target_width=160, target_height=120):
    """缩放为160×120，同步参考代码色彩处理逻辑"""
    try:
        # 参考代码逻辑：直接使用RGB数据，不做RGBA2BGR转换
        resized_img = cv2.resize(img_array, (target_width, target_height), interpolation=cv2.INTER_LANCZOS4)
        img_path = os.path.join(output_dir, f"{idx:04d}.png")
        cv2.imwrite(img_path, resized_img)
        return True
    except Exception as e:
        print(f"保存图像{idx}失败: {e}")
        return False

# -------------------------- 主循环 --------------------------
def main():
    try:
        world, bp_lib, vehicle, spawn_points, spectator, agent, traffic_manager, collision_sensor = init_carla_environment()
    except Exception as e:
        print(f"初始化失败: {e}")
        return

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    sensor_queue = queue.Queue(maxsize=1000)
    try:
        camera, cam_transform = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
        imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
    except Exception as e:
        print(f"传感器初始化失败: {e}")
        clear_all_actors(world)
        return

    time_aligner = TimeAligner()
    init_pose = vehicle.get_transform()
    init_pos = [init_pose.location.x, init_pose.location.y, init_pose.location.z]
    init_att = [math.radians(init_pose.rotation.roll),
                math.radians(init_pose.rotation.pitch),
                math.radians(init_pose.rotation.yaw)]
    ekf = EKF_VIO(init_pos+init_att, [0,0,0])

    # 数据保存参数（每1帧对齐数据保存1帧，确保时间戳同步）
    img_idx = 0
    fusion_log = open(os.path.join(OUTPUT_DIR, 'fusion_pose.txt'), 'w')
    fusion_log.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,imu_pos_x,imu_pos_y,imu_pos_z\n")

    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
    stagnant_count = 0

    try:
        while True:                                                                                                  
            world.tick()
    
            while not sensor_queue.empty():
                try:
                    data = sensor_queue.get(block=False)
                    time_aligner.add_data(data)
                except:
                    break


            # 处理对齐的图像和IMU数据
            for img_data, imu_data in time_aligner.get_aligned_pairs():
                ekf.imu_prediction(imu_data.data)
                imu_pos, _ = ekf.get_current_pose()

                carla_pose = vehicle.get_transform()
                visual_pose = [
                    carla_pose.location.x, carla_pose.location.y, carla_pose.location.z,
                    math.radians(carla_pose.rotation.roll),
                    math.radians(carla_pose.rotation.pitch),
                    math.radians(carla_pose.rotation.yaw)
                ]

                ekf.visual_update(visual_pose)
                fusion_pos, fusion_att = ekf.get_current_pose()

                # 参考代码逻辑：raw_data为RGBA格式，取前3通道作为RGB
                img = np.reshape(np.copy(img_data.data.raw_data), (480, 640, 4))[:, :, :3]  # 仅保留RGB通道

                # 保存图像（保持每1帧对齐数据保存1帧，确保与IMU时间戳同步）
                img_idx += 1
                save_image_simple(img, OUTPUT_DIR, img_idx)  # 使用同步后的保存函数
                print(f"保存图像 {img_idx}/{MAX_SAVE_IMG}")

                # 保存IMU数据（与图像时间戳严格对齐）
                with open(os.path.join(OUTPUT_DIR, 'aligned_imu.txt'), 'a') as f:
                    f.write(f"{imu_data.timestamp:.6f},"
                            f"{imu_data.data.accelerometer.x:.6f},{imu_data.data.accelerometer.y:.6f},{imu_data.data.accelerometer.z:.6f},"
                            f"{imu_data.data.gyroscope.x:.6f},{imu_data.data.gyroscope.y:.6f},{imu_data.data.gyroscope.z:.6f}\n")

                # 保存融合结果
                fusion_log.write(f"{img_data.timestamp:.6f},"
                                f"{fusion_pos[0]:.6f},{fusion_pos[1]:.6f},{fusion_pos[2]:.6f},"
                                f"{math.degrees(fusion_att[0]):.6f},{math.degrees(fusion_att[1]):.6f},{math.degrees(fusion_att[2]):.6f},"
                                f"{imu_pos[0]:.6f},{imu_pos[1]:.6f},{imu_pos[2]:.6f}\n")

                if img_idx >= MAX_SAVE_IMG:
                    print("达到最大保存数量，退出")
                    return

                # 显示原始图像
                cv2.imshow('RGB Camera', img)

            # 避障智能体控制（保持不变）
            if agent.done():
                destination = random.choice(spawn_points).location
                agent.set_destination(destination)
                print(f"已到达，新目标：({destination.x:.1f}, {destination.y:.1f})")
            control = agent.run_step()
            control.manual_gear_shift = False
            vehicle.apply_control(control)

            # 停滞检测与重置
            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x**2 + vel.y**2)
            reset_needed = False
            if speed < 0.1:
                stagnant_count += 1
                if stagnant_count > 100:
                    print("车辆停滞，重新生成...")
                    reset_needed = True
            else:
                stagnant_count = 0

            if reset_needed:
                camera.stop()
                imu.stop()
                camera.destroy()
                imu.destroy()
                vehicle.destroy()
                vehicle, _ = safe_spawn_vehicle(world, bp_lib)
                try:
                    physics_control = vehicle.get_physics_control()
                    physics_control.use_sweep_wheel_collision = True
                    vehicle.apply_physics_control(physics_control)
                except:
                    pass
                agent = BehaviorAgent(vehicle, behavior=AGENT_BEHAVIOR)
                agent.set_max_speed(AGENT_MAX_SPEED / 3.6)
                agent.set_destination(random.choice(spawn_points).location)
                camera, _ = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)  # 重新初始化相机（使用同步参数）
                imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
                collision_sensor = CollisionSensor(vehicle)
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
        collision_sensor.sensor.stop()
        clear_all_actors(world)
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)
        cv2.destroyAllWindows()
        print("资源清理完成")


if __name__ == "__main__":
    main()
