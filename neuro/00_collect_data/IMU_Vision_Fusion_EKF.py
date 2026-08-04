# -*- coding: utf-8 -*-
"""
IMU + Visual Odometry EKF Fusion — CARLA 数据采集
用法: python IMU_Vision_Fusion_EKF.py [--headless] [--host HOST] [--port PORT]
"""

import os
import sys
import shutil
import math
import random
import time
import queue
import glob as _glob
import weakref

import numpy as np
import cv2
import carla
from scipy.spatial.transform import Rotation as R

# ─── 动态路径解析 ─────────────────────────────────────────────
current_dir = os.path.dirname(os.path.abspath(__file__))

# ─── 查找 CARLA agents 模块 ─────────────────────────────────
_carla_agents_found = False
_carla_agents_dir = None


def _search_agents():
    """搜索 CARLA agents 目录，返回 agents_dir 或 None"""
    _candidates = []
    _carla_root = os.environ.get('CARLA_ROOT', '')

    # 1) 优先使用 CARLA_ROOT 环境变量
    if _carla_root:
        _candidates = _glob.glob(os.path.join(_carla_root, 'PythonAPI', 'carla',
                                              'agents', 'navigation', '*.py'))

    # 2) 搜索用户主目录下常见 CARLA 目录名
    if not _candidates:
        _home = os.path.expanduser('~')
        _carla_dir_names = ['CARLA', 'carla', 'CARLA_0.9.16', 'CARLA_0.9.15',
                            'CARLA_0.9.14', 'carla-0.9.16']
        for _name in _carla_dir_names:
            _parent = os.path.join(_home, _name)
            if not os.path.isdir(_parent):
                continue
            # 直接搜索
            _candidates = _glob.glob(os.path.join(_parent, 'PythonAPI',
                                                  'carla', 'agents',
                                                  'navigation', '*.py'))
            if _candidates:
                break
            # 搜索一级子目录
            for _sub in os.listdir(_parent):
                _candidates = _glob.glob(os.path.join(_parent, _sub, 'PythonAPI',
                                                      'carla', 'agents',
                                                      'navigation', '*.py'))
                if _candidates:
                    break
            if _candidates:
                break

    # 3) 从脚本目录向上搜索（最多 5 层）
    if not _candidates:
        _search_dir = current_dir
        for _ in range(5):
            _candidates = _glob.glob(os.path.join(_search_dir, 'PythonAPI',
                                                  'carla', 'agents',
                                                  'navigation', '*.py'))
            if _candidates:
                break
            _parent = os.path.dirname(_search_dir)
            if _parent == _search_dir:
                break
            _search_dir = _parent

    if _candidates:
        return os.path.dirname(os.path.dirname(os.path.dirname(_candidates[0])))
    return None


_carla_agents_dir = _search_agents()
print(f"[DEBUG] Found agents dir: {_carla_agents_dir}")
if _carla_agents_dir:
    if _carla_agents_dir not in sys.path:
        sys.path.insert(0, _carla_agents_dir)
    _carla_agents_found = True

if not _carla_agents_found:
    raise ImportError(
        "未找到 CARLA agents 模块。请确保 CARLA 已安装，且 PythonAPI/carla/agents/ 目录存在。\n"
        "通常位于: <CARLA_ROOT>\\PythonAPI\\carla\\agents\\\n"
        "可通过设置环境变量 CARLA_ROOT 指定 CARLA 安装目录"
    )

from agents.navigation.behavior_agent import BehaviorAgent  # noqa: E402

# 导入视觉里程计
from visual_odometry_opencv import VisualOdometry, ScaleEstimator  # noqa: E402

# ═════════════════════════════════════════════════════════════
#  配置参数
# ═════════════════════════════════════════════════════════════

TARGET_MAP = "Town01"
MAX_SAVE_IMG = 5000
OUTPUT_DIR = os.path.join(current_dir, '..', 'data', 'Town01Data_IMU_Fusion')

# IMU-视觉融合参数
IMU_SAMPLE_RATE = 60       # Hz
CAMERA_SAMPLE_RATE = 20    # Hz (1/0.05)

EXPOSURE_MODE = "manual"
EXPOSURE_COMPENSATION = "0.0"
FSTOP = "4.0"
ISO = "250"
GAMMA = "2.2"
AGENT_BEHAVIOR = "cautious"
AGENT_MAX_SPEED = 20                # km/h
AGENT_SAFE_DISTANCE = 5.0           # 米
COLLISION_RESET_THRESHOLD = 3       # 碰撞次数阈值

# CARLA 连接参数（可通过命令行覆盖）
DEFAULT_CARLA_HOST = 'localhost'
DEFAULT_CARLA_PORT = 2000
CARLA_CONNECT_TIMEOUT = 60.0        # 单次连接超时
CARLA_MAX_RETRIES = 5               # 最大重试次数
CARLA_RETRY_DELAY = 5.0             # 重试间隔（秒）


# ═════════════════════════════════════════════════════════════
#  碰撞传感器
# ═════════════════════════════════════════════════════════════

class CollisionSensor:
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.collision_count = 0
        self.collision_history = []
        self.last_collision_time = 0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(),
                                        attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        current_time = time.time()
        if current_time - self.last_collision_time > 0.5:
            self.collision_count += 1
            self.last_collision_time = current_time
            actor_type = event.other_actor.type_id.split('.')[-1]
            impulse = event.normal_impulse
            intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
            self.collision_history.append({
                'time': current_time,
                'actor': actor_type,
                'intensity': intensity,
            })
            print(f"[COLLISION] [{self.collision_count}x]: {actor_type} "
                  f"(intensity: {intensity:.2f})")

    def reset_collision_count(self):
        self.collision_count = 0
        self.collision_history = []

    def has_major_collision(self):
        return self.collision_count >= COLLISION_RESET_THRESHOLD


# ═════════════════════════════════════════════════════════════
#  全局工具函数
# ═════════════════════════════════════════════════════════════

def clear_all_actors(world):
    """清理世界中所有动态 actor"""
    for actor_type in ['vehicle.*.*', 'sensor.*.*', 'walker.*.*']:
        for actor in world.get_actors().filter(actor_type):
            try:
                if 'sensor' in actor_type:
                    actor.stop()
                actor.destroy()
            except Exception:
                pass
    time.sleep(1)


def select_forward_destination(vehicle, spawn_points, min_distance=50.0):
    """选择车辆前方的目标点，优先直行路径"""
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_forward = vehicle_transform.get_forward_vector()

    forward_points = []
    for sp in spawn_points:
        to_spawn = sp.location - vehicle_location
        distance = to_spawn.length()
        if distance > min_distance:
            direction = to_spawn / distance
            dot_product = (vehicle_forward.x * direction.x +
                           vehicle_forward.y * direction.y)
            if dot_product > 0.7:
                forward_points.append((sp.location, distance, dot_product))

    if forward_points:
        forward_points.sort(key=lambda x: x[2], reverse=True)
        return forward_points[0][0]
    else:
        farthest = max(spawn_points,
                       key=lambda sp: (sp.location - vehicle_location).length())
        return farthest.location


def _get_available_vehicle_blueprint(bp_lib):
    """获取可用的车辆蓝图，支持自动回退"""
    preferred_vehicles = [
        'vehicle.lincoln.mkz_2017',
        'vehicle.tesla.model3',
        'vehicle.tesla.cybertruck',
        'vehicle.ford.mustang',
        'vehicle.dodge.charger_2020',
        'vehicle.audi.a2',
        'vehicle.audi.tt',
        'vehicle.chevrolet.impala',
        'vehicle.mini.cooper_s',
        'vehicle.nissan.patrol',
        'vehicle.bmw.grandtourer',
        'vehicle.jeep.wrangler_rubicon',
        'vehicle.mercedes.coupe',
        'vehicle.nissan.micra',
        'vehicle.citroen.c3',
        'vehicle.seat.leon',
        'vehicle.volkswagen.t2',
        'vehicle.subaru.brz',
        'vehicle.subaru.impreza',
    ]

    for vehicle_id in preferred_vehicles:
        try:
            bp = bp_lib.find(vehicle_id)
        except RuntimeError:
            continue
        if bp is not None:
            print(f"选择车辆蓝图: {vehicle_id}")
            return bp

    all_vehicles = list(bp_lib.filter('vehicle.*'))
    if not all_vehicles:
        raise RuntimeError("CARLA 蓝图库中没有任何车辆蓝图可用！")
    fallback = all_vehicles[0]
    print(f"[WARN] 所有优先车辆蓝图均不可用，回退至: {fallback.id}")
    return fallback


def safe_spawn_vehicle(world, bp_lib, max_attempts=10):
    """安全生成车辆"""
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise ValueError(f"地图 {TARGET_MAP} 未找到生成点！")
    print(f"地图 {TARGET_MAP} 找到 {len(spawn_points)} 个生成点")

    vehicle_bp = _get_available_vehicle_blueprint(bp_lib)
    vehicle_bp.set_attribute('role_name', 'hero')

    vehicle = None
    for attempt in range(max_attempts):
        chosen_spawn = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(vehicle_bp, chosen_spawn)
        if vehicle is not None:
            print(f"第{attempt + 1}次尝试成功，生成车辆")
            return vehicle, spawn_points
        print(f"第{attempt + 1}次生成失败，重试...")
        time.sleep(1)
    raise RuntimeError(f"连续{max_attempts}次生成失败！")


# ═════════════════════════════════════════════════════════════
#  CARLA 环境初始化（带重试机制）
# ═════════════════════════════════════════════════════════════

def connect_carla_with_retry(host, port, timeout=CARLA_CONNECT_TIMEOUT,
                             max_retries=CARLA_MAX_RETRIES):
    """带重试的 CARLA 客户端连接"""
    last_error = None
    for attempt in range(1, max_retries + 1):
        try:
            client = carla.Client(host, port)
            client.set_timeout(timeout)
            world = client.get_world()
            # 验证连接有效：获取地图名称
            _ = world.get_map().name
            print(f"成功连接 CARLA 服务器 ({host}:{port})")
            return client, world
        except Exception as e:
            last_error = e
            if attempt < max_retries:
                wait = CARLA_RETRY_DELAY * attempt
                print(f"[RETRY {attempt}/{max_retries}] CARLA 连接失败: {e}")
                print(f"  等待 {wait:.0f} 秒后重试...")
                time.sleep(wait)
            else:
                raise ConnectionError(
                    f"CARLA 连接失败 ({host}:{port})，已重试 {max_retries} 次。\n"
                    f"最后错误: {last_error}\n"
                    f"请确保 CARLA 仿真器已启动: CarlaUE4.exe -RenderOffScreen -quality-level=Low"
                ) from last_error


def load_map_with_retry(client, host, port, map_name, max_retries=3):
    """带重试的地图加载（load_world 会导致服务器重启，需重新连接）"""
    for attempt in range(1, max_retries + 1):
        print(f"加载地图 {map_name}... (attempt {attempt}/{max_retries})")
        try:
            # load_world 会触发服务器重新加载地图，旧连接会断开
            client.load_world(map_name)
        except Exception as e:
            print(f"[WARN] 地图加载请求失败: {e}")
            # 即使抛异常，服务器可能仍然在重启中，继续等待

        # 等待服务器重启完成并重新连接
        print(f"  等待 CARLA 服务器重启...")
        time.sleep(10)  # 给服务器足够时间重启

        # 重新创建客户端连接
        new_client = None
        for wait_attempt in range(30):
            try:
                new_client = carla.Client(host, port)
                new_client.set_timeout(10.0)
                world = new_client.get_world()
                actual_name = world.get_map().name
                if map_name in actual_name:
                    print(f"地图加载完成: {actual_name}")
                    return new_client, world
                else:
                    print(f"[WARN] 加载的地图名不匹配: {actual_name} (期望 {map_name})")
                    break
            except Exception as e:
                if wait_attempt < 29:
                    time.sleep(2)
                else:
                    print(f"[WARN] 等待服务器超时: {e}")
                    break

        if attempt < max_retries:
            # 更新 client 引用，准备下一次重试
            try:
                client = new_client if new_client else carla.Client(host, port)
                client.set_timeout(10.0)
            except Exception:
                pass
            time.sleep(5)
        else:
            raise RuntimeError(f"地图 {map_name} 加载失败，已重试 {max_retries} 次")
    raise RuntimeError(f"地图 {map_name} 加载失败")


def init_carla_environment(host=DEFAULT_CARLA_HOST, port=DEFAULT_CARLA_PORT):
    """初始化 CARLA 环境（带重试和容错）"""
    # 1) 连接 CARLA（带重试）
    client, world = connect_carla_with_retry(host, port)

    # 2) 初始化 Traffic Manager
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(3.0)
    try:
        traffic_manager.set_random_device_seed(42)
    except AttributeError:
        pass

    # 3) 清理已有 actors
    clear_all_actors(world)

    # 4) 加载目标地图（如果当前地图不是目标地图）
    current_map = world.get_map().name
    if TARGET_MAP in current_map:
        print(f"当前地图已是 {current_map}，跳过地图加载")
    else:
        print(f"当前地图 {current_map}，需要切换到 {TARGET_MAP}")
        client, world = load_map_with_retry(client, host, port, TARGET_MAP)

    # 5) 配置交通灯
    for tl in world.get_actors().filter('traffic.traffic_light*'):
        try:
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)
        except Exception:
            pass

    # 6) 生成车辆
    bp_lib = world.get_blueprint_library()
    try:
        vehicle, spawn_points = safe_spawn_vehicle(world, bp_lib)
    except Exception as e:
        print(f"[ERROR] 车辆生成失败: {e}")
        clear_all_actors(world)
        raise RuntimeError(f"车辆生成失败: {e}") from e

    # 7) 配置物理参数
    try:
        physics_control = vehicle.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle.apply_physics_control(physics_control)
    except Exception as e:
        print(f"配置车辆物理参数警告: {e}")

    # 8) 初始化智能体
    try:
        agent = BehaviorAgent(vehicle, behavior=AGENT_BEHAVIOR)
    except Exception as e:
        print(f"[ERROR] 智能体初始化失败: {e}")
        vehicle.destroy()
        raise RuntimeError(f"智能体初始化失败: {e}") from e
    agent.follow_speed_limits(False)

    try:
        agent.set_max_speed(AGENT_MAX_SPEED / 3.6)
    except AttributeError:
        try:
            agent.set_target_speed(AGENT_MAX_SPEED / 3.6)
        except AttributeError:
            agent._max_speed = AGENT_MAX_SPEED / 3.6
            print(f"使用备用方式设置速度: {AGENT_MAX_SPEED} km/h")

    # 增强避障参数
    try:
        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.8
                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.02
                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.0
        if hasattr(agent, '_min_distance'):
            agent._min_distance = AGENT_SAFE_DISTANCE
        if hasattr(agent, '_max_brake'):
            agent._max_brake = 0.8
    except (AttributeError, KeyError, TypeError):
        pass

    # 9) 选择目标点
    destination = select_forward_destination(vehicle, spawn_points)
    agent.set_destination(destination)
    print(f"避障智能体初始化完成（最大速度: {AGENT_MAX_SPEED} km/h，"
          f"安全距离: {AGENT_SAFE_DISTANCE}m）")
    print(f"目标位置: ({destination.x:.1f}, {destination.y:.1f}, {destination.z:.1f})")

    # 10) 碰撞传感器
    collision_sensor = CollisionSensor(vehicle)

    # 11) 准备输出目录
    try:
        if os.path.exists(OUTPUT_DIR):
            backup_dir = OUTPUT_DIR + '_backup_' + time.strftime('%Y%m%d_%H%M%S')
            shutil.move(OUTPUT_DIR, backup_dir)
            print(f"[INFO] 旧数据已备份至: {backup_dir}")
        os.makedirs(OUTPUT_DIR, exist_ok=True)
    except PermissionError:
        raise PermissionError(f"无权限操作目录: {OUTPUT_DIR}")
    print(f"输出目录: {OUTPUT_DIR}")

    return (world, bp_lib, vehicle, spawn_points, world.get_spectator(),
            agent, traffic_manager, collision_sensor)


# ═════════════════════════════════════════════════════════════
#  传感器数据
# ═════════════════════════════════════════════════════════════

class SensorData:
    def __init__(self, data_type, timestamp, data):
        self.data_type = data_type
        self.timestamp = timestamp
        self.data = data


def create_rgb_camera(world, bp_lib, vehicle, data_queue):
    """创建 RGB 相机传感器"""
    rgb_bp = bp_lib.find('sensor.camera.rgb')
    rgb_bp.set_attribute("image_size_x", "640")
    rgb_bp.set_attribute("image_size_y", "480")
    rgb_bp.set_attribute("sensor_tick", "0.05")
    rgb_bp.set_attribute("exposure_mode", EXPOSURE_MODE)
    rgb_bp.set_attribute("exposure_compensation", EXPOSURE_COMPENSATION)
    rgb_bp.set_attribute("fstop", FSTOP)
    rgb_bp.set_attribute("iso", ISO)
    rgb_bp.set_attribute("gamma", GAMMA)

    for attr_name in ("bloom_intensity", "chromatic_aberration_intensity",
                      "lens_flare_intensity"):
        try:
            rgb_bp.set_attribute(attr_name, "0.0")
        except Exception:
            pass

    transform = carla.Transform(
        carla.Location(x=0.2, y=0, z=4.2),
        carla.Rotation(pitch=-20),
    )

    def image_callback(image):
        data_queue.put(SensorData('image', image.timestamp, image))

    camera = world.spawn_actor(rgb_bp, transform, attach_to=vehicle)
    camera.listen(image_callback)
    print("RGB相机初始化完成")
    return camera, transform


def create_imu_sensor(world, bp_lib, vehicle, data_queue, transform):
    """创建 IMU 传感器"""
    imu_bp = bp_lib.find("sensor.other.imu")
    imu_bp.set_attribute('sensor_tick', str(1 / 60))
    imu_bp.set_attribute('noise_accel_stddev_x', '0.1')
    imu_bp.set_attribute('noise_gyro_stddev_x', '0.001')
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    imu.listen(lambda data: data_queue.put(SensorData('imu', data.timestamp, data)))
    print("IMU传感器初始化完成")
    return imu


# ═════════════════════════════════════════════════════════════
#  时间戳对齐
# ═════════════════════════════════════════════════════════════

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
            if len(self.image_buffer) > self.max_buffer // 10:
                self.image_buffer.pop(0)

    def get_aligned_pairs(self):
        pairs = []
        if not self.image_buffer or not self.imu_buffer:
            return pairs
        for img in self.image_buffer:
            diffs = [abs(img.timestamp - imu.timestamp) for imu in self.imu_buffer]
            min_idx = int(np.argmin(diffs))
            if diffs[min_idx] <= self.time_threshold:
                pairs.append((img, self.imu_buffer[min_idx]))
                del self.imu_buffer[min_idx]
        self.image_buffer = []
        return pairs


# ═════════════════════════════════════════════════════════════
#  EKF 融合
# ═════════════════════════════════════════════════════════════

class EKF_VIO:
    def __init__(self, init_pose, init_vel, dt=0.05):
        self.x = np.array([
            init_pose[0], init_pose[1], init_pose[2],
            init_vel[0], init_vel[1], init_vel[2],
            init_pose[3], init_pose[4], init_pose[5],
        ], dtype=np.float64)
        self.dt = dt
        self.init_z = init_pose[2]
        self.init_pose = np.array(init_pose, dtype=np.float64)  # 保存初始位姿用于 VO 绝对位姿对齐

        # 初始协方差：位置不确定性 0.5m²，速度 1.0 (m/s)²，姿态 0.02 rad²
        self.P = np.diag([0.5, 0.5, 0.1, 1.0, 1.0, 0.5, 0.02, 0.02, 0.02])

        # 过程噪声 Q（连续时间，将乘以 dt 离散化）
        # 位置噪声小（IMU 双重积分），速度噪声中等，姿态噪声小
        self.Q_cont = np.diag([0.01, 0.01, 0.001,   # 位置
                                0.1, 0.1, 0.05,      # 速度
                                0.001, 0.001, 0.001])  # 姿态

        # 视觉观测噪声 R：信任 VO 位姿观测（位置 ~2m std，姿态 ~0.05rad std → 方差）
        self.R_base = np.diag([4.0, 4.0, 1.0,    # 位置观测噪声 (m²)
                                0.0025, 0.0025, 0.0025])  # 姿态观测噪声 (rad²)
        self.R = self.R_base.copy()

        self._nis_ema = 1.0
        self._gyro_bias = np.zeros(3)
        self._accel_bias = np.zeros(3)
        self._bias_samples = 0
        self._bias_max_samples = 200

        self.innovation_history = []
        self.uncertainty_history = []
        self.mahalanobis_history = []

        # 卡方检验阈值：6 自由度，95% 置信度 ≈ 12.6，适当放宽到 25
        self.chi2_threshold = 25.0
        self.innovation_accepted = 0
        self.innovation_rejected = 0

        # 调试日志开关
        self._debug_log = True
        self._log_counter = 0
        self._last_vo_pose = None  # 上一帧 VO 绝对位姿，用于异常检测

    def _gate_innovation(self, y, S):
        try:
            S_inv = np.linalg.inv(S)
            d = float(y.T @ S_inv @ y)
            return d < self.chi2_threshold, d
        except np.linalg.LinAlgError:
            return False, float('inf')

    def _adapt_R(self, y, S):
        try:
            S_inv = np.linalg.inv(S + np.eye(6) * 1e-8)
            nis = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return
        expected_nis = 6.0
        self._nis_ema = 0.9 * self._nis_ema + 0.1 * (nis / expected_nis)
        scale = np.clip(self._nis_ema, 0.2, 10.0)
        self.R = self.R_base * scale

    def _estimate_imu_bias(self, accel, gyro):
        accel_mag = np.linalg.norm(accel)
        gyro_mag = np.linalg.norm(gyro)
        is_static = (abs(accel_mag - 9.81) < 0.5) and (gyro_mag < 0.02)
        if is_static and self._bias_samples < self._bias_max_samples:
            alpha = 1.0 / (self._bias_samples + 1)
            self._gyro_bias = (1 - alpha) * self._gyro_bias + alpha * gyro
            self._accel_bias = ((1 - alpha) * self._accel_bias +
                                alpha * (accel - self._gravity_dir(accel_mag)))
            self._bias_samples += 1

    @staticmethod
    def _gravity_dir(mag):
        return np.array([0.0, 0.0, mag])

    def imu_prediction(self, imu_data):
        accel_raw = np.array([imu_data.accelerometer.x,
                              imu_data.accelerometer.y,
                              imu_data.accelerometer.z])
        gyro_raw = np.array([imu_data.gyroscope.x,
                             imu_data.gyroscope.y,
                             imu_data.gyroscope.z])

        self._estimate_imu_bias(accel_raw, gyro_raw)
        gyro = gyro_raw - self._gyro_bias
        accel = accel_raw - self._accel_bias

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        new_roll = (roll + gyro[0] * self.dt + np.pi) % (2 * np.pi) - np.pi
        new_pitch = (pitch + gyro[1] * self.dt + np.pi) % (2 * np.pi) - np.pi
        new_yaw = (yaw + gyro[2] * self.dt + np.pi) % (2 * np.pi) - np.pi

        R_body2world = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        accel_world = R_body2world @ accel

        velocity_decay = 0.98
        new_vx = velocity_decay * (self.x[3] + accel_world[0] * self.dt)
        new_vy = velocity_decay * (self.x[4] + accel_world[1] * self.dt)
        new_vz = 0.0

        new_x = self.x[0] + new_vx * self.dt
        new_y = self.x[1] + new_vy * self.dt
        new_z = self.x[2]

        self.x = np.array([new_x, new_y, new_z,
                           new_vx, new_vy, new_vz,
                           new_roll, new_pitch, new_yaw])

        # 状态转移矩阵 F = I + A*dt (连续时间线性化)
        # 简化：位置由速度驱动，速度有衰减，姿态由陀螺仪驱动
        F = np.eye(9)
        F[0, 3] = self.dt   # x += vx * dt
        F[1, 4] = self.dt   # y += vy * dt
        F[3, 3] = velocity_decay  # vx *= decay
        F[4, 4] = velocity_decay  # vy *= decay

        # 离散化过程噪声: Q_d = Q_cont * dt
        Q_d = self.Q_cont * self.dt
        self.P = F @ self.P @ F.T + Q_d
        self.P = 0.5 * (self.P + self.P.T)  # 强制对称

    def visual_update(self, visual_pose):
        z = np.array(visual_pose, dtype=np.float64)
        H = np.zeros((6, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1
        H[3, 6] = H[4, 7] = H[5, 8] = 1

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R + np.eye(6) * 1e-8

        accepted, mahal_dist = self._gate_innovation(y, S)
        self.mahalanobis_history.append(mahal_dist)
        if not accepted:
            self.innovation_rejected += 1
            self.innovation_history.append(np.linalg.norm(y))
            return

        self.innovation_accepted += 1
        self.innovation_history.append(np.linalg.norm(y))
        self.uncertainty_history.append(np.trace(self.P[:3, :3]))

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.eye(9, 6) * 0.1

        self.x += K @ y
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self._adapt_R(y, S)

        # Soft flat-ground pseudo-measurement
        z_flat = np.array([self.init_z, 0.0, 0.0])
        H_flat = np.zeros((3, 9))
        H_flat[0, 2] = H_flat[1, 6] = H_flat[2, 7] = 1.0
        R_flat = np.diag([0.01, 0.001, 0.001])

        y_flat = z_flat - H_flat @ self.x
        S_flat = H_flat @ self.P @ H_flat.T + R_flat + np.eye(3) * 1e-8
        try:
            K_flat = self.P @ H_flat.T @ np.linalg.inv(S_flat)
        except np.linalg.LinAlgError:
            K_flat = np.zeros((9, 3))
        self.x += K_flat @ y_flat
        I_KH_flat = np.eye(9) - K_flat @ H_flat
        self.P = I_KH_flat @ self.P @ I_KH_flat.T + K_flat @ R_flat @ K_flat.T
        self.P = 0.5 * (self.P + self.P.T)

    def get_current_pose(self):
        return self.x[:3].copy(), self.x[6:9].copy()

    def get_current_velocity(self):
        return self.x[3:6].copy()

    def get_position_uncertainty(self):
        return np.sqrt(np.diag(self.P[:3, :3]))

    def get_fusion_quality_metrics(self):
        total = self.innovation_accepted + self.innovation_rejected
        rejection_rate = (self.innovation_rejected / total
                          if total > 0 else 0.0)
        n_innov = min(len(self.innovation_history), 100)
        n_uncert = min(len(self.uncertainty_history), 100)
        n_mahal = min(len(self.mahalanobis_history), 100)
        return {
            'avg_innovation': float(np.mean(self.innovation_history[-n_innov:]))
            if n_innov > 0 else 0.0,
            'avg_uncertainty': float(np.mean(self.uncertainty_history[-n_uncert:]))
            if n_uncert > 0 else 0.0,
            'innovation_std': float(np.std(self.innovation_history[-n_innov:]))
            if n_innov > 1 else 0.0,
            'rejection_rate': rejection_rate,
            'avg_mahalanobis': float(np.mean(self.mahalanobis_history[-n_mahal:]))
            if n_mahal > 0 else 0.0,
        }


# ═════════════════════════════════════════════════════════════
#  图像后处理
# ═════════════════════════════════════════════════════════════

def save_image_simple(img_array, output_dir, idx,
                      target_width=160, target_height=120):
    """缩放并保存图像"""
    try:
        resized = cv2.resize(img_array, (target_width, target_height),
                             interpolation=cv2.INTER_LANCZOS4)
        img_path = os.path.join(output_dir, f"{idx:04d}.png")
        cv2.imwrite(img_path, resized)
        return True
    except Exception as e:
        print(f"保存图像{idx}失败: {e}")
        return False


# ═════════════════════════════════════════════════════════════
#  数据完整性校验
# ═════════════════════════════════════════════════════════════

def validate_output_data(output_dir, min_images=10):
    """校验输出目录中的数据完整性，返回 (valid, report)"""
    report_lines = []
    checks = []

    # 检查目录是否存在
    if not os.path.isdir(output_dir):
        return False, [f"[FAIL] 输出目录不存在: {output_dir}"]

    # 检查必需文件
    required_files = {
        'ground_truth.txt': 'Ground Truth 轨迹',
        'fusion_pose.txt': 'EKF 融合位姿',
        'visual_odometry.txt': '视觉里程计轨迹',
        'aligned_imu.txt': '对齐的 IMU 数据',
        'dataset_metadata.txt': '数据集元数据',
    }

    all_ok = True
    for fname, desc in required_files.items():
        fpath = os.path.join(output_dir, fname)
        exists = os.path.isfile(fpath)
        size = os.path.getsize(fpath) if exists else 0
        status = f"[OK]  {fname}" if exists and size > 0 else f"[FAIL] {fname}"
        checks.append(f"{status}  ({desc})")
        if not exists or size == 0:
            all_ok = False

    # 检查图像数量
    png_files = [f for f in os.listdir(output_dir) if f.endswith('.png')]
    png_count = len(png_files)
    img_ok = png_count >= min_images
    checks.append(f"{'[OK]' if img_ok else '[FAIL]'} 图像文件: {png_count} 张 (最少 {min_images} 张)")
    if not img_ok:
        all_ok = False

    # 检查数据文件行数
    for fname in ['ground_truth.txt', 'fusion_pose.txt']:
        fpath = os.path.join(output_dir, fname)
        if os.path.isfile(fpath):
            try:
                with open(fpath, 'r', encoding='utf-8') as f:
                    line_count = sum(1 for _ in f)
                checks.append(f"[OK]  {fname}: {line_count} 行")
                if line_count < min_images:
                    checks.append(f"  [WARN] 数据行数 ({line_count}) < 图像数 ({min_images})")
            except Exception as e:
                checks.append(f"[FAIL] {fname}: 读取失败 - {e}")
                all_ok = False
        else:
            checks.append(f"[FAIL] {fname}: 文件不存在")
            all_ok = False

    # 检查 ground_truth.txt 格式
    gt_path = os.path.join(output_dir, 'ground_truth.txt')
    if os.path.isfile(gt_path) and os.path.getsize(gt_path) > 0:
        try:
            with open(gt_path, 'r', encoding='utf-8') as f:
                first_line = f.readline().strip()
            parts = first_line.split(',')
            if len(parts) >= 7:
                checks.append(f"[OK]  ground_truth.txt 格式正确 ({len(parts)} 列)")
            else:
                checks.append(f"[FAIL] ground_truth.txt 格式异常: 期望 >=7 列, 实际 {len(parts)} 列")
                all_ok = False
        except Exception as e:
            checks.append(f"[FAIL] ground_truth.txt 读取失败: {e}")
            all_ok = False

    report_lines = ["=" * 60,
                    f"  数据完整性校验: {output_dir}",
                    "=" * 60]
    report_lines.extend(checks)
    report_lines.append("=" * 60)
    if all_ok:
        report_lines.append("[PASS] 数据完整性校验通过")
    else:
        report_lines.append("[FAIL] 数据不完整，请检查上述问题")
    report_lines.append("=" * 60)

    return all_ok, report_lines


# ═════════════════════════════════════════════════════════════
#  主循环
# ═════════════════════════════════════════════════════════════

def main(headless=False, host=DEFAULT_CARLA_HOST, port=DEFAULT_CARLA_PORT):
    """主入口：数据采集 + 完整性校验"""
    # 初始化环境
    try:
        (world, bp_lib, vehicle, spawn_points, spectator,
         agent, traffic_manager, collision_sensor) = init_carla_environment(host, port)
    except Exception as e:
        print(f"\n[FATAL] 初始化失败: {e}")
        sys.exit(1)

    # 传感器队列
    sensor_queue = queue.Queue()
    camera, cam_transform = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
    imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)

    # 时间对齐器
    aligner = TimeAligner(time_threshold=0.02)

    # 获取车辆初始位姿
    vehicle_transform = vehicle.get_transform()
    init_location = vehicle_transform.location
    init_rotation = vehicle_transform.rotation
    init_pose = [init_location.x, init_location.y, init_location.z,
                 math.radians(init_rotation.roll),
                 math.radians(init_rotation.pitch),
                 math.radians(init_rotation.yaw)]
    init_vel = [0.0, 0.0, 0.0]

    # EKF 融合器
    ekf = EKF_VIO(init_pose, init_vel, dt=0.05)

    # 视觉里程计
    vo = VisualOdometry()
    scale_estimator = ScaleEstimator()

    # ---- VO 绝对位姿累积器（关键修复） ----
    # VO process_frame() 返回的是帧间相对运动 [dx,dy,dz,roll,pitch,yaw]
    # EKF visual_update() 需要绝对位姿观测
    # 因此需要累积 VO 相对运动，构建 VO 绝对位姿
    vo_abs_pose = list(init_pose)  # 初始化为车辆初始位姿
    vo_prev_relative = None  # 上一帧 VO 相对运动，用于尺度估计

    # 打开输出文件
    gt_log = open(os.path.join(OUTPUT_DIR, 'ground_truth.txt'), 'w', encoding='utf-8')
    fusion_log = open(os.path.join(OUTPUT_DIR, 'fusion_pose.txt'), 'w', encoding='utf-8')
    vo_log = open(os.path.join(OUTPUT_DIR, 'visual_odometry.txt'), 'w', encoding='utf-8')
    aligned_imu_f = open(os.path.join(OUTPUT_DIR, 'aligned_imu.txt'), 'w', encoding='utf-8')

    # 写 CSV 头
    gt_log.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw\n")
    fusion_log.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,"
                     "imu_pos_x,imu_pos_y,imu_pos_z,"
                     "vx,vy,vz,"
                     "uncert_x,uncert_y,uncert_z\n")
    vo_log.write("timestamp,vo_x,vo_y,vo_z,roll,pitch,yaw\n")
    aligned_imu_f.write("timestamp,accel_x,accel_y,accel_z,"
                        "gyro_x,gyro_y,gyro_z\n")

    # 设置同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    img_idx = 0
    stagnant_count = 0

    print(f"\n{'=' * 60}")
    print(f"  开始数据采集...")
    print(f"{'=' * 60}\n")

    try:
        while img_idx < MAX_SAVE_IMG:
            world.tick()

            # 收集传感器数据
            while not sensor_queue.empty():
                data = sensor_queue.get_nowait()
                aligner.add_data(data)

            # 获取对齐的图像-IMU 对
            pairs = aligner.get_aligned_pairs()
            for img_data, imu_data in pairs:
                # 读取图像
                img_array = np.frombuffer(img_data.data.raw_data, dtype=np.uint8)
                img_array = img_array.reshape((img_data.data.height, img_data.data.width, 4))
                img = img_array[:, :, :3].copy()

                # 视觉里程计
                vo_pose, num_matches = vo.process_frame(img)
                if vo_pose is None:
                    continue

                # ---- 时间戳对齐校验 ----
                timestamp_diff = abs(img_data.data.timestamp - imu_data.data.timestamp)
                if timestamp_diff > 0.05:
                    print(f"[EKF DEBUG] timestamp diff too large: {timestamp_diff:.4f}s, skip update")
                    # 仍执行 IMU 预测，但跳过 VO 更新
                    ekf.imu_prediction(imu_data.data)
                    # 捕获纯 IMU 位置（预测后、更新前）
                    imu_pos, _ = ekf.get_current_pose()
                    fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()
                    fusion_vel = ekf.get_current_velocity()
                    pos_uncertainty = ekf.get_position_uncertainty()
                    ekf._log_counter += 1
                    if ekf._debug_log and ekf._log_counter % 50 == 0:
                        print(f"[EKF DEBUG] Frame {img_idx}: timestamp diff={timestamp_diff:.4f}s > 0.05s, VO update SKIPPED. "
                              f"Accum: accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}")
                    # 仍写入记录
                    gt_loc = vehicle.get_location()
                    gt_rot = vehicle.get_transform().rotation
                    gt_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                                 f"{math.radians(gt_rot.roll):.6f},"
                                 f"{math.radians(gt_rot.pitch):.6f},"
                                 f"{math.radians(gt_rot.yaw):.6f}\n")
                    vo_log.write(f"{img_data.data.timestamp:.6f},0,0,0,0,0,0\n")
                    img_idx += 1
                    save_image_simple(img, OUTPUT_DIR, img_idx)
                    aligned_imu_f.write(
                        f"{imu_data.data.timestamp:.6f},"
                        f"{imu_data.data.accelerometer.x:.6f},"
                        f"{imu_data.data.accelerometer.y:.6f},"
                        f"{imu_data.data.accelerometer.z:.6f},"
                        f"{imu_data.data.gyroscope.x:.6f},"
                        f"{imu_data.data.gyroscope.y:.6f},"
                        f"{imu_data.data.gyroscope.z:.6f}\n")
                    fusion_log.write(
                        f"{img_data.data.timestamp:.6f},"
                        f"{fusion_pos[0]:.6f},{fusion_pos[1]:.6f},{fusion_pos[2]:.6f},"
                        f"{math.degrees(fusion_att[0]):.6f},"
                        f"{math.degrees(fusion_att[1]):.6f},"
                        f"{math.degrees(fusion_att[2]):.6f},"
                        f"{imu_pos[0]:.6f},{imu_pos[1]:.6f},{imu_pos[2]:.6f},"
                        f"{fusion_vel[0]:.6f},{fusion_vel[1]:.6f},{fusion_vel[2]:.6f},"
                        f"{pos_uncertainty[0]:.6f},{pos_uncertainty[1]:.6f},{pos_uncertainty[2]:.6f}\n")
                    if img_idx % 10 == 0:
                        fusion_log.flush()
                        gt_log.flush()
                        vo_log.flush()
                        aligned_imu_f.flush()
                    continue

                # ---- VO 异常值过滤：零运动检测 ----
                vo_motion_norm = np.linalg.norm(vo_pose[:3])
                vo_rot_norm = np.linalg.norm(vo_pose[3:6])
                if vo_motion_norm < 1e-6 and vo_rot_norm < 1e-6:
                    # VO 返回零运动（特征不足/匹配失败），跳过 VO 更新
                    ekf.imu_prediction(imu_data.data)
                    imu_pos, _ = ekf.get_current_pose()
                    fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()
                    fusion_vel = ekf.get_current_velocity()
                    pos_uncertainty = ekf.get_position_uncertainty()
                    ekf._log_counter += 1
                    if ekf._debug_log and ekf._log_counter % 50 == 0:
                        print(f"[EKF DEBUG] Frame {img_idx}: VO zero motion (matches={num_matches}), VO update SKIPPED. "
                              f"Accum: accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}")
                    # 同步写入 GT / VO 零运动 / 纯 IMU 状态
                    gt_loc = vehicle.get_location()
                    gt_rot = vehicle.get_transform().rotation
                    gt_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                                 f"{math.radians(gt_rot.roll):.6f},"
                                 f"{math.radians(gt_rot.pitch):.6f},"
                                 f"{math.radians(gt_rot.yaw):.6f}\n")
                    vo_log.write(f"{img_data.data.timestamp:.6f},0,0,0,0,0,0\n")
                    img_idx += 1
                    save_image_simple(img, OUTPUT_DIR, img_idx)
                    aligned_imu_f.write(
                        f"{imu_data.data.timestamp:.6f},"
                        f"{imu_data.data.accelerometer.x:.6f},"
                        f"{imu_data.data.accelerometer.y:.6f},"
                        f"{imu_data.data.accelerometer.z:.6f},"
                        f"{imu_data.data.gyroscope.x:.6f},"
                        f"{imu_data.data.gyroscope.y:.6f},"
                        f"{imu_data.data.gyroscope.z:.6f}\n")
                    fusion_log.write(
                        f"{img_data.data.timestamp:.6f},"
                        f"{fusion_pos[0]:.6f},{fusion_pos[1]:.6f},{fusion_pos[2]:.6f},"
                        f"{math.degrees(fusion_att[0]):.6f},"
                        f"{math.degrees(fusion_att[1]):.6f},"
                        f"{math.degrees(fusion_att[2]):.6f},"
                        f"{imu_pos[0]:.6f},{imu_pos[1]:.6f},{imu_pos[2]:.6f},"
                        f"{fusion_vel[0]:.6f},{fusion_vel[1]:.6f},{fusion_vel[2]:.6f},"
                        f"{pos_uncertainty[0]:.6f},{pos_uncertainty[1]:.6f},{pos_uncertainty[2]:.6f}\n")
                    if img_idx % 10 == 0:
                        fusion_log.flush()
                        gt_log.flush()
                        vo_log.flush()
                        aligned_imu_f.flush()
                    continue

                # ---- 尺度估计（使用固定尺度，由外部 ScaleEstimator 管理） ----
                # 注意：单目 VO 尺度不可观，使用恒定尺度因子
                scale = scale_estimator.get_current_scale()

                # ---- 累积 VO 相对运动 → 绝对位姿（关键修复） ----
                # VO process_frame() 返回的是帧间相对运动 [dx, dy, dz, roll, pitch, yaw]
                # 需要按尺度缩放后累积到绝对位姿 vo_abs_pose
                vo_relative = vo_pose.copy()
                vo_relative[:3] = [vo_pose[0] * scale,
                                   vo_pose[1] * scale,
                                   vo_pose[2] * scale]

                # 累积：绝对位置 += 相对位移（世界坐标系下近似）
                vo_abs_pose[0] += vo_relative[0]
                vo_abs_pose[1] += vo_relative[1]
                vo_abs_pose[2] += vo_relative[2]
                # 姿态：相对旋转叠加（用欧拉角近似）
                vo_abs_pose[3] = (vo_abs_pose[3] + vo_relative[3] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[4] = (vo_abs_pose[4] + vo_relative[4] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[5] = (vo_abs_pose[5] + vo_relative[5] + np.pi) % (2 * np.pi) - np.pi

                # 构建当前 VO 绝对位姿观测（深拷贝）
                vo_abs_pose_current = list(vo_abs_pose)

                # ---- VO 位姿异常值检测 ----
                vo_jump_skip = False
                if ekf._last_vo_pose is not None:
                    vo_abs_delta = np.linalg.norm(
                        np.array(vo_abs_pose_current[:3]) - np.array(ekf._last_vo_pose[:3]))
                    if vo_abs_delta > 10.0:  # 位置跳变超过 10m 视为异常
                        vo_jump_skip = True
                        print(f"[EKF DEBUG] Frame {img_idx}: VO position jump detected: "
                              f"delta={vo_abs_delta:.2f}m > 10m, skip update")
                ekf._last_vo_pose = vo_abs_pose_current.copy()

                # ---- EKF 预测 + 更新 ----
                ekf.imu_prediction(imu_data.data)

                # 捕获纯 IMU 位置（预测后，VO 更新前）
                imu_pos, _ = ekf.get_current_pose()

                # 执行 VO 视觉更新（如果无异常跳变）
                if not vo_jump_skip:
                    ekf.visual_update(vo_abs_pose_current)
                    fusion_pos, fusion_att = ekf.get_current_pose()
                else:
                    fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()

                fusion_vel = ekf.get_current_velocity()
                pos_uncertainty = ekf.get_position_uncertainty()

                # ---- 调试日志 ----
                ekf._log_counter += 1
                if ekf._debug_log and ekf._log_counter % 50 == 0:
                    kalman_gain_trace = np.trace(ekf.P[:3, :3])  # 位置协方差迹作为 Kalman 增益的代理指标
                    innovation_norm = ekf.innovation_history[-1] if ekf.innovation_history else 0.0
                    print(f"[EKF DEBUG] Frame {img_idx}: "
                          f"VO update={'EXECUTED' if not vo_jump_skip else 'SKIPPED(jump)'}, "
                          f"accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}, "
                          f"innov_norm={innovation_norm:.4f}, "
                          f"P_trace_pos={kalman_gain_trace:.4f}, "
                          f"matches={num_matches}, scale={scale:.4f}")

                # ---- 写入 Ground Truth ----
                gt_loc = vehicle.get_location()
                gt_rot = vehicle.get_transform().rotation
                gt_log.write(f"{img_data.data.timestamp:.6f},"
                             f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                             f"{math.radians(gt_rot.roll):.6f},"
                             f"{math.radians(gt_rot.pitch):.6f},"
                             f"{math.radians(gt_rot.yaw):.6f}\n")

                # ---- 写入 VO（绝对位姿） ----
                vo_log.write(f"{img_data.data.timestamp:.6f},"
                             f"{vo_abs_pose_current[0]:.6f},{vo_abs_pose_current[1]:.6f},{vo_abs_pose_current[2]:.6f},"
                             f"{vo_abs_pose_current[3]:.6f},{vo_abs_pose_current[4]:.6f},{vo_abs_pose_current[5]:.6f}\n")

                # 保存图像
                img_idx += 1
                save_image_simple(img, OUTPUT_DIR, img_idx)

                # 保存对齐的 IMU 数据
                aligned_imu_f.write(
                    f"{imu_data.data.timestamp:.6f},"
                    f"{imu_data.data.accelerometer.x:.6f},"
                    f"{imu_data.data.accelerometer.y:.6f},"
                    f"{imu_data.data.accelerometer.z:.6f},"
                    f"{imu_data.data.gyroscope.x:.6f},"
                    f"{imu_data.data.gyroscope.y:.6f},"
                    f"{imu_data.data.gyroscope.z:.6f}\n")

                # 保存融合结果
                fusion_log.write(
                    f"{img_data.data.timestamp:.6f},"
                    f"{fusion_pos[0]:.6f},{fusion_pos[1]:.6f},{fusion_pos[2]:.6f},"
                    f"{math.degrees(fusion_att[0]):.6f},"
                    f"{math.degrees(fusion_att[1]):.6f},"
                    f"{math.degrees(fusion_att[2]):.6f},"
                    f"{imu_pos[0]:.6f},{imu_pos[1]:.6f},{imu_pos[2]:.6f},"
                    f"{fusion_vel[0]:.6f},{fusion_vel[1]:.6f},{fusion_vel[2]:.6f},"
                    f"{pos_uncertainty[0]:.6f},{pos_uncertainty[1]:.6f},{pos_uncertainty[2]:.6f}\n")

                # 每 10 帧 flush
                if img_idx % 10 == 0:
                    fusion_log.flush()
                    gt_log.flush()
                    vo_log.flush()
                    aligned_imu_f.flush()

                # 每 100 帧打印质量
                if img_idx % 100 == 0 and img_idx > 0:
                    metrics = ekf.get_fusion_quality_metrics()
                    print(f"[Fusion Q] Frame {img_idx}: "
                          f"innov={metrics['avg_innovation']:.4f}, "
                          f"uncert={metrics['avg_uncertainty']:.4f}, "
                          f"matches={num_matches}, scale={scale:.4f}")

                if img_idx >= MAX_SAVE_IMG:
                    print(f"达到最大保存数量 ({MAX_SAVE_IMG})，退出采集")
                    break

                if not headless:
                    cv2.imshow('RGB Camera', img)

            # 智能体控制
            if agent.done():
                destination = select_forward_destination(vehicle, spawn_points)
                agent.set_destination(destination)
                print(f"[OK] 到达目标, 新目标: ({destination.x:.1f}, {destination.y:.1f})")

            try:
                control = agent.run_step()
                control.manual_gear_shift = False
                vehicle.apply_control(control)
            except Exception as e:
                print(f"警告：控制命令执行失败 - {e}")
                control = carla.VehicleControl()
                control.brake = 1.0
                vehicle.apply_control(control)

            # 碰撞重置
            reset_needed = False
            reset_reason = ""

            if collision_sensor.has_major_collision():
                print(f"[WARN] 碰撞过多 ({collision_sensor.collision_count}), 重置车辆...")
                reset_needed = True
                reset_reason = "碰撞过多"

            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            if speed < 0.1:
                stagnant_count += 1
                if stagnant_count > 150:
                    print(f"[STUCK] 车辆停滞 ({stagnant_count} 帧), 重置...")
                    reset_needed = True
                    reset_reason = "停滞"
            else:
                stagnant_count = 0

            if reset_needed:
                print(f"[RESET] 原因: {reset_reason}")
                try:
                    camera.stop()
                    imu.stop()
                    collision_sensor.sensor.stop()
                    camera.destroy()
                    imu.destroy()
                    collision_sensor.sensor.destroy()
                    vehicle.destroy()
                    time.sleep(0.5)

                    vehicle, _ = safe_spawn_vehicle(world, bp_lib)
                    physics_control = vehicle.get_physics_control()
                    physics_control.use_sweep_wheel_collision = True
                    vehicle.apply_physics_control(physics_control)

                    agent = BehaviorAgent(vehicle, behavior=AGENT_BEHAVIOR)
                    agent.follow_speed_limits(False)
                    try:
                        agent.set_max_speed(AGENT_MAX_SPEED / 3.6)
                    except AttributeError:
                        try:
                            agent.set_target_speed(AGENT_MAX_SPEED / 3.6)
                        except AttributeError:
                            agent._max_speed = AGENT_MAX_SPEED / 3.6

                    try:
                        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
                            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.8
                                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.02
                                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.0
                        if hasattr(agent, '_min_distance'):
                            agent._min_distance = AGENT_SAFE_DISTANCE
                        if hasattr(agent, '_max_brake'):
                            agent._max_brake = 0.8
                    except (AttributeError, KeyError, TypeError):
                        pass

                    destination = select_forward_destination(vehicle, spawn_points)
                    agent.set_destination(destination)
                    print(f"新目标: ({destination.x:.1f}, {destination.y:.1f})")

                    camera, cam_transform = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
                    imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
                    collision_sensor = CollisionSensor(vehicle)
                    stagnant_count = 0
                    print(f"[OK] 重置完成")
                except Exception as e:
                    print(f"[ERROR] 重置失败: {e}")

            # 视角
            spec_transform = carla.Transform(
                vehicle.get_transform().transform(carla.Location(x=-4, z=50)),
                carla.Rotation(yaw=-180, pitch=-90))
            spectator.set_transform(spec_transform)

            if not headless:
                if cv2.waitKey(1) == ord('q'):
                    print("用户退出")
                    break

    except Exception as e:
        print(f"主循环错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭文件
        gt_log.close()
        fusion_log.close()
        vo_log.close()
        aligned_imu_f.close()

        # 清理传感器
        try:
            camera.stop()
            imu.stop()
            collision_sensor.sensor.stop()
            camera.destroy()
            imu.destroy()
            collision_sensor.sensor.destroy()
        except Exception:
            pass

        # 清理世界
        try:
            clear_all_actors(world)
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(False)
        except Exception:
            pass

        if not headless:
            cv2.destroyAllWindows()

        print("资源清理完成")

        # 写入元数据
        try:
            meta_path = os.path.join(OUTPUT_DIR, 'dataset_metadata.txt')
            with open(meta_path, 'w', encoding='utf-8') as f:
                f.write(f"timestamp={time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"map={TARGET_MAP}\n")
                f.write(f"total_images={img_idx}\n")
                f.write(f"max_speed_kmh={AGENT_MAX_SPEED}\n")
                f.write(f"behavior={AGENT_BEHAVIOR}\n")
                f.write(f"imu_rate_hz={IMU_SAMPLE_RATE}\n")
                f.write(f"camera_rate_hz={CAMERA_SAMPLE_RATE}\n")
        except Exception as e:
            print(f"[WARN] 元数据写入失败: {e}")

        # 数据完整性校验
        print(f"\n{'=' * 60}")
        print(f"  数据完整性校验")
        print(f"{'=' * 60}")
        valid, report = validate_output_data(OUTPUT_DIR, min_images=10)
        for line in report:
            print(line)
        print()

        if not valid:
            print("[ERROR] 数据采集不完整！请检查上述失败项。")
            sys.exit(1)
        else:
            print(f"[OK] 数据采集成功！共 {img_idx} 帧，输出目录: {OUTPUT_DIR}")
            print(f"     绝对路径: {os.path.abspath(OUTPUT_DIR)}")


if __name__ == "__main__":
    import argparse as _argparse
    _parser = _argparse.ArgumentParser(
        description="IMU + Visual Odometry EKF Fusion — CARLA 数据采集")
    _parser.add_argument('--headless', action='store_true',
                         help='无头模式（不显示GUI窗口）')
    _parser.add_argument('--host', type=str, default=DEFAULT_CARLA_HOST,
                         help=f'CARLA 服务器地址 (默认: {DEFAULT_CARLA_HOST})')
    _parser.add_argument('--port', type=int, default=DEFAULT_CARLA_PORT,
                         help=f'CARLA 服务器端口 (默认: {DEFAULT_CARLA_PORT})')
    _args = _parser.parse_args()
    main(headless=_args.headless, host=_args.host, port=_args.port)
