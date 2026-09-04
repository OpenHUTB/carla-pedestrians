# -*- coding: utf-8 -*-
"""
IMU + Visual Odometry EKF Fusion — CARLA 数据采集
用法: python IMU_Vision_Fusion_EKF.py [--headless] [--host HOST] [--port PORT] [--map MAP]
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
from collections import deque

import numpy as np
import cv2
import carla
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import pandas as pd

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
#  配置参数（TARGET_MAP / OUTPUT_DIR 可通过 --map 命令行参数覆盖）
# ═════════════════════════════════════════════════════════════

DEFAULT_TARGET_MAP = "Town05"
TARGET_MAP = DEFAULT_TARGET_MAP
MAX_SAVE_IMG = 5000
OUTPUT_DIR = os.path.join(current_dir, '..', 'data', f'{TARGET_MAP}Data_IMU_Fusion')

# IMU-视觉融合参数
IMU_SAMPLE_RATE = 60       # Hz
CAMERA_SAMPLE_RATE = 20    # Hz (1/0.05)

EXPOSURE_MODE = "manual"
EXPOSURE_COMPENSATION = "0.0"
FSTOP = "4.0"
ISO = "250"
GAMMA = "2.2"
AGENT_BEHAVIOR = "cautious"
AGENT_MAX_SPEED = 12                # 【MOD:A1】20→12 km/h，Town02 密集城市需降速避免撞车
AGENT_SAFE_DISTANCE = 6.0           # 【MOD:E1】8.0→6.0 米，空旷道路避免误触发避险刹车
COLLISION_RESET_THRESHOLD = 3       # 碰撞次数阈值

# 消融开关：True = R 恒取基准值（关闭内点数/残差自适应缩放），
# 用于隔离验证噪声模型本身对融合性能的影响。
# 默认 False = 启用残差自适应 R（含滑动窗口平滑，见 RESID_SMOOTH_*）。
FIXED_R = False

# 残差滑动窗口平滑（自适应 R 用）：单帧 VO 突发跳变会使残差瞬时跳变 →
# 自适应 R 剧烈跳动 → 轨迹抖动。对最近 N 帧马氏距离做均值滤波后再映射 R。
# 卡方离群门仍用原始单帧残差（不受平滑影响），窗口 N 为可配置常量。
# ENABLE_RESIDUAL_SMOOTH: True=窗口平滑；False=窗口长度退化为1，等价原始
# 单帧残差行为，用于消融对比（全局唯一开关，禁止按地图分支定制）。
ENABLE_RESIDUAL_SMOOTH = True
RESID_SMOOTH_WINDOW = 10        # 速度/位置分支（2 自由度）窗口长度（帧）
RESID_SMOOTH_WINDOW_ATT = 10    # 姿态分支（3 自由度）窗口长度（帧）
# 残差‑R 映射灵敏度（仅 ENABLE_RESIDUAL_SMOOTH=True 时生效）：
# 窗口均值滤波会抬高输入残差，二次映射 (d/dof)² 被推离 R_min，K 持续偏小 →
# 可靠 VO 修正不足。乘以灵敏度后保证残差较小时 R 仍能下探到 R_min；
# R_min/R_max 全局固定不变，所有地图共用同一系数。
RESID_MAP_SENSITIVITY = 5.0
# [改动1] 残差→R 映射曲线形态（微调，R_min/R_max 不变，全局一套、无地图分支）：
#   resid = RESID_MAP_BASE + (eff/dof)**RESID_MAP_POWER,  eff = dist/SENS（平滑开时）
#   BASE：残差很小时把 R 抬离下限 → K<1，IMU 与观测合理分配，而非 K→1 直接抄 VO；
#   POWER：大残差时 R 的增长速率（保持稳健降权）。仅 ENABLE_RESIDUAL_SMOOTH=True 时生效；
#   平滑关闭时回退原始纯二次 (dist/dof)²（BASE=0/POWER=2/无灵敏度），保留消融基线。
RESID_MAP_BASE = 0.05
RESID_MAP_POWER = 1.5

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


def select_forward_destination(vehicle, spawn_points, min_distance=25.0):
    """选择车辆前方的目标点，优先直行路径，带多级容错降级

    解决 Town03/Town05 等弯道多、分叉路口多的地图找不到合法路点的问题：
    - Tier 1: 严格筛选（原始逻辑，min_distance=25, dot>0.4）
    - Tier 2: 放宽距离至 15m，dot>0.2
    - Tier 3: 大幅放宽，基本只要在前半球 (dot>-0.3)
    - Tier 4: 最终兜底，选最远生成点
    """
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_forward = vehicle_transform.get_forward_vector()

    # 多级降级策略
    tiers = [
        (25.0, 0.4,  "Tier1-strict"),    # 原逻辑
        (15.0, 0.2,  "Tier2-relaxed"),   # 放宽距离和方向
        (8.0,  -0.3, "Tier3-hemisphere"), # 大幅放宽，前半球即可
    ]

    for dist_thresh, dot_thresh, tier_name in tiers:
        forward_points = []
        for sp in spawn_points:
            to_spawn = sp.location - vehicle_location
            distance = to_spawn.length()
            if distance > dist_thresh:
                direction = to_spawn / (distance + 1e-8)
                dot_product = (vehicle_forward.x * direction.x +
                               vehicle_forward.y * direction.y)
                if dot_product > dot_thresh:
                    forward_points.append((sp.location, distance, dot_product))

        if forward_points:
            forward_points.sort(key=lambda x: x[2], reverse=True)
            chosen = forward_points[0][0]
            if tier_name != "Tier1-strict":
                print(f"[NAV] select_forward_destination 降级: {tier_name} "
                      f"(dist>{dist_thresh}m, dot>{dot_thresh}), "
                      f"候选 {len(forward_points)} 个")
            return chosen

    # 最终兜底：选最远生成点
    farthest = max(spawn_points,
                   key=lambda sp: (sp.location - vehicle_location).length())
    print(f"[NAV] select_forward_destination 最终兜底(fallback): 使用最远生成点, "
          f"距离={(farthest.location - vehicle_location).length():.1f}m")
    return farthest.location


# ═════════════════════════════════════════════════════════════
#  地图自适应辅助函数
# ═════════════════════════════════════════════════════════════

def estimate_road_width(vehicle, world):
    """估算当前车辆所在道路的宽度（米）

    通过 CARLA waypoint API 获取当前车道宽度，用于自适应安全距离。
    """
    try:
        vehicle_loc = vehicle.get_location()
        waypoint = world.get_map().get_waypoint(vehicle_loc)
        if waypoint is not None:
            return waypoint.lane_width
    except Exception:
        pass
    return 4.0  # 默认 4m


def compute_adaptive_safe_distance(road_width):
    """根据道路宽度计算自适应安全距离

    窄路（<3.5m，如 Town10HD）: 2.5m  → 避免误判墙壁为障碍物
    中等（3.5-5m）:         4.0m
    较宽（5-7m）:           5.5m
    宽路（>7m）:            7.0m
    """
    if road_width < 3.5:
        return 2.5
    elif road_width < 5.0:
        return 4.0
    elif road_width < 7.0:
        return 5.5
    else:
        return 7.0


def estimate_path_curvature(agent):
    """估算路径前方曲率（弧度），用于自适应 PID 参数

    通过采样路点队列首/中/尾三点计算转弯角度。
    返回 0 表示直道，越大表示弯道越急。
    """
    try:
        if not hasattr(agent, '_local_planner'):
            return 0.0
        wp_queue = agent._local_planner.waypoints_queue
        if not wp_queue or len(wp_queue) < 3:
            return 0.0
        # 采样 3 个点：起点、中点、终点
        indices = [0, len(wp_queue) // 2, len(wp_queue) - 1]
        pts = []
        for i in indices:
            wp = wp_queue[i][0]
            pts.append(np.array([wp.transform.location.x,
                                 wp.transform.location.y]))
        v1 = pts[1] - pts[0]
        v2 = pts[2] - pts[1]
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_angle = np.dot(v1, v2) / (n1 * n2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return float(abs(angle))
    except Exception:
        return 0.0


def count_nearby_dynamic_actors(vehicle, world, radius=15.0):
    """统计车辆周围半径内的动态 actor（车辆/行人）数量

    用于判断当前是否真的存在动态障碍物，避免窄路误判墙壁触发避险刹车。
    """
    try:
        vehicle_loc = vehicle.get_location()
        actor_list = world.get_actors()
        count = 0
        for actor in actor_list:
            if actor.id == vehicle.id:
                continue
            type_id = actor.type_id
            # 只统计车辆和行人
            if not (type_id.startswith('vehicle.') or type_id.startswith('walker.')):
                continue
            try:
                dist = actor.get_location().distance(vehicle_loc)
                if dist < radius:
                    count += 1
            except Exception:
                pass
        return count
    except Exception:
        return 0


def validate_agent_path(agent, vehicle, spawn_points, world, max_retries=3):
    """验证导航路径有效性，无效则重试选择新目标

    解决 Town03/Town05 路径生成失败导致车辆不动的问题。
    返回 True 表示路径有效，False 表示重试耗尽。
    """
    for attempt in range(max_retries):
        try:
            if hasattr(agent, '_local_planner'):
                wp_queue = agent._local_planner.waypoints_queue
                if wp_queue is not None and len(wp_queue) > 0:
                    target_wp = wp_queue[-1][0]
                    twp_loc = target_wp.transform.location
                    veh_loc = vehicle.get_location()
                    print(f"[NAV] 路径有效: {len(wp_queue)} 个路点, "
                          f"目标=({twp_loc.x:.1f}, {twp_loc.y:.1f}), "
                          f"距离={veh_loc.distance(twp_loc):.1f}m")
                    return True
        except Exception as e:
            print(f"[NAV] 路径检查异常: {e}")

        # 路径无效，重试
        print(f"[NAV] 路径无效 (attempt {attempt+1}/{max_retries}), 重新选择目标...")
        destination = select_forward_destination(vehicle, spawn_points)
        agent.set_destination(destination)
        # set_destination 内部同步更新 _local_planner，无需额外 tick

    print(f"[NAV] 路径验证失败，已重试 {max_retries} 次，使用当前设置继续")
    # 最终兜底：设前方 50m 为目的地，确保车辆至少有一个目标
    try:
        vehicle_loc = vehicle.get_location()
        forward = vehicle.get_transform().get_forward_vector()
        fallback = carla.Location(
            vehicle_loc.x + forward.x * 50,
            vehicle_loc.y + forward.y * 50,
            vehicle_loc.z
        )
        agent.set_destination(fallback)
        print(f"[NAV] 兜底目标: 前方 50m ({fallback.x:.1f}, {fallback.y:.1f})")
    except Exception:
        pass
    return False


def apply_adaptive_pid(agent, curvature):
    """根据路径曲率自适应调整 PID 参数

    弯道：降低 K_P 防止转向过猛，增大 K_D 增加阻尼
    直道：使用默认参数
    """
    try:
        if not hasattr(agent, '_vehicle_controller') or agent._vehicle_controller is None:
            return
        if not hasattr(agent._vehicle_controller, '_args_lateral_dict'):
            return

        lat = agent._vehicle_controller._args_lateral_dict
        if curvature > 0.3:  # 急弯（约 17°+）
            lat['K_P'] = 0.2
            lat['K_I'] = 0.005
            lat['K_D'] = 0.15
        elif curvature > 0.15:  # 中等弯道
            lat['K_P'] = 0.25
            lat['K_I'] = 0.008
            lat['K_D'] = 0.12
        else:  # 直道/缓弯
            lat['K_P'] = 0.3
            lat['K_I'] = 0.01
            lat['K_D'] = 0.1
    except (AttributeError, KeyError, TypeError):
        pass


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
    # 【MOD:MAP】自适应安全距离：根据道路宽度动态调整，窄路降低避免误触发避险刹车
    road_width = estimate_road_width(vehicle, world)
    adaptive_safe_dist = compute_adaptive_safe_distance(road_width)
    print(f"[MAP] 道路宽度: {road_width:.1f}m, 自适应安全距离: {adaptive_safe_dist:.1f}m")

    try:
        # 【MOD:C1】降低横向 PID 增益：K_P 0.8→0.3, K_I 0.02→0.01, 新增 K_D=0.1
        # 理由：原 K_P=0.8 导致转向过猛，Town02 窄路频繁甩头撞墙
        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.3
                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.01
                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.1
            # 【MOD:C2】纵向 PID 增益：降低 I 项防积分饱和
            if hasattr(agent._vehicle_controller, '_args_longitudinal_dict'):
                agent._vehicle_controller._args_longitudinal_dict['K_P'] = 1.0
                agent._vehicle_controller._args_longitudinal_dict['K_I'] = 0.02   # 【MOD:E3】0.05→0.02
                agent._vehicle_controller._args_longitudinal_dict['K_D'] = 0.0
        if hasattr(agent, '_min_distance'):
            agent._min_distance = adaptive_safe_dist
        if hasattr(agent, '_max_brake'):
            agent._max_brake = 0.8
    except (AttributeError, KeyError, TypeError):
        pass

    # 9) 选择目标点 + 路径验证
    destination = select_forward_destination(vehicle, spawn_points)
    agent.set_destination(destination)
    # 【MOD:MAP】验证路径有效性，失败则重试
    validate_agent_path(agent, vehicle, spawn_points, world)
    print(f"避障智能体初始化完成（最大速度: {AGENT_MAX_SPEED} km/h，"
          f"安全距离: {adaptive_safe_dist:.1f}m）")
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
    # EKF 约定 IMU 系 == 车体系（imu_prediction 用陀螺直接积姿态、静止重力方向 [0,0,g]）。
    # 传入的 transform 是相机变换（pitch=-20）：若沿用，静止重力补偿存在 ~9.81*sin20°≈3.36m/s²
    # 恒值误差 → IMU 航位推算漂到几十 km → 位置卡方门恒拒 → 硬重锚抄 VO → 融合退化为 VO。
    # 故此处清零旋转、对齐车体系挂载（仅保留位置偏移）。
    imu_transform = carla.Transform(transform.location, carla.Rotation(0.0, 0.0, 0.0))
    imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
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
        self.init_pose = np.array(init_pose, dtype=np.float64)

        # 初始协方差
        self.P = np.diag([2.0, 2.0, 0.5, 2.0, 2.0, 1.0, 0.05, 0.05, 0.05])

        # 过程噪声 Q（连续时间，离散化 Q_d = Q_cont * dt）
        # 区分平移(位置/速度)与旋转(姿态)：平移噪声明显更大，
        # 保证 P 不过度收缩、卡尔曼增益 K 具备合理量级，视觉观测能修正状态
        self.Q_cont = np.diag([0.18, 0.18, 0.09,   # 平移-位置 m²/s（0.10→0.18: 适度降低对IMU位置预测置信度；过大易致轨迹抖动）
                                0.72, 0.72, 0.36,   # 平移-速度 (m/s)²/s（0.40→0.72: 微调降IMU速度置信度，配合低R下限让VO速度观测主导修正）
                                0.0072, 0.0072, 0.018])  # 旋转-姿态 rad²/s（0.004/0.010→0.0072/0.018: 微调，VO姿态观测参与修正且不引入抖动）

        # 观测噪声 R：区分速度/姿态/位置三个独立矩阵
        # R 不宜过大，否则 S≈R 主导，K=P H'(H P H'+R)^-1→0，观测无法修正状态
        # R 基准值（Town01 低质量VO场景调优）；visual_update 上再乘质量自适应因子 qf：
        # 高质量VO地图(Town02)自动缩小R→增益增大充分信任VO；低质量地图放大R→降权防抖
        self.R_vel = np.diag([0.20, 0.20, 0.10])     # 速度观测 (m/s)² 基准
        self.R_att = np.diag([0.05, 0.50, 2.00])     # 姿态观测 rad² 基准（实测VO累积漂移 roll~0.1/pitch~0.4/yaw~1.0rad，需覆盖）
        self.R_pos = np.diag([0.06, 0.06, 0.03])     # 位置观测 m² 基准
        self._vo_match_ref = 60.0   # VO内点数参考: inliers≥ref→qf=1满信任; inliers=ref/3→qf=3降权
        # 残差自适应R上下限（乘性因子，等效R的max/min）：下限再降 0.01→0.008，
        # 小残差时R更充分收缩→增益增大，VO修正力度增强；
        # 上限保持10不变，抵御异常观测；限幅防R无界
        self.R_ADAPT_FLOOR = 0.008
        self.R_ADAPT_CEIL = 10.0
        self.fixed_r = FIXED_R      # 固定R消融：True = R 恒为基准值

        # 卡方门限：速度/姿态保持0.99严格（零帧创新大，防离群放行）；位置放宽到0.999
        # 减少误拒有效锚定观测（位置误拒会触发硬重锚定，扰动协方差）
        self.chi2_vel = 9.21    # χ²(2,0.99)
        self.chi2_att = 11.34   # χ²(3,0.99)
        self.chi2_pos = 13.82   # χ²(2,0.999)

        self._gyro_bias = np.zeros(3)
        self._accel_bias = np.zeros(3)
        self._bias_samples = 0
        self._bias_max_samples = 200

        self.innovation_history = []
        self.uncertainty_history = []

        self.innovation_accepted = 0
        self.innovation_rejected = 0

        # 残差滑动窗口（自适应 R 用）：仅缓存最近 N 帧马氏距离，
        # 均值滤波后作为 _r_scale 的输入，抑制单帧 VO 跳变导致的 R 剧烈跳动。
        # 卡方门仍用原始单帧残差，离群剔除逻辑不变。
        # ENABLE_RESIDUAL_SMOOTH=False 时窗口长度退化为 1（等价原始单帧残差）
        _win = RESID_SMOOTH_WINDOW if ENABLE_RESIDUAL_SMOOTH else 1
        _win_att = RESID_SMOOTH_WINDOW_ATT if ENABLE_RESIDUAL_SMOOTH else 1
        self._resid_win_vel = deque(maxlen=_win)
        self._resid_win_att = deque(maxlen=_win_att)
        self._resid_win_pos = deque(maxlen=_win)

        self._debug_log = True
        self._log_counter = 0
        self._last_vo_pose = None
        self._last_vo_inliers = int(self._vo_match_ref)  # 当前帧VO内点数（主循环喂入，驱动质量自适应R）

        # 初始尺度 1.0：与 ScaleEstimator 固定尺度一致（VO 已是米制），EMA 在线自适应
        self._vo_scale_ema = 1.0
        self._vo_scale_initialized = True  # 立即启用，EMA 自适应收敛

        self._last_K = None
        self._last_residual = None
        self._prev_vo_obs = None

        self._pos_skip_count = 0
        self._update_call_count = 0
        self._raw_vel = np.zeros(3)      # 原始 IMU 速度（备用）
        self._vo_z0 = None               # 首帧 VO 位置基准（坐标系换算用）
        self._vo_aligned = False         # VO 首帧一次性对齐标志（替代每帧硬锚定）

        # 独立纯 IMU 航位推算状态（仅 IMU 积分，无 VO 修正，自带陀螺姿态）
        # 用于消融 Pure-IMU 基线：原来 imu_pos 取的是已被 VO 修正的 EKF 状态，
        # 导致 Pure-IMU 列与 Fusion 列几乎完全一致，掩盖融合实际增益。
        self._imu_dr_pos = np.array(init_pose[:3], dtype=np.float64)
        self._imu_dr_vel = np.array(init_vel, dtype=np.float64)
        self._imu_dr_att = np.array(init_pose[3:6], dtype=np.float64)
        self._imu_steps_since_update = 0   # 自上次 visual_update 起的 IMU 步数（求真实 VO 时间间隔）

    def _mahalanobis_gate(self, y, S, chi2_threshold):
        """卡方门控: 马氏距离超阈值返回 False，拒绝观测"""
        try:
            S_inv = np.linalg.inv(S)
            d = float(y.T @ S_inv @ y)
            return d < chi2_threshold, d
        except np.linalg.LinAlgError:
            return False, float('inf')

    def _vo_quality_gain(self, num_matches):
        """VO观测质量因子: 内点多→qf小(R缩小、增益增大、充分信任VO)；内点少→qf大(R放大、降权防抖)"""
        f = self._vo_match_ref / float(max(int(num_matches), 1))
        return float(np.clip(f, 1.0 / 3.0, 3.0))

    def _adaptive_r_factor(self, base_qf, dist, dof):
        """残差自适应R因子。[改动2] 仅微调映射曲线形态，R_min/R_max（clip 边界）不变。
        平滑开启: resid = RESID_MAP_BASE + (eff/dof)**RESID_MAP_POWER, eff = dist/SENS。
          BASE 使小残差时 R 抬离下限 → K<1，IMU/观测合理分配；POWER 控制大残差增长速率。
        平滑关闭: 回退原始纯二次 (dist/dof)²，等价原始版本（保留消融基线）。
        卡方离群门仍用原始单帧残差，此处不参与异常剔除。"""
        if ENABLE_RESIDUAL_SMOOTH:
            eff = max(float(dist), 1e-6) / float(RESID_MAP_SENSITIVITY)
            resid = RESID_MAP_BASE + (eff / float(dof)) ** RESID_MAP_POWER
        else:
            resid = (max(float(dist), 1e-6) / float(dof)) ** 2
        return float(np.clip(base_qf * resid, self.R_ADAPT_FLOOR, self.R_ADAPT_CEIL))

    def _r_scale(self, base_qf, dist, dof):
        """固定R消融：fixed_r 开启时返回 1.0，否则走残差自适应因子"""
        if self.fixed_r:
            return 1.0
        return self._adaptive_r_factor(base_qf, dist, dof)

    def _smooth_residual(self, window, dist):
        """残差滑动窗口均值滤波：缓存最近 N 帧马氏距离，返回均值。
        窗口未填满时用已有样本均值；dist 为原始单帧马氏距离。"""
        window.append(float(dist))
        return float(np.mean(np.asarray(window, dtype=np.float64)))

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

    def _regularize_P(self):
        """P 矩阵数值保护：确保对称、有限、正定（防奇异/非正定）"""
        self.P = 0.5 * (self.P + self.P.T)
        # 防 NaN/Inf：数值污染时回退到安全对角协方差
        if not np.all(np.isfinite(self.P)):
            self.P = np.diag([2.0, 2.0, 0.5, 2.0, 2.0, 1.0, 0.05, 0.05, 0.05])
            return
        # 最小特征值低于下限则整体平移，保证正定
        min_eig = float(np.min(np.linalg.eigvalsh(self.P)))
        if min_eig < 1e-6:
            self.P += np.eye(9) * (1e-6 - min_eig + 1e-9)

    def _clamp_covariance(self):
        """协方差限幅"""
        max_diag = np.array([100.0, 100.0, 25.0,
                              25.0, 25.0, 10.0,
                              0.5, 0.5, 0.5])
        for i in range(9):
            if self.P[i, i] > max_diag[i]:
                self.P[i, i] = max_diag[i]
        self.P = 0.5 * (self.P + self.P.T)

    def imu_prediction(self, imu_data):
        accel_raw = np.array([imu_data.accelerometer.x,
                              imu_data.accelerometer.y,
                              imu_data.accelerometer.z])
        gyro_raw = np.array([imu_data.gyroscope.x,
                             imu_data.gyroscope.y,
                             imu_data.gyroscope.z])

        accel_mag = np.linalg.norm(accel_raw)
        if accel_mag > 100.0:
            return

        self._estimate_imu_bias(accel_raw, gyro_raw)
        gyro = gyro_raw - self._gyro_bias
        accel = accel_raw - self._accel_bias

        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]
        new_roll = (roll + gyro[0] * self.dt + np.pi) % (2 * np.pi) - np.pi
        new_pitch = (pitch + gyro[1] * self.dt + np.pi) % (2 * np.pi) - np.pi
        new_yaw = (yaw + gyro[2] * self.dt + np.pi) % (2 * np.pi) - np.pi

        R_body2world = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        accel_world = R_body2world @ accel
        accel_world[2] -= 9.81  # 补偿重力：IMU 测量包含重力，世界系减去

        # 独立纯 IMU 航位推算（供消融 Pure-IMU 基线）：用自己的陀螺姿态积分，
        # 不接收任何 VO 修正，反映真实 IMU 漂移（≈16878m vs 融合 470m）
        R_dr = R.from_euler('xyz', self._imu_dr_att).as_matrix()
        aw_dr = R_dr @ accel
        aw_dr[2] -= 9.81
        self._imu_dr_vel = self._imu_dr_vel + aw_dr * self.dt
        self._imu_dr_pos = self._imu_dr_pos + self._imu_dr_vel * self.dt
        self._imu_dr_att = np.array([
            (self._imu_dr_att[0] + gyro[0] * self.dt + np.pi) % (2 * np.pi) - np.pi,
            (self._imu_dr_att[1] + gyro[1] * self.dt + np.pi) % (2 * np.pi) - np.pi,
            (self._imu_dr_att[2] + gyro[2] * self.dt + np.pi) % (2 * np.pi) - np.pi,
        ])

        vx_curr = self.x[3]
        vy_curr = self.x[4]

        new_vx = self.x[3] + accel_world[0] * self.dt
        new_vy = self.x[4] + accel_world[1] * self.dt
        new_vz = 0.0

        new_x = self.x[0] + vx_curr * self.dt
        new_y = self.x[1] + vy_curr * self.dt
        new_z = self.x[2]

        self.x = np.array([new_x, new_y, new_z,
                           new_vx, new_vy, new_vz,
                           new_roll, new_pitch, new_yaw])

        # 原始 IMU 速度积分（用于尺度估计，打破 EKF 反馈）
        self._raw_vel[0] += accel_world[0] * self.dt
        self._raw_vel[1] += accel_world[1] * self.dt

        F = np.eye(9)
        F[0, 3] = self.dt
        F[1, 4] = self.dt

        Q_d = self.Q_cont * self.dt
        self.P = F @ self.P @ F.T + Q_d
        self._regularize_P()
        self._clamp_covariance()
        self._imu_steps_since_update += 1

    def visual_update(self, visual_pose):
        self._update_call_count += 1
        # 调试：打印 update 实际执行次数，确认视觉观测真正进入 update
        print(f"[EKF UPDATE] visual_update count={self._update_call_count}, "
              f"ts_pose=({visual_pose[0]:.2f},{visual_pose[1]:.2f},{visual_pose[2]:.2f})")
        z = np.array(visual_pose, dtype=np.float64)

        # 基础质量因子(内点数)；残差因子按分支叠加并限幅到[R_ADAPT_FLOOR, R_ADAPT_CEIL]
        # 固定R消融：fixed_r 开启时质量因子恒 1，R 不随内点数缩放
        qf_base = 1.0 if self.fixed_r else self._vo_quality_gain(self._last_vo_inliers)

        # === 尺度（固定 1.0，关闭在线 EMA 反馈） ===
        # 根因：在线尺度 EMA 用 IMU 速度做参考形成反馈闭环，IMU 漂移时尺度发散
        # （实测 →4.0），将速度/位置观测量纲整体放大 → 卡方门恒拒 → 退化为纯 IMU。
        # VO 绝对位姿已由主循环按 ScaleEstimator 固定尺度累积为世界米制，此处不再缩放。
        vo_disp_pos = np.zeros(3)
        if self._prev_vo_obs is not None:
            vo_disp_pos = z[:3] - self._prev_vo_obs[:3]
        self._prev_vo_obs = z.copy()

        # === 速度观测（独立卡方门控，仅观测水平 xy；VO 单目 z 漂移不可观测，剔除） ===
        # 真实 VO 时间间隔 = 自上次更新以来的 IMU 步数 × dt（VO 稀疏时固定 dt
        # 会高估速度 → 速度卡方门恒拒）；首帧退化为 dt
        eff_dt = max(self._imu_steps_since_update, 1) * self.dt
        self._imu_steps_since_update = 0
        vo_disp_scaled = vo_disp_pos * self._vo_scale_ema
        v_obs = vo_disp_scaled / eff_dt

        H_vel = np.zeros((2, 9))
        H_vel[0, 3] = H_vel[1, 4] = 1

        y_vel = v_obs[:2] - H_vel @ self.x
        R_vel_nom = self.R_vel[:2, :2] * qf_base
        S = H_vel @ self.P @ H_vel.T + R_vel_nom
        S = 0.5 * (S + S.T) + np.eye(2) * 1e-8

        innov_norm = float(np.linalg.norm(y_vel))
        self.innovation_history.append(innov_norm)

        # 残差卡方门限（名义S）：仅过滤严重离群观测
        accept_vel, dist_vel = self._mahalanobis_gate(y_vel, S, self.chi2_vel)
        if not accept_vel:
            self.innovation_rejected += 1
        else:
            # 残差自适应R: 残差小→R缩小→增益增大；残差大→R放大→降权
            # 用滑动窗口平滑后的残差（均值滤波）映射 R，抑制单帧跳变导致的 R 抖动；
            # 卡方门仍用原始单帧残差 dist_vel
            dist_vel_s = self._smooth_residual(self._resid_win_vel, dist_vel)
            R_use = self.R_vel[:2, :2] * self._r_scale(qf_base, dist_vel_s, 2)
            S = H_vel @ self.P @ H_vel.T + R_use
            S = 0.5 * (S + S.T) + np.eye(2) * 1e-8
            try:
                K = self.P @ H_vel.T @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                K = np.eye(9, 2) * 0.05
            self._last_K = K.copy()
            self._last_residual = y_vel.copy()

            self.x += K @ y_vel
            self.innovation_accepted += 1
            I_KH = np.eye(9) - K @ H_vel
            self.P = I_KH @ self.P @ I_KH.T + K @ R_use @ K.T
            self._regularize_P()

            self.uncertainty_history.append(np.trace(self.P[:3, :3]))

        # === 姿态观测（独立卡方门控，与速度解耦，避免VO姿态漂移拖累速度修正） ===
        H_att = np.zeros((3, 9))
        H_att[0, 6] = H_att[1, 7] = H_att[2, 8] = 1

        y_att = z[3:6] - H_att @ self.x
        y_att = (y_att + np.pi) % (2 * np.pi) - np.pi
        R_att_nom = self.R_att * qf_base
        S_att = H_att @ self.P @ H_att.T + R_att_nom
        S_att = 0.5 * (S_att + S_att.T) + np.eye(3) * 1e-8

        accept_att, dist_att = self._mahalanobis_gate(y_att, S_att, self.chi2_att)
        if accept_att:
            # 残差自适应R: 残差小→R缩小→增益增大；残差大→R放大→降权
            # 用滑动窗口平滑后的残差映射 R；卡方门仍用原始单帧残差 dist_att
            dist_att_s = self._smooth_residual(self._resid_win_att, dist_att)
            R_att_use = self.R_att * self._r_scale(qf_base, dist_att_s, 3)
            S_att = S_att - R_att_nom + R_att_use
            try:
                K_att = self.P @ H_att.T @ np.linalg.inv(S_att)
            except np.linalg.LinAlgError:
                K_att = np.zeros((9, 3))
            self.x += K_att @ y_att
            I_KH_att = np.eye(9) - K_att @ H_att
            self.P = (I_KH_att @ self.P @ I_KH_att.T
                      + K_att @ R_att_use @ K_att.T)
            self._regularize_P()

        # === 位置观测（卡方门控） ===
        # 坐标系换算：状态在 CARLA 初始位姿坐标系，VO 观测在首帧原点坐标系，
        # 残差恒等于两坐标系原点差(≈259m) → 门恒拒、位置分支永不执行（退化为纯IMU）。
        # 用首帧 VO 位置 z0 做基准偏移：z_state = init + (z - z0)，兼容新旧数据。
        if self._vo_z0 is None:
            self._vo_z0 = z[:3].copy()
        z_pos = self.init_pose[:3] + (z[:3] - self._vo_z0) * self._vo_scale_ema
        H_pos = np.zeros((2, 9))
        H_pos[0, 0] = H_pos[1, 1] = 1.0

        y_pos = z_pos[:2] - H_pos @ self.x
        R_pos_nom = self.R_pos[:2, :2] * qf_base
        S_pos = H_pos @ self.P @ H_pos.T + R_pos_nom
        S_pos = 0.5 * (S_pos + S_pos.T) + np.eye(2) * 1e-8

        accept_pos, dist_pos = self._mahalanobis_gate(y_pos, S_pos, self.chi2_pos)
        if accept_pos:
            # 残差自适应R: 残差小→R缩小→增益增大；残差大→R放大→降权
            # 用滑动窗口平滑后的残差映射 R；卡方门仍用原始单帧残差 dist_pos
            dist_pos_s = self._smooth_residual(self._resid_win_pos, dist_pos)
            R_pos_use = self.R_pos[:2, :2] * self._r_scale(qf_base, dist_pos_s, 2)
            S_pos = S_pos - R_pos_nom + R_pos_use
            try:
                K_pos = self.P @ H_pos.T @ np.linalg.inv(S_pos)
            except np.linalg.LinAlgError:
                K_pos = np.zeros((9, 2))
            self.x += K_pos @ y_pos
            I_KH_pos = np.eye(9) - K_pos @ H_pos
            self.P = (I_KH_pos @ self.P @ I_KH_pos.T
                      + K_pos @ R_pos_use @ K_pos.T)
            self._regularize_P()
        else:
            self._pos_skip_count += 1
            # 残差超阈 = 异常观测：直接跳过该帧位置观测（不修正状态、
            # 不强制重锚），防止离群 VO 覆盖滤波器状态
            pass

        # === 平坦地面伪观测 ===
        z_flat = np.array([self.init_z, 0.0, 0.0])
        H_flat = np.zeros((3, 9))
        H_flat[0, 2] = H_flat[1, 6] = H_flat[2, 7] = 1.0
        R_flat = np.diag([0.1, 0.01, 0.01])

        y_flat = z_flat - H_flat @ self.x
        S_flat = H_flat @ self.P @ H_flat.T + R_flat + np.eye(3) * 1e-8
        try:
            K_flat = self.P @ H_flat.T @ np.linalg.inv(S_flat)
        except np.linalg.LinAlgError:
            K_flat = np.zeros((9, 3))
        self.x += K_flat @ y_flat
        I_KH_flat = np.eye(9) - K_flat @ H_flat
        self.P = (I_KH_flat @ self.P @ I_KH_flat.T
                  + K_flat @ R_flat @ K_flat.T)
        self._regularize_P()
        self._clamp_covariance()

    def get_current_pose(self):
        return self.x[:3].copy(), self.x[6:9].copy()

    def get_current_velocity(self):
        return self.x[3:6].copy()

    def get_imu_dead_reckoning_pose(self):
        # 独立纯 IMU 航位推算位姿（无 VO 修正）——消融 Pure-IMU 基线用
        return self._imu_dr_pos.copy(), self._imu_dr_att.copy()

    def get_position_uncertainty(self):
        return np.sqrt(np.diag(self.P[:3, :3]))

    def get_fusion_quality_metrics(self):
        total = self.innovation_accepted + self.innovation_rejected
        rejection_rate = (self.innovation_rejected / total
                          if total > 0 else 0.0)
        n_innov = min(len(self.innovation_history), 100)
        n_uncert = min(len(self.uncertainty_history), 100)
        return {
            'avg_innovation': float(np.mean(self.innovation_history[-n_innov:]))
            if n_innov > 0 else 0.0,
            'avg_uncertainty': float(np.mean(self.uncertainty_history[-n_uncert:]))
            if n_uncert > 0 else 0.0,
            'innovation_std': float(np.std(self.innovation_history[-n_innov:]))
            if n_innov > 1 else 0.0,
            'rejection_rate': rejection_rate,
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
#  轨迹可视化（论文用图）
# ═════════════════════════════════════════════════════════════

def _load_csv_columns(path, col_names):
    """Read CSV and return numpy array of specified columns. Returns None on failure."""
    if not os.path.isfile(path):
        return None
    try:
        df = pd.read_csv(path, encoding='utf-8')
        vals = df[col_names].to_numpy(dtype=float)
        if len(vals) == 0:
            return None
        return vals  # (N, len(col_names))
    except Exception:
        return None


def compute_ate(gt_xy, pred_xy):
    """Absolute Trajectory Error (RMSE) in meters."""
    n = min(len(gt_xy), len(pred_xy))
    diff = gt_xy[:n] - pred_xy[:n]
    return float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))


def compute_rpe(gt_xy, pred_xy):
    """Relative Pose Error per step (mean) in meters/frame."""
    n = min(len(gt_xy), len(pred_xy))
    if n < 2:
        return 0.0
    gt_delta = np.diff(gt_xy[:n], axis=0)
    pred_delta = np.diff(pred_xy[:n], axis=0)
    return float(np.mean(np.linalg.norm(gt_delta - pred_delta, axis=1)))


def compute_max_error(gt_xy, pred_xy):
    """Maximum pointwise Euclidean error in meters."""
    n = min(len(gt_xy), len(pred_xy))
    diff = gt_xy[:n] - pred_xy[:n]
    return float(np.max(np.linalg.norm(diff, axis=1)))


def compute_drift_rate(gt_xy, pred_xy):
    """Drift rate: ATE / total GT path length (%)."""
    n = min(len(gt_xy), len(pred_xy))
    gt_dist = np.sum(np.linalg.norm(np.diff(gt_xy[:n], axis=0), axis=1))
    ate = compute_ate(gt_xy, pred_xy)
    return float(ate / gt_dist * 100) if gt_dist > 0 else float('inf')


def plot_trajectory_comparison(output_dir, town_name=None):
    """Generate publication-quality trajectory comparison figure.

    Reads ground_truth.txt and fusion_pose.txt from output_dir,
    produces a two-panel figure saved as PNG and PDF.
    """
    gt_path = os.path.join(output_dir, 'ground_truth.txt')
    fusion_path = os.path.join(output_dir, 'fusion_pose.txt')

    gt_data = _load_csv_columns(gt_path, ['pos_x', 'pos_y'])
    fusion_data = _load_csv_columns(fusion_path, ['pos_x', 'pos_y'])

    if gt_data is None:
        print(f"[VIZ] 跳过: 缺少 ground_truth.txt 或数据为空")
        return
    if fusion_data is None:
        print(f"[VIZ] 跳过: 缺少 fusion_pose.txt 或数据为空")
        return

    # 归一化到起点
    gt_xy = gt_data - gt_data[0]
    fusion_xy = fusion_data - fusion_data[0]

    # 截断到较短长度
    n = min(len(gt_xy), len(fusion_xy))
    gt_xy = gt_xy[:n]
    fusion_xy = fusion_xy[:n]

    # 计算指标
    ate = compute_ate(gt_xy, fusion_xy)
    rpe = compute_rpe(gt_xy, fusion_xy)
    max_err = compute_max_error(gt_xy, fusion_xy)
    drift = compute_drift_rate(gt_xy, fusion_xy)

    # 逐帧位置误差
    frame_errors = np.linalg.norm(gt_xy - fusion_xy, axis=1)

    # ---- 论文风格设置 ----
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'DejaVu Serif'],
        'mathtext.fontset': 'stix',
        'font.size': 11,
        'axes.titlesize': 13,
        'axes.labelsize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.dpi': 150,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.05,
    })

    fig, (ax_traj, ax_err) = plt.subplots(1, 2, figsize=(14, 5.5))

    # ── 左图：轨迹对比 ──
    ax_traj.plot(gt_xy[:, 0], gt_xy[:, 1], color='#1f77b4', lw=1.8,
                 label='Ground Truth', zorder=3)
    ax_traj.plot(fusion_xy[:, 0], fusion_xy[:, 1], color='#d62728', lw=1.4,
                 ls='--', label='EKF Fusion', zorder=4)

    # 起点和终点标记
    ax_traj.scatter(gt_xy[0, 0], gt_xy[0, 1], marker='o', s=60,
                    c='#1f77b4', edgecolors='k', linewidths=0.5, zorder=5)
    ax_traj.scatter(gt_xy[-1, 0], gt_xy[-1, 1], marker='s', s=60,
                    c='#1f77b4', edgecolors='k', linewidths=0.5, zorder=5)
    ax_traj.scatter(fusion_xy[0, 0], fusion_xy[0, 1], marker='o', s=60,
                    c='#d62728', edgecolors='k', linewidths=0.5, zorder=5)
    ax_traj.scatter(fusion_xy[-1, 0], fusion_xy[-1, 1], marker='s', s=60,
                    c='#d62728', edgecolors='k', linewidths=0.5, zorder=5)

    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')
    title = f'Trajectory Comparison'
    if town_name:
        title += f' — {town_name}'
    ax_traj.set_title(title)
    ax_traj.legend(loc='best', framealpha=0.85)
    ax_traj.grid(True, alpha=0.3, linestyle='--')
    ax_traj.set_aspect('equal', adjustable='box')
    ax_traj.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax_traj.yaxis.set_minor_locator(AutoMinorLocator(2))

    # ── 右图：定位偏差随时间变化 ──
    ax_err.plot(np.arange(n), frame_errors, color='#9467bd', lw=1.0, alpha=0.85)
    ax_err.fill_between(np.arange(n), 0, frame_errors, color='#9467bd', alpha=0.12)
    ax_err.axhline(y=ate, color='#d62728', lw=1.2, ls='--',
                   label=f'ATE = {ate:.2f} m')
    ax_err.set_xlabel('Frame')
    ax_err.set_ylabel('Position Error (m)')
    ax_err.set_title('Localization Error Over Time')
    ax_err.legend(loc='upper right', framealpha=0.85)
    ax_err.grid(True, alpha=0.3, linestyle='--')
    ax_err.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax_err.yaxis.set_minor_locator(AutoMinorLocator(2))

    # 统计信息文本
    stats_text = (
        f'ATE: {ate:.3f} m\n'
        f'RPE: {rpe:.4f} m/frame\n'
        f'Max Error: {max_err:.3f} m\n'
        f'Drift: {drift:.2f}%'
    )
    ax_err.text(0.97, 0.97, stats_text, transform=ax_err.transAxes,
                fontsize=9, verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                          edgecolor='gray', alpha=0.85))

    plt.tight_layout()

    # 保存 PNG 和 PDF
    png_path = os.path.join(output_dir, 'trajectory_comparison.png')
    pdf_path = os.path.join(output_dir, 'trajectory_comparison.pdf')
    fig.savefig(png_path, dpi=300)
    fig.savefig(pdf_path, dpi=300)
    plt.close(fig)

    print(f"\n[VIZ] 轨迹对比图已保存:")
    print(f"      PNG: {png_path}")
    print(f"      PDF: {pdf_path}")
    print(f"      指标: ATE={ate:.3f}m, RPE={rpe:.4f}m/frame, "
          f"MaxErr={max_err:.3f}m, Drift={drift:.2f}%")

    return ate, rpe, max_err, drift


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
    # recoverPose 只恢复单位方向（无度量尺度）：初值取 IMU 帧间位移量级
    # (≈speed×dt≈0.1)，随后由 estimate_scale 在线跟踪速度变化；初值 1.0 会在
    # EMA 收敛前把 VO 轨迹过积分 ~18m → 位置残差恒超阈 → 位置分支永不执行
    scale_estimator = ScaleEstimator(fixed_scale_value=0.10)

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

    # 【MOD:D2】油门/刹车平滑状态初始化
    prev_throttle = 0.0
    prev_brake = 0.0

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
                    imu_dr, _ = ekf.get_imu_dead_reckoning_pose()   # 独立纯IMU航位(消融基线)
                    fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()
                    fusion_vel = ekf.get_current_velocity()
                    pos_uncertainty = ekf.get_position_uncertainty()
                    ekf._log_counter += 1
                    if ekf._debug_log and ekf._log_counter % 50 == 0:
                        print(f"[EKF DEBUG] Frame {img_idx}: timestamp diff={timestamp_diff:.4f}s > 0.05s, VO update SKIPPED. "
                              f"Accum: accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}")
                    # 仍写入记录（VO 日志同样写携带绝对位姿，口径与零运动分支一致）
                    gt_loc = vehicle.get_location()
                    gt_rot = vehicle.get_transform().rotation
                    gt_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                                 f"{math.radians(gt_rot.roll):.6f},"
                                 f"{math.radians(gt_rot.pitch):.6f},"
                                 f"{math.radians(gt_rot.yaw):.6f}\n")
                    vo_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{vo_abs_pose[0]:.6f},{vo_abs_pose[1]:.6f},{vo_abs_pose[2]:.6f},"
                                 f"{vo_abs_pose[3]:.6f},{vo_abs_pose[4]:.6f},{vo_abs_pose[5]:.6f}\n")
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
                        f"{imu_dr[0]:.6f},{imu_dr[1]:.6f},{imu_dr[2]:.6f},"
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
                    # VO 返回零运动（特征不足/匹配失败）：携带最近一次有效绝对位姿
                    # 进入 update 做位置锚定，防止 ~40% 无特征帧纯 IMU 漂移累积
                    # （位移=0 → 速度观测≈0，会被速度卡方门按状态速度自动拒绝，
                    #  仅位置观测生效 → 锚定到最后有效 VO 位姿）
                    ekf.imu_prediction(imu_data.data)
                    if ekf._last_vo_pose is not None:
                        # VO 已锚定过：携带最近一次有效绝对位姿执行 update
                        ekf._last_vo_inliers = num_matches
                        ekf.visual_update(vo_abs_pose)
                        fusion_pos, fusion_att = ekf.get_current_pose()
                    else:
                        # 首个有效 VO 之前：保持原纯 IMU 积分状态
                        imu_pos, _ = ekf.get_current_pose()
                        fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()
                    imu_dr, _ = ekf.get_imu_dead_reckoning_pose()   # 独立纯IMU航位(消融基线)
                    fusion_vel = ekf.get_current_velocity()
                    pos_uncertainty = ekf.get_position_uncertainty()
                    ekf._log_counter += 1
                    if ekf._debug_log and ekf._log_counter % 50 == 0:
                        print(f"[EKF DEBUG] Frame {img_idx}: VO zero motion (matches={num_matches}), carried VO pose update. "
                              f"Accum: accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}")
                    # 同步写入 GT / VO(零运动帧携带最近有效绝对位姿) / 纯 IMU 状态
                    gt_loc = vehicle.get_location()
                    gt_rot = vehicle.get_transform().rotation
                    gt_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                                 f"{math.radians(gt_rot.roll):.6f},"
                                 f"{math.radians(gt_rot.pitch):.6f},"
                                 f"{math.radians(gt_rot.yaw):.6f}\n")
                    # VO 日志写携带绝对位姿(特征丢失期间 VO 轨迹估计保持不变)；
                    # 原 0,0,0 把"零增量"误记为"零绝对位姿"，使 Pure-VO 基线虚低
                    vo_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{vo_abs_pose[0]:.6f},{vo_abs_pose[1]:.6f},{vo_abs_pose[2]:.6f},"
                                 f"{vo_abs_pose[3]:.6f},{vo_abs_pose[4]:.6f},{vo_abs_pose[5]:.6f}\n")
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
                        f"{imu_dr[0]:.6f},{imu_dr[1]:.6f},{imu_dr[2]:.6f},"
                        f"{fusion_vel[0]:.6f},{fusion_vel[1]:.6f},{fusion_vel[2]:.6f},"
                        f"{pos_uncertainty[0]:.6f},{pos_uncertainty[1]:.6f},{pos_uncertainty[2]:.6f}\n")
                    if img_idx % 10 == 0:
                        fusion_log.flush()
                        gt_log.flush()
                        vo_log.flush()
                        aligned_imu_f.flush()
                    continue

                # ---- 尺度（单目 VO 尺度歧义 = 车体速度 × 帧间隔） ----
                # recoverPose 只恢复单位方向（t 无度量信息）：度量尺度 = 速度×dt
                # ≈0.10（GT 路径 495m/250s×0.05s 回归）。此前 current_scale 恒 1.0
                # → 位置/速度观测量纲错误 → 卡方门恒拒 → 融合退化为纯 IMU。
                # 在线 estimate_scale 以漂移的 IMU 航位位移为参考会随漂移发散(→0.7)，
                # 故取固定尺度（ScaleEstimator 推荐用法）。
                scale = scale_estimator.get_current_scale()

                # ---- 累积 VO 相对运动 → 绝对位姿（坐标系修正） ----
                # VO process_frame() 返回帧间相对运动：平移在【相机坐标系】
                # （x右/y下/z光轴），前进运动主要在光轴 dz 上；roll/pitch/yaw
                # 是帧间【增量】欧拉角（源自帧间 R）。
                # 旧累积把 (dx,dy) 当水平、dz 当高度 → 前进量被记成高度（vo_z 达
                # -1603m）。修正：按相机固定安装 pitch=-20° 做正交变换
                # （光轴 dz → 车辆前进，dy → 高度，dx → 左），再按当前偏航旋转
                # 到世界系累加，并应用度量尺度。
                cam_dx, cam_dy, cam_dz = float(vo_pose[0]), float(vo_pose[1]), float(vo_pose[2])
                # 相机固定安装(pitch=-20°，光轴指向车体后下方)的正交映射：
                # 光轴 -dz → 车辆前进、dx → 左、dy → 高度（符号经录制数据回归验证）
                c20 = math.cos(math.radians(20.0))
                s20 = math.sin(math.radians(20.0))
                veh_fwd = -(s20 * cam_dy + c20 * cam_dz)
                veh_left = cam_dx
                veh_up = s20 * cam_dz - c20 * cam_dy
                # 车体系 → 世界系（用当前 VO 偏航旋转后累加），应用度量尺度
                vo_yaw = vo_abs_pose[5]
                cos_y = math.cos(vo_yaw)
                sin_y = math.sin(vo_yaw)
                vo_abs_pose[0] += scale * (veh_fwd * cos_y - veh_left * sin_y)
                vo_abs_pose[1] += scale * (veh_fwd * sin_y + veh_left * cos_y)
                vo_abs_pose[2] += scale * veh_up
                # 姿态：roll/pitch/yaw 为帧间增量，叠加后回绕到 (-pi, pi]
                vo_abs_pose[3] = (vo_abs_pose[3] + vo_pose[3] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[4] = (vo_abs_pose[4] + vo_pose[4] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[5] = (vo_abs_pose[5] + vo_pose[5] + np.pi) % (2 * np.pi) - np.pi

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
                imu_dr, _ = ekf.get_imu_dead_reckoning_pose()   # 独立纯IMU航位(消融基线)

                # 执行 VO 视觉更新（如果无异常跳变）
                if not vo_jump_skip:
                    ekf._last_vo_inliers = num_matches  # 喂VO内点数，驱动质量自适应R
                    ekf.visual_update(vo_abs_pose_current)
                    fusion_pos, fusion_att = ekf.get_current_pose()
                else:
                    fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()

                fusion_vel = ekf.get_current_velocity()
                pos_uncertainty = ekf.get_position_uncertainty()

                # ---- 调试日志 ----
                ekf._log_counter += 1
                if ekf._debug_log and ekf._log_counter % 50 == 0:
                    kalman_gain_trace = np.trace(ekf.P[:3, :3])
                    innovation_norm = ekf.innovation_history[-1] if ekf.innovation_history else 0.0
                    vel_K = ekf._last_K
                    vel_K_norm = np.linalg.norm(vel_K[:3, :3]) if vel_K is not None else 0
                    print(f"[EKF DEBUG] Frame {img_idx}: "
                          f"VO={'OK' if not vo_jump_skip else 'SKIP'}, "
                          f"calls={ekf._update_call_count}, "
                          f"scale_ema={ekf._vo_scale_ema:.4f}, "
                          f"vel_K={vel_K_norm:.4f}, "
                          f"innov={innovation_norm:.3f}, "
                          f"pos_skip={ekf._pos_skip_count}, "
                          f"accepted={ekf.innovation_accepted}, "
                          f"matches={num_matches}")

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
                    f"{imu_dr[0]:.6f},{imu_dr[1]:.6f},{imu_dr[2]:.6f},"
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
                # 【MOD:MAP】换目标后验证路径有效性
                validate_agent_path(agent, vehicle, spawn_points, world)

            # 【MOD:MAP】每 100 帧动态更新自适应安全距离和 PID
            if img_idx % 100 == 0:
                road_width = estimate_road_width(vehicle, world)
                adaptive_safe_dist = compute_adaptive_safe_distance(road_width)
                try:
                    if hasattr(agent, '_min_distance'):
                        agent._min_distance = adaptive_safe_dist
                except (AttributeError, KeyError, TypeError):
                    pass
                # 曲率自适应 PID
                curvature = estimate_path_curvature(agent)
                apply_adaptive_pid(agent, curvature)

            # 【MOD:E6】提前获取车速，供控制逻辑和停滞检测使用
            vel = vehicle.get_velocity()
            speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

            # 【MOD:MAP】诊断日志：每 200 帧打印路径和路点信息
            if img_idx % 200 == 0 and img_idx > 0:
                try:
                    if hasattr(agent, '_local_planner'):
                        wpq = agent._local_planner.waypoints_queue
                        wp_count = len(wpq) if wpq else 0
                        curv = estimate_path_curvature(agent)
                        rw = estimate_road_width(vehicle, world)
                        n_dyn = count_nearby_dynamic_actors(vehicle, world)
                        print(f"[DIAG] Frame {img_idx}: speed={speed:.1f}m/s, "
                              f"waypoints={wp_count}, curvature={curv:.3f}rad, "
                              f"road_width={rw:.1f}m, nearby_dynamic={n_dyn}, "
                              f"collisions={collision_sensor.collision_count}")
                except Exception:
                    pass

            try:
                control = agent.run_step()
                control.manual_gear_shift = False

                # 【MOD:D1】转向角限幅：最大 ±0.5（约 ±30°），防止急转弯
                max_steer = 0.5
                control.steer = max(-max_steer, min(max_steer, control.steer))

                # 【MOD:E4】油门/刹车平滑，α 从 0.3 提高到 0.6，减少过度压制
                alpha = 0.6
                control.throttle = alpha * control.throttle + (1 - alpha) * prev_throttle
                control.brake = alpha * control.brake + (1 - alpha) * prev_brake
                prev_throttle = control.throttle
                prev_brake = control.brake

                # 【MOD:E5】碰撞制动时限：仅碰撞后 1.5 秒内强制减速，之后自动释放
                # 理由：原逻辑 collision_count>0 永真，一次碰撞后永久刹车导致停滞
                if collision_sensor.collision_count > 0:
                    if time.time() - collision_sensor.last_collision_time < 1.5:
                        control.throttle = 0.0
                        control.brake = max(control.brake, 0.3)

                # 【MOD:MAP】刹车歧视：窄路无动态障碍物时抑制过度刹车
                # 避免 Town10HD 等窄地图误判墙壁为障碍物导致频繁无故刹停
                if control.brake > 0.3 and collision_sensor.collision_count == 0:
                    rw = estimate_road_width(vehicle, world)
                    n_dyn = count_nearby_dynamic_actors(vehicle, world)
                    if rw < 5.0 and n_dyn == 0:
                        # 窄路且无动态障碍物，大幅降低刹车
                        control.brake = min(control.brake, 0.15)
                        # 低速时补充油门，防止卡死
                        if speed < 2.0:
                            control.throttle = max(control.throttle, 0.15)

                # 【MOD:E6】停滞恢复：车速 < 0.05 且无碰撞时，连续 80 帧后强制释放刹车
                if speed < 0.05 and collision_sensor.collision_count == 0:
                    if stagnant_count > 80:
                        control.brake = 0.0
                        control.throttle = max(control.throttle, 0.3)
                        if stagnant_count == 81:
                            print(f"[STUCK RECOVERY] Frame {img_idx}: 强制释放刹车，尝试恢复前进")

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

            # 【MOD:E6】speed 已在控制段之前计算，此处复用；停滞超 150 帧重置
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
                        # 【MOD:C1】重置时同步 PID 参数，与 init_carla_environment 保持一致
                        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
                            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.3
                                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.01
                                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.1
                            if hasattr(agent._vehicle_controller, '_args_longitudinal_dict'):
                                agent._vehicle_controller._args_longitudinal_dict['K_P'] = 1.0
                                agent._vehicle_controller._args_longitudinal_dict['K_I'] = 0.02
                                agent._vehicle_controller._args_longitudinal_dict['K_D'] = 0.0
                        # 【MOD:MAP】重置时使用自适应安全距离
                        rw = estimate_road_width(vehicle, world)
                        adaptive_safe_dist = compute_adaptive_safe_distance(rw)
                        if hasattr(agent, '_min_distance'):
                            agent._min_distance = adaptive_safe_dist
                        if hasattr(agent, '_max_brake'):
                            agent._max_brake = 0.8
                    except (AttributeError, KeyError, TypeError):
                        pass

                    destination = select_forward_destination(vehicle, spawn_points)
                    agent.set_destination(destination)
                    # 【MOD:MAP】重置后验证路径
                    validate_agent_path(agent, vehicle, spawn_points, world)
                    print(f"新目标: ({destination.x:.1f}, {destination.y:.1f})")

                    camera, cam_transform = create_rgb_camera(world, bp_lib, vehicle, sensor_queue)
                    imu = create_imu_sensor(world, bp_lib, vehicle, sensor_queue, cam_transform)
                    collision_sensor = CollisionSensor(vehicle)
                    stagnant_count = 0
                    prev_throttle = 0.0   # 【MOD:D2】重置时清空平滑状态
                    prev_brake = 0.0
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

        # EKF 融合统计
        try:
            total = ekf.innovation_accepted + ekf.innovation_rejected
            acc_rate = (ekf.innovation_accepted / total * 100) if total > 0 else 0
            print(f"\n[EKF STATS] 尺度初始化: {ekf._vo_scale_initialized}")
            print(f"            尺度 EMA: {ekf._vo_scale_ema:.4f}")
            print(f"            visual_update 调用: {ekf._update_call_count}")
            print(f"            速度/姿态更新: accepted={ekf.innovation_accepted}, "
                  f"rejected={ekf.innovation_rejected} "
                  f"({acc_rate:.1f}% accepted)")
            print(f"            位置观测跳过(残差>100m): {ekf._pos_skip_count}")
            print(f"            总帧数: {img_idx}")  # 修复：total_frames 未定义，改用同作用域 img_idx
        except Exception as e:
            print(f"[EKF STATS] 统计失败: {e}")

        # 生成轨迹可视化对比图（论文用图）
        try:
            plot_trajectory_comparison(OUTPUT_DIR, town_name=TARGET_MAP)
        except Exception as e:
            print(f"[VIZ] 轨迹可视化失败: {e}")


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
    _parser.add_argument('--map', type=str, default=DEFAULT_TARGET_MAP,
                         help=f'CARLA 地图名称 (默认: {DEFAULT_TARGET_MAP}), '
                              f'例如: Town01, Town02, Town03, Town05, Town10HD')
    _args = _parser.parse_args()

    # 根据命令行参数覆盖地图和输出目录
    TARGET_MAP = _args.map
    OUTPUT_DIR = os.path.join(current_dir, '..', 'data', f'{TARGET_MAP}Data_IMU_Fusion')

    main(headless=_args.headless, host=_args.host, port=_args.port)
