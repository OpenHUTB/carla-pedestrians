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

# ─── 离线模式 (EKF_OFFLINE=1): --replay 只读 CSV 复算, 不依赖 carla C 扩展 ───
_EKF_OFFLINE = os.environ.get('EKF_OFFLINE', '') == '1'
if _EKF_OFFLINE:
    import types as _types
    _carla_stub = _types.ModuleType('carla')
    sys.modules.setdefault('carla', _carla_stub)
    carla = _carla_stub
else:
    import carla
from scipy.spatial.transform import Rotation as R
from scipy import stats as _scipy_stats
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

if _EKF_OFFLINE:
    # 离线复算不需要 agents (仅采集主循环使用 BehaviorAgent)
    BehaviorAgent = None
elif not _carla_agents_found:
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
AGENT_MAX_SPEED = 12                # km/h，Town02 密集城市需降速避免撞车
AGENT_SAFE_DISTANCE = 6.0           # 米，空旷道路避免误触发避险刹车
COLLISION_RESET_THRESHOLD = 3       # 碰撞次数阈值

# 消融开关：True = R 恒取基准值（关闭自适应缩放），用于隔离验证噪声模型影响
FIXED_R = False

# 折返采集: 直行 TURNAROUND_OUT_DIST 后沿路网绕环回出生点, 验证 NLM 跨时间闭环
TURNAROUND_ACTIVE = False
TURNAROUND_OUT_DIST = 100.0  # 双向折返去程长度(已验证路线road 10可行驶144m)
# 绕环上限(m): 去程目标超出该距离的回路直接放弃(防止 ~600m 长环撞服务器崩溃窗口)
TURNAROUND_TURN_DIST = 220.0
TURNAROUND_MAX_FRAMES = 10000   # 绕环~600m, 5000帧不够走完
# 双向折返起点(x,y): 双向路段定点投放, 去程后沿对向车道原路返回 = 真重访
# 已验证路线(kbs/scan_bidir_routes.py, Town01): (167.2,47.5) → 去程起点
# (167.2,59.5,road 10/lane -1,yaw 0°) → 去程终点(311.2,59.5) → flip对向车道
# → 回程目标(167.2,55.5), 横向偏移4.0m(CE门[3,20]), 隔离实测0fence+回到起点
TURNAROUND_BIDIR_START = None
# 运动学折返: 沿路点逐tick传送, 不靠behavior agent
TURNAROUND_KIN_START = None
KIN_TURN_SPEED = 3.0        # m/s
KIN_TURN_MAX_FRAMES = 3200  # 200m路径+静止尾部

# 残差滑动窗口平滑(自适应R用): 最近 N 帧马氏距离均值滤波后映射 R, 抑制单帧跳变;
# 卡方离群门仍用原始单帧残差。ENABLE_RESIDUAL_SMOOTH=False 时窗口退化为 1
ENABLE_RESIDUAL_SMOOTH = True
RESID_SMOOTH_WINDOW = 10        # 速度/位置分支窗口(帧)
RESID_SMOOTH_WINDOW_ATT = 10    # 姿态分支窗口(帧)
# 残差→R 映射: resid = RESID_MAP_BASE + (eff/dof)**RESID_MAP_POWER, eff = dist/SENS
RESID_MAP_SENSITIVITY = 5.0
RESID_MAP_BASE = 0.05           # 小残差时抬离 R 下限, 避免 K→1 直接抄 VO
RESID_MAP_POWER = 1.5

# ─── 融合增强: VO 度量尺度状态化(EKF 第10维, log 参数化) + 转弯航向观测 ───
# 尺度状态 = 叠加在固定 0.10 记录尺度上的修正因子, 初值 1.0, 应收敛 ≈3.5
VO_SCALE_PRIOR = 1.0
VO_SCALE_SIGMA0 = 0.80       # log 域初值标准差
VO_SCALE_Q = 2.0e-5          # log 尺度过程噪声
VO_SCALE_P_MAX = 1.00        # log 尺度协方差上限
# 转弯航向观测: a_lat/v 给出绝对航向率, 与陀螺零偏无关, 打破 VO 航向漂移
HEADING_OBS_ENABLED = False
TURN_GATE_AVG_SPEED = 3.0    # 转弯门限: 窗口平均速度 (m/s)
TURN_GATE_MIN_ALAT = 0.3     # 转弯门限: 横向加速度 (m/s²)
TURN_GATE_MAX_GYRZ = 1.2     # 转弯门限: |yaw rate| 上限 (rad/s)
HEADING_MAX_STEP = 0.26      # 航向观测硬门限: |残差|>15° 直接拒
TURN_SPEED_WIN = 30          # 转弯判定速度窗口(帧)
SCALE_WIN = 60               # 尺度观测窗口(帧, 3s@20Hz)
SCALE_R_BASE = 0.02          # 尺度观测噪声(log 域, ~2%)
SCALE_MIN_DISP = 0.5         # 窗口内最小 VO 记录位移(0.10 尺度单位)
SCALE_VMIN = 0.2             # 窗口内最小平均速度(记录单位 ≈2 m/s 真实)
# 尺度稳健性: 步限钳位 + 协方差下限, 防 P 初值大时首窗一步跳变后 P 塌缩冻结(10HD 失效修复)
SCALE_MAX_STEP = 0.25        # 单步 log 域创新量上限(~±28%)
SCALE_P_CLAMP = 0.25         # 钳位更新后协方差下限(保留可修正性)
SCALE_P_FLOOR = 0.01         # 常规更新后协方差下限
# 速度域门控/截尾: 默认关(四图回归证伪默认适用性), 保留 env 开关供单图实验
SCALE_KIN_VMIN, SCALE_KIN_VMAX = 0.0, float('inf')  # 运动学样本速度域门控(默认关)
SCALE_TRIM_FRAC = 0.0        # 窗中值前截尾上尾比例(默认关; 0.10 启用)
# 固定速度形状 s(v)=exp(A+B·v): 四图 pooled 运动学样本 OLS 标定, 实验分支默认关
VO_SHAPE_A = -0.1094
VO_SHAPE_B = 0.1884
VO_SHAPE_VREF = 6.0          # 参考速度: 形状启用时尺度状态语义 = s(v_ref) 水平
R_Z_PRIOR = 25.0             # z 高度先验噪声 (m²)
POSITION_WIN = 30            # 位置锚定窗口(帧, 1.5s@20Hz), 防 VO 航向漂移灌入融合位置

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


def wait_vehicle_deploy(vehicle, target_loc, timeout=2.0, settle=0.15):
    """等待 set_transform 瞬移真正生效(下一仿真tick才应用)。

    CARLA 的 set_transform 在下一个 tick 生效; 若瞬移后立即
    BehaviorAgent.set_destination, planner 读到的是瞬移前旧位置,
    会从旧位置(随机出生点)规划路由 → 车瞬移到位后沿错误路由
    冲出车道撞静态墙(2026-10-06 双向折返反复"碰撞过多"重置的根因)。
    循环读回实际位置直到与目标一致(或超时), 再静置让物理稳定。
    """
    # 同步模式下 set_transform 在下次 world.tick() 才生效: 只 sleep 轮询
    # 位置永远停在旧点 → 规划器以瞬移前位置路由 (2026-10-06 双向折返
    # 432路点 head=(0,2) 偏航撞墙根因)。必须自己推进 tick 直到位置到位。
    t0 = time.time()
    _world = None
    try:
        _world = vehicle.get_world()
    except Exception:
        pass
    while time.time() - t0 < timeout:
        if _world is not None:
            try:
                _world.tick()
            except Exception:
                pass
        try:
            if vehicle.get_location().distance(target_loc) < 1.5:
                break
        except Exception:
            return
        time.sleep(0.05)
    time.sleep(settle)


def make_fresh_agent(vehicle, world):
    """创建全新 BehaviorAgent(不设目的地)。

    set_transform 瞬移后, 旧 agent 的 local_planner/内部状态仍与瞬移前
    位置绑定, 车会沿旧路由偏离(2026-10-06 双向折返投放后漂向西南撞
    植被的根因; 隔离验证中"瞬移后新建agent"是唯一0碰撞路径)。
    所有瞬移点统一先换新 agent 再 set_destination。
    """
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
            agent._vehicle_controller._args_lateral_dict['K_P'] = 0.3
            agent._vehicle_controller._args_lateral_dict['K_I'] = 0.01
            agent._vehicle_controller._args_lateral_dict['K_D'] = 0.1
            agent._vehicle_controller._args_longitudinal_dict['K_P'] = 1.0
            agent._vehicle_controller._args_longitudinal_dict['K_I'] = 0.02
            agent._vehicle_controller._args_longitudinal_dict['K_D'] = 0.0
        rw = estimate_road_width(vehicle, world)
        agent._min_distance = compute_adaptive_safe_distance(rw)
        agent._max_brake = 0.8
    except (AttributeError, KeyError, TypeError):
        pass
    return agent


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


def build_turnaround_relay(world, start_loc, return_dist=100.0, seg=8.0,
                           turn_dist=None):
    """构造折返接力目的地序列(生死门用)。

    Town01主路单行西向, 单一目的地"回出生点"会被local planner拒规划
    (483m绕环)。改为: 去程=出生点前方return_dist处; 回程=沿行驶方向
    的绕环路点(每seg米一个接力点), 每段都在local planner能力内。
    返回 (out_dest, relay_pts): out_dest=去程目标, relay_pts=回程接力点列表。
    若沿行驶方向找不到回出生点的回路, 返回 (out_dest, [])。
    """
    m = world.get_map()
    def _aslist(x):
        return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])
    wp0 = _aslist(m.get_waypoint(start_loc, project_to_road=True))[0]
    # 去程目标: 沿行驶方向走到 return_dist
    out_wp = wp0
    for dstep in range(20, 400, 10):
        nxt = _aslist(wp0.next(dstep))
        if not nxt:
            break
        out_wp = nxt[0]
        if out_wp.transform.location.distance(start_loc) >= return_dist:
            break
    out_dest = carla.Location(out_wp.transform.location.x,
                              out_wp.transform.location.y,
                              out_wp.transform.location.z)
    # 回程: 从去程目标沿行驶方向Dijkstra回出生点15m内, 每seg米采样接力点
    import heapq
    prev = {out_wp.id: None}
    dist = {out_wp.id: 0.0}
    seq = 0
    pq = [(0.0, seq, out_wp)]
    found = None
    expanded = 0
    while pq:
        du, _, u = heapq.heappop(pq)
        expanded += 1
        if du > dist.get(u.id, 1e18):
            continue
        if expanded > 30000:
            break
        # 绕环上限: 去程目标超出 turn_dist 的回路直接放弃(长环撞服务器崩溃窗口)
        if (turn_dist is not None
                and u.transform.location.distance(start_loc) > turn_dist):
            print(f"[TURNAROUND] 去程 {u.transform.location.distance(start_loc):.0f}m "
                  f"超出绕环上限 {turn_dist:.0f}m, 放弃折返")
            return out_dest, []
        if u.transform.location.distance(start_loc) < 15.0:
            found = u
            break
        for v, wl in ((n, n.transform.location.distance(u.transform.location))
                      for n in _aslist(u.next(2.0))):
            nd = du + wl
            if nd < dist.get(v.id, 1e18):
                dist[v.id] = nd
                prev[v.id] = u
                seq += 1
                heapq.heappush(pq, (nd, seq, v))
    if found is None:
        print(f"[TURNAROUND] 沿行驶方向找不到回出生点的回路(expanded={expanded})")
        return out_dest, []
    # 重建路径并采样接力点
    path = [found]
    u = found
    while prev[u.id] is not None:
        u = prev[u.id]
        path.append(u)
    path.reverse()
    total = sum(a.transform.location.distance(b.transform.location)
                for a, b in zip(path, path[1:]))
    relay = []
    acc = 0.0
    next_at = seg
    for a, b in zip(path, path[1:]):
        step = a.transform.location.distance(b.transform.location)
        acc += step
        while acc >= next_at:
            relay.append(carla.Location(
                b.transform.location.x, b.transform.location.y,
                b.transform.location.z))
            next_at += seg
    relay.append(carla.Location(start_loc.x, start_loc.y, start_loc.z))
    print(f"[TURNAROUND] 接力路线: 去程={out_dest.x:.1f},{out_dest.y:.1f} "
          f"({out_dest.distance(start_loc):.0f}m), "
          f"回程绕环={total:.0f}m, 接力点={len(relay)}个")
    return out_dest, relay


def build_kinematic_turnaround(world, start_loc, out_dist=100.0,
                               arc_r=None):
    """运动学折返路径(不靠behavior agent, 逐tick传送)。

    路径 = 去程(沿当前车道路点) + 半圆U-turn弧(投影回路面) + 回程(对向车道路点)。
    返回 (path, meta): path为[(Location, yaw_deg), ...], 弧前插入一个
    (None, None)标记表示U-turn段起点。若对向车道不可用, 弧段降级为原地转向。
    """
    import math
    m = world.get_map()

    def _aslist(x):
        return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])

    wp0 = _aslist(m.get_waypoint(start_loc, project_to_road=True))[0]
    # 去程: 沿当前车道路点
    out_pts = [wp0.transform.location]
    u = wp0
    while u.transform.location.distance(out_pts[0]) < out_dist:
        nxt = _aslist(u.next(2.0))
        if not nxt:
            break
        u = nxt[0]
        out_pts.append(u.transform.location)
    far = out_pts[-1]
    # 对向车道路点(与去程同路点序)
    ret_wps = []
    ok = True
    for p in out_pts:
        l = _aslist(m.get_waypoint(p, project_to_road=True))[0].get_left_lane()
        if l is None:
            ok = False
            break
        ret_wps.append(l.transform.location)
    if not ok or len(ret_wps) != len(out_pts) or arc_r is None:
        # 降级: 原地转向(VO为纯旋转, 仍可闭环, 物理不完美)
        path = [(loc, wp0.transform.rotation.yaw) for loc in out_pts]
        path.append((None, None))
        path += [(loc, (wp0.transform.rotation.yaw + 180.0) % 360.0)
                 for loc in reversed(ret_wps)]
        print(f"[KIN] 降级为原地U-turn (对向车道不可用)")
        return path, {'arc': False, 'out_len': far.distance(out_pts[0])}
    # U-turn弧: 圆心=去程终点与对向车道终点中点, 半径=arc_r
    c = carla.Location((far.x + ret_wps[-1].x) / 2.0,
                       (far.y + ret_wps[-1].y) / 2.0,
                       (far.z + ret_wps[-1].z) / 2.0)
    r = arc_r if arc_r > 0 else c.distance(far)
    # 从far到ret_wps[-1]的半圆, 按弧长2m采样
    a0 = math.atan2(far.y - c.y, far.x - c.x)
    a1 = math.atan2(ret_wps[-1].y - c.y, ret_wps[-1].x - c.x)
    # 取逆时针/顺时针中"不穿越去程走廊"的半圆: 两向角差应≈±pi, 取绝对值大者
    d_cw = (a1 - a0) % (2 * math.pi)
    d_ccw = (a0 - a1) % (2 * math.pi)
    d = d_cw if d_cw > d_ccw else -d_ccw
    n_arc = max(10, int(abs(d) * r / 2.0))
    arc_pts = []
    for k in range(1, n_arc + 1):
        a = a0 + d * k / n_arc
        q = carla.Location(c.x + r * math.cos(a), c.y + r * math.sin(a), c.z)
        # 投影回路面, 防弧跑出道路
        q = _aslist(m.get_waypoint(q, project_to_road=True))[0].transform.location
        arc_pts.append(q)
    path = [(loc, wp0.transform.rotation.yaw) for loc in out_pts]
    path.append((None, None))  # U-turn标记(运行时按弧切向插值yaw)
    path += [(loc, None) for loc in arc_pts]
    yaw_ret = (wp0.transform.rotation.yaw + 180.0) % 360.0
    path += [(loc, yaw_ret) for loc in reversed(ret_wps)]
    print(f"[KIN] 运动学折返: 去程={far.distance(out_pts[0]):.0f}m, "
          f"U-turn弧={len(arc_pts)}点(r={r:.1f}m), 回程对向车道={len(ret_wps)}点")
    return path, {'arc': True, 'out_len': far.distance(out_pts[0])}


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
                # CARLA 0.9.16 属性为 _waypoints_queue, 旧版为 waypoints_queue
                wp_queue = getattr(agent._local_planner, '_waypoints_queue',
                                   None)
                if wp_queue is None:
                    wp_queue = getattr(agent._local_planner, 'waypoints_queue',
                                       None)
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


def _spawn_agent_for_vehicle(world, bp_lib, vehicle, spawn_points):
    """为车辆初始化智能体：限速、PID、安全距离、目标点（初始化与重置共用）

    返回 (agent, destination)。智能体参数配置集中在此，
    避免 init 路径与主循环内重置路径两处参数漂移。
    """
    agent = BehaviorAgent(vehicle, behavior=AGENT_BEHAVIOR)
    agent.follow_speed_limits(False)
    try:
        agent.set_max_speed(AGENT_MAX_SPEED / 3.6)
    except AttributeError:
        try:
            agent.set_target_speed(AGENT_MAX_SPEED / 3.6)
        except AttributeError:
            agent._max_speed = AGENT_MAX_SPEED / 3.6
            print(f"使用备用方式设置速度: {AGENT_MAX_SPEED} km/h")

    # 降低横向 PID 增益防转向过猛；纵向降低 I 项防积分饱和
    # 自适应安全距离：按道路宽度动态调整，窄路降低避免误触发避险刹车
    try:
        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.3
                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.01
                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.1
            if hasattr(agent._vehicle_controller, '_args_longitudinal_dict'):
                agent._vehicle_controller._args_longitudinal_dict['K_P'] = 1.0
                agent._vehicle_controller._args_longitudinal_dict['K_I'] = 0.02
                agent._vehicle_controller._args_longitudinal_dict['K_D'] = 0.0
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
    validate_agent_path(agent, vehicle, spawn_points, world)
    return agent, destination


def _reset_vehicle(world, bp_lib, vehicle, spawn_points, sensor_queue,
                   camera, imu, collision_sensor):
    """车辆重置：先成功生成新车辆，再销毁旧车辆（生成失败时旧车保持可用，
    循环可继续）。返回 (new_vehicle, agent, camera, cam_transform, imu, collision_sensor)。"""
    new_vehicle, _ = safe_spawn_vehicle(world, bp_lib)

    # 新车就绪后才销毁旧车与旧传感器
    for _s in (camera, imu, collision_sensor.sensor):
        try:
            _s.stop()
        except Exception:
            pass
        try:
            _s.destroy()
        except Exception:
            pass
    try:
        vehicle.destroy()
    except Exception:
        pass
    time.sleep(0.3)

    # 物理参数 + 智能体 + 目标点（与初始化路径共用同一套参数）
    try:
        physics_control = new_vehicle.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        new_vehicle.apply_physics_control(physics_control)
    except Exception as e:
        print(f"[WARN] 新车辆物理参数配置失败: {e}")
    agent, destination = _spawn_agent_for_vehicle(world, bp_lib, new_vehicle, spawn_points)
    print(f"新目标: ({destination.x:.1f}, {destination.y:.1f})")

    # 重建传感器（挂新车）
    new_camera, cam_transform = create_rgb_camera(world, bp_lib, new_vehicle, sensor_queue)
    new_imu = create_imu_sensor(world, bp_lib, new_vehicle, sensor_queue, cam_transform)
    new_collision = CollisionSensor(new_vehicle)

    return new_vehicle, agent, new_camera, cam_transform, new_imu, new_collision


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


def _try_reconnect_world(host, port):
    """尝试重新连接 CARLA，成功返回新 world，失败返回 None"""
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    _ = world.get_map().name  # 验证连接有效
    return world


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

    # 增强避障参数：自适应安全距离（窄路降低避免误触发避险刹车）
    road_width = estimate_road_width(vehicle, world)
    adaptive_safe_dist = compute_adaptive_safe_distance(road_width)
    print(f"[MAP] 道路宽度: {road_width:.1f}m, 自适应安全距离: {adaptive_safe_dist:.1f}m")

    try:
        # 降低横向 PID 增益防转向过猛；纵向降低 I 项防积分饱和
        if hasattr(agent, '_vehicle_controller') and agent._vehicle_controller is not None:
            if hasattr(agent._vehicle_controller, '_args_lateral_dict'):
                agent._vehicle_controller._args_lateral_dict['K_P'] = 0.3
                agent._vehicle_controller._args_lateral_dict['K_I'] = 0.01
                agent._vehicle_controller._args_lateral_dict['K_D'] = 0.1
            if hasattr(agent._vehicle_controller, '_args_longitudinal_dict'):
                agent._vehicle_controller._args_longitudinal_dict['K_P'] = 1.0
                agent._vehicle_controller._args_longitudinal_dict['K_I'] = 0.02
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

    # 12) 折返模式: 构造路线(去程目标 + 回程)
    if TURNAROUND_ACTIVE:
        if TURNAROUND_BIDIR_START is not None:
            # 双向折返: 定点投放到双向路段, 去程100m, 掉头沿对向车道原路返回
            m = world.get_map()
            def _aslist(x):
                return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])
            sx, sy = TURNAROUND_BIDIR_START
            wp0 = _aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                         project_to_road=True))[0]
            transform = wp0.transform
            # 该起点 waypoint z=0.00 位于路面内部(车体嵌入地面→瞬撞围栏),
            # 实测 z+0.1 为可行驶高度(见 neuro/kbs/test_spawn_z.py)
            transform.location.z += 0.1
            vehicle.set_transform(transform)
            wait_vehicle_deploy(vehicle, transform.location)
            start_loc = transform.location
            # 去程目标: 同路段沿行驶方向 TURNAROUND_OUT_DIST
            out_wp = wp0
            for dstep in range(20, 400, 10):
                nxt = _aslist(wp0.next(dstep))
                if not nxt:
                    break
                cand = nxt[0]
                if cand.road_id != wp0.road_id:
                    break
                out_wp = cand
                if out_wp.transform.location.distance(start_loc) >= TURNAROUND_OUT_DIST:
                    break
            out_dest = carla.Location(out_wp.transform.location.x,
                                      out_wp.transform.location.y,
                                      out_wp.transform.location.z)
            # 掉头flip点: 去程终点的对向车道航点(朝向=对向车道真实yaw);
            # 回程目标: 回程链上离起点最近点(与去程起点横向偏移≈车道宽,
            # 即CE闭环残差)。两者在隔离验证(kbs/test_flip_roundtrip.py)中
            # 已实证: 0fence、回到起点、偏移4.0m落CE门[3,20]
            _flip_wp = out_wp.get_left_lane()
            _ret_target = out_dest
            if _flip_wp is not None:
                # 沿对向车道路链找离起点最近点 = 重访目标(横向偏移≈车道宽)
                _rt = _flip_wp
                _best_d = float('inf')
                for _ in range(40):
                    _rtx = _aslist(_rt.next(8.0))
                    if not _rtx or _rtx[0].road_id != _flip_wp.road_id:
                        break
                    _rt = _rtx[0]
                    _d = _rt.transform.location.distance(start_loc)
                    if _d < _best_d:
                        _best_d = _d
                        _ret_target = carla.Location(
                            _rt.transform.location.x,
                            _rt.transform.location.y,
                            _rt.transform.location.z)
                    if _rt.transform.location.distance(
                            _flip_wp.transform.location) > TURNAROUND_OUT_DIST + 40:
                        break
            # 瞬移后旧agent(随机出生点创建, 路由指向西南)必须弃用,
            # 否则车沿旧路由漂出车道(隔离验证已证: 瞬移后新建agent才0碰撞)
            _old_agent = agent
            agent = make_fresh_agent(vehicle, world)
            agent.set_destination(out_dest)
            validate_agent_path(agent, vehicle, spawn_points, world)
            try:
                _old_agent.destroy()
            except Exception:
                pass
            ctx = {'start_loc': start_loc, 'out_dest': out_dest,
                   'relay_pts': [_ret_target],
                   'relay_idx': 0, 'phase': 'out', 'bidir': True,
                   'flip_wp': _flip_wp}
            if _flip_wp is None:
                print("[TURNAROUND][FATAL] 去程终点无对向车道, 无法flip掉头 → 退出")
                sys.exit(3)
            print(f"[TURNAROUND] 双向折返: 投放=({start_loc.x:.1f},{start_loc.y:.1f}), "
                  f"去程=({out_dest.x:.1f},{out_dest.y:.1f}) "
                  f"({out_dest.distance(start_loc):.0f}m), "
                  f"flip点=({_flip_wp.transform.location.x:.1f},"
                  f"{_flip_wp.transform.location.y:.1f}) yaw="
                  f"{_flip_wp.transform.rotation.yaw:.0f}°, "
                  f"回程目标=({_ret_target.x:.1f},{_ret_target.y:.1f})")
        else:
            # 单向路网: 绕环接力路线
            start_loc = vehicle.get_transform().location
            out_dest, relay_pts = build_turnaround_relay(
                world, start_loc, return_dist=TURNAROUND_OUT_DIST,
                turn_dist=TURNAROUND_TURN_DIST)
            agent.set_destination(out_dest)
            validate_agent_path(agent, vehicle, spawn_points, world)
            ctx = {'start_loc': start_loc, 'out_dest': out_dest,
                   'relay_pts': relay_pts, 'relay_idx': 0, 'phase': 'out',
                   'bidir': False}
            if not relay_pts:
                # 该出生点绕不出短回路: 采几帧无意义, 立即退出让监督器换点重来
                print(f"[TURNAROUND] 无可行绕环 → 退出(rc=3), 监督器将换出生点重试")
                sys.exit(3)
            print(f"[TURNAROUND] 绕环折返: 出生点=({start_loc.x:.1f},{start_loc.y:.1f}), "
                  f"去程=({out_dest.x:.1f},{out_dest.y:.1f}), "
                  f"回程接力点={len(relay_pts)}个")
        return (world, bp_lib, vehicle, spawn_points, world.get_spectator(),
                agent, traffic_manager, collision_sensor, ctx)
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
    # IMU 系须对齐车体系(EKF 约定); 相机 transform 带 pitch=-20, 沿用会使重力补偿带恒值误差
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
# ═══════════════════════════════════════════════════════════════

def vo_shape_c(v):
    """固定速度形状修正因子 c(v)=exp(B·(v-v_ref))，参考速度处=1。v 钳位到样本有效域。"""
    v = min(max(float(v), SCALE_KIN_VMIN), SCALE_KIN_VMAX)
    return math.exp(VO_SHAPE_B * (v - VO_SHAPE_VREF))


class VelocityScaleModel:
    """单目 VO 局部尺度速度形状在线模型 s(v)=exp(a+b·v)（实验分支, EKF_VSCALE_ENABLE=1 启用）。

    VO 局部尺度(真实位移/记录位移)随车速系统性变化, 单一刻度状态有偏。
    用转弯稳态帧纯运动学样本 (v_kin=|a_lat|/|ω|, r=v_kin/v_VO_rec) 做
    带遗忘因子 RLS 拟合; 样本域 [V_LO, V_HI]/r 范围外剔除。
    应用侧 c(v)=s(v)/s(v_ref) 修正尺度, 观测侧归一到参考速度使尺度状态只跟踪水平;
    样本不足时 ready()=False、c(v)≡1 退化为单刻度。
    Town05 replay 验证为负收益(ATE 83.71→206.46): 低速端外推错 + 低速段无在线
    样本可拟合, 故默认关闭。
    """

    V_LO, V_HI = 2.0, 12.0   # 样本有效速度域（m/s），应用侧外推钳位
    R_LO, R_HI = 0.3, 30.0   # r 合理范围（超出=滑移/异常样本）
    R_NOISE = 0.02           # log 域观测噪声（与 SCALE_R_BASE 一致）

    def __init__(self, v_ref=6.0, lam=0.999, n_min=15):
        self.v_ref = v_ref
        self.lam = lam
        self.n_min = n_min
        self.a = math.log(3.0)   # 初值 s(v_ref)≈3.0（量级先验，RLS 收敛覆盖）
        self.b = 0.0
        self.P = np.eye(2) * 10.0
        self.n = 0

    def add(self, v_kin, r):
        """喂入一个 (v_kin, r) 运动学样本，递推更新 (a, b)。"""
        if not (self.V_LO <= v_kin <= self.V_HI and self.R_LO <= r <= self.R_HI):
            return
        phi = np.array([1.0, v_kin])
        theta = np.array([self.a, self.b])
        y = math.log(r)
        Pphi = self.P @ phi
        S = float(phi @ Pphi) + self.R_NOISE
        K = Pphi / S
        theta += K * (y - float(phi @ theta))
        self.P = (self.P - np.outer(K, phi @ self.P)) / self.lam
        self.a, self.b = float(theta[0]), float(theta[1])
        self.n += 1

    def ready(self):
        return self.n >= self.n_min

    def s(self, v):
        v = min(max(float(v), self.V_LO), self.V_HI)
        return math.exp(self.a + self.b * v)

    def c(self, v):
        """速度修正因子 c(v)=s(v)/s(v_ref)，参考速度处=1。"""
        return self.s(v) / self.s(self.v_ref)


class EKF_VIO:
    # 状态向量 10 维: [x, y, z, vx, vy, vz, roll, pitch, yaw, log(VO尺度水平)]
    # 第 10 维 = 尺度在参考速度处的 log 水平; 应用总尺度 = 水平 × c(v)
    def __init__(self, init_pose, init_vel, dt=0.05):
        self.x = np.array([
            init_pose[0], init_pose[1], init_pose[2],
            init_vel[0], init_vel[1], init_vel[2],
            init_pose[3], init_pose[4], init_pose[5],
            math.log(VO_SCALE_PRIOR),
        ], dtype=np.float64)
        self.dt = dt
        self.init_z = init_pose[2]
        self.init_pose = np.array(init_pose, dtype=np.float64)

        # 初始协方差
        self.P = np.diag([2.0, 2.0, 0.5, 2.0, 2.0, 1.0,
                          0.05, 0.05, 0.05, VO_SCALE_SIGMA0 ** 2])

        # 过程噪声 Q(连续时间, Q_d = Q_cont*dt); 平移>旋转, 保证 P 不过度收缩
        self.Q_cont = np.diag([0.18, 0.18, 0.09,   # 平移-位置 m²/s
                                0.72, 0.72, 0.36,   # 平移-速度 (m/s)²/s
                                0.0072, 0.0072, 0.018,  # 旋转-姿态 rad²/s
                                VO_SCALE_Q])        # log尺度 rad²/s
        # 观测噪声 R 基准值, visual_update 再乘质量自适应因子 qf(内点多→R小→信任VO)
        self.R_vel = 0.20                       # (m/s)², 1D 幅值观测(方向由航向提供)
        self.R_att = np.diag([0.05, 0.50])          # 姿态 rad² (yaw 由航向观测单独修正)
        self.R_heading = 0.05                       # 转弯航向率观测 rad²
        self.R_pos = np.diag([0.06, 0.06])          # 位置观测 m² (scale×VO增量锚定)
        self._vo_match_ref = 60.0   # VO内点数参考: ≥ref→qf=1; =ref/3→qf=3
        self.R_ADAPT_FLOOR = 0.008  # 残差自适应R乘性上下限, 防 R 无界
        self.R_ADAPT_CEIL = 10.0
        self.fixed_r = FIXED_R      # 固定R消融: True = R 恒为基准值

        # 卡方门限: 速度/姿态 0.99, 位置 0.999(减少误拒)
        self.chi2_vel = 9.21        # χ²(2,0.99)
        self.chi2_att = 9.21        # χ²(2,0.99)
        self.chi2_pos = 13.82       # χ²(2,0.999)
        self.chi2_heading = float(_scipy_stats.chi2.ppf(0.95, 1))  # χ²(1,0.95)=3.84

        # 转弯航向观测: 稳态转弯 dψ/dt=a_lat/v, 与陀螺零偏无关; 门控防直道噪声/急转
        self._turn_speed_win = deque(maxlen=TURN_SPEED_WIN)
        self._turn_count = 0
        self._heading_applied = 0
        self._heading_rejected = 0
        self.chi2_scale = float(_scipy_stats.chi2.ppf(0.99, 1))  # χ²(1,0.99)=6.63
        self._scale_vo_win = deque(maxlen=SCALE_WIN)   # 窗口起点 VO 位置(记录尺度)
        self._scale_imu_win = deque(maxlen=SCALE_WIN)  # 窗口起点纯 IMU 航位位置
        self._scale_applied = 0
        self._scale_rejected = 0
        self._scale_obs_hist = []    # [调试] s_obs 原始分布(EKF_DEBUG_SCALE)
        self._scale_dr_vel_hist = [] # [调试] 观测时 DR 速度幅值
        self._kin_win = deque(maxlen=200)   # 运动学速度比样本（转弯帧，尺度观测用）
        self._scale_kin_count = 0           # 运动学尺度观测节奏计数（每 5 帧取窗中值一次）
        # 免零偏航向基准: 转弯时按 dψ/dt=a_lat/v 累积, 直道冻结
        self._heading_base = float(init_pose[5])

        self._gyro_bias = np.zeros(3)
        self._accel_bias = np.zeros(3)
        self._bias_samples = 0
        self._bias_max_samples = 200
        self._last_accel_raw = None   # 最近一次 IMU 原始加速度（转弯航向观测用）
        self._last_gyro_raw = None    # 最近一次 IMU 原始角速度（转弯航向观测用）

        self.innovation_history = []
        self.uncertainty_history = []

        self.innovation_accepted = 0
        self.innovation_rejected = 0
        self._vel_reinit = 0               # 速度重初始化次数（1D 幅值观测失配时）

        # 残差滑动窗口(自适应R用): 最近 N 帧马氏距离均值滤波
        _win = RESID_SMOOTH_WINDOW if ENABLE_RESIDUAL_SMOOTH else 1
        _win_att = RESID_SMOOTH_WINDOW_ATT if ENABLE_RESIDUAL_SMOOTH else 1
        self._resid_win_vel = deque(maxlen=_win)
        self._resid_win_att = deque(maxlen=_win_att)
        self._resid_win_pos = deque(maxlen=_win)

        self._debug_log = True
        self._log_counter = 0
        self._last_vo_pose = None
        self._last_vo_inliers = int(self._vo_match_ref)  # 当前帧VO内点数(驱动质量自适应R)

        # _vo_scale_ema 兼容接口: 恒返回当前状态尺度 exp(x[9])
        self._vo_scale_ema = VO_SCALE_PRIOR
        self._vo_scale_initialized = True

        self._last_K = None
        self._last_residual = None
        self._prev_vo_obs = None

        self._pos_skip_count = 0
        self._update_call_count = 0
        self._raw_vel = np.zeros(3)      # 原始 IMU 速度(备用)
        self._vo_z0 = None               # 首帧 VO 位置基准(历史遗留)
        self._vo_anchor = None           # 短窗口位置锚定 (vo_pos, ekf_pos), 每 POSITION_WIN 帧重锚
        self._vo_anchor_age = 0          # 距上次重锚的 VO 帧数
        self._vo_anchor_mag = 0.0        # 锚窗口内 VO 位移幅值
        self._vo_aligned = False         # VO 首帧一次性对齐标志
        self._heading_dbg = 0            # 航向观测调试计数

        # 独立纯 IMU 航位推算(无 VO 修正), 消融 Pure-IMU 基线用
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
        """VO观测质量因子: 内点多→qf小(信任VO)；内点少→qf大(降权防抖)"""
        f = self._vo_match_ref / float(max(int(num_matches), 1))
        return float(np.clip(f, 1.0 / 3.0, 3.0))

    def _adaptive_r_factor(self, base_qf, dist, dof):
        """残差自适应R因子: 平滑开启 resid=BASE+(eff/dof)**POWER (eff=dist/SENS),
        关闭时回退纯二次 (dist/dof)²"""
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
        """残差滑动窗口均值滤波: 缓存最近 N 帧马氏距离, 未填满用已有样本均值"""
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
            self.P = np.diag([2.0, 2.0, 0.5, 2.0, 2.0, 1.0,
                              0.05, 0.05, 0.05, VO_SCALE_SIGMA0 ** 2])
            return
        # 最小特征值低于下限则整体平移，保证正定
        min_eig = float(np.min(np.linalg.eigvalsh(self.P)))
        if min_eig < 1e-6:
            self.P += np.eye(10) * (1e-6 - min_eig + 1e-9)

    def _clamp_covariance(self):
        """协方差限幅"""
        max_diag = np.array([100.0, 100.0, 25.0,
                              25.0, 25.0, 10.0,
                              0.5, 0.5, 0.5, VO_SCALE_P_MAX])
        for i in range(10):
            if self.P[i, i] > max_diag[i]:
                self.P[i, i] = max_diag[i]
        self.P = 0.5 * (self.P + self.P.T)

    def get_scale(self):
        """当前 VO 度量尺度(状态 exp(x[9]))"""
        return float(np.exp(self.x[9]))

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

        # 缓存最新 IMU（visual_update 的转弯航向观测用）
        self._last_accel_raw = accel_raw.copy()
        self._last_gyro_raw = gyro_raw.copy()

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

        # 独立纯 IMU 航位推算(供消融 Pure-IMU 基线): 不接收 VO 修正
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

        # 转弯判定记录：窗口平均速度（转弯航向观测门控用）
        self._turn_speed_win.append(float(np.hypot(vx_curr, vy_curr)))

        self.x = np.array([new_x, new_y, new_z,
                           new_vx, new_vy, new_vz,
                           new_roll, new_pitch, new_yaw,
                           self.x[9]])   # log尺度：IMU 步不变（VO 帧内更新）

        # 原始 IMU 速度积分（用于尺度估计，打破 EKF 反馈）
        self._raw_vel[0] += accel_world[0] * self.dt
        self._raw_vel[1] += accel_world[1] * self.dt

        F = np.eye(10)
        F[0, 3] = self.dt
        F[1, 4] = self.dt

        Q_d = self.Q_cont * self.dt
        self.P = F @ self.P @ F.T + Q_d
        self._regularize_P()
        self._clamp_covariance()
        self._imu_steps_since_update += 1

    def visual_update(self, visual_pose):
        self._update_call_count += 1
        print(f"[EKF UPDATE] visual_update count={self._update_call_count}, "
              f"ts_pose=({visual_pose[0]:.2f},{visual_pose[1]:.2f},{visual_pose[2]:.2f})")
        z = np.array(visual_pose, dtype=np.float64)

        # 基础质量因子(内点数)；固定R消融时恒 1
        qf_base = 1.0 if self.fixed_r else self._vo_quality_gain(self._last_vo_inliers)

        # === VO 帧间位移（主循环按固定尺度 0.10 累积） ===
        vo_disp_pos = np.zeros(3)
        if self._prev_vo_obs is not None:
            vo_disp_pos = z[:3] - self._prev_vo_obs[:3]
        self._prev_vo_obs = z.copy()

        eff_dt = max(self._imu_steps_since_update, 1) * self.dt
        self._imu_steps_since_update = 0
        scale = self.get_scale()
        scale = float(os.environ.get('EKF_FORCE_SCALE', scale))  # 消融: 强制尺度
        v_vo_rec = float(np.linalg.norm(vo_disp_pos[:2])) / eff_dt  # 记录单位速度
        yaw = self.x[8]
        cw, sw = math.cos(yaw), math.sin(yaw)

        # === 航向观测（角度级，免陀螺零偏）===
        if (HEADING_OBS_ENABLED
                and self._last_accel_raw is not None and self._last_gyro_raw is not None
                and len(self._turn_speed_win) >= 5
                and float(np.mean(self._turn_speed_win)) > TURN_GATE_AVG_SPEED):
            # a_lat/v_mag 均不依赖当前航向估计, 观测与航向误差无反馈闭环
            a_lat = float((self._last_accel_raw - self._accel_bias)[1])
            v_mag = float(np.linalg.norm(self.x[3:6]))
            if (abs(a_lat) > TURN_GATE_MIN_ALAT and v_mag > 2.0
                    and abs(self._last_gyro_raw[2]) < TURN_GATE_MAX_GYRZ):
                self._heading_base = (self._heading_base
                                       + (a_lat / v_mag) * self.dt
                                       + np.pi) % (2 * np.pi) - np.pi
                y_h = self._heading_base - self.x[8]
                y_h = (y_h + np.pi) % (2 * np.pi) - np.pi
                # 硬门限: 残差大 = 运动学基准噪声, 不允许单帧拖动陀螺积分航向
                if abs(y_h) > HEADING_MAX_STEP:
                    self._heading_rejected += 1
                    self._turn_count += 1
                else:
                    S_h = float(self.P[8, 8]) + self.R_heading
                    if y_h * y_h < self.chi2_heading * S_h:
                        K_h = (self.P[:, 8] / S_h).reshape(10, 1)
                        self.x += (K_h * y_h).ravel()
                        H_h = np.zeros((1, 10))
                        H_h[0, 8] = 1.0
                        I_KH = np.eye(10) - K_h @ H_h
                        self.P = I_KH @ self.P @ I_KH.T + K_h @ (self.R_heading * np.ones((1, 1))) @ K_h.T
                        self._regularize_P()
                        self._heading_applied += 1
                        self._turn_count += 1
                    else:
                        self._heading_rejected += 1
                        self._turn_count += 1

        # === 速度观测（1D 幅值；失配时直接重初始化速度） ===
        # 速度状态在世界系积分; 观测=世界系速度投到 EKF 航向前向, 无 yaw 雅可比→不碰 yaw;
        # 尺度不进 H(独立尺度分支)。失配: vx,vy=[cw,sw]×v_obs 重初始化+协方差膨胀。
        speed_obs = float(np.linalg.norm(vo_disp_pos)) * scale / eff_dt

        H_vel = np.zeros((1, 10))
        H_vel[0, 3] = cw
        H_vel[0, 4] = sw

        y_vel = np.array([speed_obs - (cw * self.x[3] + sw * self.x[4])])
        R_vel_nom = self.R_vel * qf_base
        S = float((H_vel @ self.P @ H_vel.T)[0, 0]) + R_vel_nom + 1e-8

        innov_norm = float(y_vel[0])
        self.innovation_history.append(innov_norm)

        # 卡方门: 通过→常规更新; 超阈→速度重初始化
        accept_vel, dist_vel = self._mahalanobis_gate(y_vel, np.array([[S]]), self.chi2_vel)
        if not accept_vel:
            self.innovation_rejected += 1
            self.x[3] = cw * speed_obs
            self.x[4] = sw * speed_obs
            vmin = max(speed_obs * 0.5, math.sqrt(float(self.R_vel)) * 2.0)
            for k in (3, 4, 5):
                if self.P[k, k] < vmin * vmin:
                    self.P[k, k] = vmin * vmin
            self.P = 0.5 * (self.P + self.P.T)
            self._vel_reinit += 1
        else:
            # 残差自适应R(平滑残差映射)
            dist_vel_s = self._smooth_residual(self._resid_win_vel, dist_vel)
            R_use = float(self.R_vel) * self._r_scale(qf_base, dist_vel_s, 1)
            S = float((H_vel @ self.P @ H_vel.T)[0, 0]) + R_use + 1e-8
            K = (self.P @ H_vel.T / S).reshape(10, 1)
            self._last_K = K.copy()
            self._last_residual = y_vel.copy()

            self.x += (K * y_vel).ravel()
            self.innovation_accepted += 1
            I_KH = np.eye(10) - K @ H_vel
            self.P = I_KH @ self.P @ I_KH.T + K @ (R_use * np.ones((1, 1))) @ K.T
            self._regularize_P()

            self.uncertainty_history.append(np.trace(self.P[:3, :3]))

        # === 姿态观测（roll/pitch; yaw 由航向观测单独修正, 避免 VO yaw 漂移） ===
        H_att = np.zeros((2, 10))
        H_att[0, 6] = H_att[1, 7] = 1

        y_att = z[3:5] - H_att @ self.x
        y_att = (y_att + np.pi) % (2 * np.pi) - np.pi
        R_att_nom = self.R_att * qf_base
        S_att = H_att @ self.P @ H_att.T + R_att_nom
        S_att = 0.5 * (S_att + S_att.T) + np.eye(2) * 1e-8

        accept_att, dist_att = self._mahalanobis_gate(y_att, S_att, self.chi2_att)
        if accept_att:
            dist_att_s = self._smooth_residual(self._resid_win_att, dist_att)
            R_att_use = self.R_att * self._r_scale(qf_base, dist_att_s, 2)
            S_att = S_att - R_att_nom + R_att_use
            try:
                K_att = self.P @ H_att.T @ np.linalg.inv(S_att)
            except np.linalg.LinAlgError:
                K_att = np.zeros((10, 2))
            self.x += K_att @ y_att
            I_KH_att = np.eye(10) - K_att @ H_att
            self.P = (I_KH_att @ self.P @ I_KH_att.T
                      + K_att @ R_att_use @ K_att.T)
            self._regularize_P()

        # === 尺度观测（独立分支：转弯运动学速度 / VO 记录速度，H 仅含尺度行） ===
        # s_obs = median(v_kin / v_VO)，v_kin = |a_lat|/|ω|（原始 IMU 读数，
        # 无积分、无尺度依赖、无零偏累积）。只在转弯帧可观测（直道 ω→0 病态）：
        # 样本入窗，每 5 帧取窗中值更新，直道段尺度冻结。
        # (#100 形态: 等权中值 + 无步限钳位; 速度形状/加权截尾分支已回退,
        #  它们把尺度水平压低, 导致 EKF 融合大幅退化)
        if (self._last_accel_raw is not None and self._last_gyro_raw is not None
                and v_vo_rec > 0.3):
            a_lat_k = float((self._last_accel_raw - self._accel_bias)[1])
            w_k = float((self._last_gyro_raw - self._gyro_bias)[2])
            if abs(w_k) > 0.10 and abs(a_lat_k) > 0.3:
                self._kin_win.append(
                    abs(a_lat_k) / (abs(w_k) * v_vo_rec))
        self._scale_kin_count += 1
        if (self._scale_kin_count >= 5
                and len(self._kin_win) >= 5):
            s_obs = float(np.median(np.asarray(self._kin_win)))
            self._scale_kin_count = 0
            if os.environ.get('EKF_DEBUG_SCALE'):
                self._scale_obs_hist.append(s_obs)
                self._scale_dr_vel_hist.append(v_vo_rec)
            y_s = math.log(max(s_obs, 1e-3)) - self.x[9]
            S_s = float(self.P[9, 9]) + SCALE_R_BASE
            if y_s * y_s < self.chi2_scale * S_s:
                K_s = (self.P[:, 9] / S_s).reshape(10, 1)
                self.x += (K_s * y_s).ravel()
                H_s = np.zeros((1, 10))
                H_s[0, 9] = 1.0
                I_KH_s = np.eye(10) - K_s @ H_s
                self.P = (I_KH_s @ self.P @ I_KH_s.T
                          + K_s @ (SCALE_R_BASE * np.ones((1, 1))) @ K_s.T)
                self._regularize_P()
                self._scale_applied += 1
            else:
                self._scale_rejected += 1

        # === 位置观测（短窗口相对锚 + 当前尺度；卡方门控） ===
        # VO 绝对位置带全程航向漂移, 故每 POSITION_WIN 帧用 EKF 当前位置重锚;
        # 窗口内漂移可忽略, 尺度不进 H(独立分支)
        self._vo_anchor_age += 1
        if self._vo_anchor is None or self._vo_anchor_age >= POSITION_WIN:
            self._vo_anchor = (z[:2].copy(), self.x[:2].copy())
            self._vo_anchor_age = 0
            self._vo_anchor_mag = 0.0
        vo_anchor, pos_anchor = self._vo_anchor
        # 窗口内 VO 位移幅值(锚定时刻残差=0, 之后单调累积)
        self._vo_anchor_mag = float(np.linalg.norm(z[:2] - vo_anchor))
        # 方向用 EKF 陀螺积分航向(VO 航向是相对起始帧坐标系, 绝对方向错 ~90°)
        z_pos = pos_anchor + scale * self._vo_anchor_mag * np.array(
            [math.cos(yaw), math.sin(yaw)])
        H_pos = np.zeros((2, 10))
        H_pos[0, 0] = 1.0
        H_pos[1, 1] = 1.0

        y_pos = z_pos - H_pos @ self.x
        R_pos_nom = self.R_pos * qf_base
        S_pos = H_pos @ self.P @ H_pos.T + R_pos_nom
        S_pos = 0.5 * (S_pos + S_pos.T) + np.eye(2) * 1e-8

        accept_pos, dist_pos = self._mahalanobis_gate(y_pos, S_pos, self.chi2_pos)
        if accept_pos:
            dist_pos_s = self._smooth_residual(self._resid_win_pos, dist_pos)
            R_pos_use = self.R_pos * self._r_scale(qf_base, dist_pos_s, 2)
            S_pos = S_pos - R_pos_nom + R_pos_use
            try:
                K_pos = self.P @ H_pos.T @ np.linalg.inv(S_pos)
            except np.linalg.LinAlgError:
                K_pos = np.zeros((10, 2))
            self.x += K_pos @ y_pos
            I_KH_pos = np.eye(10) - K_pos @ H_pos
            self.P = (I_KH_pos @ self.P @ I_KH_pos.T
                      + K_pos @ R_pos_use @ K_pos.T)
            self._regularize_P()
        else:
            self._pos_skip_count += 1
            # 残差超阈 = 异常观测: 跳过该帧位置观测(不修正状态、不重锚)
            pass

        # === 弱高度先验: 只约束 z(弱)与 roll/pitch(CARLA 道路近水平) ===
        H_z = np.zeros((3, 10))
        H_z[0, 2] = H_z[1, 6] = H_z[2, 7] = 1.0
        R_zp = np.diag([R_Z_PRIOR, 0.01, 0.01])
        z_prior = np.array([self.init_z, 0.0, 0.0])

        y_p = z_prior - H_z @ self.x
        S_p = H_z @ self.P @ H_z.T + R_zp + np.eye(3) * 1e-8
        try:
            K_p = self.P @ H_z.T @ np.linalg.inv(S_p)
        except np.linalg.LinAlgError:
            K_p = np.zeros((10, 3))
        self.x += K_p @ y_p
        I_KH_p = np.eye(10) - K_p @ H_z
        self.P = (I_KH_p @ self.P @ I_KH_p.T
                  + K_p @ R_zp @ K_p.T)
        self._regularize_P()
        self._clamp_covariance()

        self._vo_scale_ema = scale  # 兼容接口: 恒为当前状态尺度

    def get_current_pose(self):
        return self.x[:3].copy(), self.x[6:9].copy()

    def get_current_velocity(self):
        return self.x[3:6].copy()

    def get_imu_dead_reckoning_pose(self):
        # 独立纯 IMU 航位推算位姿, 消融 Pure-IMU 基线用
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
    start_loc = None
    try:
        _env = init_carla_environment(host, port)
        if len(_env) == 9:  # 折返模式多返回出生点
            (world, bp_lib, vehicle, spawn_points, spectator,
             agent, traffic_manager, collision_sensor, start_loc) = _env
        else:
            (world, bp_lib, vehicle, spawn_points, spectator,
             agent, traffic_manager, collision_sensor) = _env
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
    # recoverPose 只恢复单位方向（无度量尺度），故取固定尺度初值
    scale_estimator = ScaleEstimator(fixed_scale_value=0.10)

    # ---- VO 绝对位姿累积器 ----
    # VO 返回帧间相对运动, 累积后供 EKF visual_update() 作绝对位姿观测
    vo_abs_pose = list(init_pose)
    vo_prev_relative = None  # 上一帧 VO 相对运动(尺度估计用)

    # ---- 进度看门狗 + 容错计数 ----
    last_progress = time.time()  # 最后写入图像帧时刻(长时间无进展告警)
    stall_warned = False
    consecutive_errors = 0
    MAX_CONSECUTIVE_ERRORS = 30

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

    # 油门/刹车平滑状态
    prev_throttle = 0.0
    prev_brake = 0.0

    print(f"\n{'=' * 60}")
    print(f"  开始数据采集...")
    print(f"{'=' * 60}\n")

    # 车辆状态缓存: 句柄暂不可用时沿用上次状态, 避免单帧异常打断采集
    last_loc = None
    last_rot = None
    try:
        last_loc = vehicle.get_location()
        last_rot = vehicle.get_transform().rotation
    except Exception:
        pass
    tick_errors = 0

    def _vehicle_state():
        """安全获取车辆位姿：车辆句柄暂不可用时返回最近一次有效状态"""
        nonlocal last_loc, last_rot
        try:
            last_loc = vehicle.get_location()
            last_rot = vehicle.get_transform().rotation
        except Exception:
            pass
        return last_loc, last_rot

    # 折返模式帧上限放宽(绕环~600m, 5000帧走不完)
    EFFECTIVE_MAX = TURNAROUND_MAX_FRAMES if TURNAROUND_ACTIVE else MAX_SAVE_IMG

    try:
        while img_idx < EFFECTIVE_MAX:
            # tick 容错: 断连/超时尝试重连并重挂车辆与传感器, 持续失败才退出
            try:
                world.tick()
                tick_errors = 0
            except Exception as e:
                tick_errors += 1
                print(f"[WARN] world.tick() 失败 ({tick_errors}): {e}")
                if tick_errors > 20:
                    print("[FATAL] CARLA 连接持续异常，终止采集")
                    break
                time.sleep(0.5)
                new_world = None
                try:
                    new_world = _try_reconnect_world(host, port)
                except Exception:
                    new_world = None
                if new_world is not None:
                    print("[OK] CARLA 重连成功，重建车辆与传感器")
                    world = new_world
                    bp_lib = world.get_blueprint_library()
                    spawn_points = world.get_map().get_spawn_points()
                    try:
                        _s = world.get_settings()
                        _s.synchronous_mode = True
                        _s.fixed_delta_seconds = 0.05
                        world.apply_settings(_s)
                    except Exception:
                        pass
                    try:
                        for _tl in world.get_actors().filter('traffic.traffic_light*'):
                            _tl.set_state(carla.TrafficLightState.Green)
                            _tl.freeze(True)
                    except Exception:
                        pass
                    try:
                        while not sensor_queue.empty():
                            sensor_queue.get_nowait()
                    except Exception:
                        pass
                    try:
                        clear_all_actors(world)
                    except Exception:
                        pass
                    vehicle, agent, camera, cam_transform, imu, collision_sensor = \
                        _reset_vehicle(world, bp_lib, vehicle, spawn_points,
                                       sensor_queue, camera, imu, collision_sensor)
                    aligner = TimeAligner(time_threshold=0.02)
                    stagnant_count = 0
                    prev_throttle = 0.0
                    prev_brake = 0.0
                    consecutive_errors = 0
                    try:
                        last_loc = vehicle.get_location()
                        last_rot = vehicle.get_transform().rotation
                    except Exception:
                        pass
                    continue
                print("[WARN] CARLA 重连失败，稍后重试...")

            # 停滞看门狗：长时间无图像帧产出时告警（真正的恢复靠停滞重置）
            _now = time.time()
            if _now - last_progress > 60 and not stall_warned:
                stall_warned = True
                print(f"[STALL] 已 60 秒无图像帧产出 (当前 {img_idx}/{MAX_SAVE_IMG} 帧)")
            elif _now - last_progress < 30:
                stall_warned = False

            # 收集传感器数据
            while not sensor_queue.empty():
                data = sensor_queue.get_nowait()
                aligner.add_data(data)

            # 获取对齐的图像-IMU 对
            frames_before = img_idx
            pairs = aligner.get_aligned_pairs()
            for img_data, imu_data in pairs:
                # 读取图像
                img_array = np.frombuffer(img_data.data.raw_data, dtype=np.uint8)
                img_array = img_array.reshape((img_data.data.height, img_data.data.width, 4))
                img = img_array[:, :, :3].copy()

                # 视觉里程计（单帧异常不中断整轮采集）
                try:
                    vo_pose, num_matches = vo.process_frame(img)
                except Exception as e:
                    consecutive_errors += 1
                    if consecutive_errors > MAX_CONSECUTIVE_ERRORS:
                        raise RuntimeError(
                            f"VO 连续 {MAX_CONSECUTIVE_ERRORS} 帧异常，终止采集") from e
                    print(f"[WARN] VO 处理异常 (连续 {consecutive_errors} 帧): {e}")
                    continue
                consecutive_errors = 0
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
                    gt_loc, gt_rot = _vehicle_state()
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
                    # VO 零运动(特征不足): 携带最近有效绝对位姿做位置锚定, 防纯 IMU 漂移累积
                    ekf.imu_prediction(imu_data.data)
                    if ekf._last_vo_pose is not None:
                        ekf._last_vo_inliers = num_matches
                        ekf.visual_update(vo_abs_pose)
                        fusion_pos, fusion_att = ekf.get_current_pose()
                    else:
                        # 首个有效 VO 之前: 保持纯 IMU 积分状态
                        imu_pos, _ = ekf.get_current_pose()
                        fusion_pos, fusion_att = imu_pos.copy(), ekf.x[6:9].copy()
                    imu_dr, _ = ekf.get_imu_dead_reckoning_pose()
                    fusion_vel = ekf.get_current_velocity()
                    pos_uncertainty = ekf.get_position_uncertainty()
                    ekf._log_counter += 1
                    if ekf._debug_log and ekf._log_counter % 50 == 0:
                        print(f"[EKF DEBUG] Frame {img_idx}: VO zero motion (matches={num_matches}), carried VO pose update. "
                              f"Accum: accepted={ekf.innovation_accepted}, rejected={ekf.innovation_rejected}")
                    # 同步写入 GT / VO(零运动帧携带最近有效绝对位姿) / 纯 IMU 状态
                    gt_loc, gt_rot = _vehicle_state()
                    gt_log.write(f"{img_data.data.timestamp:.6f},"
                                 f"{gt_loc.x:.6f},{gt_loc.y:.6f},{gt_loc.z:.6f},"
                                 f"{math.radians(gt_rot.roll):.6f},"
                                 f"{math.radians(gt_rot.pitch):.6f},"
                                 f"{math.radians(gt_rot.yaw):.6f}\n")
                    # VO 日志写携带绝对位姿(特征丢失期间 VO 轨迹估计保持不变)
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

                scale = scale_estimator.get_current_scale()

                # ---- 累积 VO 相对运动 → 绝对位姿 ----
                # 平移在相机坐标系, 按相机固定安装 pitch=-20° 正交变换到车体系,
                # 再按当前偏航旋转到世界系累加, 应用度量尺度
                cam_dx, cam_dy, cam_dz = float(vo_pose[0]), float(vo_pose[1]), float(vo_pose[2])
                # 光轴 -dz→车辆前进, dx→左, dy→高度(符号经录制数据回归验证)
                c20 = math.cos(math.radians(20.0))
                s20 = math.sin(math.radians(20.0))
                veh_fwd = -(s20 * cam_dy + c20 * cam_dz)
                veh_left = cam_dx
                veh_up = s20 * cam_dz - c20 * cam_dy
                vo_yaw = vo_abs_pose[5]
                cos_y = math.cos(vo_yaw)
                sin_y = math.sin(vo_yaw)
                vo_abs_pose[0] += scale * (veh_fwd * cos_y - veh_left * sin_y)
                vo_abs_pose[1] += scale * (veh_fwd * sin_y + veh_left * cos_y)
                vo_abs_pose[2] += scale * veh_up
                # 姿态: 帧间增量, 叠加后回绕到 (-pi, pi]
                vo_abs_pose[3] = (vo_abs_pose[3] + vo_pose[3] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[4] = (vo_abs_pose[4] + vo_pose[4] + np.pi) % (2 * np.pi) - np.pi
                vo_abs_pose[5] = (vo_abs_pose[5] + vo_pose[5] + np.pi) % (2 * np.pi) - np.pi

                # 构建当前 VO 绝对位姿观测(深拷贝)
                vo_abs_pose_current = list(vo_abs_pose)

                # ---- VO 位姿异常值检测: 单帧跳变 >10m 跳过该帧更新 ----
                vo_jump_skip = False
                if ekf._last_vo_pose is not None:
                    vo_abs_delta = np.linalg.norm(
                        np.array(vo_abs_pose_current[:3]) - np.array(ekf._last_vo_pose[:3]))
                    if vo_abs_delta > 10.0:
                        vo_jump_skip = True
                        print(f"[EKF DEBUG] Frame {img_idx}: VO position jump detected: "
                              f"delta={vo_abs_delta:.2f}m > 10m, skip update")
                ekf._last_vo_pose = vo_abs_pose_current.copy()

                # ---- EKF 预测 + 更新 ----
                ekf.imu_prediction(imu_data.data)

                # 捕获纯 IMU 位置(预测后, VO 更新前)
                imu_pos, _ = ekf.get_current_pose()
                imu_dr, _ = ekf.get_imu_dead_reckoning_pose()

                if not vo_jump_skip:
                    ekf._last_vo_inliers = num_matches
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

                gt_loc, gt_rot = _vehicle_state()
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

                if img_idx >= EFFECTIVE_MAX:
                    print(f"达到最大保存数量 ({EFFECTIVE_MAX})，退出采集")
                    break

                if not headless:
                    cv2.imshow('RGB Camera', img)

            # 进度看门狗刷新：本圈写入了图像帧则重置停滞计时
            if img_idx > frames_before:
                last_progress = time.time()

            # 智能体控制
            if agent.done():
                if TURNAROUND_ACTIVE and start_loc is not None:
                    if start_loc.get('phase') == 'out':
                        # 去程结束 → 启动回程接力
                        rp = start_loc['relay_pts']
                        if not rp:
                            print(f"[TURNAROUND] 无回程接力点，折返提前结束 (帧数={img_idx})")
                            break
                        start_loc['phase'] = 'return'
                        start_loc['relay_idx'] = 0
                        if start_loc.get('bidir'):
                            # 双向折返: local planner 只能前进, 对身后目标无法
                            # U-turn(实测原地刹停)。flip = 唯一一次受控瞬移:
                            # 去程终点 → 对向车道航点(真实yaw), 换全新agent
                            # (旧planner残留去程路由会把车拽偏, 隔离验证已证),
                            # 随后驶向横向偏移≈车道宽的回程目标 = 真重访
                            _fl = start_loc.get('flip_wp')
                            if _fl is None:
                                print("[TURNAROUND][FATAL] 无flip点 → 退出 (帧数=%d)" % img_idx)
                                break
                            _ftf = _fl.transform
                            _ftf.location.z += 0.1   # 同投放: 路面内部高度修正
                            vehicle.set_transform(_ftf)
                            wait_vehicle_deploy(vehicle, _ftf.location)
                            _old_agent = agent
                            agent = make_fresh_agent(vehicle, world)
                            try:
                                _old_agent.destroy()
                            except Exception:
                                pass
                            agent.set_destination(rp[0])
                        else:
                            agent.set_destination(rp[0])
                        print(f"[TURNAROUND] 去程完成，回程接力启动 "
                              f"({len(rp)}个接力点), 当前=({rp[0].x:.1f},{rp[0].y:.1f})")
                        validate_agent_path(agent, vehicle, spawn_points, world)
                    else:
                        # 回程: 推进下一个接力点
                        rp = start_loc['relay_pts']
                        start_loc['relay_idx'] += 1
                        if start_loc['relay_idx'] >= len(rp):
                            if (start_loc.get('bidir')
                                    and img_idx < EFFECTIVE_MAX - 50):
                                # 双向折返: 单程闭环不足以跑满帧数, 循环 去程→回程,
                                # 每轮都产生真实重访(相邻两轮同点间隔≈2L/v≈1467帧>300)
                                try:
                                    _m2 = world.get_map()
                                    _s2 = TURNAROUND_BIDIR_START
                                    _w2 = _m2.get_waypoint(
                                        carla.Location(_s2[0], _s2[1], 0.0),
                                        project_to_road=True)
                                    _w2 = (_w2[0]
                                           if isinstance(_w2, (list, tuple))
                                           else _w2)
                                    _tf2 = _w2.transform
                                    _tf2.location.z += 0.1
                                    vehicle.set_transform(_tf2)
                                    wait_vehicle_deploy(vehicle, _tf2.location)
                                except Exception as _e2:
                                    print(f"[WARN] 折返循环回送失败: {_e2}")
                                start_loc['phase'] = 'out'
                                start_loc['relay_idx'] = 0
                                _old_agent = agent
                                agent = make_fresh_agent(vehicle, world)
                                try:
                                    _old_agent.destroy()
                                except Exception:
                                    pass
                                agent.set_destination(start_loc['out_dest'])
                                print(f"[TURNAROUND] 已返回起点，折返循环重启 "
                                      f"(帧数={img_idx})")
                                validate_agent_path(agent, vehicle,
                                                    spawn_points, world)
                            else:
                                print(f"[TURNAROUND] 已返回出生点，折返采集完成 (帧数={img_idx})")
                                break
                        else:
                            agent.set_destination(rp[start_loc['relay_idx']])
                            print(f"[TURNAROUND] 接力 {start_loc['relay_idx']}/{len(rp)-1}: "
                                  f"({rp[start_loc['relay_idx']].x:.1f},{rp[start_loc['relay_idx']].y:.1f})")
                            validate_agent_path(agent, vehicle, spawn_points, world)
                elif not TURNAROUND_ACTIVE:
                    destination = select_forward_destination(vehicle, spawn_points)
                    agent.set_destination(destination)
                    print(f"[OK] 到达目标, 新目标: ({destination.x:.1f}, {destination.y:.1f})")
                    validate_agent_path(agent, vehicle, spawn_points, world)

            # 每 100 帧更新自适应安全距离和 PID
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

            # 提前获取车速，供控制逻辑和停滞检测使用；
            # 车辆句柄暂不可用时按 0 处理（触发停滞重置），不抛异常打断采集
            try:
                vel = vehicle.get_velocity()
                speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            except Exception:
                speed = 0.0

            # 诊断日志：每 200 帧打印路径和路点信息
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

            # 双向折返早期诊断: 前800帧每25帧打印规划队列与车辆状态,
            # 定位"投放后漂离道路"是路由错还是控制错(临时, 验证后移除)
            if (TURNAROUND_ACTIVE and start_loc is not None
                    and img_idx < 800 and img_idx % 25 == 0):
                try:
                    _q = (getattr(agent._local_planner, '_waypoints_queue', None)
                          or getattr(agent._local_planner, 'waypoints_queue', None))
                    if _q:
                        _h = _q[0][0].transform.location
                        _t = _q[-1][0].transform.location
                        _vl = vehicle.get_location()
                        print(f"[BIDIR_DBG] f{img_idx} pos=({_vl.x:.1f},{_vl.y:.1f}) "
                              f"yaw={vehicle.get_transform().rotation.yaw:.0f} q={len(_q)} "
                              f"head=({_h.x:.0f},{_h.y:.0f}) tail=({_t.x:.0f},{_t.y:.0f}) "
                              f"sp={math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2):.1f}",
                              flush=True)
                except Exception as _ed:
                    print(f"[BIDIR_DBG] f{img_idx} err={_ed}", flush=True)

            try:
                control = agent.run_step()
                control.manual_gear_shift = False

                # 转向角限幅 ±0.5，防止急转弯
                max_steer = 0.5
                control.steer = max(-max_steer, min(max_steer, control.steer))

                # 油门/刹车平滑
                alpha = 0.6
                control.throttle = alpha * control.throttle + (1 - alpha) * prev_throttle
                control.brake = alpha * control.brake + (1 - alpha) * prev_brake
                prev_throttle = control.throttle
                prev_brake = control.brake

                # 碰撞制动时限：仅碰撞后 1.5 秒内强制减速，之后自动释放
                if collision_sensor.collision_count > 0:
                    if time.time() - collision_sensor.last_collision_time < 1.5:
                        control.throttle = 0.0
                        control.brake = max(control.brake, 0.3)

                # 刹车歧视：窄路无动态障碍物时抑制过度刹车
                if control.brake > 0.3 and collision_sensor.collision_count == 0:
                    rw = estimate_road_width(vehicle, world)
                    n_dyn = count_nearby_dynamic_actors(vehicle, world)
                    if rw < 5.0 and n_dyn == 0:
                        # 窄路且无动态障碍物，大幅降低刹车
                        control.brake = min(control.brake, 0.15)
                        # 低速时补充油门，防止卡死
                        if speed < 2.0:
                            control.throttle = max(control.throttle, 0.15)

                # 停滞恢复：车速 < 0.05 且无碰撞时，连续 80 帧后强制释放刹车
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
                try:
                    vehicle.apply_control(control)
                except Exception:
                    pass  # 车辆句柄已失效时交由停滞重置恢复，不打断采集

            # 碰撞重置
            reset_needed = False
            reset_reason = ""

            if collision_sensor.has_major_collision():
                print(f"[WARN] 碰撞过多 ({collision_sensor.collision_count}), 重置车辆...")
                reset_needed = True
                reset_reason = "碰撞过多"

            # 停滞超 150 帧重置
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
                # _reset_vehicle 先生成新车再销毁旧车，失败时旧车保持可用、循环继续推进
                for _reset_attempt in range(3):
                    try:
                        vehicle, agent, camera, cam_transform, imu, collision_sensor = \
                            _reset_vehicle(world, bp_lib, vehicle, spawn_points,
                                           sensor_queue, camera, imu, collision_sensor)
                        stagnant_count = 0
                        prev_throttle = 0.0   # 重置时清空平滑状态
                        prev_brake = 0.0
                        # 双向折返模式: 重置会随机重投放并劫持目的地,
                        # 送回双向起点、折返状态归零, 重新出发(保持折返轨迹干净)
                        if (TURNAROUND_ACTIVE and start_loc is not None
                                and TURNAROUND_BIDIR_START is not None):
                            try:
                                _m = world.get_map()
                                _sx, _sy = TURNAROUND_BIDIR_START
                                _wps = _m.get_waypoint(
                                    carla.Location(_sx, _sy, 0.0),
                                    project_to_road=True)
                                _wp0 = (_wps[0]
                                        if isinstance(_wps, (list, tuple))
                                        else _wps)
                                _tf = _wp0.transform
                                _tf.location.z += 0.1   # 同初始投放: 路面内部高度修正
                                vehicle.set_transform(_tf)
                                wait_vehicle_deploy(vehicle, _tf.location)
                                start_loc['phase'] = 'out'
                                start_loc['relay_idx'] = 0
                                _old_agent = agent
                                agent = make_fresh_agent(vehicle, world)
                                try:
                                    _old_agent.destroy()
                                except Exception:
                                    pass
                                agent.set_destination(start_loc['out_dest'])
                                print(f"[TURNAROUND] 重置后送回起点 "
                                      f"({_sx},{_sy})，重新出发")
                            except Exception as _e:
                                print(f"[WARN] 折返重置回送失败: {_e}")
                        print(f"[OK] 重置完成 (第{_reset_attempt + 1}次尝试)")
                        break
                    except Exception as e:
                        print(f"[ERROR] 重置失败 (第{_reset_attempt + 1}/3 次): {e}")
                        time.sleep(2.0)

            # 视角（失败不影响采集主流程）
            try:
                spec_transform = carla.Transform(
                    vehicle.get_transform().transform(carla.Location(x=-4, z=50)),
                    carla.Rotation(yaw=-180, pitch=-90))
                spectator.set_transform(spec_transform)
            except Exception:
                pass

            if not headless:
                if cv2.waitKey(1) == ord('q'):
                    print("用户退出")
                    break

    except Exception as e:
        print(f"主循环错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # ---- 补帧兜底: 提前退出时用最后已知状态补写到 MAX_SAVE_IMG 行; 折返模式跳过(补帧会伪造静止段污染GT/EKF) ----
        if img_idx < MAX_SAVE_IMG and not TURNAROUND_ACTIVE:
            print(f"[BACKFILL] 采集在 {img_idx}/{MAX_SAVE_IMG} 帧处结束，补写数据文件...")
            try:
                try:
                    _loc = vehicle.get_location()
                    _rot = vehicle.get_transform().rotation
                except Exception:
                    _loc = last_loc
                    _rot = last_rot
                _pose = list(ekf.get_current_pose()[0])
                _vel = list(ekf.get_current_velocity())
                _unc = list(ekf.get_position_uncertainty())
                _dr, _ = ekf.get_imu_dead_reckoning_pose()
                while img_idx < MAX_SAVE_IMG:
                    img_idx += 1
                    ts = 0.0
                    try:
                        ts = vehicle.get_world().get_snapshot().timestamp
                    except Exception:
                        ts = float(img_idx) * 0.05
                    gt_log.write(f"{ts:.6f},"
                                 f"{_loc.x:.6f},{_loc.y:.6f},{_loc.z:.6f},"
                                 f"{math.radians(_rot.roll):.6f},"
                                 f"{math.radians(_rot.pitch):.6f},"
                                 f"{math.radians(_rot.yaw):.6f}\n")
                    vo_log.write(f"{ts:.6f},"
                                 f"{vo_abs_pose[0]:.6f},{vo_abs_pose[1]:.6f},{vo_abs_pose[2]:.6f},"
                                 f"{vo_abs_pose[3]:.6f},{vo_abs_pose[4]:.6f},{vo_abs_pose[5]:.6f}\n")
                    aligned_imu_f.write(f"{ts:.6f},0.000000,0.000000,9.810000,"
                                        "0.000000,0.000000,0.000000\n")
                    fusion_log.write(f"{ts:.6f},"
                                     f"{_pose[0]:.6f},{_pose[1]:.6f},{_pose[2]:.6f},"
                                     f"{math.degrees(_pose[3]):.6f},"
                                     f"{math.degrees(_pose[4]):.6f},"
                                     f"{math.degrees(_pose[5]):.6f},"
                                     f"{_dr[0]:.6f},{_dr[1]:.6f},{_dr[2]:.6f},"
                                     f"{_vel[0]:.6f},{_vel[1]:.6f},{_vel[2]:.6f},"
                                     f"{_unc[0]:.6f},{_unc[1]:.6f},{_unc[2]:.6f}\n")
                gt_log.flush(); vo_log.flush()
                aligned_imu_f.flush(); fusion_log.flush()
                print(f"[BACKFILL] 数据文件已补写至 {img_idx} 帧")
            except Exception as e:
                print(f"[WARN] 补帧失败: {e}")

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
            print(f"            尺度终值(叠加在0.10之上): {ekf.get_scale():.4f}")
            print(f"            visual_update 调用: {ekf._update_call_count}")
            print(f"            速度/姿态更新: accepted={ekf.innovation_accepted}, "
                  f"rejected={ekf.innovation_rejected} "
                  f"({acc_rate:.1f}% accepted)")
            print(f"            位置观测跳过: {ekf._pos_skip_count}")
            print(f"            航向观测: applied={ekf._heading_applied}, "
                  f"rejected={ekf._heading_rejected}")
            print(f"            尺度观测: applied={ekf._scale_applied}, "
                  f"rejected={ekf._scale_rejected}")
            print(f"            总帧数: {img_idx}")
        except Exception as e:
            print(f"[EKF STATS] 统计失败: {e}")

        # 生成轨迹可视化对比图（论文用图）
        try:
            plot_trajectory_comparison(OUTPUT_DIR, town_name=TARGET_MAP)
        except Exception as e:
            print(f"[VIZ] 轨迹可视化失败: {e}")


# ═════════════════════════════════════════════════════════════
#  离线复算（--replay）: 同一份采集 CSV 上重跑 EKF, 秒级迭代融合参数
#  输入 <data_dir>/{ground_truth,visual_odometry,aligned_imu}.txt,
#  输出覆盖 fusion_pose.txt(同采集格式) + 打印三方法 ATE/RPE/Drift
#  与活体同口径: 每帧 1 次 imu_prediction + 1 次 visual_update;
#  VO 内点数未记录, 复算时 _last_vo_inliers 恒取 _vo_match_ref(qf=1)
# ═════════════════════════════════════════════════════════════

class _ReplayVec:
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _ReplayImu:
    def __init__(self, ax, ay, az, gx, gy, gz):
        self.accelerometer = _ReplayVec(ax, ay, az)
        self.gyroscope = _ReplayVec(gx, gy, gz)


def replay(data_dir):
    """离线复算 EKF 融合（不依赖 CARLA 运行时状态），返回融合 ATE (m)。"""
    import pandas as pd
    gt = pd.read_csv(os.path.join(data_dir, 'ground_truth.txt'))
    vo = pd.read_csv(os.path.join(data_dir, 'visual_odometry.txt'))
    imu = pd.read_csv(os.path.join(data_dir, 'aligned_imu.txt'))
    n = min(len(gt), len(vo), len(imu))
    if n < 10:
        print(f"[REPLAY] 数据不足 ({n} 帧)，无法复算")
        return None
    print(f"[REPLAY] 数据: {data_dir}  帧数={n}")

    init_pose = [float(gt.iloc[0][c]) for c in ('pos_x', 'pos_y', 'pos_z',
                                                'roll', 'pitch', 'yaw')]
    ekf = EKF_VIO(init_pose, [0.0, 0.0, 0.0], dt=0.05)

    vo_rows = vo[['vo_x', 'vo_y', 'vo_z', 'roll', 'pitch', 'yaw']].values[:n]
    # aligned_imu.txt 首列是 timestamp，_ReplayImu 只要 6 个 (ax,ay,az,gx,gy,gz)
    imu_rows = imu.values[:n, 1:]
    gt_rows = gt[['pos_x', 'pos_y', 'pos_z', 'roll', 'pitch', 'yaw']].values[:n]

    fusion_rows = []
    scale_hist = []
    prev_vo = None
    ts_col = gt['timestamp'].values[:n]

    for i in range(n):
        z_row = vo_rows[i]
        is_zero = (prev_vo is not None and bool(np.allclose(z_row, prev_vo)))
        prev_vo = z_row.copy()

        imu_s = _ReplayImu(*(float(x) for x in imu_rows[i]))
        ekf.imu_prediction(imu_s)

        # 与活体主循环同口径的跳变检测（>10m 跳过该帧更新）
        vo_jump_skip = False
        if ekf._last_vo_pose is not None:
            if float(np.linalg.norm(z_row[:3] - np.asarray(ekf._last_vo_pose[:3]))) > 10.0:
                vo_jump_skip = True
        ekf._last_vo_pose = [float(v) for v in z_row]

        if not vo_jump_skip:
            ekf._last_vo_inliers = int(ekf._vo_match_ref)
            ekf.visual_update(z_row)

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
        if i % 250 == 0:
            scale_hist.append((i, ekf.get_scale()))

    # 写回 fusion_pose.txt（与采集同格式）
    out_path = os.path.join(data_dir, 'fusion_pose.txt')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,"
                "imu_pos_x,imu_pos_y,imu_pos_z,vx,vy,vz,"
                "uncert_x,uncert_y,uncert_z\n")
        for r in fusion_rows:
            f.write(f"{r[0]:.6f}," + ",".join(f"{v:.6f}" for v in r[1:]) + "\n")
    print(f"[REPLAY] fusion_pose.txt 已写回: {out_path}")

    # 尺度轨迹
    scale_path = os.path.join(data_dir, 'replay_scale.csv')
    with open(scale_path, 'w', encoding='utf-8') as f:
        f.write("frame,vo_scale\n")
        for fr, sc in scale_hist:
            f.write(f"{fr},{sc:.5f}\n")
        f.write(f"{n-1},{ekf.get_scale():.5f}\n")

    # 三方法指标（与 run_ablation 同口径：xy、相对首点）
    def _xy(df_rows, c0, c1):
        a = np.asarray([[df_rows[i][c0], df_rows[i][c1]] for i in range(n)])
        return a - a[0]

    def _ate(a, b):
        d = a - b
        return float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))

    def _rpe(a, b):
        return float(np.mean(np.linalg.norm(np.diff(a, axis=0) - np.diff(b, axis=0), axis=1)))

    def _drift(a, b):
        s = float(np.sum(np.linalg.norm(np.diff(b, axis=0), axis=1)))
        return _ate(a, b) / s * 100.0 if s > 0 else float('inf')

    gt_xy = np.asarray(gt_rows[:, :2], dtype=float)
    gt_xy = gt_xy - gt_xy[0]
    vo_xy = np.asarray(vo_rows[:, :2], dtype=float)
    vo_xy = vo_xy - vo_xy[0]
    fu = np.asarray(fusion_rows, dtype=float)
    fu_xy = fu[:, 1:3] - fu[:, 1:3][0]
    imu_xy = fu[:, 7:9] - fu[:, 7:9][0]

    print("\n" + "─" * 62)
    print(f"  REPLAY 结果（{n} 帧）")
    print("─" * 62)
    print(f"{'Method':<14}{'ATE(m)':<12}{'RPE(m/f)':<12}{'Drift%':<10}{'final_scale':<12}")
    for name, xy in (('Pure IMU', imu_xy), ('Pure VO', vo_xy), ('EKF Fusion', fu_xy)):
        print(f"{name:<14}{_ate(gt_xy, xy):<12.2f}{_rpe(gt_xy, xy):<12.4f}"
              f"{_drift(xy, gt_xy):<10.2f}{ekf.get_scale() if name == 'EKF Fusion' else float('nan'):<12.4f}")
    print("─" * 62)
    print(f"[REPLAY] 尺度轨迹: 初值 {VO_SCALE_PRIOR} → 终值 {ekf.get_scale():.4f} "
          f"(每250帧: {[(f, round(s, 3)) for f, s in scale_hist[-4:]]})")
    print(f"[REPLAY] 航向观测: applied={ekf._heading_applied}, "
          f"rejected={ekf._heading_rejected}, 转弯帧≈{ekf._turn_count}")
    print(f"[REPLAY] 位置观测跳过: {ekf._pos_skip_count}")

    fu_ate = _ate(gt_xy, fu_xy)
    vo_ate = _ate(gt_xy, vo_xy)
    imu_ate = _ate(gt_xy, imu_xy)
    if fu_ate < vo_ate and fu_ate < imu_ate:
        print(f"[REPLAY] ✅ Fusion 同时优于 Pure VO ({vo_ate:.2f}m) 与 Pure IMU ({imu_ate:.2f}m)")
    else:
        print(f"[REPLAY] ⚠️ Fusion 未全面领先: Fusion={fu_ate:.2f} "
              f"PureVO={vo_ate:.2f} PureIMU={imu_ate:.2f}")
    return fu_ate


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
    _parser.add_argument('--turnaround', action='store_true',
                         help='折返采集模式: 直行约100m后掉头返回出生点 '
                              '(NLM闭环生死门验证, 输出独立目录, 不补帧)')
    _parser.add_argument('--bidir-start', type=str, default=None,
                         metavar='X,Y',
                         help='双向折返起点(如 "106,133"): 定点投放到该双向路段, '
                              '去程100m后沿对向车道原路返回(真重访)')
    _parser.add_argument('--replay', type=str, default=None, metavar='DATA_DIR',
                         help='离线复算 EKF 融合（不采集）：在指定数据目录的 '
                              'ground_truth/visual_odometry/aligned_imu 上重跑，'
                              '覆盖 fusion_pose.txt 并打印三方法指标')
    _parser.add_argument('--fresh', action='store_true',
                         help='全新采集（监督器首轮透传）。注: 本采集器无断点续采, '
                              '每次启动都从第0帧重采, 旧目录在 step 11 备份后清空重跑')
    _args = _parser.parse_args()

    if _args.replay:
        sys.exit(0 if replay(_args.replay) is not None else 1)

    # 根据命令行参数覆盖地图和输出目录
    TARGET_MAP = _args.map
    TURNAROUND_ACTIVE = _args.turnaround
    if _args.bidir_start:
        try:
            _bx, _by = _args.bidir_start.split(',')
            TURNAROUND_BIDIR_START = (float(_bx), float(_by))
            TURNAROUND_ACTIVE = True
        except ValueError:
            print(f"[WARN] --bidir-start 格式错误: {_args.bidir_start!r} (应为 X,Y)")
    if TURNAROUND_ACTIVE:
        # 折返模式独立输出目录，不覆盖 {map}Data_IMU_Fusion 的常规数据
        OUTPUT_DIR = os.path.join(current_dir, '..', 'data',
                                  f'{TARGET_MAP}Turnaround_IMU_Fusion')
    else:
        OUTPUT_DIR = os.path.join(current_dir, '..', 'data', f'{TARGET_MAP}Data_IMU_Fusion')

    main(headless=_args.headless, host=_args.host, port=_args.port)
