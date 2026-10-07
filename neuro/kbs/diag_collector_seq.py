#!/usr/bin/env python3
# 忠实复现采集器投放序列, 定位"432路点 head=(0,2)"的成因:
#   1) 随机出生点 try_spawn (同 safe_spawn_vehicle)
#   2) 建 agent A, set_destination(西南点 (33.4,-2.0)) 并 validate (同 init 步骤8-9)
#   3) teleport 到 (167.2,59.5,z+0.1) + wait(同 wait_vehicle_deploy)
#   4) 建全新 agent B, set_destination(去程 (265.2,59.5))
# 打印 A/B 各自的 route head/tail/n/总长。若 B 的 head=(0,2) 而非(167,59),
# 即证明"先建西南agent再teleport"污染了新planner起点引用。
import math
import random
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402
sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
from agents.navigation.behavior_agent import BehaviorAgent  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def qinfo(agent, label):
    lp = getattr(agent, '_local_planner', None)
    q = (getattr(lp, '_waypoints_queue', None) or getattr(lp, 'waypoints_queue', None))
    if not q:
        print(f'  [{label}] 队列为空', flush=True)
        return
    h = q[0][0].transform.location
    t = q[-1][0].transform.location
    total = sum(q[i+1][0].transform.location.distance(q[i][0].transform.location)
                for i in range(len(q)-1))
    print(f'  [{label}] n={len(q)} head=({h.x:.1f},{h.y:.1f}) '
          f'tail=({t.x:.1f},{t.y:.1f}) 总长={total:.0f}m', flush=True)


def wait_deploy(vehicle, target, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.1)
        try:
            if vehicle.get_location().distance(target) < 1.5:
                break
        except Exception:
            return
    time.sleep(0.15)


def fresh_agent(vehicle):
    a = BehaviorAgent(vehicle, behavior='cautious')
    a.follow_speed_limits(False)
    try:
        a.set_max_speed(12 / 3.6)
    except AttributeError:
        a._max_speed = 12 / 3.6
    return a


def main():
    random.seed(7)  # 复现某个随机出生点
    c = carla.Client('localhost', 2000)
    c.set_timeout(30.0)
    world = c.get_world()
    try:
        s = world.get_settings()
        if s.synchronous_mode:
            s.synchronous_mode = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
    except Exception:
        pass
    m = world.get_map()
    bpm = world.get_blueprint_library()
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'walker.', 'sensor.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()

    spawn_pts = m.get_spawn_points()
    sp = random.choice(spawn_pts)
    car = world.try_spawn_actor(bpm.find('vehicle.lincoln.mkz_2017'), sp)
    if car is None:
        print('随机出生点spawn失败'); return
    print(f'随机出生点=({sp.location.x:.1f},{sp.location.y:.1f},{sp.location.z:.1f}) '
          f'实际落位=({car.get_location().x:.1f},{car.get_location().y:.1f})',
          flush=True)

    # 步骤2: 西南 agent A (同采集器 init)
    agentA = fresh_agent(car)
    sw = carla.Location(33.4, -2.0, 0.0)
    agentA.set_destination(sw)
    print('--- 步骤2后(teleport前) ---', flush=True)
    qinfo(agentA, 'A 西南目标(33.4,-2)')
    print(f'  此时车在=({car.get_location().x:.1f},{car.get_location().y:.1f})', flush=True)

    # 步骤3: teleport + wait (同 wait_vehicle_deploy)
    wp0 = aslist(m.get_waypoint(carla.Location(167.2, 59.5, 0.0),
                                project_to_road=True))[0]
    tf = carla.Transform(carla.Location(wp0.transform.location.x,
                                        wp0.transform.location.y,
                                        wp0.transform.location.z + 0.1),
                         wp0.transform.rotation)
    car.set_transform(tf)
    wait_deploy(car, tf.location)
    print(f'--- 步骤3后(teleport+wait) 车在=({car.get_location().x:.1f},'
          f'{car.get_location().y:.1f}) ---', flush=True)

    # 步骤4: 全新 agent B + 去程目标
    agentB = fresh_agent(car)
    out = carla.Location(265.2, 59.5, 0.0)
    agentB.set_destination(out)
    print('--- 步骤4后(全新agent B + 去程) ---', flush=True)
    qinfo(agentB, 'B 去程目标(265.2,59.5)')

    car.destroy()
    print('done', flush=True)


if __name__ == '__main__':
    main()
