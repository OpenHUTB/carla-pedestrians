#!/usr/bin/env python3
# 诊断: 车在(167.2,59.5)朝+yaw0, 目标(265.2,59.5)
#   A) 全新agent + set_destination
#   B) 预置目的地agent: d = bp.find('car...') 不行; 用 BehaviorAgent 后先 set 再 tick 3次
#   C) 全新agent + set_destination 后再 world.tick()*5
# 打印每种方式的 waypoints_queue 数量/首点/尾点/总长, 定位"432路点绕中心"的成因。
# 同时打印世界里的 vehicle actor 列表(排查残留旧车)。
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402
sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
from agents.navigation.behavior_agent import BehaviorAgent  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def queue_info(agent, label):
    q = (getattr(agent._local_planner, '_waypoints_queue', None)
         or getattr(agent._local_planner, 'waypoints_queue', None))
    if not q:
        print(f'  [{label}] 队列为空', flush=True)
        return
    h = q[0][0].transform.location
    t = q[-1][0].transform.location
    total = sum(q[i+1][0].transform.location.distance(q[i][0].transform.location)
                for i in range(len(q)-1))
    print(f'  [{label}] n={len(q)} head=({h.x:.1f},{h.y:.1f}) '
          f'tail=({t.x:.1f},{t.y:.1f}) 路径总长={total:.0f}m', flush=True)


def main():
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
    # 清场
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'walker.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()
    vehs = [a for a in world.get_actors() if a.type_id.startswith('vehicle.')]
    print(f'清场后世界车辆: {[a.type_id for a in vehs]}', flush=True)

    wp0 = aslist(m.get_waypoint(carla.Location(167.2, 59.5, 0.0),
                                project_to_road=True))[0]
    car = world.try_spawn_actor(bpm.find('vehicle.lincoln.mkz_2020'),
                                carla.Transform(
                                    carla.Location(wp0.transform.location.x,
                                                   wp0.transform.location.y,
                                                   wp0.transform.location.z + 0.1),
                                    wp0.transform.rotation))
    if car is None:
        print('spawn失败'); return
    dest = carla.Location(265.2, 59.5, 0.0)

    # A: 全新agent立即set_destination
    a = BehaviorAgent(car, behavior='cautious')
    a.follow_speed_limits(False)
    try:
        a.set_max_speed(12 / 3.6)
    except AttributeError:
        a._max_speed = 12 / 3.6
    a.set_destination(dest)
    queue_info(a, 'A 全新agent立即set')
    a.destroy()

    # B: 全新agent, set后sleep 0.3s
    b = BehaviorAgent(car, behavior='cautious')
    b.follow_speed_limits(False)
    try:
        b.set_max_speed(12 / 3.6)
    except AttributeError:
        b._max_speed = 12 / 3.6
    b.set_destination(dest)
    time.sleep(0.3)
    queue_info(b, 'B set后sleep0.3')
    b.destroy()

    # C: 全新agent, set后手动run_step 10次
    cc = BehaviorAgent(car, behavior='cautious')
    cc.follow_speed_limits(False)
    try:
        cc.set_max_speed(12 / 3.6)
    except AttributeError:
        cc._max_speed = 12 / 3.6
    cc.set_destination(dest)
    for _ in range(10):
        ctl = cc.run_step()
        ctl.manual_gear_shift = False
        car.apply_control(ctl)
        time.sleep(0.03)
    queue_info(cc, 'C set后run_step10次')
    cc.destroy()

    # D: local_planner 直接 plan_route (不经过 agent)
    from agents.navigation.local_planner import LocalPlanner, Behavior  # noqa: E402
    lp = LocalPlanner(Behavior.CAUTIOUS, speed=12)
    car_id = car.id
    lp.set_destination(dest)
    # plan_route 需要 world + car_id
    try:
        ok = lp.plan_route(world, car_id, False)
        print(f'  [D] plan_route直接: ok={ok}', flush=True)
        q = (getattr(lp, '_waypoints_queue', None)
             or getattr(lp, 'waypoints_queue', None))
        if q:
            h = q[0][0].transform.location
            t = q[-1][0].transform.location
            total = sum(q[i+1][0].transform.location.distance(q[i][0].transform.location)
                        for i in range(len(q)-1))
            print(f'  [D] n={len(q)} head=({h.x:.1f},{h.y:.1f}) '
                  f'tail=({t.x:.1f},{t.y:.1f}) 总长={total:.0f}m', flush=True)
    except Exception as e:
        print(f'  [D] 异常: {e}', flush=True)

    car.destroy()
    print('done', flush=True)


if __name__ == '__main__':
    main()
