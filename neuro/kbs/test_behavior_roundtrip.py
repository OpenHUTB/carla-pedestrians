#!/usr/bin/env python3
# 精确复现采集器驱动: teleport 到 (start, z+0.1), BehaviorAgent
# 去程→out_dest, 到达后再 set_destination 回起点(让 local planner 自己掉头),
# 记录每段 fence / z范围 / 是否到达。直接回答"采集器的驱动逻辑本身是否可行"。
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402
sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
from agents.navigation.behavior_agent import BehaviorAgent  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def main():
    SX, SY, ZLIFT = 173.1, 2.0, 0.1
    OUT_X, OUT_Y = 293.1, 2.0
    FRAMES = 320

    c = carla.Client('localhost', 2000)
    c.set_timeout(30.0)
    world = c.get_world()
    try:
        s0 = world.get_settings()
        if s0.synchronous_mode:
            s0.synchronous_mode = False
            s0.fixed_delta_seconds = None
            world.apply_settings(s0)
    except Exception:
        pass
    m = world.get_map()
    bpm = world.get_blueprint_library()
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'sensor.', 'walker.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()
    tm = c.get_trafficmanager()
    tm.set_synchronous_mode(False)

    wp0 = aslist(m.get_waypoint(carla.Location(SX, SY, 0.0),
                                project_to_road=True))[0]
    tf = carla.Transform(carla.Location(
        wp0.transform.location.x, wp0.transform.location.y,
        wp0.transform.location.z + ZLIFT), wp0.transform.rotation)
    car = world.try_spawn_actor(bpm.find('vehicle.lincoln.mkz_2020'), tf)
    if car is None:
        print('spawn失败'); return
    print(f'spawn=({tf.location.x:.1f},{tf.location.y:.1f},{tf.location.z:.2f}) '
          f'yaw={tf.rotation.yaw:.1f}', flush=True)

    fence = {'n': 0}

    def on_col(e):
        if 'fence' in e.other_actor.type_id.lower():
            fence['n'] += 1
    col = world.spawn_actor(bpm.find('sensor.other.collision'),
                            carla.Transform(), car)
    col.listen(on_col)

    agent = BehaviorAgent(car, behavior='cautious')
    agent.follow_speed_limits(False)
    try:
        agent.set_max_speed(12 / 3.6)
    except AttributeError:
        agent._max_speed = 12 / 3.6

    def drive(dest, label, frames):
        agent.set_destination(carla.Location(dest[0], dest[1],
                                             dest[2] if len(dest) > 2 else 0))
        zs = []
        for f in range(frames):
            try:
                ctl = agent.run_step()
                ctl.manual_gear_shift = False
                ctl.steer = max(-0.5, min(0.5, ctl.steer))
            except Exception as e:
                ctl = carla.VehicleControl(throttle=0, brake=1)
            car.apply_control(ctl)
            t = car.get_transform()
            v = car.get_velocity()
            zs.append(t.location.z)
            if f % 40 == 0:
                print(f'  {label} f{f:>3} x={t.location.x:7.1f} '
                      f'y={t.location.y:6.1f} z={t.location.z:6.2f} '
                      f'yaw={t.rotation.yaw:6.1f} '
                      f'sp={math.sqrt(v.x**2+v.y**2+v.z**2):4.1f} '
                      f'fence={fence["n"]}', flush=True)
            time.sleep(0.03)
        t = car.get_transform()
        d = t.location.distance(carla.Location(dest[0], dest[1], 0))
        print(f'  {label}结束: x={t.location.x:.1f} y={t.location.y:.1f} '
              f'z={t.location.z:.2f} 距目标={d:.1f} z范围='
              f'{min(zs):.2f}~{max(zs):.2f}', flush=True)
        return d

    print('=== 去程 → (293,2) ===', flush=True)
    d1 = drive((OUT_X, OUT_Y), '去程', FRAMES)
    print('=== 回程 → (173,2) (local planner 自动掉头) ===', flush=True)
    d2 = drive((SX, SY), '回程', FRAMES)

    print(f'\n汇总: 去程距目标={d1:.1f}m, 回程距起点={d2:.1f}m, '
          f'总fence={fence["n"]}', flush=True)
    car.destroy()
    col.destroy()


if __name__ == '__main__':
    main()
