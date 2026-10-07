#!/usr/bin/env python3
# 决定性对比: 同一目标点(167.2,59.5, z+0.1)
#   A) try_spawn_actor 直接投放(隔离测试用的方式, 已证0撞)
#   B) 先在随机点 try_spawn, 再 set_transform 瞬移到目标(采集器的bidir投放方式)
# 比较两者实际落位z、是否嵌入、碰撞数。定位"采集器反复撞wall/traffic_light"
# 到底是 set_transform 瞬移把车压进了静态墙/红绿灯, 还是其它。
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def clear_dyn(world):
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'sensor.', 'walker.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()


def deploy_and_check(world, bp_lib, mode, sx, sy, zlift, drive_frames=120):
    m = world.get_map()
    wp0 = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))[0]
    target_z = wp0.transform.location.z + zlift
    clear_dyn(world)
    if mode == 'spawn':
        car = world.try_spawn_actor(
            bp_lib.find('vehicle.lincoln.mkz_2020'),
            carla.Transform(carla.Location(wp0.transform.location.x,
                                           wp0.transform.location.y, target_z),
                            wp0.transform.rotation))
    else:  # teleport: 先随机点出生再瞬移
        spawn_pts = m.get_spawn_points()
        import random
        sp = random.choice(spawn_pts)
        car = world.try_spawn_actor(bp_lib.find('vehicle.lincoln.mkz_2020'), sp)
        if car is None:
            return None
        car.set_transform(carla.Transform(
            carla.Location(wp0.transform.location.x, wp0.transform.location.y,
                           target_z), wp0.transform.rotation))
    if car is None:
        return None
    hits = {'n': 0, 'types': []}

    def on_col(e):
        t = e.other_actor.type_id
        if any(k in t for k in ('fence', 'wall', 'traffic', 'building', 'vehicle')):
            hits['n'] += 1
            hits['types'].append(f'{t}(i={e.intensity:.0f})')
    col = world.spawn_actor(bp_lib.find('sensor.other.collision'),
                            carla.Transform(), car)
    col.listen(on_col)
    # 静止30帧让物理稳定, 再直行
    for f in range(30 + drive_frames):
        if f < 30:
            ctl = carla.VehicleControl(throttle=0, brake=1)
        else:
            ctl = carla.VehicleControl(throttle=0.6, brake=0, steer=0)
        car.apply_control(ctl)
        time.sleep(0.03)
    t = car.get_transform()
    res = dict(z=round(t.location.z, 3), x=round(t.location.x, 1),
               y=round(t.location.y, 1), hits=hits['n'], types=hits['types'][:6])
    car.destroy()
    col.destroy()
    return res


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
    bpm = world.get_blueprint_library()
    for mode in ('spawn', 'teleport'):
        print(f'=== 方式={mode} @ (167.2,59.5) z+0.1 ===', flush=True)
        for trial in range(3):
            r = deploy_and_check(world, bpm, mode, 167.2, 59.5, 0.1)
            if r is None:
                print(f'  试{trial+1}: 失败', flush=True)
            else:
                print(f'  试{trial+1}: 落位z={r["z"]} ({r["x"]},{r["y"]}) '
                      f'撞={r["hits"]} {r["types"]}', flush=True)


if __name__ == '__main__':
    main()
