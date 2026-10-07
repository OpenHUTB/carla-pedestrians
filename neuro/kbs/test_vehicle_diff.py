#!/usr/bin/env python3
# 隔离变量实验: 在已验证路线(167.2,59.5, yaw0)分别用 2020 与 2017 车辆
# 投放(z+0.1)并直行, 统计碰撞。定位"采集器用2017反复撞wall/traffic_light"
# 是车辆尺寸问题还是投放姿态/环境残留问题。
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def run_vehicle(world, bp_lib, model, sx, sy, zlift, frames=240):
    m = world.get_map()
    wp0 = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))[0]
    # 清场(与采集器 clear_all_actors 等价但用 list 而非 filter, 防 API 漂移)
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'sensor.', 'walker.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()
    tf = carla.Transform(carla.Location(wp0.transform.location.x,
                                        wp0.transform.location.y,
                                        wp0.transform.location.z + zlift),
                         wp0.transform.rotation)
    car = world.try_spawn_actor(bp_lib.find(model), tf)
    if car is None:
        return None
    print(f'  {model}: spawn=({tf.location.x:.1f},{tf.location.y:.1f},'
          f'{tf.location.z:.2f}) yaw={tf.rotation.yaw:.0f} '
          f'车辆尺寸={car.bounding_box.extent}', flush=True)
    hits = {'n': 0, 'types': []}

    def on_col(e):
        t = e.other_actor.type_id
        if 'fence' in t or 'wall' in t or 'traffic' in t or 'building' in t:
            hits['n'] += 1
            hits['types'].append(f'{t}(i={e.intensity:.0f})')
    col = world.spawn_actor(bp_lib.find('sensor.other.collision'),
                            carla.Transform(), car)
    col.listen(on_col)
    # 直行控制
    for f in range(frames):
        ctl = carla.VehicleControl(throttle=0.6, brake=0, steer=0)
        car.apply_control(ctl)
        t = car.get_transform()
        if f % 60 == 0:
            v = car.get_velocity()
            print(f'    f{f:>3} x={t.location.x:7.1f} y={t.location.y:6.1f} '
                  f'z={t.location.z:6.2f} sp={math.sqrt(v.x**2+v.y**2+v.z**2):4.1f} '
                  f'撞={hits["n"]}', flush=True)
        time.sleep(0.03)
    res = (hits['n'], hits['types'][:5])
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
    for model in ('vehicle.lincoln.mkz_2020', 'vehicle.lincoln.mkz_2017'):
        print(f'=== {model} @ (167.2,59.5) z+0.1 直行240帧 ===', flush=True)
        r = run_vehicle(world, bpm, model, 167.2, 59.5, 0.1)
        if r is None:
            print('  spawn失败', flush=True)
        else:
            print(f'  → 碰撞={r[0]}  {r[1]}', flush=True)


if __name__ == '__main__':
    main()
