#!/usr/bin/env python3
# 确认 bidir 起点 z 高度: 对候选点, 打印 waypoint 的 z, 并试
# try_spawn 在 z=0.0 / +0.1 / +0.3 / +0.5 各偏移下是否成功。
import sys
sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla

def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])

def main():
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
        if act.type_id.startswith(('vehicle.', 'sensor.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()

    pts = [(173.1, 2.0), (144.1, 133.2), (322.1, 129.4)]
    vbp = bpm.find('vehicle.lincoln.mkz_2020')
    print(f"{'点':>14} {'wp.z':>7} {'+0.0':>5} {'+0.1':>5} {'+0.3':>5} {'+0.5':>5} {'+0.8':>5}")
    for (x, y) in pts:
        wps = aslist(m.get_waypoint(carla.Location(x, y, 0.0),
                                    project_to_road=True))
        wp0 = wps[0]
        wz = wp0.transform.location.z
        row = []
        for dz in (0.0, 0.1, 0.3, 0.5, 0.8):
            tf = carla.Transform(
                carla.Location(wp0.transform.location.x,
                               wp0.transform.location.y, wz + dz),
                wp0.transform.rotation)
            car = world.try_spawn_actor(vbp, tf)
            ok = car is not None
            if ok:
                car.destroy()
            row.append('ok' if ok else 'NO')
        print(f'({x:6.1f},{y:6.1f}) {wz:7.2f} {row[0]:>5} {row[1]:>5} '
              f'{row[2]:>5} {row[3]:>5} {row[4]:>5}')

if __name__ == '__main__':
    main()
