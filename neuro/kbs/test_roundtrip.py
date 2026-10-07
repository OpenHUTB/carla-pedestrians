#!/usr/bin/env python3
# 验证: 在 bidir 起点用【简单航点跟随】(非BehaviorAgent) 跑完整往返
# 去程120m → 掉头 → 回程120m, 统计 fence 碰撞 / z范围 / 是否真正回到起点。
# 若 0fence 且 z 稳定在路面 且 完成往返 → 该控制器可集成进采集器。
import argparse
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def build_waypoints(m, sx, sy, out_dist, step=4.0):
    """从起点沿同 road_id 生成去程航点, 再取对向车道回程航点(逆序)。"""
    wp0 = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))[0]
    # 去程
    out = []
    u = wp0
    out.append(u.transform.location)
    while u.transform.location.distance(wp0.transform.location) < out_dist:
        nxt = aslist(u.next(step))
        if not nxt or nxt[0].road_id != wp0.road_id:
            break
        u = nxt[0]
        out.append(u.transform.location)
    out_len = sum(out[i+1].distance(out[i]) for i in range(len(out)-1))
    if out_len < 0.6 * out_dist:
        return None, out_len
    # 回程: 从去程终点切对向车道, 沿 next() 生成, 然后逆序(朝起点走)
    end_wp = aslist(m.get_waypoint(out[-1], project_to_road=True))[0]
    left = end_wp.get_left_lane()
    if left is None:
        return None, out_len
    ret = []
    u = left
    ret.append(u.transform.location)
    while u.transform.location.distance(out[-1]) < out_len + 5:
        nxt = aslist(u.next(step))
        if not nxt or nxt[0].road_id != wp0.road_id:
            break
        u = nxt[0]
        ret.append(u.transform.location)
    ret.reverse()  # 朝起点方向
    return (out, ret), out_len


def follow(world, car, wp_list, out_dest, max_frames, log_every=20):
    """沿航点列表行驶, 返回 (fence, zs, reached_end, last_loc)"""
    fence = {'n': 0}

    def on_col(e):
        if 'fence' in e.other_actor.type_id.lower():
            fence['n'] += 1
    col = world.spawn_actor(
        world.get_blueprint_library().find('sensor.other.collision'),
        carla.Transform(), car)
    col.listen(on_col)
    zs = []
    for f in range(max_frames):
        loc = car.get_transform().location
        rot = car.get_transform().rotation
        # 找最近航点(前瞻: 选前方最近)
        best = None
        best_d = 1e9
        for wp in wp_list:
            d = wp.distance(loc)
            if d < best_d:
                best_d = d
                best = wp
        if best is not None and best_d > 1.0:
            dx, dy = best.x - loc.x, best.y - loc.y
            ang_to = math.degrees(math.atan2(dy, dx))
            err = (ang_to - rot.yaw + 180) % 360 - 180
            steer = max(-0.5, min(0.5, err / 45.0))
            throttle = 1.0 if abs(err) < 60 else 0.5
            brake = 0.0
        else:
            steer, throttle, brake = 0.0, 0.0, 0.0
        car.apply_control(carla.VehicleControl(
            throttle=throttle, steer=steer, brake=brake))
        zs.append(car.get_transform().location.z)
        if f % log_every == 0:
            v = car.get_velocity()
            sp = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            print(f'  f{f:>4} x={loc.x:7.1f} y={loc.y:6.1f} z={loc.z:6.2f} '
                  f'sp={sp:4.1f} fence={fence["n"]} 距下一航点={best_d:5.1f}',
                  flush=True)
        time.sleep(0.03)
    car.get_transform()
    reached = car.get_transform().location.distance(out_dest) < 8.0
    col.destroy()
    return fence['n'], zs, reached, car.get_transform().location


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--start', type=str, default='173.1,2.0')
    ap.add_argument('--out-dist', type=float, default=110.0)
    ap.add_argument('--z', type=float, default=0.1)
    ap.add_argument('--frames', type=int, default=450)   # 每段最大帧
    a = ap.parse_args()

    c = carla.Client('localhost', a.port)
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
    # 清残留
    bpm = world.get_blueprint_library()
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'sensor.', 'walker.')):
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()

    sx, sy = [float(v) for v in a.start.split(',')]
    wps, out_len = build_waypoints(m, sx, sy, a.out_dist)
    if wps is None:
        print(f'✗ 该点无法构建往返航点(out_len={out_len:.1f}m)')
        return
    out, ret = wps
    print(f'去程航点={len(out)} (≈{out_len:.0f}m), 回程航点={len(ret)}', flush=True)

    wp0 = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))[0]
    spawn_z = wp0.transform.location.z + a.z
    tf = carla.Transform(carla.Location(wp0.transform.location.x,
                                        wp0.transform.location.y, spawn_z),
                         wp0.transform.rotation)
    car = world.try_spawn_actor(bpm.find('vehicle.lincoln.mkz_2020'), tf)
    if car is None:
        print(f'✗ spawn失败 z={spawn_z}')
        return
    print(f'spawn=({tf.location.x:.1f},{tf.location.y:.1f},{tf.location.z:.2f})',
          flush=True)

    print('=== 去程(朝终点) ===', flush=True)
    fence1, zs1, reach_out, _ = follow(world, car, out, out[-1], a.frames)
    print(f'去程结束: fence={fence1}, 到达终点={reach_out}, '
          f'z范围={min(zs1):.2f}~{max(zs1):.2f}', flush=True)

    print('=== 回程(朝起点) ===', flush=True)
    fence2, zs2, reach_start, end_loc = follow(world, car, ret, out[0],
                                               a.frames)
    zs = zs1 + zs2
    print(f'回程结束: fence={fence2}, 回到起点={reach_start}, 终点位置='
          f'({end_loc.x:.1f},{end_loc.y:.1f},{end_loc.z:.1f})', flush=True)
    print(f'\n汇总: 总fence={fence1+fence2}, 全程z范围={min(zs):.2f}~{max(zs):.2f}, '
          f'往返完成={reach_out and reach_start}', flush=True)
    car.destroy()


if __name__ == '__main__':
    main()
