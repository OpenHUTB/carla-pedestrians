#!/usr/bin/env python3
# 物理可驱动性验证: 对 probe 给出的候选 bidir 起点, 逐个测试
#   (1) try_spawn_actor(wp.transform) 是否成功(几何有效+物理不重叠)
#   (2) 从该点朝去程目标行驶 60 帧, 记录 fence 碰撞数/平均速度/z范围
# 只保留 可spawn 且 行驶60帧fence=0 的候选, 作为真正可采集的闭环起点。
import argparse
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def test_candidate(world, c, bpm, out_dist, frames):
    """返回 (spawnable, fence, avg_speed, z_range, out_dist_actual)"""
    sx, sy = c
    m = world.get_map()
    wp0 = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))
    if not wp0:
        return (False, -1, 0.0, (0, 0), 0.0)
    wp0 = wp0[0]
    # 去程目标(与probe/采集器同逻辑)
    out_wp = wp0
    for dstep in range(20, 400, 10):
        nxt = aslist(wp0.next(dstep))
        if not nxt or nxt[0].road_id != wp0.road_id:
            break
        out_wp = nxt[0]
        if out_wp.transform.location.distance(
                wp0.transform.location) >= out_dist:
            break
    out_dest = out_wp.transform.location
    # (1) 物理spawn
    car = world.try_spawn_actor(
        bpm.find('vehicle.lincoln.mkz_2020'), wp0.transform)
    if car is None:
        return (False, -1, 0.0, (0, 0),
                out_dest.distance(wp0.transform.location))
    fence = {'n': 0}

    def on_col(e):
        if 'fence' in e.other_actor.type_id.lower():
            fence['n'] += 1
    col = world.spawn_actor(bpm.find('sensor.other.collision'),
                            carla.Transform(), car)
    col.listen(on_col)
    # (2) 行驶 frames 帧, 简单P控制朝 out_dest
    speeds = []
    zs = []
    for f in range(frames):
        loc = car.get_transform().location
        rot = car.get_transform().rotation
        dx, dy = out_dest.x - loc.x, out_dest.y - loc.y
        ang_to = math.degrees(math.atan2(dy, dx))
        steer = (ang_to - rot.yaw) / 90.0
        ctl = carla.VehicleControl(
            throttle=1.0, steer=max(-0.5, min(0.5, steer)))
        car.apply_control(ctl)
        v = car.get_velocity()
        speeds.append(math.sqrt(v.x**2 + v.y**2 + v.z**2))
        zs.append(loc.z)
        time.sleep(0.03)
    car.destroy()
    col.destroy()
    avg_sp = sum(speeds[10:]) / max(1, len(speeds) - 10)
    return (True, fence['n'], avg_sp, (min(zs), max(zs)),
            out_dest.distance(wp0.transform.location))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--out-dist', type=float, default=110.0)
    ap.add_argument('--frames', type=int, default=60)
    a = ap.parse_args()

    # probe 给出的 Town01 前10候选 (x, y)
    cands = [
        (173.1, 2.0), (144.1, 133.2), (322.1, 129.4), (105.6, 133.8),
        (128.9, 133.2), (220.1, 133.2), (248.4, 129.7), (396.0, 191.1),
        (272.3, 55.8), (234.7, 195.3),
    ]

    c = carla.Client('localhost', a.port)
    c.set_timeout(30.0)
    world = c.get_world()
    # 恢复异步(之前异常退出可能冻结世界)
    try:
        s0 = world.get_settings()
        if s0.synchronous_mode:
            s0.synchronous_mode = False
            s0.fixed_delta_seconds = None
            world.apply_settings(s0)
    except Exception:
        pass
    bpm = world.get_blueprint_library()
    # 清残留
    for act in world.get_actors():
        if act.type_id.startswith(('vehicle.', 'sensor.', 'walker.')) \
                or 'trafficmanager' in act.type_id:
            try:
                act.destroy()
            except Exception:
                pass
    world.tick()

    print(f"{'idx':>3} {'x':>7} {'y':>7} {'spawn':>6} {'fence':>6} "
          f"{'均速':>6} {'z范围':>12} {'去程':>6}  判定", flush=True)
    good = []
    for i, (x, y) in enumerate(cands):
        ok, fence, sp, zr, od = test_candidate(
            world, (x, y), bpm, a.out_dist, a.frames)
        zstr = f'{zr[0]:.1f}~{zr[1]:.1f}'
        if not ok:
            verdict = '不可spawn'
        elif fence == 0 and sp > 1.0:
            verdict = '★可驱动'
            good.append((x, y, od))
        else:
            verdict = '行驶异常'
        print(f'{i+1:>3} {x:>7.1f} {y:>7.1f} {"ok" if ok else "NO":>6} '
              f'{fence:>6} {sp:>6.1f} {zstr:>12} {od:>6.0f}  {verdict}',
              flush=True)

    if good:
        x, y, od = good[0]
        print(f'\n★ 推荐可驱动起点: --bidir-start {x},{y}  去程≈{od:.0f}m')
    else:
        print('\n⚠ 无候选通过物理可驱动性测试, 需换地图或调整候选')


if __name__ == '__main__':
    main()
