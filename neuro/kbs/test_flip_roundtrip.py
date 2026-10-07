#!/usr/bin/env python3
# 验证 teleport-flip 闭环(拓扑鲁棒版):
#   - 起点点位可能处于多路重叠/歧义区(get_waypoint 不同调用结果不同),
#     故枚举该点所有候选车道, 对每条车道:
#       去程链 = 沿 next() 同 road_id 的最长链(需≥0.85*OUT_DIST)
#       回程链 = 去程终点的对向车道 next() 链(需≥0.85*OUT_DIST 且
#                行驶方向须指回起点, 排除同向邻道)
#     取 min(去程长, 回程长) 最大者, 消除非确定性
#   - 掉头 = 唯一一次 teleport: 去程终点 → 对向车道, 朝向=对向车道真实yaw
#   - 去程/回程各用独立 BehaviorAgent(干净 local_planner)
# 统计 fence / z / 回到起点距离 / 横向偏移(CE残差), 判定机制可否集成。
import math
import sys
import time

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402
from agents.navigation.behavior_agent import BehaviorAgent  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def probe(m, wp, max_dist):
    """沿 wp 方向(同 road_id) next() 链, 返回 (长度, 终点航点, 航点列表)"""
    u = wp
    pts = [wp]
    for _ in range(400):
        if u.transform.location.distance(wp.transform.location) >= max_dist:
            break
        nxt = aslist(u.next(8.0))
        if not nxt:
            break
        u2 = nxt[0]
        if u2.road_id != wp.road_id:
            break
        u = u2
        pts.append(u)
    L = sum(pts[i+1].transform.location.distance(pts[i].transform.location)
            for i in range(len(pts)-1))
    return L, u, pts


def fwd_vec(lane):
    nxt = aslist(lane.next(10.0))
    if not nxt:
        return None
    a = lane.transform.location
    b = nxt[0].transform.location
    v = (b.x - a.x, b.y - a.y)
    n = math.hypot(v[0], v[1])
    if n < 1e-6:
        return None
    return (v[0] / n, v[1] / n)


def pick_route(m, sx, sy, out_dist):
    """枚举起点所有候选车道, 返回 (score, dep_lane, dep_end, dep_chain,
    ret_lane, ret_chain, ret_target) 或 None"""
    wps = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))
    best = None
    seen = set()
    for wp in wps:
        for lane in (wp, wp.get_left_lane(), wp.get_right_lane()):
            if lane is None:
                continue
            key = (lane.road_id, lane.lane_id)
            if key in seen or lane.lane_type == carla.LaneType.Parking:
                continue
            seen.add(key)
            L, dep_end, dep_chain = probe(m, lane, out_dist * 1.3)
            if L < 0.85 * out_dist:
                continue
            ret_lane = dep_end.get_left_lane()
            if ret_lane is None:
                continue
            # 回程方向必须指回起点(排除同向邻道)
            fv = fwd_vec(ret_lane)
            if fv is None:
                continue
            ex, ey = dep_end.transform.location.x, dep_end.transform.location.y
            tx, ty = sx - ex, sy - ey
            tl = math.hypot(tx, ty)
            if tl < 1e-6 or (fv[0]*tx + fv[1]*ty) / tl < 0.5:
                continue
            Lor, ret_end, ret_chain = probe(m, ret_lane, out_dist * 1.3)
            if Lor < 0.85 * out_dist:
                continue
            # 回程目标 = 回程链上离起点最近点
            ret_target = min(ret_chain,
                             key=lambda w: w.transform.location.distance(
                                 carla.Location(sx, sy, 0)))
            score = min(L, Lor)
            if best is None or score > best[0]:
                best = (score, lane, dep_end, dep_chain,
                        ret_lane, ret_chain, ret_target)
    return best


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=str, default='167.2,59.5')
    ap.add_argument('--out-dist', type=float, default=110.0)
    a = ap.parse_args()
    SX, SY = [float(v) for v in a.start.split(',')]
    ZLIFT, OUT_DIST, BUDGET = 0.1, a.out_dist, 800

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

    picked = pick_route(m, SX, SY, OUT_DIST)
    if picked is None:
        print(f'✗ 点位({SX},{SY})无满足条件的双向可行驶路线')
        return
    score, dep_lane, dep_end, dep_chain, ret_lane, ret_chain, ret_target = picked
    dep_start = dep_chain[0]
    print(f'选定路线: score={score:.0f}m  去程起点=({dep_start.transform.location.x:.1f},'
          f'{dep_start.transform.location.y:.1f}) yaw={dep_start.transform.rotation.yaw:.0f}° '
          f'road={dep_lane.road_id} lane={dep_lane.lane_id}  去程终点=({dep_end.transform.location.x:.1f},'
          f'{dep_end.transform.location.y:.1f})  回程目标=({ret_target.transform.location.x:.1f},'
          f'{ret_target.transform.location.y:.1f}) 横向偏移='
          f'{abs(ret_target.transform.location.y - dep_start.transform.location.y):.1f}m',
          flush=True)

    car = world.try_spawn_actor(
        bpm.find('vehicle.lincoln.mkz_2020'),
        carla.Transform(carla.Location(
            dep_start.transform.location.x,
            dep_start.transform.location.y,
            dep_start.transform.location.z + ZLIFT),
            dep_start.transform.rotation))
    if car is None:
        print('✗ spawn失败'); return
    print(f'spawn=({dep_start.transform.location.x:.1f},{dep_start.transform.location.y:.1f},'
          f'{car.get_transform().location.z:.2f})', flush=True)

    fence = {'n': 0}

    def on_col(e):
        if 'fence' in e.other_actor.type_id.lower():
            fence['n'] += 1
    col = world.spawn_actor(bpm.find('sensor.other.collision'),
                            carla.Transform(), car)
    col.listen(on_col)

    def mkagent():
        a = BehaviorAgent(car, behavior='cautious')
        a.follow_speed_limits(False)
        try:
            a.set_max_speed(12 / 3.6)
        except AttributeError:
            a._max_speed = 12 / 3.6
        return a

    def drive(ag, dest, label, budget, stop=15.0):
        ag.set_destination(carla.Location(dest.x, dest.y, 0.0))
        zs = []
        last = car.get_transform().location
        for f in range(budget):
            try:
                ctl = ag.run_step()
                ctl.manual_gear_shift = False
                ctl.steer = max(-0.5, min(0.5, ctl.steer))
            except Exception:
                ctl = carla.VehicleControl(throttle=0, brake=1)
            car.apply_control(ctl)
            t = car.get_transform()
            zs.append(t.location.z)
            d = t.location.distance(dest)
            if f % 100 == 0:
                v = car.get_velocity()
                print(f'  {label} f{f:>3} x={t.location.x:7.1f} '
                      f'y={t.location.y:6.1f} z={t.location.z:6.2f} '
                      f'sp={math.sqrt(v.x**2+v.y**2+v.z**2):4.1f} '
                      f'距目标={d:5.1f} fence={fence["n"]}', flush=True)
            last = t.location
            if d < stop:
                print(f'  {label} 到达(距{d:.1f}m) @f{f}', flush=True)
                break
            time.sleep(0.03)
        return last, zs

    print('=== 去程 ===', flush=True)
    a1 = mkagent()
    out_end, zs1 = drive(a1, dep_end.transform.location, '去程', BUDGET)

    # 掉头: teleport 到去程终点的对向车道(唯一一次不连续)
    flip_loc = carla.Location(ret_lane.transform.location.x,
                              ret_lane.transform.location.y,
                              car.get_transform().location.z + ZLIFT)
    flip_rot = ret_lane.transform.rotation
    print(f'=== 掉头 teleport → ({flip_loc.x:.1f},{flip_loc.y:.1f}) '
          f'yaw={flip_rot.yaw:.0f}° ===', flush=True)
    car.set_transform(carla.Transform(flip_loc, flip_rot))
    time.sleep(0.3)

    print('=== 回程 ===', flush=True)
    a2 = mkagent()
    ret_end, zs2 = drive(a2, ret_target.transform.location, '回程', BUDGET)

    zs = zs1 + zs2
    sx0 = dep_start.transform.location.x
    sy0 = dep_start.transform.location.y
    revisit_d = math.hypot(ret_end.x - sx0, ret_end.y - sy0)
    lat = abs(ret_end.y - sy0)
    print(f'\n汇总: 总fence={fence["n"]}, z范围={min(zs):.2f}~{max(zs):.2f}, '
          f'回到起点平面距离={revisit_d:.1f}m, 横向偏移(CE残差)={lat:.1f}m '
          f'(CE门[3,20])', flush=True)
    ok = (fence['n'] == 0 and revisit_d < 20.0 and 3.0 <= lat <= 20.0)
    print('判定:', '★ 机制可行(0fence+回到起点+偏移入门)' if ok else '✗ 未满足',
          flush=True)
    car.destroy()
    col.destroy()


if __name__ == '__main__':
    main()
