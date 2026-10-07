#!/usr/bin/env python3
# 网格扫描 Town01 双向折返闭环路线(单次会话, 显式加载 Town01):
#   对每个候选点枚举其所有候选车道, 要求
#     去程链(同road_id, next()链) ≥ 0.85*OUT_DIST 且
#     对向车道回程链 ≥ 0.85*OUT_DIST 且 行驶方向指回起点(cos>0.5)
#   取 min(去程长,回程长) 最大者; 输出 top 路线 + 最佳点的 spawn 可行性。
# 目的: 避开歧义点位(get_waypoint 非确定性), 找到唯一可驱动闭环路线。
import math
import sys

sys.path.insert(0, '/home/yangrb/下载/carla/CARLA_0.9.16/PythonAPI/carla')
import carla  # noqa: E402


def aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def probe(m, wp, max_dist):
    u = wp
    pts = [wp]
    for _ in range(60):
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
    wps = aslist(m.get_waypoint(carla.Location(sx, sy, 0.0),
                                project_to_road=True))
    if not wps:
        return None
    best = None
    seen = set()
    for wp in wps:
        for lane in (wp, wp.get_left_lane(), wp.get_right_lane()):
            if lane is None or lane.lane_type == carla.LaneType.Parking:
                continue
            key = (lane.road_id, lane.lane_id)
            if key in seen:
                continue
            seen.add(key)
            L, dep_end, dep_chain = probe(m, lane, out_dist * 1.3)
            if L < 0.85 * out_dist:
                continue
            ret_lane = dep_end.get_left_lane()
            if ret_lane is None:
                continue
            fv = fwd_vec(ret_lane)
            if fv is None:
                continue
            ex = dep_end.transform.location.x
            ey = dep_end.transform.location.y
            tx, ty = sx - ex, sy - ey
            tl = math.hypot(tx, ty)
            if tl < 1e-6 or (fv[0]*tx + fv[1]*ty) / tl < 0.5:
                continue
            Lor, _ret_end, ret_chain = probe(m, ret_lane, out_dist * 1.3)
            if Lor < 0.85 * out_dist:
                continue
            ret_target = min(ret_chain,
                             key=lambda w: w.transform.location.distance(
                                 carla.Location(sx, sy, 0)))
            score = min(L, Lor)
            if best is None or score > best[0]:
                best = (score, lane, dep_end, dep_chain,
                        ret_lane, ret_chain, ret_target)
    return best


def main():
    OUT_DIST = 110.0
    # 已知候选点(probe给出) 各加 ±10m 网格
    base = [
        (173.1, 2.0), (144.1, 133.2), (322.1, 129.4), (105.6, 133.8),
        (128.9, 133.2), (220.1, 133.2), (248.4, 129.7), (396.0, 191.1),
        (272.3, 55.8), (234.7, 195.3), (167.2, 57.5), (300.0, 133.0),
        (180.0, 150.0), (260.0, 145.0),
    ]
    pts = set()
    for (bx, by) in base:
        for dx in (-10.0, 0.0, 10.0):
            for dy in (-10.0, 0.0, 10.0):
                pts.add((round(bx+dx, 1), round(by+dy, 1)))
    pts = sorted(pts)
    print(f'扫描 {len(pts)} 个点 ...', flush=True)

    c = carla.Client('localhost', 2000)
    c.set_timeout(60.0)
    world = c.get_world()
    if world.get_map().name != 'Town01':
        world = c.load_world('Town01')
    m = world.get_map()
    print(f'地图={m.name}', flush=True)

    results = []
    for i, (x, y) in enumerate(pts):
        try:
            r = pick_route(m, x, y, OUT_DIST)
        except Exception:
            r = None
        if r is not None:
            score, dep_lane, dep_end, dep_chain, ret_lane, ret_chain, ret_tgt = r
            dep0 = dep_chain[0]
            lat = abs(ret_tgt.transform.location.y - dep0.transform.location.y) \
                if abs(ret_tgt.transform.location.x - dep0.transform.location.x) < 30 \
                else abs(ret_tgt.transform.location.x - dep0.transform.location.x)
            results.append((score, x, y, dep_lane, dep0, dep_end, ret_tgt, lat))
        if (i+1) % 30 == 0:
            print(f'  已扫 {i+1}/{len(pts)}, 命中 {len(results)}', flush=True)

    results.sort(key=lambda r: -r[0])
    print(f'\n命中路线 {len(results)} 条, top10:')
    print(f"{'score':>6} {'起点':>16} {'去程终':>16} {'回程目标':>16} "
          f"{'横向偏移':>8} {'road/lane':>12} {'去程yaw':>7} {'回程方向yaw':>11}")
    for (score, x, y, dep_lane, dep0, dep_end, ret_tgt, lat) in results[:10]:
        rt = ret_tgt.transform.rotation.yaw
        print(f'{score:6.0f} ({x:7.1f},{y:6.1f}) '
              f'({dep_end.transform.location.x:6.1f},{dep_end.transform.location.y:5.1f}) '
              f'({ret_tgt.transform.location.x:6.1f},{ret_tgt.transform.location.y:5.1f}) '
              f'{lat:7.1f}  {dep_lane.road_id}/{dep_lane.lane_id}'
              f'{dep0.transform.rotation.yaw:8.0f} {rt:11.0f}')

    if results:
        (score, x, y, dep_lane, dep0, dep_end, ret_tgt, lat) = results[0]
        bpm = world.get_blueprint_library()
        tf = carla.Transform(
            carla.Location(dep0.transform.location.x,
                           dep0.transform.location.y,
                           dep0.transform.location.z + 0.1),
            dep0.transform.rotation)
        car = world.try_spawn_actor(bpm.find('vehicle.lincoln.mkz_2020'), tf)
        ok = car is not None
        if ok:
            car.destroy()
        print(f'\n★ 最佳: 查询点({x},{y}) → 去程起点航点=({dep0.transform.location.x:.1f},'
              f'{dep0.transform.location.y:.1f},z={dep0.transform.location.z:.2f}) '
              f'road={dep_lane.road_id} lane={dep_lane.lane_id} '
              f'score={score:.0f}m 横向偏移={lat:.1f}m '
              f'spawn(z+0.1)={"OK" if ok else "失败"}')
        if ok:
            print(f'  采集命令: --bidir-start {dep0.transform.location.x:.1f},'
                  f'{dep0.transform.location.y:.1f}  (查询点可留 {x},{y})')
    else:
        print('\n✗ 无满足条件的路线, 需放宽 OUT_DIST 或换地图')


if __name__ == '__main__':
    main()
