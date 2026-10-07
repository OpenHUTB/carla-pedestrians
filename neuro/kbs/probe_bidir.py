"""probe_bidir.py — 在指定 CARLA 端口/地图上找一条"真闭环"双向折返起点。

判据(与 NLM 闭环门对齐, 见 exp_map_iteration.m):
  - 重访残差(CE) 落在 [CE_MIN=3, CE_MAX=20] m —— 双向折返残差≈车道偏移+漂移
  - 时间间隔 > MIN_GAP=300 帧 (20Hz → 15s)
  - 朝向门 60° —— 对向车道(反向)满足
本脚本只做只读探测, 不采集; 输出候选起点的 (x,y) 与几何校验, 供采集器
`--turnaround --bidir-start X,Y` 使用。

用法: probe_bidir.py [--port 2001] [--map Town01] [--out-dist 100]
"""
import argparse
import math
import carla


def _aslist(x):
    return x if isinstance(x, (list, tuple)) else ([x] if x is not None else [])


def path_len(pts):
    return sum(a.distance(b) for a, b in zip(pts, pts[1:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2001)
    ap.add_argument('--map', default='Town01')
    ap.add_argument('--out-dist', type=float, default=100.0,
                    help='去程目标距离(m), 默认100 (TURNAROUND_OUT_DIST)')
    ap.add_argument('--top', type=int, default=5, help='打印前N个候选')
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    # 服务器默认地图可能就是目标地图: 先取当前世界, 地图名不符再重载(避免重复加载超时)
    world = client.get_world()
    if world.get_map().name != args.map:
        world = client.load_world(args.map)
    m = world.get_map()
    spawns = m.get_spawn_points()
    print(f'地图 {args.map}: spawn点={len(spawns)}')

    HZ = 20.0          # fixed_delta_seconds=0.05
    MIN_GAP_F = 300    # 帧
    CE_MIN, CE_MAX = 3.0, 20.0

    cands = []
    for sp in spawns:
        loc = sp.location
        wps = _aslist(m.get_waypoint(loc, project_to_road=True))
        if not wps:
            continue
        wp = wps[0]
        if wp.lane_type == carla.LaneType.Parking:
            continue
        left = wp.get_left_lane()
        if left is None:          # 非双向路
            continue
        # 去程: 沿同 road_id 走到 out_dist
        out_pts = [wp.transform.location]
        u = wp
        while u.transform.location.distance(out_pts[0]) < args.out_dist:
            nxt = _aslist(u.next(2.0))
            if not nxt:
                break
            u = nxt[0]
            if u.road_id != wp.road_id:
                break
            out_pts.append(u.transform.location)
        out_len = path_len(out_pts)
        if out_len < 0.85 * args.out_dist:
            continue              # 路太短, 走不到目标
        chord = out_pts[-1].distance(out_pts[0])
        straight = chord / out_len if out_len > 0 else 0
        lane_w = wp.lane_width
        # 回程≈对向车道折返, 残差≈车道偏移(取该路段平均车道宽)
        resid_est = lane_w
        # 时间间隔: 去程过点d时刻 d/v; 回程过点d时刻 (2*L - d)/v (v≈3m/s, 与
        # 既有数据集实测 2.8m/s 一致)。MIN_GAP 门要求 2(L-d)/v*HZ > 300 帧
        # → 有效走廊 = d < L - 300/(2*HZ*v)*... 即 d < L - 15s*v/2
        v = 3.0
        L = out_len
        # gap门: 去程点 d 满足 2(L-d)/v*HZ>300 帧 ⇔ d < L-15s*v/2 = L-22.5m
        valid_len = max(0.0, L - (MIN_GAP_F / HZ) * v / 2.0)
        max_gap_f = 2.0 * L / v * HZ        # 走廊起点(d=0)处间隔(帧), 仅展示
        # 回程走廊验证: 车辆去程到 u(终点)后切对向车道, 沿对向 next() 走回 L 米,
        # 必须落回起点附近(local planner 原路返回走的就是这条对向车道链)
        ret_wp = [u.get_left_lane()] if u.get_left_lane() is not None else []
        ret_ok = False
        if ret_wp:
            cur = ret_wp[0]
            rpts = [cur.transform.location]
            while path_len(rpts) < L and len(rpts) < 200:
                nx = _aslist(cur.next(2.0))
                if not nx:
                    break
                cur = nx[0]
                if cur.road_id != wp.road_id:
                    break
                rpts.append(cur.transform.location)
            ret_ok = (path_len(rpts) >= 0.85 * L
                      and rpts[-1].distance(out_pts[0]) < 15.0)
        # 判定: CE残差入门 + 有效走廊≥5m + 回程走廊成立
        ok_ce = CE_MIN <= resid_est <= CE_MAX
        ok_gap = valid_len >= 5.0
        verdict = 'TRUE_LOOP' if (ok_ce and ok_gap and ret_ok) else 'no'
        cands.append((straight, loc, out_len, lane_w, resid_est,
                      max_gap_f, ok_ce, ok_gap, verdict, wp.road_id,
                      valid_len, ret_ok))

    # 按 判定>有效走廊>直线度 排序; loc 不可比较, 用 key 避免 tuple 直接 <
    cands.sort(key=lambda c: (c[8] == 'TRUE_LOOP', c[10], c[0]), reverse=True)
    print(f'\n候选双向折返起点(按 TRUE_LOOP>有效走廊>直线度 排序, 前{args.top}个):')
    print(f"{'rank':>4} {'x':>8} {'y':>8} {'去程长':>7} {'车道宽':>6} "
          f"{'残差估':>6} {'最大gap':>7} {'有效走廊':>8} {'CE':>4} {'gap':>4} {'回程':>4}  判定  road")
    for i, c in enumerate(cands[:args.top]):
        s, loc, out_len, lane_w, resid, max_gap, ok_ce, ok_gap, verdict, rid = c[:10]
        vlen, ret_ok = c[10], c[11]
        print(f'{i+1:>4} {loc.x:>8.1f} {loc.y:>8.1f} {out_len:>7.1f} '
              f'{lane_w:>6.1f} {resid:>6.1f} {max_gap:>7.0f} {vlen:>8.1f} '
              f'{"ok" if ok_ce else "no":>4} {"ok" if ok_gap else "no":>4} '
              f'{"ok" if ret_ok else "no":>4}  {verdict:<9} {rid}')

    true_loops = [c for c in cands if c[8] == 'TRUE_LOOP']
    if true_loops:
        c = true_loops[0]
        s, loc, out_len, lane_w, resid, max_gap, ok_ce, ok_gap, verdict, rid = c[:10]
        vlen, ret_ok = c[10], c[11]
        print(f'\n推荐起点: --bidir-start {loc.x:.1f},{loc.y:.1f},{loc.z:.1f}  (map={args.map})')
        print(f'  ⚠ z={loc.z:.1f} 是真实路面高度: 部署必须带z(路可能悬空/下沉, z=0会在空中撞围栏)')
        print(f'  去程≈{out_len:.0f}m, 车道宽≈{lane_w:.1f}m, 预期CE残差≈{resid:.1f}m'
              f'(门[3,20]), 满足gap门的有效走廊={vlen:.0f}m, 最大重访间隔≈{max_gap:.0f}帧, '
              f'回程走廊验证={"通过" if ret_ok else "失败"}')
    elif cands:
        print('\n⚠ 无 TRUE_LOOP 候选: CE门/gap门/回程走廊至少一项不满足, 换地图或放宽参数')


if __name__ == '__main__':
    main()
