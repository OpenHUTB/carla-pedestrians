"""xodr_bidir_scan.py — 离线解析 OpenDRIVE, 找"双向折返真闭环"候选起点。
判据与采集器 bidir 模式 + NLM 闭环门对齐:
  - 道路双向(左右 driving lane), 长度>=110m (去程100m+余量)
  - 去程100m段尽量直(chord/arc>0.95), 保证回程=原路最短
  - 优先死路(无successor): 去程终点必须U-turn, 回程唯一
  - 车道宽3~4.5m → CE残差≈车道宽+漂移, 落在[3,20]门内
用法: xodr_bidir_scan.py /path/to/Town01.xodr [--top 8]
"""
import argparse, math, xml.etree.ElementTree as ET

def _geom_type(geom):
    for ch in geom:
        if ch.tag in ('line', 'arc', 'spiral', 'paramPoly3'):
            return ch.tag, ch
    return None, None

def eval_geom(geom, s):
    """返回 (x, y, yaw) 在几何段内距段起点 s 处 (OpenDRIVE 1.4: <geometry> 内嵌形状子元素)."""
    x0, y0, hdg = float(geom.get('x')), float(geom.get('y')), float(geom.get('hdg'))
    L = float(geom.get('length'))
    gtype, _ = _geom_type(geom)
    if gtype == 'line':
        return x0 + s*math.cos(hdg), y0 + s*math.sin(hdg), hdg
    if gtype == 'arc':
        k = float(geom.findtext('arc/curvature') or 0)
        if abs(k) < 1e-8:
            return x0 + s*math.cos(hdg), y0 + s*math.sin(hdg), hdg
        a = hdg + k*s
        return (x0 + (math.sin(a)-math.sin(hdg))/k,
                y0 + (math.cos(hdg)-math.cos(a))/k, a)
    if gtype == 'spiral':
        k0 = float(geom.findtext('spiral/curvStart') or 0)
        k1 = float(geom.findtext('spiral/curvEnd') or 0)
        x, y, h = x0, y0, hdg
        ds = 0.5
        while s > ds: x += ds*math.cos(h); y += ds*math.sin(h); h += (k0+(k1-k0)*s/L)*ds; s -= ds
        x += s*math.cos(h); y += s*math.sin(h)
        return x, y, h
    if gtype == 'paramPoly3':
        pr = float(geom.get('pRange') or 1.0)
        u = min(s, pr)/pr
        au = float(geom.findtext('paramPoly3/poly3/aU')); bu=float(geom.findtext('paramPoly3/poly3/bU'))
        cu = float(geom.findtext('paramPoly3/poly3/cU')); du=float(geom.findtext('paramPoly3/poly3/dU'))
        av = float(geom.findtext('paramPoly3/poly3/aV')); bv=float(geom.findtext('paramPoly3/poly3/bV'))
        cv = float(geom.findtext('paramPoly3/poly3/cV')); dv=float(geom.findtext('paramPoly3/poly3/dV'))
        pU, pV = au+bu*u+cu*u*u+du*u**3, av+bv*u+cv*u*u+dv*u**3
        return x0 + pU*math.cos(hdg) - pV*math.sin(hdg), y0 + pU*math.sin(hdg) + pV*math.cos(hdg), None
    return x0, y0, hdg

def road_center(road, s):
    geoms = list(road.find('planView'))
    for geom in geoms:
        gs = float(geom.get('s')); L = float(geom.get('length'))
        if s <= gs + L + 1e-6 or geom is geoms[-1]:
            return eval_geom(geom, max(0.0, s-gs))
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('xodr')
    ap.add_argument('--top', type=int, default=8)
    args = ap.parse_args()
    root = ET.parse(args.xodr).getroot()
    cands = []
    for road in root.findall('road'):
        try:
            rid = int(road.get('id')); length = float(road.get('length'))
        except (TypeError, ValueError):
            continue
        if length < 110: continue
        # 双向判定: 任一laneSection同时有 left lane1 driving 与 right lane-1 driving
        bidir, w_left, w_right = False, 0.0, 0.0
        for ls in road.find('lanes').findall('laneSection') if road.find('lanes') is not None else []:
            L = ls.find('left'); R = ls.find('right')
            if L is None or R is None: continue
            l1 = L.find('lane[@id="1"]'); r1 = R.find('lane[@id="-1"]')
            if l1 is not None and r1 is not None and l1.get('type')=='driving' and r1.get('type')=='driving':
                bidir = True
                wl = l1.find('width'); wr = r1.find('width')
                w_left = float(wl.get('a')) if wl is not None else 0.0
                w_right = float(wr.get('a')) if wr is not None else 0.0
                break
        if not bidir: continue
        x0, y0, _ = road_center(road, 0.0)
        # 去程100m直线度: 采样centerline
        pts = [road_center(road, s) for s in range(0, 101, 5)]
        if any(p is None for p in pts): continue
        arc = sum(math.hypot(p[0]-q[0], p[1]-q[1]) for p, q in zip(pts, pts[1:]))
        chord = math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1])
        straight = chord/arc if arc > 0 else 0
        # 死路判定: 无successor link
        link = road.find('link'); dead_end = True
        if link is not None:
            succ = link.find('successor')
            dead_end = (succ is None or succ.get('elementId') in (None, '', '0'))
        # 回环风险: 去程终点附近100m内有并行近路? 用采样点间距粗估(>15m视为隔离)
        cands.append((straight, dead_end, rid, length, x0, y0, w_left, w_right, chord))
    cands.sort(key=lambda c: (-c[1], -c[0]))
    print(f"{'road':>6}{'死路':>5}{'长':>8}{'起点(x,y)':>22}{'左宽':>6}{'右宽':>6}{'直线度':>7}  100m弦")
    for s, de, rid, L, x0, y0, wl, wr, chord in cands[:args.top]:
        print(f"{rid:>6}{'√' if de else ' ':>5}{L:>8.0f}{f'({x0:6.1f},{y0:6.1f})':>22}"
              f"{wl:>6.1f}{wr:>6.1f}{s:>7.3f}{chord:>8.1f}")
    if cands:
        s, de, rid, L, x0, y0, wl, wr, chord = cands[0]
        print(f"\n推荐: --bidir-start {x0:.1f},{y0:.1f}  (road {rid}, 死路={de}, 直线度={s:.3f})")

if __name__ == '__main__':
    main()
