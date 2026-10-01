#!/usr/bin/env python3
"""
从 legacy rosbag-dump CSV (MH_01_easy/{cam0-image_raw.csv,imu0.csv,leica-position.csv})
构建 test_euroc_fusion_slam.m 期望的目录布局:
    <out>/cam0/data/<ts_ns>.png          8-bit 灰度
    <out>/mav0/imu0/data.csv             #ts,wRSx,wRSy,wRSz,aRSx,aRSy,aRSz (200Hz)
    <out>/mav0/state_groundtruth_estimate0/data.csv   ts,x,y,z,qx,qy,qz,qw
注: legacy 源无姿态四元数, GT 四元数填 [0,0,0,1] (MATLAB simple对齐只用位置);
    cam1 源缺失但 core 只读 cam0, 不影响。
用法: python euroc_csv_to_layout.py <in_dir> <out_dir>
"""
import os
import sys
import csv
import ast
import numpy as np
from PIL import Image

csv.field_size_limit(sys.maxsize)


def main():
    in_dir, out_dir = sys.argv[1], sys.argv[2]
    cam0_dir = os.path.join(out_dir, 'cam0', 'data')
    imu_dir = os.path.join(out_dir, 'mav0', 'imu0')
    gt_dir = os.path.join(out_dir, 'mav0', 'state_groundtruth_estimate0')
    for d in (cam0_dir, imu_dir, gt_dir):
        os.makedirs(d, exist_ok=True)

    # --- cam0: data列是 b'...' 的字节串repr ---
    src = os.path.join(in_dir, 'cam0-image_raw.csv')
    n = 0
    t0 = t1 = None
    with open(src, 'r', newline='') as f:
        rd = csv.reader(f)
        header = next(rd)
        idx = {k: i for i, k in enumerate(header)}
        for row in rd:
            ts_ns = int(row[idx['header.stamp.secs']]) * 10**9 + int(row[idx['header.stamp.nsecs']])
            h, w = int(row[idx['height']]), int(row[idx['width']])
            b = ast.literal_eval(row[idx['data']])
            if len(b) != h * w:
                print(f'!! 帧 ts={ts_ns} 字节 {len(b)} != {h*w}, 跳过')
                continue
            img = np.frombuffer(b, dtype=np.uint8).reshape(h, w)
            Image.fromarray(img, mode='L').save(os.path.join(cam0_dir, f'{ts_ns}.png'))
            if t0 is None:
                t0 = ts_ns
            t1 = ts_ns
            n += 1
            if n % 500 == 0:
                print(f'  cam0: {n} 帧')
    print(f'cam0: {n} 帧, 时长 {(t1-t0)/1e9:.2f}s, 平均 {(t1-t0)/1e9/max(1,n-1)*1000:.2f}ms/帧')

    # --- imu0: 列18:21角速度, 列30:33线加速度 (EuRoC dump布局) ---
    src = os.path.join(in_dir, 'imu0.csv')
    with open(src, 'r', newline='') as f:
        rd = csv.reader(f)
        header = next(rd)
        idx = {k: i for i, k in enumerate(header)}
        iwx, iwy, iwk = (idx['angular_velocity.x'], idx['angular_velocity.y'], idx['angular_velocity.z'])
        iax, iay, iaz = (idx['linear_acceleration.x'], idx['linear_acceleration.y'], idx['linear_acceleration.z'])
        isecs, ins = idx['header.stamp.secs'], idx['header.stamp.nsecs']
        out = open(os.path.join(imu_dir, 'data.csv'), 'w')
        out.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],'
                  'a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
        m = 0
        for row in rd:
            ts_ns = int(row[isecs]) * 10**9 + int(row[ins])
            out.write('%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n' % (
                ts_ns, float(row[iwx]), float(row[iwy]), float(row[iwk]),
                float(row[iax]), float(row[iay]), float(row[iaz])))
            m += 1
        out.close()
    print(f'imu0: {m} 行, 时长 {m/200:.2f}s (200Hz)')

    # --- leica GT: 仅位置, 四元数置单位 (MATLAB对齐只用位置) ---
    src = os.path.join(in_dir, 'leica-position.csv')
    with open(src, 'r', newline='') as f:
        rd = csv.reader(f)
        header = next(rd)
        idx = {k: i for i, k in enumerate(header)}
        isecs, ins = idx['header.stamp.secs'], idx['header.stamp.nsecs']
        ix, iy, iz = idx['point.x'], idx['point.y'], idx['point.z']
        out = open(os.path.join(gt_dir, 'data.csv'), 'w')
        out.write('#timestamp,pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w\n')
        pts = []
        g = 0
        for row in rd:
            ts_ns = int(row[isecs]) * 10**9 + int(row[ins])
            out.write('%d,%.6f,%.6f,%.6f,0,0,0,1\n' % (ts_ns, float(row[ix]), float(row[iy]), float(row[iz])))
            pts.append((float(row[ix]), float(row[iy]), float(row[iz])))
            g += 1
        out.close()
    p = np.array(pts)
    L = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    print(f'GT(leica): {g} 行, 轨迹长 {L:.2f}m, 范围 x[{p[:,0].min():.1f},{p[:,0].max():.1f}] '
          f'y[{p[:,1].min():.1f},{p[:,1].max():.1f}] z[{p[:,2].min():.1f},{p[:,2].max():.1f}]')
    print('DONE ->', out_dir)


if __name__ == '__main__':
    main()
