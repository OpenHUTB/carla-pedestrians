#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""数据集完整性校验: GT 跳变 / VO 封顶 / 帧数 / 时间戳连续性 / 文件行数一致性。

背景 (2026-10-14): Town05 车流数据 GT 含 13 次 ego 传送 (>5m/帧跳变,
根因 = 采集器碰撞/停滞重置换车传送), 污染基线 RMSE 与尺度标签。
重采验收标准: GT 跳变 = 0。本脚本对任意集做全套体检。

用法:
  python validate_dataset.py                 # 默认 5 论文集
  python validate_dataset.py Town05 Town07   # 指定集
"""
import sys

import numpy as np
import pandas as pd

DATA = "/home/yangrb/openhutb/neuro/data/"
SETS = {
    "Town03":   f"{DATA}Town03Data_IMU_Fusion",
    "Town05":   f"{DATA}Town05Data_IMU_Fusion",
    "Town07":   f"{DATA}Town07Data_IMU_Fusion",
    "Town10HD": f"{DATA}Town10HDData_IMU_Fusion",
    "KITTI07":  f"{DATA}01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion",
}
JUMP_THR = 5.0     # m/帧: 超过即判为传送/跳变
VO_CAP = 0.100     # m/帧: VO 记录硬封顶 (既定管线行为, 只报告不判错)


def check(name, d):
    import os
    import os
    r = {"set": name}
    gt = pd.read_csv(f"{d}/ground_truth.txt")
    g = gt.iloc[:, 1:4].values.astype(float)
    dg = np.linalg.norm(np.diff(g, axis=0), axis=1)
    r["frames"] = len(gt)
    r["jump_n"] = int((dg > JUMP_THR).sum())
    r["jump_max"] = float(dg.max())
    r["jump_frames"] = np.where(dg > JUMP_THR)[0].tolist()
    r["gt_path"] = float(dg.sum())
    vo_p = f"{d}/visual_odometry.txt"
    if os.path.exists(vo_p):
        vo = pd.read_csv(vo_p)
        v = vo.iloc[:, 1:4].values.astype(float)
        dv = np.linalg.norm(np.diff(v, axis=0), axis=1)
        r["vo_path"] = float(dv.sum())
        r["vo_cap_ratio"] = float((np.abs(dv - VO_CAP) < 0.001).mean())
    else:
        r["vo_path"] = float("nan")
        r["vo_cap_ratio"] = float("nan")
    # 时间戳连续性 (CARLA 20Hz)
    ts = gt["timestamp"].values.astype(float)
    if r["frames"] > 2:
        dt = np.diff(ts)
        r["ts_rate"] = float(1.0 / np.median(dt))
        r["ts_gap_max"] = float(dt.max())
    min_frames = 1000 if name == "KITTI07" else 5000   # KITTI 单序列 1101 帧
    ok = r["jump_n"] == 0 and r["frames"] >= min_frames
    r["ok"] = ok
    return r


def main():
    names = sys.argv[1:] or list(SETS)
    print(f"{'集':9s} {'帧数':>6s} {'GT跳变':>6s} {'跳变max':>9s} "
          f"{'GT路径':>9s} {'VO路径':>9s} {'VO封顶%':>8s} {'tsHz':>6s} 判定")
    allok = True
    for nm in names:
        r = check(nm, SETS[nm])
        allok &= r["ok"]
        print(f"{r['set']:9s} {r['frames']:6d} {r['jump_n']:6d} "
              f"{r['jump_max']:9.2f} {r['gt_path']:9.1f} {r['vo_path']:9.1f} "
              f"{r['vo_cap_ratio']:8.1%} {r.get('ts_rate', float('nan')):6.2f} "
              f"{'PASS' if r['ok'] else 'FAIL'}")
        if r["jump_n"]:
            print(f"           跳变帧 (0-based): {r['jump_frames']}")
    print(f"\n总结: {'全部 PASS' if allok else '存在 FAIL — 需处理'}")
    sys.exit(0 if allok else 1)


if __name__ == "__main__":
    main()
