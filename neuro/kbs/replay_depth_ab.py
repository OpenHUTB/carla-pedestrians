#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""P1.1/P2 EKF replay A/B 驱动: 指定深度文件跑 replay, 输出 sim3_full 对比。

用法:
  python replay_depth_ab.py --sets Town05 Town03 Town07 Town10HD \
      --depth-file depth_scale_head.txt --deadband 0.15 --tag head015
  python replay_depth_ab.py --sets Town05 --nodp --tag nodp      # 无深度基线
行为 (每集):
  1. replay 前备份 fusion_pose.txt → _p11_backup_20261014/<set>/fp_live.txt
  2. 用 --depth-file / --nodp / 默认(目录内 depth_scale.txt) 跑 replay
  3. replay 输出 fusion_pose → _p11_backup_20261014/<set>/fp_<tag>.txt
  4. 恢复原 fusion_pose.txt
  5. 打印 sim3_full RMSE: 活体 baseline / 本次 run
"""
import argparse
import os
import shutil
import subprocess
import sys

import numpy as np
import pandas as pd

DATA = "/home/yangrb/openhutb/neuro/data"
BACKUP = os.path.join(DATA, "_p11_backup_20261014")
EKF = "/home/yangrb/openhutb/neuro/00_collect_data/IMU_Vision_Fusion_EKF.py"
PY = "/home/yangrb/openhutb/carla/bin/python"
SETS = {s: os.path.join(DATA, s + "Data_IMU_Fusion")
        for s in ("Town05", "Town03", "Town07", "Town10HD")}
SETS["KITTI07"] = os.path.join(DATA, "01_NeuroSLAM_Datasets",
                               "KITTI07Data_IMU_Fusion")


def sim3_full_rmse(fp_path, gt_path):
    fp = pd.read_csv(fp_path)[["pos_x", "pos_y", "pos_z"]].values
    gt = pd.read_csv(gt_path)[["pos_x", "pos_y", "pos_z"]].values
    n = min(len(fp), len(gt))
    fp, gt = fp[:n], gt[:n]
    ec, gc = fp - fp.mean(0), gt - gt.mean(0)
    U, S, Vt = np.linalg.svd(gc.T @ ec)
    dd = np.linalg.det(Vt.T @ U.T)
    T = Vt.T @ np.diag([1, 1, dd]) @ U.T
    s = (S @ np.array([1, 1, dd])) / (ec ** 2).sum()
    ea = s * (fp @ T) + (gt.mean(0) - s * (fp.mean(0) @ T))
    return float(np.sqrt((np.linalg.norm(ea - gt, axis=1) ** 2).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sets", nargs="+", default=list(SETS))
    ap.add_argument("--depth-file", default=None,
                    help="深度文件名 (在数据目录内找) 或绝对路径; 缺省=目录内 depth_scale.txt")
    ap.add_argument("--nodp", action="store_true", help="禁用深度支路 (指向不存在路径)")
    ap.add_argument("--deadband", type=float, default=None,
                    help="覆盖 DEPTH_SCALE_DEADBAND (log 域)")
    ap.add_argument("--tag", default=None, help="结果标签 (默认 nodp/dp/head)")
    args = ap.parse_args()

    tag = args.tag or ("nodp" if args.nodp else "dp")
    if args.nodp:
        depth_arg = ["/nonexistent_depth_scale.txt"]
    elif args.depth_file:
        p = args.depth_file
        depth_arg = [p if os.path.isabs(p) else None]
        if depth_arg[0] is None:
            depth_arg = None  # 相对名: 每集在各自目录找
    else:
        depth_arg = None

    print(f"{'集':10s} {'活体baseline':>13s} {tag: >13s}  Δ")
    rows = []
    for name in args.sets:
        d = SETS[name]
        b = os.path.join(BACKUP, name)
        os.makedirs(b, exist_ok=True)
        fp_live = os.path.join(d, "fusion_pose.txt")
        gt = os.path.join(d, "ground_truth.txt")
        shutil.copy(fp_live, os.path.join(b, "fp_live.txt"))

        cmd = [PY, EKF, "--replay", d]
        if args.nodp:
            cmd += ["--replay-depth-file", "/nonexistent_depth_scale.txt"]
        elif depth_arg is None and args.depth_file:
            cand = os.path.join(d, args.depth_file)
            if os.path.exists(cand):
                cmd += ["--replay-depth-file", cand]
        if args.deadband is not None:
            cmd += ["--replay-deadband", str(args.deadband)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"[{name}] replay 失败:\n{r.stderr[-800:]}")
            continue
        out_fp = os.path.join(b, f"fp_{tag}.txt")
        shutil.copy(fp_live, out_fp)

        live = sim3_full_rmse(os.path.join(b, "fp_live.txt"), gt)
        new = sim3_full_rmse(out_fp, gt)
        # 恢复
        shutil.copy(os.path.join(b, "fp_live.txt"), fp_live)
        dep_msg = [l for l in r.stdout.splitlines() if "深度尺度" in l]
        dep_str = " | ".join(x.strip() for x in dep_msg[:2])
        print(f"{name:10s} {live:13.2f} {new:13.2f} {new-live:+9.2f}   {dep_str}")
        rows.append((name, live, new, new - live))

    if len(rows) >= 2:
        import statistics
        d = [x[3] for x in rows]
        print(f"\n  平均 Δ = {statistics.mean(d):+.2f} m  "
              f"(改善 {sum(1 for x in d if x < 0)}/{len(d)} 集)")


if __name__ == "__main__":
    main()
