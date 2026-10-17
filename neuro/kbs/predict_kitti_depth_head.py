#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""P2 尺度头零样本预测 (KITTI07 新真VIO VO): 加载现成 pr_scale_head.pt (不重训),
对新 npz (scale_series_KITTI07.npz, 对新VO重算) 的逐帧特征预测 log s,
写 <KITTI07Data_IMU_Fusion>/depth_scale_head.txt (EKF 深度尺度支路输入,
与 depth_scale.txt 同格式: 行 k=帧号(1-based) log_s inliers; 帧1=nan).

不重训 → 与 CARLA 尺度线用同一头/同一归一化 (mu/sd 来自 checkpoint),
保证口径一致 (train-once, zero-shot deploy).

前置: scale_series_p1.py --seqs KITTI07 --with-davl2 已跑 (新 npz).
用法: python predict_kitti_depth_head.py
"""
import sys
import numpy as np
import torch

sys.path.insert(0, "../kbs")
from pr_scale_head import Head, load_set, SETS, HEAD_PT  # noqa: E402

NAME = "KITTI07"

ckpt = torch.load(HEAD_PT, map_location="cpu", weights_only=False)  # torch>=2.6 默认 weights_only=True; ckpt 含 numpy(mu/sd)
model = Head()
model.load_state_dict(ckpt["state"])
model.eval()
mu = np.asarray(ckpt["mu"], dtype=np.float64)
sd = np.asarray(ckpt["sd"], dtype=np.float64)

X, y, inl, m, nf = load_set(NAME)
valid = m.astype(bool)
if valid.sum() == 0:
    raise SystemExit(f"[error] {NAME} 无有效特征 (新 npz 可能未生成)")
Xn = (X[valid] - mu) / sd
with torch.no_grad():
    p = model(torch.from_numpy(Xn).float()).cpu().numpy()
pred = np.full(len(y), np.nan)
pred[valid] = p

outp = SETS[NAME]["dir"] / "depth_scale_head.txt"
with open(outp, "w", encoding="utf-8") as f:
    f.write("# frame log_s inliers\n")
    f.write(f"# calibrated by P2 scale head (predict_kitti_depth_head.py, 不重训, 现成头); {NAME}\n")
    f.write("1 nan 0\n")
    for k in range(2, nf + 1):
        i = k - 2
        if i < len(pred) and valid[i]:
            f.write(f"{k} {float(pred[i]):.6f} {int(inl[i])}\n")
        else:
            f.write(f"{k} nan 0\n")

med = float(np.nanmedian(pred))
print(f"[write] {outp} ({NAME}, 有效 {int(valid.sum())}/{nf - 1})")
print(f"  预测 log s 中值={med:.4f} (s={np.exp(med):.3f})  "
      f"std(log s)={np.nanstd(pred):.4f}")
print(f"  [参考] 新EKF全局尺度偏置 s≈1.815 (0.625xGT), 头应把尺度状态拉向度量")
