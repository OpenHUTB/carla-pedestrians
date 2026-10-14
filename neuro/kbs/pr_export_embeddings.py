#!/usr/bin/env python3
"""pr_export_embeddings.py — DINOv2 嵌入 npy → MATLAB .mat (PR 分支用)

{set}/slam_results/pr_dinov2s.npy  →  {set}/slam_results/pr_embeddings.mat
变量名 PR_EMB, (N,384) float32; 行 i 对应第 i 帧 (与 ground_truth 行序一致)。
"""
import os

import numpy as np
import scipy.io as sio

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion',
        'Town10HDData_IMU_Fusion', 'KITTI07Data_IMU_Fusion']

for name in SETS:
    sub = os.path.join(D, name)
    src = os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')
    dst = os.path.join(sub, 'slam_results', 'pr_embeddings.mat')
    e = np.load(src).astype(np.float32)
    sio.savemat(dst, {'PR_EMB': e})
    print(f"{name}: {e.shape} -> {os.path.getsize(dst)/1e6:.1f} MB")
print('DONE')
