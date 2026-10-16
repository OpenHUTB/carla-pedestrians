#!/usr/bin/env python3
"""pr_dinov2_embed.py — DINOv2-S 逐帧全局场景嵌入 (DL-NLM Phase 0)

输入:  neuro/data/01_NeuroSLAM_Datasets/{set}/ (图像在根目录: CARLA 4位1based / KITTI 6位0based)
输出:  {set}/slam_results/pr_dinov2s.npy  (N, 384) float32, L2 归一, 帧 i 对应 GT 第 i 行

模型: facebook/dinov2-small (HF 镜像), eval, patch-token 均值池化 (去 CLS)
输入图: 392x518 (h x w, 4:3 贴合 640x480, 均被 14 整除)
用法:  pr_dinov2_embed.py [set_name ...]   (无参 = 全部 5 集)
"""
import os
import sys

import numpy as np
import cv2
import torch
from transformers import AutoModel

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
os.environ.setdefault('HF_ENDPOINT', 'https://hf-mirror.com')

SETS = {
    'Town05Data_IMU_Fusion': dict(pad=4, base=1),
    'Town03Data_IMU_Fusion': dict(pad=4, base=1),
    'Town07Data_IMU_Fusion': dict(pad=4, base=1),
    'Town10HDData_IMU_Fusion': dict(pad=4, base=1),
    'KITTI07Data_IMU_Fusion': dict(pad=6, base=0),
    'KITTI09Data_IMU_Fusion': dict(pad=6, base=0),
}

MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def gt_rows(sub):
    with open(os.path.join(sub, 'ground_truth.txt')) as f:
        f.readline()
        return sum(1 for _ in f)


def main(only=None):
    model = AutoModel.from_pretrained('facebook/dinov2-small').eval().cuda()
    for p_ in model.parameters():
        p_.requires_grad_(False)
    B = 64
    for name, cfg in SETS.items():
        if only and name not in only:
            continue
        sub = os.path.join(D, name)
        N = gt_rows(sub)
        out = os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')
        emb = np.zeros((N, 384), dtype=np.float32)
        files = [os.path.join(sub, f"{cfg['base'] + i:0{cfg['pad']}d}.png") for i in range(N)]
        missing = [p for p in files if not os.path.exists(p)]
        if missing:
            print(f"[{name}] 缺 {len(missing)} 图, e.g. {missing[0]}", flush=True)
            continue
        done = 0
        for s in range(0, N, B):
            batch = files[s:s + B]
            xs = np.empty((len(batch), 3, 392, 518), dtype=np.float32)
            for k, p in enumerate(batch):
                img = cv2.imread(p, cv2.IMREAD_COLOR)          # BGR (480, 640, 3)
                img = cv2.resize(img, (518, 392), interpolation=cv2.INTER_AREA)
                rgb = img[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
                xs[k] = (rgb - MEAN[:, None, None]) / STD[:, None, None]
            xt = torch.from_numpy(xs).cuda()
            with torch.no_grad():
                h = model(xt).last_hidden_state                 # (B, 1+H*W, 384)
                e = h[:, 1:, :].mean(dim=1).cpu().numpy()
            e = e / (np.linalg.norm(e, axis=1, keepdims=True) + 1e-12)
            emb[s:s + len(batch)] = e
            done = s + len(batch)
            if done % 500 == 0 or done == N:
                print(f"[{name}] {done}/{N}", flush=True)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        np.save(out, emb)
        print(f"[{name}] saved {out} {emb.shape}", flush=True)
    print('ALL DONE', flush=True)


if __name__ == '__main__':
    main(only=sys.argv[1:] or None)
