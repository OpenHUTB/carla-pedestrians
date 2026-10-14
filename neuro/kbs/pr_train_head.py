#!/usr/bin/env python3
"""pr_train_head.py — P2 train-once: 冻结 DINOv2 特征上的轻量头 (度量学习)

设计:
  输入   = Phase 0 预计算冻结 DINOv2-S 特征 (384d, L2 归一) —— 训练无需 DINOv2 前向。
  模型   = head: 384 -> 384(LN+GELU) -> 128, 输出 L2 归一; 微调嵌入 = head(冻结特征)。
  损失   = triplet (重访 anchor/正对/难负) + 难负 hinge (自相似非重访压到阈值下)。
           直接治零样本假闭环根因: 分离 "真重访" 与 "自相似非重访"。
  训练集 = Town05 + Town03 + Town07 (CARLA 合成, train-once); KITTI 不进训练 (零样本)。
           正对来自 Town05(3场所)/Town03(1场所); 难负来自全部 3 集。
  实现   = 所有嵌入按帧索引直接取 (FEAT[idx] -> head), 不做 batch 成员 searchsorted,
           避免索引对齐 bug。head 极轻, 多次前向代价可忽略。
用法:  pr_train_head.py [--epochs 300] [--batch 512] [--margin 0.3] [--neg-m 0.3]
输出:  neuro/kbs/pr_head.pt  (head state_dict + 元信息)
"""
import os
import argparse

import numpy as np
import torch
import torch.nn as nn

D = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'
TRAIN_SETS = ['Town05Data_IMU_Fusion', 'Town03Data_IMU_Fusion', 'Town07Data_IMU_Fusion']
OUT = '/home/yangrb/openhutb/neuro/kbs/pr_head.pt'


def load_set(name):
    sub = os.path.join(D, name)
    emb = np.load(os.path.join(sub, 'slam_results', 'pr_dinov2s.npy')).astype(np.float32)
    lab = np.load(os.path.join(sub, 'slam_results', 'pr_loop_labels.npz'))
    return emb, lab['place_id'], lab['hard_neg']


class Head(nn.Module):
    def __init__(self, din=384, dmid=384, dout=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(din, dmid), nn.LayerNorm(dmid), nn.GELU(),
            nn.Linear(dmid, dout),
        )

    def forward(self, x):
        return nn.functional.normalize(self.net(x), dim=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--epochs', type=int, default=300)
    ap.add_argument('--batch', type=int, default=512)
    ap.add_argument('--margin', type=float, default=0.3)
    ap.add_argument('--neg-m', type=float, default=0.3)
    ap.add_argument('--pos-floor', type=float, default=0.7)
    ap.add_argument('--neg-ceiling', type=float, default=-0.3)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--pos-cap', type=int, default=25,
                    help='每场所每 epoch 正对上限 (2026-10-14: 3 巨型密集簇各抽 85 正对'
                         '把相似度空间压饱和 — Town05/07/10HD τ=0.9 各 ~200 假事件; '
                         '上限 25 后正负信号再平衡)')
    args = ap.parse_args()

    torch.manual_seed(0)
    np.random.seed(0)

    embs, pids, hnegs = [], [], []
    for name in TRAIN_SETS:
        e, pid, hn = load_set(name)
        embs.append(e); pids.append(pid); hnegs.append(hn)
    FEAT = torch.from_numpy(np.concatenate(embs, axis=0)).cuda()      # (Nt,384)
    off = np.concatenate([[0], np.cumsum([len(x) for x in embs[:-1]])])
    PID = np.concatenate(pids)
    HN = np.concatenate([hn + o for hn, o in zip(hnegs, off)])        # 全局索引, -1 填充

    revisit_idx = np.where(PID >= 0)[0]
    groups = {}
    for i in revisit_idx:
        groups.setdefault(int(PID[i]), []).append(i)
    group_members = {k: torch.from_numpy(np.array(v, dtype=np.int64)).cuda()
                     for k, v in groups.items() if len(v) >= 2}
    group_keys = list(group_members.keys())
    hasneg_idx = np.where(HN[:, 0] >= 0)[0]
    print(f"训练帧={FEAT.shape[0]} 场所={len(group_keys)} "
          f"(成员={{{', '.join(str(k) + ':' + str(group_members[k].shape[0]) for k in group_keys)}}}) "
          f"有难负帧={len(hasneg_idx)}", flush=True)

    model = Head().cuda()
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)

    for ep in range(args.epochs):
        model.train()
        # ── triplet: 每场所抽 anchor + 同场所正对 + 该 anchor 的难负 (直接索引) ──
        anchors, positives = [], []
        per = min(max(8, args.batch // max(1, len(group_keys) * 2)), args.pos_cap)
        for gk in group_keys:
            mem = group_members[gk]
            n = mem.shape[0]
            t = min(n, per)
            a = mem[torch.randperm(n)[:t]]
            p = mem[torch.randperm(n)[:t]]
            same = (a == p)
            if same.any():
                p[same] = (a[same] + 1) % n
            anchors.append(a)
            positives.append(p)
        a = torch.cat(anchors)
        p = torch.cat(positives)
        hn = torch.from_numpy(HN[a.cpu().numpy(), 0]).cuda()
        ok = hn >= 0
        a, p, hn = a[ok], p[ok], hn[ok]

        ea = model(FEAT[a])
        ep_ = model(FEAT[p])
        en = model(FEAT[hn])
        s_ap = (ea * ep_).sum(1)
        s_an = (ea * en).sum(1)
        # 绝对地板: 正对 sim >= pos-floor (margin triplet 只保相对序, 正对绝对值会塌到 ~0.25)
        tri = torch.clamp(args.pos_floor - s_ap, min=0.0).mean() \
            + torch.clamp(s_an - args.neg_ceiling, min=0.0).mean()

        # ── 难负 hinge: 随机帧 + 其自相似非重访 (直接索引) ──
        j = torch.from_numpy(np.random.choice(
            hasneg_idx, size=args.batch,
            replace=len(hasneg_idx) < args.batch)).cuda()
        jhn = torch.from_numpy(HN[j.cpu().numpy(), 0]).cuda()
        okj = jhn >= 0
        j, jhn = j[okj], jhn[okj]
        ej = model(FEAT[j])
        ejn = model(FEAT[jhn])
        hinge = torch.clamp((ej * ejn).sum(1) + args.neg_m, min=0.0).mean()

        loss = tri + hinge
        opt.zero_grad()
        loss.backward()
        opt.step()
        sched.step()
        if ep % 20 == 0 or ep == args.epochs - 1:
            print(f"ep{ep:3d}  loss={loss.item():.4f}  tri={tri.item():.4f} "
                  f"hinge={hinge.item():.4f}  anchors={a.shape[0]}", flush=True)

    torch.save(dict(state=model.state_dict(), dout=128, din=384,
                    train_sets=TRAIN_SETS, margin=args.margin, neg_m=args.neg_m), OUT)
    print(f"saved {OUT}  final_loss={loss.item():.4f}", flush=True)


if __name__ == '__main__':
    main()
