#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""P2 尺度头: 轻量头标定 DAv2 零样本尺度 bias (train once 合成, zero-shot 真实)。

背景 (P1.1 结论, 2026-10-14): DAv2 零样本 PnP 尺度 bias 场景相关
(CARLA +62~74%, KITTI +19.8%; 逐帧 corr(log,log)=0.67-0.93), 零样本直接注入
EKF 深度支路不稳 (Town03 -48% … Town05 +8%)。本头学习 log s_gt = f(逐帧特征)
把 bias 标定掉, 让 EKF 深度支路一致受益。

输入 (每帧 8 维): [log s_dav2, log d_med, log d_std, log d_p10, log d_p90,
                  log(inl+1), fx/W, 1]   (d_*=DAv2 metric 深度图统计)
目标: log s_gt (逐帧 GT 尺度, drift-aware; 噪声标签 → MSE 收敛到条件均值)
模型: MLP 8→64→64→1, Adam lr=1e-3, wd=1e-3; 对照 OLS 线性基线 (同 8 特征)
训练: Town03/05/07 (合成 train-once); 评估: held-out Town10HD + 零样本 KITTI07
输出: pr_scale_head.pt + <数据目录>/depth_scale_head.txt (标定后支路输入,
      与 depth_scale.txt 同格式, 不覆盖原文件)

前置: scale_series_p1.py --with-davl2 已重跑 (npz 含 depth_stat / s_depth_inl)
用法:
  python pr_scale_head.py                # 训练 + 评估 + 写 5 集 depth_scale_head.txt
  python pr_scale_head.py --no-write     # 只训练评估
"""
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

NPZ_DIR = Path("/home/yangrb/openhutb/neuro/kbs/scale_p1")
DATA = Path("/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets")
HEAD_PT = Path("/home/yangrb/openhutb/neuro/kbs/pr_scale_head.pt")
SETS = {
    "Town03":   {"dir": DATA / "Town03Data_IMU_Fusion",   "fxw": 320.0 / 640.0},
    "Town05":   {"dir": DATA / "Town05Data_IMU_Fusion",   "fxw": 320.0 / 640.0},
    "Town07":   {"dir": DATA / "Town07Data_IMU_Fusion",   "fxw": 320.0 / 640.0},
    "Town10HD": {"dir": DATA / "Town10HDData_IMU_Fusion", "fxw": 320.0 / 640.0},
    "KITTI07":  {"dir": DATA / "KITTI07Data_IMU_Fusion",  "fxw": 707.0912 / 1242.0},
}
TRAIN = ["Town03", "Town05", "Town07"]
ALL = ["Town03", "Town05", "Town07", "Town10HD", "KITTI07"]
N_FEAT = 8
WIN = 50


def load_set(name):
    """返回 (X N×8, y N, inl N, m N, n_frames) — 全长度, 无效帧 X/y 为 nan,
    m 为有效掩码。N = 区间数 (n_frames-1)。"""
    z = np.load(NPZ_DIR / f"scale_series_{name}.npz")
    sd = np.asarray(z["s_depth"], np.float64)
    ds = np.atleast_2d(np.asarray(z["depth_stat"], np.float64))
    sg = np.asarray(z["s_gt"], np.float64)
    inl = np.asarray(z["s_depth_inl"], np.float64)
    fxw = float(SETS[name]["fxw"])
    n = min(len(sd), len(sg), ds.shape[0], len(inl))
    sd, sg, inl, ds = sd[:n], sg[:n], inl[:n], ds[:n]
    with np.errstate(invalid="ignore", divide="ignore"):
        X = np.column_stack([
            np.log(sd),
            np.log(ds[:, 0]), np.log(ds[:, 1] + 1e-6),
            np.log(ds[:, 2]), np.log(ds[:, 3]),
            np.log(inl + 1.0),
            np.full(n, fxw), np.ones(n),
        ])
    y = np.full(n, np.nan)
    pos = sg > 0
    y[pos] = np.log(sg[pos])
    m = np.isfinite(X).all(1) & np.isfinite(y)
    n_frames = int(z["frames"].size)
    return X, y, inl, m, n_frames


def windowed_median(x, W=WIN):
    n = len(x)
    out = np.full(n, np.nan)
    for i in range(n):
        lo, hi = max(0, i - W // 2), min(n, i + W // 2)
        seg = x[lo:hi]
        if np.isfinite(seg).sum() >= max(3, W // 5):
            out[i] = np.nanmedian(seg)
    return out


class Head(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(N_FEAT, 64), nn.ReLU(),
            nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, 1))

    def forward(self, x):
        return self.net(x).squeeze(-1)


def ols(X):
    c, *_ = np.linalg.lstsq(X, np.ones(len(X)) * 0 + np.zeros(len(X)), rcond=None)
    # 占位, 实际在评估里用 y 拟合
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-write", action="store_true")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--lr", type=float, default=1e-3)
    args = ap.parse_args()

    data = {s: load_set(s) for s in ALL}
    Xtr = np.concatenate([data[s][0][data[s][3]] for s in TRAIN])
    ytr = np.concatenate([data[s][1][data[s][3]] for s in TRAIN])
    print(f"[train] {len(TRAIN)} 集有效区间 {len(ytr)}")

    # 特征标准化: 只用训练集 (防泄漏); test 时复用同参数
    mu = Xtr.mean(0)
    sdv = Xtr.std(0)
    sdv[sdv < 1e-8] = 1.0
    norm = lambda X: (X - mu) / sdv

    # 训练/验证帧级切分 (防过拟合早停)
    rng = np.random.default_rng(0)
    perm = rng.permutation(len(ytr))
    n_val = max(200, len(ytr) // 10)
    val_idx, tr_idx = perm[:n_val], perm[n_val:]

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = Head().to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-3)
    Xn_tr = norm(Xtr[tr_idx]); y_tr = ytr[tr_idx]
    Xn_val = norm(Xtr[val_idx]); y_val = ytr[val_idx]
    Xt = torch.from_numpy(Xn_tr).float().to(dev)
    yt = torch.from_numpy(y_tr).float().to(dev)
    Xv = torch.from_numpy(Xn_val).float().to(dev)
    yv = torch.from_numpy(y_val).float().to(dev)
    best, best_state = float("inf"), None
    for ep in range(args.epochs):
        model.train()
        for i in range(0, len(Xt), 256):
            opt.zero_grad()
            nn.functional.mse_loss(model(Xt[i:i+256]), yt[i:i+256]).backward()
            opt.step()
        model.eval()
        with torch.no_grad():
            vloss = float(nn.functional.mse_loss(model(Xv), yv))
        if vloss < best:
            best, best_state = vloss, {k: v.cpu().clone()
                                       for k, v in model.state_dict().items()}
        if (ep + 1) % 50 == 0:
            print(f"  ep{ep+1} val_rmse={vloss**0.5:.4f} (best {best**0.5:.4f})")
    model.load_state_dict(best_state)
    model.eval()
    print(f"  训练完成 best_val_rmse={best**0.5:.4f}")

    # 评估: 逐帧 + 窗中值残差 std (log 域); 头 vs OLS 线性基线
    print("\n  评估 (残差 std, log 域, 越小越好):")
    print(f"  {'集':10s} {'角色':9s} {'头-逐帧':>9s} {'头-窗':>7s} {'OLS-窗':>8s}  有效")
    results = {}
    for name in ALL:
        X, y, inl, m, nf = data[name]
        Xn = norm(X[m])
        with torch.no_grad():
            p = model(torch.from_numpy(Xn).float().to(dev)).cpu().numpy()
        pred = np.full(len(y), np.nan)
        pred[m] = p
        yv_ = y[m]
        pw = windowed_median(pred); lw = windowed_median(y)
        mw = np.isfinite(pw) & np.isfinite(lw)
        # OLS 线性基线 (同 8 特征, 训练集拟合)
        c, *_ = np.linalg.lstsq(np.concatenate([data[s][0][data[s][3]] for s in TRAIN]),
                                np.concatenate([data[s][1][data[s][3]] for s in TRAIN]),
                                rcond=None)
        po = Xn @ c
        po_full = np.full(len(y), np.nan); po_full[m] = po
        pow_ = windowed_median(po_full)
        mwo = np.isfinite(pow_) & np.isfinite(lw)
        role = "train" if name in TRAIN else ("held-out" if name == "Town10HD" else "零样本")
        print(f"  {name:10s} {role:9s} {np.std(p - yv_):9.3f} {np.std((pw-lw)[mw]):7.3f} "
              f"{np.std((pow_-lw)[mwo]):8.3f}  {int(m.sum())}")
        results[name] = {"head_perframe": float(np.std(p - yv_)),
                         "head_windowed": float(np.std((pw - lw)[mw])),
                         "ols_windowed": float(np.std((pow_ - lw)[mwo])),
                         "n": int(m.sum())}

    torch.save({"state": model.state_dict(), "mu": mu, "sd": sdv,
                "results": results}, HEAD_PT)
    print(f"\n[done] 头保存 {HEAD_PT}")

    # 写标定后 depth_scale_head.txt (与 depth_scale.txt 同格式; 帧 k↔区间 k-1→k)
    if not args.no_write:
        for name in ALL:
            X, y, inl, m, nf = data[name]
            Xn = norm(X[m])
            with torch.no_grad():
                p = model(torch.from_numpy(Xn).float().to(dev)).cpu().numpy()
            pred = np.full(len(y), np.nan)
            pred[m] = p
            outp = SETS[name]["dir"] / "depth_scale_head.txt"
            with open(outp, "w", encoding="utf-8") as f:
                f.write("# frame log_s inliers\n")
                f.write(f"# calibrated by P2 scale head (pr_scale_head.py); {name}\n")
                f.write("1 nan 0\n")
                for k in range(2, nf + 1):
                    i = k - 2
                    if i < len(pred) and m[i]:
                        f.write(f"{k} {float(pred[i]):.6f} {int(inl[i])}\n")
                    else:
                        f.write(f"{k} nan 0\n")
            print(f"  [write] {outp.name} ({name}, 有效 {int(m.sum())}/{nf-1})")


if __name__ == "__main__":
    main()
