#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P1.0 三对比尺度序列 + 双侧 bias 测量 (纯 Python, 不碰现有管线).

对每条序列计算三种"逐帧尺度序列"。它们都估计同一个量:
    把 VO 的(尺度无关)位移映射到度量位移所需的尺度因子 s。
  - GT 尺度      s_gt    = |dGT| / |dVO|            全帧, 真值参考 (所有序列)
  - 运动学尺度    s_kin   = (|a_lat|/|w|) / v_vo      转弯帧, 现有 EKF 支路口径 (CARLA only)
  - DAv2-PnP 尺度 s_depth = |t_PnP| / |dVO|           全帧, 需 torch+DAv2 (--with-davl2)

输出 (每条序列):
  - 指标表: scale_bias = median(s_est/s_gt) - 1 (系统性偏差)
            scale_std  = std(s_est/s_gt)        (逐帧噪声)
            退化率     = 无效帧占比
  - 叠加对比图 (log 尺度)

用法:
  python scale_series_p1.py                       # GT + 运动学(CARLA) / GT(KITTI)
  python scale_series_p1.py --with-davl2          # 额外加 DAv2-PnP (需 torch)
  python scale_series_p1.py --seqs Town05 KITTI07
  python scale_series_p1.py --out <dir>
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

BASE = Path("/home/yangrb/openhutb/neuro/data")
OUT_DEFAULT = Path("/home/yangrb/openhutb/neuro/kbs/scale_p1")

# CARLA 640x480, 默认 FOV=90deg -> fx=fy=(W/2)/tan(45)=320, cx=320, cy=240
K_CARLA = np.array([[320.0, 0.0, 320.0], [0.0, 320.0, 240.0], [0.0, 0.0, 1.0]])
# KITTI cam2 (gray image_0): fx=fy=707.0912, cx=601.8873, cy=183.1104
K_KITTI = np.array([[707.0912, 0.0, 601.8873], [0.0, 707.0912, 183.1104], [0.0, 0.0, 1.0]])

SEQUENCES = {
    "Town05": {
        "dir": BASE / "Town05Data_IMU_Fusion",
        "gt": "ground_truth.txt", "vo": "visual_odometry.txt", "imu": "aligned_imu.txt",
        "K": K_CARLA, "carla": True, "img": r"(\d+)\.png",
    },
    "Town03": {
        "dir": BASE / "01_NeuroSLAM_Datasets" / "Town03Data_IMU_Fusion",
        "gt": "ground_truth.txt", "vo": "visual_odometry.txt", "imu": "aligned_imu.txt",
        "K": K_CARLA, "carla": True, "img": r"(\d+)\.png",
    },
    "Town07": {
        "dir": BASE / "01_NeuroSLAM_Datasets" / "Town07Data_IMU_Fusion",
        "gt": "ground_truth.txt", "vo": "visual_odometry.txt", "imu": "aligned_imu.txt",
        "K": K_CARLA, "carla": True, "img": r"(\d+)\.png",
    },
    "Turnaround": {
        "dir": BASE / "01_NeuroSLAM_Datasets" / "Town01Turnaround_IMU_Fusion",
        "gt": "ground_truth.txt", "vo": "visual_odometry.txt", "imu": "aligned_imu.txt",
        "K": K_CARLA, "carla": True, "img": r"(\d+)\.png",
    },
    "Town10HD": {
        "dir": BASE / "01_NeuroSLAM_Datasets" / "Town10HDData_IMU_Fusion",
        "gt": "ground_truth.txt", "vo": "visual_odometry.txt", "imu": "aligned_imu.txt",
        "K": K_CARLA, "carla": True, "img": r"(\d+)\.png",
    },
    "KITTI07": {
        "dir": BASE / "_kitti07_newraw",   # [2026-10-15] 新真VIO VO (旧 BASE/"KITTI_07" 为合成管线)
        "gt": "ground_truth_ekf.txt", "vo": "visual_odometry_ekf.txt", "imu": None,
        "K": K_KITTI, "carla": False, "img": r"(\d+)\.png",
        "img_dir": "image_0", "img_fmt": "{:06d}", "img_off": 0,
    },
}


# --------------------------------------------------------------------------- #
# loaders
# --------------------------------------------------------------------------- #
def _gen(path):
    return np.genfromtxt(path, delimiter=",", skip_header=1)


def load_carla_pose(path):
    """timestamp,x,y,z,roll,pitch,yaw -> (times, pos Nx3)."""
    d = _gen(path)
    return d[:, 0], d[:, [1, 2, 3]]


def load_kitti_gt(path):
    """timestamp,x,y,z,roll,pitch,yaw,vel_x,vel_y,vel_z -> (times, pos)."""
    d = _gen(path)
    return d[:, 0], d[:, [1, 2, 3]]


def load_kitti_vo(path):
    """timestamp,vo_x,vo_y,vo_z,vo_roll,vo_pitch,vo_yaw,num_matches,scale."""
    d = _gen(path)
    return d[:, 0], d[:, [1, 2, 3]], d[:, 7], d[:, 8]


def load_imu(path):
    """timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z -> (t, accel, gyro)."""
    d = _gen(path)
    return d[:, 0], d[:, [1, 2, 3]], d[:, [4, 5, 6]]


def load_seq(cfg):
    if cfg["carla"]:
        gt_t, gt_p = load_carla_pose(cfg["dir"] / cfg["gt"])
        vo_t, vo_p = load_carla_pose(cfg["dir"] / cfg["vo"])
    else:
        gt_t, gt_p = load_kitti_gt(cfg["dir"] / cfg["gt"])
        vo_t, vo_p, nmatch, vscale = load_kitti_vo(cfg["dir"] / cfg["vo"])
    out = {"gt_t": gt_t, "gt_p": gt_p, "vo_t": vo_t, "vo_p": vo_p}
    if cfg["imu"] is not None and (cfg["dir"] / cfg["imu"]).exists():
        im_t, accel, gyro = load_imu(cfg["dir"] / cfg["imu"])
        out["accel"], out["gyro"], out["imu_t"] = accel, gyro, im_t
    else:
        out["accel"] = out["gyro"] = None
    return out


# --------------------------------------------------------------------------- #
# scale series
# --------------------------------------------------------------------------- #
def gt_scale_series(gt_p, vo_p, min_disp=0.02):
    """s_gt[i] = |GT[i+1]-GT[i]| / |VO[i+1]-VO[i]|, 长度 N-1.
    仅"车真的在动"的帧有效 (dGT 与 dVO 均显著) — 静止帧尺度不可观测, 比值只是 VO 漂移."""
    dgt = np.linalg.norm(np.diff(gt_p, axis=0), axis=1)
    dvo = np.linalg.norm(np.diff(vo_p, axis=0), axis=1)
    s = np.full(len(dvo), np.nan)
    m = (dvo > min_disp) & (dgt > min_disp) & np.isfinite(dgt) & np.isfinite(dvo)
    s[m] = dgt[m] / dvo[m]
    return s, dvo


def kinematics_scale_series(accel, gyro, vo_p, vo_t,
                            omega_th=0.10, a_th=0.30, max_a=30.0, max_w=3.0):
    """s_kin[i] = (|a_lat[i+1]|/|w[i+1]|) / v_vo[i], 长度 N-1, 转弯帧有效.

    a_lat = 横向比力 (body y), w = 偏航率 (body z), v_vo = VO 区间速度.
    与现有 EKF 转弯运动学尺度支路同口径 (|a_lat|/|w| = 度量速度).
    """
    if accel is None or gyro is None:
        return None
    n = len(vo_p)
    dvo = np.linalg.norm(np.diff(vo_p, axis=0), axis=1)
    dt = np.diff(vo_t)
    v_vo = np.full(len(dvo), np.nan)
    mt = dt > 0
    v_vo[mt] = dvo[mt] / dt[mt]

    a_lat = np.abs(accel[:, 1])
    w = np.abs(gyro[:, 2])
    v_imu = np.full(len(accel), np.nan)
    m = (w > omega_th) & (a_lat > a_th) & (a_lat < max_a) & (w < max_w)
    v_imu[m] = a_lat[m] / w[m]

    s = np.full(n - 1, np.nan)
    # 对齐: 帧 i+1 的 IMU 状态 对应 区间 i (t -> t+1) 的 VO 速度
    if len(v_imu) >= n:
        vi = v_imu[1:n]
        mm = np.isfinite(vi) & (v_vo > 0) & np.isfinite(v_vo)
        s[mm] = vi[mm] / v_vo[mm]
    return s


def windowed_cumulative_scale(gt_p, vo_p, W=50, min_win_disp=2.0):
    """窗内累积位移比 (尺度是低频量, 累积位移主导 -> 比值稳定, 远优于逐帧).
    s[c] = ||GT[c+W/2]-GT[c-W/2]|| / ||VO[c+W/2]-VO[c-W/2]||, 仅窗内 VO 位移>阈值有效."""
    n = len(gt_p)
    h = W // 2
    s = np.full(n, np.nan)
    g = gt_p[2 * h:n] - gt_p[0:n - 2 * h]
    v = vo_p[2 * h:n] - vo_p[0:n - 2 * h]
    dgt = np.linalg.norm(g, axis=1)
    dvo = np.linalg.norm(v, axis=1)
    m = dvo > min_win_disp
    seg = s[h:n - h]
    seg[m] = dgt[m] / dvo[m]
    return s


def windowed_median(s_perframe, W=50):
    """滑动中值 (与 EKF 转弯窗中值同口径); 先对 nan 插值再滑窗."""
    if s_perframe is None or not np.any(np.isfinite(s_perframe)):
        return None
    x = s_perframe.astype(float).copy()
    nans = ~np.isfinite(x)
    if nans.all():
        return None
    x[nans] = np.interp(np.where(nans)[0], np.where(~nans)[0], x[~nans])
    from scipy.ndimage import median_filter
    return median_filter(x, size=W, mode="nearest")


def global_scale(src, dst):
    """Umeyama Sim(3): dst ≈ s*(R@src.T).T + t, 返回真尺度 s* (src=VO -> dst=GT).
    与论文 Sim(3) ATE 对齐同口径; 是全局单值, 不受窗内弦长/静止帧噪声污染."""
    src = src.astype(float); dst = dst.astype(float)
    n = src.shape[0]
    mu_s = src.mean(0); mu_d = dst.mean(0)
    sc = src - mu_s; dc = dst - mu_d
    M = (sc.T @ dc) / n
    U, S, Vt = np.linalg.svd(M)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U) * np.linalg.det(Vt))
    R = Vt.T @ D @ U.T
    rsc = sc @ R.T
    return float((dc * rsc).sum() / (sc * sc).sum())


def est_metrics(s_est, s_gt):
    """估计序列 s_est 相对逐帧 GT 尺度 s_gt 的 bias + 一致性 (drift-aware).
    比值 s_est/s_gt 中 VO 噪声相消: depth->|t_PnP|/|dGT|, kin->v_imu*dt/|dGT|.
    逐帧口径天然适应尺度漂移, 不受单一 s* 假设限制."""
    if s_est is None:
        return None
    n = min(len(s_est), len(s_gt))
    s_est, s_gt = s_est[:n], s_gt[:n]
    m = np.isfinite(s_est) & np.isfinite(s_gt) & (s_gt > 0)
    if m.sum() < 10:
        return {"n_valid": int(m.sum()), "note": "too few"}
    r = s_est[m] / s_gt[m]
    return {
        "n_valid": int(m.sum()),
        "bias": float(np.median(r) - 1.0),
        "bias_mean": float(np.mean(r) - 1.0),
        "consistency": float(np.std(r)),
        "degenerate": float(1.0 - m.mean()),
    }


def davn2_pnp_scale_seq(cfg, data, device="cuda", max_frames=None,
                        da_model="depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf"):
    """DAv2 metric 深度 + ORB 匹配 + PnP -> s_depth[i] |t_PnP|/|dVO[i]|.

    逐帧: DAv2 (transformers pipeline) 对帧 i+1 出 metric 深度, 帧 i/i+1 ORB 匹配,
    inlier 2D 点用 metric 深度反投影成 3D (帧 i+1 相机系), PnP 解出到帧 i 相机系的
    metric 平移 t_PnP, |t_PnP|/|dVO| 即该帧尺度.
    """
    import cv2
    import torch
    from PIL import Image
    from transformers import AutoModelForDepthEstimation, AutoImageProcessor
    model = AutoModelForDepthEstimation.from_pretrained(da_model).eval().to(device)
    proc = AutoImageProcessor.from_pretrained(da_model)

    def metric_depth(img_rgb):
        inputs = proc(images=Image.fromarray(img_rgb), return_tensors="pt").to(device)
        with torch.no_grad():
            d = model(**inputs).predicted_depth
        d = d.cpu().numpy().squeeze()
        if d.ndim == 1:
            h = int(round(d.size ** 0.5))
            d = d.reshape(h, h)
        h, w = img_rgb.shape[:2]
        return cv2.resize(d.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)

    K = cfg["K"]
    invK = np.linalg.inv(K)
    vo_p = data["vo_p"]
    n = len(vo_p)
    if max_frames:
        n = min(n, max_frames + 1)
    dvo = np.linalg.norm(np.diff(vo_p, axis=0), axis=1)[:n - 1]

    orb = cv2.ORB_create(nfeatures=800)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    s = np.full(n - 1, np.nan)
    inl = np.zeros(n - 1, dtype=int)
    d_stat = np.full((n - 1, 4), np.nan)   # [median, std, p10, p90] metric 深度图 (P2 尺度头特征)
    n_ok = 0
    img_dir = cfg["dir"] / cfg.get("img_dir", "")
    fmt = cfg.get("img_fmt", "{:04d}")
    off = cfg.get("img_off", 1)  # CARLA: 行k↔文件k; KITTI: 行k↔文件k-1
    for i in range(n - 1):
        I0 = cv2.imread(str(img_dir / f"{fmt.format(i+off)}.png"), cv2.IMREAD_GRAYSCALE)
        I1 = cv2.imread(str(img_dir / f"{fmt.format(i+1+off)}.png"), cv2.IMREAD_GRAYSCALE)
        if I0 is None or I1 is None:
            continue
        rgb = cv2.cvtColor(I1, cv2.COLOR_GRAY2RGB)
        D = metric_depth(rgb)
        df = D.ravel()
        d_stat[i] = [np.median(df), np.std(df),
                     np.percentile(df, 10), np.percentile(df, 90)]
        k0, d0 = orb.detectAndCompute(I0, None)
        k1, d1 = orb.detectAndCompute(I1, None)
        if d0 is None or d1 is None or len(d0) < 20 or len(d1) < 20:
            continue
        ms = bf.knnMatch(d0, d1, k=2)
        good = [m for m, nn in ms if m.distance < 0.75 * nn.distance]
        if len(good) < 12:
            continue
        pts0 = np.array([k0[m.queryIdx].pt for m in good], np.float32)
        pts1 = np.array([k1[m.trainIdx].pt for m in good], np.float32)
        uvh = np.hstack([pts1, np.ones((len(pts1), 1))])
        Xc = (invK @ uvh.T).T
        dv = D[pts1[:, 1].astype(int), pts1[:, 0].astype(int)]
        X = Xc * dv[:, None]
        ok, _R, tvec, _inl = cv2.solvePnPRansac(X, pts0, K, None,
                                                reprojectionError=4.0, iterationsCount=200,
                                                confidence=0.99, flags=0)
        if not ok or tvec is None:
            continue
        tm = float(np.linalg.norm(tvec))
        if dvo[i] > 0.02 and np.isfinite(tm) and tm > 0:
            s[i] = tm / dvo[i]
            inl[i] = 0 if _inl is None else int(len(np.asarray(_inl).ravel()))
            n_ok += 1
        if (i + 1) % 250 == 0:
            print(f"    [daV2] {i+1}/{n-1} ok={n_ok}", flush=True)
    return s, inl, d_stat, n_ok, (n - 1)


# --------------------------------------------------------------------------- #
# metrics
# --------------------------------------------------------------------------- #
def _write_depth_file(cfg, s, inl, n_frames):
    """写 depth_scale.txt (EKF 深度尺度支路输入).

    行 k (1-based 帧号) = 区间 k-1→k 的 log 尺度 + PnP 内点数; 帧 1 无前区间 → nan.
    帧号 = ground_truth/visual_odometry 行号 (CARLA 口径: 行 k ↔ 图像 k).
    """
    p = cfg["dir"] / "depth_scale.txt"
    with open(p, "w", encoding="utf-8") as f:
        f.write("# frame log_s inliers\n")
        f.write("# frame k = interval k-1->k (1-based); frame1=nan; log domain; DAv2-PnP (scale_series_p1.py)\n")
        f.write("1 nan 0\n")
        for k in range(2, n_frames + 1):
            v = s[k - 2]
            if np.isfinite(v):
                f.write(f"{k} {float(np.log(v)):.6f} {int(inl[k - 2])}\n")
            else:
                f.write(f"{k} nan 0\n")
    n_ok = int(np.isfinite(s).sum())
    print(f"  [depth-file] 写入 {p} ({n_frames} 帧, 有效 {n_ok})")


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seqs", nargs="+", default=list(SEQUENCES.keys()))
    ap.add_argument("--out", type=str, default=str(OUT_DEFAULT))
    ap.add_argument("--with-davl2", action="store_true")
    ap.add_argument("--davl2-frames", type=int, default=None,
                    help="DAv2 最多跑多少帧 (调试用, 默认全部)")
    ap.add_argument("--write-depth-file", action="store_true",
                    help="把 DAv2 逐帧尺度写入 <数据目录>/depth_scale.txt (供 EKF 深度尺度支路)")
    ap.add_argument("--win", type=int, default=50,
                    help="窗长(帧), 累积位移比/滑动中值的窗口, 默认 50 (=2.5s@20Hz)")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    torch_ok = False
    if args.with_davl2:
        try:
            import torch  # noqa
            torch_ok = torch.cuda.is_available()
            print(f"[daV2] torch available, cuda={torch_ok}")
        except Exception as e:
            print(f"[daV2] torch 不可用, 跳过 DAv2-PnP: {e}")

    report = {}
    for name in args.seqs:
        cfg = SEQUENCES[name]
        if not (cfg["dir"] / cfg["gt"]).exists():
            print(f"[{name}] 跳过: 无 GT 文件 {cfg['dir']/cfg['gt']}")
            continue
        print(f"\n=== {name} ===")
        data = load_seq(cfg)
        n = min(len(data["gt_p"]), len(data["vo_p"]))
        data["gt_p"], data["vo_p"] = data["gt_p"][:n], data["vo_p"][:n]
        print(f"  frames={n}")

        W = args.win
        s_gt, dvo = gt_scale_series(data["gt_p"], data["vo_p"])
        s_gt_win = windowed_median(s_gt, W)
        s_star = global_scale(data["vo_p"], data["gt_p"])
        s_kin = kinematics_scale_series(data["accel"], data["gyro"], data["vo_p"], data["vo_t"])
        s_kin_win = windowed_median(s_kin, W) if s_kin is not None else None

        gm = float(np.nanmedian(s_gt))
        res = {"n_frames": int(n), "win": int(W), "s_star": float(s_star),
               "gt_perframe_median": gm,
               "scale_drift_spread": float(np.nanstd(s_gt) / gm) if gm > 0 else float("nan")}
        res["kinematics_perframe"] = est_metrics(s_kin, s_gt) if s_kin is not None else None
        res["kinematics_windowed"] = est_metrics(s_kin_win, s_gt_win) if s_kin_win is not None else None

        s_depth = None
        s_depth_inl = None
        s_depth_win = None
        d_stat = None
        if args.with_davl2 and torch_ok:
            try:
                s_depth, s_depth_inl, d_stat, n_ok, n_tot = davn2_pnp_scale_seq(
                    cfg, data, device="cuda", max_frames=args.davl2_frames)
                s_depth_win = windowed_median(s_depth, W)
                res["davn2_perframe"] = est_metrics(s_depth, s_gt)
                res["davn2_perframe"]["n_ok"] = int(n_ok)
                res["davn2_windowed"] = est_metrics(s_depth_win, s_gt_win)
                if args.write_depth_file:
                    _write_depth_file(cfg, s_depth, s_depth_inl, n)
            except Exception as e:
                import traceback
                traceback.print_exc()
                res["davn2"] = {"error": str(e)}
                print(f"  [daV2] 失败: {e}")

        report[name] = res
        np.savez_compressed(out / f"scale_series_{name}.npz",
                            s_star=np.array([s_star]), s_gt=s_gt,
                            s_kin=s_kin if s_kin is not None else np.array([]),
                            s_kin_win=s_kin_win if s_kin_win is not None else np.array([]),
                            s_depth=s_depth if s_depth is not None else np.array([]),
                            s_depth_inl=s_depth_inl if s_depth_inl is not None else np.array([]),
                            s_depth_win=s_depth_win if s_depth_win is not None else np.array([]),
                            depth_stat=d_stat if d_stat is not None else np.array([]),
                            frames=np.arange(n))
        _print_row(name, res)
        _plot(name, s_gt, s_kin_win, s_star, s_depth_win, out)

    (out / "p10_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n[done] 报告写入 {out/'p10_report.json'}")


def _fmt(m):
    if not m or "bias" not in m:
        return "n/a"
    return (f"bias={m['bias']:+.3f} 一致性={m['consistency']:.3f} "
            f"valid={m['n_valid']} 退化={m['degenerate']:.2f}")


def _print_row(name, res):
    print(f"  {name}: frames={res['n_frames']} win={res.get('win')} "
          f"s*(全局)={res.get('s_star', float('nan')):.3f} "
          f"GT逐帧中值={res.get('gt_perframe_median', float('nan')):.3f} "
          f"尺度漂移σ/μ={res.get('scale_drift_spread', float('nan')):.2f}")
    if res.get("kinematics_perframe") is not None:
        print(f"    运动学 逐帧: {_fmt(res['kinematics_perframe'])}")
    if res.get("kinematics_windowed") is not None:
        print(f"    运动学 窗  : {_fmt(res['kinematics_windowed'])}")
    if res.get("davn2"):
        d = res["davn2"]
        if "error" in d:
            print(f"    DAv2-PnP: ERROR {d['error'][:80]}")
    if res.get("davn2_perframe") is not None:
        print(f"    DAv2  逐帧: {_fmt(res['davn2_perframe'])}")
    if res.get("davn2_windowed") is not None:
        print(f"    DAv2  窗  : {_fmt(res['davn2_windowed'])}")


def _plot(name, s_gt, s_kin, s_star, s_depth, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"  [plot] 跳过 (matplotlib 不可用): {e}")
        return
    fig, ax = plt.subplots(figsize=(11, 4.5))
    fr = np.arange(len(s_gt))
    ax.plot(fr, s_gt, lw=0.5, alpha=0.55, color="black", label="GT scale (per-frame, ref)")
    ax.axhline(s_star, color="gray", lw=1.2, ls=":", label=f"global s*={s_star:.2f}")
    if s_kin is not None:
        sk = s_kin[:len(fr)]
        ax.plot(fr[:len(sk)], sk, lw=0.7, alpha=0.9, color="tab:blue", label="kinematics (windowed)")
    if s_depth is not None:
        sd = s_depth[:len(fr)]
        ax.plot(fr[:len(sd)], sd, lw=0.7, alpha=0.9, color="tab:red", label="DAv2-PnP (windowed)")
    ax.set_yscale("log")
    ax.set_xlabel("frame")
    ax.set_ylabel("scale s (log)")
    ax.set_title(f"{name}: scale estimators vs GT (P1.0)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / f"scale_series_{name}.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
