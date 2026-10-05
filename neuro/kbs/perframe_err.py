#!/usr/bin/env python3
"""perframe_err.py — 逐帧误差定位: NLM(新,P1ON) vs EKF, 7-DoF 对齐后。
找 NLM 输 EKF 的帧段, 判断是局部尖峰(可用 P1 平滑)还是持续漂移(架构性)。"""
import numpy as np, os, json
exec(open('/home/yangrb/openhutb/neuro/kbs/regen_table.py').read().split("# ---------- 口径验证")[0])
ROOT = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets'

def err_curve(gt, est, anchor=None):
    n = min(len(gt), len(est)); gt = gt[:n]; est = est[:n]
    a = n if anchor is None else min(anchor, n)
    gs = gt[:a] - gt[:a].mean(0); es = est[:a] - est[:a].mean(0)
    H = gs.T @ es
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = S.sum() / max(np.sum(es**2), 1e-12)
    t = gt[:a].mean(0) - (s * R) @ est[:a].mean(0)
    est_a = (s * (R @ est.T)).T + t
    return np.linalg.norm(est_a - gt, axis=1)

datasets = [('Town01Data_IMU_Fusion','Town01'), ('Town02Data_IMU_Fusion','Town02'),
            ('Town05Data_IMU_Fusion','Town05'), ('Town10HDData_IMU_Fusion','Town10HD'),
            ('KITTI07Data_IMU_Fusion','KITTI07')]
out = {}
for ds, key in datasets:
    gt  = load_pos(f'{ROOT}/{ds}/ground_truth.txt')
    nlm = load_pos(f'{ROOT}/{ds}/slam_results/exp_trajectory.txt')
    ekf = load_pos(f'{ROOT}/{ds}/fusion_pose.txt')
    en = err_curve(gt, nlm)
    ee = err_curve(gt, ekf)
    n = min(len(en), len(ee))
    en, ee = en[:n], ee[:n]
    worse = en > ee                      # NLM 比 EKF 差
    # 找连续"更差"段(长度>50帧), 按段内 NLM 均误差排序
    segs = []
    i = 0
    while i < n:
        if worse[i]:
            j = i
            while j < n and worse[j]: j += 1
            if j - i >= 50:
                segs.append((i, j, float(en[i:j].mean()), float(en[i:j].max()), float(ee[i:j].mean())))
            i = j
        else:
            i += 1
    segs.sort(key=lambda s: -s[2])
    out[key] = {
        'NLM_rmse': float(np.sqrt((en**2).mean())), 'EKF_rmse': float(np.sqrt((ee**2).mean())),
        'NLM_worse_frac': float(worse.mean()),
        'NLM_p95': float(np.percentile(en,95)), 'EKF_p95': float(np.percentile(ee,95)),
        'NLM_max': float(en.max()), 'EKF_max': float(ee.max()),
        'worst_segs': [(int(a),int(b),round(c,2),round(d,2),round(e,2)) for a,b,c,d,e in segs[:4]],
    }
    # 存逐帧曲线供画图
    np.savez(f'/tmp/perframe_{key}.npz', frame=np.arange(n), nlm=en, ekf=ee)

for key, v in out.items():
    print(f'=== {key}  NLM={v["NLM_rmse"]:.2f} EKF={v["EKF_rmse"]:.2f}  NLM更差帧占比={v["NLM_worse_frac"]*100:.1f}%')
    print(f'    p95: NLM={v["NLM_p95"]:.2f} EKF={v["EKF_p95"]:.2f} | max: NLM={v["NLM_max"]:.2f} EKF={v["EKF_max"]:.2f}')
    for a,b,cm,cx,em in v['worst_segs']:
        print(f'    更差段 [{a:4d}-{b:4d}] 长{b-a:4d}  NLM均={cm:6.2f} max={cx:6.2f}  (EKF同段={em:6.2f})')
json.dump(out, open('/tmp/perframe_summary.json','w'), indent=1)
print('\n已存 /tmp/perframe_*.npz + /tmp/perframe_summary.json')
