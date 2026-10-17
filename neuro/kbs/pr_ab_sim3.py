import sys, numpy as np, pandas as pd, scipy.io as sio
D='../data/01_NeuroSLAM_Datasets/'
def sim3_full(est, gt):
    n=min(len(est),len(gt)); est,gt=est[:n],gt[:n]
    ec,gc=est-est.mean(0),gt-gt.mean(0)
    U,S,Vt=np.linalg.svd(gc.T@ec); d=np.linalg.det(Vt.T@U.T)
    T=Vt.T@np.diag([1,1,d])@U.T
    s=(S@np.array([1,1,d]))/(ec**2).sum()
    ea=s*(est@T)+(gt.mean(0)-s*(est.mean(0)@T))
    err=np.linalg.norm(ea-gt,axis=1)
    L=float(np.linalg.norm(np.diff(gt,axis=0),axis=1).sum())
    return float(np.sqrt((err**2).mean())), float(err[-1]/L*100)
def run(dset, sub):
    gt=pd.read_csv(D+dset+'/ground_truth.txt')[['pos_x','pos_y','pos_z']].values
    m=sio.loadmat(D+dset+'/'+sub+'/trajectories.mat')
    nlm=np.array(m['exp_trajectory'],dtype=float)
    r,dr=sim3_full(nlm,gt)
    print(f"  {dset:26s} {sub:28s}  RMSE={r:8.2f}m  drift={dr:5.2f}%  (n={len(nlm)})")
pairs=[
 ('Town05Data_IMU_Fusion','slam_results'),          # 基线
 ('Town05Data_IMU_Fusion','slam_results_pr2_smoke'),# A-fix+S2
 ('Town05Data_IMU_Fusion','slam_results_pr3_smoke'),# 仅A-fix
 ('KITTI07Data_IMU_Fusion','slam_results_pr_kitti'),# A-fix+S2
 ('KITTI07Data_IMU_Fusion','slam_results_pr_kitti_af'),# 仅A-fix
]
for d,s in pairs:
    try: run(d,s)
    except Exception as e: print(f"  {d:26s} {s:28s}  ERR {e}")
