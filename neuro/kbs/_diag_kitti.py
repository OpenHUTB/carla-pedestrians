import numpy as np
from scipy.io import loadmat

sub = 'neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion'

def f(x):
    x = np.asarray(x)
    while x.dtype == object:
        x = np.asarray(x.ravel()[0])
    return float(np.ravel(x).flat[0])

def loadp(p, cols, skip=0):
    a = np.loadtxt(p, delimiter=',', skiprows=skip)
    return a[:, list(cols)]

gt  = loadp(sub+'/ground_truth.txt', (1,2,3), skip=1)
ekf = loadp(sub+'/fusion_pose.txt', (1,2,3), skip=1)
e_nlm  = loadp(sub+'/dc_final/exp_trajectory.txt', (0,1,2))
e_base = loadp(sub+'/slam_results/exp_trajectory.txt', (0,1,2))
L = np.sqrt((np.diff(gt,0)**2).sum(1)).sum()

def p7(g,e):
    n=min(len(g),len(e)); g=g[:n]; e=e[:n]
    gs=g-g.mean(0); es=e-e.mean(0)
    U,S,Vt=np.linalg.svd(gs.T@es); D=np.diag([1,1,np.sign(np.linalg.det(U@Vt))]); R=U@D@Vt
    s=S.sum()/max((es**2).sum(),1e-12); t=g.mean(0)-(s*R)@e.mean(0)
    return (s*(R@e.T)).T+t

def anchored100(g,e):
    n=min(len(g),len(e)); g=g[:n]; e=e[:n]
    k=100
    gs=g[:k]-g[:k].mean(0); es=e[:k]-e[:k].mean(0)
    U,S,Vt=np.linalg.svd(gs.T@es); D=np.diag([1,1,np.sign(np.linalg.det(U@Vt))]); R=U@D@Vt
    s=S.sum()/max((es**2).sum(),1e-12); t=g[:k].mean(0)-(s*R)@e[:k].mean(0)
    return (s*(R@e.T)).T+t

print(f'GT path length = {L:.0f} m, {len(gt)} frames')
print(f'{"metric":28s} {"ATE":>8s} {"end":>8s} {"end%":>6s} {"first100":>9s} {"last100":>8s}')
for name, e, fn in [('EKF', ekf, p7), ('NLM_dc_final', e_nlm, p7), ('NLM_base', e_base, p7)]:
    A = fn(gt, e); n=min(len(gt),len(A)); d=A[:n]-gt[:n]; err=np.sqrt((d**2).sum(1))
    print(f'{name+" (7dof)":28s} {np.sqrt((err**2).mean()):8.2f} {err[-1]:8.2f} {err[-1]/L*100:6.1f} {err[:100].mean():9.2f} {err[-100:].mean():8.2f}')
print('-'*75)
for name, e in [('EKF', ekf), ('NLM_dc_final', e_nlm), ('NLM_base', e_base)]:
    A = anchored100(gt, e); n=min(len(gt),len(A)); d=A[:n]-gt[:n]; err=np.sqrt((d**2).sum(1))
    print(f'{name+" (anch100)":28s} {np.sqrt((err**2).mean()):8.2f} {err[-1]:8.2f} {err[-1]/L*100:6.1f} {err[:100].mean():9.2f} {err[-100:].mean():8.2f}')

# ---- experiences: nodes / links / loop edges ----
raw = loadmat(sub+'/dc_final/experiences.mat')['EXPERIENCES']
if isinstance(raw, np.ndarray) and raw.dtype == object:
    raw = raw.ravel()[0]
E = raw
n = len(E)
ex = np.array([[f(E[i]['x_exp']), f(E[i]['y_exp']), f(E[i]['z_exp'])] for i in range(n)])
links=[]
for i in range(n):
    nl = int(f(E[i]['numlinks']))
    for j in range(nl):
        tid = int(f(E[i]['links'][j]['exp_id']))
        dxy = f(E[i]['links'][j]['d_xy'])
        links.append((i,tid,dxy))
nloop = sum(1 for i,t,dl in links if t<i)
dl = np.array([dl for i,t,dl in links])
print(f'\nnodes={n}  links={len(links)}  loop_edges(back-edges)={nloop}')
print(f'link d_xy: median={np.median(dl):.2f}  p90={np.percentile(dl,90):.2f}  max={np.max(dl):.2f}')
print(f'node bbox x[{ex[:,0].min():.0f},{ex[:,0].max():.0f}] y[{ex[:,1].min():.0f},{ex[:,1].max():.0f}]')
print(f'GT    bbox x[{gt[:,0].min():.0f},{gt[:,0].max():.0f}] y[{gt[:,1].min():.0f},{gt[:,1].max():.0f}]')

# ---- re-anchoring events from match_log ----
M = loadmat(sub+'/dc_final/match_log.mat')['DIAG_MATCH_LOG']
M = np.asarray(M, float)
if M.ndim == 2 and M.shape[0] > M.shape[1]:
    M = M.T if M.shape[1] == 9 else M
print(f'\n=== re-anchoring events (dc_final): {M.shape[0]} ===')
print('frame   from -> to   age    ce(m)')
for r in M:
    print(f'{int(r[0]):5d}  {int(r[1]):3d} -> {int(r[2]):3d}   {int(r[3]):4d}  {r[8]:8.3f}')

# ---- dc_events (re-anchoring jumps used by DC field) ----
de = loadmat(sub+'/dc_final/dc_events.mat')
ev = np.asarray(de['dc_events'], float).reshape(-1, 4)
print(f'\n=== DC re-anchor jumps: {ev.shape[0]}  total norm = {np.sqrt((ev[:,1:]**2).sum(1)).sum():.2f} m ===')
for r in ev:
    print(f'frame {int(r[0]):5d}  jump={np.sqrt((r[1:]**2).sum()):7.3f} m')
