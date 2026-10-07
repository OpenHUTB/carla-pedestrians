import numpy as np
from scipy.io import loadmat

sub = 'neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion'
gt   = np.loadtxt(sub+'/ground_truth.txt',  delimiter=',', skiprows=1)[:, 1:4]
base = np.loadtxt(sub+'/slam_results/exp_trajectory.txt', delimiter=',')   # NLM_base, DC off
ev   = np.asarray(loadmat(sub+'/dc_final/dc_events.mat')['dc_events'], float).reshape(-1, 4)
N = len(base)
Lgt = np.sqrt((np.diff(gt,0)**2).sum(1)).sum()
print(f'N={N}  GT len={Lgt:.0f} m  jumps={len(ev)}  sum|c|={np.sqrt((ev[:,1:]**2).sum(1)).sum():.1f} m')

def align(g, e, anchor=0):
    n = min(len(g), len(e)); g = g[:n]; e = e[:n]
    k = n if anchor == 0 else min(anchor, n)
    gs = g[:k] - g[:k].mean(0); es = e[:k] - e[:k].mean(0)
    U, S, Vt = np.linalg.svd(gs.T @ es)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); R = U @ D @ Vt
    s = S.sum() / max((es**2).sum(), 1e-12)
    t = g[:k].mean(0) - (s * R) @ e[:k].mean(0)
    return (s * (R @ e.T)).T + t

def metric(g, e, anchor=0):
    A = align(g, e, anchor); n = min(len(g), len(A)); err = np.sqrt(((A[:n]-g[:n])**2).sum(1))
    return np.sqrt((err**2).mean()), err[-1]

def staircase(ev):
    C = np.zeros((N, 3))
    for r in ev:
        fr, c = int(r[0]), r[1:4]
        C[fr-1:] += c
    return C

def field_v3(ev, W0=400, Wpm=80, Wmax=4000, CMAX=np.inf, tail=True):
    """reproduce DC v3: ramp starts AT jump, holds after."""
    d = np.zeros((N, 3))
    for r in ev:
        fr, c = int(r[0]), r[1:4]
        if np.linalg.norm(c) > CMAX: continue
        f0 = fr - 1
        Wi = min(max(W0 + Wpm*np.linalg.norm(c), W0), Wmax)
        if tail: Wi = min(Wi, N - f0)
        ramp = np.arange(0, N) >= f0
        t = np.arange(N) - f0
        F = np.zeros(N)
        m = (t >= 0) & (t < Wi)
        F[m] = 0.5*(1-np.cos(np.pi*t[m]/max(Wi,1e-9)))
        F[t >= Wi] = 1.0
        d += F[:, None] * c
    return d

def field_dist(ev, L0=300, Lpm=20, Lmax=3000, CMAX=np.inf, drop=np.inf, tail=True):
    """DC v2: ramp ENDS at jump (correction approached along drift path)."""
    d = np.zeros((N, 3))
    for r in ev:
        fr, c = int(r[0]), r[1:4]
        mag = np.linalg.norm(c)
        if mag > drop:        # reject low-confidence (oversized) jumps
            continue
        if mag > CMAX: continue
        f0 = fr - 1
        Li = min(max(L0 + Lpm*mag, L0), Lmax)
        if tail: Li = min(Li, max(f0, 1))
        start = f0 - Li
        t = np.arange(N) - start
        F = np.zeros(N)
        m = (t >= 0) & (t < Li)
        F[m] = 0.5*(1-np.cos(np.pi*t[m]/max(Li,1e-9)))
        F[t >= Li] = 1.0
        d += F[:, None] * c
    return d

C = staircase(ev)
dr = base - C          # jump-free concatenated-DR base

print(f'\n{"method":34s} {"ATE7":>7s} {"end7":>7s} {"ATEa":>7s} {"enda":>7s}')
def rep(name, out):
    a7, e7 = metric(gt, out, 0); aa, ea = metric(gt, out, 100)
    print(f'{name:34s} {a7:7.2f} {e7:7.2f} {aa:7.2f} {ea:7.2f}')

rep('base (raw, DC off)', base)
rep('v3 current (W0=400,Wpm=80)', dr + field_v3(ev))
rep('v3 + CMAX=8 (gate big)',    dr + field_v3(ev, CMAX=8))
rep('drop top1 (17m@102)',       dr + field_dist(ev, drop=17.5, L0=1))
rep('drop top3 (>8m)',           dr + field_dist(ev, drop=8.0, L0=1))
rep('v2 dist all (L0=300,Lpm=20)', dr + field_dist(ev))
rep('v2 dist small+drop big',    dr + field_dist(ev, drop=8.0))
rep('v2 dist small, L0=100,Lpm=10', dr + field_dist(ev, drop=8.0, L0=100, Lpm=10))
rep('v2 dist small, L0=500,Lpm=40', dr + field_dist(ev, drop=8.0, L0=500, Lpm=40))
rep('v2 dist small, L0=1500,Lpm=100', dr + field_dist(ev, drop=8.0, L0=1500, Lpm=100))
print('\nmetric: ATE7=end-7dof Procrustes; ATEa=anchor-first-100 Sim(3)')
