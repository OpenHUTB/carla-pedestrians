#!/usr/bin/env python3
"""loop_event_deltas.py — per-event effect of the loop re-anchoring.

For each accepted loop event of the Town01 turnaround benchmark, report the
anchored-Sim(3) error immediately before the event and at the event frame.
Same caliber as Section 4.5 (similarity fitted on the first 100 frames and
applied unchanged), so the numbers belong to the same measurement.

Usage: python3 neuro/kbs/loop_event_deltas.py [<bundle>/datasets/Town01Turnaround_IMU_Fusion]
"""
import json, os, sys
import numpy as np
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))


def anchored_errors(traj, gt, k=100):
    gs = gt[:k] - gt[:k].mean(0)
    es = traj[:k] - traj[:k].mean(0)
    U, S, Vt = np.linalg.svd(gs.T @ es)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = S.sum() / max(np.sum(es ** 2), 1e-12)
    t = gt[:k].mean(0) - s * R @ traj[:k].mean(0)
    A = (s * (R @ traj.T)).T + t
    return np.linalg.norm(A - gt, axis=1)


def main(seq=None):
    seq = seq or os.environ.get('NLM_TURNAROUND_SEQ') or os.path.join(
        os.environ.get('NLM_DATA_ROOT',
                       os.path.join(HERE, '..', 'data', '01_NeuroSLAM_Datasets')),
        'Town01Turnaround_IMU_Fusion')
    seq = os.path.abspath(seq) + '/'
    gt = np.loadtxt(seq + 'ground_truth.txt', delimiter=',', skiprows=1, ndmin=2)[:, :3]
    fin = np.loadtxt(seq + 'a_final/exp_trajectory.txt', delimiter=',', ndmin=2)[:, :3]
    n = min(len(gt), len(fin))
    gt, fin = gt[:n], fin[:n]
    e = anchored_errors(fin, gt)
    ev = np.atleast_2d(np.asarray(
        sio.loadmat(seq + 'a_final/loop_constraint_events.mat')['NLM_LOOP_EVENTS'], float))
    rows, rise = [], 0
    for f in [int(v) for v in ev[:, 0]]:
        k = f - 1
        if k + 1 >= n:
            continue
        before, after = float(e[max(0, k - 1)]), float(e[k])
        rows.append({'frame': int(f), 'err_before_m': round(before, 2),
                     'err_after_m': round(after, 2), 'delta_m': round(after - before, 2)})
        rise += after > before
    out = {'_meta': {'caliber': 'anchored Sim(3) on first 100 frames, applied unchanged (same as Section 4.5)',
                     'note': 'error immediately before the accepted event vs at the event frame'},
           'n_events': len(rows), 'n_error_increased': int(rise), 'events': rows}
    path = os.path.join(HERE, 'loop_event_deltas.json')
    with open(path, 'w') as fh:
        json.dump(out, fh, indent=2, ensure_ascii=False)
        fh.write('\n')
    print(f'{len(rows)} events, {rise} raise the instantaneous error; worst: '
          f'{max(rows, key=lambda r: r["delta_m"])}')
    print(f'written {path}')
    return out


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
