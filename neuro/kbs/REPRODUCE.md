# Reproducing the paper — script map

Every number in the manuscript traces to one of the scripts below. Paths are
relative to the repository root; scripts are intended to be run from
`neuro/kbs/` (most of them `chdir` to their own location, so either works).

Environment: `pip install -r neuro/requirements.txt` (PyTorch is needed only for
the two learned heads; `transformers` only for DINOv2 embeddings).

---

## 0. Pre-flight

| Step | Command | Checks |
|---|---|---|
| dataset structure | `python3 neuro/kbs/check_datasets.py` | the five per-sequence files the metrics need (`ground_truth.txt`, `fusion_pose.txt`, `visual_odometry.txt`, `slam_results/trajectories.mat`, `slam_results/experiences.mat`); exit code 0 = ready |

---

## 1. Table by table

### Table 1 — dataset characteristics
Not a computed table: lengths and frame counts come from the collection logs,
and the template/node/link counts are the ones written into
`neuro/kbs/final_metrics.json` by step 2 below (`NUM_EXPS`, template count,
accepted-link count per sequence).

### Table 2 — observation branches and thresholds
The thresholds are source constants, not a produced table. Authoritative file:
`neuro/00_collect_data/IMU_Vision_Fusion_EKF.py`, constant block at the top
(`R_heading`, `R_vel`, `R_att`, `R_pos`, `R_Z_PRIOR`, `SCALE_R_BASE`,
`DEPTH_SCALE_R`, `DEPTH_SCALE_DEADBAND`, `POSITION_WIN`, `HEADING_MAX_STEP`,
`TURN_GATE_*`, `VO_SCALE_Q`, `Q_cont`, `P0`). The chi-square thresholds sit in
the same class (`chi2_heading`, `chi2_vel`, `chi2_att`, `chi2_scale`,
`chi2_pos`). The heading branch is switched by `HEADING_OBS_ENABLED`, which is
`False` in the configuration that produced the reported numbers.

### Table 3 — main results (5 scenarios × 2 calibers)
```bash
python3 neuro/kbs/se3_metrics.py               # -> se3_metrics.json : Table 3, all four configurations × both calibers
python3 neuro/kbs/compute_final_metrics.py     # -> final_metrics.json : loop-line caliber only (Table 1 counts + §4.4 ablation row)
```

**Three trajectory directories feed these numbers, and only the first is the
full system.** Prepare them per sequence before running either script:

| directory / file | configuration it holds | used by |
|---|---|---|
| `slam_results_scale_nlm/trajectories.mat` | **full system** (EKF core + scale head + loop line) — this is the `Ours` column of Table 3 | `se3_metrics.py` → `NLM-全系统` |
| `slam_results/trajectories.mat` | loop line only, no scale branch | `se3_metrics.py` → `NLM-闭环线`; `compute_final_metrics.py` → `nlm` |
| `slam_results_orbonly/trajectories.mat` | loop line with classical ORB descriptors (§4.4 ablation) | `se3_metrics.py` → `ORB-only` |
| `_p11_backup_20261014/<seq>/fp_head030.txt` | EKF core + scale head, no loop (dead-band 0.30 replay) | `se3_metrics.py` → `EKF+尺度` |

Consequently `final_metrics.json` reports the **loop-line** value (Town05:
41.14 m), not the 18.07 m quoted in Table 3; the full-system column must be
read from `se3_metrics.json`. Run both scripts, and compare against
`se3_metrics.json` for Table 3 and against `final_metrics.json` only for the
Table 1 counts and the "loop line over the unscaled EKF core" ablation row.

MATLAB cross-check of the same two calibers: `neuro/kbs/crosscheck_sim3.m`.
Per-frame error curves for a single sequence: `python3 neuro/kbs/perframe_err.py`.

### Ablation (§4.4)
```bash
# scale branch on/off, and the deployed dead-band setting
python3 neuro/kbs/replay_depth_ab.py --sets Town05 Town03 Town07 Town10HD \
        --depth-file depth_scale_head.txt --deadband 0.30 --tag head030
python3 neuro/kbs/replay_depth_ab.py --sets Town05 --nodp --tag nodp
```
MATLAB re-anchoring variants: `kb_replay_ab.m`, `kb_replay_ab_simple.m`.
Fixed-R ablation is a runtime flag inside the filter (`fixed_r`), which makes
`_r_scale` return 1.0.

### Scale branch (§3.6)
```bash
python3 neuro/kbs/scale_series_p1.py      # per-frame GT / kinematic / depth scale series -> scale_p1/scale_series_<seq>.npz
python3 neuro/kbs/pr_scale_head.py        # train-once on Town05/03/07 -> pr_scale_head.pt
python3 neuro/kbs/predict_kitti_depth_head.py   # zero-shot on KITTI 07 (the failure reported in S.1)
```

### Loop-closure line (§3.8)
```bash
python3 neuro/kbs/pr_dinov2_embed.py      # frozen DINOv2-S embeddings per frame (needs `transformers`)
python3 neuro/kbs/pr_train_head.py        # train-once metric-learning head -> pr_head.pt
python3 neuro/kbs/pr_eval_finetuned.py    # fine-tuned embeddings + P/R on all five sequences
python3 neuro/kbs/tau_scan.py             # similarity-threshold sweep behind tau = 0.94
python3 neuro/kbs/diag_pr_gates.py        # gate-by-gate attribution of every true/false loop
```
Gate constants used by the MATLAB detector: `HART_INPUT_GATE / HART_FORGET_GATE /
HART_OUTPUT_GATE` (defaults 0.5 / 0.5 / 0.9); grid search: `grid_search_gates.sh`.

### Closed-loop benchmark (§4.5, 842.54 m → 55.92 m)
Route: Town01 **turnaround** (out-and-back on the same streets), re-collected as
an empty town — same sensor suite, 20 Hz logging and 12 km/h speed command as the
five main sequences, but no background traffic, so the measured drift is
attributable to the loop machinery alone. Length 881 m / 500 s / 10000 frames,
realized average 6.3 km/h; 36 accepted loop-closure events.

Caliber: **anchored Sim(3)** — the similarity transform is fitted on the first
100 frames and then applied unchanged to the whole trajectory (stricter than the
full-Sim(3) refit of Table 3, which would absorb the drift the benchmark is meant
to expose). MATLAB implementation: `neuro/kbs/kb_s2_se3_check.m`.

Inputs, as that script expects them, are four whitespace-free CSV files of
`x,y,z` columns written to `/tmp` (one row per frame, truncated to the common
length):

| file | content |
|---|---|
| `/tmp/se3_gt.txt` | ground truth (`Town01Turnaround_IMU_Fusion/ground_truth.txt`) |
| `/tmp/se3_pres2.txt` | trajectory before the loop correction (A only) |
| `/tmp/se3_final.txt` | deployed trajectory (A + loop re-anchoring) |
| `/tmp/se3_ekf.txt` | EKF-core trajectory (`fusion_pose.txt`) |
| `/tmp/se3_events.txt` | accepted loop frames, one index per line (`a_final/loop_constraint_events.mat`) |

**Known gap**: the export step that turns the `.mat`/`.txt` logs into the five
`/tmp/se3_*.txt` files is not yet a script in this repository — it was run by
hand when the benchmark was produced, and `/tmp` is wiped between sessions.
Until `export_closed_loop_inputs.py` lands, the three numbers can be re-derived
from the files above but not by a single command. The route itself is part of
the released asset bundle, not of this repository.

---

## 2. Figures

```bash
python3 neuro/kbs/regen_figures.py         # representative_performance.pdf, performance_summary.pdf, ablation/visual-template, KITTI in/out
python3 neuro/kbs/make_all_results_fig.py  # 5-scenario Sim(3)/SE(3) overview + KITTI external RTE
python3 neuro/kbs/regen_ekf_curves.py      # EKF error curves used in the failure analysis
```
All figures land in `neuro/kbs/fig/` and are the files `\includegraphics` pulls
in the LaTeX sources.

---

## 3. Manuscript sources

`NeuroSLAM_KBS.tex` (main, elsarticle) inputs `sec_related_work.tex`,
`sec_method_expanded.tex`, `experiment_section.tex`. Build:

```bash
pdflatex NeuroSLAM_KBS.tex && bibtex NeuroSLAM_KBS \
  && pdflatex NeuroSLAM_KBS.tex && pdflatex NeuroSLAM_KBS.tex
```

Supplementary: `supplementary_KBS.tex` (includes `supplementary.tex`).
Cover letter: `cover_letter_KBS.tex`. Highlights: `highlights.tex`.

---

## 4. Known gaps

| Gap | Status |
|---|---|
| processed CARLA sequences not hosted | available on request (tens of GB) |
| closed-loop route/event log not hosted | available on request |
| EKF gate thresholds not swept by a shipped script | fixed before the ablation runs, values are source constants |
| `*.m` legacy scripts | kept for provenance; published numbers come from the Python path |
