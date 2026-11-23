# Reproducing the paper — script map

Every number in the manuscript traces to one of the scripts below. Paths are
relative to the repository root; scripts are intended to be run from
`neuro/kbs/` (most of them `chdir` to their own location, so either works).

Environment: `pip install -r neuro/requirements.txt`; for the learned components
(scale-head training, DINOv2 embeddings) additionally
`pip install -r neuro/requirements-learned.txt`.

---

## 0. Pre-flight

| Step | Command | Checks |
|---|---|---|
| dataset structure | `python3 neuro/kbs/check_datasets.py` | the five per-sequence files the metrics need (`ground_truth.txt`, `fusion_pose.txt`, `visual_odometry.txt`, `slam_results/trajectories.mat`, `slam_results/experiences.mat`); exit code 0 = ready |

Re-run log against the released bundle (2026-11, six sequences): `check_datasets.py`
passes on all; `compute_final_metrics.py` and `se3_metrics.py` reproduce the
committed numbers **cell for cell** — loop-line 41.14 / 22.91 / 39.49 / 18.61 /
18.45 m (Sim(3)), `EKF` and `VO` columns identical to Table 4, cross-check
`PASS`, closed-loop terminal drift 842.54 → 55.92 m (15.07×). What the bundle
does *not* contain is listed in §1, "What the released asset bundle covers".

---

## 1. Table by table

Manuscript table numbering (as compiled in `NeuroSLAM_KBS.pdf`): Table 1 = related-work
comparison, Table 2 = observation branches and thresholds, Table 3 = dataset
characteristics, Table 4 = main results, Table 5 = per-module runtime, Table 6 =
external comparison. The internal JSON keys `table1` / `table3` in
`expected_values.json` still refer to the datasets / results tables respectively.

### Table 3 — dataset characteristics
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

### Supplementary numbers (S.1 / S.2)
`neuro/kbs/supplementary_metrics.json` records the figures quoted in the supplementary
material. **S.2** (KITTI-09) is derivable from `se3_metrics.json` (the
`KITTI09Data_IMU_Fusion` row, `carried_row: true`): loop line 42.37 / 54.63 m vs EKF
42.34 / 54.77 m (Sim(3) / SE(3)). **S.1** (KITTI-07 true-VIO replay) comes from an internal
batch whose trajectories are **not** part of the released bundle, so those numbers
(17.75 / 44.01 / 23.13 / 23.12 / 40.99 m) cannot be recomputed here; they are recorded in
the same JSON for traceability and are available from the authors on request.

### Table 5 — per-module runtime and throughput
The timing numbers (67--76~FPS on the four CARLA towns, 39~FPS / 25.6~ms on
KITTI~07, and the per-module shares in Table~4) come from full `matlab -batch`
wall-clock runs plus one MATLAB-profiler pass on a 250-frame KITTI~07 subset;
the underlying measurements are recorded in
`neuro/kbs/FPS_MEASUREMENT_EVIDENCE_20261023.md` (per-town frame counts, wall
clock, ms/frame, profiler shares). They are **not** recomputable from the
released asset bundle: they need the raw frames and a MATLAB install on the
reference hardware (Intel Core i9-14900HX, 32~GB RAM, CPU only). Per-module
shares are quoted from the profiler pass because profiler overhead inflates
absolute times (34.4 vs 25.6~ms/frame); the end-to-end FPS numbers are from the
clean wall-clock runs.

### Table 4 — main results (5 scenarios × 2 calibers)
```bash
python3 neuro/kbs/se3_metrics.py               # -> se3_metrics.json : Table 4, all four configurations × both calibers
python3 neuro/kbs/compute_final_metrics.py     # -> final_metrics.json : loop-line caliber only (Table 3 counts + §4.4 ablation row)
```

Both scripts honour `NLM_DATA_ROOT`, so the released bundle can be evaluated in place
without copying it into `neuro/data/`:

```bash
NLM_DATA_ROOT=<bundle>/datasets python3 neuro/kbs/compute_final_metrics.py
NLM_DATA_ROOT=<bundle>/datasets python3 neuro/kbs/se3_metrics.py
```

**Three trajectory directories feed these numbers, and only the first is the
full system.** Prepare them per sequence before running either script:

| directory / file | configuration it holds | used by |
|---|---|---|
| `slam_results_scale_nlm/trajectories.mat` | **full system** (EKF core + scale head + loop line) — this is the `Ours` column of Table 4 | `se3_metrics.py` → `NLM-全系统` |
| `slam_results/trajectories.mat` | loop line only, no scale branch | `se3_metrics.py` → `NLM-闭环线`; `compute_final_metrics.py` → `nlm` |
| `slam_results_orbonly/trajectories.mat` | loop line with classical ORB descriptors (§4.4 ablation) | `se3_metrics.py` → `ORB-only` |
| `_p11_backup_20261014/<seq>/fp_head030.txt` | EKF core + scale head, no loop (dead-band 0.30 replay) | `se3_metrics.py` → `EKF+尺度` |

Consequently `final_metrics.json` reports the **loop-line** value (Town05:
41.14 m), not the 18.07 m quoted in Table 4; the full-system column must be
read from `se3_metrics.json`. Run both scripts, and compare against
`se3_metrics.json` for Table 4 and against `final_metrics.json` only for the
Table 3 counts and the "loop line over the unscaled EKF core" ablation row.

Data roots are overridable, so the released asset bundle can be evaluated
without copying it into `neuro/data/`:

```bash
NLM_DATA_ROOT=/path/NLM_Repro_Assets_*/datasets \
NLM_BACKUP_ROOT=/path/_p11_backup_20261014 \
python3 neuro/kbs/se3_metrics.py
```

### What the released asset bundle covers (checked 2026-11)

`se3_metrics.json` marks every cell it computed from the data root with no
flag, and every cell carried over from an earlier batch with `"carried": true`
(listed in `_meta.configs_carried`). Running `se3_metrics.py` over
`NLM_Repro_Assets_*` reproduces, cell by cell:

| column | status in the asset bundle | note |
|---|---|---|
| `EKF`, `VO` (both calibers) | **recomputed** | 15 cells, all match Table 4 to the printed precision |
| `NLM-闭环线` (both calibers) | **recomputed** | 41.14 / 46.37 (Town05), 18.45 / 29.99 (KITTI 07) — this is also the `Ours` cell on KITTI 07 |
| `Ours` on 4 CARLA sets (18.07 / 15.31 / 28.75 / 14.44 Sim(3)) | carried | needs `slam_results_scale_nlm/`, not in the bundle |
| `EKF+尺度` (20.10 / 16.07 / 27.90 / 14.44) | carried | needs `_p11_backup_20261014/<seq>/fp_head030.txt` |
| `ORB-only` (44.72 on Town05) | carried | needs `slam_results_orbonly/` |
| KITTI 09 row (Supplementary S.2) | carried | KITTI 09 is not in the bundle at all |

To close the three carried groups, run this on the machine that holds the
original run tree — it copies what the evaluator needs (inputs, the closed-loop
`slam_results/`, and the missing artifacts), verifies row counts against the
ground truth, and writes a sha256 manifest. `--src` accepts the run tree
(`<src>/01_NeuroSLAM_Datasets/...`), the released bundle root
(`<src>/datasets/...`), or its `datasets/` sub-directory — the level is
auto-detected:

```bash
python3 neuro/kbs/export_repro_bundle.py --src /path/to/runs --out neuro/kbs/repro_assets --check-only  # dry run
python3 neuro/kbs/export_repro_bundle.py --src /path/to/runs --out neuro/kbs/repro_assets
```

`neuro/kbs/repro_assets/` is `.gitignore`-exempt (about 13 MB) and is read
directly by the evaluator, so no copying into `neuro/data/` is needed:

```bash
NLM_DATA_ROOT=neuro/kbs/repro_assets/01_NeuroSLAM_Datasets \
NLM_BACKUP_ROOT=neuro/kbs/repro_assets/_p11_backup_20261014 \
python3 neuro/kbs/se3_metrics.py
```

The expected `--src` layout is documented in the script header
(`01_NeuroSLAM_Datasets/<seq>/slam_results_*` plus `_p11_backup_20261014/<short>/fp_head030.txt`).
Once those artifacts exist, the `carried` cells turn into recomputed cells on
the next `se3_metrics.py` run — no edits to the JSON are needed.

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
full-Sim(3) refit of Table 4, which would absorb the drift the benchmark is meant
to expose). The quantity reported in §4.5 is the **terminal error** (last-frame
position error after the anchored fit), *not* the RMSE over the whole trajectory.
`kb_s2_se3_check` prints both and labels the caliber, because an earlier version
returned RMSE only and therefore printed 87.25 / 477.76 instead of the paper's
55.92 / 842.54.

```bash
python3 neuro/kbs/export_closed_loop_inputs.py \
        --seq ../data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion --out /tmp
# then, in MATLAB:
#   kb_s2_se3_check            % or kb_s2_se3_check('/tmp')
```

`export_closed_loop_inputs.py` writes the five `/tmp/se3_*.txt` files that the
MATLAB script reads (whitespace-free `x,y,z`, one row per frame, truncated to the
common length; events one frame index per line). Verified values on the released
bundle:

| file | content | terminal (paper) | RMSE (diagnostic) |
|---|---|---|---|
| `se3_final.txt` | deployed trajectory (A + loop re-anchoring) | **55.92 m** | 87.25 m |
| `se3_ekf.txt` | EKF core (`fusion_pose.txt`) | **842.54 m** | 477.76 m |
| `se3_pres2.txt` | before the loop correction (A only) | 565.31 m (not reported) | 331.66 m |

Ground truth length 881.5 m over 10000 frames; 36 accepted events
(`a_final/loop_constraint_events.mat`, `NLM_LOOP_EVENTS` = 36 rows). Ratio
842.54 / 55.92 = 15.07, the "15x" of §4.5.

---

## 2. Figures

All figure scripts read the same data root as the metric scripts; point it at the
released bundle (or at a full run tree) before running them:

```bash
export NLM_DATA_ROOT=<repro_assets>/datasets        # <seq>/ with GT, fusion_pose, VO, slam_results
export NLM_KITTI_DIR=<repro_assets>/KITTI_07        # optional, defaults to $NLM_DATA_ROOT/../KITTI_07
export NLM_BACKUP_ROOT=<repro_assets>/_p11_backup_20261014   # only for the ablation figure

python3 neuro/kbs/regen_figures.py         # representative_performance, performance_summary, ablation, KITTI in/out
python3 neuro/kbs/make_all_results_fig.py  # 5-scenario Sim(3)/SE(3) overview + KITTI external RTE
python3 neuro/kbs/regen_ekf_curves.py      # EKF error curves used in the failure analysis
```

Two caveats, both scripted to fail loudly rather than silently:

- `ablation_unified.pdf` also needs the EKF+scale replay
  (`$NLM_BACKUP_ROOT/Town05/fp_head030.txt`), which the small asset bundle does not
  carry. With the bundle alone the script prints `[SKIP] ablation_unified.pdf`
  and still writes the other three figures; run
  `export_repro_bundle.py` first to obtain that file.
- `perframe_err.py` needs the complete run tree (`slam_results/exp_trajectory.txt`),
  not the `.mat`-only bundle; missing sequences are skipped with a message.

All figures land in `neuro/kbs/fig/` and are the files `\includegraphics` pulls
in the LaTeX sources. `neuro/kbs/SCRIPTS.md` indexes every script in this
directory and says which ones belong to the published pipeline.

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
| processed CARLA sequences not hosted | minimal asset bundle (six sequences, ground truth + EKF/VO + loop-line trajectories) is released; full raw frames available on request |
| `Ours` column on the 4 CARLA sets, `EKF+尺度` column, `ORB-only` ablation | artifacts not in the released bundle; use `export_repro_bundle.py` on the original run tree to package them, then re-run `se3_metrics.py` |
| KITTI 09 row (Supplementary S.2) | sequence not in the released bundle; row is carried over from the 2026-10 batch |
| EKF gate thresholds not swept by a shipped script | fixed before the ablation runs, values are source constants |
| `*.m` legacy scripts | kept for provenance; published numbers come from the Python path |

`se3_metrics.json` records this itself: computed cells are plain, carried cells
carry `"carried": true`, and `_meta.configs_carried` lists them.
