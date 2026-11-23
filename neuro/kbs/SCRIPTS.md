# Script index — `neuro/kbs/`

Every `*.py` in this directory, grouped by role. Only the first group is part of
the published reproduction pipeline (see `REPRODUCE.md`); the other two are kept
for provenance so that no number in the paper is orphaned from the code that
produced it.

Data root convention for all groups: `NLM_DATA_ROOT` (defaults to `../data/01_NeuroSLAM_Datasets`);
`NLM_KITTI_DIR`, `NLM_BACKUP_ROOT` where noted in `REPRODUCE.md`.

## 1. Reproduction pipeline (documented in `REPRODUCE.md`)

| script | role |
|---|---|
| `check_datasets.py` | data layout pre-check for the five sequences |
| `se3_metrics.py` | rigid SE(3) ATE matrix (Table 4 SE(3) half) + KITTI 09 |
| `compute_final_metrics.py` | Table 4 Sim(3) half: full-Sim(3) RMSE/drift per method |
| `export_repro_bundle.py` | collect the artefacts the bundle is missing; --check-only audits |
| `export_closed_loop_inputs.py` | export the five se3_*.txt inputs for kb_s2_se3_check.m |
| `run_all_checks.py` | one command: recompute every number in the paper against the data |
| `check_baseline.py` | guard against regressions of the baseline numbers (--offline) |
| `regen_table.py` | recompute Table-4-style metrics from trajectories (imported by perframe_err.py) |
| `regen_figures.py` | redraw the four main figures |
| `make_all_results_fig.py` | redraw the five-scenario overview + KITTI RTE figure |
| `loop_event_deltas.py` | per-event effect of the loop re-anchoring on the closed-loop benchmark (Section 4.5 transparency) |
| `kitti_rte.py` | Table 6, our rows: official KITTI relative translation error (raw / Sim(3)-aligned / after dropping top-50 frames) |
| `regen_ekf_curves.py` | redraw the EKF error curves of the failure analysis |
| `perframe_err.py` | per-frame error localisation, NLM vs EKF (needs full run trees) |

## 2. Learned components (Sec. 3.5, Suppl. S.1)

| script | role |
|---|---|
| `pr_dinov2_embed.py` | DINOv2-S per-frame embeddings (loop-closure line input) |
| `pr_mine_loops.py` | mines ground-truth loop labels for training |
| `pr_train_head.py` | trains the lightweight head on frozen DINOv2 features |
| `pr_eval_finetuned.py` | recomputes fine-tuned embeddings and the Phase-0 P/R evaluation |
| `pr_scale_head.py` | trains the monocular scale-correction head |
| `predict_kitti_depth_head.py` | zero-shot scale prediction on KITTI 07 with the shipped head |
| `replay_depth_ab.py` | EKF replay A/B over depth inputs |
| `scale_series_p1.py` | three-way scale series and bias measurement (needs the original work tree under `NLM_WORK_ROOT`, not the released bundle) |
| `diag_pr_gates.py` | offline replica of the MATLAB PR-detector gate chain |

## 3. One-off analyses and sweeps (not in the paper pipeline)

Ad-hoc diagnostics, sweeps and assembly helpers used while the experiments were
running. They are not required to reproduce any reported number; several expect a
local run tree, a CARLA instance or `/tmp` state from the original session, and a
few have no docstring because they were scratch scripts. Their presence here is
documented rather than hidden:

| script | role |
|---|---|
| `afix_prototype.py` | A-fix field-shape prototype (Python, does not invoke MATLAB) |
| `dc_review.py` | review material for NLM DC v3: per-frame error comparison plots over five sequences (base NLM / DC v3 / EKF) plus a metric summary |
| `dc_sweep.py` | offline sweep of the distributed closed-loop drift field (DC v3) in the 7-DoF Procrustes caliber used by the paper tables |
| `nlm_eval.py` | NLM regression evaluation helper (2026-10-04) |
| `promote_t94_final.py` | one-shot promotion of the τ=0.94 solution to final (2026-10-14) |
| `sweep_metrics.py` | aggregate ATE over the (VT, CE) combinations under /tmp/nlm_sweep_newfoot |
| `sweep_weak_sets.py` | targeted sweep on the weak sets (Town03/Town07/Town10HD), 2026-10-12 |
| `tar_afix_sweep.py` | offline sweep of the A-fix closed-loop constraint drift field on the Turnaround sequence |
| `tau_scan.py` | fine scan of the PR-branch similarity threshold τ (0.90–0.98, event-level detail) |
| `turnaround_sim3_eval.py` | A/B variants of the Turnaround sequence converted to the paper caliber (first-100-frame anchored Sim(3) 7-DoF Procrustes, 3D) |
| `route2_dyaw_events.py` | route 2: measure the visual relative pose (dyaw) for every closed-loop event pair recorded by NLM |
| `pr_ab_sim3.py` | no description in source |
| `pr_eval.py` | DL-NLM Phase 0 zero-shot loop evaluation (DINOv2 vs ORB vs oracle) |
| `pr_export_embeddings.py` | DINOv2 embeddings, npy → MATLAB .mat (for the PR branch) |
| `pr_geom_gate.py` | Phase 0.5: geometric verification gate for DINOv2 loop candidates (SIFT + RANSAC essential matrix) |
| `pr_mapdist_gate.py` | Phase 0.5c: straight-line map-distance gate (the correct loop discriminator) |
| `pr_oracle_loops.py` | true-loop upper bound (DL-NLM Phase 0, event level) |
| `pr_oracle_revisit.py` | oracle revisit upper bound (DL-NLM Phase 0) |
| `pr_pnp_gate.py` | Phase 0.5b: PnP geometric gate on pose hypotheses (discriminative-ceiling test) |
| `diag_bidir_drive.py` | no description in source |
| `diag_collector_seq.py` | no description in source |
| `diag_planner_route.py` | no description in source |
| `probe_bidir.py` | find a bidirectional turnaround start point that forms a true loop on a given CARLA port / map |
| `scan_bidir_routes.py` | no description in source |
| `validate_bidir_candidates.py` | no description in source |
| `xodr_bidir_scan.py` | offline OpenDRIVE parsing to find candidate bidirectional turnaround start points |
| `kitti09_assemble_raw.py` | unpack the raw 0033_sync.zip results into the input layout expected by kitti_prepare_fusion.py |
| `kitti_prepare_fusion.py` | KITTI conversion pipeline: raw KITTI odometry data → standard IMU_Fusion layout plus a true visual-inertial EKF |
| `_diag_kitti.py` | no description in source |
| `_improve_exp.py` | no description in source |

## 4. Tests and smoke checks (need CARLA, not run in CI)

| script | role |
|---|---|
| `test_behavior_roundtrip.py` | no description in source |
| `test_deploy_mech.py` | no description in source |
| `test_flip_roundtrip.py` | no description in source |
| `test_roundtrip.py` | no description in source |
| `test_spawn_z.py` | no description in source |
| `test_vehicle_diff.py` | no description in source |
| `validate_dataset.py` | dataset integrity check: GT jumps / VO saturation / frame counts / timestamp continuity / row-count agreement |

`validate_dataset.py` also serves as an offline consistency check of a run tree
(GT jumps, VO saturation, frame counts, timestamp monotonicity).

## 5. Diagram sources (`src/`)

Early draft drawing scripts for the conceptual diagrams. Their outputs
(`bio_nav_architecture.pdf`, `experience_map.pdf`, `grid_cell_activity.pdf`,
`hdc_network.pdf`) are **not** the files compiled in the manuscript: the three
method figures (`2_framework.pdf`, `3d_grid_cell_network.pdf`,
`imu_visual_complementary_fusion.pdf`) and the graphical abstract (`1_abstract.pdf`)
were exported from the `.pptx` sources in `fig/`, not from these scripts.

| script | note |
|---|---|
| `draw_bio_architecture_compact.py` / `..._elegant.py` / `..._elegant_v2.py` / `..._final.py` / `draw_bio_architecture.py` / `..._v2.py` | six iterations of the same architecture drawing |
| `draw_grid_cell_network.py` | grid-cell diagram draft (`grid_cell_activity.pdf`) |
| `draw_method_diagrams.py` | experience-map / HDC blocks (`experience_map.pdf`, `hdc_network.pdf`) |
| `draw_neuroslam_architecture.py` | pipeline diagram draft |
