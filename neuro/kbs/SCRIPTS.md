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
| `scale_series_p1.py` | three-way scale series and bias measurement |
| `diag_pr_gates.py` | offline replica of the MATLAB PR-detector gate chain |

## 3. One-off analyses and sweeps (not in the paper pipeline)

Ad-hoc diagnostics, sweeps and assembly helpers used while the experiments were
running. They are not required to reproduce any reported number; several expect a
local run tree, a CARLA instance or `/tmp` state from the original session, and a
few have no docstring because they were scratch scripts. Their presence here is
documented rather than hidden:

| script | role |
|---|---|
| `afix_prototype.py` | A-fix 字段形态原型 (Python, 不跑 MATLAB) |
| `dc_review.py` | NLM DC v3 审阅材料: 5集逐帧误差对比图(base NLM / DC v3 / EKF) + 指标汇总 |
| `dc_sweep.py` | 分布式闭环漂移场(DC v3)离线扫参, 7-DoF Procrustes 口径(论文Table口径) |
| `nlm_eval.py` | NLM 回归评测工具 (2026-10-04) |
| `promote_t94_final.py` | τ=0.94 定解一键定稿 (2026-10-14) |
| `sweep_metrics.py` | 汇总 /tmp/nlm_sweep_newfoot 各 (VT,CE) 组合的 ATE |
| `sweep_weak_sets.py` | 弱集 (Town03/Town07/Town10HD) 定向扫参, 2026-10-12 |
| `tar_afix_sweep.py` | Turnaround 折返集 A-fix 闭环约束漂移场离线扫参 |
| `tau_scan.py` | PR 分支相似度阈值 τ 细扫 (0.90-0.98, 事件级明细) |
| `turnaround_sim3_eval.py` | 折返集 A/B 换算到论文口径(首100帧锚定 Sim(3) 7-DoF Procrustes, 3D)。 |
| `route2_dyaw_events.py` | 路线2: 对 NLM 记录的全部闭环事件对测量视觉相对位姿 (dyaw) |
| `pr_ab_sim3.py` | no description in source |
| `pr_eval.py` | DL-NLM Phase 0 零样本闭环评估 (DINOv2 vs ORB vs oracle) |
| `pr_export_embeddings.py` | DINOv2 嵌入 npy → MATLAB .mat (PR 分支用) |
| `pr_geom_gate.py` | Phase 0.5: DINOv2 候选闭环的几何验证门 (SIFT + RANSAC 本质矩阵) |
| `pr_mapdist_gate.py` | Phase 0.5c: 地图直线距离门 (正确的闭环判别器) |
| `pr_oracle_loops.py` | 真闭环上限 (DL-NLM Phase 0, 事件级) |
| `pr_oracle_revisit.py` | oracle 回访上限 (DL-NLM Phase 0) |
| `pr_pnp_gate.py` | Phase 0.5b: 基于位姿假设的 PnP 几何验证门 (判别力天花板测试) |
| `diag_bidir_drive.py` | no description in source |
| `diag_collector_seq.py` | no description in source |
| `diag_planner_route.py` | no description in source |
| `probe_bidir.py` | 在指定 CARLA 端口/地图上找一条"真闭环"双向折返起点。 |
| `scan_bidir_routes.py` | no description in source |
| `validate_bidir_candidates.py` | no description in source |
| `xodr_bidir_scan.py` | 离线解析 OpenDRIVE, 找"双向折返真闭环"候选起点。 |
| `kitti09_assemble_raw.py` | 从 raw 0033_sync.zip 解压结果组装 kitti_prepare_fusion.py 输入布局 |
| `kitti_prepare_fusion.py` | KITTI 转换管线: 原始 KITTI odometry 数据 → 标准 IMU_Fusion 格式 + 真视觉惯性 EKF |
| `kitti_rte.py` | KITTI 官方相对平移误差 (RTE %) 侦察/计算 (缺口1<->缺口2 耦合证据) |
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
| `validate_dataset.py` | 数据集完整性校验: GT 跳变 / VO 封顶 / 帧数 / 时间戳连续性 / 文件行数一致性。 |

`validate_dataset.py` also serves as an offline consistency check of a run tree
(GT jumps, VO saturation, frame counts, timestamp monotonicity).
