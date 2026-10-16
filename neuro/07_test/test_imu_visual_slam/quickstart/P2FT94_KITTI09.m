% P2FT94_KITTI09.m — τ=0.94 定解口径零样本迁移 (KITTI09 1591帧, 独立MATLAB进程)
% 口径: 与 P2FT94_KITTI07 完全一致 (CARLA core默认参数 + NLM_USE_EKF_ODO=true)
% 数据: neuro/data/01_NeuroSLAM_Datasets/KITTI09Data_IMU_Fusion
%       (由 kitti09_assemble_raw.py + kitti_prepare_fusion.py 组装, 真VIO EKF)
% 运行: cd quickstart && matlab -batch P2FT94_KITTI09
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'KITTI09Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 1591;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;


% ── P2 定解 (帧基候选 + 微调嵌入, 仅A-fix, S2关) — 与 07 同参数零样本迁移 ──
global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_p2ft_t94';
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = true;
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.94;     % τ定解 (2026-10-14 五集A/B), 09 零样本迁移不动
PR_LOOP_RADIUS = 1e9;
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;   % S2 关 (Town05 实测有害: +2.72m)
NLM_SIM3_WIN = 250;

fprintf('=== P2定解 KITTI09 (1591帧, NLM_USE_EKF_ODO=true, 零样本迁移) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== P2FT94_KITTI09 完成 ===\n');
