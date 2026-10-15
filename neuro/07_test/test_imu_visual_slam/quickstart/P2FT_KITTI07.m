% P2FT_KITTI07.m — KITTI07 重跑 (1101帧, 独立MATLAB进程)
% 口径: CARLA core默认参数 + NLM_USE_EKF_ODO=true (与4图同批重跑一致)
% 数据: neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion (由 kitti07_assemble.py 组装)
% 运行: cd quickstart && matlab -batch P2FT_KITTI07
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'KITTI07Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 1101;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;


% ── P2 定解 A/B (帧基候选 + 微调嵌入, 仅A-fix, S2关) ──
global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_p2ft';
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = true;
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.94;     % τ定解 (2026-10-14 五集A/B: 4胜1平0负, 杀假环0.907-0.935/留强真环0.949+)
PR_LOOP_RADIUS = 1e9;
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;   % S2 关 (Town05 实测有害: +2.72m)
NLM_SIM3_WIN = 250;

fprintf('=== P2定解A/B KITTI07 (1101帧, NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== P2FT_KITTI07 完成 ===\n');
