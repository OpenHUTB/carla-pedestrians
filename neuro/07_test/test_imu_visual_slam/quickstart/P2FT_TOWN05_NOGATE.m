% P2FT_TOWN05_NOGATE.m — C类同批重跑 (Town05, 全量5000帧, 独立MATLAB进程)
% 口径: 原RUN_SLAM_TOWN05.m参数覆盖 + NLM_USE_EKF_ODO=true(路线①), 全量帧(非1000快速)
% 运行: cd quickstart && matlab -batch P2FT_TOWN05_NOGATE
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town05Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% Visual Template
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.040;

% Experience Map
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;

% Map Relaxation
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 12;
EXP_CORRECTION_OVERRIDE = 0.5;

% IMU融合
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.70;
IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
global IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;

% Grid Cell VT注入
global GC_VT_INJECT_ENERGY_OVERRIDE;
GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;


% ── P2 微调 A/B: DINOv2 微调嵌入 + 闭环约束模式 (A-fix+S2, 零地图反馈) ──
global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_p2ft_nogate';
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = true;
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.85;
PR_LOOP_RADIUS = 1e9;   % P2: 去掉地图距离门 -- 漂移地图上不可靠(真环地图距离>>10m 被挡, 假环反而近), 靠微调嵌入精度+朝向/长间隔门
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = true;
NLM_SIM3_WIN = 250;

fprintf('=== P2微调A/B Town05 (全量5000帧, NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== P2FT_TOWN05_NOGATE 完成 ===\n');
