% P2AB_TOWN07_ORBONLY.m — C类同批重跑 (Town07, 全量5000帧, 独立MATLAB进程)
% 口径: 与 RERUN_TOWN05 同参 (VT=0.040, HDC=18, loops=12, corr=0.5,
%       IMU 0.70/0.30/0.6, GC=0.4, NLM_USE_EKF_ODO=true), 全量帧(非1000快速)
% 运行: cd quickstart && matlab -batch P2AB_TOWN07_ORBONLY
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town07Data_IMU_Fusion';

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


% ── P2 定解 A/B (帧基候选 + 微调嵌入, 仅A-fix, S2关) ──
global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_orbonly';   % [诊断] ORB-only 独立目录
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = false;   % [诊断] ORB-only 对照 (新数据基线)
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.9;
PR_LOOP_RADIUS = 1e9;
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;   % S2 关 (Town05 实测有害: +2.72m)
NLM_SIM3_WIN = 250;

fprintf('=== P2定解A/B Town07 (全量5000帧, NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== P2AB_TOWN07_ORBONLY 完成 ===\n');
