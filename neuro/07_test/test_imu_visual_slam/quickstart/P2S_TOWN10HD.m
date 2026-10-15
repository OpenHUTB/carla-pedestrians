% P2S_TOWN10HD.m — C类同批重跑 (Town10HD, 全量5000帧, 独立MATLAB进程)
% 口径: 原RUN_SLAM_TOWN10.m参数覆盖(数据集Town10HDData_IMU_Fusion) + NLM_USE_EKF_ODO=true(路线①)
% 运行: cd quickstart && matlab -batch P2S_TOWN10HD
%
% 【2026-10-10 恢复EKF地基后 重扫定解】
%   新地基 EKF=13.4m(7-DoF Sim(3) ATE), 旧配置 vt0.04/ce20 = 13.28m(与EKF打平)。
%   VT×CE 20组扫描(/tmp/nlm_sweep_newfoot, kbs/sweep_metrics.py):
%     vt0.12/ce10 = 12.74m ← 最优, 胜 EKF 5.0%
%   CE 方向收紧有利, 本集 VT 敏感度低(全网格 12.7~13.4m)。
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town10HDData_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% Visual Template (10-12 新数据弱集扫参定解: vt0.02/ce10 = 4.34m, 胜 EKF 4.35;
% 10-10 旧数据重扫 vt0.12/ce10=12.74m 不再适用)
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.02;
global EXP_MAX_LOOP_CE;
EXP_MAX_LOOP_CE = 10;

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
RESULT_SUBDIR = 'slam_results_scale_nlm';   % [尺度线NLM层A/B] 独立目录防覆盖
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = true;
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.94;     % τ定解 (同 P2FT 五集A/B, 尺度线NLM层一致性)
PR_LOOP_RADIUS = 1e9;
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;   % S2 关 (Town05 实测有害: +2.72m)
NLM_SIM3_WIN = 250;

fprintf('=== P2定解A/B Town10HD (全量5000帧, NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== P2S_TOWN10HD 完成 ===\n');
