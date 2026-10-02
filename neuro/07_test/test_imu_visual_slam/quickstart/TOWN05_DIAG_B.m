% TOWN05_DIAG_B.m — 对照实验B: 正常匹配(th=18)+禁迭代修正(loops=0)
% 对照原配置(th=18, loops=12 → ATE 297m): 若B≈47.6则闭环修正是主因; 若B仍≈297则匹配/锚点跳变是主因
clear all; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));
global DATASET_NAME; DATASET_NAME = 'Town05Data_IMU_Fusion';
global FAST_TEST_MODE FAST_TEST_FRAMES; FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
global NLM_USE_EKF_ODO; NLM_USE_EKF_ODO = true;
global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
global EXP_LOOPS_OVERRIDE; global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 0; EXP_CORRECTION_OVERRIDE = 0.5;
global IMU_YAW_WEIGHT_OVERRIDE; global IMU_TRANS_WEIGHT_OVERRIDE; global IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30; IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
global GC_VT_INJECT_ENERGY_OVERRIDE; GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
fprintf('=== DIAG_B: Town05 匹配开+修正关 ===\n');
test_imu_visual_fusion_slam;
fprintf('=== DIAG_B 完成 ===\n');
