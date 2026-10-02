% NLM_DIAG_TOWN05.m — 一次性诊断脚本(Town05, 1000帧快速模式)
% 运行: cd /home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/quickstart && matlab -batch NLM_DIAG_TOWN05
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town05Data_IMU_Fusion';

% 快速测试模式: 1000帧
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 1000;

% 路线①: NLM运动源切换为EKF融合轨迹
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% 与 RUN_SLAM_TOWN05.m 相同的参数覆盖(保证口径一致)
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 12;
EXP_CORRECTION_OVERRIDE = 0.5;
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.70;
IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
global IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
global GC_VT_INJECT_ENERGY_OVERRIDE;
GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;

fprintf('=== NLM回归: Town05 1000帧快速模式 ===\n');
test_imu_visual_fusion_slam;
fprintf('=== NLM回归完成 ===\n');
