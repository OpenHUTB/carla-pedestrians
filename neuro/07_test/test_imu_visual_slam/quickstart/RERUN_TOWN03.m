% RERUN_TOWN03.m — C类同批重跑 (Town03, 全量5000帧, 独立MATLAB进程)
% 口径: 与 RERUN_TOWN05 同参 (VT=0.040, HDC=18, loops=12, corr=0.5,
%       IMU 0.70/0.30/0.6, GC=0.4, NLM_USE_EKF_ODO=true), 全量帧(非1000快速)
% 运行: cd quickstart && matlab -batch RERUN_TOWN03
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town03Data_IMU_Fusion';

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

fprintf('=== C类重跑 Town03 (全量5000帧, NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== RERUN_TOWN03 完成 ===\n');
