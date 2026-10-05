% DC_TOWN01.m — Town01 全量5000帧, DC(分布式闭环漂移校正) ON 对照实验
% 口径: 与 RERUN_TOWN01.m 完全一致(vt08/ce20, P1 ON, NLM_USE_EKF_ODO=true),
%       仅叠加 NLM_DC_ENABLE=true; 结果写入独立目录, 不污染 slam_results 基线。
% 运行: cd quickstart && ~/matlab_batch.sh DC_TOWN01
% 基线参照: slam_results (P1ON, DC关) Town01 7-DoF ATE 15.59 / EKF 15.94
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

global NLM_DC_ENABLE;
NLM_DC_ENABLE = true;

global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_dc_on_1004';

fprintf('=== DC对照实验 Town01 (全量5000帧, NLM_USE_EKF_ODO=true, NLM_DC_ENABLE=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== DC_TOWN01 完成 ===\n');
