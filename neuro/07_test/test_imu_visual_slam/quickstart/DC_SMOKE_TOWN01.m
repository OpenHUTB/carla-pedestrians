% DC_SMOKE_TOWN01.m — DC代码冒烟测试(100帧, 验证语法/全局变量/事件收集无运行错误)
% 帧数由 SMOKE_FRAMES 控制; 若基线首帧重锚定晚于100帧, 可上调覆盖首锚点
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
SMOKE_FRAMES = 100;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = SMOKE_FRAMES;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

global NLM_DC_ENABLE;
NLM_DC_ENABLE = true;

global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_dc_smoke';

fprintf('=== DC冒烟测试 Town01 (%d帧) ===\n', SMOKE_FRAMES);
test_imu_visual_fusion_slam;
fprintf('=== DC_SMOKE_TOWN01 完成 ===\n');
