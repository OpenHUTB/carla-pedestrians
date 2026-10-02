% NLM_DIAG_TOWN01.m — 一次性诊断脚本(Town01, 5000帧全量)
% 生死门: NLM_USE_EKF_ODO=true 下, Town01闭环数是否>0
% 运行: cd /home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/quickstart && matlab -batch NLM_DIAG_TOWN01
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';

% 全量5000帧(关闭快速测试模式)
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

% 路线①: NLM运动源切换为EKF融合轨迹
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

fprintf('=== NLM生死门: Town01 5000帧全量 (NLM_USE_EKF_ODO=true) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== NLM生死门诊断完成 ===\n');
