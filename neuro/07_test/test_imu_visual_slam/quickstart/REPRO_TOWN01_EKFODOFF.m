% REPRO_TOWN01_EKFODOFF.m — 裁决用: 复现 10-03 基线
% 假设: 10-03 14:17 那批用的是 EKF-odo 关闭 + 无任何 global 覆盖 (即老式 MATLAB IMU-VO 运动源)
% 与 RERUN_TOWN01 (EKF-odo 开 + VT/EXP 覆盖) 对照, 看哪个能复现 1170m/68节点
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

% 显式关闭 EKF-odo, 不设任何 VT/EXP/IMU 覆盖 = 走 core 默认 (MATLAB IMU-VO 运动源)
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = false;
global NLM_REANCHOR_SMOOTH_N NLM_SPIKE_CLIP_THR;
NLM_REANCHOR_SMOOTH_N = 0;
NLM_SPIKE_CLIP_THR = 0;

% 单独存放, 不覆写 slam_results
global RESULT_SUBDIR;
RESULT_SUBDIR = 'repro_ekfodoff';

fprintf('=== 裁决: Town01 EKF-odo OFF, 无覆盖, 对照 10-03 基线 (期望 1170m/68节点) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== 完成, 结果在 repro_ekfodoff/ ===\n');
