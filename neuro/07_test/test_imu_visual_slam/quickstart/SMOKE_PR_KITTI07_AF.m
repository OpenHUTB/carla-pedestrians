% SMOKE_PR_KITTI07.m — KITTI07 (真实数据, 1101帧) NLM-PR 约束模式 A/B
% 参数同 RERUN_KITTI07 (核心默认 + EKF-odo) + DINOv2 闭环 + A-fix
clear all; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));
global DATASET_NAME;
DATASET_NAME = 'KITTI07Data_IMU_Fusion';
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 1101;
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;
global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_pr_kitti_af';
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP;
NLM_USE_PR = true;
PR_SIM_THRESHOLD = 0.7; PR_LOOP_RADIUS = 10; PR_ARC_GAP = 50; PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;
NLM_SIM3_WIN = 250;
fprintf('=== PR KITTI07 (tau=0.7 R=10 A-fix) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== PR KITTI07 完成 ===\n');
