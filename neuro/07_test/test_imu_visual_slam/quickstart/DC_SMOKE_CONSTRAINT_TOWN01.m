% DC_SMOKE_CONSTRAINT_TOWN01.m — A-fix 闭环约束漂移场冒烟测试(2026-10-06)
% 验证: 语法/全局变量/事件记录(born_frame+NLM_FRAME_IDX)/约束漂移场注入/
% 事件落盘 全链路无运行错误; 100帧内若无真闭环(预期事件=0), 也验证事件集为
% 空时的注入分支安全。
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

% A-fix 约束模式(与 DC_FINAL 'constraint' 同一组开关)
global NLM_LOOP_CONSTRAINT NLM_LOOP_FIX_ENABLE NLM_LOOP_CORR;
NLM_LOOP_CONSTRAINT = true;
NLM_LOOP_FIX_ENABLE = true;
NLM_LOOP_CORR = [0, 0, 0];

global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_a_smoke';

fprintf('=== A-fix 约束冒烟测试 Town01 (%d帧) ===\n', SMOKE_FRAMES);
test_imu_visual_fusion_slam;
fprintf('=== DC_SMOKE_CONSTRAINT_TOWN01 完成 ===\n');
