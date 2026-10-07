% NLM_TURNAROUND_TOWN01_CONSTRAINT.m — 折返(真闭环)数据 A/B: DC注入+闭环约束(A-fix)
% 口径: 与 NLM_TURNAROUND_TOWN01_DC.m 逐行一致, 仅 NLM_LOOP_CONSTRAINT 开;
%       结果 → a_final/ (不污染 slam_results 基线)
% 运行: ~/matlab_batch.sh NLM_TURNAROUND_TOWN01_CONSTRAINT
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town01Turnaround_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% ===== DC v3 漂移场注入 (与 NLM_TURNAROUND_TOWN01 定解一致) =====
global NLM_DC_ENABLE NLM_DC_COLLECT_ONLY;
global NLM_DC_W0 NLM_DC_W_PER_M NLM_DC_W_MAX NLM_DC_TAIL_COMPLETE NLM_DC_CMAX;
NLM_DC_ENABLE = true;
NLM_DC_COLLECT_ONLY = false;
NLM_DC_W_MAX = 4000;
NLM_DC_TAIL_COMPLETE = true;
NLM_DC_W0 = 100;  NLM_DC_W_PER_M = 0;   NLM_DC_CMAX = 2.0;

% 闭环约束(A-fix): 开启 — 真闭环事件记录 + 输出层约束漂移场注入
global NLM_LOOP_CONSTRAINT NLM_LOOP_FIX_ENABLE;
global NLM_LOOP_CORR;
NLM_LOOP_CONSTRAINT = true;
NLM_LOOP_FIX_ENABLE = true;
NLM_LOOP_CORR = [0, 0, 0];

global RESULT_SUBDIR;
RESULT_SUBDIR = 'a_final';

fprintf('=== Turnaround A/B: constraint (DC+闭环约束, 10000帧) → a_final/ ===\n');
test_imu_visual_fusion_slam;
fprintf('=== NLM_TURNAROUND_TOWN01_CONSTRAINT 结束 ===\n');
