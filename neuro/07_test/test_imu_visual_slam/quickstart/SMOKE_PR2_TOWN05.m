% SMOKE_PR2_TOWN05.m — PR 分支约束模式冒烟 (v2: 零地图反馈, A-fix+S2 校正)
% 与 SMOKE_PR_TOWN05.m 同参, 差异:
%   NLM_LOOP_CONSTRAINT=true  → 闭环只记事件, 零地图反馈
%   NLM_SIM3_REANCHOR=true    → 输出层全 Sim(3) 重锚 (S2, win=250)
% 运行: cd quickstart && matlab -batch SMOKE_PR2_TOWN05
clear all; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town05Data_IMU_Fusion';

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

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

global RESULT_SUBDIR;
RESULT_SUBDIR = 'slam_results_pr2_smoke';

% ── PR 分支 (DINOv2 场景嵌入闭环, 约束模式) ──
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP;
NLM_USE_PR = true;
PR_SIM_THRESHOLD = 0.7;   % tau
PR_LOOP_RADIUS = 10;      % R (m)
PR_ARC_GAP = 50;          % 弧长间隔 (m)
PR_MIN_GAP = 30;          % 事件去抖 (帧)

% ── A-fix 约束漂移场 + S2 全 Sim(3) 重锚 (输出层校正) ──
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = true;
NLM_SIM3_WIN = 250;

fprintf('=== PR2 冒烟 Town05 (tau=0.7, R=10m, arc>50m, A-fix+S2, 约束模式) ===\n');
test_imu_visual_fusion_slam;
fprintf('=== PR2 冒烟完成 ===\n');
