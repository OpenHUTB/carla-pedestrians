% NLM_SWEEP_NEWFOOT_ONE.m — 单组 (VT, CE) 扫描运行器 (独立MATLAB进程, 由 bash 循环驱动)
% 环境变量: SWEEP_DS=数据集  SWEEP_VT=VT阈值  SWEEP_CE=CE门
% 口径: 与 RERUN_TOWNxx.m 基线一致, 仅覆盖 VT/CE (FIX 固定=0)
% 归档: /tmp/nlm_sweep_newfoot/{ds}/vt{x}_ce{y}/exp_trajectory.txt
% 用环境变量传参而非函数局部变量: core 的 clearvars 会清掉函数基区变量
clear all; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

ds0  = getenv('SWEEP_DS');
vt0  = str2double(getenv('SWEEP_VT'));
ce0  = str2double(getenv('SWEEP_CE'));
if isempty(ds0) || isnan(vt0) || isnan(ce0)
    error('用法: SWEEP_DS=... SWEEP_VT=... SWEEP_CE=... matlab -batch NLM_SWEEP_NEWFOOT_ONE');
end

% ---- 各集基线参数覆盖 (与 RERUN_TOWNxx.m 一致) ----
if strcmp(ds0, 'Town10HDData_IMU_Fusion')
    global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
    global EXP_LOOPS_OVERRIDE;
    global EXP_CORRECTION_OVERRIDE;
    global IMU_YAW_WEIGHT_OVERRIDE;
    global IMU_TRANS_WEIGHT_OVERRIDE;
    global IMU_HEIGHT_WEIGHT_OVERRIDE;
    global GC_VT_INJECT_ENERGY_OVERRIDE;
    DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
    EXP_LOOPS_OVERRIDE = 12;
    EXP_CORRECTION_OVERRIDE = 0.5;
    IMU_YAW_WEIGHT_OVERRIDE = 0.70;
    IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
    IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
    GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
end

global DATASET_NAME FAST_TEST_MODE FAST_TEST_FRAMES NLM_USE_EKF_ODO;
global VT_MATCH_THRESHOLD_OVERRIDE;
global EXP_MAX_LOOP_CE NLM_LOOP_FIX_ENABLE NLM_LOOP_CORR;
DATASET_NAME = ds0;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;
NLM_USE_EKF_ODO = true;
VT_MATCH_THRESHOLD_OVERRIDE = vt0;
EXP_MAX_LOOP_CE = ce0;
NLM_LOOP_FIX_ENABLE = false;
NLM_LOOP_CORR = [0, 0, 0];

t0 = tic;
test_imu_visual_fusion_slam;

% core 的 clearvars 后从 env 重新推导路径 (全局变量已清, 但 env 还在)
ds0  = getenv('SWEEP_DS');
vt0  = str2double(getenv('SWEEP_VT'));
ce0  = str2double(getenv('SWEEP_CE'));
tag = sprintf('vt%s_ce%d', num2str(vt0), ce0);
root = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
out = fullfile('/tmp/nlm_sweep_newfoot', ds0, tag);
if ~isfolder(out), mkdir(out); end
copyfile(fullfile(root, ds0, 'slam_results', 'exp_trajectory.txt'), fullfile(out, 'exp_trajectory.txt'));
fprintf('\n>>> SAVED %s  (%.0fs)\n', out, toc(t0));
disp('SWEEP_ONE DONE');
