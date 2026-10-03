% FTEST_ONE.m — 闭环漏斗诊断驱动: 逐帧记录每个VT决策点, 定位真闭环episode死在哪一级
% 环境变量: FUNNEL_DS=数据集名 (默认Town10HDData_IMU_Fusion)
% 口径 = 对应 RERUN 基线参数覆盖 + EXP_MAX_LOOP_CE=200 + 持续性修正关闭(纯重锚定)
% 输出: /tmp/funnel/{ds}/funnel.txt  (每行: frame branch min_delta ratio match_id ce age d_xy vt_id)
clear all; close all; clc;

ds0 = getenv('FUNNEL_DS');
if isempty(ds0), ds0 = 'Town10HDData_IMU_Fusion'; end

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = ds0;

global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false;
switch ds0
    case 'KITTI07Data_IMU_Fusion', FAST_TEST_FRAMES = 1101;
    otherwise, FAST_TEST_FRAMES = 5000;
end

global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% ---- 各集参数覆盖(与对应 RERUN_TOWNxx.m 基线口径一致) ----
global VT_MATCH_THRESHOLD_OVERRIDE;
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
global IMU_HEIGHT_WEIGHT_OVERRIDE;
global GC_VT_INJECT_ENERGY_OVERRIDE;
switch ds0
    case 'Town10HDData_IMU_Fusion'
        VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        EXP_LOOPS_OVERRIDE = 12;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
        IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town05Data_IMU_Fusion'
        VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        EXP_LOOPS_OVERRIDE = 12;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
        IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town02Data_IMU_Fusion'
        VT_MATCH_THRESHOLD_OVERRIDE = 0.055;
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 15;
        EXP_LOOPS_OVERRIDE = 10;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.65;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.25;
    otherwise
        % Town01 / KITTI07: 无参数覆盖(core默认), 与RERUN口径一致
end

% ==== 纯重锚定口径 + 漏斗日志开启 ====
global EXP_MAX_LOOP_CE;
EXP_MAX_LOOP_CE = 200;
global NLM_LOOP_FIX_ENABLE;
NLM_LOOP_FIX_ENABLE = false;
global NLM_LOOP_CORR;
NLM_LOOP_CORR = [0, 0, 0];
global FUNNEL_LOG_PATH;
funnel_out = fullfile('/tmp/funnel', ds0);
if ~isfolder(funnel_out), mkdir(funnel_out); end
delete(fullfile(funnel_out, 'funnel.txt'));  % 每次运行全新
FUNNEL_LOG_PATH = {fullfile(funnel_out, 'funnel.txt')};

fprintf('\n================ FTEST(漏斗诊断): %s  CE=200 ================\n', ds0);
test_imu_visual_fusion_slam;

% ---- core的clearvars后重新推导 (全局变量仍可用) ----
ds = getenv('FUNNEL_DS');
if isempty(ds), ds = ds0; end
dataRoot = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
out = fullfile('/tmp/funnel', ds);
if ~isfolder(out), mkdir(out); end
copyfile(fullfile(dataRoot, ds, 'slam_results', 'exp_trajectory.txt'), fullfile(out, 'exp_trajectory.txt'));

try
    global DIAG_VT_REVISIT DIAG_MULTI_CAND DIAG_MULTI_REJECT ...
           DIAG_SINGLE_BELOW DIAG_SINGLE_MATCH DIAG_CE_REJECT DIAG_CE_MAX ...
           DIAG_LOOP_ACCEPT DIAG_YAW_REJECT NLM_LOOP_CORR;
    if isempty(NLM_LOOP_CORR), NLM_LOOP_CORR = [0, 0, 0]; end
    df = fopen(fullfile(out, 'diag_funnel.txt'), 'w');
    fprintf(df, 'ds=%s mode=funnel-diagnostics CE=200 FIX_ENABLE=false\n', ds);
    fprintf(df, 'VT_REVISIT=%d BELOW_THR_S=%d SINGLE_MATCH=%d MULTI_CAND=%d MULTI_REJECT=%d CE_REJECT=%d CE_MAX=%.2f LOOP_ACCEPT=%d YAW_REJECT=%d\n', ...
        DIAG_VT_REVISIT, DIAG_SINGLE_BELOW, DIAG_SINGLE_MATCH, DIAG_MULTI_CAND, ...
        DIAG_MULTI_REJECT, DIAG_CE_REJECT, DIAG_CE_MAX, DIAG_LOOP_ACCEPT, DIAG_YAW_REJECT);
    fclose(df);
    disp('DIAG saved');
catch me
    disp(['DIAG save failed: ' me.message]);
end
disp(['FUNNEL SAVED: ' out]);
disp('FTEST ONE DONE');
