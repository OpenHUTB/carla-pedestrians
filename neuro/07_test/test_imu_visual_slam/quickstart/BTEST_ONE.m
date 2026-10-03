% BTEST_ONE.m — 纯重锚定重跑单数据集 (CE门200m, 持续性修正关闭)
% 环境变量: SWEEP_DS=数据集名
% 参数口径 = 对应 RERUN_TOWNxx.m 的参数覆盖(基线口径) + EXP_MAX_LOOP_CE=200 + NLM_LOOP_FIX_ENABLE=false
% 输出: /tmp/b_fix/{ds}/ {exp_trajectory.txt, experiences.mat, trajectories.mat, diag_funnel.txt, diag_match_log.txt}
clear all; close all; clc;

ds0 = getenv('SWEEP_DS');
if isempty(ds0), error('SWEEP_DS 环境变量未设置'); end

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

% ==== 纯重锚定: CE门放宽200m(放真闭环进来), 持续性修正关闭 ====
global EXP_MAX_LOOP_CE;
EXP_MAX_LOOP_CE = 200;
global NLM_LOOP_FIX_ENABLE;
NLM_LOOP_FIX_ENABLE = false;
global NLM_LOOP_CORR;
NLM_LOOP_CORR = [0, 0, 0];

fprintf('\n================ BTEST(纯重锚定): %s  CE=200 ================\n', ds0);
test_imu_visual_fusion_slam;

% ---- core的clearvars后重新推导 (全局变量仍可用) ----
ds = getenv('SWEEP_DS');
dataRoot = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
out = fullfile('/tmp/b_fix', ds);
if ~isfolder(out), mkdir(out); end
copyfile(fullfile(dataRoot, ds, 'slam_results', 'exp_trajectory.txt'), fullfile(out, 'exp_trajectory.txt'));
copyfile(fullfile(dataRoot, ds, 'slam_results', 'experiences.mat'), fullfile(out, 'experiences.mat'));
copyfile(fullfile(dataRoot, ds, 'slam_results', 'trajectories.mat'), fullfile(out, 'trajectories.mat'));
disp('RESULT files saved');

try
    global DIAG_VT_REVISIT DIAG_MULTI_CAND DIAG_MULTI_REJECT ...
           DIAG_SINGLE_BELOW DIAG_SINGLE_MATCH DIAG_CE_REJECT DIAG_CE_MAX ...
           DIAG_LOOP_ACCEPT DIAG_YAW_REJECT DIAG_MATCH_LOG NLM_LOOP_CORR;
    if isempty(NLM_LOOP_CORR), NLM_LOOP_CORR = [0, 0, 0]; end
    df = fopen(fullfile(out, 'diag_funnel.txt'), 'w');
    fprintf(df, 'ds=%s mode=pure-reanchor CE=200 FIX_ENABLE=false\n', ds);
    fprintf(df, 'VT_REVISIT=%d BELOW_THR_S=%d SINGLE_MATCH=%d MULTI_CAND=%d MULTI_REJECT=%d CE_REJECT=%d CE_MAX=%.2f LOOP_ACCEPT=%d YAW_REJECT=%d CORR_NORM=%.2f\n', ...
        DIAG_VT_REVISIT, DIAG_SINGLE_BELOW, DIAG_SINGLE_MATCH, DIAG_MULTI_CAND, ...
        DIAG_MULTI_REJECT, DIAG_CE_REJECT, DIAG_CE_MAX, DIAG_LOOP_ACCEPT, DIAG_YAW_REJECT, norm(NLM_LOOP_CORR));
    fclose(df);
    if ~isempty(DIAG_MATCH_LOG)
        dlmwrite(fullfile(out, 'diag_match_log.txt'), DIAG_MATCH_LOG, 'precision', 3);
    end
    disp('DIAG saved');
catch me
    disp(['DIAG save failed: ' me.message]);
end
disp(['SAVED: ' out]);
disp('BTEST ONE DONE');
