% SWEEP_CE_ONE.m — 运行单个 (数据集, CE门) 组合 (每组独立MATLAB进程, 由shell循环驱动)
% 环境变量: SWEEP_DS=数据集名  SWEEP_CE=CE门(m)
% 注意: core脚本的 clearvars -except 会清掉基区脚本变量(ds/ce/out/dataRoot),
%       但全局变量(DATASET_NAME/EXP_MAX_LOOP_CE/DIAG_*)存活 → core调用后从env重新推导路径
clear all; close all; clc;
ds0 = getenv('SWEEP_DS');
ce0 = str2double(getenv('SWEEP_CE'));
if isempty(ds0) || isempty(ce0)
    error('SWEEP_DS / SWEEP_CE 环境变量未设置');
end

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME FAST_TEST_MODE FAST_TEST_FRAMES NLM_USE_EKF_ODO EXP_MAX_LOOP_CE
DATASET_NAME = ds0;
FAST_TEST_MODE = false;
FAST_TEST_FRAMES = 5000;
NLM_USE_EKF_ODO = true;
EXP_MAX_LOOP_CE = ce0;

fprintf('\n================ SWEEP_ONE: %s  CE=%dm ================\n', ds0, ce0);
test_imu_visual_fusion_slam;

% ---- core的clearvars后重新推导 (全局变量仍可用) ----
ds = getenv('SWEEP_DS');
ce = str2double(getenv('SWEEP_CE'));
dataRoot = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
out = fullfile('/tmp/sweep_ce', sprintf('%s_ce%d', ds, ce));
if ~isfolder(out), mkdir(out); end
copyfile(fullfile(dataRoot, ds, 'slam_results', 'exp_trajectory.txt'), fullfile(out, 'exp_trajectory.txt'));
copyfile(fullfile(dataRoot, ds, 'slam_results', 'experiences.mat'), fullfile(out, 'experiences.mat'));
copyfile(fullfile(dataRoot, ds, 'slam_results', 'trajectories.mat'), fullfile(out, 'trajectories.mat'));
disp('RESULT files saved');

try
    global DIAG_VT_REVISIT DIAG_BELOW_THR DIAG_MULTI_CAND DIAG_MULTI_REJECT ...
           DIAG_SINGLE_BELOW DIAG_SINGLE_MATCH DIAG_CE_REJECT DIAG_CE_MAX DIAG_MATCH_LOG
    df = fopen(fullfile(out, 'diag_funnel.txt'), 'w');
    fprintf(df, 'ds=%s ce_set=%d\n', ds, ce);
    fprintf(df, 'VT_REVISIT=%d BELOW_THR=%d MULTI_CAND=%d MULTI_REJECT=%d SINGLE_BELOW=%d SINGLE_MATCH=%d CE_REJECT=%d CE_MAX=%.2f\n', ...
        DIAG_VT_REVISIT, DIAG_BELOW_THR, DIAG_MULTI_CAND, DIAG_MULTI_REJECT, ...
        DIAG_SINGLE_BELOW, DIAG_SINGLE_MATCH, DIAG_CE_REJECT, DIAG_CE_MAX);
    fclose(df);
    if ~isempty(DIAG_MATCH_LOG)
        dlmwrite(fullfile(out, 'diag_match_log.txt'), DIAG_MATCH_LOG, 'precision', 3);
    end
    disp('DIAG saved');
catch me
    disp(['DIAG save failed: ' me.message]);
end
disp(['SAVED: ' out]);
disp('SWEEP_ONE DONE');
