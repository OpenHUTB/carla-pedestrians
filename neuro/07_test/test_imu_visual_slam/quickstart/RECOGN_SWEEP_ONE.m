% RECOGN_SWEEP_ONE.m — 检测器升级扫描单组: VT召回阈值 × CE门 × B-fix传播开关
% 环境变量: SWEEP_DS=数据集名  VT_THR=VT阈值  CE_VAL=CE门  FIX_ON=1/0(持续性修正)
% 口径 = 对应 RERUN 基线参数覆盖 + VT/CE/FIX 三个扫描变量
% 输出: /tmp/reco_sweep/{ds}_vt{VT_THR}_ce{CE_VAL}_fix{FIX_ON}/ {exp_trajectory.txt, diag_funnel.txt, diag_match_log.txt}
clear all; close all; clc;

ds0 = getenv('SWEEP_DS');
if isempty(ds0), ds0 = 'Town10HDData_IMU_Fusion'; end
vt_thr = str2double(getenv('VT_THR'));
if isempty(vt_thr) || isnan(vt_thr), vt_thr = 0.040; end
ce_val = str2double(getenv('CE_VAL'));
if isempty(ce_val) || isnan(ce_val), ce_val = 50; end
fix_on = strcmp(getenv('FIX_ON'), '1');

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
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        EXP_LOOPS_OVERRIDE = 12;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
        IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town05Data_IMU_Fusion'
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        EXP_LOOPS_OVERRIDE = 12;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
        IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town02Data_IMU_Fusion'
        DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 15;
        EXP_LOOPS_OVERRIDE = 10;
        EXP_CORRECTION_OVERRIDE = 0.5;
        IMU_YAW_WEIGHT_OVERRIDE = 0.65;
        IMU_TRANS_WEIGHT_OVERRIDE = 0.25;
    otherwise
        % Town01 / KITTI07: 无参数覆盖(core默认), 与RERUN口径一致
end

% ==== 扫描变量: VT召回阈值(放宽→真闭环VT重识别) ====
VT_MATCH_THRESHOLD_OVERRIDE = vt_thr;

% ==== CE门(收紧→挡假匹配) + B-fix持续性修正传播 ====
global EXP_MAX_LOOP_CE;
EXP_MAX_LOOP_CE = ce_val;
global NLM_LOOP_FIX_ENABLE;
NLM_LOOP_FIX_ENABLE = fix_on;
global NLM_LOOP_CORR;
NLM_LOOP_CORR = [0, 0, 0];

fprintf('\n================ RECOGN_SWEEP: %s VT=%.3f CE=%.0f FIX=%d ================\n', ...
    ds0, vt_thr, ce_val, fix_on);
test_imu_visual_fusion_slam;

% ---- core的clear all后重新推导 (core是脚本, clear all会连带清掉本脚本局部变量; 全局变量仍可用) ----
ds0 = getenv('SWEEP_DS');
vt_thr = str2double(getenv('VT_THR'));
if isempty(vt_thr) || isnan(vt_thr), vt_thr = 0.040; end
ce_val = str2double(getenv('CE_VAL'));
if isempty(ce_val) || isnan(ce_val), ce_val = 50; end
fix_on = strcmp(getenv('FIX_ON'), '1');
tag = sprintf('%s_vt%s_ce%g_fix%d', ds0, num2str(vt_thr), ce_val, fix_on);
dataRoot = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
out = fullfile('/tmp/reco_sweep', tag);
if ~isfolder(out), mkdir(out); end
copyfile(fullfile(dataRoot, ds0, 'slam_results', 'exp_trajectory.txt'), fullfile(out, 'exp_trajectory.txt'));
copyfile(fullfile(dataRoot, ds0, 'slam_results', 'experiences.mat'), fullfile(out, 'experiences.mat'));

try
    global DIAG_VT_REVISIT DIAG_MULTI_CAND DIAG_MULTI_REJECT ...
           DIAG_SINGLE_BELOW DIAG_SINGLE_MATCH DIAG_CE_REJECT DIAG_CE_MAX ...
           DIAG_LOOP_ACCEPT DIAG_YAW_REJECT DIAG_MATCH_LOG NLM_LOOP_CORR;
    if isempty(NLM_LOOP_CORR), NLM_LOOP_CORR = [0, 0, 0]; end
    df = fopen(fullfile(out, 'diag_funnel.txt'), 'w');
    fprintf(df, 'ds=%s VT=%.3f CE=%.0f FIX=%d\n', ds0, vt_thr, ce_val, fix_on);
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
disp('RECOGN SWEEP ONE DONE');
