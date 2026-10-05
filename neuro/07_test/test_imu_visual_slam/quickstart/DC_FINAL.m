function DC_FINAL(ds)
% DC_FINAL — 分布式闭环漂移场(DC v3): 最终注入跑(2026-10-04)
% 与各自 RERUN_*.m 基线同口径(同参数覆盖+同帧数), 叠加 DC v3 漂移场注入:
%   NLM_DC_ENABLE=true, COLLECT_ONLY=false, 逐集 W0/W_PER_M/CMAX 由离线
%   dc_sweep.py 网格选定(7-DoF Procrustes 口径), 结果写入 dc_final/。
% 用法: 瘦驱动调用, 如 DC_FINAL_TOWN01.m:  ds = 'Town01'; DC_FINAL(ds);
% 数据集: Town01 / Town02 / Town05 / Town10HD / KITTI07
% 本函数不执行 clear all (会清掉传入的局部参数ds; 清理由瘦驱动负责)。
assert(nargin >= 1, 'Usage: DC_FINAL(Town01|Town02|Town05|Town10HD|KITTI07)');
ds = char(ds);

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% ===== 基线参数覆盖 (与 RERUN_*.m 逐行一致, 勿改) =====
switch ds
    case 'Town01'
        DATASET_NAME = 'Town01Data_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
    case 'Town02'
        DATASET_NAME = 'Town02Data_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.055;
        global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 15;
        global EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
        EXP_LOOPS_OVERRIDE = 10; EXP_CORRECTION_OVERRIDE = 0.5;
        global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = 0.65; IMU_TRANS_WEIGHT_OVERRIDE = 0.25;
    case 'Town05'
        DATASET_NAME = 'Town05Data_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
        global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        global EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
        EXP_LOOPS_OVERRIDE = 12; EXP_CORRECTION_OVERRIDE = 0.5;
        global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30; IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        global GC_VT_INJECT_ENERGY_OVERRIDE; GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town10HD'
        DATASET_NAME = 'Town10HDData_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
        global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        global EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
        EXP_LOOPS_OVERRIDE = 12; EXP_CORRECTION_OVERRIDE = 0.5;
        global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30; IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        global GC_VT_INJECT_ENERGY_OVERRIDE; GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'KITTI07'
        DATASET_NAME = 'KITTI07Data_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 1101;
    otherwise
        error('未知数据集: %s (Town01/Town02/Town05/Town10HD/KITTI07)', ds);
end

% ===== DC v3 漂移场注入参数 (dc_sweep.py 网格定解, 7-DoF口径) =====
global NLM_DC_ENABLE NLM_DC_COLLECT_ONLY;
global NLM_DC_W0 NLM_DC_W_PER_M NLM_DC_W_MAX NLM_DC_TAIL_COMPLETE NLM_DC_CMAX;
NLM_DC_ENABLE = true;
NLM_DC_COLLECT_ONLY = false;
NLM_DC_W_MAX = 4000;
NLM_DC_TAIL_COMPLETE = true;
switch ds
    case 'Town01'
        NLM_DC_W0 = 100;  NLM_DC_W_PER_M = 0;   NLM_DC_CMAX = 2.0;
    case 'Town02'
        NLM_DC_W0 = 800;  NLM_DC_W_PER_M = 20;  NLM_DC_CMAX = inf;
    case 'Town05'
        NLM_DC_W0 = 2400; NLM_DC_W_PER_M = 80;  NLM_DC_CMAX = 3.0;
    case 'Town10HD'
        NLM_DC_W0 = 400;  NLM_DC_W_PER_M = 80;  NLM_DC_CMAX = 6.0;
    case 'KITTI07'
        NLM_DC_W0 = 400;  NLM_DC_W_PER_M = 80;  NLM_DC_CMAX = 6.0;
end

global RESULT_SUBDIR;
RESULT_SUBDIR = 'dc_final';

fprintf('=== DC最终注入: %s  (W0=%d WP=%d CMAX=%s) ===\n', ds, NLM_DC_W0, NLM_DC_W_PER_M, num2str(NLM_DC_CMAX));
test_imu_visual_fusion_slam;
fprintf('=== DC_FINAL(%s) 完成 ===\n', ds);
end
