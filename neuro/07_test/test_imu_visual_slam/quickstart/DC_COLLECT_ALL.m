function DC_COLLECT_ALL(ds)
% DC_COLLECT_ALL — 分布式闭环漂移场: 5数据集 pass1 事件采集(2026-10-04)
% 口径: 与各自 RERUN_*.m 基线完全一致(同参数覆盖+同帧数), 仅叠加
%       NLM_DC_ENABLE=true + NLM_DC_COLLECT_ONLY=true:
%       在线记录重锚定跳变事件并落盘 dc_events.mat, 轨迹保持基线逐帧不变。
% 用法: 瘦驱动脚本调用, 如 DC_COLLECT_TOWN01.m:
%       ds = 'Town01'; DC_COLLECT_ALL(ds);
% 数据集: Town01 / Town02 / Town05 / Town10HD / KITTI07
% 输出: <dataset>/dc_pass1/dc_events.mat + 基线口径 exp_trajectory.txt(可交叉验证md5)
% 注意: 本函数不执行 clear all (会清掉传入的局部参数ds; 清理由瘦驱动负责)。
assert(nargin >= 1, 'Usage: DC_COLLECT_ALL(Town01|Town02|Town05|Town10HD|KITTI07)');
ds = char(ds);

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% ===== 各数据集基线参数覆盖 (与 RERUN_*.m 逐行一致, 勿改) =====
switch ds
    case 'Town01'
        DATASET_NAME = 'Town01Data_IMU_Fusion';
        global FAST_TEST_MODE FAST_TEST_FRAMES;
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        % (无参数覆盖, core默认)

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
        % (无参数覆盖, core默认)

    otherwise
        error('未知数据集: %s (Town01/Town02/Town05/Town10HD/KITTI07)', ds);
end

% ===== DC v3: 只采集事件, 不注入漂移场 (轨迹=基线) =====
global NLM_DC_ENABLE NLM_DC_COLLECT_ONLY;
NLM_DC_ENABLE = true;
NLM_DC_COLLECT_ONLY = true;

global RESULT_SUBDIR;
RESULT_SUBDIR = 'dc_pass1';

fprintf('=== DC pass1 事件采集: %s (COLLECT_ONLY, 基线口径) ===\n', ds);
test_imu_visual_fusion_slam;
fprintf('=== DC_COLLECT_ALL(%s) 完成 ===\n', ds);
end
