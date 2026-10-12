% SWEEP_ONE.m — 跑单个扫参组合 (读环境变量 SET/VT/CE/HDC), 2026-10-12
% 用法: SET=Town03 VT=0.02 CE=20 HDC=18 matlab -batch SWEEP_ONE
% 口径与 RERUN_TOWN03/07/10HD.m 完全一致 (C类重跑 + EKF-odo 钉扎),
% 仅 VT/CE/HDC 来自环境变量; 结果暂存 /tmp/nlm_sweep_weak/<SET>/<tag>/
%
% 设计: 脚本形式 + 每组合独立 MATLAB 进程 (与 RERUN 驱动同构, 已验证可靠)。
% 原因: core 的 [10/10] 综合报告调用 compare_datasets.m (开头 clear all),
% 函数形式驱动中 core 调用后使用局部变量会踩到已清除的变量
% (SWEEP_WEAK.m 首 run 即因此崩溃, 已弃用)。
% 组合参数走文件 (combo.txt): 本机 MATLAB 无 readenv (R2014a 前版本),
% 环境变量传参不可用; 文件传参任意版本可用。
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

combo_file = '/tmp/nlm_sweep_weak/combo.txt';
fid = fopen(combo_file, 'r');
if fid < 0
    error('找不到 %s (先由 kbs/sweep_weak_sets.py 写入)', combo_file);
end
parts = strsplit(strtrim(fgetl(fid)));
fclose(fid);
if numel(parts) ~= 4
    error('combo.txt 格式应为 "Set VT CE HDC", 实际: %s', fgetl(fid));
end
setname = parts{1};
vt  = str2double(parts{2});
ce  = str2double(parts{3});
hdc = str2double(parts{4});
tag = sprintf('%s_vt%s_ce%d_hdc%d', setname, num2str(vt), ce, hdc);

global DATASET_NAME FAST_TEST_MODE FAST_TEST_FRAMES NLM_USE_EKF_ODO;
global VT_MATCH_THRESHOLD_OVERRIDE EXP_MAX_LOOP_CE;
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
global GC_VT_INJECT_ENERGY_OVERRIDE;
global NLM_LOOP_CONSTRAINT NLM_LOOP_FIX_ENABLE NLM_LOOP_CORR;
DATASET_NAME = [setname 'Data_IMU_Fusion'];
FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
NLM_USE_EKF_ODO = true;
VT_MATCH_THRESHOLD_OVERRIDE = vt;
EXP_MAX_LOOP_CE = ce;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = hdc;
EXP_LOOPS_OVERRIDE = 12; EXP_CORRECTION_OVERRIDE = 0.5;
IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
NLM_LOOP_CONSTRAINT = false; NLM_LOOP_FIX_ENABLE = false; NLM_LOOP_CORR = [0, 0, 0];

fprintf('>>> SWEEP_ONE %s 开始\n', tag);
test_imu_visual_fusion_slam;
% 注意: core 的 [10/10] 综合报告调用 compare_datasets (开头 clear all),
% 会清掉本脚本(base 工作区)的所有变量 — 此处之后不得再引用任何变量。
% 产物暂存由 kbs/sweep_weak_sets.py 在本进程退出后执行 (从 slam_results 拷走)。
fprintf('>>> SWEEP_ONE core 完成, 退出 (暂存交给编排器)\n');
