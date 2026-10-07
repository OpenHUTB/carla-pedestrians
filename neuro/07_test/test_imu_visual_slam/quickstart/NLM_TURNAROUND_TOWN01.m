% NLM_TURNAROUND_TOWN01.m — Town01 双向折返(真闭环)数据 A/B 驱动
% 目的: 在真正回到起点的折返路线上验证闭环约束创新(A-fix)的正向增益。
%   开放路线(Town01/05/KITTI07)已验证: 约束0事件=中性, KITTI07唯一1事件(+1.36m)。
%   本路线为双向折返(去程100m+掉头原路返回), 重访间隔~60s(≫MIN_GAP=300帧),
%   CE残差≈车道宽4m+漂移, 应落在CE门[3,20]内 → 闭环约束应激活并体现增益。
%
% 用法: ~/matlab_batch.sh NLM_TURNAROUND_TOWN01 dc        → dc_final/  (DC注入, 无约束)
%       ~/matlab_batch.sh NLM_TURNAROUND_TOWN01 constraint → a_final/  (DC注入+闭环约束)
%
% 口径: 与 DC_FINAL 的 Town01 完全一致(同参数覆盖+同DC参数), 仅数据集不同:
%   数据集 = Town01Turnaround_IMU_Fusion (采集器 --turnaround --bidir-start 输出)
mode = 'dc';
if nargin >= 1
    mode = char(mode);
end
a_mode = strcmp(mode, 'constraint');

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;

% ===== 基线参数 (与 DC_FINAL 的 Town01 case 逐行一致, 勿改) =====
DATASET_NAME = 'Town01Turnaround_IMU_Fusion';
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;

% ===== DC v3 漂移场注入 (与 DC_FINAL Town01 定解一致: 离线simple口径重扫) =====
global NLM_DC_ENABLE NLM_DC_COLLECT_ONLY;
global NLM_DC_W0 NLM_DC_W_PER_M NLM_DC_W_MAX NLM_DC_TAIL_COMPLETE NLM_DC_CMAX;
NLM_DC_ENABLE = true;
NLM_DC_COLLECT_ONLY = false;
NLM_DC_W_MAX = 4000;
NLM_DC_TAIL_COMPLETE = true;
NLM_DC_W0 = 100;  NLM_DC_W_PER_M = 0;   NLM_DC_CMAX = 2.0;

if a_mode
    global NLM_LOOP_CONSTRAINT NLM_LOOP_FIX_ENABLE;
    global NLM_LOOP_CORR;
    NLM_LOOP_CONSTRAINT = true;
    NLM_LOOP_FIX_ENABLE = true;
    NLM_LOOP_CORR = [0, 0, 0];
    global RESULT_SUBDIR;
    RESULT_SUBDIR = 'a_final';
    fprintf('=== Turnaround A/B: constraint (DC+闭环约束) → a_final/ ===\n');
else
    global RESULT_SUBDIR;
    RESULT_SUBDIR = 'dc_final';
    fprintf('=== Turnaround A/B: dc (纯DC注入基线) → dc_final/ ===\n');
end

% 核心是script, 在其调用方工作区执行; 末尾 clearvars -except 会清掉本函数
% 局部变量, 故完成标记用字面量(结果在核心脚本内已落盘 RESULT_SUBDIR)
test_imu_visual_fusion_slam;
fprintf('=== NLM_TURNAROUND_TOWN01 结束, 结果已落盘 RESULT_SUBDIR ===\n');
