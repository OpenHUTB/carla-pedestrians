% NLM_ABLATION_BIO.m — 脑启发模块单关消融驱动 (网格细胞/头方向细胞)
% 用法: NLM_ABL_SCEN=Town05 NLM_ABL_MODE=full|nogc|nohdc ~/matlab_batch.sh NLM_ABLATION_BIO
% 口径: 与论文 NLM-fullsystem(P2S 定解)逐行一致——微调嵌入 PR(τ=0.94)+A-fix
%       约束+S2 关+无 DC; 仅关掉目标模块; 结果写 <数据集>/abl_<mode>/
% 注: 腹侧流关闭(vtId=0)会使 HDC/GC/经验图整体失效, 其等价行即 Table 4 的
%     NLM-loopline(EKF 核+闭环线), 故本驱动不再单跑 novt。
scen = getenv('NLM_ABL_SCEN');
mode = getenv('NLM_ABL_MODE');
assert(~isempty(scen) && ~isempty(mode), ...
    'Usage: NLM_ABL_SCEN=<Town05|Town03|Town07|Town10HD|KITTI07> NLM_ABL_MODE=<full|nogc|nohdc>');

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME NLM_USE_EKF_ODO FAST_TEST_MODE FAST_TEST_FRAMES;
global NLM_ABLATE_GC NLM_ABLATE_HDC NLM_ABLATE_VT;
NLM_ABLATE_GC  = strcmp(mode, 'nogc');
NLM_ABLATE_HDC = strcmp(mode, 'nohdc');
NLM_ABLATE_VT  = false;   % novt 由 Table 4 loopline 行等价替代

global RESULT_SUBDIR;
RESULT_SUBDIR = ['abl_' mode];   % 独立目录, 不覆盖论文结果

NLM_USE_EKF_ODO = true;

% ── 各场景参数覆盖 (与 P2S_*/RERUN_TOWN03 逐行一致, 勿改) ──
switch scen
    case 'Town05'
        DATASET_NAME = 'Town05Data_IMU_Fusion';
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.040;
        global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        global EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
        EXP_LOOPS_OVERRIDE = 12; EXP_CORRECTION_OVERRIDE = 0.5;
        global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30; IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        global GC_VT_INJECT_ENERGY_OVERRIDE; GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town03'
        DATASET_NAME = 'Town03Data_IMU_Fusion';
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 5000;
        global VT_MATCH_THRESHOLD_OVERRIDE; VT_MATCH_THRESHOLD_OVERRIDE = 0.12;
        global EXP_MAX_LOOP_CE; EXP_MAX_LOOP_CE = 10;
        global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE; DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18;
        global EXP_LOOPS_OVERRIDE EXP_CORRECTION_OVERRIDE;
        EXP_LOOPS_OVERRIDE = 12; EXP_CORRECTION_OVERRIDE = 0.5;
        global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = 0.70; IMU_TRANS_WEIGHT_OVERRIDE = 0.30; IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;
        global GC_VT_INJECT_ENERGY_OVERRIDE; GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;
    case 'Town07'
        DATASET_NAME = 'Town07Data_IMU_Fusion';
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
        FAST_TEST_MODE = false; FAST_TEST_FRAMES = 1101;
    otherwise
        error('未知场景: %s (Town05/Town03/Town07/Town10HD/KITTI07)', scen);
end

% ── P2S 定解 PR 块 (微调嵌入 + A-fix 约束, S2 关; 与 P2S_*/P2FT94_* 一致) ──
global NLM_USE_PR PR_SIM_THRESHOLD PR_LOOP_RADIUS PR_ARC_GAP PR_MIN_GAP PR_EMB_FILE;
NLM_USE_PR = true;
PR_EMB_FILE = 'pr_embeddings_ft.mat';
PR_SIM_THRESHOLD = 0.94;
PR_LOOP_RADIUS = 1e9;    % 距离门禁用 (漂移/压缩地图上不可靠)
PR_ARC_GAP = 50;
PR_MIN_GAP = 30;
global NLM_LOOP_CONSTRAINT NLM_SIM3_REANCHOR NLM_SIM3_WIN;
NLM_LOOP_CONSTRAINT = true;
NLM_SIM3_REANCHOR = false;
NLM_SIM3_WIN = 250;

% ── DC 注入: P2S 口径不带 DC, 显式关掉防会话残留 ──
global NLM_DC_ENABLE NLM_DC_COLLECT_ONLY;
NLM_DC_ENABLE = false; NLM_DC_COLLECT_ONLY = false;

% 冒烟模式: NLM_ABL_SMOKE=1 → 150帧快速验证开关机制
if strcmp(getenv('NLM_ABL_SMOKE'), '1')
    FAST_TEST_MODE = true;
    FAST_TEST_FRAMES = 150;
    fprintf('=== [SMOKE] 150帧快速验证 ===\n');
end

% 注意: 核心脚本末尾 clearvars -except 会清掉本驱动的 scen/mode,
% 完成标记用 getenv(字面量), 不依赖工作区变量
test_imu_visual_fusion_slam;
fprintf('=== NLM_ABLATION_BIO(%s, %s) 完成, 结果已落盘 abl_%s ===\n', ...
    getenv('NLM_ABL_SCEN'), getenv('NLM_ABL_MODE'), getenv('NLM_ABL_MODE'));
