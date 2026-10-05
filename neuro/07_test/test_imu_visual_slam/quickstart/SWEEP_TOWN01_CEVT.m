% SWEEP_TOWN01_CEVT.m — 现架构(EKF-odo+锚点DR输出+P1平滑)下的 VT 阈值扫描
% 已测: vt08_ce20(=core默认, P1ON) = 75.70; vt08_ce50 = 84.02 (CE放宽有害)
% 本脚本用函数形式: core 的 clearvars -except 只清 base 工作区, 函数局部变量安全
function sweep_town01_cevt()
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

% [VT, CE] 组合; vt08_ce20 已有(75.70) 不再跑
vt_ce = [0.10 20;
         0.12 20;
         0.15 20;
         0.15 50];
sweep_root = '/tmp/nlm_sweep_t01';
if ~isfolder(sweep_root), mkdir(sweep_root); end
base_dir = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results';

for i = 1:size(vt_ce, 1)
    vt = vt_ce(i, 1); ce = vt_ce(i, 2);
    tag = sprintf('vt%02d_ce%d', round(vt*100), ce);
    fprintf('\n########## SWEEP %s (VT=%.2f CE=%d) ##########\n', tag, vt, ce);
    t0 = tic;

    global DATASET_NAME FAST_TEST_MODE FAST_TEST_FRAMES NLM_USE_EKF_ODO;
    global VT_MATCH_THRESHOLD_OVERRIDE;
    global EXP_MAX_LOOP_CE NLM_LOOP_FIX_ENABLE NLM_LOOP_CORR;
    DATASET_NAME = 'Town01Data_IMU_Fusion';
    FAST_TEST_MODE = false;
    FAST_TEST_FRAMES = 5000;
    NLM_USE_EKF_ODO = true;
    VT_MATCH_THRESHOLD_OVERRIDE = vt;
    EXP_MAX_LOOP_CE = ce;
    NLM_LOOP_FIX_ENABLE = false;
    NLM_LOOP_CORR = [0, 0, 0];

    test_imu_visual_fusion_slam;

    out = fullfile(sweep_root, tag);
    mkdir(out);
    copyfile(fullfile(base_dir, 'exp_trajectory.txt'), out);
    copyfile(fullfile(base_dir, 'experiences.mat'), out);
    copyfile(fullfile(base_dir, 'performance_report.txt'), out);
    rep = fileread(fullfile(out, 'performance_report.txt'));
    idx = strfind(rep, 'Experience Map:');
    rmse = 'NA'; final_err = 'NA';
    if ~isempty(idx)
        block = rep(idx:min(end, idx+200));
        a = regexp(block, 'RMSE: ([0-9.]+)', 'tokens');
        b = regexp(block, '终点误差: ([0-9.]+)', 'tokens');
        if ~isempty(a), rmse = a{1}{1}; end
        if ~isempty(b), final_err = b{1}{1}; end
    end
    fprintf('>>> %s 耗时 %.0fs | 7DoF RMSE=%s m  终点=%s m\n', tag, toc(t0), rmse, final_err);
    clear -globals
end
fprintf('\n=== SWEEP_TOWN01_CEVT 全部完成, 结果在 %s ===\n', sweep_root);
end
