% SWEEP_CE.m — CE门(EXP_MAX_LOOP_CE)参数扫描
% 目的: 验证放宽闭环约束误差门能否让 NLM 抓住真闭环(离线上限估计已证明空间)
% 运行: ~/matlab_batch.sh SWEEP_CE
% 输出: 每次运行结果即时拷贝到 /tmp/sweep_ce/{ds}_ce{ce}/exp_trajectory.txt
clear all; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME FAST_TEST_MODE FAST_TEST_FRAMES NLM_USE_EKF_ODO EXP_MAX_LOOP_CE

rootDir = fileparts(fileparts(fileparts(script_dir)));  % neuro/
dataRoot = fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets');

combos = {
    'Town10HDData_IMU_Fusion', 200;
    'Town10HDData_IMU_Fusion', 100;
    'Town10HDData_IMU_Fusion',  50;
    'Town01Data_IMU_Fusion',    200;
    'Town01Data_IMU_Fusion',    100;
    'Town01Data_IMU_Fusion',     50;
    'KITTI07Data_IMU_Fusion',   200;
    'KITTI07Data_IMU_Fusion',   100;
};

for i = 1:2:size(combos, 1)
    ds = combos{i,1}; ce = combos{i,2};
    fprintf('\n================ SWEEP: %s  CE=%dm ================\n', ds, ce);
    DATASET_NAME = ds;
    FAST_TEST_MODE = false;
    FAST_TEST_FRAMES = 5000;
    NLM_USE_EKF_ODO = true;
    EXP_MAX_LOOP_CE = ce;
    try
        test_imu_visual_fusion_slam;
        out = fullfile('/tmp/sweep_ce', sprintf('%s_ce%d', ds, ce));
        if ~isfolder(out), mkdir(out); end
        src = fullfile(dataRoot, ds, 'slam_results', 'exp_trajectory.txt');
        copyfile(src, fullfile(out, 'exp_trajectory.txt'));
        fprintf('>> 已保存: %s/exp_trajectory.txt\n', out);
    catch ME
        fprintf('!! 运行失败: %s (CE=%d)\n', ME.message, ce);
    end
end
fprintf('\n=== SWEEP_CE 全部完成 ===\n');
