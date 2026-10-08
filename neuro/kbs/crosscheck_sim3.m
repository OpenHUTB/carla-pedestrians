% crosscheck_sim3.m — 用仓库MATLAB口径(compute_metrics_with_alignment, 全轨迹Sim3)
% 交叉核验 Python Umeyama 计算的5集 Sim(3) RMSE。运行: matlab -batch crosscheck_sim3
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
base = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
sets = {'Town01Data_IMU_Fusion','Town02Data_IMU_Fusion','Town05Data_IMU_Fusion', ...
        'Town10HDData_IMU_Fusion','KITTI07Data_IMU_Fusion'};
for k = 1:numel(sets)
    ds = sets{k};
    sr = fullfile(base, ds, 'slam_results');
    gt  = csvread(fullfile(sr, 'ground_truth_backup.txt'));
    nlm = csvread(fullfile(sr, 'exp_trajectory.txt'));
    ekf = csvread(fullfile(sr, 'imu_aided_trajectory.txt'));
    vo  = csvread(fullfile(sr, 'pure_visual_trajectory.txt'));
    L = sum(sqrt(sum(diff(gt).^2, 2)));
    [rn, fen, dn] = compute_metrics_with_alignment(nlm, gt, L);
    [re, fee, de] = compute_metrics_with_alignment(ekf, gt, L);
    [rv, fev, dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('%s: NLM=%.2f(终点%.2f,%.2f%%) EKF=%.2f(%.2f%%) VO=%.2f(%.2f%%)\n', ...
        ds, rn, fen, dn, re, de, rv, dv);
end
fprintf('CROSSCHECK_DONE\n');
