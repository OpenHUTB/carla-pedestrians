% tar_paper_caliber.m — 一锤定音: 用主 ablation 的 compute_metrics_with_alignment(版本B, 全轨迹锚定)
% 复现论文 Table 值(Town01) + 确认 Turnaround 坍缩, 并对比 前100帧(版本A) 口径
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');  % 版本B 所在

datasets = {...
    'Town01',        '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion', ...
        'ground_truth.txt', 'slam_results/exp_trajectory.txt', 'fusion_pose.txt', 'visual_odometry.txt';
    'Town01Turnaround','/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion', ...
        'a_final/ground_truth_backup.txt', 'dc_final/exp_trajectory.txt', 'a_final/imu_aided_trajectory.txt', 'a_final/pure_visual_trajectory.txt'};

for d = 1:size(datasets,1)
    R = datasets{d,2}; gt = csvread(fullfile(R, datasets{d,3}));
    nlm = csvread(fullfile(R, datasets{d,4}));
    ekf = csvread(fullfile(R, datasets{d,5}));
    vo  = csvread(fullfile(R, datasets{d,6}));
    L = sum(sqrt(sum(diff(gt).^2,2)));
    fprintf('=== %s (GT %d帧 %.1fm) ===\n', datasets{d,1}, size(gt,1), L);
    % 版本B: compute_metrics_with_alignment (主 ablation, 全轨迹锚定)
    fprintf('  [版本B 全轨迹锚定]  NLM=%8.2f  EKF=%8.2f  VO=%8.2f\n', ...
        compute_metrics_with_alignment(nlm,gt,L), ...
        compute_metrics_with_alignment(ekf,gt,L), ...
        compute_metrics_with_alignment(vo,gt,L));
    % 版本A: 前100帧锚定 (内联, 用内置 procrustes)
    af = 100;
    for k = 1:3
        tr = {nlm,ekf,vo}{k}; nm = {'NLM','EKF','VO'}{k};
        [M, trA, T] = procrustes(gt(1:af,:), tr(1:af,:), 'Scaling', true);
        n = min(size(gt,1),size(tr,1));
        fullA = T.b * tr(1:n,:) * T.T + repmat(T.c(1,:), n, 1);
        fprintf('  [版本A 前100帧锚定]  %s=%8.2f  (scale=%.3f)\n', nm, ...
            sqrt(mean(sum((fullA-gt(1:n,:)).^2,2))), T.b);
    end
    fprintf('\n');
end
fprintf('PAPER_CALIBER_DONE\n');
