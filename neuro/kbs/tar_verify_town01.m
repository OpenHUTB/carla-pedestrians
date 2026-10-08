% tar_verify_town01.m — 用论文口径(版本A 首100帧Sim3) 在原Town01上复算 EKF/a_disk/dc/VO
% 目的: 确认论文" NLM>EKF "是否真实成立, 及 a_disk(A-fix) 相对 EKF 的差距量级
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
t01 = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01_IMU_Fusion';
gt   = csvread(fullfile(t01,'a_final','ground_truth_backup.txt'));
ekf  = csvread(fullfile(t01,'a_final','imu_aided_trajectory.txt'));
adisk= csvread(fullfile(t01,'a_final','exp_trajectory.txt'));
dc   = csvread(fullfile(t01,'dc_final','exp_trajectory.txt'));
vo   = csvread(fullfile(t01,'a_final','pure_visual_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('原Town01  GT %d帧 路径%.1fm\n', size(gt,1), L);
cfgs = {'EKF',ekf; 'a_disk(A-fix)',adisk; 'dc基线',dc; 'VO',vo};
fprintf('\n=== 版本A 口径(首100帧锚定Sim3, 论文Table口径) ===\n');
for k = 1:size(cfgs,1)
    fprintf('  %-14s ATE = %8.2f m   drift%%=%.3f\n', cfgs{k,1}, compute_metrics_with_alignment(cfgs{k,2}, gt, L));
end
% 路径长 & 尺度漂移
fprintf('\n=== 路径长 / GT ===\n');
for k = 1:size(cfgs,1)
    L2 = sum(sqrt(sum(diff(cfgs{k,2}).^2,2)));
    fprintf('  %-14s %8.1f m  /GT=%.3f\n', cfgs{k,1}, L2, L2/L);
end
% 首100帧尺度 vs 全程尺度(尺度漂移量化)
fprintf('\n=== 首100帧尺度 vs 全程尺度 ===\n');
for k = 1:size(cfgs,1)
    tr = cfgs{k,2}; n=min(size(tr,1),size(gt,1)); tr=tr(1:n,:); g=gt(1:n,:);
    [~,~,Tf] = procrustes(g, tr, 'Scaling', true);
    [~,~,T1] = procrustes(g(1:100,:), tr(1:100,:), 'Scaling', true);
    fprintf('  %-14s 首100=%.4f  全程=%.4f  漂移=%.1f%%\n', cfgs{k,1}, T1.b, Tf.b, 100*(Tf.b-T1.b)/T1.b);
end
fprintf('VERIFY_TOWN01_DONE\n');
