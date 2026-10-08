% kb_replay_ab.m — Town05 EKF 恢复 A/B + 论文 Sim(3) 口径 (首100帧锚定, Table 1 同口径)
% A = 恢复后(#100尺度形态) replay 输出; B = 恢复前(HEAD尺度形态) replay 输出; C = 磁盘10-07基线
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
ad = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town05Data_IMU_Fusion';
gt = readmatrix(fullfile(ad,'ground_truth.txt'));
vo = readmatrix(fullfile(ad,'visual_odometry.txt'));
n = min([size(gt,1), size(vo,1)]);
L = sum(sqrt(sum(diff(gt(1:n,2:5)).^2,2)));
gtp = gt(1:n,2:5); vop = vo(1:n,2:5);

vo_rmse = compute_metrics_with_alignment(vop, gtp, L);
fprintf('Pure VO (visual_odometry.txt)  Sim3-ATE = %8.2f m\n', vo_rmse);

files = { '/tmp/t05_ab_a/fusion_pose.txt', 'A 恢复后(#100尺度)';
          '/tmp/t05_ab_b/fusion_pose.txt', 'B 恢复前(HEAD尺度)';
          fullfile(ad,'fusion_pose.txt'),  'C 磁盘基线(10-07)' };
for i = 1:2:size(files,1)
    f = readmatrix(files{i,1});
    ekf_rmse = compute_metrics_with_alignment(f(1:n,2:4), gtp, L);
    imu_rmse = compute_metrics_with_alignment(f(1:n,8:10), gtp, L);
    fprintf('%s\n  EKF Fusion  Sim3-ATE = %8.2f m\n  Pure IMU    Sim3-ATE = %8.2f m\n', ...
        files{i,2}, ekf_rmse, imu_rmse);
end
fprintf('KB_REPLAY_AB_DONE\n');
