% kb_replay_ab_simple.m — Town05 EKF 恢复 A/B, 论文 simple 口径 (起点+长度匹配, 无旋转)
% 用户口径 "EKF 大幅超 VO/IMU" = 该口径 (论文 Table 1: EKF 72.25 vs VO 161.84)
clear all; close all; clc;
ad = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town05Data_IMU_Fusion';
gt = readmatrix(fullfile(ad,'ground_truth.txt'));
vo = readmatrix(fullfile(ad,'visual_odometry.txt'));
n = min([size(gt,1), size(vo,1)]);
L = sum(sqrt(sum(diff(gt(1:n,2:4)).^2,2)));
gtp = gt(1:n,2:4);
fprintf('GT: %d 帧, 长度 %.2f m\n', n, L);

vo_rmse = simple_ate(vo(1:n,2:4), gtp, L);
fprintf('Pure VO (visual_odometry.txt)  simple-ATE = %8.2f m\n', vo_rmse);

files = { '/tmp/t05_ab_a/fusion_pose.txt', 'A 恢复后(#100尺度)';
          '/tmp/t05_ab_b/fusion_pose.txt', 'B 恢复前(HEAD尺度)';
          fullfile(ad,'fusion_pose.txt'),  'C 磁盘基线(10-07 CARLA)' };
for i = 1:size(files,1)
    f = readmatrix(files{i,1});
    m = min(n, size(f,1));
    ekf_rmse = simple_ate(f(1:m,2:4), gtp, L);
    imu_rmse = simple_ate(f(1:m,8:10), gtp, L);
    fprintf('%s (%d 帧)\n  EKF Fusion  simple-ATE = %8.2f m\n  Pure IMU    simple-ATE = %8.2f m\n', ...
        files{i,2}, m, ekf_rmse, imu_rmse);
end
fprintf('KB_REPLAY_AB_SIMPLE_DONE\n');

function rmse = simple_ate(traj, gt, L)
    c = traj - traj(1,:); g = gt - gt(1,:);
    s = L/sum(sqrt(sum(diff(c).^2,2)));
    rmse = sqrt(mean(sum((c*s-g).^2,2)));
end
