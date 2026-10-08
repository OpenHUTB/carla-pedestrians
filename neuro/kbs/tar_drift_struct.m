% tar_drift_struct.m — 拆解 Turnaround 各轨迹的"系统性"误差成分: 尺度/旋转/平移
% 目的: 搞清 version A(745m) 与 version B(28.7m) 的 26x 差距到底是什么
clear all; close all; clc;
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
ekf  = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
dc   = csvread(fullfile(tar,'dc_final','exp_trajectory.txt'));
adisk= csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
vo   = csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'));

cfgs = {'EKF',ekf; 'dc',dc; 'a_disk',adisk; 'VO',vo};
Lgt = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('GT 路径长 = %.1f m (%d帧)\n', Lgt, size(gt,1));
fprintf('\n=== 各轨迹路径长 & 与GT的比例(≈全局尺度因子) ===\n');
for k = 1:size(cfgs,1)
    L = sum(sqrt(sum(diff(cfgs{k,2}).^2,2)));
    fprintf('  %-8s 路径长 %8.1f m   /GT = %.3f\n', cfgs{k,1}, L, L/Lgt);
end

fprintf('\n=== version B 全局Procrustes 拟合的 (尺度, 旋转角, 平移) ===\n');
for k = 1:size(cfgs,1)
    tr = cfgs{k,2}; n=min(size(tr,1),size(gt,1)); tr=tr(1:n,:); g=gt(1:n,:);
    [M,A,T] = procrustes(g, tr, 'Scaling', true);
    % T: 结构体, T.b=scale, T.R=旋转矩阵, T.c=平移向量(在原始坐标系)
    rotang = acosd((trace(T.R)-1)/2); if rotang>180, rotang=360-rotang; end
    fprintf('  %-8s scale=%.4f  旋转=%6.2f deg  |平移|=%7.2f m\n', cfgs{k,1}, T.b, rotang, norm(T.c));
end

% 首100帧的尺度 vs 全程尺度 → 看是否存在"尺度漂移"
fprintf('\n=== 尺度漂移: 首100帧拟合尺度 vs 全程拟合尺度 ===\n');
for k = 1:size(cfgs,1)
    tr = cfgs{k,2}; n=min(size(tr,1),size(gt,1)); tr=tr(1:n,:); g=gt(1:n,:);
    [~,~,Tfull] = procrustes(g, tr, 'Scaling', true);
    [~,~,T100]  = procrustes(g(1:100,:), tr(1:100,:), 'Scaling', true);
    fprintf('  %-8s 首100帧尺度=%.4f   全程尺度=%.4f   漂移=%.3f%%\n', ...
        cfgs{k,1}, T100.b, Tfull.b, 100*(Tfull.b-T100.b)/T100.b);
end
fprintf('DRIFT_STRUCT_DONE\n');
