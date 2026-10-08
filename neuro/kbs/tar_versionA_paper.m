% tar_versionA_paper.m — 用 版本A(首100帧锚定 Sim3, 论文实际口径) 在 Turnaround 上算 EKF/dc/a_disk/VO
% 版本A 位于 neuro/07_test/07_test/test_imu_visual_slam/ablation/compute_metrics_with_alignment.m
% 这是决定 "论文口径下 NLM 能否胜过 EKF" 的判据测试
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');  % 版本A
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
ekf  = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
dc   = csvread(fullfile(tar,'dc_final','exp_trajectory.txt'));
adisk= csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
vo   = csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('GT %d帧 路径%.1fm\n', size(gt,1), L);

% 验证版本A确为首100帧锚定: 打印其内部 align_frames
% (通过对比 align=100 与 align=full 的 ATE 区分)
cfgs = {'EKF',ekf; 'dc基线',dc; 'a_disk(A-fix)',adisk; 'VO',vo};
fprintf('\n=== 版本A 口径 (首100帧锚定 Sim3, 论文Table口径) ===\n');
for k = 1:size(cfgs,1)
    fprintf('  %-14s ATE = %8.2f m\n', cfgs{k,1}, compute_metrics_with_alignment(cfgs{k,2}, gt, L));
end

% 对照: 版本B (全轨迹锚定) 在同一批数据
clear classes; 
try
    b_fn = @(tr) compute_metrics_with_alignment(tr, gt, L);  % 版本A
catch
end
fprintf('\n=== 版本B 口径 (全轨迹Procrustes, 主ablation当前文件) ===\n');
for k = 1:size(cfgs,1)
    tr = cfgs{k,2};
    min_len = min(size(tr,1),size(gt,1)); tr=tr(1:min_len,:); gt2=gt(1:min_len,:);
    [M,trA,T] = procrustes(gt2, tr, 'Scaling', true);
    fullA = T.b * tr * T.T + repmat(T.c(1,:), min_len, 1);
    fprintf('  %-14s ATE = %8.2f m  (scale=%.3f)\n', cfgs{k,1}, sqrt(mean(sum((fullA-gt2).^2,2))), T.b);
end
fprintf('\n判读: 版本A(论文口径)下 若 a_disk < EKF → 闭环真实增益可达; 若 a_disk >= EKF → 论文口径下无法证明 NLM>EKF.\n');
fprintf('VERSIONA_PAPER_DONE\n');
