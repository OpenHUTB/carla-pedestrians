% tar_caliber_check.m — 独立验证 compute_metrics_with_alignment 在 Turnaround 的真实行为
% 暴露中间量: procrustes 结构体字段 / 首100帧 scale / 旋转迹 / 首100帧拟合残差
% 目的: 判定 28.7 坍缩是"函数性质"还是"transform.T/transform.c 字段缺失导致的静默错误"
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';

gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
ekf  = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
base = csvread(fullfile(tar,'dc_final','exp_trajectory.txt'));
adisk= csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
vo   = csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('GT %d帧 路径%.1fm\n', size(gt,1), L);

af = 100;
trArr = {ekf, base, adisk, vo};
nmArr = {'EKF','dc','a_disk','VO'};
for i = 1:4
    tr = trArr{i};
    nm = nmArr{i};
    [M, trA, T] = procrustes(gt(1:af,:), tr(1:af,:), 'Scaling', true);
    fprintf('\n[%s] procrustes 结构体字段: %s\n', nm, strjoin(fieldnames(T),','));
    s = T.b;
    fprintf('  scale b = %s\n', mat2str(s,4));
    if isfield(T,'R'), fprintf('  rot trace(R) = %.4f\n', trace(T.R)); end
    if isfield(T,'T'), fprintf('  (异常) 存在 T 字段 = %s\n', mat2str(T.T,3)); end
    if isfield(T,'c'), fprintf('  (异常) 存在 c 字段 = %s\n', mat2str(T.c,3)); end
    % 首100帧拟合残差(procrustes 返回的 trA 是 X 对齐到 Y, 这里反向验证)
    fitRes = sqrt(mean(sum((trA(1:af,:) - gt(1:af,:)).^2,2)));
    fprintf('  首100帧 拟合残差(RMSE) = %8.2f m\n', fitRes);
    % 全段 ATE, 用函数同款公式
    n = min(size(gt,1), size(tr,1));
    if isfield(T,'T') && isfield(T,'c')
        fullA = T.b * tr(1:n,:) * T.T + repmat(T.c(1,:), n, 1);
        fprintf('  全段 ATE (函数公式 transform.T/c) = %8.2f m\n', sqrt(mean(sum((fullA-gt(1:n,:)).^2,2))));
    end
    if isfield(T,'R') && isfield(T,'t')
        fullA2 = T.b * tr(1:n,:) * T.R + repmat(T.t(1,:), n, 1);
        fprintf('  全段 ATE (标准字段 transform.R/t) = %8.2f m\n', sqrt(mean(sum((fullA2-gt(1:n,:)).^2,2))));
    end
end
fprintf('\nCALIBER_CHECK_DONE\n');
