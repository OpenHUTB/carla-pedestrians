% tar_drift2.m — (1) 原Town01 论文口径复算 (2) Turnaround 逐lap尺度/旋转漂移剖面
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');

%% (1) 原Town01: 确认论文 NLM>EKF 前提
t01 = '/home/yangrb/openhutb/neuro/data/Town01Data_IMU_Fusion/slam_results';
gt   = csvread(fullfile(t01,'ground_truth_backup.txt'));
ekf  = csvread(fullfile(t01,'imu_aided_trajectory.txt'));
adisk= csvread(fullfile(t01,'exp_trajectory.txt'));
vo   = csvread(fullfile(t01,'pure_visual_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('=== (1) 原Town01  GT %d帧 路径%.1fm — 版本A(首100帧Sim3, 论文Table口径) ===\n', size(gt,1), L);
cfgs = {'EKF',ekf; 'a_disk(A-fix)',adisk; 'VO',vo};
for k = 1:size(cfgs,1)
    [rmse,fe,dr] = compute_metrics_with_alignment(cfgs{k,2}, gt, L);
    fprintf('  %-14s ATE=%8.2f m  final=%8.2f m  drift%%=%.3f\n', cfgs{k,1}, rmse, fe, dr);
end

%% (2) Turnaround 逐lap 尺度漂移剖面
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
c2 = {'EKF', csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
      'a_disk(A-fix)', csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
      'VO', csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'))};
bounds = [59;2386;4714;7037;9365];
fprintf('\n=== (2) Turnaround 逐lap Procrustes 尺度(每lap独立拟合) ===\n');
for k = 1:size(c2,1)
    nm = c2{k,1}; tr = c2{k,2};
    line = sprintf('  %-14s', nm);
    for j = 1:numel(bounds)-1
        a = bounds(j)+1; b = bounds(j+1);
        [~,~,T] = procrustes(gt(a:b,:), tr(a:b,:), 'Scaling', true);
        line = [line, sprintf('  lap%d=%.3f', j-1, T.b)];
    end
    n = min(size(tr,1),size(gt,1));
    [~,~,Tf] = procrustes(gt(1:n,:), tr(1:n,:), 'Scaling', true);
    [~,~,T1] = procrustes(gt(1:100,:), tr(1:100,:), 'Scaling', true);
    rotfull = acosd((trace(Tf.T)-1)/2); if rotfull>180, rotfull=360-rotfull; end
    fprintf('%s  全程=%.3f  首100=%.3f  全程旋转=%5.1fdeg\n', line, Tf.b, T1.b, rotfull);
end
fprintf('\n判读: 逐lap尺度差异大(如 0.4→1.2) = 时变全局尺度漂移, 单一锚点无法吸收,\n');
fprintf('      闭环若能在建图期逐段补偿尺度/旋转, 版本A口径下可获得真实(不可后处理)增益.\n');
fprintf('DRIFT2_DONE\n');
