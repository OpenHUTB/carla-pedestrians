% tar_simple_eval.m — 新A-fix(全局Sim3重锚) 在【两种论文口径】下完整验证
%   口径1: 'simple' (平移+全局长度匹配, 无旋转) — 与 core test_imu_visual_fusion_slam.m 一致
%   口径2: 版本A 首100帧Sim(3)锚定 — ablation Table口径
% 数据: Turnaround (4 lap 闭环)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
adisk= csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
ekf  = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
vo   = csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
bounds = [59;2386;4714;7037;9365];
win = 400;

% --- 构造新A-fix轨迹: 每闭环把后续段重锚到首锚窗口 (全局Sim3) ---
anchorWin = adisk(bounds(1)+1 : bounds(1)+win, :);
corr = adisk;
for i = 2:numel(bounds)
    f = bounds(i);
    if i < numel(bounds), f_next = bounds(i+1); else, f_next = size(adisk,1)+1; end
    curWin = adisk(max(f-win+1,1) : min(f, f_next-1), :);
    [~, ~, T] = procrustes(anchorWin, curWin, 'Scaling', true);
    seg = (f+1) : (f_next-1);
    corr(seg, :) = T.b * adisk(seg, :) * T.T + repmat(T.c(1,:), numel(seg), 1);
end

trajs = {'a_disk现状', adisk; '新A-fix(重锚)', corr; 'EKF', ekf; 'VO', vo};
fprintf('GT %d帧 %.1fm | lap边界 %s\n\n', size(gt,1), L, mat2str(bounds));
fprintf('路径长/GT: ');
for k = 1:size(trajs,1)
    fprintf('%s=%.3f  ', trajs{k,1}, sum(sqrt(sum(diff(trajs{k,2}).^2,2)))/L);
end
fprintf('\n\n');

% --- 口径1: 'simple' (core口径: 起点平移 + 全局长度匹配, 无旋转) ---
fprintf('=== 口径1: simple (core: 起点+长度匹配, 无旋转) ===\n');
for k = 1:size(trajs,1)
    tr = trajs{k,2};
    c = tr - tr(1,:); g = gt - gt(1,:);
    l1 = sum(sqrt(sum(diff(c).^2,2))); l2 = sum(sqrt(sum(diff(g).^2,2)));
    s = l2/l1;
    rmse = sqrt(mean(sum((c*s - g).^2,2)));
    fprintf('  %-14s ATE = %8.2f m  (scale=%.4f)\n', trajs{k,1}, rmse, s);
end

% --- 口径2: 版本A 首100帧Sim3 ---
fprintf('\n=== 口径2: 版本A (首100帧Sim3锚定, ablation Table口径) ===\n');
for k = 1:size(trajs,1)
    fprintf('  %-14s ATE = %8.2f m\n', trajs{k,1}, compute_metrics_with_alignment(trajs{k,2}, gt, L));
end

% --- 对照: per-lap(oracle) 上界 ---
fprintf('\n=== 对照: per-lap oracle (每lap独立Sim3对齐GT, 局部真实水平) ===\n');
for k = 1:2
    pl = zeros(numel(bounds)-1,1);
    for j = 1:numel(bounds)-1
        a = bounds(j)+1; b = bounds(j+1);
        pl(j) = compute_metrics_with_alignment(trajs{k,2}(a:b,:), gt(a:b,:), sum(sqrt(sum(diff(gt(a:b,:)).^2,2))));
    end
    fprintf('  %-14s 各lap: %s\n', trajs{k,1}, mat2str(pl,4));
end
fprintf('\n判读: 新A-fix 在 口径1 与 口径2 下是否都 < EKF → 机制对两种论文口径都有效.\n');
fprintf('SIMPLE_EVAL_DONE\n');
