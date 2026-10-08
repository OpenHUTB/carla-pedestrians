% tar_global_reanchor.m — 新A-fix形式(闭环时重锚全局Sim3: 旋转+尺度+平移, 而非仅位置snap) 离线验证
% 非oracle: 锚点 = a_disk 自身首锚窗口(lap0), 每个闭环把"当前到访lap"重锚到该锚点
% 模拟 NLM 锚点节点机制: 闭环校正 = 把当前帧的漂移(全局Sim3)清零, 拉回锚点帧
% 目标: 版本A(首100帧Sim3, 论文Table口径) ATE 从 a_disk 861m 降到 << EKF 745m
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
adisk= csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
ekf  = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
bounds = [59;2386;4714;7037;9365];   % lap 边界(闭环点)
win = 400;                            % 拟合窗口

fprintf('GT %d帧 %.1fm | lap边界 %s\n\n', size(gt,1), L, mat2str(bounds));

% 锚点窗口 = a_disk 首锚(lap0)
a0 = bounds(1);
anchorWin = adisk(a0+1 : a0+win, :);

corr = adisk;
for i = 2:numel(bounds)
    f = bounds(i);
    if i < numel(bounds), f_next = bounds(i+1); else, f_next = size(adisk,1)+1; end
    curWin = adisk(max(f-win+1,1) : min(f, f_next-1), :);   % 当前lap窗口(原始帧)
    [~, ~, T] = procrustes(anchorWin, curWin, 'Scaling', true);  % T: cur -> anchor (全局Sim3重锚)
    seg = (f+1) : (f_next-1);
    corr(seg, :) = T.b * adisk(seg, :) * T.T + repmat(T.c(1,:), numel(seg), 1);
    r = acosd((trace(T.T)-1)/2); if r>180, r=360-r; end
    fprintf('  lap%d 闭环@%5d: 重锚 scale=%.3f rot=%6.2fdeg |平移|=%7.2fm\n', i-1, f, T.b, r, norm(T.c));
end

fprintf('\n=== 版本A(首100帧Sim3, 论文Table口径) ATE ===\n');
fprintf('  a_disk 现状(A-fix 仅位置snap)   = %8.2f m\n', compute_metrics_with_alignment(adisk, gt, L));
fprintf('  a_disk + 全局Sim3重锚(新A-fix)  = %8.2f m\n', compute_metrics_with_alignment(corr, gt, L));
fprintf('  EKF 基线                        = %8.2f m\n', compute_metrics_with_alignment(ekf, gt, L));
% 对照: per-lap(oracle逐lap对齐GT) 上界
nl = numel(bounds)-1; pl = zeros(nl,1);
for j = 1:nl
    a = bounds(j)+1; b = bounds(j+1);
    pl(j) = compute_metrics_with_alignment(adisk(a:b,:), gt(a:b,:), sum(sqrt(sum(diff(gt(a:b,:)).^2,2))));
end
fprintf('\n  per-lap(oracle, 上界) 各lap: %s  (NLM闭环局部真实水平)\n', mat2str(pl,4));
fprintf('\n判读: 新A-fix 版本A << EKF(745) 且 接近 per-lap(16.6) → 全局Sim3重锚是真实且可达的增益.\n');
fprintf('      若 新A-fix 仍接近 861 → 锚点匹配质量是瓶颈, 需先修闭环检测.\n');
fprintf('GLOBAL_REANCHOR_DONE\n');
