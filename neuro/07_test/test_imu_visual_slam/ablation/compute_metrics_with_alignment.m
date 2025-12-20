function [rmse, final_error, drift_rate, traj_aligned] = compute_metrics_with_alignment(traj, gt, gt_length)
%% 带轨迹对齐的度量计算
% 
% 输入:
%   traj - 估计轨迹 [N x 3]
%   gt - Ground Truth轨迹 [N x 3]
%   gt_length - 真实轨迹长度 (m)
%
% 输出:
%   rmse - RMSE (m)
%   final_error - 终点误差 (m)
%   drift_rate - 漂移率 (%)
%   traj_aligned - 对齐后的轨迹 [N x 3]

    % 裁剪到相同长度
    min_len = min(size(traj,1), size(gt,1));
    traj = traj(1:min_len, :);
    gt = gt(1:min_len, :);
    
    % 使用前100帧进行对齐（避免受后期误差影响）
    align_frames = min(100, min_len);
    
    % Procrustes对齐（7-DoF: 旋转+平移+缩放）
    [~, traj_aligned, transform] = procrustes(gt(1:align_frames,:), traj(1:align_frames,:), 'Scaling', true);
    
    % 将变换应用到整个轨迹
    traj_aligned = transform.b * traj * transform.T + repmat(transform.c(1,:), min_len, 1);
    
    % 计算RMSE
    errors = sqrt(sum((traj_aligned - gt).^2, 2));
    rmse = sqrt(mean(errors.^2));
    final_error = errors(end);
    drift_rate = (final_error / gt_length) * 100;
end
