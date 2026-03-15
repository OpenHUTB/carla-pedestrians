function [swap, sx, sy] = best_axis_match_xy(est_xy, gt_xy)
    % BEST_AXIS_MATCH_XY 找到最佳的轴匹配（处理坐标系差异）
    %
    % 在SLAM系统中，估计轨迹和GT轨迹可能存在坐标轴交换或翻转的情况
    % 本函数尝试所有可能的组合，找到误差最小的匹配
    %
    % 输入:
    %   est_xy - [N×2] 估计轨迹的XY坐标
    %   gt_xy  - [N×2] Ground Truth的XY坐标
    %
    % 输出:
    %   swap - 是否需要交换X和Y轴 (true/false)
    %   sx   - X轴符号 (+1 或 -1)
    %   sy   - Y轴符号 (+1 或 -1)
    %
    % 使用方法:
    %   [swap, sx, sy] = best_axis_match_xy(est_xy, gt_xy);
    %   if swap
    %       est_xy = [est_xy(:,2), est_xy(:,1)];
    %   end
    %   est_xy = [sx * est_xy(:,1), sy * est_xy(:,2)];
    
    % 确保输入是N×2矩阵
    if size(est_xy, 2) ~= 2 || size(gt_xy, 2) ~= 2
        error('输入必须是N×2矩阵');
    end
    
    if size(est_xy, 1) ~= size(gt_xy, 1)
        error('两个轨迹长度必须相同');
    end
    
    % 中心化（去除平移影响）
    est_centered = est_xy - mean(est_xy, 1);
    gt_centered = gt_xy - mean(gt_xy, 1);
    
    % 归一化（去除缩放影响）
    est_scale = max(std(est_centered(:)), 1e-6);
    gt_scale = max(std(gt_centered(:)), 1e-6);
    est_norm = est_centered / est_scale;
    gt_norm = gt_centered / gt_scale;
    
    % 尝试所有8种组合 (2种交换 × 4种符号)
    best_error = inf;
    swap = false;
    sx = 1;
    sy = 1;
    
    for do_swap = [false, true]
        for sign_x = [-1, 1]
            for sign_y = [-1, 1]
                % 应用变换
                if do_swap
                    test_xy = [est_norm(:,2), est_norm(:,1)];
                else
                    test_xy = est_norm;
                end
                test_xy = [sign_x * test_xy(:,1), sign_y * test_xy(:,2)];
                
                % 计算误差（使用相关性或MSE）
                % 方法1: 使用MSE
                error_mse = mean(sum((test_xy - gt_norm).^2, 2));
                
                % 方法2: 使用负相关性（越小越好）
                % corr_x = corr(test_xy(:,1), gt_norm(:,1));
                % corr_y = corr(test_xy(:,2), gt_norm(:,2));
                % error_corr = -0.5 * (corr_x + corr_y);
                
                current_error = error_mse;
                
                if current_error < best_error
                    best_error = current_error;
                    swap = do_swap;
                    sx = sign_x;
                    sy = sign_y;
                end
            end
        end
    end
    
    % 输出调试信息
    % fprintf('  轴匹配结果: swap=%d, sx=%d, sy=%d, error=%.4f\n', swap, sx, sy, best_error);
end
