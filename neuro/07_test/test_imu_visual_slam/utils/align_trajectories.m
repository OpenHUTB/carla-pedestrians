function [traj1_aligned, traj2_aligned, R, t, scale] = align_trajectories(traj1, traj2, method)
    % ALIGN_TRAJECTORIES 对齐两条轨迹（去除平移、旋转、缩放差异）
    %
    % 输入:
    %   traj1 - [N×3] 第一条轨迹（待对齐）
    %   traj2 - [N×3] 第二条轨迹（参考轨迹）
    %   method - 对齐方法: 'umeyama'（Umeyama算法，支持缩放）或 'simple'（简单对齐）
    %
    % 输出:
    %   traj1_aligned - 对齐后的第一条轨迹
    %   traj2_aligned - 对齐后的第二条轨迹（平移到原点）
    %   R - 旋转矩阵
    %   t - 平移向量
    %   scale - 缩放因子
    
    if nargin < 3
        method = 'simple';
    end
    
    % 确保输入是N×3矩阵
    if size(traj1, 1) ~= size(traj2, 1)
        error('两条轨迹长度必须相同');
    end
    
    if size(traj1, 2) ~= 3 || size(traj2, 2) ~= 3
        error('轨迹必须是N×3矩阵');
    end
    
    switch method
        case 'simple'
            % 简单对齐：平移+缩放（使轨迹长度匹配）
            % 1. 平移：将两条轨迹的起点对齐到原点
            start1 = traj1(1, :);
            start2 = traj2(1, :);
            
            traj1_centered = traj1 - start1;
            traj2_centered = traj2 - start2;
            
            % 2. 计算轨迹长度
            len1 = sum(sqrt(sum(diff(traj1_centered).^2, 2)));
            len2 = sum(sqrt(sum(diff(traj2_centered).^2, 2)));
            
            % 3. 计算缩放因子（使轨迹长度匹配）
            if len1 > 0
                scale = len2 / len1;
            else
                scale = 1.0;
            end
            
            % 4. 应用缩放
            traj1_aligned = traj1_centered * scale;
            traj2_aligned = traj2_centered;
            
            R = eye(3);
            t = start1 - start2;
            
        case 'umeyama'
            % Umeyama算法：计算最优的相似变换（旋转+平移+缩放）
            % 参考: Umeyama, "Least-squares estimation of transformation parameters
            %       between two point patterns", IEEE TPAMI, 1991
            
            % 1. 中心化
            mean1 = mean(traj1, 1);
            mean2 = mean(traj2, 1);
            
            X = (traj1 - mean1)';  % 3×N
            Y = (traj2 - mean2)';  % 3×N
            
            % 2. 计算协方差矩阵
            H = X * Y';  % 3×3
            
            % 3. SVD分解
            [U, ~, V] = svd(H);
            
            % 4. 计算旋转矩阵
            S = eye(3);
            if det(U) * det(V) < 0
                S(3, 3) = -1;
            end
            R = V * S * U';
            
            % 5. 计算缩放因子
            var1 = sum(sum(X.^2)) / size(X, 2);
            scale = trace(diag(svd(H)) * S) / var1;
            
            % 6. 计算平移向量
            t = mean2' - scale * R * mean1';
            
            % 7. 应用变换
            traj1_aligned = (scale * R * traj1')' + t';
            traj2_aligned = traj2;
            
        otherwise
            error('未知的对齐方法: %s', method);
    end
    
    fprintf('轨迹对齐完成 (%s方法)\n', method);
    fprintf('  旋转角度: %.2f度\n', acosd(trace(R)/2 - 0.5));
    fprintf('  平移距离: %.2f米\n', norm(t));
    fprintf('  缩放因子: %.4f\n', scale);
end
