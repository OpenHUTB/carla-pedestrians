function [traj1_aligned, traj2_aligned, R, t, scale] = align_trajectories(traj1, traj2, method)
    % ALIGN_TRAJECTORIES 对齐两条轨迹（去除平移、旋转、缩放差异）
    %
    % 输入:
    %   traj1 - [N×3] 第一条轨迹（待对齐）
    %   traj2 - [N×3] 第二条轨迹（参考轨迹）
    %   method - 对齐方法: 'umeyama'（Umeyama算法，支持缩放）或 'umeyama_reflect'（允许反射，选最优）或 'simple'（简单对齐）
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
            % 简单对齐：平移+旋转+缩放
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
            
            % 限制缩放范围
            scale = max(0.1, min(10, scale));
            
            % 4. 计算初始方向（前10帧的平均方向，用于旋转对齐）
            n_frames = min(10, size(traj1, 1));
            if n_frames > 1
                % 计算初始运动方向
                dir1 = traj1_centered(n_frames, 1:2) - traj1_centered(1, 1:2);
                dir2 = traj2_centered(n_frames, 1:2) - traj2_centered(1, 1:2);
                
                % 计算旋转角度（只在XY平面）
                angle1 = atan2(dir1(2), dir1(1));
                angle2 = atan2(dir2(2), dir2(1));
                angle_diff = angle2 - angle1;
                
                % 创建Z轴旋转矩阵
                R = eye(3);
                R(1:2, 1:2) = [cos(angle_diff), -sin(angle_diff); 
                               sin(angle_diff), cos(angle_diff)];
            else
                R = eye(3);
            end
            
            % 5. 应用缩放和旋转
            traj1_aligned = (R * (traj1_centered * scale)')';
            traj2_aligned = traj2_centered;
            
            t = start1 - start2;

        case 'rigid_2d'
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);

            X = (traj1(:, 1:2) - mean_traj1(1:2))';
            Y = (traj2(:, 1:2) - mean_traj2(1:2))';

            n = size(X, 2);
            H2 = (X * Y') / max(n, 1);
            [U2, ~, V2] = svd(H2);

            S2 = eye(2);
            if det(U2) * det(V2) < 0
                S2(2, 2) = -1;
            end
            R2 = V2 * S2 * U2';

            scale = 1.0;
            t2 = mean_traj2(1:2)' - R2 * mean_traj1(1:2)';

            R = eye(3);
            R(1:2, 1:2) = R2;
            t = [t2; mean_traj2(3) - mean_traj1(3)];

            traj1_aligned = (R * traj1')' + repmat(t', size(traj1, 1), 1);
            traj2_aligned = traj2;

        case 'rigid_3d'
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);

            X = (traj1 - mean_traj1)';
            Y = (traj2 - mean_traj2)';

            n = size(X, 2);
            H = (X * Y') / max(n, 1);
            [U, ~, V] = svd(H);

            S = eye(3);
            if det(U) * det(V) < 0
                S(3, 3) = -1;
            end
            R = V * S * U';

            scale = 1.0;
            t = mean_traj2' - R * mean_traj1';

            traj1_aligned = (R * traj1')' + repmat(t', size(traj1, 1), 1);
            traj2_aligned = traj2;

        case 'umeyama_2d'
            % Umeyama算法（2D）：仅在XY平面估计相似变换（旋转+平移+缩放），避免Z维退化导致的翻转
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);

            X = (traj1(:, 1:2) - mean_traj1(1:2))';  % 2×N
            Y = (traj2(:, 1:2) - mean_traj2(1:2))';  % 2×N

            n = size(X, 2);
            H2 = (X * Y') / max(n, 1); % 2×2
            [U2, D2, V2] = svd(H2);

            S2 = eye(2);
            if det(U2) * det(V2) < 0
                S2(2, 2) = -1;
            end

            R2 = V2 * S2 * U2';
            var1 = sum(sum(X.^2)) / max(n, 1);
            if var1 > 0
                scale = trace(D2 * S2) / var1;
            else
                scale = 1.0;
            end

            if scale > 10
                warning('缩放因子过大 (%.2f)，限制为10', scale);
                scale = 10;
            elseif scale < 0.01
                warning('缩放因子过小 (%.2f)，限制为0.01', scale);
                scale = 0.01;
            end

            t2 = mean_traj2(1:2)' - scale * R2 * mean_traj1(1:2)';

            R = eye(3);
            R(1:2, 1:2) = R2;
            t = [t2; mean_traj2(3) - scale * mean_traj1(3)];

            traj1_aligned = (scale * R * traj1')' + t';
            traj2_aligned = traj2;

        case 'umeyama_2d_reflect'
            % Umeyama算法（2D，允许反射）：在不反射和反射两种解里选择误差更小的
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);

            X = (traj1(:, 1:2) - mean_traj1(1:2))';  % 2×N
            Y = (traj2(:, 1:2) - mean_traj2(1:2))';  % 2×N

            n = size(X, 2);
            H2 = (X * Y') / max(n, 1);
            [U2, D2, V2] = svd(H2);

            var1 = sum(sum(X.^2)) / max(n, 1);
            if var1 <= 0
                R = eye(3);
                t = (mean_traj2 - mean_traj1)';
                scale = 1.0;
                traj1_aligned = traj1 + repmat(t', size(traj1, 1), 1);
                traj2_aligned = traj2;
            else
                % Candidate 1: no reflection
                S1 = eye(2);
                R21 = V2 * S1 * U2';
                s1 = trace(D2 * S1) / var1;
                s1 = max(0.01, min(10, s1));
                t21 = mean_traj2(1:2)' - s1 * R21 * mean_traj1(1:2)';
                est1 = (s1 * R21 * traj1(:, 1:2)')' + repmat(t21', size(traj1, 1), 1);
                err1 = mean(sum((est1 - traj2(:, 1:2)).^2, 2));

                % Candidate 2: allow reflection
                S2 = eye(2);
                S2(2, 2) = -1;
                R22 = V2 * S2 * U2';
                s2 = trace(D2 * S2) / var1;
                s2 = max(0.01, min(10, s2));
                t22 = mean_traj2(1:2)' - s2 * R22 * mean_traj1(1:2)';
                est2 = (s2 * R22 * traj1(:, 1:2)')' + repmat(t22', size(traj1, 1), 1);
                err2 = mean(sum((est2 - traj2(:, 1:2)).^2, 2));

                if err2 < err1
                    R2 = R22;
                    scale = s2;
                    t2 = t22;
                else
                    R2 = R21;
                    scale = s1;
                    t2 = t21;
                end

                R = eye(3);
                R(1:2, 1:2) = R2;
                t = [t2; mean_traj2(3) - scale * mean_traj1(3)];

                traj1_aligned = (scale * R * traj1')' + t';
                traj2_aligned = traj2;
            end
            
        case 'umeyama'
            % Umeyama算法：计算最优的相似变换（旋转+平移+缩放）
            % 参考: Umeyama, "Least-squares estimation of transformation parameters
            %       between two point patterns", IEEE TPAMI, 1991
            
            % 1. 中心化
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);
            
            X = (traj1 - mean_traj1)';  % 3×N
            Y = (traj2 - mean_traj2)';  % 3×N
            
            % 2. 计算协方差矩阵
            n = size(X, 2);
            H = (X * Y') / max(n, 1);  % 3×3（按N归一化，避免scale随N放大）
            
            % 3. SVD分解
            [U, D, V] = svd(H);
            
            % 4. 计算旋转矩阵
            S = eye(3);
            if det(U) * det(V) < 0
                S(3, 3) = -1;
            end
            R = V * S * U';
            
            % 5. 计算缩放因子
            var1 = sum(sum(X.^2)) / max(n, 1);
            if var1 > 0
                scale = trace(D * S) / var1;
            else
                scale = 1.0;
            end
            
            % 限制缩放因子范围（避免异常值）
            if scale > 10
                warning('缩放因子过大 (%.2f)，限制为10', scale);
                scale = 10;
            elseif scale < 0.01
                warning('缩放因子过小 (%.2f)，限制为0.01', scale);
                scale = 0.01;
            end
            
            % 6. 计算平移向量
            t = mean_traj2' - scale * R * mean_traj1';
            
            % 7. 应用变换
            traj1_aligned = (scale * R * traj1')' + t';
            traj2_aligned = traj2;

        case 'umeyama_reflect'
            % Umeyama算法（允许反射）：在det(R)=+1和det(R)=-1两种解里选择误差更小的
            mean_traj1 = mean(traj1, 1);
            mean_traj2 = mean(traj2, 1);
            X = (traj1 - mean_traj1)';
            Y = (traj2 - mean_traj2)';

            n = size(X, 2);
            H = (X * Y') / max(n, 1);
            [U, D, V] = svd(H);
            var1 = sum(sum(X.^2)) / max(n, 1);
            if var1 <= 0
                R = eye(3);
                t = (mean_traj2 - mean_traj1)';
                scale = 1.0;
                traj1_aligned = traj1 + repmat(t', size(traj1, 1), 1);
                traj2_aligned = traj2;
            else
                S1 = eye(3);
                R1 = V * S1 * U';
                s1 = trace(D * S1) / var1;
                if s1 > 10
                    warning('缩放因子过大 (%.2f)，限制为10', s1);
                    s1 = 10;
                elseif s1 < 0.1
                    warning('缩放因子过小 (%.2f)，限制为0.1', s1);
                    s1 = 0.1;
                end
                t1 = mean_traj2' - s1 * R1 * mean_traj1';
                est1 = (s1 * R1 * traj1')' + t1';
                err1 = mean(sum((est1 - traj2).^2, 2));

                S2 = eye(3);
                S2(3, 3) = -1;
                R2 = V * S2 * U';
                s2 = trace(D * S2) / var1;
                if s2 > 10
                    warning('缩放因子过大 (%.2f)，限制为10', s2);
                    s2 = 10;
                elseif s2 < 0.1
                    warning('缩放因子过小 (%.2f)，限制为0.1', s2);
                    s2 = 0.1;
                end
                t2 = mean_traj2' - s2 * R2 * mean_traj1';
                est2 = (s2 * R2 * traj1')' + t2';
                err2 = mean(sum((est2 - traj2).^2, 2));

                if err2 < err1
                    R = R2;
                    scale = s2;
                    t = t2;
                else
                    R = R1;
                    scale = s1;
                    t = t1;
                end

                traj1_aligned = (scale * R * traj1')' + t';
                traj2_aligned = traj2;
            end
            
        otherwise
            error('未知的对齐方法: %s', method);
    end
    
    fprintf('轨迹对齐完成 (%s方法)\n', method);
    fprintf('  旋转角度: %.2f度\n', acosd((trace(R) - 1) / 2));
    fprintf('  平移距离: %.2f米\n', norm(t));
    fprintf('  缩放因子: %.4f\n', scale);
end
