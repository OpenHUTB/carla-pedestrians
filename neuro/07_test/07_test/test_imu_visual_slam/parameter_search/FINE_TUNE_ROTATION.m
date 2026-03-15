%% 精细调整蓝色轨迹旋转
%  尝试更多旋转角度和镜像翻转

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('                    精细调整Baseline旋转\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

if ~exist('baseline_traj_interp', 'var')
    error('请先运行 DIAGNOSE_AND_VISUALIZE');
end

n_target = size(gt_traj_target, 1);

%% 对Baseline尝试更精细的旋转 + 镜像
angles_to_try = 0:5:355;  % 每5度
flip_options = [false, true];  % 是否X轴镜像

best_bl_ate = inf;
best_bl_aligned = [];
best_bl_angle = 0;
best_bl_flip = false;

for do_flip = flip_options
    for angle = angles_to_try
        traj = baseline_traj_interp;
        
        % 镜像翻转
        if do_flip
            traj(:,1) = -traj(:,1);
        end
        
        % 旋转
        theta = angle * pi / 180;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        traj_rotated = (R * traj')';
        
        % Sim3对齐
        aligned = align_traj_sim3(traj_rotated, gt_traj_target);
        
        % 计算ATE
        errors = sqrt(sum((aligned - gt_traj_target).^2, 2));
        ate = mean(errors);
        
        if ate < best_bl_ate
            best_bl_ate = ate;
            best_bl_aligned = aligned;
            best_bl_angle = angle;
            best_bl_flip = do_flip;
        end
    end
end

fprintf('Baseline最佳: 旋转%d度, 镜像=%d, ATE=%.2fm\n', best_bl_angle, best_bl_flip, best_bl_ate);

%% 对Ours也做同样处理
best_ours_ate = inf;
best_ours_aligned = [];
best_ours_angle = 0;
best_ours_flip = false;

for do_flip = flip_options
    for angle = angles_to_try
        traj = ours_traj_interp;
        
        if do_flip
            traj(:,1) = -traj(:,1);
        end
        
        theta = angle * pi / 180;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        traj_rotated = (R * traj')';
        
        aligned = align_traj_sim3(traj_rotated, gt_traj_target);
        
        errors = sqrt(sum((aligned - gt_traj_target).^2, 2));
        ate = mean(errors);
        
        if ate < best_ours_ate
            best_ours_ate = ate;
            best_ours_aligned = aligned;
            best_ours_angle = angle;
            best_ours_flip = do_flip;
        end
    end
end

fprintf('Ours最佳: 旋转%d度, 镜像=%d, ATE=%.2fm\n', best_ours_angle, best_ours_flip, best_ours_ate);

%% 可视化
figure('Position', [50, 50, 1000, 800], 'Color', 'w');

plot(gt_traj_target(:,1), gt_traj_target(:,2), 'g-', 'LineWidth', 3, 'DisplayName', 'Ground Truth');
hold on;
plot(best_bl_aligned(:,1), best_bl_aligned(:,2), 'b-', 'LineWidth', 2, 'DisplayName', sprintf('Baseline (ATE=%.1fm)', best_bl_ate));
plot(best_ours_aligned(:,1), best_ours_aligned(:,2), 'r-', 'LineWidth', 2, 'DisplayName', sprintf('Ours (ATE=%.1fm)', best_ours_ate));

% 起点终点
plot(gt_traj_target(1,1), gt_traj_target(1,2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(gt_traj_target(end,1), gt_traj_target(end,2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'End');

xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);

improvement = (best_bl_ate - best_ours_ate) / best_bl_ate * 100;
title(sprintf('Town01轨迹对比\nBaseline ATE=%.1fm, Ours ATE=%.1fm, 改进=%.1f%%', ...
    best_bl_ate, best_ours_ate, improvement), 'FontSize', 16, 'FontWeight', 'bold');

legend('Location', 'best', 'FontSize', 12);
grid on; axis equal;
set(gca, 'FontSize', 12);

% 保存
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
try
    print(gcf, fullfile(data_path, 'trajectory_fine_tuned.png'), '-dpng', '-r300');
    fprintf('\n图片已保存!\n');
catch
end

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('最终结果:\n');
fprintf('  Baseline ATE: %.2f m\n', best_bl_ate);
fprintf('  Ours ATE: %.2f m\n', best_ours_ate);
fprintf('  改进: %.2f%%\n', improvement);
fprintf('═══════════════════════════════════════════════════════════════\n');

%% 辅助函数
function aligned = align_traj_sim3(traj, ref)
    tc = mean(traj, 1);
    rc = mean(ref, 1);
    
    t_c = traj - tc;
    r_c = ref - rc;
    
    s = sqrt(sum(r_c(:).^2) / sum(t_c(:).^2));
    t_s = t_c * s;
    
    H = t_s' * r_c;
    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0
        V(:,end) = -V(:,end);
        R = V * U';
    end
    
    aligned = (R * t_s')' + rc;
end
