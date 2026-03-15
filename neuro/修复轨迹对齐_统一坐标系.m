%% 修复test_imu_visual_fusion_slam2.m的轨迹对齐问题
% 核心思路: 让Baseline和Ours使用相同的轴匹配参数

fprintf('========== 修复轨迹对齐 - 统一坐标系 ==========\n\n');

% 1. 加载数据
result_file = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\comparison_results\comparison_results.mat';
if ~exist(result_file, 'file')
    error('未找到结果文件，请先运行test_imu_visual_fusion_slam2.m');
end

fprintf('[1/4] 加载数据...\n');
load(result_file);

if ~exist('gt_data', 'var')
    error('需要Ground Truth数据才能进行对齐');
end

% 2. 裁剪到相同长度
min_len = min([size(gt_data.pos, 1), size(baseline_exp_traj, 1), size(ours_exp_traj, 1)]);
fprintf('  对齐帧数: %d\n', min_len);

gt_pos = gt_data.pos(1:min_len, :);
baseline_exp = baseline_exp_traj(1:min_len, :);
baseline_odo = baseline_odo_traj(1:min_len, :);
ours_exp = ours_exp_traj(1:min_len, :);
ours_odo = ours_odo_traj(1:min_len, :);

% Town数据集: 2D对齐
gt_pos(:, 3) = 0;
baseline_exp(:, 3) = 0;
baseline_odo(:, 3) = 0;
ours_exp(:, 3) = 0;
ours_odo(:, 3) = 0;

% 3. 统一轴匹配 - 使用Baseline的匹配结果应用到Ours
fprintf('\n[2/4] 统一轴匹配...\n');

% 采样点用于匹配
n_map = min(1000, min_len);
idx_map = round(linspace(1, min_len, n_map));
gt_xy = gt_pos(idx_map, 1:2);

% 先对Baseline进行轴匹配
fprintf('  步骤1: 匹配Baseline轴...\n');
try
    [swap_bl, sx_bl, sy_bl] = best_axis_match_xy(baseline_exp(idx_map, 1:2), gt_xy);
    fprintf('    Baseline轴匹配: swap=%d, sx=%d, sy=%d\n', swap_bl, sx_bl, sy_bl);
    
    % 应用到Baseline
    bl_exp_xy = baseline_exp(:, 1:2);
    if swap_bl
        bl_exp_xy = [bl_exp_xy(:,2), bl_exp_xy(:,1)];
    end
    bl_exp_xy = [sx_bl * bl_exp_xy(:,1), sy_bl * bl_exp_xy(:,2)];
    baseline_exp(:, 1:2) = bl_exp_xy;
    
    bl_odo_xy = baseline_odo(:, 1:2);
    if swap_bl
        bl_odo_xy = [bl_odo_xy(:,2), bl_odo_xy(:,1)];
    end
    bl_odo_xy = [sx_bl * bl_odo_xy(:,1), sy_bl * bl_odo_xy(:,2)];
    baseline_odo(:, 1:2) = bl_odo_xy;
    
    fprintf('    ✓ Baseline轴匹配完成\n');
catch ME
    warning('Baseline轴匹配失败: %s', ME.message);
    swap_bl = 0; sx_bl = 1; sy_bl = 1;
end

% ★★★ 关键修复: 使用Baseline的轴匹配参数应用到Ours ★★★
fprintf('  步骤2: 应用相同轴匹配到Ours...\n');
fprintf('    使用Baseline的参数: swap=%d, sx=%d, sy=%d\n', swap_bl, sx_bl, sy_bl);

% 应用到Ours（使用相同的轴匹配参数）
ours_exp_xy = ours_exp(:, 1:2);
if swap_bl
    ours_exp_xy = [ours_exp_xy(:,2), ours_exp_xy(:,1)];
end
ours_exp_xy = [sx_bl * ours_exp_xy(:,1), sy_bl * ours_exp_xy(:,2)];
ours_exp(:, 1:2) = ours_exp_xy;

ours_odo_xy = ours_odo(:, 1:2);
if swap_bl
    ours_odo_xy = [ours_odo_xy(:,2), ours_odo_xy(:,1)];
end
ours_odo_xy = [sx_bl * ours_odo_xy(:,1), sy_bl * ours_odo_xy(:,2)];
ours_odo(:, 1:2) = ours_odo_xy;

fprintf('    ✓ Ours轴匹配完成（使用统一坐标系）\n');

% 4. Sim(3)对齐
fprintf('\n[3/4] Sim(3)对齐...\n');

align_method = 'rigid_2d';
fit_n = min(1000, min_len);
fit_idx = unique(round(linspace(1, min_len, fit_n)));

% 对齐Baseline
try
    [~, ~, R_bl, t_bl, s_bl] = align_trajectories(baseline_exp(fit_idx, :), gt_pos(fit_idx, :), align_method);
    baseline_exp_aligned = (s_bl * R_bl * baseline_exp')' + repmat(t_bl', min_len, 1);
    
    [~, ~, R_bl_odo, t_bl_odo, s_bl_odo] = align_trajectories(baseline_odo(fit_idx, :), gt_pos(fit_idx, :), align_method);
    baseline_odo_aligned = (s_bl_odo * R_bl_odo * baseline_odo')' + repmat(t_bl_odo', min_len, 1);
    
    fprintf('  ✓ Baseline对齐完成 (scale=%.3f)\n', s_bl);
catch ME
    warning('Baseline对齐失败: %s', ME.message);
    baseline_exp_aligned = baseline_exp;
    baseline_odo_aligned = baseline_odo;
end

% 对齐Ours
try
    [~, ~, R_ours, t_ours, s_ours] = align_trajectories(ours_exp(fit_idx, :), gt_pos(fit_idx, :), align_method);
    ours_exp_aligned = (s_ours * R_ours * ours_exp')' + repmat(t_ours', min_len, 1);
    
    [~, ~, R_ours_odo, t_ours_odo, s_ours_odo] = align_trajectories(ours_odo(fit_idx, :), gt_pos(fit_idx, :), align_method);
    ours_odo_aligned = (s_ours_odo * R_ours_odo * ours_odo')' + repmat(t_ours_odo', min_len, 1);
    
    fprintf('  ✓ Ours对齐完成 (scale=%.3f)\n', s_ours);
catch ME
    warning('Ours对齐失败: %s', ME.message);
    ours_exp_aligned = ours_exp;
    ours_odo_aligned = ours_odo;
end

gt_pos_aligned = gt_pos;

% 强制2D
gt_pos_aligned(:, 3) = 0;
baseline_exp_aligned(:, 3) = 0;
baseline_odo_aligned(:, 3) = 0;
ours_exp_aligned(:, 3) = 0;
ours_odo_aligned(:, 3) = 0;

% 5. 重新计算精度
fprintf('\n[4/4] 重新计算精度...\n');

bl_exp_error = sqrt(sum((baseline_exp_aligned - gt_pos_aligned).^2, 2));
bl_odo_error = sqrt(sum((baseline_odo_aligned - gt_pos_aligned).^2, 2));
ours_exp_error = sqrt(sum((ours_exp_aligned - gt_pos_aligned).^2, 2));
ours_odo_error = sqrt(sum((ours_odo_aligned - gt_pos_aligned).^2, 2));

bl_exp_ate = mean(bl_exp_error);
bl_exp_rmse = sqrt(mean(bl_exp_error.^2));
ours_exp_ate = mean(ours_exp_error);
ours_exp_rmse = sqrt(mean(ours_exp_error.^2));

ate_improvement = (bl_exp_ate - ours_exp_ate) / bl_exp_ate * 100;
rmse_improvement = (bl_exp_rmse - ours_exp_rmse) / bl_exp_rmse * 100;

fprintf('\n========== 修复后精度对比 ==========\n');
fprintf('指标              Baseline        Ours          改进\n');
fprintf('--------------------------------------------------------\n');
fprintf('经验地图 ATE      %8.3f m    %8.3f m    %+.2f%%\n', bl_exp_ate, ours_exp_ate, ate_improvement);
fprintf('经验地图 RMSE     %8.3f m    %8.3f m    %+.2f%%\n', bl_exp_rmse, ours_exp_rmse, rmse_improvement);
fprintf('里程计 ATE        %8.3f m    %8.3f m\n', mean(bl_odo_error), mean(ours_odo_error));
fprintf('里程计 RMSE       %8.3f m    %8.3f m\n', sqrt(mean(bl_odo_error.^2)), sqrt(mean(ours_odo_error.^2)));

% 6. 可视化
fprintf('\n生成修复后的可视化...\n');

figure('Name', '修复后轨迹对比', 'Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
plot(baseline_exp_aligned(1,1), baseline_exp_aligned(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(baseline_exp_aligned(end,1), baseline_exp_aligned(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
hold off;
xlabel('X (m)'); ylabel('Y (m)');
title('修复后: 经验地图轨迹对比');
legend('Location', 'best');
grid on; axis equal;

subplot(1, 2, 2);
hold on;
plot(bl_exp_error, 'b-', 'LineWidth', 1, 'DisplayName', sprintf('Baseline (%.2fm)', bl_exp_ate));
plot(ours_exp_error, 'r-', 'LineWidth', 1, 'DisplayName', sprintf('Ours (%.2fm)', ours_exp_ate));
hold off;
xlabel('Frame'); ylabel('Position Error (m)');
title('修复后: 位置误差');
legend('Location', 'best');
grid on;

sgtitle(sprintf('修复后对比 (ATE改进: %.1f%%)', ate_improvement), 'FontSize', 16, 'FontWeight', 'bold');

% 保存
result_path = fileparts(result_file);
saveas(gcf, fullfile(result_path, 'trajectory_comparison_fixed.png'));
fprintf('✓ 修复后图片已保存: trajectory_comparison_fixed.png\n');

% 保存修复后的数据
save(fullfile(result_path, 'comparison_results_fixed.mat'), ...
    'gt_pos_aligned', 'baseline_exp_aligned', 'baseline_odo_aligned', ...
    'ours_exp_aligned', 'ours_odo_aligned', ...
    'bl_exp_ate', 'bl_exp_rmse', 'ours_exp_ate', 'ours_exp_rmse', ...
    'ate_improvement', 'rmse_improvement', ...
    'swap_bl', 'sx_bl', 'sy_bl');

fprintf('\n✓ 修复后数据已保存: comparison_results_fixed.mat\n');
fprintf('\n========== 修复完成 ==========\n');
fprintf('关键修复: Baseline和Ours现在使用统一的轴匹配参数\n');
fprintf('  swap=%d, sx=%d, sy=%d\n', swap_bl, sx_bl, sy_bl);
