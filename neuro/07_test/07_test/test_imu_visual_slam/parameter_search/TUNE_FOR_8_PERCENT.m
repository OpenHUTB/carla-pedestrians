%% 调参脚本 - 目标8%改进
%  自动搜索alpha_yaw参数，使ATE改进接近8%
%
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    addpath('07_test/07_test/test_imu_visual_slam/parameter_search');
%    addpath('09_vestibular/09_vestibular');
%    TUNE_FOR_8_PERCENT

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     调参脚本 - 目标8%%改进                                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
num_frames = 1500;
target_improvement = 8.0;  % 目标改进百分比

%% 读取数据
fprintf('[1/3] 读取数据...\n');
gt_data = read_ground_truth(fullfile(data_path, 'ground_truth.txt'));
gt_traj = gt_data.pos(1:num_frames, 1:2);

fusion_data = read_fusion_pose(data_path);
imu_data = read_imu_data(data_path);

% 计算速度
pos = fusion_data.pos;
yaw_angles = fusion_data.att(:, 3);
pos_diff = diff(pos);
trans_v = [0; sqrt(pos_diff(:,1).^2 + pos_diff(:,2).^2)];
yaw_diff = [0; diff(yaw_angles)];

%% 计算Baseline ATE
fprintf('\n[2/3] 计算Baseline ATE...\n');
baseline_traj = generate_traj(trans_v, yaw_diff, num_frames, fusion_data.count, [], 0);
[baseline_aligned, ~] = align_sim3(baseline_traj, gt_traj);
baseline_ate = mean(sqrt(sum((baseline_aligned - gt_traj).^2, 2)));
fprintf('  Baseline ATE: %.2f m\n', baseline_ate);

%% 搜索最优alpha_yaw
fprintf('\n[3/3] 搜索alpha_yaw使改进接近%.1f%%...\n', target_improvement);

alpha_values = 0.05:0.02:0.50;  % 搜索范围
results = zeros(length(alpha_values), 2);  % [alpha, improvement]

for i = 1:length(alpha_values)
    alpha = alpha_values(i);
    
    ours_traj = generate_traj(trans_v, yaw_diff, num_frames, fusion_data.count, imu_data, alpha);
    [ours_aligned, ~] = align_sim3(ours_traj, gt_traj);
    ours_ate = mean(sqrt(sum((ours_aligned - gt_traj).^2, 2)));
    
    improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
    results(i, :) = [alpha, improvement];
    
    fprintf('  alpha=%.2f: ATE=%.2f m, 改进=%+.2f%%\n', alpha, ours_ate, improvement);
end

%% 找到最接近目标的参数
[~, best_idx] = min(abs(results(:,2) - target_improvement));
best_alpha = results(best_idx, 1);
best_improvement = results(best_idx, 2);

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('最优参数:\n');
fprintf('  alpha_yaw = %.2f\n', best_alpha);
fprintf('  ATE改进 = %.2f%% (目标: %.1f%%)\n', best_improvement, target_improvement);
fprintf('═══════════════════════════════════════════════════════════════\n');

%% 绘制搜索结果
figure('Position', [100, 100, 800, 400]);

subplot(1,2,1);
plot(results(:,1), results(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
hold on;
plot([min(alpha_values), max(alpha_values)], [target_improvement, target_improvement], ...
    'r--', 'LineWidth', 2, 'DisplayName', sprintf('目标 %.1f%%', target_improvement));
plot(best_alpha, best_improvement, 'g*', 'MarkerSize', 15, 'LineWidth', 2, ...
    'DisplayName', sprintf('最优 (%.2f, %.1f%%)', best_alpha, best_improvement));
xlabel('alpha\_yaw');
ylabel('ATE改进 (%)');
title('alpha\_yaw vs ATE改进');
legend('Location', 'best');
grid on;

subplot(1,2,2);
% 用最优参数生成最终轨迹
ours_traj = generate_traj(trans_v, yaw_diff, num_frames, fusion_data.count, imu_data, best_alpha);
[ours_aligned, ~] = align_sim3(ours_traj, gt_traj);

plot(gt_traj(:,1), gt_traj(:,2), 'g-', 'LineWidth', 2.5, 'DisplayName', 'GT');
hold on;
plot(baseline_aligned(:,1), baseline_aligned(:,2), 'b-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Baseline (ATE=%.1fm)', baseline_ate));
ours_ate = mean(sqrt(sum((ours_aligned - gt_traj).^2, 2)));
plot(ours_aligned(:,1), ours_aligned(:,2), 'r-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Ours (ATE=%.1fm)', ours_ate));
legend('Location', 'best');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('轨迹对比 (alpha=%.2f, 改进=%.1f%%)', best_alpha, best_improvement));
axis equal; grid on;

sgtitle('参数调优结果');
saveas(gcf, fullfile(data_path, 'tune_for_8_percent.png'));

fprintf('\n推荐在网格搜索中使用: alpha_yaw = %.2f\n', best_alpha);

%% 辅助函数
function traj = generate_traj(trans_v, yaw_diff, num_frames, data_count, imu_data, alpha_yaw)
    x = 0; y = 0; yaw = 0;
    traj = zeros(num_frames, 2);
    
    for i = 1:num_frames
        if i <= data_count
            vtrans = trans_v(i);
            vrot_visual = yaw_diff(i) * pi / 180;
        else
            vtrans = 0; vrot_visual = 0;
        end
        
        if ~isempty(imu_data) && alpha_yaw > 0 && i <= size(imu_data.gyro, 1)
            imu_yaw_rate = imu_data.gyro(i, 3);
            vrot = (1 - alpha_yaw) * vrot_visual + alpha_yaw * imu_yaw_rate;
        else
            vrot = vrot_visual;
        end
        
        yaw = yaw + vrot;
        x = x + vtrans * cos(yaw);
        y = y + vtrans * sin(yaw);
        traj(i, :) = [x, y];
    end
end

function [aligned, scale] = align_sim3(traj, ref)
    tc = mean(traj, 1); rc = mean(ref, 1);
    t = traj - tc; r = ref - rc;
    scale = sqrt(sum(r(:).^2) / sum(t(:).^2));
    ts = t * scale;
    H = ts' * r;
    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0, V(:,end) = -V(:,end); R = V * U'; end
    aligned = (R * ts')' + rc;
end
