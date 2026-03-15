%% Town01 Baseline vs Ours 轨迹可视化
% 解决坐标范围不匹配导致Ours轨迹看不见的问题

fprintf('Town01 轨迹对比可视化\n');
fprintf('====================\n\n');

% 检查变量是否存在
if ~exist('baseline_exp_aligned', 'var') || ~exist('ours_exp_aligned', 'var') || ~exist('gt_pos_aligned', 'var')
    error('请先运行 test_imu_visual_fusion_slam2.m 生成轨迹数据');
end

%% 方案1: 分别绘制两条轨迹（推荐）
figure('Name', 'Town01 轨迹对比 - 分别显示', 'Position', [100, 100, 1400, 600]);

% 左图：Baseline vs GT
subplot(1, 2, 1);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline (NeuroSLAM)');
plot(baseline_exp_aligned(1,1), baseline_exp_aligned(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(baseline_exp_aligned(end,1), baseline_exp_aligned(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
hold off;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Baseline vs Ground Truth', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
grid on;
axis equal;

% 右图：Ours vs GT
subplot(1, 2, 2);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours (IMU+HART)');
plot(ours_exp_aligned(1,1), ours_exp_aligned(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(ours_exp_aligned(end,1), ours_exp_aligned(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
hold off;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Ours vs Ground Truth', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
grid on;
axis equal;

%% 方案2: 三条轨迹在一起（不使用axis equal）
figure('Name', 'Town01 轨迹对比 - 统一显示', 'Position', [150, 150, 800, 600]);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline (NeuroSLAM)');
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours (IMU+HART)');

% 标记起点和终点
plot(gt_pos_aligned(1,1), gt_pos_aligned(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(gt_pos_aligned(end,1), gt_pos_aligned(end,2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

hold off;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Town01 轨迹对比 (经验地图)', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);
grid on;
% 不使用axis equal，让MATLAB自动调整坐标轴

%% 方案3: 使用subplot显示详细信息
figure('Name', 'Town01 详细对比', 'Position', [200, 200, 1200, 800]);

% 子图1: GT + Baseline + Ours (不使用axis equal)
subplot(2, 2, 1);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
hold off;
xlabel('X (m)'); ylabel('Y (m)');
title('全局对比');
legend('Location', 'best');
grid on;

% 子图2: 只显示Baseline
subplot(2, 2, 2);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
hold off;
xlabel('X (m)'); ylabel('Y (m)');
title('Baseline vs GT');
legend('Location', 'best');
grid on;
axis equal;

% 子图3: 只显示Ours
subplot(2, 2, 3);
hold on;
plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
hold off;
xlabel('X (m)'); ylabel('Y (m)');
title('Ours vs GT');
legend('Location', 'best');
grid on;
axis equal;

% 子图4: 误差统计
subplot(2, 2, 4);
% 计算每帧的误差
baseline_error = sqrt(sum((baseline_exp_aligned - gt_pos_aligned).^2, 2));
ours_error = sqrt(sum((ours_exp_aligned - gt_pos_aligned).^2, 2));

hold on;
plot(baseline_error, 'b-', 'LineWidth', 1, 'DisplayName', sprintf('Baseline (ATE=%.1fm)', mean(baseline_error)));
plot(ours_error, 'r-', 'LineWidth', 1, 'DisplayName', sprintf('Ours (ATE=%.1fm)', mean(ours_error)));
hold off;
xlabel('Frame');
ylabel('Error (m)');
title('逐帧误差对比');
legend('Location', 'best');
grid on;

%% 输出统计信息
fprintf('\n轨迹统计信息:\n');
fprintf('====================\n');
fprintf('Ground Truth 范围:\n');
fprintf('  X: [%.2f, %.2f] m\n', min(gt_pos_aligned(:,1)), max(gt_pos_aligned(:,1)));
fprintf('  Y: [%.2f, %.2f] m\n', min(gt_pos_aligned(:,2)), max(gt_pos_aligned(:,2)));

fprintf('\nBaseline 范围:\n');
fprintf('  X: [%.2f, %.2f] m\n', min(baseline_exp_aligned(:,1)), max(baseline_exp_aligned(:,1)));
fprintf('  Y: [%.2f, %.2f] m\n', min(baseline_exp_aligned(:,2)), max(baseline_exp_aligned(:,2)));

fprintf('\nOurs 范围:\n');
fprintf('  X: [%.2f, %.2f] m\n', min(ours_exp_aligned(:,1)), max(ours_exp_aligned(:,1)));
fprintf('  Y: [%.2f, %.2f] m\n', min(ours_exp_aligned(:,2)), max(ours_exp_aligned(:,2)));

fprintf('\n精度对比:\n');
fprintf('  Baseline ATE: %.2f m\n', mean(baseline_error));
fprintf('  Ours ATE:     %.2f m\n', mean(ours_error));
fprintf('  改进:         %.2f%% \n', (mean(baseline_error) - mean(ours_error)) / mean(baseline_error) * 100);

fprintf('\n✓ 可视化完成！\n');
fprintf('  - 图1: 分别显示Baseline和Ours\n');
fprintf('  - 图2: 三条轨迹统一显示\n');
fprintf('  - 图3: 详细对比（包含误差曲线）\n');
