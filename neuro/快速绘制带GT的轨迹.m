%% 快速绘制带GT的轨迹对比图
% 确保GT正确显示

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  绘制带GT的轨迹对比图                                        ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 加载数据
fprintf('[1/4] 加载数据...\n');

% 数据路径
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';

% 尝试加载MAT文件
mat_file = fullfile(data_path, 'comparison_results', 'comparison_results.mat');
if exist(mat_file, 'file')
    fprintf('  从MAT文件加载...\n');
    load(mat_file);
    fprintf('  ✓ MAT文件加载成功\n');
else
    fprintf('  ✗ MAT文件不存在\n');
    return;
end

% 检查变量
fprintf('\n[2/4] 检查变量...\n');
fprintf('  baseline_exp_traj: ');
if exist('baseline_exp_traj', 'var')
    fprintf('%s\n', mat2str(size(baseline_exp_traj)));
else
    fprintf('不存在!\n');
end

fprintf('  ours_exp_traj: ');
if exist('ours_exp_traj', 'var')
    fprintf('%s\n', mat2str(size(ours_exp_traj)));
else
    fprintf('不存在!\n');
end

fprintf('  gt_data: ');
if exist('gt_data', 'var')
    fprintf('存在\n');
    if isstruct(gt_data) && isfield(gt_data, 'pos')
        fprintf('    gt_data.pos: %s\n', mat2str(size(gt_data.pos)));
    else
        fprintf('    但不是预期的结构!\n');
    end
else
    fprintf('不存在! 需要重新加载GT\n');
    
    % 重新读取GT
    fprintf('\n  正在重新读取Ground Truth...\n');
    gt_file = fullfile(data_path, 'ground_truth.txt');
    if exist(gt_file, 'file')
        gt_table = readtable(gt_file);
        gt_data = struct();
        gt_data.timestamp = gt_table.timestamp;
        % 注意：ground_truth.txt的列名是pos_x, pos_y, pos_z
        gt_data.pos = [gt_table.pos_x, gt_table.pos_y, gt_table.pos_z];
        fprintf('  ✓ GT重新加载成功: %s\n', mat2str(size(gt_data.pos)));
    else
        fprintf('  ✗ GT文件不存在: %s\n', gt_file);
        return;
    end
end

%% 3. 绘制轨迹
fprintf('\n[3/4] 绘制轨迹...\n');

% 裁剪到相同长度
min_len = min([size(baseline_exp_traj, 1), size(ours_exp_traj, 1), size(gt_data.pos, 1)]);
fprintf('  对齐长度: %d 帧\n', min_len);

baseline_trim = baseline_exp_traj(1:min_len, :);
ours_trim = ours_exp_traj(1:min_len, :);
gt_trim = gt_data.pos(1:min_len, :);

% 创建figure
fig = figure('Name', 'Town01轨迹对比(带GT)', 'Position', [100, 100, 1400, 600]);

% 子图1: Baseline vs GT
subplot(1, 2, 1);
plot(gt_trim(:,1), gt_trim(:,2), 'k--', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
hold on;
plot(baseline_trim(:,1), baseline_trim(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
hold off;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Baseline轨迹 vs GT', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
axis equal;
set(gca, 'FontSize', 11);

% 子图2: Ours vs GT
subplot(1, 2, 2);
plot(gt_trim(:,1), gt_trim(:,2), 'k--', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
hold on;
plot(ours_trim(:,1), ours_trim(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours (IMU-Visual Fusion)');
hold off;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Ours轨迹 vs GT', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
axis equal;
set(gca, 'FontSize', 11);

fprintf('  ✓ 轨迹图已生成\n');

%% 4. 计算简单的误差
fprintf('\n[4/4] 计算误差...\n');

% 简单的欧氏距离(未对齐)
baseline_errors = sqrt(sum((baseline_trim - gt_trim).^2, 2));
ours_errors = sqrt(sum((ours_trim - gt_trim).^2, 2));

fprintf('  Baseline平均误差: %.2f 米\n', mean(baseline_errors));
fprintf('  Ours平均误差: %.2f 米\n', mean(ours_errors));

if mean(ours_errors) < mean(baseline_errors)
    improvement = (mean(baseline_errors) - mean(ours_errors)) / mean(baseline_errors) * 100;
    fprintf('  ✓ Ours改进: %.1f%%\n', improvement);
else
    fprintf('  ⚠️ Ours误差更大\n');
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  完成!                                                       ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('\n注意: 这是未对齐的误差,实际ATE需要Sim(3)对齐后计算\n');
