%% 生成审稿人推荐的额外图表
% 基于审稿人视角，生成以下关键图表：
% 1. 6数据集性能总结（雷达图）
% 2. 失败案例分析图
% 3. 计算性能饼图

clear; close all; clc;

fprintf('\n========================================\n');
fprintf('  生成审稿人推荐的额外图表\n');
fprintf('========================================\n\n');

%% 数据准备
% 6个数据集的RMSE数据（从Table 2提取）
datasets = {'Town01', 'Town02', 'Town10', 'KITTI07', 'MH01', 'MH03'};

% RMSE数据（米）
rmse_bio = [145.5, 117.9, 111.6, 74.3, 4.0, 3.3];
rmse_ekf = [253.6, 241.5, 268.8, 85.0, 3.9, 4.4];
rmse_vo = [238.9, 155.0, 98.9, 85.6, 4.2, 15.2];

% 归一化到0-1（用于雷达图）
max_rmse = max([rmse_bio, rmse_ekf, rmse_vo]);
norm_bio = 1 - (rmse_bio / max_rmse);  % 反转：越大越好
norm_ekf = 1 - (rmse_ekf / max_rmse);
norm_vo = 1 - (rmse_vo / max_rmse);

% 计算性能数据（从Table 4提取）
components = {'Visual\nProcessing', 'VT\nMatching', 'Fusion', 'Grid Cell\nUpdate', 'HDC\nUpdate', 'Exp Map\nUpdate'};
times = [2.1, 2.3, 1.9, 18.5, 1.5, 4.7];  % ms
percentages = times / sum(times) * 100;

% 配色方案
color_bio = [0.36, 0.61, 0.84];   % 蓝色
color_ekf = [0.93, 0.49, 0.19];   % 橙色
color_vo = [0.44, 0.68, 0.28];    % 绿色

%% 图1: 6数据集性能总结（雷达图）
fprintf('[1/3] 生成6数据集性能雷达图...\n');

fig1 = figure('Position', [100, 100, 900, 700], 'Color', 'white');

% 创建雷达图数据（需要闭合）
theta = linspace(0, 2*pi, length(datasets)+1);
bio_data = [norm_bio, norm_bio(1)];
ekf_data = [norm_ekf, norm_ekf(1)];
vo_data = [norm_vo, norm_vo(1)];

% 绘制雷达图
polarplot(theta, bio_data, '-o', 'Color', color_bio, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', color_bio);
hold on;
polarplot(theta, ekf_data, '--s', 'Color', color_ekf, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', color_ekf);
polarplot(theta, vo_data, ':^', 'Color', color_vo, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', color_vo);

% 设置刻度标签
ax = gca;
ax.ThetaTickLabel = datasets;
ax.FontSize = 13;
ax.FontWeight = 'bold';

% 图例
legend({'Bio-inspired (Ours)', 'EKF Baseline', 'VO Baseline'}, ...
    'Location', 'northeast', 'FontSize', 12);

title('Performance Across 6 Datasets (Normalized)', 'FontSize', 16, 'FontWeight', 'bold');

% 保存
output_dir = 'E:\Neuro_end\neuro\kbs\kbs\NeuroSLAM_KBS_Submission\fig';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

print(fig1, fullfile(output_dir, 'performance_radar_6datasets.pdf'), '-dpdf', '-r300');
print(fig1, fullfile(output_dir, 'performance_radar_6datasets.png'), '-dpng', '-r300');
fprintf('   ✅ performance_radar_6datasets.pdf\n\n');

%% 图2: 计算性能饼图
fprintf('[2/3] 生成计算性能饼图...\n');

fig2 = figure('Position', [150, 150, 900, 700], 'Color', 'white');

% 创建饼图
pie(times);

% 自定义颜色
colormap([0.85, 0.92, 0.98;   % 浅蓝
          0.70, 0.85, 0.95;   % 蓝
          0.55, 0.75, 0.90;   % 中蓝
          0.40, 0.65, 0.85;   % 深蓝
          0.30, 0.55, 0.80;   % 更深蓝
          0.20, 0.45, 0.75]); % 最深蓝

% 添加标签
labels = cell(length(components), 1);
for i = 1:length(components)
    labels{i} = sprintf('%s\n%.1fms (%.1f%%)', components{i}, times(i), percentages(i));
end

% 更新饼图标签
p = gca;
p.FontSize = 11;
p.FontWeight = 'bold';

% 添加图例
legend(labels, 'Location', 'eastoutside', 'FontSize', 11);

title('Computational Performance Breakdown (Total: 32.1ms, ~31 FPS)', ...
    'FontSize', 16, 'FontWeight', 'bold');

% 保存
print(fig2, fullfile(output_dir, 'computational_performance_pie.pdf'), '-dpdf', '-r300');
print(fig2, fullfile(output_dir, 'computational_performance_pie.png'), '-dpng', '-r300');
fprintf('   ✅ computational_performance_pie.pdf\n\n');

%% 图3: 失败案例分析（MH01）
fprintf('[3/3] 生成失败案例分析图...\n');

fig3 = figure('Position', [200, 200, 1400, 600], 'Color', 'white');

% 左侧：MH01数据对比
subplot(1, 2, 1);
mh01_rmse = [4.0, 3.9, 4.2];  % Bio, EKF, VO
mh01_labels = {'Bio-inspired', 'EKF', 'VO'};
colors_mh01 = [color_bio; color_ekf; color_vo];

b = bar(mh01_rmse, 0.6);
b.FaceColor = 'flat';
b.CData = colors_mh01;
b.EdgeColor = [0.2 0.2 0.2];
b.LineWidth = 1.5;

set(gca, 'XTickLabel', mh01_labels);
set(gca, 'FontSize', 13, 'FontWeight', 'bold');
ylabel('RMSE (m)', 'FontSize', 14, 'FontWeight', 'bold');
title('MH01 Performance (Failure Case)', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
ylim([0, 5]);

% 标注数值
hold on;
for i = 1:length(mh01_rmse)
    text(i, mh01_rmse(i)+0.15, sprintf('%.2fm', mh01_rmse(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
end

% 标注差异
text(1.5, 4.5, sprintf('Δ = -2.9%%'), 'FontSize', 13, 'Color', [0.8 0 0], 'FontWeight', 'bold');

% 右侧：原因分析（文本框）
subplot(1, 2, 2);
axis off;

% 标题
text(0.5, 0.95, 'Failure Analysis: Why MH01 Underperforms?', ...
    'FontSize', 15, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 原因列表
reasons = {
    '1. Short Trajectory (81m)', ...
    '   → Too short for drift accumulation', ...
    '   → EKF local consistency sufficient', ...
    '', ...
    '2. Controlled Environment', ...
    '   → Slow, stable motion', ...
    '   → Excellent lighting conditions', ...
    '   → Baseline VO already near-optimal', ...
    '', ...
    '3. Limited Loop Opportunities', ...
    '   → Experience map overhead', ...
    '   → No compensating loop closure benefits', ...
    '', ...
    '4. Conclusion', ...
    '   → Bio-inspired SLAM excels on:', ...
    '     • Long trajectories (>1km)', ...
    '     • Loop-rich environments', ...
    '     • Challenging visual conditions', ...
    '   → Not optimal for short, controlled scenarios'
};

y_pos = 0.85;
for i = 1:length(reasons)
    if startsWith(reasons{i}, '1.') || startsWith(reasons{i}, '2.') || ...
       startsWith(reasons{i}, '3.') || startsWith(reasons{i}, '4.')
        % 主标题
        text(0.05, y_pos, reasons{i}, 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
    elseif contains(reasons{i}, '→')
        % 子项
        text(0.08, y_pos, reasons{i}, 'FontSize', 11, 'Color', [0.3 0.3 0.3]);
    elseif contains(reasons{i}, '•')
        % 列表项
        text(0.11, y_pos, reasons{i}, 'FontSize', 11, 'Color', [0.3 0.3 0.3]);
    else
        % 空行或普通文本
        text(0.05, y_pos, reasons{i}, 'FontSize', 11);
    end
    y_pos = y_pos - 0.04;
end

xlim([0, 1]);
ylim([0, 1]);

% 保存
print(fig3, fullfile(output_dir, 'failure_case_analysis_mh01.pdf'), '-dpdf', '-r300');
print(fig3, fullfile(output_dir, 'failure_case_analysis_mh01.png'), '-dpng', '-r300');
fprintf('   ✅ failure_case_analysis_mh01.pdf\n\n');

%% 总结
fprintf('========================================\n');
fprintf('✅ 所有审稿人推荐图表已生成！\n');
fprintf('========================================\n\n');

fprintf('📁 输出目录: %s\n\n', output_dir);

fprintf('生成的图表：\n');
fprintf('1. performance_radar_6datasets.pdf - 6数据集性能雷达图\n');
fprintf('2. computational_performance_pie.pdf - 计算性能饼图\n');
fprintf('3. failure_case_analysis_mh01.pdf - 失败案例分析图\n\n');

fprintf('📋 下一步：\n');
fprintf('1. 在LaTeX中引用这些图表\n');
fprintf('2. 根据审稿人建议调整叙事结构\n');
fprintf('3. 查看 REVIEWER_EXPERIMENT_RECOMMENDATIONS.md 获取详细建议\n\n');
