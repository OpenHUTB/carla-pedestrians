%% 生成Town01消融实验论文图表
% 两张独立的图：
% 1. 饼图 - 组件贡献占比
% 2. 柱状图 - RMSE对比

clear; close all; clc;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   Town01 消融实验论文图表生成                            ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 配置路径
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(fileparts(script_dir))));
output_dir = fullfile(neuro_root, 'data', 'Town01Data_IMU_Fusion', 'ablation_results');

%% Town01 消融实验数据
configs_short = {'Full', 'w/o IMU', 'w/o Exp', 'w/o Trans', 'w/o Dual'};
rmse_vals = [145.46, 315.31, 186.01, 183.02, 180.03];
drift_vals = [11.93, 38.17, 17.83, 18.06, 17.56];

baseline_rmse = rmse_vals(1);
degradation = ((rmse_vals - baseline_rmse) / baseline_rmse) * 100;

%% ========== 图1：饼图（组件贡献占比）==========
fprintf('[1/2] 生成饼图...\n');

fig1 = figure('Position', [100, 100, 600, 500], 'Color', 'white');

contributions = degradation(2:end);
total_contrib = sum(contributions);
contrib_percent = contributions / total_contrib * 100;

pie_labels = {'IMU Fusion', 'Experience Map', 'Transformer', 'Dual-Stream'};
pie_colors = [0.85, 0.40, 0.15;   % 橙红 - IMU
              0.35, 0.58, 0.78;   % 中蓝
              0.50, 0.70, 0.85;   % 浅蓝
              0.68, 0.82, 0.92];  % 最浅蓝

[sorted_contrib, idx] = sort(contrib_percent, 'descend');
sorted_labels = pie_labels(idx);
sorted_colors = pie_colors(idx, :);

p = pie(sorted_contrib);

for i = 1:length(sorted_contrib)
    p(2*i-1).FaceColor = sorted_colors(i, :);
    p(2*i-1).EdgeColor = [0.3 0.3 0.3];
    p(2*i-1).LineWidth = 1.5;
    p(2*i).String = sprintf('%s\n%.1f%%', sorted_labels{i}, sorted_contrib(i));
    p(2*i).FontSize = 12;
    p(2*i).FontWeight = 'bold';
end

title('Component Contribution Ratio (Town01)', 'FontSize', 16, 'FontWeight', 'bold');

% 保存fig
savefig(fig1, fullfile(output_dir, 'ablation_pie.fig'));
fprintf('  ✓ ablation_pie.fig\n');

%% ========== 图2：柱状图（RMSE对比）==========
fprintf('[2/2] 生成柱状图...\n');

fig2 = figure('Position', [150, 150, 700, 500], 'Color', 'white');

bar_colors = [0.20, 0.50, 0.72;   % 深蓝 - Full
              0.85, 0.40, 0.15;   % 橙红 - w/o IMU
              0.40, 0.62, 0.80;   % 中蓝
              0.52, 0.72, 0.86;   % 浅蓝
              0.65, 0.82, 0.92];  % 最浅蓝

b = bar(rmse_vals, 0.65);
b.FaceColor = 'flat';
b.CData = bar_colors;
b.EdgeColor = [0.25 0.25 0.25];
b.LineWidth = 1.5;

set(gca, 'XTickLabel', configs_short, 'FontSize', 12);
ylabel('RMSE (m)', 'FontSize', 14, 'FontWeight', 'bold');
title('RMSE Comparison (Town01)', 'FontSize', 16, 'FontWeight', 'bold');

hold on;
for i = 1:5
    text(i, rmse_vals(i) + 12, sprintf('%.0fm', rmse_vals(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    if i > 1
        text(i, rmse_vals(i) + 30, sprintf('+%.0f%%', degradation(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0.7 0.2 0.1], 'FontWeight', 'bold');
    end
end
hold off;

grid on;
set(gca, 'GridAlpha', 0.3);
ylim([0, max(rmse_vals) * 1.18]);

% 保存fig
savefig(fig2, fullfile(output_dir, 'ablation_bar.fig'));
fprintf('  ✓ ablation_bar.fig\n');

%% 完成
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   ✅ 图表生成完成！                                      ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('📁 输出目录: %s\n', output_dir);
fprintf('\n');
fprintf('📌 手动导出矢量图步骤:\n');
fprintf('   1. 在Figure窗口点击: 文件 -> 另存为\n');
fprintf('   2. 保存类型选择: PDF文件(*.pdf) 或 EPS文件(*.eps)\n');
fprintf('   3. 文件名: ablation_pie.pdf / ablation_bar.pdf\n');
fprintf('\n');
