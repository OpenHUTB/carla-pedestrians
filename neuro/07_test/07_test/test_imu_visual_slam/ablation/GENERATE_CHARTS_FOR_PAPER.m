%% 为论文生成高质量矢量图表
% 特点：
% 1. EPS矢量格式（可在LaTeX中编辑/拖动元素）
% 2. 大字体（易读）
% 3. 高分辨率
% 
% 使用方法：由 UPDATE_TOWN01_CHARTS.m 自动调用

clear; close all; clc;

%% 配置
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');

% 加载数据
results_file = fullfile(results_dir, 'complete_ablation_results.mat');
if ~exist(results_file, 'file')
    error('请先运行 UPDATE_TOWN01_CHARTS');
end
load(results_file);

%% 提取数据
dataset_names = fieldnames(all_results);
config_list = {'Complete', 'No_IMU', 'EKF_Baseline'};
config_labels = {'Bio-inspired (Ours)', 'w/o IMU (Pure VO)', 'EKF Baseline'};

% 构建数据矩阵
rmse_matrix = [];
drift_matrix = [];
final_error_matrix = [];

for d = 1:length(dataset_names)
    dataset_name = dataset_names{d};
    dataset_res = all_results.(dataset_name);
    
    rmse_row = [];
    drift_row = [];
    final_row = [];
    
    for c = 1:length(config_list)
        config_name = config_list{c};
        rmse_row(c) = dataset_res.experiments.(config_name).rmse;
        drift_row(c) = dataset_res.experiments.(config_name).drift_rate;
        final_row(c) = dataset_res.experiments.(config_name).final_error;
    end
    
    rmse_matrix = [rmse_matrix; rmse_row];
    drift_matrix = [drift_matrix; drift_row];
    final_error_matrix = [final_error_matrix; final_row];
end

fprintf('\n生成论文级图表（EPS矢量格式 + 大字体）\n\n');

%% 图1: 热图（增大字体）
fprintf('[1/5] 热图...\n');
fig1 = figure('Position', [100, 100, 1000, 600]);
set(fig1, 'Color', 'white');

normalized_rmse = zeros(size(rmse_matrix));
for i = 1:size(rmse_matrix, 1)
    normalized_rmse(i, :) = rmse_matrix(i, :) / rmse_matrix(i, 1);
end

h = heatmap(config_labels, dataset_names, normalized_rmse);
h.Title = 'Ablation Heatmap - RMSE Degradation Ratio';
h.XLabel = 'Configuration';
h.YLabel = 'Dataset';
% 自定义清新配色：白色（好）-> 浅蓝 -> 中蓝 -> 深蓝（差）
custom_colormap = [1.0 1.0 1.0;      % 白色（1.0x，最好）
                   0.85 0.92 0.98;   % 极浅蓝
                   0.70 0.85 0.95;   % 浅蓝
                   0.55 0.75 0.90;   % 浅中蓝
                   0.40 0.65 0.85;   % 中蓝
                   0.30 0.55 0.80;   % 深蓝
                   0.20 0.45 0.75];  % 更深蓝（2.2x，最差）
h.Colormap = custom_colormap;
h.ColorbarVisible = 'on';
h.CellLabelFormat = '%.1f×';
h.FontSize = 20;  % 增大字体（论文级别）
h.FontName = 'Arial';

saveas(fig1, fullfile(results_dir, 'ablation_heatmap.eps'), 'epsc');
saveas(fig1, fullfile(results_dir, 'ablation_heatmap.png'));
fprintf('  ✓ ablation_heatmap.eps\n');

%% 图2: 2D对比柱状图（大字体）
fprintf('[2/5] 对比柱状图...\n');
fig2 = figure('Position', [150, 150, 900, 650]);
set(fig2, 'Color', 'white');

rmse_vals = rmse_matrix(1, :);
b = bar(rmse_vals, 0.6);

colors_bar = [0.36, 0.61, 0.84;   % 柔和蓝色（Bio-inspired）
              0.93, 0.49, 0.19;   % 柔和橙色（VO）
              0.44, 0.68, 0.28];  % 柔和绿色（EKF）

b.FaceColor = 'flat';
b.CData = colors_bar;
b.EdgeColor = [0.2 0.2 0.2];
b.LineWidth = 1.5;

set(gca, 'XTickLabel', {'Bio-inspired\n(Ours)', 'w/o IMU\n(Pure VO)', 'EKF\n(Baseline)'});
set(gca, 'FontSize', 18, 'FontName', 'Arial');  % 大字体（论文级别）
ylabel('RMSE (m)', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
title('Ablation Study: RMSE Comparison', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Arial');
grid on;
set(gca, 'GridAlpha', 0.3);
ylim([0, max(rmse_vals)*1.15]);

% 数值标注
hold on;
for i = 1:length(rmse_vals)
    text(i, rmse_vals(i)+20, sprintf('%.1fm', rmse_vals(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Arial');
end

% 改进百分比
improvement_vo = ((rmse_vals(2) - rmse_vals(1)) / rmse_vals(2)) * 100;
improvement_ekf = ((rmse_vals(3) - rmse_vals(1)) / rmse_vals(3)) * 100;

text(1.5, max(rmse_vals)*0.85, sprintf('↓%.1f%%', improvement_vo), ...
    'FontSize', 17, 'Color', [0 0.6 0], 'FontWeight', 'bold', 'FontName', 'Arial');
text(2, max(rmse_vals)*1.05, sprintf('↓%.1f%%', improvement_ekf), ...
    'FontSize', 17, 'Color', [0 0.6 0], 'FontWeight', 'bold', 'FontName', 'Arial');
hold off;

saveas(fig2, fullfile(results_dir, 'ablation_comparison_bar.eps'), 'epsc');
saveas(fig2, fullfile(results_dir, 'ablation_comparison_bar.png'));
fprintf('  ✓ ablation_comparison_bar.eps\n');

%% 图3: 气泡图（大字体）
fprintf('[3/5] 气泡图...\n');
fig3 = figure('Position', [200, 200, 900, 650]);
set(fig3, 'Color', 'white');

colors_bubble = [0.36, 0.61, 0.84; 0.93, 0.49, 0.19; 0.44, 0.68, 0.28];

for d = 1:length(dataset_names)
    for c = 1:length(config_list)
        % 增大标记尺寸：除以2而不是5，最小200
        marker_size = max(final_error_matrix(d, c) / 2, 200);
        
        if c == 1
            marker = 'o';
        elseif c == 2
            marker = 's';
        else
            marker = '^';
        end
        
        scatter(rmse_matrix(d, c), drift_matrix(d, c), marker_size, ...
            colors_bubble(c,:), 'filled', 'Marker', marker, ...
            'MarkerEdgeColor', [0.2 0.2 0.2], 'LineWidth', 2.5);
        hold on;
        
        text(rmse_matrix(d, c)+10, drift_matrix(d, c), config_labels{c}, ...
            'FontSize', 16, 'FontName', 'Arial');
    end
end

xlabel('RMSE (m)', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
ylabel('Drift Rate (%)', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
title('Bubble Chart: RMSE vs Drift Rate', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(dataset_names, 'Location', 'best', 'FontSize', 17, 'FontName', 'Arial');
grid on;
set(gca, 'GridAlpha', 0.3, 'FontSize', 18, 'FontName', 'Arial');
hold off;

saveas(fig3, fullfile(results_dir, 'ablation_bubble.eps'), 'epsc');
saveas(fig3, fullfile(results_dir, 'ablation_bubble.png'));
fprintf('  ✓ ablation_bubble.eps\n');

%% 图4: 雷达图（大字体）
fprintf('[4/5] 雷达图...\n');

% 归一化
norm_rmse = 1 - (rmse_matrix - min(rmse_matrix(:))) / (max(rmse_matrix(:)) - min(rmse_matrix(:)));
norm_drift = 1 - (drift_matrix - min(drift_matrix(:))) / (max(drift_matrix(:)) - min(drift_matrix(:)));
norm_final = 1 - (final_error_matrix - min(final_error_matrix(:))) / (max(final_error_matrix(:)) - min(final_error_matrix(:)));

for d = 1:length(dataset_names)
    fig4 = figure('Position', [250, 250, 800, 800]);
    set(fig4, 'Color', 'white');
    
    pax = polaraxes('Position', [0.1 0.1 0.8 0.8]);
    hold(pax, 'on');
    
    combined_scores = [norm_rmse(d,:)', norm_drift(d,:)', norm_final(d,:)'];
    spider_labels = {'RMSE', 'Drift Rate', 'End Error'};
    theta = linspace(0, 2*pi, size(combined_scores, 2) + 1);
    
    colors_spider = [0.36, 0.61, 0.84; 0.93, 0.49, 0.19; 0.44, 0.68, 0.28];
    
    for c = 1:size(combined_scores, 1)
        r = [combined_scores(c, :), combined_scores(c, 1)];
        polarplot(pax, theta, r, '-o', 'LineWidth', 3.5, ...
            'Color', colors_spider(c,:), 'MarkerSize', 14, ...
            'MarkerFaceColor', colors_spider(c,:));
    end
    
    pax.ThetaTick = rad2deg(theta(1:end-1));
    pax.ThetaTickLabel = spider_labels;
    pax.FontSize = 20;  % 大字体（论文级别）
    pax.FontName = 'Arial';
    pax.RLim = [0 1];
    pax.RTick = [0 0.2 0.4 0.6 0.8 1.0];
    
    title(pax, sprintf('%s - Comprehensive Radar (Normalized, Higher is Better)', dataset_names{d}), ...
        'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Arial');
    legend(pax, config_labels, 'Location', 'northeast', 'FontSize', 18, 'FontName', 'Arial');
    hold(pax, 'off');
    
    saveas(fig4, fullfile(results_dir, sprintf('ablation_radar_%s.eps', dataset_names{d})), 'epsc');
    saveas(fig4, fullfile(results_dir, sprintf('ablation_radar_%s.png', dataset_names{d})));
    fprintf('  ✓ ablation_radar_%s.eps\n', dataset_names{d});
    close(fig4);
end

%% 图5: 综合分析图（4子图）
fprintf('[5/5] 综合分析图...\n');
fig5 = figure('Position', [100, 100, 1600, 1200]);
set(fig5, 'Color', 'white');

% 子图1: RMSE对比
subplot(2, 2, 1);
b1 = bar(rmse_matrix', 'grouped');
set(gca, 'XTickLabel', config_labels, 'FontSize', 17, 'FontName', 'Arial');
ylabel('RMSE (m)', 'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Arial');
title('(a) RMSE Comparison', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(dataset_names, 'FontSize', 16, 'FontName', 'Arial');
grid on;

% 子图2: 漂移率对比
subplot(2, 2, 2);
b2 = bar(drift_matrix', 'grouped');
set(gca, 'XTickLabel', config_labels, 'FontSize', 17, 'FontName', 'Arial');
ylabel('Drift Rate (%)', 'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Arial');
title('(b) Drift Rate Comparison', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(dataset_names, 'FontSize', 16, 'FontName', 'Arial');
grid on;

% 子图3: 终点误差
subplot(2, 2, 3);
b3 = bar(final_error_matrix', 'grouped');
set(gca, 'XTickLabel', config_labels, 'FontSize', 17, 'FontName', 'Arial');
ylabel('Final Error (m)', 'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Arial');
title('(c) Final Error Comparison', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(dataset_names, 'FontSize', 16, 'FontName', 'Arial');
grid on;

% 子图4: 归一化雷达图（使用polaraxes）
pax = polaraxes('Position', [0.55 0.1 0.4 0.35]);  % 手动设置位置对应subplot(2,2,4)
hold(pax, 'on');
colors_radar = [0.36 0.61 0.84; 0.93 0.49 0.19; 0.44 0.68 0.28];
for c = 1:3
    combined = [norm_rmse(1,c), norm_drift(1,c), norm_final(1,c), norm_rmse(1,c)];
    theta = linspace(0, 2*pi, 4);
    polarplot(pax, theta, combined, '-o', 'LineWidth', 2.5, 'MarkerSize', 8, ...
        'Color', colors_radar(c,:), 'MarkerFaceColor', colors_radar(c,:));
end
title(pax, '(d) Normalized Performance', 'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Arial');
legend(pax, config_labels, 'FontSize', 16, 'FontName', 'Arial', 'Location', 'best');
pax.FontSize = 17;
pax.FontName = 'Arial';
hold(pax, 'off');

sgtitle('Comprehensive Ablation Analysis', 'FontSize', 24, 'FontWeight', 'bold', 'FontName', 'Arial');

saveas(fig5, fullfile(results_dir, 'ablation_comprehensive.eps'), 'epsc');
saveas(fig5, fullfile(results_dir, 'ablation_comprehensive.png'));
fprintf('  ✓ ablation_comprehensive.eps\n');

fprintf('\n✅ 所有图表已生成（EPS矢量 + PNG预览）\n');
fprintf('📁 保存位置: %s\n\n', results_dir);
