
%% 生成更多消融实验图表
% 基于已有的ablation结果生成多种可视化

clear all; close all; clc;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   生成额外可视化图表                                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 加载结果
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');
results_file = fullfile(results_dir, 'complete_ablation_results.mat');

if ~exist(results_file, 'file')
    error('请先运行 RUN_COMPLETE_ABLATION 生成结果数据');
end

load(results_file);

%% 提取数据
dataset_names = fieldnames(all_results);
config_list = {'Complete', 'No_IMU', 'No_Fusion', 'Visual_Template_Only'};
config_labels = {'完整系统', '去掉IMU', '去掉融合', '仅VT匹配'};

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

%% 图1: 热图 - RMSE矩阵
fprintf('[1/6] 生成热图...\n');
fig1 = figure('Position', [100, 100, 1000, 600]);
set(fig1, 'Color', [0.95 0.95 0.97]);

% 归一化显示（相对于baseline）
normalized_rmse = zeros(size(rmse_matrix));
for i = 1:size(rmse_matrix, 1)
    normalized_rmse(i, :) = rmse_matrix(i, :) / rmse_matrix(i, 1);
end

h = heatmap(config_labels, dataset_names, normalized_rmse);
h.Title = '消融实验热图 - RMSE相对退化倍数';
h.XLabel = '系统配置';
h.YLabel = '数据集';
h.Colormap = hot;
h.ColorbarVisible = 'on';
h.CellLabelFormat = '%.1f×';
h.FontSize = 12;

saveas(fig1, fullfile(results_dir, 'ablation_heatmap.png'));
fprintf('  ✓ 保存: ablation_heatmap.png\n');

%% 图2: 瀑布图 - 组件贡献
fprintf('[2/6] 生成瀑布图...\n');
fig2 = figure('Position', [150, 150, 1400, 700]);
set(fig2, 'Color', [0.95 0.95 0.97]);

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    
    baseline = rmse_matrix(d, 1);
    increments = [baseline, diff(rmse_matrix(d, :))];
    
    % 瀑布图
    bar_data = [increments; zeros(1, length(increments))];
    bar_handle = bar(bar_data', 'stacked');
    
    % 设置颜色
    colors = [0.2, 0.6, 0.8;  % 蓝色 - baseline
              0.8, 0.2, 0.2;  % 红色 - 增量
              0.8, 0.6, 0.2;  % 橙色
              0.6, 0.2, 0.8]; % 紫色
    
    for i = 1:length(increments)
        if i <= size(colors, 1)
            bar_handle(1).FaceColor = 'flat';
            bar_handle(1).CData(i,:) = colors(i,:);
        end
    end
    
    set(gca, 'XTickLabel', config_labels);
    xtickangle(15);
    ylabel('RMSE (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('%s - 组件去除的影响', dataset_names{d}), ...
        'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    set(gca, 'GridAlpha', 0.3);
    
    % 添加累积值标注
    cumsum_vals = cumsum(increments);
    for i = 1:length(cumsum_vals)
        text(i, cumsum_vals(i), sprintf('%.1fm', cumsum_vals(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 10, 'FontWeight', 'bold');
    end
end

sgtitle('瀑布图：去除各组件的累积影响', 'FontSize', 16, 'FontWeight', 'bold');
saveas(fig2, fullfile(results_dir, 'ablation_waterfall.png'));
fprintf('  ✓ 保存: ablation_waterfall.png\n');

%% 图3: 气泡图 - RMSE vs 漂移率
fprintf('[3/6] 生成气泡图...\n');
fig3 = figure('Position', [200, 200, 1200, 600]);
set(fig3, 'Color', [0.95 0.95 0.97]);

colors_bubble = [0, 0.4470, 0.7410;    % Town01 - 蓝色
                 0.8500, 0.3250, 0.0980];  % Town10 - 橙色
markers = {'o', 's', '^', 'd'};

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    hold on;
    
    for c = 1:length(config_list)
        % X轴: RMSE, Y轴: 漂移率, 气泡大小: 终点误差
        bubble_size = final_error_matrix(d, c) * 2;
        
        scatter(rmse_matrix(d, c), drift_matrix(d, c), bubble_size, ...
            colors_bubble(d,:), markers{c}, 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        
        % 标注
        text(rmse_matrix(d, c), drift_matrix(d, c), config_labels{c}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 9, 'FontWeight', 'bold');
    end
    
    xlabel('RMSE (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('漂移率 (%)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('%s - RMSE vs 漂移率', dataset_names{d}), ...
        'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    set(gca, 'GridAlpha', 0.3);
    legend(config_labels, 'Location', 'best', 'FontSize', 10);
end

sgtitle('气泡图：RMSE vs 漂移率（气泡大小=终点误差）', 'FontSize', 16, 'FontWeight', 'bold');
saveas(fig3, fullfile(results_dir, 'ablation_bubble.png'));
fprintf('  ✓ 保存: ablation_bubble.png\n');

%% 图4: 改进百分比饼图
fprintf('[4/6] 生成饼图...\n');
fig4 = figure('Position', [250, 250, 1200, 600]);
set(fig4, 'Color', [0.95 0.95 0.97]);

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    
    baseline = rmse_matrix(d, 1);
    degradations = rmse_matrix(d, 2:end) - baseline;
    
    pie_data = [baseline, degradations];
    pie_labels = [{'Baseline'}, config_labels(2:end)];
    
    pie(pie_data);
    legend(pie_labels, 'Location', 'best', 'FontSize', 10);
    title(sprintf('%s - RMSE组成', dataset_names{d}), ...
        'FontSize', 14, 'FontWeight', 'bold');
end

sgtitle('饼图：各配置的RMSE组成分析', 'FontSize', 16, 'FontWeight', 'bold');
saveas(fig4, fullfile(results_dir, 'ablation_pie.png'));
fprintf('  ✓ 保存: ablation_pie.png\n');

%% 图5: 3D柱状图
fprintf('[5/6] 生成3D柱状图...\n');
fig5 = figure('Position', [300, 300, 1200, 700]);
set(fig5, 'Color', [0.95 0.95 0.97]);

bar3(rmse_matrix');
set(gca, 'XTickLabel', config_labels);
set(gca, 'YTickLabel', dataset_names);
xlabel('系统配置', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('数据集', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('RMSE (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('3D柱状图：RMSE对比', 'FontSize', 14, 'FontWeight', 'bold');
colormap('jet');
colorbar;
grid on;
view(45, 30);

saveas(fig5, fullfile(results_dir, 'ablation_3d_bar.png'));
fprintf('  ✓ 保存: ablation_3d_bar.png\n');

%% 图6: 综合对比雷达图（使用极坐标）
fprintf('[6/6] 生成综合雷达图...\n');

% 归一化所有指标到0-1范围
norm_rmse = 1 - (rmse_matrix - min(rmse_matrix(:))) / (max(rmse_matrix(:)) - min(rmse_matrix(:)));
norm_drift = 1 - (drift_matrix - min(drift_matrix(:))) / (max(drift_matrix(:)) - min(drift_matrix(:)));
norm_final = 1 - (final_error_matrix - min(final_error_matrix(:))) / (max(final_error_matrix(:)) - min(final_error_matrix(:)));

% 分别为每个数据集创建图表
for d = 1:length(dataset_names)
    fig6 = figure('Position', [350 + d*50, 350 + d*50, 700, 700]);
    set(fig6, 'Color', [0.95 0.95 0.97]);
    
    % 显式创建polar axes（关键！）
    pax = polaraxes('Position', [0.1 0.1 0.8 0.8]);
    hold(pax, 'on');
    
    % 组合三个指标
    combined_scores = [norm_rmse(d,:)', norm_drift(d,:)', norm_final(d,:)'];
    
    spider_labels = {'RMSE', '漂移率', '终点误差'};
    theta = linspace(0, 2*pi, size(combined_scores, 2) + 1);
    
    % 绘制每个配置
    colors_spider = lines(4);
    
    for c = 1:size(combined_scores, 1)
        r = [combined_scores(c, :), combined_scores(c, 1)];
        polarplot(pax, theta, r, '-o', 'LineWidth', 2.5, ...
            'Color', colors_spider(c,:), ...
            'MarkerSize', 8, 'MarkerFaceColor', colors_spider(c,:));
    end
    
    thetaticks(pax, rad2deg(theta(1:end-1)));
    thetaticklabels(pax, spider_labels);
    title(pax, sprintf('%s - 综合性能雷达图（归一化分数，越高越好）', dataset_names{d}), ...
        'FontSize', 14, 'FontWeight', 'bold');
    legend(pax, config_labels, 'Location', 'best', 'FontSize', 11);
    rlim(pax, [0 1]);
    
    % 保存单独的图表
    saveas(fig6, fullfile(results_dir, sprintf('ablation_comprehensive_radar_%s.png', dataset_names{d})));
    fprintf('  ✓ 保存: ablation_comprehensive_radar_%s.png\n', dataset_names{d});
end

%% 完成
fprintf('\n✅ 所有图表已生成完成！\n');
fprintf('\n📊 生成的图表列表:\n');
fprintf('   1. ablation_heatmap.png                        - 热图（RMSE退化倍数）\n');
fprintf('   2. ablation_waterfall.png                      - 瀑布图（组件累积影响）\n');
fprintf('   3. ablation_bubble.png                         - 气泡图（RMSE vs 漂移率）\n');
fprintf('   4. ablation_pie.png                            - 饼图（RMSE组成）\n');
fprintf('   5. ablation_3d_bar.png                         - 3D柱状图（全局对比）\n');
fprintf('   6. ablation_comprehensive_radar_Town01.png     - Town01综合雷达图\n');
fprintf('   7. ablation_comprehensive_radar_Town10.png     - Town10综合雷达图\n');
fprintf('\n💡 共%d张专业图表，可用于论文不同部分！\n\n', 7);
