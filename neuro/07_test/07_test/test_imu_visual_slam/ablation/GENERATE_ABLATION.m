%% 生成纯消融实验图表（7个图表，不含EKF）
% 适合论文Ablation Study部分
% 只包含3个配置：Complete, w/o ExpMap, w/o IMU

clear all; close all; clc;

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   生成消融实验图表（7个图表，纯消融）                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% 加载结果
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');
results_file = fullfile(results_dir, 'ablation_results_aligned.mat');

if ~exist(results_file, 'file')
    error('请先运行 RUN_ABLATION 生成结果数据');
end

load(results_file);

%% 提取数据
dataset_names = fieldnames(all_results);

% 只使用消融实验的3个配置
ablation_config_list = {'Complete', 'No_ExpMap', 'No_IMU'};
ablation_config_labels = {'Complete System', 'w/o Experience Map', 'w/o IMU Fusion'};

% 构建数据矩阵
ablation_rmse_matrix = [];
ablation_drift_matrix = [];
ablation_final_error_matrix = [];

for d = 1:length(dataset_names)
    dataset_name = dataset_names{d};
    
    rmse_row = [];
    drift_row = [];
    final_row = [];
    
    for c = 1:length(ablation_config_list)
        config_name = ablation_config_list{c};
        rmse_row(c) = all_results.(dataset_name).(config_name).rmse;
        drift_row(c) = all_results.(dataset_name).(config_name).drift_rate;
        final_row(c) = all_results.(dataset_name).(config_name).final_error;
    end
    
    ablation_rmse_matrix = [ablation_rmse_matrix; rmse_row];
    ablation_drift_matrix = [ablation_drift_matrix; drift_row];
    ablation_final_error_matrix = [ablation_final_error_matrix; final_row];
end

%% 图1: 热图 - 消融实验RMSE退化倍数
fprintf('[1/7] 生成消融实验热图...\n');
fig1 = figure('Position', [100, 100, 900, 500]);
set(fig1, 'Color', 'white');

% 归一化显示（相对于baseline）
normalized_rmse = zeros(size(ablation_rmse_matrix));
for i = 1:size(ablation_rmse_matrix, 1)
    normalized_rmse(i, :) = ablation_rmse_matrix(i, :) / ablation_rmse_matrix(i, 1);
end

% 修复数据集名称显示
dataset_labels_display = cellfun(@(x) strrep(x, '_', '\_'), dataset_names, 'UniformOutput', false);
h = heatmap(ablation_config_labels, dataset_labels_display, normalized_rmse);
h.Title = 'Ablation Study Heatmap - RMSE Degradation Ratio';
h.XLabel = 'Configuration';
h.YLabel = 'Dataset';
h.Colormap = parula;
h.FontSize = 14;
h.CellLabelFormat = '%.2f';

saveas(fig1, fullfile(results_dir, 'ablation_heatmap.png'));
fprintf('  ✓ 保存: ablation_heatmap.png\n');

%% 图2: 清晰的消融实验柱状图（主图）
fprintf('[2/7] 生成消融实验主图（柱状图）...\n');
fig2 = figure('Position', [150, 150, 1600, 800]);
set(fig2, 'Color', 'white');

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    
    bar_data = ablation_rmse_matrix(d, :);
    bar_handle = bar(bar_data, 'FaceColor', 'flat');
    
    % 颜色：绿色（最优）→ 橙色 → 红色（最差）
    colors = [
        0.0, 0.7, 0.3;  % 深绿 - Complete 
        1.0, 0.6, 0.0;  % 橙色 - w/o ExpMap
        0.9, 0.2, 0.2;  % 红色 - w/o IMU 
    ];
    bar_handle.CData = colors;
    
    set(gca, 'XTickLabel', ablation_config_labels);
    set(gca, 'XTickLabelRotation', 20);
    set(gca, 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('RMSE (m)', 'FontSize', 24, 'FontWeight', 'bold');
    
    dataset_name_display = strrep(dataset_names{d}, '_', '\_');
    title(sprintf('%s Dataset', dataset_name_display), ...
        'FontSize', 24, 'FontWeight', 'bold');
    grid on;
    set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.5);
    
    % 添加数值标注
    for i = 1:length(bar_data)
        if bar_data(i) < 10
            text(i, bar_data(i), sprintf('%.3f m', bar_data(i)), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 18, 'FontWeight', 'bold', 'Color', 'black');
        else
            text(i, bar_data(i), sprintf('%.2f m', bar_data(i)), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 18, 'FontWeight', 'bold', 'Color', 'black');
        end
    end
    
    ylim([0, max(bar_data)*1.2]);
end

sgtitle('Ablation Study: Component Contribution Analysis', ...
    'FontSize', 28, 'FontWeight', 'bold');
saveas(fig2, fullfile(results_dir, 'ablation_main_figure.png'));
fprintf('  ✓ 保存: ablation_main_figure.png\n');

%% 图3: 气泡图 - RMSE vs 漂移率
fprintf('[3/7] 生成气泡图...\n');
fig3 = figure('Position', [200, 200, 1800, 900]);
set(fig3, 'Color', 'white');

colors_bubble = [
    0.0, 0.7, 0.3;   % 深绿 - Complete
    1.0, 0.6, 0.0;   % 橙色 - w/o ExpMap
    0.9, 0.2, 0.2;   % 红色 - w/o IMU
];

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    hold on;
    
    for c = 1:length(ablation_config_list)
        % 气泡大小根据终点误差
        bubble_size = ablation_final_error_matrix(d, c) * 20;
        scatter(ablation_rmse_matrix(d, c), ablation_drift_matrix(d, c), ...
            bubble_size, colors_bubble(c,:), 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % 添加配置标签
        text(ablation_rmse_matrix(d, c), ablation_drift_matrix(d, c), ablation_config_labels{c}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 13, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
    end
    
    xlabel('RMSE (m)', 'FontSize', 18, 'FontWeight', 'bold');
    ylabel('Drift Rate (%)', 'FontSize', 18, 'FontWeight', 'bold');
    dataset_name_display = strrep(dataset_names{d}, '_', '\_');
    title(sprintf('%s - RMSE vs Drift Rate\\fontsize{14}(Bubble Size = End Error)', dataset_name_display), ...
        'FontSize', 20, 'FontWeight', 'bold');
    grid on;
    set(gca, 'FontSize', 14, 'GridAlpha', 0.3);
    hold off;
end

sgtitle('Ablation Study: RMSE vs Drift Rate Comparison', ...
    'FontSize', 24, 'FontWeight', 'bold');
saveas(fig3, fullfile(results_dir, 'ablation_bubble.png'));
fprintf('  ✓ 保存: ablation_bubble.png\n');

%% 图4: 性能退化百分比图
fprintf('[4/7] 生成性能退化分析图...\n');
fig4 = figure('Position', [250, 250, 1600, 800]);
set(fig4, 'Color', 'white');

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    
    baseline = ablation_rmse_matrix(d, 1);
    degradation_pct = (ablation_rmse_matrix(d, 2:end) - baseline) / baseline * 100;
    
    bar_handle = bar(degradation_pct, 'FaceColor', 'flat');
    bar_handle.CData = [
        1.0, 0.6, 0.0;  % 橙色
        0.9, 0.2, 0.2;  % 红色
    ];
    
    set(gca, 'XTickLabel', ablation_config_labels(2:end));
    set(gca, 'XTickLabelRotation', 20);
    set(gca, 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Performance Degradation (%)', 'FontSize', 24, 'FontWeight', 'bold');
    
    dataset_name_display = strrep(dataset_names{d}, '_', '\_');
    title(sprintf('%s Dataset', dataset_name_display), ...
        'FontSize', 24, 'FontWeight', 'bold');
    grid on;
    set(gca, 'GridAlpha', 0.3, 'LineWidth', 1.5);
    
    % 添加数值标注
    for i = 1:length(degradation_pct)
        if abs(degradation_pct(i)) < 10
            text(i, degradation_pct(i), sprintf('+%.2f%%', degradation_pct(i)), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 18, 'FontWeight', 'bold');
        else
            text(i, degradation_pct(i), sprintf('+%.1f%%', degradation_pct(i)), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 18, 'FontWeight', 'bold');
        end
    end
    
    ylim([0, max(degradation_pct)*1.3]);
end

sgtitle('Ablation Study: Performance Degradation without Components', ...
    'FontSize', 28, 'FontWeight', 'bold');
saveas(fig4, fullfile(results_dir, 'ablation_degradation_figure.png'));
fprintf('  ✓ 保存: ablation_degradation_figure.png\n');

%% 图5: 3D柱状图
fprintf('[5/7] 生成3D柱状图...\n');
fig5 = figure('Position', [300, 300, 1200, 700]);
set(fig5, 'Color', 'white');

bar3(ablation_rmse_matrix');
colormap(jet);
set(gca, 'XTickLabel', ablation_config_labels, 'FontSize', 14, 'FontWeight', 'bold');
dataset_labels_display = cellfun(@(x) strrep(x, '_', '\_'), dataset_names, 'UniformOutput', false);
set(gca, 'YTickLabel', dataset_labels_display, 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Configuration', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Dataset', 'FontSize', 16, 'FontWeight', 'bold');
zlabel('RMSE (m)', 'FontSize', 16, 'FontWeight', 'bold');
title('3D Bar Chart: Complete RMSE Comparison (Ablation Study)', ...
    'FontSize', 20, 'FontWeight', 'bold');
colorbar;
view(45, 30);

saveas(fig5, fullfile(results_dir, 'ablation_3d_bar.png'));
fprintf('  ✓ 保存: ablation_3d_bar.png\n');

%% 图6: 综合对比雷达图（单独归一化每个数据集）
fprintf('[6/7] 生成综合雷达图...\n');

for d = 1:length(dataset_names)
    % 为每个数据集单独归一化
    dataset_rmse = ablation_rmse_matrix(d, :);
    dataset_drift = ablation_drift_matrix(d, :);
    dataset_final = ablation_final_error_matrix(d, :);
    
    % 归一化：越小越好，所以用1减去
    if max(dataset_rmse) > min(dataset_rmse)
        norm_rmse_d = 1 - (dataset_rmse - min(dataset_rmse)) / (max(dataset_rmse) - min(dataset_rmse));
    else
        norm_rmse_d = ones(size(dataset_rmse));
    end
    
    if max(dataset_drift) > min(dataset_drift)
        norm_drift_d = 1 - (dataset_drift - min(dataset_drift)) / (max(dataset_drift) - min(dataset_drift));
    else
        norm_drift_d = ones(size(dataset_drift));
    end
    
    if max(dataset_final) > min(dataset_final)
        norm_final_d = 1 - (dataset_final - min(dataset_final)) / (max(dataset_final) - min(dataset_final));
    else
        norm_final_d = ones(size(dataset_final));
    end
    
    fig6 = figure('Position', [350 + d*50, 350 + d*50, 900, 900]);
    set(fig6, 'Color', 'white');
    
    pax = polaraxes('Position', [0.1 0.1 0.8 0.8]);
    hold(pax, 'on');
    
    % 组合三个指标
    combined_scores = [norm_rmse_d', norm_drift_d', norm_final_d'];
    
    spider_labels = {'RMSE', 'Drift Rate', 'End Error'};
    theta = linspace(0, 2*pi, size(combined_scores, 2) + 1);
    
    % 绘制每个配置
    colors_spider = [
        0.0, 0.7, 0.3;   % 深绿 - Complete
        1.0, 0.6, 0.0;   % 橙色 - w/o ExpMap
        0.9, 0.2, 0.2;   % 红色 - w/o IMU
    ];
    
    for c = 1:size(combined_scores, 1)
        r = [combined_scores(c, :), combined_scores(c, 1)];
        polarplot(pax, theta, r, '-o', 'LineWidth', 4.5, ...
            'Color', colors_spider(c,:), ...
            'MarkerSize', 14, 'MarkerFaceColor', colors_spider(c,:), ...
            'DisplayName', ablation_config_labels{c});
    end
    
    thetaticks(pax, rad2deg(theta(1:end-1)));
    thetaticklabels(pax, spider_labels);
    
    dataset_name_display = strrep(dataset_names{d}, '_', '\_');
    title(pax, sprintf('%s - Performance Radar\\fontsize{16}(Outer = Better)', dataset_name_display), ...
        'FontSize', 24, 'FontWeight', 'bold');
    legend(pax, ablation_config_labels, 'Location', 'best', 'FontSize', 18);
    rlim(pax, [0 1]);
    
    pax.GridColor = [0.3 0.3 0.3];
    pax.GridAlpha = 0.6;
    pax.FontSize = 18;
    pax.FontWeight = 'bold';
    
    saveas(fig6, fullfile(results_dir, sprintf('ablation_radar_%s.png', dataset_names{d})));
    fprintf('  ✓ 保存: ablation_radar_%s.png\n', dataset_names{d});
end

%% 图7: 组件贡献分解图（堆叠柱状图）
fprintf('[7/7] 生成组件贡献分解图...\n');
fig7 = figure('Position', [400, 400, 1400, 700]);
set(fig7, 'Color', 'white');

for d = 1:length(dataset_names)
    subplot(1, 2, d);
    
    baseline = ablation_rmse_matrix(d, 1);
    
    % 计算每个组件的贡献（RMSE增量）
    expmap_contribution = ablation_rmse_matrix(d, 2) - ablation_rmse_matrix(d, 1);
    imu_contribution = ablation_rmse_matrix(d, 3) - ablation_rmse_matrix(d, 2);
    
    % 堆叠柱状图数据
    stack_data = [
        baseline, 0, 0;  % Complete: 只有baseline
        baseline, expmap_contribution, 0;  % w/o ExpMap: baseline + expmap
        baseline, expmap_contribution, imu_contribution;  % w/o IMU: 全部
    ];
    
    bar_handle = bar(stack_data, 'stacked');
    bar_handle(1).FaceColor = [0.0, 0.7, 0.3];  % 绿色 - baseline
    bar_handle(2).FaceColor = [1.0, 0.6, 0.0];  % 橙色 - ExpMap contribution
    bar_handle(3).FaceColor = [0.9, 0.2, 0.2];  % 红色 - IMU contribution
    
    set(gca, 'XTickLabel', ablation_config_labels);
    set(gca, 'XTickLabelRotation', 20);
    set(gca, 'FontSize', 18, 'FontWeight', 'bold');
    ylabel('RMSE (m)', 'FontSize', 20, 'FontWeight', 'bold');
    
    dataset_name_display = strrep(dataset_names{d}, '_', '\_');
    title(sprintf('%s - Component Contribution', dataset_name_display), ...
        'FontSize', 22, 'FontWeight', 'bold');
    legend({'Baseline', 'ExpMap Impact', 'IMU Impact'}, ...
        'Location', 'northwest', 'FontSize', 14);
    grid on;
    set(gca, 'GridAlpha', 0.3);
    
    ylim([0, max(ablation_rmse_matrix(d,:))*1.2]);
end

sgtitle('Ablation Study: Component Contribution Breakdown', ...
    'FontSize', 26, 'FontWeight', 'bold');
saveas(fig7, fullfile(results_dir, 'ablation_component_breakdown.png'));
fprintf('  ✓ 保存: ablation_component_breakdown.png\n');

%% 完成
fprintf('\n✅ 所有消融实验图表已生成完成！\n');
fprintf('\n📊 生成的图表列表（7个，纯消融实验）:\n');
fprintf('   1. ablation_heatmap.png                  - 热图：RMSE退化倍数\n');
fprintf('   2. ablation_main_figure.png              - 主图：RMSE柱状对比\n');
fprintf('   3. ablation_bubble.png                   - 气泡图：RMSE vs 漂移率\n');
fprintf('   4. ablation_degradation_figure.png       - 性能退化百分比\n');
fprintf('   5. ablation_3d_bar.png                   - 3D柱状图：完整对比\n');
fprintf('   6. ablation_radar_*.png                  - 雷达图（每个数据集）\n');
fprintf('   7. ablation_component_breakdown.png      - 组件贡献分解图\n');
fprintf('\n💡 这些图表适合论文Ablation Study部分\n');
fprintf('   - 只包含3个配置（你系统的组件）\n');
fprintf('   - 不含EKF（纯消融实验）\n');
fprintf('   - 字体大，颜色清晰\n');
fprintf('   - 7种不同可视化角度\n\n');
