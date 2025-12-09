function generate_ablation_visualizations(results, output_dir)
%% 生成消融实验可视化图表
% 包含：柱状图、雷达图、热力图、性能对比图等

fprintf('正在生成可视化图表...\n');

%% 提取数据
exp_names = results.experiment_names;
exp_labels = {
    '完整系统';
    '去掉IMU';
    '去掉LSTM';
    '去掉Transformer';
    '去掉双流';
    '去掉注意力';
    '简化特征'
};

vt_counts = results.vt_counts;
exp_counts = results.exp_counts;
rmse_values = results.rmse_values;
rpe_values = results.rpe_values;
drift_rates = results.drift_rates;
times = results.processing_times;

%% 1. 综合性能柱状图
fig1 = figure('Position', [100, 100, 1400, 800]);
set(fig1, 'Color', 'w');

% 子图1: RMSE对比
subplot(2, 3, 1);
bars = bar(rmse_values, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];  % 蓝色 - Baseline
for i = 2:length(rmse_values)
    bars.CData(i, :) = [0.8, 0.4, 0.4];  % 红色 - 消融组
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('RMSE (米)', 'FontSize', 12, 'FontWeight', 'bold');
title('(a) 绝对轨迹误差对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
% 添加数值标签
for i = 1:length(rmse_values)
    text(i, rmse_values(i) + 5, sprintf('%.1f', rmse_values(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% 子图2: VT数量对比
subplot(2, 3, 2);
bars = bar(vt_counts, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];
for i = 2:length(vt_counts)
    bars.CData(i, :) = [0.8, 0.4, 0.4];
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('VT数量', 'FontSize', 12, 'FontWeight', 'bold');
title('(b) 视觉模板数量对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:length(vt_counts)
    text(i, vt_counts(i) + 10, sprintf('%d', vt_counts(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% 子图3: 经验节点对比
subplot(2, 3, 3);
bars = bar(exp_counts, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];
for i = 2:length(exp_counts)
    bars.CData(i, :) = [0.8, 0.4, 0.4];
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('经验节点数', 'FontSize', 12, 'FontWeight', 'bold');
title('(c) 经验地图节点对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:length(exp_counts)
    text(i, exp_counts(i) + 15, sprintf('%d', exp_counts(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% 子图4: 漂移率对比
subplot(2, 3, 4);
bars = bar(drift_rates, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];
for i = 2:length(drift_rates)
    bars.CData(i, :) = [0.8, 0.4, 0.4];
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('漂移率 (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('(d) 轨迹漂移率对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:length(drift_rates)
    text(i, drift_rates(i) + 0.3, sprintf('%.2f', drift_rates(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% 子图5: RPE对比
subplot(2, 3, 5);
bars = bar(rpe_values, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];
for i = 2:length(rpe_values)
    bars.CData(i, :) = [0.8, 0.4, 0.4];
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('RPE (米)', 'FontSize', 12, 'FontWeight', 'bold');
title('(e) 相对位姿误差对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:length(rpe_values)
    text(i, rpe_values(i) + 0.05, sprintf('%.3f', rpe_values(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% 子图6: 处理时间对比
subplot(2, 3, 6);
bars = bar(times / 60, 'FaceColor', 'flat');
bars.CData(1, :) = [0.2, 0.6, 0.8];
for i = 2:length(times)
    bars.CData(i, :) = [0.8, 0.4, 0.4];
end
set(gca, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('处理时间 (分钟)', 'FontSize', 12, 'FontWeight', 'bold');
title('(f) 计算效率对比', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:length(times)
    text(i, times(i)/60 + 0.1, sprintf('%.1f', times(i)/60), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

sgtitle('消融实验 - 综合性能对比', 'FontSize', 16, 'FontWeight', 'bold');
saveas(fig1, fullfile(output_dir, 'ablation_comprehensive_comparison.png'));
fprintf('  ✓ 综合性能柱状图已保存\n');

%% 2. 雷达图（归一化性能）- 简化版本
fig2 = figure('Position', [150, 150, 900, 700]);
set(fig2, 'Color', 'w');

% 归一化指标（值越小越好的要反转）
norm_rmse = 1 - (rmse_values - min(rmse_values)) / (max(rmse_values) - min(rmse_values));
norm_vt = (vt_counts - min(vt_counts)) / (max(vt_counts) - min(vt_counts));
norm_exp = (exp_counts - min(exp_counts)) / (max(exp_counts) - min(exp_counts));
norm_rpe = 1 - (rpe_values - min(rpe_values)) / (max(rpe_values) - min(rpe_values));
norm_drift = 1 - (drift_rates - min(drift_rates)) / (max(drift_rates) - min(drift_rates));

% 准备雷达图数据
categories = {'RMSE', 'VT Count', 'Nodes', 'RPE', 'Drift'};
data_radar = [norm_rmse, norm_vt, norm_exp, norm_rpe, norm_drift];

% 使用分组柱状图代替雷达图（更兼容）
indices_to_plot = [1, 2, 3, 7];  % 完整系统、无IMU、无LSTM、简化特征
labels_radar = {'Full System', 'w/o IMU', 'w/o LSTM', 'Simplified'};

% 准备数据
plot_data = data_radar(indices_to_plot, :) * 100;

% 绘制分组柱状图
b = bar(plot_data', 'grouped');
b(1).FaceColor = [0.2, 0.6, 0.8];
b(2).FaceColor = [0.8, 0.4, 0.4];
b(3).FaceColor = [0.6, 0.8, 0.4];
b(4).FaceColor = [0.8, 0.6, 0.4];

set(gca, 'XTickLabel', categories, 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Normalized Score (0-100)', 'FontSize', 13, 'FontWeight', 'bold');
legend(labels_radar, 'Location', 'northwest', 'FontSize', 11);
title('Ablation Study - Performance Profile', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
ylim([0, 110]);

saveas(fig2, fullfile(output_dir, 'ablation_performance_profile.png'));
fprintf('  ✓ 性能轮廓图已保存\n');

%% 3. 性能热力图
fig3 = figure('Position', [200, 200, 1000, 600]);
set(fig3, 'Color', 'w');

% 准备数据矩阵（归一化到0-100）
metrics_matrix = [
    norm_rmse * 100, ...
    norm_vt * 100, ...
    norm_exp * 100, ...
    norm_rpe * 100, ...
    norm_drift * 100
];

% 绘制热力图
imagesc(metrics_matrix);
colormap(hot);
colorbar('FontSize', 12);
caxis([0, 100]);

% 设置标签
set(gca, 'XTick', 1:5, 'XTickLabel', {'定位精度', 'VT数量', '地图节点', '相对误差', '漂移率'});
set(gca, 'YTick', 1:7, 'YTickLabel', exp_labels);
set(gca, 'FontSize', 11, 'FontWeight', 'bold');

% 添加数值标签
for i = 1:size(metrics_matrix, 1)
    for j = 1:size(metrics_matrix, 2)
        value = metrics_matrix(i, j);
        if value > 50
            text_color = 'white';
        else
            text_color = 'black';
        end
        text(j, i, sprintf('%.1f', value), ...
            'HorizontalAlignment', 'center', ...
            'FontSize', 10, 'FontWeight', 'bold', 'Color', text_color);
    end
end

title('消融实验 - 性能热力图 (归一化得分)', 'FontSize', 15, 'FontWeight', 'bold');
xlabel('性能指标', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('实验配置', 'FontSize', 13, 'FontWeight', 'bold');

saveas(fig3, fullfile(output_dir, 'ablation_heatmap.png'));
fprintf('  ✓ 性能热力图已保存\n');

%% 4. 相对性能降低图（相对Baseline）
fig4 = figure('Position', [250, 250, 1200, 700]);
set(fig4, 'Color', 'w');

% 计算相对变化率（相对完整系统）
baseline_idx = 1;
rel_rmse = ((rmse_values - rmse_values(baseline_idx)) / rmse_values(baseline_idx)) * 100;
rel_vt = ((vt_counts - vt_counts(baseline_idx)) / vt_counts(baseline_idx)) * 100;
rel_exp = ((exp_counts - exp_counts(baseline_idx)) / exp_counts(baseline_idx)) * 100;
rel_rpe = ((rpe_values - rpe_values(baseline_idx)) / rpe_values(baseline_idx)) * 100;
rel_drift = ((drift_rates - drift_rates(baseline_idx)) / drift_rates(baseline_idx)) * 100;

% 准备数据
x_positions = 2:7;  % 跳过baseline
data_rel = [
    rel_rmse(2:end), ...
    rel_vt(2:end), ...
    rel_exp(2:end), ...
    rel_rpe(2:end), ...
    rel_drift(2:end)
];

% 分组柱状图
b = bar(x_positions, data_rel);
b(1).FaceColor = [0.8, 0.3, 0.3];  % RMSE - 红色
b(2).FaceColor = [0.3, 0.6, 0.8];  % VT - 蓝色
b(3).FaceColor = [0.4, 0.8, 0.4];  % Exp - 绿色
b(4).FaceColor = [0.8, 0.6, 0.3];  % RPE - 橙色
b(5).FaceColor = [0.6, 0.4, 0.8];  % Drift - 紫色

% 设置
set(gca, 'XTick', x_positions, 'XTickLabel', exp_labels(2:end), 'XTickLabelRotation', 45);
ylabel('相对变化率 (%)', 'FontSize', 13, 'FontWeight', 'bold');
title('消融实验 - 相对完整系统的性能变化', 'FontSize', 15, 'FontWeight', 'bold');
legend({'RMSE', 'VT数量', '经验节点', 'RPE', '漂移率'}, ...
    'Location', 'northwest', 'FontSize', 11);
grid on;
hold on;
plot([1.5, 7.5], [0, 0], 'k--', 'LineWidth', 1.5);  % 基准线

saveas(fig4, fullfile(output_dir, 'ablation_relative_performance.png'));
fprintf('  ✓ 相对性能变化图已保存\n');

%% 5. 组件贡献度分析图
fig5 = figure('Position', [300, 300, 1000, 600]);
set(fig5, 'Color', 'w');

% 计算每个组件移除后的RMSE增加量
component_names = {'IMU融合', 'LSTM记忆', 'Transformer', '双流架构', '空间注意力', '完整特征'};
rmse_increases = rmse_values(2:7) - rmse_values(1);

% 按贡献度排序
[sorted_increases, sort_idx] = sort(rmse_increases, 'descend');
sorted_names = component_names(sort_idx);

% 绘制水平柱状图
barh(sorted_increases, 'FaceColor', [0.4, 0.7, 0.9]);
set(gca, 'YTick', 1:length(sorted_names), 'YTickLabel', sorted_names);
set(gca, 'FontSize', 12, 'FontWeight', 'bold');
xlabel('RMSE增加量 (米)', 'FontSize', 13, 'FontWeight', 'bold');
title('组件贡献度分析 (移除后的性能下降)', 'FontSize', 15, 'FontWeight', 'bold');
grid on;

% 添加数值标签
for i = 1:length(sorted_increases)
    text(sorted_increases(i) + 0.5, i, sprintf('+%.1f m', sorted_increases(i)), ...
        'FontSize', 11, 'FontWeight', 'bold');
end

saveas(fig5, fullfile(output_dir, 'ablation_component_contribution.png'));
fprintf('  ✓ 组件贡献度图已保存\n');

%% 6. 综合得分图（加权平均）
fig6 = figure('Position', [350, 350, 900, 600]);
set(fig6, 'Color', 'w');

% 计算综合得分（归一化后加权平均）
% 权重：RMSE(0.4) + VT(0.15) + Exp(0.15) + RPE(0.2) + Drift(0.1)
weights = [0.4, 0.15, 0.15, 0.2, 0.1];
overall_scores = data_radar * weights';

% 绘制
bars = bar(overall_scores * 100, 'FaceColor', 'flat');
for i = 1:length(overall_scores)
    if i == 1
        bars.CData(i, :) = [0.2, 0.7, 0.4];  % 绿色 - Baseline
    else
        bars.CData(i, :) = [0.7, 0.5, 0.5];  % 灰红色
    end
end

set(gca, 'XTick', 1:7, 'XTickLabel', exp_labels, 'XTickLabelRotation', 45);
ylabel('Overall Score', 'FontSize', 13, 'FontWeight', 'bold');
title('消融实验 - 综合得分对比 (满分100)', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
ylim([0, 110]);

% 添加数值标签和排名
[sorted_scores, rank_idx] = sort(overall_scores, 'descend');
for i = 1:length(overall_scores)
    score = overall_scores(i) * 100;
    rank = find(rank_idx == i);
    text(i, score + 3, sprintf('%.1f\n(#%d)', score, rank), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

saveas(fig6, fullfile(output_dir, 'ablation_overall_score.png'));
fprintf('  ✓ 综合得分图已保存\n');

fprintf('所有可视化图表生成完成！\n');

end
