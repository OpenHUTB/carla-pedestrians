function [metrics] = evaluate_slam_accuracy(estimated_trajectory, ground_truth, save_dir, method_name)
%EVALUATE_SLAM_ACCURACY 评估SLAM轨迹精度
%   计算多种精度指标来评估SLAM算法性能
%   
%   输入:
%       estimated_trajectory - 估计轨迹 [x, y, z] (N x 3)
%       ground_truth - 真值轨迹 [x, y, z] (N x 3)
%       save_dir - 保存目录（可选，默认当前目录）
%       method_name - 方法名称（可选，用于生成文件名）
%   输出:
%       metrics - 精度指标结构体,包含:
%           rmse - 均方根误差
%           ate - 绝对轨迹误差
%           rpe - 相对位姿误差
%           final_error - 终点误差
%           drift_rate - 漂移率
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   Accuracy Evaluation Module (2024)

    if nargin < 3
        save_dir = '.';
    end
    if nargin < 4
        method_name = '';
    end

    if size(estimated_trajectory, 1) ~= size(ground_truth, 1)
        error('估计轨迹和真值轨迹长度不匹配');
    end
    
    N = size(estimated_trajectory, 1);
    
    % 1. 绝对轨迹误差 (Absolute Trajectory Error, ATE)
    pos_errors = sqrt(sum((estimated_trajectory - ground_truth).^2, 2));
    metrics.ate.mean = mean(pos_errors);
    metrics.ate.median = median(pos_errors);
    metrics.ate.std = std(pos_errors);
    metrics.ate.rmse = sqrt(mean(pos_errors.^2));
    metrics.ate.max = max(pos_errors);
    metrics.ate.min = min(pos_errors);
    
    % 2. 相对位姿误差 (Relative Pose Error, RPE)
    % 计算连续帧之间的相对误差
    if N > 1
        est_diff = diff(estimated_trajectory);
        gt_diff = diff(ground_truth);
        rpe_errors = sqrt(sum((est_diff - gt_diff).^2, 2));
        metrics.rpe.mean = mean(rpe_errors);
        metrics.rpe.median = median(rpe_errors);
        metrics.rpe.std = std(rpe_errors);
        metrics.rpe.rmse = sqrt(mean(rpe_errors.^2));
    else
        metrics.rpe = struct('mean', 0, 'median', 0, 'std', 0, 'rmse', 0);
    end
    
    % 3. 终点误差
    metrics.final_error = norm(estimated_trajectory(end,:) - ground_truth(end,:));
    
    % 4. 轨迹长度
    est_length = sum(sqrt(sum(diff(estimated_trajectory).^2, 2)));
    gt_length = sum(sqrt(sum(diff(ground_truth).^2, 2)));
    metrics.trajectory_length.estimated = est_length;
    metrics.trajectory_length.ground_truth = gt_length;
    metrics.trajectory_length.error = abs(est_length - gt_length);
    metrics.trajectory_length.error_ratio = metrics.trajectory_length.error / gt_length;
    
    % 5. 漂移率 (Drift Rate)
    % 定义为终点误差与轨迹长度的比值
    metrics.drift_rate = metrics.final_error / gt_length;
    
    % 6. 每段误差分析 (将轨迹分为10段)
    segment_size = floor(N / 10);
    if segment_size > 0
        for i = 1:10
            start_idx = (i-1) * segment_size + 1;
            end_idx = min(i * segment_size, N);
            segment_errors = pos_errors(start_idx:end_idx);
            metrics.segment_errors(i) = mean(segment_errors);
        end
    else
        metrics.segment_errors = mean(pos_errors);
    end
    
    % 7. 打印结果
    fprintf('\n========== SLAM精度评估结果 ==========\n');
    fprintf('绝对轨迹误差 (ATE):\n');
    fprintf('  RMSE:     %.4f m\n', metrics.ate.rmse);
    fprintf('  平均值:   %.4f m\n', metrics.ate.mean);
    fprintf('  中位数:   %.4f m\n', metrics.ate.median);
    fprintf('  标准差:   %.4f m\n', metrics.ate.std);
    fprintf('  最大值:   %.4f m\n', metrics.ate.max);
    fprintf('  最小值:   %.4f m\n', metrics.ate.min);
    fprintf('\n');
    
    fprintf('相对位姿误差 (RPE):\n');
    fprintf('  RMSE:     %.4f m\n', metrics.rpe.rmse);
    fprintf('  平均值:   %.4f m\n', metrics.rpe.mean);
    fprintf('  中位数:   %.4f m\n', metrics.rpe.median);
    fprintf('  标准差:   %.4f m\n', metrics.rpe.std);
    fprintf('\n');
    
    fprintf('轨迹长度:\n');
    fprintf('  估计值:   %.2f m\n', metrics.trajectory_length.estimated);
    fprintf('  真值:     %.2f m\n', metrics.trajectory_length.ground_truth);
    fprintf('  误差:     %.2f m (%.2f%%)\n', ...
        metrics.trajectory_length.error, metrics.trajectory_length.error_ratio * 100);
    fprintf('\n');
    
    fprintf('终点误差:   %.4f m\n', metrics.final_error);
    fprintf('漂移率:     %.4f%% (终点误差/轨迹长度)\n', metrics.drift_rate * 100);
    fprintf('======================================\n\n');
    
    % 8. 绘制增强的误差分析图（2x3布局）- 专业高级设计
    fig = figure('Name', 'SLAM Accuracy Evaluation', 'Position', [50, 50, 1600, 900]);
    set(fig, 'Color', [0.96 0.96 0.98]);  % 极淡灰蓝背景
    
    % 子图1: ATE随时间变化 - 专业对比度
    subplot(2, 3, 1);
    hold on;
    
    % 专业网格 - 中等对比度
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');  % 纯白背景
    
    % 高对比度配色 - 深蓝/深橙/深绿
    plot(1:N, pos_errors, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 3.0);
    plot([1, N], [metrics.ate.mean, metrics.ate.mean], '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 3.0);
    plot([1, N], [metrics.ate.rmse, metrics.ate.rmse], '--', 'Color', [0.47 0.67 0.19], 'LineWidth', 3.0);  % 深绿
    
    % 加强坐标轴文字
    xlabel('Frame Index', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Position Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('Trajectory Error Evolution', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    % 专业图例
    leg = legend('ATE', 'Mean', 'RMSE', 'Location', 'best', 'FontSize', 10);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 子图2: 误差累积分布函数(CDF) - 渐变填充
    subplot(2, 3, 2);
    sorted_errors = sort(pos_errors);
    cdf_values = (1:N) / N;
    hold on;
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    % 深色渐变填充
    x_fill = [sorted_errors; max(sorted_errors); 0];
    y_fill = [cdf_values'; 0; 0];
    fill(x_fill, y_fill, [0.00 0.45 0.74], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    
    plot(sorted_errors, cdf_values, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 3.5);
    hold on;
    % 标注关键百分位点
    percentiles = [50, 75, 95];
    colors = ['r', 'm', 'k'];
    for i = 1:length(percentiles)
        p = percentiles(i);
        idx = round(N * p / 100);
        if idx > 0 && idx <= N
            plot([sorted_errors(idx), sorted_errors(idx)], [0, p/100], [colors(i), '--'], 'LineWidth', 1.5);
            plot([0, sorted_errors(idx)], [p/100, p/100], [colors(i), '--'], 'LineWidth', 1.5);
            text(sorted_errors(idx), p/100, sprintf('  %d%%: %.2fm', p, sorted_errors(idx)), ...
                'FontSize', 8, 'Color', colors(i));
        end
    end
    xlabel('Position Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Cumulative Probability', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('Error Distribution (CDF)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    xlim([0, max(sorted_errors)]);
    ylim([0, 1]);
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 子图3: XYZ轴误差分解 - 高对比三色
    subplot(2, 3, 3);
    hold on;
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    xyz_errors = abs(estimated_trajectory - ground_truth);
    % 高对比配色 - 鲜红/鲜蓝/鲜绿
    plot(1:N, xyz_errors(:,1), '-', 'Color', [0.93 0.11 0.14], 'LineWidth', 3.2, 'DisplayName', 'X-axis');
    plot(1:N, xyz_errors(:,2), '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 3.2, 'DisplayName', 'Y-axis');
    plot(1:N, xyz_errors(:,3), '-', 'Color', [0.47 0.67 0.19], 'LineWidth', 3.2, 'DisplayName', 'Z-axis');  % 鲜绿
    
    xlabel('Frame Index', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Absolute Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('3D Error Decomposition', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    leg = legend('Location', 'best', 'FontSize', 10);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 子图4: 误差箱线图（整体+分段）
    subplot(2, 3, 4);
    has_boxplot = exist('boxplot', 'file') == 2;
    if has_boxplot
        try
            if length(metrics.segment_errors) > 1
                num_segments = length(metrics.segment_errors);
                segment_size = floor(N / num_segments);
                segment_data = nan(segment_size, num_segments);
                for i = 1:num_segments
                    start_idx = (i-1)*segment_size + 1;
                    end_idx = min(i*segment_size, N);
                    seg = pos_errors(start_idx:end_idx);
                    seg = seg(:);
                    L = min(length(seg), segment_size);
                    segment_data(1:L, i) = seg(1:L);
                end
                h = boxplot(segment_data, 'Labels', cellstr(num2str((1:num_segments)' )), 'Symbol', 'o');
            else
                h = boxplot(pos_errors, 'Colors', [0.00 0.45 0.74], 'Symbol', 'o');
            end
            set(h, 'LineWidth', 2.5);
        catch
            has_boxplot = false;
        end
    end
    if ~has_boxplot
        if length(metrics.segment_errors) > 1
            bar(metrics.segment_errors, 'FaceColor', [0.00 0.45 0.74], 'EdgeColor', [0.08 0.35 0.55], 'LineWidth', 1.2);
            set(gca, 'XTick', 1:length(metrics.segment_errors));
            xlabel('Trajectory Segment', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
            ylabel('Segment RMSE (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
            title('Segmented Error Analysis', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
        else
            histogram(pos_errors, 30, 'FaceColor', [0.00 0.45 0.74], 'EdgeColor', [0.08 0.35 0.55]);
            xlabel('Position Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
            ylabel('Count', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
            title('Overall Error Statistics', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
        end
    end
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 子图5: RPE vs 距离段分析
    subplot(2, 3, 5);
    distances = cumsum([0; sqrt(sum(diff(ground_truth).^2, 2))]);
    % 计算分段RPE
    num_dist_segments = min(20, floor(N/10));
    if num_dist_segments > 1
        dist_edges = linspace(0, max(distances), num_dist_segments+1);
        dist_centers = (dist_edges(1:end-1) + dist_edges(2:end)) / 2;
        rpe_by_dist = zeros(1, num_dist_segments);
        for i = 1:num_dist_segments
            idx = distances >= dist_edges(i) & distances < dist_edges(i+1);
            if sum(idx) > 0
                rpe_by_dist(i) = mean(pos_errors(idx));
            end
        end
        % 专业柱状图
        b = bar(dist_centers, rpe_by_dist, 'FaceColor', [0.12 0.47 0.71], 'EdgeColor', [0.08 0.35 0.55], 'LineWidth', 1.2);
        
        grid on;
        set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
        set(gca, 'Box', 'on', 'LineWidth', 1.2);
        set(gca, 'Color', 'w');
        
        xlabel('Distance Traveled (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
        ylabel('Position Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
        title('Distance-based Error Analysis', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
        set(gca, 'FontSize', 10, 'FontName', 'Arial');
        ax = gca;
        ax.XAxis.Color = [0.2 0.2 0.2];
        ax.YAxis.Color = [0.2 0.2 0.2];
        ax.LineWidth = 1.2;
    end
    
    % 子图6: 误差热力图（2D轨迹+误差着色）- 专业热力图
    subplot(2, 3, 6);
    if size(ground_truth, 2) >= 2
        hold on;
        
        grid on;
        set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
        set(gca, 'Box', 'on', 'LineWidth', 1.2);
        set(gca, 'Color', 'w');
        
        % 高对比度热力图
        scatter(ground_truth(:,1), ground_truth(:,2), 50, pos_errors, 'filled', ...
            'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.7);
        
        % 使用彩色热力图渐变（蓝-青-绿-黄-红）
        colormap(subplot(2,3,6), jet);
        
        cb = colorbar;
        cb.Label.String = 'Position Error (m)';
        cb.Label.FontSize = 11;
        cb.Label.FontWeight = 'bold';
        cb.Color = [0 0 0];
        cb.Box = 'on';
        cb.LineWidth = 0.5;
        
        % 高对比度起点/终点标记
        scatter(ground_truth(1,1), ground_truth(1,2), 150, [0.17 0.63 0.17], 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'Start', 'Marker', '^');
        scatter(ground_truth(end,1), ground_truth(end,2), 150, [0.84 0.15 0.16], 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'End', 'Marker', 'v');
        
        xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
        ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
        title('Spatial Error Heatmap', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
        axis equal;
        
        leg = legend('Location', 'best', 'FontSize', 9);
        set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
        
        set(gca, 'FontSize', 10, 'FontName', 'Arial');
        ax = gca;
        ax.XAxis.Color = [0.2 0.2 0.2];
        ax.YAxis.Color = [0.2 0.2 0.2];
        ax.LineWidth = 1.2;
    end
    
    % 根据方法名生成不同的文件名
    if ~isempty(method_name)
        filename = sprintf('slam_accuracy_%s', method_name);
    else
        filename = 'slam_accuracy_evaluation';
    end
    try
        print(gcf, fullfile(save_dir, [filename '.pdf']), '-dpdf', '-bestfit');
    catch
    end
    try
        saveas(gcf, fullfile(save_dir, [filename '.png']));
    catch
    end
    fprintf('精度评估图已保存: %s\n', fullfile(save_dir, filename));
    
    % 9. 生成统计摘要图 - 专业风格
    figure('Name', 'SLAM Statistics Summary', 'Position', [100, 100, 1400, 500], 'Color', [0.96 0.96 0.98]);
    
    % 子图1: 多维度误差对比 - 专业配色
    subplot(1, 3, 1);
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    categories = {'RMSE', 'Mean', 'Median', 'Std', 'Max'};
    values = [metrics.ate.rmse, metrics.ate.mean, metrics.ate.median, metrics.ate.std, metrics.ate.max];
    bar(values, 'FaceColor', [0.12 0.47 0.71], 'EdgeColor', [0.08 0.35 0.55], 'LineWidth', 1.2);
    set(gca, 'XTickLabel', categories);
    
    ylabel('Error (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('ATE Statistical Metrics', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 在每个柱子上标注数值
    for i = 1:length(values)
        text(i, values(i), sprintf('%.2f', values(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0 0]);
    end
    
    % 子图2: 轨迹长度对比 - 专业绿色
    subplot(1, 3, 2);
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    length_data = [metrics.trajectory_length.ground_truth, metrics.trajectory_length.estimated];
    bar_h = bar(length_data, 'FaceColor', [0.17 0.63 0.17], 'EdgeColor', [0.12 0.45 0.12], 'LineWidth', 1.2);
    set(gca, 'XTickLabel', {'Ground Truth', 'Estimated'});
    
    ylabel('Path Length (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title(sprintf('Path Length (Error: %.2f%%)', metrics.trajectory_length.error_ratio*100), ...
        'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    set(gca, 'FontSize', 10, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 标注数值
    for i = 1:length(length_data)
        text(i, length_data(i), sprintf('%.2fm', length_data(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0 0]);
    end
    
    % 子图3: 性能雷达图（归一化）
    subplot(1, 3, 3);
    % 计算归一化性能指标（越低越好，转换为越高越好）
    max_error = max(pos_errors);
    if max_error > 0
        accuracy_score = 1 - (metrics.ate.rmse / max_error);  % RMSE性能
        precision_score = 1 - (metrics.ate.std / max_error);  % 精度性能
        consistency_score = 1 - (metrics.ate.max - metrics.ate.min) / max_error;  % 一致性
        length_score = 1 - abs(metrics.trajectory_length.error_ratio);  % 长度准确度
        drift_score = 1 - min(metrics.drift_rate, 1);  % 漂移控制
        
        scores = [accuracy_score, precision_score, consistency_score, length_score, drift_score];
        score_labels = {'Accuracy', 'Precision', 'Consistency', 'Length', 'Drift Control'};
        
        grid on;
        set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
        set(gca, 'Box', 'on', 'LineWidth', 1.2);
        set(gca, 'Color', 'w');
        
        bar(scores * 100, 'FaceColor', [0.90 0.40 0.00], 'EdgeColor', [0.65 0.30 0.00], 'LineWidth', 1.2);
        set(gca, 'XTickLabel', score_labels);
        xtickangle(45);
        
        ylabel('Performance (%)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 0]);
        title('Performance Scores', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
        ylim([0, 100]);
        set(gca, 'FontSize', 10, 'FontName', 'Arial');
        ax = gca;
        ax.XAxis.Color = [0.2 0.2 0.2];
        ax.YAxis.Color = [0.2 0.2 0.2];
        ax.LineWidth = 1.2;
        
        % 标注数值
        for i = 1:length(scores)
            text(i, scores(i)*100, sprintf('%.1f%%', scores(i)*100), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', [0 0 0]);
        end
    end
    
    % 保存统计摘要图
    if ~isempty(method_name)
        stats_filename = sprintf('slam_statistics_%s', method_name);
    else
        stats_filename = 'slam_statistics_summary';
    end
    try
        print(gcf, fullfile(save_dir, [stats_filename '.pdf']), '-dpdf', '-bestfit');
    catch
    end
    try
        saveas(gcf, fullfile(save_dir, [stats_filename '.png']));
    catch
    end
    fprintf('统计摘要图已保存: %s\n', fullfile(save_dir, stats_filename));
end
