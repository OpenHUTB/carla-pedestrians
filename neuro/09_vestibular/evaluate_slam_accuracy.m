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
    
    % 8. 绘制增强的误差分析图（2x3布局）
    figure('Name', 'SLAM Accuracy Evaluation', 'Position', [50, 50, 1600, 900]);
    
    % 子图1: ATE随时间变化
    subplot(2, 3, 1);
    plot(1:N, pos_errors, 'b-', 'LineWidth', 1.5);
    hold on;
    plot([1, N], [metrics.ate.mean, metrics.ate.mean], 'r--', 'LineWidth', 2);
    plot([1, N], [metrics.ate.rmse, metrics.ate.rmse], 'g--', 'LineWidth', 1.5);
    xlabel('Frame', 'FontSize', 10);
    ylabel('Position Error (m)', 'FontSize', 10);
    title('Absolute Trajectory Error (ATE)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('ATE', 'Mean', 'RMSE', 'Location', 'best', 'FontSize', 9);
    grid on;
    
    % 子图2: 误差累积分布函数(CDF)
    subplot(2, 3, 2);
    sorted_errors = sort(pos_errors);
    cdf_values = (1:N) / N;
    plot(sorted_errors, cdf_values, 'b-', 'LineWidth', 2);
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
    xlabel('Position Error (m)', 'FontSize', 10);
    ylabel('Cumulative Probability', 'FontSize', 10);
    title('Error CDF (Cumulative Distribution)', 'FontSize', 11, 'FontWeight', 'bold');
    grid on;
    xlim([0, max(sorted_errors)]);
    ylim([0, 1]);
    
    % 子图3: XYZ轴误差分解
    subplot(2, 3, 3);
    xyz_errors = abs(estimated_trajectory - ground_truth);
    plot(1:N, xyz_errors(:,1), 'r-', 'LineWidth', 1, 'DisplayName', 'X-axis');
    hold on;
    plot(1:N, xyz_errors(:,2), 'g-', 'LineWidth', 1, 'DisplayName', 'Y-axis');
    plot(1:N, xyz_errors(:,3), 'b-', 'LineWidth', 1, 'DisplayName', 'Z-axis');
    xlabel('Frame', 'FontSize', 10);
    ylabel('Axis Error (m)', 'FontSize', 10);
    title('Error Decomposition (X/Y/Z)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    
    % 子图4: 误差箱线图（整体+分段）
    subplot(2, 3, 4);
    if length(metrics.segment_errors) > 1
        % 分段误差箱线图
        segment_size = floor(N / length(metrics.segment_errors));
        segment_data = cell(1, length(metrics.segment_errors));
        for i = 1:length(metrics.segment_errors)
            start_idx = (i-1)*segment_size + 1;
            end_idx = min(i*segment_size, N);
            segment_data{i} = pos_errors(start_idx:end_idx);
        end
        boxplot([segment_data{:}], 'Labels', cellstr(num2str((1:length(metrics.segment_errors))')));
        xlabel('Trajectory Segment', 'FontSize', 10);
        ylabel('Position Error (m)', 'FontSize', 10);
        title('Error Statistics by Segment', 'FontSize', 11, 'FontWeight', 'bold');
        grid on;
    else
        boxplot(pos_errors);
        xlabel('Full Trajectory', 'FontSize', 10);
        ylabel('Position Error (m)', 'FontSize', 10);
        title('Overall Error Statistics', 'FontSize', 11, 'FontWeight', 'bold');
        grid on;
    end
    
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
        bar(dist_centers, rpe_by_dist, 'FaceColor', [0.3, 0.6, 0.9], 'EdgeColor', 'k');
        xlabel('Distance Interval (m)', 'FontSize', 10);
        ylabel('Average Error (m)', 'FontSize', 10);
        title('Error vs. Distance Intervals', 'FontSize', 11, 'FontWeight', 'bold');
        grid on;
    end
    
    % 子图6: 误差热力图（2D轨迹+误差着色）
    subplot(2, 3, 6);
    if size(ground_truth, 2) >= 2
        scatter(ground_truth(:,1), ground_truth(:,2), 30, pos_errors, 'filled');
        colorbar;
        colormap(jet);
        xlabel('X (m)', 'FontSize', 10);
        ylabel('Y (m)', 'FontSize', 10);
        title('Error Heatmap on Trajectory', 'FontSize', 11, 'FontWeight', 'bold');
        axis equal;
        grid on;
        % 添加起点和终点标记
        hold on;
        plot(ground_truth(1,1), ground_truth(1,2), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');
        plot(ground_truth(end,1), ground_truth(end,2), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'End');
        legend('Location', 'best', 'FontSize', 8);
    end
    
    % 根据方法名生成不同的文件名
    if ~isempty(method_name)
        filename = sprintf('slam_accuracy_%s.png', method_name);
    else
        filename = 'slam_accuracy_evaluation.png';
    end
    save_path = fullfile(save_dir, filename);
    saveas(gcf, save_path);
    fprintf('精度评估图已保存: %s\n', save_path);
    
    % 9. 生成统计摘要图
    figure('Name', 'SLAM Statistics Summary', 'Position', [100, 100, 1400, 500]);
    
    % 子图1: 多维度误差对比
    subplot(1, 3, 1);
    categories = {'RMSE', 'Mean', 'Median', 'Std', 'Max'};
    values = [metrics.ate.rmse, metrics.ate.mean, metrics.ate.median, metrics.ate.std, metrics.ate.max];
    bar(values, 'FaceColor', [0.2, 0.6, 0.8]);
    set(gca, 'XTickLabel', categories);
    ylabel('Error (m)', 'FontSize', 11);
    title('ATE Statistics Overview', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    % 在每个柱子上标注数值
    for i = 1:length(values)
        text(i, values(i), sprintf('%.2f', values(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
    
    % 子图2: 轨迹长度对比
    subplot(1, 3, 2);
    length_data = [metrics.trajectory_length.ground_truth, metrics.trajectory_length.estimated];
    bar_h = bar(length_data, 'FaceColor', [0.4, 0.7, 0.4]);
    set(gca, 'XTickLabel', {'Ground Truth', 'Estimated'});
    ylabel('Trajectory Length (m)', 'FontSize', 11);
    title(sprintf('Trajectory Length (Error: %.2f%%)', metrics.trajectory_length.error_ratio*100), ...
        'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    % 标注数值
    for i = 1:length(length_data)
        text(i, length_data(i), sprintf('%.2fm', length_data(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
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
        
        bar(scores * 100, 'FaceColor', [0.8, 0.4, 0.4]);
        set(gca, 'XTickLabel', score_labels);
        xtickangle(45);
        ylabel('Performance Score (%)', 'FontSize', 11);
        title('Performance Metrics', 'FontSize', 12, 'FontWeight', 'bold');
        ylim([0, 100]);
        grid on;
        % 标注数值
        for i = 1:length(scores)
            text(i, scores(i)*100, sprintf('%.1f%%', scores(i)*100), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
        end
    end
    
    % 保存统计摘要图
    if ~isempty(method_name)
        stats_filename = sprintf('slam_statistics_%s.png', method_name);
    else
        stats_filename = 'slam_statistics_summary.png';
    end
    stats_path = fullfile(save_dir, stats_filename);
    saveas(gcf, stats_path);
    fprintf('统计摘要图已保存: %s\n', stats_path);
end
