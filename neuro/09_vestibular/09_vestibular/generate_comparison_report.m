function generate_comparison_report(result_path, dataset_name)
% GENERATE_COMPARISON_REPORT 生成专业的多方法对比报告图
%
% 创建包含以下内容的综合对比图：
%   1. 雷达图 - 多维性能对比
%   2. 误差随时间演化曲线
%   3. 累积误差分布对比
%   4. 性能指标热力图
%   5. 轨迹质量评分仪表盘
%
% 参数:
%   result_path - 结果保存路径
%   dataset_name - 数据集名称

    if nargin < 2
        dataset_name = 'Town01';
    end

    %% 加载数据
    fprintf('正在生成综合对比报告...\n');
    
    % 尝试加载轨迹数据（优先EuRoC专用文件）
    euroc_traj_file = fullfile(result_path, 'euroc_trajectories.mat');
    traj_file = fullfile(result_path, 'trajectories.mat');
    
    if exist(euroc_traj_file, 'file')
        data = load(euroc_traj_file);
        fprintf('  加载EuRoC轨迹文件: %s\n', euroc_traj_file);
    elseif exist(traj_file, 'file')
        data = load(traj_file);
        fprintf('  加载标准轨迹文件: %s\n', traj_file);
    else
        error('未找到轨迹数据文件: %s', traj_file);
    end
    
    % 提取对齐后的轨迹（优先使用对齐后的版本）
    if isfield(data, 'gt_pos_aligned')
        gt_traj = data.gt_pos_aligned;
    elseif isfield(data, 'gt_data') && isfield(data.gt_data, 'pos')
        gt_traj = data.gt_data.pos;
    else
        error('未找到Ground Truth数据');
    end
    
    if isfield(data, 'exp_traj_aligned')
        bio_traj = data.exp_traj_aligned;  % Bio-inspired系统（对齐后）
    elseif isfield(data, 'exp_trajectory')
        bio_traj = data.exp_trajectory;
    else
        error('未找到经验地图轨迹数据');
    end
    
    if isfield(data, 'fusion_pos_aligned')
        ekf_traj = data.fusion_pos_aligned;  % EKF前端（对齐后）
    elseif isfield(data, 'fusion_data') && isfield(data.fusion_data, 'pos')
        ekf_traj = data.fusion_data.pos;
    else
        error('未找到融合轨迹数据');
    end
    
    if isfield(data, 'odo_traj_aligned')
        vo_traj = data.odo_traj_aligned;  % 视觉里程计（对齐后）
    elseif isfield(data, 'odo_trajectory')
        vo_traj = data.odo_trajectory;
    else
        error('未找到视觉里程计轨迹数据');
    end
    
    fprintf('  已加载轨迹数据: GT=%d帧, Bio=%d帧, EKF=%d帧, VO=%d帧\n', ...
        size(gt_traj, 1), size(bio_traj, 1), size(ekf_traj, 1), size(vo_traj, 1));
    
    %% 计算各方法的性能指标
    methods = {'Bio-inspired Fusion', 'EKF Fusion', 'Visual Odometry'};
    trajectories = {bio_traj, ekf_traj, vo_traj};
    colors = {[0.84 0.15 0.16], [0.12 0.47 0.71], [0.17 0.63 0.17]};
    
    n_methods = length(methods);
    metrics = struct();
    
    for i = 1:n_methods
        traj = trajectories{i};
        
        % 确保长度一致
        min_len = min(size(traj, 1), size(gt_traj, 1));
        traj = traj(1:min_len, :);
        gt = gt_traj(1:min_len, :);
        
        % 计算误差
        errors = sqrt(sum((traj - gt).^2, 2));
        
        metrics(i).name = methods{i};
        metrics(i).rmse = sqrt(mean(errors.^2));
        metrics(i).mean_error = mean(errors);
        metrics(i).max_error = max(errors);
        metrics(i).std_error = std(errors);
        metrics(i).errors = errors;
        
        % 计算轨迹长度
        traj_len = sum(sqrt(sum(diff(gt).^2, 2)));
        
        % 终点误差和漂移率
        metrics(i).end_error = errors(end);
        metrics(i).drift_rate = (errors(end) / traj_len) * 100;
        
        % 相对位姿误差 (RPE)
        rpe = sqrt(sum(diff(traj - gt).^2, 2));
        metrics(i).rpe = sqrt(mean(rpe.^2));
        
        % 轨迹平滑度 (曲率变化)
        if size(traj, 1) > 2
            d1 = diff(traj);
            d2 = diff(d1);
            metrics(i).smoothness = mean(sqrt(sum(d2.^2, 2)));
        else
            metrics(i).smoothness = 0;
        end
    end
    
    %% 创建综合对比图
    fig = figure('Position', [50, 50, 1600, 1000], 'Color', 'w');
    
    %% 子图1: 雷达图 - 多维性能对比
    subplot(2, 3, 1);
    
    % 准备雷达图数据 (归一化到0-1，越大越好)
    radar_labels = {'Accuracy', 'Drift Control', 'Consistency'};
    n_dims = length(radar_labels);
    
    % 使用更公平的核心指标构建评分（越小越好 -> 越大越好）
    rmse_vals = [metrics.rmse];
    drift_vals = [metrics.drift_rate];
    std_vals = [metrics.std_error];
    
    max_rmse = max(rmse_vals);
    min_rmse = min(rmse_vals);
    max_drift = max(drift_vals);
    min_drift = min(drift_vals);
    max_std = max(std_vals);
    min_std = min(std_vals);
    
    radar_data = zeros(n_methods, n_dims);
    for i = 1:n_methods
        radar_data(i, 1) = (max_rmse - metrics(i).rmse) / (max_rmse - min_rmse + eps);          % Accuracy (RMSE)
        radar_data(i, 2) = (max_drift - metrics(i).drift_rate) / (max_drift - min_drift + eps); % Drift Control
        radar_data(i, 3) = (max_std - metrics(i).std_error) / (max_std - min_std + eps);        % Consistency
    end
    
    % 绘制雷达图
    draw_radar_chart(radar_data, radar_labels, methods, colors);
    title('Multi-dimensional Performance Comparison', 'FontSize', 12, 'FontWeight', 'bold');
    
    %% 子图2: 误差随时间演化 (带置信区间)
    subplot(2, 3, 2);
    hold on;
    grid on;
    set(gca, 'GridAlpha', 0.3);
    
    for i = 1:n_methods
        errors = metrics(i).errors;
        x = 1:length(errors);
        
        % 计算滑动窗口统计
        window = 50;
        smooth_err = movmean(errors, window);
        smooth_std = movstd(errors, window);
        
        % 绘制置信区间（不加入图例，避免出现data1/data2/data3）
        fill([x, fliplr(x)], [smooth_err' + smooth_std', fliplr(smooth_err' - smooth_std')], ...
            colors{i}, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % 绘制主曲线
        plot(x, smooth_err, '-', 'Color', colors{i}, 'LineWidth', 2.5, ...
            'DisplayName', methods{i});
    end
    
    xlabel('Frame Index', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Position Error (m)', 'FontSize', 11, 'FontWeight', 'bold');
    title('Error Evolution with Confidence Interval', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 9);
    set(gca, 'FontSize', 10);
    
    %% 子图3: 累积误差分布 (小提琴图风格)
    subplot(2, 3, 3);
    hold on;
    
    positions = 1:n_methods;
    width = 0.35;
    
    for i = 1:n_methods
        errors = metrics(i).errors;
        
        [f, xi] = approx_density(errors, 100);
        f = f / max(f) * width;  % 归一化宽度
        
        % 绘制小提琴形状
        fill([positions(i) - f, fliplr(positions(i) + f)], [xi, fliplr(xi)], ...
            colors{i}, 'FaceAlpha', 0.6, 'EdgeColor', colors{i}, 'LineWidth', 1.5);
        
        % 绘制中位数和四分位数
        q1 = prctile(errors, 25);
        q2 = prctile(errors, 50);
        q3 = prctile(errors, 75);
        
        plot([positions(i)-0.1, positions(i)+0.1], [q2, q2], '-', ...
            'Color', 'w', 'LineWidth', 3);
        plot([positions(i), positions(i)], [q1, q3], '-', ...
            'Color', 'w', 'LineWidth', 2);
    end
    
    set(gca, 'XTick', positions, 'XTickLabel', {'Bio-inspired', 'EKF', 'VO'});
    ylabel('Position Error (m)', 'FontSize', 11, 'FontWeight', 'bold');
    title('Error Distribution (Violin Plot)', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    set(gca, 'GridAlpha', 0.3, 'FontSize', 10);
    xlim([0.3, n_methods + 0.7]);
    
    %% 子图4: 性能指标热力图
    subplot(2, 3, 4);
    
    % 准备热力图数据
    metric_names = {'RMSE (m)', 'Mean Error (m)', 'Max Error (m)', 'Drift Rate (%)', 'RPE (m)'};
    heatmap_data = zeros(n_methods, length(metric_names));
    
    for i = 1:n_methods
        heatmap_data(i, 1) = metrics(i).rmse;
        heatmap_data(i, 2) = metrics(i).mean_error;
        heatmap_data(i, 3) = metrics(i).max_error;
        heatmap_data(i, 4) = metrics(i).drift_rate;
        heatmap_data(i, 5) = metrics(i).rpe;
    end
    
    % 归一化用于颜色映射 (每列独立归一化)
    heatmap_norm = zeros(size(heatmap_data));
    for j = 1:size(heatmap_data, 2)
        col = heatmap_data(:, j);
        heatmap_norm(:, j) = (col - min(col)) / (max(col) - min(col) + eps);
    end
    
    % 绘制热力图
    imagesc(heatmap_norm);
    colormap(gca, flipud(autumn));  % 反转颜色：绿色=好，红色=差
    
    % 添加数值标签
    for i = 1:n_methods
        for j = 1:length(metric_names)
            if j == 4  % 漂移率用百分比
                text(j, i, sprintf('%.1f%%', heatmap_data(i, j)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
            else
                text(j, i, sprintf('%.1f', heatmap_data(i, j)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
            end
        end
    end
    
    set(gca, 'XTick', 1:length(metric_names), 'XTickLabel', metric_names, ...
        'YTick', 1:n_methods, 'YTickLabel', {'Bio-inspired', 'EKF', 'VO'});
    xtickangle(30);
    title('Performance Metrics Heatmap', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'FontSize', 10);
    
    % 添加颜色条
    cb = colorbar;
    cb.Label.String = 'Relative Performance (lower is better)';
    cb.Label.FontSize = 9;
    
    %% 子图5: 综合评分仪表盘
    subplot(2, 3, 5);
    
    % 计算综合评分 (0-100)
    scores = zeros(1, n_methods);
    for i = 1:n_methods
        % 加权平均各项指标的归一化分数
        scores(i) = mean(radar_data(i, :)) * 100;
    end
    
    % 绘制仪表盘风格的评分图
    theta = linspace(0, pi, 100);
    
    for i = 1:n_methods
        % 背景弧
        r_bg = 0.8 + (i-1) * 0.3;
        x_bg = r_bg * cos(theta);
        y_bg = r_bg * sin(theta);
        plot(x_bg, y_bg, '-', 'Color', [0.9 0.9 0.9], 'LineWidth', 20);
        hold on;
        
        % 评分弧
        score_angle = pi * scores(i) / 100;
        theta_score = linspace(0, score_angle, 50);
        x_score = r_bg * cos(theta_score);
        y_score = r_bg * sin(theta_score);
        plot(x_score, y_score, '-', 'Color', colors{i}, 'LineWidth', 20);
        
        % 添加分数标签
        text(r_bg * cos(pi/2), r_bg * sin(pi/2) + 0.15, ...
            sprintf('%.0f', scores(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', ...
            'Color', colors{i});
    end
    
    % 添加方法标签
    text(0.8, -0.15, 'Bio-inspired', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'Color', colors{1}, 'FontWeight', 'bold');
    text(1.1, -0.15, 'EKF', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'Color', colors{2}, 'FontWeight', 'bold');
    text(1.4, -0.15, 'VO', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'Color', colors{3}, 'FontWeight', 'bold');
    
    axis equal;
    axis off;
    title('Overall Performance Score', 'FontSize', 12, 'FontWeight', 'bold');
    
    %% 子图6: 关键指标对比条形图 (带误差条)
    subplot(2, 3, 6);
    
    % 准备数据
    bar_data = [[metrics.rmse]; [metrics.drift_rate]; [metrics.rpe]]';
    bar_labels = {'RMSE (m)', 'Drift Rate (%)', 'RPE (m)'};
    
    % 创建分组条形图
    x = 1:length(bar_labels);
    bar_width = 0.25;
    
    hold on;
    for i = 1:n_methods
        bar_x = x + (i - 2) * bar_width;
        b = bar(bar_x, bar_data(i, :), bar_width, 'FaceColor', colors{i}, ...
            'EdgeColor', 'none', 'FaceAlpha', 0.8);
        
        % 添加数值标签
        for j = 1:length(bar_labels)
            if j == 2  % 漂移率
                text(bar_x(j), bar_data(i, j) + max(bar_data(:, j)) * 0.05, ...
                    sprintf('%.1f%%', bar_data(i, j)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
            else
                text(bar_x(j), bar_data(i, j) + max(bar_data(:, j)) * 0.05, ...
                    sprintf('%.1f', bar_data(i, j)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
            end
        end
    end
    
    set(gca, 'XTick', x, 'XTickLabel', bar_labels);
    ylabel('Value', 'FontSize', 11, 'FontWeight', 'bold');
    title('Key Metrics Comparison', 'FontSize', 12, 'FontWeight', 'bold');
    legend(methods, 'Location', 'northeast', 'FontSize', 9);
    grid on;
    set(gca, 'GridAlpha', 0.3, 'FontSize', 10);
    
    %% 添加总标题
    sgtitle(sprintf('Comprehensive Performance Comparison - %s Dataset', dataset_name), ...
        'FontSize', 16, 'FontWeight', 'bold', 'Interpreter', 'none');
    
    %% 保存图形（高分辨率）
    try
        save_file = fullfile(result_path, 'comprehensive_comparison.pdf');
        print(fig, save_file, '-dpdf', '-bestfit');
        fprintf('综合对比报告已保存: %s\n', save_file);
    catch
    end
    try
        save_file = fullfile(result_path, 'comprehensive_comparison.png');
        print(fig, save_file, '-dpng', '-r300');
        fprintf('综合对比报告已保存: %s\n', save_file);
    catch
    end
    
    %% 生成文本报告
    generate_text_report(metrics, result_path, dataset_name);
end

%% 辅助函数: 绘制雷达图
function draw_radar_chart(data, labels, legend_names, colors)
    n_dims = size(data, 2);
    n_methods = size(data, 1);
    
    % 计算角度
    angles = linspace(0, 2*pi, n_dims + 1);
    angles = angles(1:end-1);
    
    % 绘制背景网格
    hold on;
    for r = 0.2:0.2:1.0
        x = r * cos(angles);
        y = r * sin(angles);
        plot([x, x(1)], [y, y(1)], '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
    end
    
    % 绘制轴线
    for i = 1:n_dims
        plot([0, cos(angles(i))], [0, sin(angles(i))], '-', ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    end
    
    % 绘制数据
    for m = 1:n_methods
        values = data(m, :);
        x = values .* cos(angles);
        y = values .* sin(angles);
        
        % 填充区域
        fill([x, x(1)], [y, y(1)], colors{m}, 'FaceAlpha', 0.25, ...
            'EdgeColor', colors{m}, 'LineWidth', 2);
        
        % 绘制数据点
        scatter(x, y, 60, colors{m}, 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1);
    end
    
    % 添加标签
    label_radius = 1.15;
    for i = 1:n_dims
        x = label_radius * cos(angles(i));
        y = label_radius * sin(angles(i));
        text(x, y, labels{i}, 'HorizontalAlignment', 'center', ...
            'FontSize', 10, 'FontWeight', 'bold');
    end
    
    axis equal;
    axis off;
    xlim([-1.4, 1.4]);
    ylim([-1.2, 1.4]);
    
    % 添加图例
    for m = 1:n_methods
        text(-1.3, 1.2 - (m-1)*0.15, legend_names{m}, ...
            'Color', colors{m}, 'FontSize', 9, 'FontWeight', 'bold');
    end
end

%% 辅助函数: 生成文本报告
function generate_text_report(metrics, result_path, dataset_name)
    report_file = fullfile(result_path, 'comparison_report.txt');
    fid = fopen(report_file, 'w');
    
    fprintf(fid, '================================================================\n');
    fprintf(fid, '       COMPREHENSIVE SLAM PERFORMANCE COMPARISON REPORT\n');
    fprintf(fid, '================================================================\n\n');
    fprintf(fid, 'Dataset: %s\n', dataset_name);
    fprintf(fid, 'Generated: %s\n\n', datestr(now));
    
    fprintf(fid, '----------------------------------------------------------------\n');
    fprintf(fid, '                    PERFORMANCE SUMMARY\n');
    fprintf(fid, '----------------------------------------------------------------\n\n');
    
    % 找出最佳方法
    [~, best_rmse] = min([metrics.rmse]);
    [~, best_drift] = min([metrics.drift_rate]);
    
    fprintf(fid, '🏆 Best RMSE:       %s (%.2f m)\n', metrics(best_rmse).name, metrics(best_rmse).rmse);
    fprintf(fid, '🏆 Best Drift Rate: %s (%.2f%%)\n\n', metrics(best_drift).name, metrics(best_drift).drift_rate);
    
    fprintf(fid, '----------------------------------------------------------------\n');
    fprintf(fid, '                    DETAILED METRICS\n');
    fprintf(fid, '----------------------------------------------------------------\n\n');
    
    for i = 1:length(metrics)
        fprintf(fid, '【%s】\n', metrics(i).name);
        fprintf(fid, '  RMSE:        %.2f m\n', metrics(i).rmse);
        fprintf(fid, '  Mean Error:  %.2f m\n', metrics(i).mean_error);
        fprintf(fid, '  Max Error:   %.2f m\n', metrics(i).max_error);
        fprintf(fid, '  Std Error:   %.2f m\n', metrics(i).std_error);
        fprintf(fid, '  End Error:   %.2f m\n', metrics(i).end_error);
        fprintf(fid, '  Drift Rate:  %.2f%%\n', metrics(i).drift_rate);
        fprintf(fid, '  RPE:         %.4f m\n\n', metrics(i).rpe);
    end
    
    fprintf(fid, '----------------------------------------------------------------\n');
    fprintf(fid, '                    IMPROVEMENT ANALYSIS\n');
    fprintf(fid, '----------------------------------------------------------------\n\n');
    
    % 计算Bio-inspired相对于其他方法的提升
    bio_rmse = metrics(1).rmse;
    bio_drift = metrics(1).drift_rate;
    
    for i = 2:length(metrics)
        rmse_improve = (metrics(i).rmse - bio_rmse) / metrics(i).rmse * 100;
        drift_improve = (metrics(i).drift_rate - bio_drift) / metrics(i).drift_rate * 100;
        
        fprintf(fid, 'Bio-inspired vs %s:\n', metrics(i).name);
        fprintf(fid, '  RMSE Improvement:  %.1f%%\n', rmse_improve);
        fprintf(fid, '  Drift Improvement: %.1f%%\n\n', drift_improve);
    end
    
    fprintf(fid, '================================================================\n');
    fprintf(fid, '                         END OF REPORT\n');
    fprintf(fid, '================================================================\n');
    
    fclose(fid);
    fprintf('文本报告已保存: %s\n', report_file);
end

function [f, xi] = approx_density(values, numPoints)
    values = values(:);
    values = values(isfinite(values));
    if isempty(values)
        xi = linspace(0, 1, numPoints);
        f = zeros(size(xi));
        return;
    end
    vmin = min(values);
    vmax = max(values);
    if vmin == vmax
        xi = linspace(vmin - 1, vmax + 1, numPoints);
        f = zeros(size(xi));
        f(round(numPoints/2)) = 1;
        return;
    end
    edges = linspace(vmin, vmax, numPoints + 1);
    counts = histcounts(values, edges, 'Normalization', 'pdf');
    xi = (edges(1:end-1) + edges(2:end)) / 2;
    smooth_win = max(3, 2 * floor(numPoints/25) + 1);
    f = movmean(counts, smooth_win);
end
