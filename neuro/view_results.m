%% View Results - Quick visualization from saved MAT file or workspace
% This script can work with either saved MAT file or current workspace variables

close all;

fprintf('========== View Comparison Results ==========\n');

% Data path
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
result_path = fullfile(data_path, 'comparison_results');
mat_file = fullfile(result_path, 'comparison_results.mat');

% Try to load from file first
if exist(mat_file, 'file')
    fprintf('Loading from file: %s\n', mat_file);
    load(mat_file);
    from_file = true;
else
    fprintf('No saved file, using workspace variables\n');
    from_file = false;
    % Check if variables exist in workspace
    if ~exist('baseline_exp_aligned', 'var')
        error('No data found. Please run experiment or complete_visualization first');
    end
end

fprintf('OK Data loaded\n');
fprintf('  Dataset: %s\n', dataset_name);
fprintf('  Frames: %d\n', num_frames);
fprintf('  Baseline: VT=%d, Exp=%d\n', NUM_VT_BL, NUM_EXPS_BL);
fprintf('  Ours: VT=%d, Exp=%d\n', NUM_VT_OURS, NUM_EXPS_OURS);

%% Calculate trajectory lengths
fprintf('\n========== Trajectory Length Statistics ==========\n');

% Ground Truth length
if exist('gt_pos_aligned', 'var') && ~isempty(gt_pos_aligned)
    gt_len = sum(sqrt(sum(diff(gt_pos_aligned).^2, 2)));
    fprintf('Ground Truth: %.1f m\n', gt_len);
else
    gt_len = NaN;
    fprintf('Ground Truth: no data\n');
end

% Baseline trajectory length
bl_exp_len = sum(sqrt(sum(diff(baseline_exp_aligned).^2, 2)));
bl_odo_len = sum(sqrt(sum(diff(baseline_odo_aligned).^2, 2)));
fprintf('Baseline Exp Map: %.1f m (%.1f%% of GT)\n', bl_exp_len, bl_exp_len/gt_len*100);
fprintf('Baseline Odometry: %.1f m (%.1f%% of GT)\n', bl_odo_len, bl_odo_len/gt_len*100);

% Ours trajectory length
ours_exp_len = sum(sqrt(sum(diff(ours_exp_aligned).^2, 2)));
ours_odo_len = sum(sqrt(sum(diff(ours_odo_aligned).^2, 2)));
fprintf('Ours Exp Map: %.1f m (%.1f%% of GT)\n', ours_exp_len, ours_exp_len/gt_len*100);
fprintf('Ours Odometry: %.1f m (%.1f%% of GT)\n', ours_odo_len, ours_odo_len/gt_len*100);

% Endpoint error
if exist('gt_pos_aligned', 'var') && ~isempty(gt_pos_aligned)
    bl_exp_endpoint_err = norm(baseline_exp_aligned(end,:) - gt_pos_aligned(end,:));
    ours_exp_endpoint_err = norm(ours_exp_aligned(end,:) - gt_pos_aligned(end,:));
    fprintf('\nEndpoint Error:\n');
    fprintf('  Baseline: %.1f m\n', bl_exp_endpoint_err);
    fprintf('  Ours: %.1f m (improvement %.1f%%)\n', ours_exp_endpoint_err, ...
        (bl_exp_endpoint_err - ours_exp_endpoint_err)/bl_exp_endpoint_err*100);
end

%% Accuracy statistics
if exist('bl_exp_ate', 'var')
    fprintf('\n========== Accuracy Comparison ==========\n');
    fprintf('Metric            Baseline        Ours          Improvement\n');
    fprintf('--------------------------------------------------------\n');
    fprintf('Exp Map ATE       %8.3f m    %8.3f m    %+.1f%%\n', bl_exp_ate, ours_exp_ate, ate_improvement);
    fprintf('Exp Map RMSE      %8.3f m    %8.3f m    %+.1f%%\n', bl_exp_rmse, ours_exp_rmse, rmse_improvement);
    fprintf('Exp Map Max       %8.3f m    %8.3f m\n', bl_exp_max, ours_exp_max);
    fprintf('Odometry ATE      %8.3f m    %8.3f m\n', bl_odo_ate, ours_odo_ate);
    fprintf('Odometry RMSE     %8.3f m    %8.3f m\n', bl_odo_rmse, ours_odo_rmse);
end

%% Visualization
fprintf('\n========== Generating Visualization ==========\n');

% Figure 1: Trajectory comparison
figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
hold on;
if exist('gt_pos_aligned', 'var') && ~isempty(gt_pos_aligned)
    plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
end
plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.8, 'DisplayName', ...
    sprintf('Baseline (%.1fm)', bl_exp_len));
plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.8, 'DisplayName', ...
    sprintf('Ours (%.1fm)', ours_exp_len));
plot(baseline_exp_aligned(1,1), baseline_exp_aligned(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(baseline_exp_aligned(end,1), baseline_exp_aligned(end,2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
hold off;
xlabel('X (m)', 'FontSize', 13);
ylabel('Y (m)', 'FontSize', 13);
title('Experience Map Trajectory Comparison', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
axis equal;

subplot(1, 2, 2);
hold on;
if exist('gt_pos_aligned', 'var') && ~isempty(gt_pos_aligned)
    plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
end
plot(baseline_odo_aligned(:,1), baseline_odo_aligned(:,2), 'b--', 'LineWidth', 1.8, 'DisplayName', ...
    sprintf('Baseline VO (%.1fm)', bl_odo_len));
plot(ours_odo_aligned(:,1), ours_odo_aligned(:,2), 'r-', 'LineWidth', 1.8, 'DisplayName', ...
    sprintf('Ours IMU+VO (%.1fm)', ours_odo_len));
hold off;
xlabel('X (m)', 'FontSize', 13);
ylabel('Y (m)', 'FontSize', 13);
title('Odometry Trajectory Comparison', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
axis equal;

sgtitle(sprintf('%s: Trajectory Comparison (GT: %.1fm)', dataset_name, gt_len), 'FontSize', 17, 'FontWeight', 'bold');

% Save if result_path exists
if exist('result_path', 'var') && exist(result_path, 'dir')
    saveas(gcf, fullfile(result_path, 'trajectory_comparison.png'));
    fprintf('OK Saved: trajectory_comparison.png\n');
end

% Figure 2: Error analysis
if exist('bl_exp_ate', 'var')
    % Recalculate errors
    bl_exp_error = sqrt(sum((baseline_exp_aligned - gt_pos_aligned).^2, 2));
    ours_exp_error = sqrt(sum((ours_exp_aligned - gt_pos_aligned).^2, 2));
    
    figure('Name', 'Error Analysis', 'Position', [150, 150, 1400, 600]);
    
    subplot(1, 2, 1);
    plot(bl_exp_error, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Baseline (ATE=%.2fm)', bl_exp_ate));
    hold on;
    plot(ours_exp_error, 'r-', 'LineWidth', 1.5, 'DisplayName', sprintf('Ours (ATE=%.2fm)', ours_exp_ate));
    hold off;
    xlabel('Frame', 'FontSize', 13);
    ylabel('Position Error (m)', 'FontSize', 13);
    title('Position Error Over Time', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    
    subplot(1, 2, 2);
    histogram(bl_exp_error, 40, 'FaceColor', 'b', 'FaceAlpha', 0.6, 'DisplayName', 'Baseline');
    hold on;
    histogram(ours_exp_error, 40, 'FaceColor', 'r', 'FaceAlpha', 0.6, 'DisplayName', 'Ours');
    hold off;
    xlabel('Position Error (m)', 'FontSize', 13);
    ylabel('Frequency', 'FontSize', 13);
    title('Error Distribution', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    
    sgtitle(sprintf('Error Analysis (Ours improvement: %.1f%%)', ate_improvement), 'FontSize', 17, 'FontWeight', 'bold');
    
    if exist('result_path', 'var') && exist(result_path, 'dir')
        saveas(gcf, fullfile(result_path, 'error_analysis.png'));
        fprintf('OK Saved: error_analysis.png\n');
    end
end

% Figure 3: Statistics comparison
figure('Name', 'Statistics Comparison', 'Position', [200, 200, 1200, 500]);

subplot(1, 2, 1);
categories = {'VT Count', 'Exp Nodes'};
baseline_counts = [NUM_VT_BL, NUM_EXPS_BL];
ours_counts = [NUM_VT_OURS, NUM_EXPS_OURS];
x = 1:2;
width = 0.35;
bar(x - width/2, baseline_counts, width, 'FaceColor', [0.3 0.5 0.8], 'DisplayName', 'Baseline');
hold on;
bar(x + width/2, ours_counts, width, 'FaceColor', [0.8 0.3 0.3], 'DisplayName', 'Ours');
hold off;
set(gca, 'XTick', x, 'XTickLabel', categories, 'FontSize', 12);
ylabel('Count', 'FontSize', 13);
title('Map Statistics Comparison', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;

if exist('bl_exp_ate', 'var')
    subplot(1, 2, 2);
    metrics_names = {'ATE (m)', 'RMSE (m)'};
    baseline_metrics = [bl_exp_ate, bl_exp_rmse];
    ours_metrics = [ours_exp_ate, ours_exp_rmse];
    x = 1:2;
    width = 0.35;
    bar(x - width/2, baseline_metrics, width, 'FaceColor', [0.3 0.5 0.8], 'DisplayName', 'Baseline');
    hold on;
    bar(x + width/2, ours_metrics, width, 'FaceColor', [0.8 0.3 0.3], 'DisplayName', 'Ours');
    hold off;
    set(gca, 'XTick', x, 'XTickLabel', metrics_names, 'FontSize', 12);
    ylabel('Error (m)', 'FontSize', 13);
    title('Accuracy Metrics Comparison', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
end

sgtitle('Statistics Comparison', 'FontSize', 17, 'FontWeight', 'bold');

if exist('result_path', 'var') && exist(result_path, 'dir')
    saveas(gcf, fullfile(result_path, 'statistics_comparison.png'));
    fprintf('OK Saved: statistics_comparison.png\n');
end

fprintf('OK Visualization completed\n');

%% Summary
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    Result Summary                            ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Trajectory Scale:                                           ║\n');
fprintf('║    Ground Truth: %.1f m                                  \n', gt_len);
fprintf('║    Baseline: %.1f m (%.1f%% of GT)                       \n', bl_exp_len, bl_exp_len/gt_len*100);
fprintf('║    Ours: %.1f m (%.1f%% of GT)                           \n', ours_exp_len, ours_exp_len/gt_len*100);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
if exist('bl_exp_ate', 'var')
fprintf('║  Accuracy Improvement:                                       ║\n');
fprintf('║    ATE: %.1f%% (%.3f → %.3f m)                          \n', ate_improvement, bl_exp_ate, ours_exp_ate);
fprintf('║    RMSE: %.1f%% (%.3f → %.3f m)                         \n', rmse_improvement, bl_exp_rmse, ours_exp_rmse);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Endpoint Error:                                             ║\n');
fprintf('║    Baseline: %.1f m                                      \n', bl_exp_endpoint_err);
fprintf('║    Ours: %.1f m (improvement %.1f%%)                     \n', ours_exp_endpoint_err, ...
    (bl_exp_endpoint_err - ours_exp_endpoint_err)/bl_exp_endpoint_err*100);
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

if exist('result_path', 'var') && exist(result_path, 'dir')
    fprintf('\nTip: All figures saved in %s\n', result_path);
end
