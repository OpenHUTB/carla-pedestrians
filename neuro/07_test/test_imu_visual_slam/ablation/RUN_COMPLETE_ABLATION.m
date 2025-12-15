%% Complete Ablation Study - Bio-inspired Fusion System
%
% This script runs ablation experiments to evaluate component contributions
% Experiment configurations:
%   1. Bio-inspired Fusion (Ours) - Complete System with Experience Map
%   2. w/o IMU Fusion (Pure Visual Odometry)
%   3. EKF Fusion (Baseline Reference)
%
% Data source: Extracted from existing test results, no need to re-run SLAM
% Corrected: Uses exp_trajectory as Bio-inspired Fusion (NOT fusion_data)

clear all; close all; clc;

fprintf('\n');
fprintf('================================================================\n');
fprintf('   Complete Ablation Study - Component Contribution Analysis   \n');
fprintf('   Bio-inspired Fusion System - Town01 & Town10                \n');
fprintf('================================================================\n');
fprintf('\n');

%% Configuration
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));

% Dataset configuration
datasets = {
    'Town01Data_IMU_Fusion', 'Town01', 1802.26;
    'Town10Data_IMU_Fusion', 'Town10', 1630.84;
};

%% Experiment Definitions
% Extract the following configurations from existing test results:
% CORRECTED: Bio-inspired Fusion uses exp_trajectory, NOT fusion_data
experiments = {
    % Name,                    Description,                                    Data Source
    'Complete',               'Bio-inspired IMU-Visual Fusion (Ours)',        'bio_inspired';
    'No_IMU',                 'w/o IMU Fusion (Pure VO)',                     'visual_odo';
    'EKF_Baseline',           'EKF Fusion (Baseline Reference)',              'ekf_fusion';
};

%% Results Storage
all_results = struct();

%% Process Each Dataset
for d = 1:size(datasets, 1)
    dataset_dir = datasets{d, 1};
    dataset_name = datasets{d, 2};
    gt_length = datasets{d, 3};
    
    fprintf('\n============================================================\n');
    fprintf('Dataset: %s (Trajectory Length: %.1fm)\n', dataset_name, gt_length);
    fprintf('============================================================\n\n');
    
    % Load data
    data_path = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', dataset_dir);
    traj_file = fullfile(data_path, 'slam_results', 'trajectories.mat');
    report_file = fullfile(data_path, 'slam_results', 'performance_report.txt');
    
    if ~exist(traj_file, 'file')
        fprintf('Warning: Data file not found, skipping\n');
        continue;
    end
    
    load(traj_file);
    
    % Extract Ground Truth
    gt_xyz = gt_data.pos(:, 1:3);
    
    % Store results for this dataset
    dataset_results = struct();
    dataset_results.name = dataset_name;
    dataset_results.length = gt_length;
    dataset_results.experiments = struct();
    
    %% Run each experiment configuration
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        exp_desc = experiments{e, 2};
        exp_source = experiments{e, 3};
        
        fprintf('[%d/%d] %s\n', e, size(experiments,1), exp_desc);
        
        try
            % Select trajectory based on data source
            switch exp_source
                case 'bio_inspired'
                    % Bio-inspired Fusion = Experience Map Trajectory
                    exp_xyz = exp_trajectory(:, 1:3);
                    min_len = min(size(gt_xyz,1), size(exp_xyz,1));
                    
                    errors = sqrt(sum((exp_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
                    rmse = sqrt(mean(errors.^2));
                    final_error = errors(end);
                    
                case 'visual_odo'
                    % Pure visual odometry
                    odo_xyz = odo_trajectory(:, 1:3);
                    min_len = min(size(gt_xyz,1), size(odo_xyz,1));
                    
                    errors = sqrt(sum((odo_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
                    rmse = sqrt(mean(errors.^2));
                    final_error = errors(end);
                    
                case 'ekf_fusion'
                    % EKF Fusion baseline
                    fusion_xyz = fusion_data.pos(:, 1:3);
                    min_len = min(size(gt_xyz,1), size(fusion_xyz,1));
                    
                    errors = sqrt(sum((fusion_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
                    rmse = sqrt(mean(errors.^2));
                    final_error = errors(end);
                    
                otherwise
                    rmse = NaN;
                    final_error = NaN;
            end
            
            % Calculate drift rate
            drift_rate = (final_error / gt_length) * 100;
            
            % Save results
            dataset_results.experiments.(exp_name) = struct();
            dataset_results.experiments.(exp_name).description = exp_desc;
            dataset_results.experiments.(exp_name).rmse = rmse;
            dataset_results.experiments.(exp_name).final_error = final_error;
            dataset_results.experiments.(exp_name).drift_rate = drift_rate;
            
            fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
                rmse, final_error, drift_rate);
            
        catch ME
            fprintf('  Failed: %s\n', ME.message);
            dataset_results.experiments.(exp_name) = struct();
            dataset_results.experiments.(exp_name).description = exp_desc;
            dataset_results.experiments.(exp_name).rmse = NaN;
            dataset_results.experiments.(exp_name).final_error = NaN;
            dataset_results.experiments.(exp_name).drift_rate = NaN;
        end
    end
    
    % Save dataset results
    all_results.(dataset_name) = dataset_results;
    
    % Calculate improvement ratio
    fprintf('\nImprovement Analysis:\n');
    baseline_rmse = dataset_results.experiments.Complete.rmse;
    for e = 2:size(experiments, 1)
        exp_name = experiments{e, 1};
        exp_desc = experiments{e, 2};
        exp_rmse = dataset_results.experiments.(exp_name).rmse;
        
        if ~isnan(baseline_rmse) && ~isnan(exp_rmse)
            degradation = ((exp_rmse - baseline_rmse) / baseline_rmse) * 100;
            ratio = exp_rmse / baseline_rmse;
            fprintf('  %s: +%.0f%% (%.1fx degradation)\n', exp_desc, degradation, ratio);
        end
    end
end

%% Generate Summary Report
fprintf('\n\n');
fprintf('================================================================\n');
fprintf('   Ablation Study Complete Results Summary                     \n');
fprintf('================================================================\n');
fprintf('\n');

% Town01 Results
if isfield(all_results, 'Town01')
    fprintf('=== Town01 (1802m trajectory) ===\n\n');
    fprintf('| Configuration | RMSE (m) | Drift Rate (%%) | vs Baseline |\n');
    fprintf('|------|----------|------------|-------------|\n');
    
    town01 = all_results.Town01;
    baseline_rmse = town01.experiments.Complete.rmse;
    
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        exp_data = town01.experiments.(exp_name);
        
        if e == 1
            fprintf('| %s | %.2f | %.2f | Baseline |\n', ...
                exp_data.description, exp_data.rmse, exp_data.drift_rate);
        else
            ratio = exp_data.rmse / baseline_rmse;
            degradation = ((exp_data.rmse - baseline_rmse) / baseline_rmse) * 100;
            fprintf('| %s | %.2f | %.2f | +%.0f%% (%.1f×) |\n', ...
                exp_data.description, exp_data.rmse, exp_data.drift_rate, ...
                degradation, ratio);
        end
    end
    fprintf('\n');
end

% Town10 Results
if isfield(all_results, 'Town10')
    fprintf('=== Town10 (1631m trajectory) ===\n\n');
    fprintf('| Configuration | RMSE (m) | Drift Rate (%%) | vs Baseline |\n');
    fprintf('|------|----------|------------|-------------|\n');
    
    town10 = all_results.Town10;
    baseline_rmse = town10.experiments.Complete.rmse;
    
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        exp_data = town10.experiments.(exp_name);
        
        if e == 1
            fprintf('| %s | %.2f | %.2f | Baseline |\n', ...
                exp_data.description, exp_data.rmse, exp_data.drift_rate);
        else
            ratio = exp_data.rmse / baseline_rmse;
            degradation = ((exp_data.rmse - baseline_rmse) / baseline_rmse) * 100;
            fprintf('| %s | %.2f | %.2f | +%.0f%% (%.1f×) |\n', ...
                exp_data.description, exp_data.rmse, exp_data.drift_rate, ...
                degradation, ratio);
        end
    end
    fprintf('\n');
end

%% Save Results
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'complete_ablation_results.mat'), 'all_results');

% Generate CSV
csvfile = fullfile(results_dir, 'complete_ablation_results.csv');
fid = fopen(csvfile, 'w');
fprintf(fid, 'Dataset,Configuration,RMSE_m,Final_Error_m,Drift_Rate_pct\n');

for d = 1:size(datasets, 1)
    dataset_name = datasets{d, 2};
    if ~isfield(all_results, dataset_name)
        continue;
    end
    
    dataset_res = all_results.(dataset_name);
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        exp_data = dataset_res.experiments.(exp_name);
        
        fprintf(fid, '%s,%s,%.2f,%.2f,%.2f\n', ...
            dataset_name, exp_data.description, ...
            exp_data.rmse, exp_data.final_error, exp_data.drift_rate);
    end
end
fclose(fid);

fprintf('Results saved:\n');
fprintf('   MAT: %s\n', fullfile(results_dir, 'complete_ablation_results.mat'));
fprintf('   CSV: %s\n', csvfile);

%% Generate Visualization Charts
fprintf('\n[Generating visualization charts...]\n');

% Figure 1: RMSE Comparison Bar Chart
fig1 = figure('Position', [100, 100, 1400, 800]);
set(fig1, 'Color', [0.95 0.95 0.97]);

% Prepare data
dataset_names = {};
config_names = {};
rmse_matrix = [];

for d = 1:size(datasets, 1)
    dataset_name = datasets{d, 2};
    if ~isfield(all_results, dataset_name)
        continue;
    end
    
    dataset_names{end+1} = dataset_name;
    dataset_res = all_results.(dataset_name);
    
    rmse_row = [];
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        rmse_row(e) = dataset_res.experiments.(exp_name).rmse;
        if d == 1
            config_names{e} = dataset_res.experiments.(exp_name).description;
        end
    end
    rmse_matrix = [rmse_matrix; rmse_row];
end

% 子图1：RMSE对比（对数刻度）
subplot(2, 2, 1);
bar_handle = bar(rmse_matrix');
set(gca, 'YScale', 'log');
set(gca, 'XTickLabel', config_names);
xtickangle(15);
ylabel('RMSE (m) - Log Scale', 'FontSize', 12, 'FontWeight', 'bold');
title('Ablation Study - RMSE Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend(dataset_names, 'Location', 'best');
grid on;
set(gca, 'GridAlpha', 0.3);

% 添加数值标注
for i = 1:length(bar_handle)
    xData = bar_handle(i).XData + bar_handle(i).XOffset;
    yData = bar_handle(i).YData;
    for j = 1:length(xData)
        if yData(j) > 0
            text(xData(j), yData(j), sprintf('%.1f', yData(j)), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 9, 'FontWeight', 'bold');
        end
    end
end

% 子图2：相对于Baseline的退化倍数
subplot(2, 2, 2);
degradation_matrix = zeros(size(rmse_matrix));
for i = 1:size(rmse_matrix, 1)
    baseline = rmse_matrix(i, 1);
    degradation_matrix(i, :) = rmse_matrix(i, :) / baseline;
end

bar_handle2 = bar(degradation_matrix');
set(gca, 'XTickLabel', config_names);
xtickangle(15);
ylabel('Relative Degradation Ratio', 'FontSize', 12, 'FontWeight', 'bold');
title('Degradation vs Complete System', 'FontSize', 14, 'FontWeight', 'bold');
legend(dataset_names, 'Location', 'best');
grid on;
set(gca, 'GridAlpha', 0.3);
yline(1, 'r--', 'LineWidth', 2, 'Label', 'Baseline');

% 子图3：漂移率对比
subplot(2, 2, 3);
drift_matrix = [];
for d = 1:length(dataset_names)
    dataset_name = dataset_names{d};
    dataset_res = all_results.(dataset_name);
    
    drift_row = [];
    for e = 1:size(experiments, 1)
        exp_name = experiments{e, 1};
        drift_row(e) = dataset_res.experiments.(exp_name).drift_rate;
    end
    drift_matrix = [drift_matrix; drift_row];
end

bar_handle3 = bar(drift_matrix');
set(gca, 'XTickLabel', config_names);
xtickangle(15);
ylabel('Drift Rate (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('End-point Drift Rate Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend(dataset_names, 'Location', 'best');
grid on;
set(gca, 'GridAlpha', 0.3);

% 子图4：组件贡献分析（堆叠柱状图）
subplot(2, 2, 4);
contribution_data = degradation_matrix(:, 2:end)' - 1;  % 减去baseline
bar(contribution_data, 'grouped');
set(gca, 'XTickLabel', config_names(2:end));
xtickangle(15);
ylabel('RMSE Increase Ratio', 'FontSize', 12, 'FontWeight', 'bold');
title('Impact of Removing Components', 'FontSize', 14, 'FontWeight', 'bold');
legend(dataset_names, 'Location', 'best');
grid on;
set(gca, 'GridAlpha', 0.3);

sgtitle('Complete Ablation Analysis - Town01 & Town10', 'FontSize', 16, 'FontWeight', 'bold');

% 保存图表
saveas(fig1, fullfile(results_dir, 'ablation_analysis_complete.png'));
fprintf('  Saved: ablation_analysis_complete.png\n');

% 图2：改进倍数雷达图
if length(dataset_names) == 2
    fig2 = figure('Position', [150, 150, 1200, 600]);
    set(fig2, 'Color', [0.95 0.95 0.97]);
    
    % Town01雷达图
    subplot(1, 2, 1);
    town01_degradation = degradation_matrix(1, :);
    theta = linspace(0, 2*pi, length(town01_degradation)+1);
    r = [town01_degradation, town01_degradation(1)];
    
    polarplot(theta, r, 'b-o', 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    hold on;
    polarplot(theta, ones(size(theta)), 'r--', 'LineWidth', 1.5);
    
    thetalabels = [config_names, config_names(1)];
    thetaticks(rad2deg(theta));
    thetaticklabels(thetalabels);
    title('Town01 - Degradation Ratio', 'FontSize', 13, 'FontWeight', 'bold');
    
    % Town10雷达图
    subplot(1, 2, 2);
    town10_degradation = degradation_matrix(2, :);
    r = [town10_degradation, town10_degradation(1)];
    
    polarplot(theta, r, 'g-o', 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    hold on;
    polarplot(theta, ones(size(theta)), 'r--', 'LineWidth', 1.5);
    
    thetaticks(rad2deg(theta));
    thetaticklabels(thetalabels);
    title('Town10 - Degradation Ratio', 'FontSize', 13, 'FontWeight', 'bold');
    
    sgtitle('Ablation Study - Radar Chart Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    saveas(fig2, fullfile(results_dir, 'ablation_radar_chart.png'));
    fprintf('  Saved: ablation_radar_chart.png\n');
end

fprintf('\nComplete ablation study finished!\n');
fprintf('Results can be used directly for paper ablation section\n');
fprintf('Generated charts:\n');
fprintf('   - ablation_analysis_complete.png (4-subplot analysis)\n');
fprintf('   - ablation_radar_chart.png (Radar chart)\n\n');
