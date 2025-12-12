%% ========== 消融实验主脚本 ==========
% 目的：验证系统各组件的贡献
% 实验设计：
%   1. 完整系统 (Baseline)
%   2. 去掉IMU (纯视觉)
%   3. 去掉LSTM (无时序记忆)
%   4. 去掉Transformer (无全局上下文)
%   5. 去掉双流 (单特征流)
%   6. 去掉注意力 (无空间注意力)
%   7. 简化特征 (基础方法)
% ========================================

% 检查数据集设置
global DATASET_NAME;
dataset_name = 'Town01Data_IMU_Fusion';  % 默认Town01
try
    if ~isempty(DATASET_NAME)
        dataset_name = DATASET_NAME;
    end
catch
    % 使用默认值
end

clear all; close all; clc;

% 恢复数据集名称
global DATASET_NAME;
if exist('dataset_name', 'var')
    DATASET_NAME = dataset_name;
else
    DATASET_NAME = 'Town01Data_IMU_Fusion';
end

fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║     HART+Transformer SLAM 消融实验             ║\n');
fprintf('║         Ablation Study Framework               ║\n');
fprintf('║     数据集: %-34s ║\n', DATASET_NAME);
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 配置
% 基于脚本位置构建相对路径
script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, '..', '..', 'data', '01_NeuroSLAM_Datasets', DATASET_NAME, 'ablation_results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
    fprintf('✓ 创建结果目录: %s\n', results_dir);
end

% 实验组定义
experiments = {
    % 实验名称,              描述,                      配置
    'Full_System',          '完整系统 (Baseline)',      struct('imu', true, 'lstm', true, 'transformer', true, 'dual_stream', true, 'attention', true, 'full_feature', true);
    'No_IMU',              '去掉IMU (纯视觉)',         struct('imu', false, 'lstm', true, 'transformer', true, 'dual_stream', true, 'attention', true, 'full_feature', true);
    'No_LSTM',             '去掉LSTM (无时序)',        struct('imu', true, 'lstm', false, 'transformer', true, 'dual_stream', true, 'attention', true, 'full_feature', true);
    'No_Transformer',      '去掉Transformer',          struct('imu', true, 'lstm', true, 'transformer', false, 'dual_stream', true, 'attention', true, 'full_feature', true);
    'No_Dual_Stream',      '去掉双流 (单流)',          struct('imu', true, 'lstm', true, 'transformer', true, 'dual_stream', false, 'attention', true, 'full_feature', true);
    'No_Attention',        '去掉注意力',               struct('imu', true, 'lstm', true, 'transformer', true, 'dual_stream', true, 'attention', false, 'full_feature', true);
    'Simplified_Feature',  '简化特征 (对比)',          struct('imu', true, 'lstm', false, 'transformer', false, 'dual_stream', false, 'attention', false, 'full_feature', false);
};

num_experiments = size(experiments, 1);

%% 存储结果
results = struct();
results.experiment_names = experiments(:, 1);
results.descriptions = experiments(:, 2);
results.vt_counts = zeros(num_experiments, 1);
results.exp_counts = zeros(num_experiments, 1);
results.rmse_values = zeros(num_experiments, 1);
results.rpe_values = zeros(num_experiments, 1);
results.drift_rates = zeros(num_experiments, 1);
results.processing_times = zeros(num_experiments, 1);

%% 运行每个实验
for i = 1:num_experiments
    exp_name = experiments{i, 1};
    exp_desc = experiments{i, 2};
    exp_config = experiments{i, 3};
    
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════\n');
    fprintf('实验 %d/%d: %s\n', i, num_experiments, exp_desc);
    fprintf('═══════════════════════════════════════════════\n');
    
    % 运行实验（使用简化版本）
    tic;
    result = run_single_ablation_simple(exp_name, exp_config);
    elapsed_time = toc;
    
    % 保存结果
    results.vt_counts(i) = result.vt_count;
    results.exp_counts(i) = result.exp_count;
    results.rmse_values(i) = result.rmse;
    results.rpe_values(i) = result.rpe;
    results.drift_rates(i) = result.drift_rate;
    results.processing_times(i) = elapsed_time;
    
    fprintf('\n结果摘要:\n');
    fprintf('  VT数量: %d\n', result.vt_count);
    fprintf('  经验节点: %d\n', result.exp_count);
    fprintf('  RMSE: %.2f m\n', result.rmse);
    fprintf('  RPE: %.4f m\n', result.rpe);
    fprintf('  漂移率: %.2f%%\n', result.drift_rate);
    fprintf('  处理时间: %.1f 秒\n', elapsed_time);
    
    % 保存单个实验结果
    save(fullfile(results_dir, sprintf('%s_result.mat', exp_name)), 'result');
end

%% 保存完整结果
save(fullfile(results_dir, 'ablation_study_results.mat'), 'results', 'experiments');

%% 生成对比图表
fprintf('\n');
fprintf('═══════════════════════════════════════════════\n');
fprintf('生成对比图表...\n');
fprintf('═══════════════════════════════════════════════\n');

generate_ablation_visualizations(results, results_dir);

%% 生成对比表格
generate_ablation_tables(results, results_dir);

%% 生成LaTeX表格
generate_latex_table(results, results_dir);

fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║         消融实验完成！                          ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n结果保存在: %s\n', results_dir);
fprintf('\n');
