%% Transformer增强权重网格搜索
%  搜索最优的 transformer_enhance_weight 和 feature_fusion_weights 参数
%  
%  目标：找到让HART+Transformer VT模块表现最优的Transformer相关参数
%  
%  参数说明：
%    transformer_enhance: Self-Attention增强权重 (当前0.17)
%    dorsal_weight: 位置流权重 (当前0.18)
%    lstm_weight: LSTM时序权重 (当前0.32)
%    transformer_weight: Transformer权重 (当前0.32)
%    v1_weight: V1基础特征权重 (当前0.18)
%  
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    addpath('07_test/07_test/test_imu_visual_slam/parameter_search');
%    addpath('07_test/07_test/test_imu_visual_slam/utils');
%    GRID_SEARCH_TRANSFORMER_WEIGHTS

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     Transformer增强权重网格搜索 (HART+Transformer VT模块)    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
% 搜索范围
% 当前值: transformer_enhance=0.17
transformer_enhance_values = [0.10, 0.15, 0.17, 0.20, 0.25];  % 5个值

% 特征融合权重组合 (dorsal, lstm, transformer, v1)
% 约束: 四个权重之和 = 1.0
fusion_weight_configs = {
    [0.18, 0.32, 0.32, 0.18], ...  % 当前配置
    [0.15, 0.35, 0.35, 0.15], ...  % 增强时序+Transformer
    [0.20, 0.30, 0.30, 0.20], ...  % 增强空间特征
    [0.10, 0.40, 0.40, 0.10], ...  % 强调时序+Transformer
    [0.25, 0.25, 0.25, 0.25], ...  % 均衡配置
    [0.15, 0.40, 0.30, 0.15], ...  % 强调LSTM
    [0.15, 0.30, 0.40, 0.15], ...  % 强调Transformer
};

% 总组合数: 5 * 7 = 35

% 快速测试模式
USE_FAST_MODE = true;
FAST_FRAMES = 1500;

% 数据集
dataset_name = 'Town01Data_IMU_Fusion';

% 禁用IMU，专注测试VT模块
USE_IMU = false;

%% 初始化路径
currentDir = fileparts(mfilename('fullpath'));
testDir = fileparts(currentDir);
rootDir = fileparts(fileparts(fileparts(fileparts(currentDir))));

addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '04_visual_template/04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '09_vestibular'));
addpath(fullfile(rootDir, '09_vestibular/09_vestibular'));
addpath(fullfile(testDir, 'utils'));
addpath(fullfile(testDir, 'core'));

%% 读取数据
fprintf('[1/3] 加载数据...\n');
data_path = fullfile(rootDir, 'data', dataset_name);
if ~exist(data_path, 'dir')
    data_path = fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets', dataset_name);
end

imu_data = read_imu_data(data_path);
img_files = dir(fullfile(data_path, '*.png'));

if USE_FAST_MODE
    num_frames = min([length(img_files), FAST_FRAMES]);
    fprintf('  快速模式: %d 帧\n', num_frames);
else
    num_frames = length(img_files);
    fprintf('  完整模式: %d 帧\n', num_frames);
end

%% 先运行一次Baseline
fprintf('\n[2/3] 运行Baseline (只需一次)...\n');
baseline_ate = run_baseline_slam(data_path, img_files, num_frames, rootDir);
fprintf('  Baseline ATE: %.2f m\n', baseline_ate);

%% 网格搜索
fprintf('\n[3/3] 开始Transformer权重网格搜索...\n');

total_combinations = length(transformer_enhance_values) * length(fusion_weight_configs);
fprintf('  transformer_enhance: %s (共%d个值)\n', mat2str(transformer_enhance_values), length(transformer_enhance_values));
fprintf('  fusion_weight配置: %d种\n', length(fusion_weight_configs));
fprintf('  总组合数: %d\n', total_combinations);
fprintf('  预计时间: %.1f 分钟\n', total_combinations * 2.5);

% 存储所有结果
all_results = [];
result_idx = 0;

% 禁用IMU
global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0;
IMU_TRANS_WEIGHT_OVERRIDE = 0;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0;

for i_enhance = 1:length(transformer_enhance_values)
    for i_fusion = 1:length(fusion_weight_configs)
        result_idx = result_idx + 1;
        
        trans_enhance = transformer_enhance_values(i_enhance);
        fusion_weights = fusion_weight_configs{i_fusion};
        
        fprintf('\n--- 测试 [%d/%d] enhance=%.2f, fusion=[%.2f,%.2f,%.2f,%.2f] ---\n', ...
            result_idx, total_combinations, trans_enhance, ...
            fusion_weights(1), fusion_weights(2), fusion_weights(3), fusion_weights(4));
        
        % 设置Transformer参数（通过全局变量）
        global TRANSFORMER_ENHANCE_OVERRIDE FUSION_WEIGHTS_OVERRIDE;
        TRANSFORMER_ENHANCE_OVERRIDE = trans_enhance;
        FUSION_WEIGHTS_OVERRIDE = fusion_weights;
        
        % 运行Ours SLAM
        [ours_ate, ours_vt] = run_ours_slam_transformer(data_path, img_files, imu_data, num_frames, rootDir);
        
        % 计算改进
        improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
        
        % 保存结果
        r = struct();
        r.transformer_enhance = trans_enhance;
        r.fusion_weights = fusion_weights;
        r.ate = ours_ate;
        r.improvement = improvement;
        r.vt_count = ours_vt;
        
        if isempty(all_results)
            all_results = r;
        else
            all_results(end+1) = r;
        end
        
        fprintf('  ATE: %.2f m, 改进: %+.2f%%, VT数: %d\n', ours_ate, improvement, ours_vt);
    end
end

%% 显示结果
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                      Transformer权重网格搜索结果                                     ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ enhance │ dorsal │  lstm  │ trans  │   v1   │   ATE (m)  │  改进 (%%)  │  VT数  ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════════════════╣\n');

% 找最优参数
best_improvement = -inf;
best_idx = 1;

for i = 1:length(all_results)
    r = all_results(i);
    fw = r.fusion_weights;
    fprintf('║  %.2f   │  %.2f  │  %.2f  │  %.2f  │  %.2f  │  %7.2f   │  %+7.2f   │  %4d  ║\n', ...
        r.transformer_enhance, fw(1), fw(2), fw(3), fw(4), r.ate, r.improvement, r.vt_count);
    
    if r.improvement > best_improvement
        best_improvement = r.improvement;
        best_idx = i;
    end
end

fprintf('╠══════════════════════════════════════════════════════════════════════════════════════╣\n');

best = all_results(best_idx);
bfw = best.fusion_weights;
fprintf('║  最优参数: enhance=%.2f, fusion=[%.2f,%.2f,%.2f,%.2f]                               ║\n', ...
    best.transformer_enhance, bfw(1), bfw(2), bfw(3), bfw(4));
fprintf('║  最优ATE: %.2f m (改进 %+.2f%%)                                                      ║\n', best.ate, best.improvement);
fprintf('║  Baseline ATE: %.2f m                                                                ║\n', baseline_ate);
fprintf('╚══════════════════════════════════════════════════════════════════════════════════════╝\n');

%% 保存结果
results = struct();
results.all_results = all_results;
results.transformer_enhance_values = transformer_enhance_values;
results.fusion_weight_configs = fusion_weight_configs;
results.baseline_ate = baseline_ate;
results.best = best;
results.num_frames = num_frames;
results.dataset = dataset_name;
results.timestamp = datestr(now);

save_path = fullfile(data_path, 'transformer_weights_search_results.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% 绘制结果图
figure('Position', [100, 100, 800, 400]);

improvement_values = [all_results.improvement];
bar(1:length(all_results), improvement_values);
hold on;
plot([0, length(all_results)+1], [0, 0], 'k-', 'LineWidth', 1);
plot([0, length(all_results)+1], [5, 5], 'g--', 'LineWidth', 2);
plot(best_idx, best.improvement, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('参数组合编号');
ylabel('改进 (%)');
title(sprintf('Transformer权重网格搜索 (%s, %d帧)', dataset_name, num_frames));
legend('改进', '零线', '目标5%', '最优', 'Location', 'best');
grid on;

saveas(gcf, fullfile(data_path, 'transformer_weights_search_results.png'));
fprintf('图表已保存到: %s\n', fullfile(data_path, 'transformer_weights_search_results.png'));

fprintf('\n✓ Transformer权重网格搜索完成！\n');
fprintf('  推荐使用: enhance=%.2f, fusion=[%.2f,%.2f,%.2f,%.2f]\n', ...
    best.transformer_enhance, bfw(1), bfw(2), bfw(3), bfw(4));
