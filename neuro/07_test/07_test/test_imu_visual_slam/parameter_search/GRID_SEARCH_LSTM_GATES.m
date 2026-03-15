%% LSTM门控参数网格搜索
%  搜索最优的 input_gate, forget_gate, output_gate 参数
%  
%  目标：找到让HART+Transformer VT模块表现最优的LSTM门控参数
%  
%  参数说明：
%    input_gate: 控制新输入信息的接收程度 (0~1)
%    forget_gate: 控制历史信息的保留程度 (0~1)
%    output_gate: 控制输出信息的强度 (0~1)
%  
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    addpath('07_test/07_test/test_imu_visual_slam/parameter_search');
%    addpath('07_test/07_test/test_imu_visual_slam/utils');
%    GRID_SEARCH_LSTM_GATES

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     LSTM门控参数网格搜索 (HART+Transformer VT模块)           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
% 搜索范围
% 当前值: input=0.55, forget=0.6, output=0.92
input_gate_values = [0.4, 0.5, 0.55, 0.6, 0.7];      % 5个值
forget_gate_values = [0.4, 0.5, 0.6, 0.7, 0.8];      % 5个值
output_gate_values = [0.85, 0.9, 0.92, 0.95];        % 4个值

% 总组合数: 5 * 5 * 4 = 100

% 快速测试模式
USE_FAST_MODE = true;
FAST_FRAMES = 1500;  % 用1500帧快速评估

% 数据集
dataset_name = 'Town01Data_IMU_Fusion';

% IMU融合参数（使用网格搜索得到的最优值，或者禁用IMU）
% 设为0禁用IMU，专注测试LSTM参数
USE_IMU = false;  % ★★★ 禁用IMU，专注测试VT模块 ★★★
ALPHA_YAW = 0.0;
ALPHA_TRANS = 0.0;
ALPHA_HEIGHT = 0.0;

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

% ★★★ 检查数据路径是否存在 ★★★
if ~exist(data_path, 'dir')
    error('数据路径不存在: %s\n请检查dataset_name设置', data_path);
end
fprintf('  数据路径: %s\n', data_path);

imu_data = read_imu_data(data_path);
fprintf('  IMU数据: %d 帧\n', imu_data.count);

fusion_data = read_fusion_pose(data_path);
fprintf('  融合位姿: %d 帧\n', size(fusion_data.pos, 1));

gt_file = fullfile(data_path, 'ground_truth.txt');
if ~exist(gt_file, 'file')
    error('Ground Truth文件不存在: %s', gt_file);
end
gt_data = read_ground_truth(gt_file);
fprintf('  Ground Truth: %d 帧\n', size(gt_data.pos, 1));

img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到图像文件: %s/*.png', data_path);
end
fprintf('  图像数量: %d 张\n', length(img_files));

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
fprintf('\n[3/3] 开始LSTM门控参数网格搜索...\n');

total_combinations = length(input_gate_values) * length(forget_gate_values) * length(output_gate_values);
fprintf('  input_gate: %s (共%d个值)\n', mat2str(input_gate_values), length(input_gate_values));
fprintf('  forget_gate: %s (共%d个值)\n', mat2str(forget_gate_values), length(forget_gate_values));
fprintf('  output_gate: %s (共%d个值)\n', mat2str(output_gate_values), length(output_gate_values));
fprintf('  总组合数: %d\n', total_combinations);
fprintf('  预计时间: %.1f 分钟\n', total_combinations * 2.5);

% 存储所有结果
all_results = [];
result_idx = 0;

% 设置IMU参数（禁用或使用固定值）
global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = ALPHA_YAW;
IMU_TRANS_WEIGHT_OVERRIDE = ALPHA_TRANS;
IMU_HEIGHT_WEIGHT_OVERRIDE = ALPHA_HEIGHT;

for i_input = 1:length(input_gate_values)
    for i_forget = 1:length(forget_gate_values)
        for i_output = 1:length(output_gate_values)
            result_idx = result_idx + 1;
            
            input_gate = input_gate_values(i_input);
            forget_gate = forget_gate_values(i_forget);
            output_gate = output_gate_values(i_output);
            
            fprintf('\n--- 测试 [%d/%d] input=%.2f, forget=%.2f, output=%.2f ---\n', ...
                result_idx, total_combinations, input_gate, forget_gate, output_gate);
            
            % 设置LSTM门控参数（通过全局变量）
            global LSTM_INPUT_GATE_OVERRIDE LSTM_FORGET_GATE_OVERRIDE LSTM_OUTPUT_GATE_OVERRIDE;
            LSTM_INPUT_GATE_OVERRIDE = input_gate;
            LSTM_FORGET_GATE_OVERRIDE = forget_gate;
            LSTM_OUTPUT_GATE_OVERRIDE = output_gate;
            
            % 运行Ours SLAM
            [ours_ate, ours_vt] = run_ours_slam_lstm(data_path, img_files, imu_data, num_frames, rootDir);
            
            % 计算改进
            improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
            
            % 保存结果
            r = struct();
            r.input_gate = input_gate;
            r.forget_gate = forget_gate;
            r.output_gate = output_gate;
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
end

%% 显示结果
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                      LSTM门控参数网格搜索结果                              ║\n');
fprintf('╠════════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║ input_gate │ forget_gate │ output_gate │   ATE (m)  │  改进 (%%)  │  VT数  ║\n');
fprintf('╠════════════════════════════════════════════════════════════════════════════╣\n');

% 找最优参数
best_improvement = -inf;
best_idx = 1;

for i = 1:length(all_results)
    r = all_results(i);
    fprintf('║    %.2f    │    %.2f     │    %.2f     │  %7.2f   │  %+7.2f   │  %4d  ║\n', ...
        r.input_gate, r.forget_gate, r.output_gate, r.ate, r.improvement, r.vt_count);
    
    if r.improvement > best_improvement
        best_improvement = r.improvement;
        best_idx = i;
    end
end

fprintf('╠════════════════════════════════════════════════════════════════════════════╣\n');

best = all_results(best_idx);
fprintf('║  最优参数: input=%.2f, forget=%.2f, output=%.2f                           ║\n', ...
    best.input_gate, best.forget_gate, best.output_gate);
fprintf('║  最优ATE: %.2f m (改进 %+.2f%%)                                            ║\n', best.ate, best.improvement);
fprintf('║  Baseline ATE: %.2f m                                                      ║\n', baseline_ate);
fprintf('╚════════════════════════════════════════════════════════════════════════════╝\n');

%% 保存结果
results = struct();
results.all_results = all_results;
results.input_gate_values = input_gate_values;
results.forget_gate_values = forget_gate_values;
results.output_gate_values = output_gate_values;
results.baseline_ate = baseline_ate;
results.best = best;
results.num_frames = num_frames;
results.dataset = dataset_name;
results.timestamp = datestr(now);
results.imu_enabled = USE_IMU;

save_path = fullfile(data_path, 'lstm_gates_search_results.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% 绘制结果图
figure('Position', [100, 100, 1000, 400]);

% 子图1: 所有组合的改进
subplot(1,2,1);
improvement_values = [all_results.improvement];
bar(1:length(all_results), improvement_values);
hold on;
plot([0, length(all_results)+1], [0, 0], 'k-', 'LineWidth', 1);
plot([0, length(all_results)+1], [5, 5], 'g--', 'LineWidth', 2);
plot(best_idx, best.improvement, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('参数组合编号');
ylabel('改进 (%)');
title('LSTM门控参数搜索结果');
legend('改进', '零线', '目标5%', '最优', 'Location', 'best');
grid on;

% 子图2: 热力图（input vs forget，固定output为最优值）
subplot(1,2,2);
% 找到最优output_gate对应的所有结果
best_output = best.output_gate;
heatmap_data = nan(length(input_gate_values), length(forget_gate_values));
for i = 1:length(all_results)
    r = all_results(i);
    if r.output_gate == best_output
        i_in = find(input_gate_values == r.input_gate);
        i_fg = find(forget_gate_values == r.forget_gate);
        if ~isempty(i_in) && ~isempty(i_fg)
            heatmap_data(i_in, i_fg) = r.improvement;
        end
    end
end

imagesc(heatmap_data);
colorbar;
colormap(jet);
xlabel('forget\_gate');
ylabel('input\_gate');
title(sprintf('改进热力图 (output=%.2f)', best_output));
set(gca, 'XTick', 1:length(forget_gate_values), 'XTickLabel', forget_gate_values);
set(gca, 'YTick', 1:length(input_gate_values), 'YTickLabel', input_gate_values);

sgtitle(sprintf('LSTM门控参数网格搜索 (%s, %d帧, IMU=%s)', ...
    dataset_name, num_frames, mat2str(USE_IMU)));

saveas(gcf, fullfile(data_path, 'lstm_gates_search_results.png'));
fprintf('图表已保存到: %s\n', fullfile(data_path, 'lstm_gates_search_results.png'));

fprintf('\n✓ LSTM门控参数网格搜索完成！\n');
fprintf('  推荐使用: input_gate=%.2f, forget_gate=%.2f, output_gate=%.2f\n', ...
    best.input_gate, best.forget_gate, best.output_gate);
fprintf('\n  下一步: 将最优参数更新到 hart_transformer_extractor.m\n');
