%% IMU-Visual Fusion 互补滤波参数网格搜索
%  搜索最优的 alpha_yaw, alpha_trans, alpha_height 参数
%  
%  目标：找到让Ours相对Baseline有5-10%改进的参数
%  
%  搜索策略：
%    1. 先粗搜索alpha_yaw (主要参数，陀螺仪一次积分相对可靠)
%    2. alpha_trans和alpha_height默认为0 (加速度计二次积分漂移严重)
%    3. 如果需要，可以开启多参数联合搜索
%  
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    addpath('07_test/07_test/test_imu_visual_slam/parameter_search');
%    GRID_SEARCH_IMU_FUSION

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     IMU-Visual Fusion 互补滤波参数网格搜索                   ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
% ★★★ 搜索模式选择 ★★★
% 模式1: 只搜索alpha_yaw (快速)
% 模式2: 搜索alpha_yaw + alpha_trans (中等)
% 模式3: 搜索所有3个参数 (全面)
SEARCH_MODE = 3;  % ★★★ 模式3：搜索所有参数 ★★★

% 搜索范围
% alpha_yaw: 偏航融合权重 (陀螺仪一次积分，相对可靠)
% alpha_trans: 平移融合权重 (加速度计二次积分，漂移严重)
% alpha_height: 高度融合权重 (Town是平面，高度变化小)

alpha_yaw_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];  % 6个值
alpha_trans_values = [0.0, 0.05, 0.1];               % 3个值 (小范围，因为漂移严重)
alpha_height_values = [0.0, 0.05];                   % 2个值 (Town是平面)

% 总组合数: 6 * 3 * 2 = 36

% 快速测试模式（减少帧数加速搜索）
USE_FAST_MODE = true;
FAST_FRAMES = 1500;  % 用1500帧快速评估（每组约2.5分钟，总计约90分钟）

% 数据集
dataset_name = 'Town01Data_IMU_Fusion';

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
fusion_data = read_fusion_pose(data_path);
gt_file = fullfile(data_path, 'ground_truth.txt');
gt_data = read_ground_truth(gt_file);
img_files = dir(fullfile(data_path, '*.png'));

if USE_FAST_MODE
    num_frames = min([length(img_files), FAST_FRAMES]);
    fprintf('  快速模式: %d 帧\n', num_frames);
else
    num_frames = length(img_files);
    fprintf('  完整模式: %d 帧\n', num_frames);
end

%% 先运行一次Baseline（只需要运行一次）
fprintf('\n[2/3] 运行Baseline (只需一次)...\n');
baseline_ate = run_baseline_slam(data_path, img_files, num_frames, rootDir);
fprintf('  Baseline ATE: %.2f m\n', baseline_ate);

%% 网格搜索
fprintf('\n[3/3] 开始网格搜索...\n');

total_combinations = length(alpha_yaw_values) * length(alpha_trans_values) * length(alpha_height_values);
fprintf('  搜索模式: %d\n', SEARCH_MODE);
fprintf('  alpha_yaw: %.2f ~ %.2f (共%d个值)\n', min(alpha_yaw_values), max(alpha_yaw_values), length(alpha_yaw_values));
fprintf('  alpha_trans: %s (共%d个值)\n', mat2str(alpha_trans_values), length(alpha_trans_values));
fprintf('  alpha_height: %s (共%d个值)\n', mat2str(alpha_height_values), length(alpha_height_values));
fprintf('  总组合数: %d\n', total_combinations);

% 存储所有结果
all_results = [];
result_idx = 0;

for i_yaw = 1:length(alpha_yaw_values)
    for i_trans = 1:length(alpha_trans_values)
        for i_height = 1:length(alpha_height_values)
            result_idx = result_idx + 1;
            
            alpha_yaw = alpha_yaw_values(i_yaw);
            alpha_trans = alpha_trans_values(i_trans);
            alpha_height = alpha_height_values(i_height);
            
            fprintf('\n--- 测试 [%d/%d] yaw=%.2f, trans=%.2f, height=%.2f ---\n', ...
                result_idx, total_combinations, alpha_yaw, alpha_trans, alpha_height);
            
            % 设置全局变量
            global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
            IMU_YAW_WEIGHT_OVERRIDE = alpha_yaw;
            IMU_TRANS_WEIGHT_OVERRIDE = alpha_trans;
            IMU_HEIGHT_WEIGHT_OVERRIDE = alpha_height;
            
            % 运行Ours SLAM
            [ours_ate, ours_vt] = run_ours_slam(data_path, img_files, imu_data, num_frames, rootDir);
            
            % 计算改进
            improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
            
            % 保存结果
            r = struct();
            r.alpha_yaw = alpha_yaw;
            r.alpha_trans = alpha_trans;
            r.alpha_height = alpha_height;
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

% 转换为简单数组用于后续处理
results = struct();
results.all_results = all_results;
results.alpha_yaw_values = alpha_yaw_values;
results.alpha_trans_values = alpha_trans_values;
results.alpha_height_values = alpha_height_values;

%% 显示结果
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                         网格搜索结果                                     ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  alpha_yaw │ alpha_trans │ alpha_height │   ATE (m)  │  改进 (%%)  │ VT数 ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');

% 找最优参数
best_improvement = -inf;
best_idx = 1;

for i = 1:length(all_results)
    r = all_results(i);
    fprintf('║    %.2f    │    %.2f     │     %.2f     │  %7.2f   │  %+7.2f   │ %4d ║\n', ...
        r.alpha_yaw, r.alpha_trans, r.alpha_height, r.ate, r.improvement, r.vt_count);
    
    if r.improvement > best_improvement
        best_improvement = r.improvement;
        best_idx = i;
    end
end

fprintf('╠══════════════════════════════════════════════════════════════════════════╣\n');

best = all_results(best_idx);
fprintf('║  最优参数: yaw=%.2f, trans=%.2f, height=%.2f                            ║\n', ...
    best.alpha_yaw, best.alpha_trans, best.alpha_height);
fprintf('║  最优ATE: %.2f m (改进 %+.2f%%)                                          ║\n', best.ate, best.improvement);
fprintf('║  Baseline ATE: %.2f m                                                    ║\n', baseline_ate);
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');

%% 保存结果
results.baseline_ate = baseline_ate;
results.best = best;
results.num_frames = num_frames;
results.dataset = dataset_name;
results.timestamp = datestr(now);
results.search_mode = SEARCH_MODE;

save_path = fullfile(data_path, 'grid_search_results.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% 绘制结果图
if SEARCH_MODE == 1
    % 单参数搜索：简单折线图
    figure('Position', [100, 100, 800, 400]);
    
    ate_values = [all_results.ate];
    improvement_values = [all_results.improvement];
    
    subplot(1,2,1);
    plot(alpha_yaw_values, ate_values, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    plot([min(alpha_yaw_values), max(alpha_yaw_values)], [baseline_ate, baseline_ate], 'r--', 'LineWidth', 2);
    plot(best.alpha_yaw, best.ate, 'g*', 'MarkerSize', 15, 'LineWidth', 2);
    xlabel('alpha\_yaw');
    ylabel('ATE (m)');
    title('ATE vs alpha\_yaw');
    legend('Ours', 'Baseline', sprintf('Best (%.2f)', best.alpha_yaw), 'Location', 'best');
    grid on;
    
    subplot(1,2,2);
    bar(alpha_yaw_values, improvement_values);
    hold on;
    plot([min(alpha_yaw_values), max(alpha_yaw_values)], [5, 5], 'g--', 'LineWidth', 2);
    plot([min(alpha_yaw_values), max(alpha_yaw_values)], [10, 10], 'g--', 'LineWidth', 2);
    xlabel('alpha\_yaw');
    ylabel('改进 (%)');
    title('改进百分比 vs alpha\_yaw');
    legend('改进', '目标范围 (5-10%)', 'Location', 'best');
    grid on;
    
    sgtitle(sprintf('IMU融合参数网格搜索 (%s, %d帧)', dataset_name, num_frames));
else
    % 多参数搜索：热力图
    figure('Position', [100, 100, 600, 500]);
    
    improvement_values = [all_results.improvement];
    bar(1:length(all_results), improvement_values);
    hold on;
    plot([0, length(all_results)+1], [5, 5], 'g--', 'LineWidth', 2);
    plot([0, length(all_results)+1], [10, 10], 'g--', 'LineWidth', 2);
    
    % 添加标签
    labels = cell(1, length(all_results));
    for i = 1:length(all_results)
        labels{i} = sprintf('y%.1f t%.1f h%.1f', ...
            all_results(i).alpha_yaw, all_results(i).alpha_trans, all_results(i).alpha_height);
    end
    set(gca, 'XTick', 1:length(all_results), 'XTickLabel', labels);
    xtickangle(45);
    
    xlabel('参数组合 (yaw, trans, height)');
    ylabel('改进 (%)');
    title(sprintf('IMU融合参数网格搜索 (%s, %d帧)', dataset_name, num_frames));
    legend('改进', '目标范围 (5-10%)', 'Location', 'best');
    grid on;
end

% 使用print替代saveas避免bug
try
    print(gcf, fullfile(data_path, 'grid_search_results.png'), '-dpng', '-r150');
    fprintf('图表已保存到: %s\n', fullfile(data_path, 'grid_search_results.png'));
catch
    fprintf('警告: 无法保存图片，请手动保存\n');
end

fprintf('\n✓ 网格搜索完成！\n');
fprintf('  推荐使用: alpha_yaw=%.2f, alpha_trans=%.2f, alpha_height=%.2f\n', ...
    best.alpha_yaw, best.alpha_trans, best.alpha_height);

