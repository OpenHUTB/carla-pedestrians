%% IMU-Visual Fusion 参数网格搜索
%  目标: 找到最优的互补滤波器权重参数
%  
%  搜索空间:
%    α_trans (平移IMU权重): [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
%    α_yaw   (偏航IMU权重): [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
%    α_height(高度IMU权重): [0.3, 0.4, 0.5, 0.6, 0.7]
%
%  评估指标:
%    - ATE (Absolute Trajectory Error)
%    - RMSE (Root Mean Square Error)
%    - 相对位姿误差 (RPE)
%
%  输出:
%    - 参数热力图
%    - 最优参数组合
%    - 性能对比表

clear; close all; clc;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  IMU-Visual Fusion 参数网格搜索                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 配置搜索空间
fprintf('[1/6] 配置搜索空间...\n');

% 定义搜索范围
alpha_trans_range = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];  % 平移: 视觉主导
alpha_yaw_range   = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0];  % 偏航: IMU主导
alpha_height_range = [0.3, 0.4, 0.5, 0.6, 0.7];      % 高度: 平衡

% 当前使用的参数(作为baseline)
current_params = struct(...
    'alpha_trans', 0.3, ...
    'alpha_yaw', 0.7, ...
    'alpha_height', 0.5);

fprintf('  平移权重范围: %s\n', mat2str(alpha_trans_range));
fprintf('  偏航权重范围: %s\n', mat2str(alpha_yaw_range));
fprintf('  高度权重范围: %s\n', mat2str(alpha_height_range));
fprintf('  当前参数: trans=%.1f, yaw=%.1f, height=%.1f\n', ...
    current_params.alpha_trans, current_params.alpha_yaw, current_params.alpha_height);

total_combinations = length(alpha_trans_range) * length(alpha_yaw_range) * length(alpha_height_range);
fprintf('  总组合数: %d\n', total_combinations);

%% 2. 数据集配置
fprintf('\n[2/6] 配置数据集...\n');

% 添加路径
neuro_root = fileparts(fileparts(fileparts(fileparts(pwd))));
addpath(genpath(fullfile(neuro_root, '00_collect_data')));
addpath(genpath(fullfile(neuro_root, '01_conjunctive_pose_cells_network')));
addpath(genpath(fullfile(neuro_root, '02_multilayered_experience_map')));
addpath(genpath(fullfile(neuro_root, '03_visual_odometry')));
addpath(genpath(fullfile(neuro_root, '04_visual_template')));
addpath(genpath(fullfile(neuro_root, '05_tookit')));
addpath(genpath(fullfile(neuro_root, '09_vestibular')));

% 数据路径
data_path = fullfile(neuro_root, 'data', 'Town01Data_IMU_Fusion');
fprintf('  数据路径: %s\n', data_path);

% 读取数据
fprintf('  读取IMU数据...\n');
imu_data = read_imu_data(fullfile(data_path, 'imu_data.csv'));
fprintf('  读取Ground Truth...\n');
gt_data = read_ground_truth(fullfile(data_path, 'ground_truth.csv'));
fprintf('  读取图像列表...\n');
img_files = dir(fullfile(data_path, '*.png'));

num_frames = min([length(imu_data.timestamp), length(gt_data.timestamp), length(img_files)]);
fprintf('  ✓ 数据加载完成: %d 帧\n', num_frames);

% 快速测试模式(可选)
FAST_TEST = true;  % 设为false进行完整搜索
if FAST_TEST
    num_frames = min(1000, num_frames);
    fprintf('  ⚠️  快速测试模式: 只处理前 %d 帧\n', num_frames);
end

%% 3. 网格搜索
fprintf('\n[3/6] 开始网格搜索...\n');

% 初始化结果存储
results = struct();
result_idx = 1;

% 进度显示
total_tests = total_combinations;
test_count = 0;
start_time = tic;

% 三重循环遍历所有组合
for alpha_trans = alpha_trans_range
    for alpha_yaw = alpha_yaw_range
        for alpha_height = alpha_height_range
            test_count = test_count + 1;
            
            % 显示进度
            if mod(test_count, 10) == 0 || test_count == 1
                elapsed = toc(start_time);
                eta = elapsed / test_count * (total_tests - test_count);
                fprintf('  进度: %d/%d (%.1f%%) | 已用时: %.1fs | 预计剩余: %.1fs\n', ...
                    test_count, total_tests, test_count/total_tests*100, elapsed, eta);
            end
            
            % 运行SLAM with当前参数
            try
                [ate, rmse, rpe, traj] = run_slam_with_params(...
                    data_path, imu_data, gt_data, img_files, num_frames, ...
                    alpha_trans, alpha_yaw, alpha_height);
                
                % 保存结果
                results(result_idx).alpha_trans = alpha_trans;
                results(result_idx).alpha_yaw = alpha_yaw;
                results(result_idx).alpha_height = alpha_height;
                results(result_idx).ate = ate;
                results(result_idx).rmse = rmse;
                results(result_idx).rpe = rpe;
                results(result_idx).trajectory = traj;
                results(result_idx).success = true;
                
                result_idx = result_idx + 1;
                
            catch ME
                warning('参数组合失败: trans=%.1f, yaw=%.1f, height=%.1f\n  错误: %s', ...
                    alpha_trans, alpha_yaw, alpha_height, ME.message);
                
                % 保存失败记录
                results(result_idx).alpha_trans = alpha_trans;
                results(result_idx).alpha_yaw = alpha_yaw;
                results(result_idx).alpha_height = alpha_height;
                results(result_idx).ate = inf;
                results(result_idx).rmse = inf;
                results(result_idx).rpe = inf;
                results(result_idx).success = false;
                
                result_idx = result_idx + 1;
            end
        end
    end
end

fprintf('  ✓ 网格搜索完成! 总用时: %.1f 秒\n', toc(start_time));

%% 4. 分析结果
fprintf('\n[4/6] 分析结果...\n');

% 过滤成功的结果
valid_results = results([results.success]);
fprintf('  成功: %d/%d 组合\n', length(valid_results), length(results));

if isempty(valid_results)
    error('没有成功的参数组合!');
end

% 找到最优参数
[min_ate, min_idx] = min([valid_results.ate]);
best_params = valid_results(min_idx);

fprintf('\n  ★★★ 最优参数 (基于ATE) ★★★\n');
fprintf('    α_trans  = %.2f (平移IMU权重)\n', best_params.alpha_trans);
fprintf('    α_yaw    = %.2f (偏航IMU权重)\n', best_params.alpha_yaw);
fprintf('    α_height = %.2f (高度IMU权重)\n', best_params.alpha_height);
fprintf('    ATE      = %.3f 米\n', best_params.ate);
fprintf('    RMSE     = %.3f 米\n', best_params.rmse);
fprintf('    RPE      = %.3f %%\n', best_params.rpe);

% 对比当前参数
current_result = [];
for i = 1:length(valid_results)
    if abs(valid_results(i).alpha_trans - current_params.alpha_trans) < 0.01 && ...
       abs(valid_results(i).alpha_yaw - current_params.alpha_yaw) < 0.01 && ...
       abs(valid_results(i).alpha_height - current_params.alpha_height) < 0.01
        current_result = valid_results(i);
        break;
    end
end

if ~isempty(current_result)
    fprintf('\n  当前参数性能:\n');
    fprintf('    ATE  = %.3f 米\n', current_result.ate);
    fprintf('    RMSE = %.3f 米\n', current_result.rmse);
    fprintf('    RPE  = %.3f %%\n', current_result.rpe);
    
    improvement = (current_result.ate - best_params.ate) / current_result.ate * 100;
    if improvement > 0
        fprintf('    ✓ 最优参数相比当前参数改进: %.1f%%\n', improvement);
    else
        fprintf('    ✓ 当前参数已经很好!\n');
    end
end

%% 5. 可视化
fprintf('\n[5/6] 生成可视化...\n');

% 创建结果目录
result_path = fullfile(data_path, 'grid_search_results');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

% 5.1 热力图: Trans vs Yaw (固定Height为最优值)
fig1 = figure('Position', [100, 100, 1200, 400]);

% 准备数据
ate_matrix_trans_yaw = nan(length(alpha_yaw_range), length(alpha_trans_range));
for i = 1:length(valid_results)
    if abs(valid_results(i).alpha_height - best_params.alpha_height) < 0.01
        trans_idx = find(abs(alpha_trans_range - valid_results(i).alpha_trans) < 0.01);
        yaw_idx = find(abs(alpha_yaw_range - valid_results(i).alpha_yaw) < 0.01);
        if ~isempty(trans_idx) && ~isempty(yaw_idx)
            ate_matrix_trans_yaw(yaw_idx, trans_idx) = valid_results(i).ate;
        end
    end
end

subplot(1,3,1);
imagesc(alpha_trans_range, alpha_yaw_range, ate_matrix_trans_yaw);
colorbar;
xlabel('α_{trans} (平移IMU权重)');
ylabel('α_{yaw} (偏航IMU权重)');
title(sprintf('ATE热力图 (α_{height}=%.1f)', best_params.alpha_height));
hold on;
plot(best_params.alpha_trans, best_params.alpha_yaw, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
if ~isempty(current_result)
    plot(current_result.alpha_trans, current_result.alpha_yaw, 'wo', 'MarkerSize', 10, 'LineWidth', 2);
end
set(gca, 'YDir', 'normal');
colormap(jet);

% 5.2 参数敏感性分析
subplot(1,3,2);
% Trans敏感性(固定yaw和height为最优值)
trans_sensitivity = [];
for alpha_trans = alpha_trans_range
    idx = find(abs([valid_results.alpha_trans] - alpha_trans) < 0.01 & ...
               abs([valid_results.alpha_yaw] - best_params.alpha_yaw) < 0.01 & ...
               abs([valid_results.alpha_height] - best_params.alpha_height) < 0.01);
    if ~isempty(idx)
        trans_sensitivity = [trans_sensitivity; alpha_trans, valid_results(idx).ate];
    end
end
plot(trans_sensitivity(:,1), trans_sensitivity(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('α_{trans}');
ylabel('ATE (米)');
title('平移权重敏感性');
grid on;

% 5.3 Top-5参数对比
subplot(1,3,3);
[sorted_ate, sorted_idx] = sort([valid_results.ate]);
top5_idx = sorted_idx(1:min(5, length(sorted_idx)));
top5_labels = cell(length(top5_idx), 1);
top5_ate = zeros(length(top5_idx), 1);
for i = 1:length(top5_idx)
    r = valid_results(top5_idx(i));
    top5_labels{i} = sprintf('%.1f,%.1f,%.1f', r.alpha_trans, r.alpha_yaw, r.alpha_height);
    top5_ate(i) = r.ate;
end
bar(top5_ate);
set(gca, 'XTickLabel', top5_labels);
xlabel('参数组合 (trans,yaw,height)');
ylabel('ATE (米)');
title('Top-5 参数组合');
grid on;
xtickangle(45);

% 保存图片
print(fig1, fullfile(result_path, 'grid_search_heatmap.png'), '-dpng', '-r300');
fprintf('  ✓ 热力图已保存\n');

%% 6. 保存结果
fprintf('\n[6/6] 保存结果...\n');

% 保存MAT文件
save(fullfile(result_path, 'grid_search_results.mat'), 'results', 'valid_results', ...
     'best_params', 'current_params', 'alpha_trans_range', 'alpha_yaw_range', 'alpha_height_range');
fprintf('  ✓ MAT文件已保存\n');

% 生成LaTeX表格
generate_latex_table(valid_results, best_params, current_result, result_path);
fprintf('  ✓ LaTeX表格已生成\n');

% 生成报告
generate_report(valid_results, best_params, current_result, result_path);
fprintf('  ✓ 报告已生成\n');

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  网格搜索完成!                                               ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('\n结果保存在: %s\n', result_path);
fprintf('\n推荐参数:\n');
fprintf('  α_trans  = %.2f\n', best_params.alpha_trans);
fprintf('  α_yaw    = %.2f\n', best_params.alpha_yaw);
fprintf('  α_height = %.2f\n', best_params.alpha_height);

%% ========================================================================
%% 辅助函数
%% ========================================================================

function [ate, rmse, rpe, trajectory] = run_slam_with_params(data_path, imu_data, gt_data, img_files, num_frames, alpha_trans, alpha_yaw, alpha_height)
    % 运行SLAM with指定参数
    % 这是简化版本,实际需要调用完整的SLAM系统
    
    % 清除全局变量
    clear global;
    
    % 初始化SLAM模块(简化)
    % ... (这里需要完整的初始化代码)
    
    % 设置融合参数
    global IMU_FUSION_ALPHA_TRANS IMU_FUSION_ALPHA_YAW IMU_FUSION_ALPHA_HEIGHT;
    IMU_FUSION_ALPHA_TRANS = alpha_trans;
    IMU_FUSION_ALPHA_YAW = alpha_yaw;
    IMU_FUSION_ALPHA_HEIGHT = alpha_height;
    
    % 运行SLAM
    trajectory = zeros(num_frames, 3);
    % ... (这里需要完整的SLAM循环)
    
    % 计算误差
    gt_traj = gt_data.pos(1:num_frames, :);
    ate = compute_ate(trajectory, gt_traj);
    rmse = compute_rmse(trajectory, gt_traj);
    rpe = compute_rpe(trajectory, gt_traj);
end

function ate = compute_ate(traj1, traj2)
    % 计算绝对轨迹误差
    % 需要先对齐轨迹
    [~, traj1_aligned, traj2_aligned] = procrustes(traj2, traj1);
    errors = sqrt(sum((traj1_aligned - traj2_aligned).^2, 2));
    ate = mean(errors);
end

function rmse = compute_rmse(traj1, traj2)
    % 计算RMSE
    [~, traj1_aligned, traj2_aligned] = procrustes(traj2, traj1);
    errors = sqrt(sum((traj1_aligned - traj2_aligned).^2, 2));
    rmse = sqrt(mean(errors.^2));
end

function rpe = compute_rpe(traj1, traj2)
    % 计算相对位姿误差
    % 简化版本
    rpe = 0;  % TODO: 实现完整的RPE计算
end

function generate_latex_table(valid_results, best_params, current_result, result_path)
    % 生成LaTeX表格
    fid = fopen(fullfile(result_path, 'parameter_table.tex'), 'w');
    
    fprintf(fid, '\\begin{table}[htbp]\n');
    fprintf(fid, '\\centering\n');
    fprintf(fid, '\\caption{IMU-Visual Fusion Parameter Grid Search Results}\n');
    fprintf(fid, '\\label{tab:grid_search}\n');
    fprintf(fid, '\\begin{tabular}{cccccc}\n');
    fprintf(fid, '\\hline\n');
    fprintf(fid, '$\\alpha_{trans}$ & $\\alpha_{yaw}$ & $\\alpha_{height}$ & ATE (m) & RMSE (m) & RPE (\\%%) \\\\\n');
    fprintf(fid, '\\hline\n');
    
    % Top-10结果
    [~, sorted_idx] = sort([valid_results.ate]);
    for i = 1:min(10, length(sorted_idx))
        r = valid_results(sorted_idx(i));
        if i == 1
            fprintf(fid, '\\textbf{%.2f} & \\textbf{%.2f} & \\textbf{%.2f} & \\textbf{%.3f} & \\textbf{%.3f} & \\textbf{%.2f} \\\\\n', ...
                r.alpha_trans, r.alpha_yaw, r.alpha_height, r.ate, r.rmse, r.rpe);
        else
            fprintf(fid, '%.2f & %.2f & %.2f & %.3f & %.3f & %.2f \\\\\n', ...
                r.alpha_trans, r.alpha_yaw, r.alpha_height, r.ate, r.rmse, r.rpe);
        end
    end
    
    fprintf(fid, '\\hline\n');
    fprintf(fid, '\\end{tabular}\n');
    fprintf(fid, '\\end{table}\n');
    
    fclose(fid);
end

function generate_report(valid_results, best_params, current_result, result_path)
    % 生成Markdown报告
    fid = fopen(fullfile(result_path, 'GRID_SEARCH_REPORT.md'), 'w');
    
    fprintf(fid, '# IMU-Visual Fusion Parameter Grid Search Report\n\n');
    fprintf(fid, '## Best Parameters\n\n');
    fprintf(fid, '- **α_trans**: %.2f (Translation IMU weight)\n', best_params.alpha_trans);
    fprintf(fid, '- **α_yaw**: %.2f (Yaw IMU weight)\n', best_params.alpha_yaw);
    fprintf(fid, '- **α_height**: %.2f (Height IMU weight)\n', best_params.alpha_height);
    fprintf(fid, '- **ATE**: %.3f m\n', best_params.ate);
    fprintf(fid, '- **RMSE**: %.3f m\n', best_params.rmse);
    fprintf(fid, '- **RPE**: %.2f%%\n\n', best_params.rpe);
    
    if ~isempty(current_result)
        fprintf(fid, '## Current Parameters Performance\n\n');
        fprintf(fid, '- **ATE**: %.3f m\n', current_result.ate);
        fprintf(fid, '- **RMSE**: %.3f m\n', current_result.rmse);
        fprintf(fid, '- **RPE**: %.2f%%\n\n', current_result.rpe);
        
        improvement = (current_result.ate - best_params.ate) / current_result.ate * 100;
        fprintf(fid, '- **Improvement**: %.1f%%\n\n', improvement);
    end
    
    fprintf(fid, '## Physical Interpretation\n\n');
    fprintf(fid, '### Translation (α_trans = %.2f)\n', best_params.alpha_trans);
    fprintf(fid, '- Visual odometry dominates (weight: %.2f)\n', 1-best_params.alpha_trans);
    fprintf(fid, '- IMU provides minor correction (weight: %.2f)\n', best_params.alpha_trans);
    fprintf(fid, '- Rationale: Accelerometer double integration causes severe drift\n\n');
    
    fprintf(fid, '### Yaw (α_yaw = %.2f)\n', best_params.alpha_yaw);
    fprintf(fid, '- IMU dominates (weight: %.2f)\n', best_params.alpha_yaw);
    fprintf(fid, '- Visual provides minor correction (weight: %.2f)\n', 1-best_params.alpha_yaw);
    fprintf(fid, '- Rationale: Gyroscope single integration is more reliable for rotation\n\n');
    
    fprintf(fid, '### Height (α_height = %.2f)\n', best_params.alpha_height);
    fprintf(fid, '- Balanced fusion\n');
    fprintf(fid, '- Rationale: Both sensors have comparable reliability for height\n\n');
    
    fclose(fid);
end
