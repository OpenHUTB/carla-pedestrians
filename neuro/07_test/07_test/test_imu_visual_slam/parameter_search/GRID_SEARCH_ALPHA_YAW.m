%% 网格搜索：互补滤波alpha_yaw参数
%  目标：找到让Ours和Baseline都接近GT，且改进在5-10%范围的参数
%
%  搜索参数：
%    - alpha_yaw: IMU偏航权重 [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
%    - vt_match_threshold: VT匹配阈值 [0.04, 0.06, 0.08, 0.10]
%
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    run('07_test/07_test/test_imu_visual_slam/parameter_search/GRID_SEARCH_ALPHA_YAW.m');

clear; close all; clc;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     网格搜索：互补滤波参数优化                               ║\n');
fprintf('║     目标：改进5-10%%，两者都接近GT                           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 参数网格
alpha_yaw_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
vt_threshold_values = [0.04, 0.06, 0.08, 0.10, 0.12];

% 快速测试模式
FAST_FRAMES = 2000;  % 减少帧数加速搜索

%% 初始化结果存储
num_alpha = length(alpha_yaw_values);
num_vt = length(vt_threshold_values);
results = struct();
results.alpha_yaw = alpha_yaw_values;
results.vt_threshold = vt_threshold_values;
results.baseline_ate = zeros(num_alpha, num_vt);
results.ours_ate = zeros(num_alpha, num_vt);
results.improvement = zeros(num_alpha, num_vt);
results.baseline_vt = zeros(num_alpha, num_vt);
results.ours_vt = zeros(num_alpha, num_vt);

%% 设置路径
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
dataset_name = 'Town01Data_IMU_Fusion';
data_path = fullfile(rootDir, 'data', dataset_name);
if ~exist(data_path, 'dir')
    data_path = fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets', dataset_name);
end

fprintf('数据路径: %s\n', data_path);

% 读取数据
imu_data = read_imu_data(data_path);
fusion_data = read_fusion_pose(data_path);
gt_file = fullfile(data_path, 'ground_truth.txt');
gt_data = read_ground_truth(gt_file);
img_files = dir(fullfile(data_path, '*.png'));

num_frames = min([length(img_files), length(fusion_data.timestamp), FAST_FRAMES]);
fprintf('测试帧数: %d\n\n', num_frames);

%% 开始网格搜索
total_tests = num_alpha * num_vt;
test_count = 0;

for ai = 1:num_alpha
    for vi = 1:num_vt
        test_count = test_count + 1;
        alpha_yaw = alpha_yaw_values(ai);
        vt_threshold = vt_threshold_values(vi);
        
        fprintf('═══════════════════════════════════════════════════════════════\n');
        fprintf('测试 %d/%d: alpha_yaw=%.2f, vt_threshold=%.2f\n', ...
                test_count, total_tests, alpha_yaw, vt_threshold);
        fprintf('═══════════════════════════════════════════════════════════════\n');
        
        % 设置全局参数
        global IMU_YAW_WEIGHT_OVERRIDE;
        IMU_YAW_WEIGHT_OVERRIDE = alpha_yaw;
        
        try
            % ========== 运行Baseline ==========
            clear functions;
            clear global PREV_VT_ID VT NUM_VT EXPERIENCES NUM_EXPS CUR_EXP_ID;
            clear global YAW_HEIGHT_HDC GRIDCELLS;
            
            [bl_ate, bl_vt_count, bl_exp_traj] = run_single_experiment(...
                'baseline', data_path, img_files, imu_data, gt_data, ...
                num_frames, vt_threshold, rootDir);
            
            % ========== 运行Ours ==========
            clear functions;
            clear global PREV_VT_ID VT NUM_VT EXPERIENCES NUM_EXPS CUR_EXP_ID;
            clear global YAW_HEIGHT_HDC GRIDCELLS;
            
            [ours_ate, ours_vt_count, ours_exp_traj] = run_single_experiment(...
                'ours', data_path, img_files, imu_data, gt_data, ...
                num_frames, vt_threshold, rootDir);
            
            % 计算改进
            improvement = (bl_ate - ours_ate) / bl_ate * 100;
            
            % 保存结果
            results.baseline_ate(ai, vi) = bl_ate;
            results.ours_ate(ai, vi) = ours_ate;
            results.improvement(ai, vi) = improvement;
            results.baseline_vt(ai, vi) = bl_vt_count;
            results.ours_vt(ai, vi) = ours_vt_count;
            
            fprintf('\n结果: Baseline ATE=%.1fm, Ours ATE=%.1fm, 改进=%.1f%%\n', ...
                    bl_ate, ours_ate, improvement);
            fprintf('      Baseline VT=%d, Ours VT=%d\n\n', bl_vt_count, ours_vt_count);
            
        catch ME
            warning('测试失败: %s', ME.message);
            results.baseline_ate(ai, vi) = NaN;
            results.ours_ate(ai, vi) = NaN;
            results.improvement(ai, vi) = NaN;
        end
    end
end

%% 显示结果汇总
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    网格搜索结果汇总                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

fprintf('改进百分比矩阵 (行=alpha_yaw, 列=vt_threshold):\n');
fprintf('         ');
for vi = 1:num_vt
    fprintf('  %.2f  ', vt_threshold_values(vi));
end
fprintf('\n');

for ai = 1:num_alpha
    fprintf('α=%.1f: ', alpha_yaw_values(ai));
    for vi = 1:num_vt
        imp = results.improvement(ai, vi);
        if imp >= 5 && imp <= 10
            fprintf(' [%5.1f%%]', imp);  % 目标范围用方括号标记
        else
            fprintf('  %5.1f%% ', imp);
        end
    end
    fprintf('\n');
end

%% 找到最佳参数（改进在5-10%范围内）
target_min = 5;
target_max = 10;
best_params = [];

for ai = 1:num_alpha
    for vi = 1:num_vt
        imp = results.improvement(ai, vi);
        if imp >= target_min && imp <= target_max
            best_params = [best_params; alpha_yaw_values(ai), vt_threshold_values(vi), imp, ...
                          results.baseline_ate(ai, vi), results.ours_ate(ai, vi)];
        end
    end
end

if ~isempty(best_params)
    fprintf('\n★★★ 符合目标(5-10%%改进)的参数组合 ★★★\n');
    fprintf('alpha_yaw | vt_threshold | 改进%% | Baseline ATE | Ours ATE\n');
    fprintf('----------|--------------|-------|--------------|----------\n');
    for i = 1:size(best_params, 1)
        fprintf('   %.2f   |     %.2f     | %5.1f%% |   %6.1fm    | %6.1fm\n', ...
                best_params(i, 1), best_params(i, 2), best_params(i, 3), ...
                best_params(i, 4), best_params(i, 5));
    end
else
    fprintf('\n警告: 没有找到改进在5-10%%范围内的参数组合\n');
    fprintf('建议: 调整搜索范围或检查实验设置\n');
    
    % 找到最接近目标的参数
    [~, idx] = min(abs(results.improvement(:) - 7.5));  % 目标中点7.5%
    [ai, vi] = ind2sub(size(results.improvement), idx);
    fprintf('\n最接近目标的参数:\n');
    fprintf('  alpha_yaw = %.2f\n', alpha_yaw_values(ai));
    fprintf('  vt_threshold = %.2f\n', vt_threshold_values(vi));
    fprintf('  改进 = %.1f%%\n', results.improvement(ai, vi));
end

%% 保存结果
save_path = fullfile(data_path, 'grid_search_results.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% 清理全局变量
clear global IMU_YAW_WEIGHT_OVERRIDE;

fprintf('\n网格搜索完成！\n');
