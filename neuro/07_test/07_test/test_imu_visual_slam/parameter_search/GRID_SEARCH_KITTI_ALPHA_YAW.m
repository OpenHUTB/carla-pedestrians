%% KITTI网格搜索：IMU融合alpha_yaw参数优化
%  目标：找到KITTI数据集的最优IMU融合参数
%
%  搜索参数：
%    - alpha_yaw: IMU偏航权重 [0.000, 0.005, 0.010, 0.012, 0.015, 0.020, 0.030]
%
%  数据路径：
%    - IMU数据: E:\Neuro_end\neuro\data\KITTI_07\KITTI_07\fusion_pose.txt
%    - 图像: E:\Neuro_end\neuro\data\KITTI_07\KITTI_07\image_0(1)\image_0\*.png
%    - Ground Truth: E:\Neuro_end\neuro\data\KITTI_07\KITTI_07\poses.txt
%
%  运行方式：
%    cd('E:\Neuro_end\neuro');
%    addpath('07_test/07_test/test_imu_visual_slam/parameter_search');
%    GRID_SEARCH_KITTI_ALPHA_YAW

clear; close all; clc;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     KITTI网格搜索：IMU融合参数优化                           ║\n');
fprintf('║     数据集：KITTI_07 (1101帧，694米轨迹)                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 参数网格
alpha_yaw_values = [0.000, 0.005, 0.010, 0.012, 0.015, 0.020, 0.030];

% 测试帧数（KITTI只有1101帧）
FAST_FRAMES = 1100;

%% 初始化结果存储
num_alpha = length(alpha_yaw_values);
results = struct();
results.alpha_yaw = alpha_yaw_values;
results.baseline_ate = zeros(num_alpha, 1);
results.ours_ate = zeros(num_alpha, 1);
results.improvement = zeros(num_alpha, 1);
results.baseline_vt = zeros(num_alpha, 1);
results.ours_vt = zeros(num_alpha, 1);
results.baseline_exp = zeros(num_alpha, 1);
results.ours_exp = zeros(num_alpha, 1);

%% 设置路径
currentDir = fileparts(mfilename('fullpath'));
testDir = fileparts(currentDir);
rootDir = fileparts(fileparts(fileparts(fileparts(currentDir))));

fprintf('[1/3] 添加依赖路径...\n');
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
toolkit_path1 = fullfile(rootDir, '05_tookit', 'process_visual_data', 'process_images_data');
toolkit_path2 = fullfile(rootDir, '05_tookit', '05_tookit', 'process_visual_data', 'process_images_data');
if exist(toolkit_path1, 'dir')
    addpath(toolkit_path1);
elseif exist(toolkit_path2, 'dir')
    addpath(toolkit_path2);
end

%% 读取KITTI数据
fprintf('[2/3] 读取KITTI数据...\n');
dataset_name = 'KITTI_07';
data_path = fullfile(rootDir, 'data', dataset_name, dataset_name);

if ~exist(data_path, 'dir')
    error('数据路径不存在: %s', data_path);
end
fprintf('  数据路径: %s\n', data_path);

% 读取IMU数据（从fusion_pose.txt）
fusion_file = fullfile(data_path, 'fusion_pose.txt');
if ~exist(fusion_file, 'file')
    error('fusion_pose.txt不存在: %s', fusion_file);
end

% 手动解析fusion_pose.txt（跳过注释行）
fid = fopen(fusion_file, 'r');
fusion_raw = [];
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line) || isempty(line)
        continue;
    end
    line_trimmed = strtrim(line);
    if isempty(line_trimmed) || line_trimmed(1) == '%' || line_trimmed(1) == '#'
        continue;
    end
    try
        data_row = str2num(line);
        if ~isempty(data_row)
            fusion_raw = [fusion_raw; data_row];
        end
    catch
        continue;
    end
end
fclose(fid);

% 创建imu_data结构
imu_data = struct();
imu_data.gyro = fusion_raw(:, 14:16);
imu_data.accel = fusion_raw(:, 11:13);
imu_data.timestamp = fusion_raw(:, 1);
imu_data.count = size(fusion_raw, 1);
fprintf('  IMU数据: %d 帧\n', imu_data.count);

% 读取Ground Truth（从poses.txt）
poses_file = fullfile(data_path, 'poses.txt');
poses_raw = load(poses_file);
gt_pos_raw = zeros(size(poses_raw, 1), 3);
for i = 1:size(poses_raw, 1)
    gt_pos_raw(i, :) = poses_raw(i, [4, 8, 12]);
end
% 坐标转换：KITTI (X前,Y高,Z左) → 标准世界系 (X前,Y左,Z高)
gt_pos = zeros(size(gt_pos_raw));
gt_pos(:, 1) = gt_pos_raw(:, 1);
gt_pos(:, 2) = gt_pos_raw(:, 3);
gt_pos(:, 3) = gt_pos_raw(:, 2);
gt_data = struct();
gt_data.pos = gt_pos;
fprintf('  Ground Truth: %d 帧\n', size(gt_pos, 1));

% 读取图像列表
img_path = '';
img_candidates = {
    fullfile(data_path, 'image_0'), ...
    fullfile(data_path, 'image_0', 'image_0'), ...
    fullfile(data_path, 'image_0(1)'), ...
    fullfile(data_path, 'image_0(1)', 'image_0')
};
for ci = 1:numel(img_candidates)
    p = img_candidates{ci};
    if exist(p, 'dir')
        tmp = dir(fullfile(p, '*.png'));
        if ~isempty(tmp)
            img_path = p;
            break;
        end
    end
end
if isempty(img_path)
    error('未找到图像文件');
end
img_files = dir(fullfile(img_path, '*.png'));
[~, idx] = sort({img_files.name});
img_files = img_files(idx);
fprintf('  图像数量: %d 张\n', length(img_files));

num_frames = min([length(img_files), imu_data.count, FAST_FRAMES]);
fprintf('  测试帧数: %d\n\n', num_frames);

%% 开始网格搜索
fprintf('[3/3] 开始网格搜索...\n');
total_tests = num_alpha * 2;  % Baseline + Ours
test_count = 0;

for ai = 1:num_alpha
    alpha_yaw = alpha_yaw_values(ai);
    
    fprintf('\n═══════════════════════════════════════════════════════════════\n');
    fprintf('测试 %d/%d: alpha_yaw=%.3f\n', ai, num_alpha, alpha_yaw);
    fprintf('═══════════════════════════════════════════════════════════════\n');
    
    % 设置全局参数
    global IMU_YAW_WEIGHT_OVERRIDE;
    IMU_YAW_WEIGHT_OVERRIDE = alpha_yaw;
    
    try
        % ========== 运行Baseline (alpha_yaw=0) ==========
        fprintf('\n[Baseline] 运行纯视觉SLAM...\n');
        clear functions;
        clear global PREV_VT_ID VT NUM_VT EXPERIENCES NUM_EXPS CUR_EXP_ID;
        clear global YAW_HEIGHT_HDC GRIDCELLS POSECELL;
        
        IMU_YAW_WEIGHT_OVERRIDE = 0.0;  % Baseline禁用IMU
        
        [bl_ate, bl_vt_count, bl_exp_count, bl_traj] = run_kitti_single_experiment(...
            data_path, img_path, img_files, imu_data, gt_data, num_frames, rootDir);
        
        % ========== 运行Ours (使用当前alpha_yaw) ==========
        fprintf('\n[Ours] 运行IMU融合SLAM (alpha_yaw=%.3f)...\n', alpha_yaw);
        clear functions;
        clear global PREV_VT_ID VT NUM_VT EXPERIENCES NUM_EXPS CUR_EXP_ID;
        clear global YAW_HEIGHT_HDC GRIDCELLS POSECELL;
        
        IMU_YAW_WEIGHT_OVERRIDE = alpha_yaw;
        
        [ours_ate, ours_vt_count, ours_exp_count, ours_traj] = run_kitti_single_experiment(...
            data_path, img_path, img_files, imu_data, gt_data, num_frames, rootDir);
        
        % 计算改进
        improvement = (bl_ate - ours_ate) / bl_ate * 100;
        
        % 保存结果
        results.baseline_ate(ai) = bl_ate;
        results.ours_ate(ai) = ours_ate;
        results.improvement(ai) = improvement;
        results.baseline_vt(ai) = bl_vt_count;
        results.ours_vt(ai) = ours_vt_count;
        results.baseline_exp(ai) = bl_exp_count;
        results.ours_exp(ai) = ours_exp_count;
        
        fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        fprintf('结果汇总 (alpha_yaw=%.3f):\n', alpha_yaw);
        fprintf('  Baseline ATE: %.2f m | VT: %d | Exp: %d\n', bl_ate, bl_vt_count, bl_exp_count);
        fprintf('  Ours ATE:     %.2f m | VT: %d | Exp: %d\n', ours_ate, ours_vt_count, ours_exp_count);
        if improvement > 0
            fprintf('  改进: +%.2f%% ✓\n', improvement);
        else
            fprintf('  退化: %.2f%% ✗\n', improvement);
        end
        fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
        
    catch ME
        warning('测试失败 (alpha_yaw=%.3f): %s', alpha_yaw, ME.message);
        results.baseline_ate(ai) = NaN;
        results.ours_ate(ai) = NaN;
        results.improvement(ai) = NaN;
    end
end

%% 显示结果汇总
fprintf('\n\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                KITTI网格搜索结果汇总                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

fprintf('alpha_yaw | Baseline ATE | Ours ATE | 改进%%   | Baseline VT | Ours VT\n');
fprintf('----------|--------------|----------|---------|-------------|--------\n');
for ai = 1:num_alpha
    fprintf('  %.3f   |   %6.2f m   | %6.2f m | %+6.2f%% |     %3d     |   %3d\n', ...
            alpha_yaw_values(ai), results.baseline_ate(ai), results.ours_ate(ai), ...
            results.improvement(ai), results.baseline_vt(ai), results.ours_vt(ai));
end

%% 找到最佳参数
[max_improvement, best_idx] = max(results.improvement);
best_alpha_yaw = alpha_yaw_values(best_idx);

fprintf('\n★★★ 最佳参数 ★★★\n');
fprintf('  alpha_yaw = %.3f\n', best_alpha_yaw);
fprintf('  改进 = %.2f%%\n', max_improvement);
fprintf('  Baseline ATE = %.2f m\n', results.baseline_ate(best_idx));
fprintf('  Ours ATE = %.2f m\n', results.ours_ate(best_idx));

%% 保存结果
save_path = fullfile(data_path, 'grid_search_results_kitti.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

%% 绘制结果图
figure('Position', [100, 100, 1200, 400]);

% 子图1: ATE对比
subplot(1, 3, 1);
plot(alpha_yaw_values, results.baseline_ate, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
plot(alpha_yaw_values, results.ours_ate, 'r-s', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('alpha\_yaw');
ylabel('ATE (m)');
title('KITTI: ATE vs alpha\_yaw');
legend('Baseline', 'Ours', 'Location', 'best');
grid on;

% 子图2: 改进百分比
subplot(1, 3, 2);
bar(alpha_yaw_values, results.improvement);
xlabel('alpha\_yaw');
ylabel('改进 (%)');
title('KITTI: 改进百分比');
grid on;
hold on;
plot([alpha_yaw_values(1), alpha_yaw_values(end)], [0, 0], 'k--', 'LineWidth', 1);

% 子图3: VT数量对比
subplot(1, 3, 3);
plot(alpha_yaw_values, results.baseline_vt, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
plot(alpha_yaw_values, results.ours_vt, 'r-s', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('alpha\_yaw');
ylabel('VT Count');
title('KITTI: VT数量');
legend('Baseline', 'Ours', 'Location', 'best');
grid on;

% 保存图表
saveas(gcf, fullfile(data_path, 'grid_search_kitti_results.png'));
fprintf('结果图表已保存\n');

fprintf('\n✅ KITTI网格搜索完成！\n');
