%% QUT Carpark Real-World SLAM Test Script
%  测试HART+Transformer在真实停车场场景的性能
%  纯视觉SLAM（无IMU数据）

clear all; close all; clc;

%% 1. 添加路径
fprintf('========== QUT Carpark Real-World SLAM Test ==========\n');
fprintf('数据集: QUT Carpark (真实场景)\n');
fprintf('[1/7] 添加依赖路径...\n');

% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
savepath;

%% 2. 初始化全局变量
fprintf('[2/7] 初始化全局变量...\n');
global PREV_VT_ID; PREV_VT_ID = -1;
global VT_TEMPLATES; VT_TEMPLATES = [];
global VT_ID_COUNT; VT_ID_COUNT = 0;
global NUM_VT; NUM_VT = 0;
global VT; VT = [];
global YAW_HEIGHT_HDC; YAW_HEIGHT_HDC = zeros(36, 36);
global GRIDCELLS; GRIDCELLS = zeros(36, 36, 36);
global EXPERIENCES; EXPERIENCES = [];
global NUM_EXPS; NUM_EXPS = 0;
global CUR_EXP_ID; CUR_EXP_ID = 0;
global PREV_TRANS_V; PREV_TRANS_V = 0;
global PREV_YAW_ROT_V; PREV_YAW_ROT_V = 0;
global PREV_HEIGHT_V; PREV_HEIGHT_V = 0;
global DEGREE_TO_RADIAN; DEGREE_TO_RADIAN = pi / 180;
global RADIAN_TO_DEGREE; RADIAN_TO_DEGREE = 180 / pi;
global YAW_HEIGHT_HDC_Y_TH_SIZE; YAW_HEIGHT_HDC_Y_TH_SIZE = 2*pi/36;

%% 3. 初始化各模块参数
fprintf('[3/7] 初始化模块参数...\n');

% 视觉里程计初始化
visual_odo_initial( ...
    'ODO_IMG_TRANS_Y_RANGE', 31:90, ...
    'ODO_IMG_TRANS_X_RANGE', 16:145, ...
    'ODO_IMG_HEIGHT_V_Y_RANGE', 11:110, ...
    'ODO_IMG_HEIGHT_V_X_RANGE', 11:150, ...
    'ODO_IMG_YAW_ROT_Y_RANGE', 31:90, ...
    'ODO_IMG_YAW_ROT_X_RANGE', 16:145, ...
    'ODO_IMG_TRANS_RESIZE_RANGE', [60, 130], ...
    'ODO_IMG_YAW_ROT_RESIZE_RANGE', [60, 130], ...
    'ODO_IMG_HEIGHT_V_RESIZE_RANGE', [100, 140], ...
    'ODO_TRANS_V_SCALE', 24, ...
    'ODO_YAW_ROT_V_SCALE', 1, ...
    'ODO_HEIGHT_V_SCALE', 20, ...
    'MAX_TRANS_V_THRESHOLD', 0.5, ...
    'MAX_YAW_ROT_V_THRESHOLD', 2.5, ...
    'MAX_HEIGHT_V_THRESHOLD', 0.45, ...
    'ODO_SHIFT_MATCH_HORI', 26, ...
    'ODO_SHIFT_MATCH_VERT', 20, ...
    'FOV_HORI_DEGREE', 81.5, ...
    'FOV_VERT_DEGREE', 50, ...
    'ODO_STEP', 1);

% 偏航-高度头部朝向细胞初始化（必须在VT之前）
yaw_height_hdc_initial( ...
    'YAW_HEIGHT_HDC_Y_DIM', 36, ...
    'YAW_HEIGHT_HDC_H_DIM', 36, ...
    'YAW_HEIGHT_HDC_EXCIT_Y_DIM', 8, ...
    'YAW_HEIGHT_HDC_EXCIT_H_DIM', 8, ...
    'YAW_HEIGHT_HDC_INHIB_Y_DIM', 5, ...
    'YAW_HEIGHT_HDC_INHIB_H_DIM', 5, ...
    'YAW_HEIGHT_HDC_EXCIT_Y_VAR', 1.9, ...
    'YAW_HEIGHT_HDC_EXCIT_H_VAR', 1.9, ...
    'YAW_HEIGHT_HDC_INHIB_Y_VAR', 3.0, ...
    'YAW_HEIGHT_HDC_INHIB_H_VAR', 3.0, ...
    'YAW_HEIGHT_HDC_GLOBAL_INHIB', 0.0002, ...
    'YAW_HEIGHT_HDC_VT_INJECT_ENERGY', 0.001, ...
    'YAW_ROT_V_SCALE', 1, ...
    'HEIGHT_V_SCALE', 1, ...
    'YAW_HEIGHT_HDC_PACKET_SIZE', 5);

% VT初始化 - 使用HART+Transformer Plan B最优配置
vt_image_initial('*.png', ...
    'VT_MATCH_THRESHOLD', 0.06, ...
    'VT_IMG_CROP_Y_RANGE', 1:120, ...
    'VT_IMG_CROP_X_RANGE', 1:160, ...
    'VT_IMG_RESIZE_X_RANGE', 16, ...
    'VT_IMG_RESIZE_Y_RANGE', 12, ...
    'VT_IMG_X_SHIFT', 5, ...
    'VT_IMG_Y_SHIFT', 3, ...
    'VT_GLOBAL_DECAY', 0.1, ...
    'VT_ACTIVE_DECAY', 2.0, ...
    'PATCH_SIZE_Y_K', 5, ...
    'PATCH_SIZE_X_K', 5, ...
    'VT_PANORAMIC', 0, ...
    'VT_STEP', 1);

fprintf('  VT方法: HART+Transformer Plan B最优 (阈值: %.3f，已验证)\n', 0.06);

% 3D网格细胞初始化
gc_initial( ...
    'GC_X_DIM', 36, ...
    'GC_Y_DIM', 36, ...
    'GC_Z_DIM', 36, ...
    'GC_EXCIT_X_DIM', 7, ...
    'GC_EXCIT_Y_DIM', 7, ...
    'GC_EXCIT_Z_DIM', 7, ...
    'GC_INHIB_X_DIM', 5, ...
    'GC_INHIB_Y_DIM', 5, ...
    'GC_INHIB_Z_DIM', 5, ...
    'GC_EXCIT_X_VAR', 1.5, ...
    'GC_EXCIT_Y_VAR', 1.5, ...
    'GC_EXCIT_Z_VAR', 1.5, ...
    'GC_INHIB_X_VAR', 2, ...
    'GC_INHIB_Y_VAR', 2, ...
    'GC_INHIB_Z_VAR', 2, ...
    'GC_GLOBAL_INHIB', 0.0002, ...
    'GC_VT_INJECT_ENERGY', 0.1, ...
    'GC_EXCIT_WRAP', 0, ...
    'GC_INHIB_WRAP', 0, ...
    'GC_HORI_TRANS_V_SCALE', 0.8, ...
    'GC_VERT_TRANS_V_SCALE', 0.8, ...
    'GC_PACKET_SIZE', 4);

% 经验地图初始化
exp_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...
    'EXP_LOOPS', 1, ...
    'EXP_CORRECTION', 0.5);

fprintf('经验地图阈值: DELTA_EXP_GC_HDC_THRESHOLD = 15\n');

%% 4. 读取真实场景图像数据
fprintf('[4/7] 读取QUT Carpark真实场景数据...\n');
data_path = fullfile(rootDir, 'DATASETS/01_NeuroSLAM_Datasets/03_QUTCarparkData');

if ~exist(data_path, 'dir')
    error('数据路径不存在: %s', data_path);
end

% 获取所有图像文件
image_files = dir(fullfile(data_path, '*.png'));
num_frames = length(image_files);
fprintf('  找到 %d 帧图像\n', num_frames);

%% 5. 运行纯视觉SLAM
fprintf('[5/7] 运行纯视觉SLAM (无IMU)...\n');
fprintf('  使用HART+Transformer特征提取\n\n');

% 初始化轨迹记录
odo_trajectory = zeros(num_frames, 3);  % [x, y, theta]
exp_trajectory = zeros(num_frames, 3);  % 经验地图轨迹

% 初始化位姿
current_x = 0;
current_y = 0;
current_theta = 0;

% 进度显示
fprintf('处理进度: ');
progress_step = max(1, floor(num_frames / 20));

tic;
for i = 1:num_frames
    % 读取图像
    img_path = fullfile(data_path, image_files(i).name);
    img = imread(img_path);
    
    % 如果是彩色图像，转换为灰度
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    % 视觉里程计
    [vtrans, vrot, vheight] = visual_odo(img, i);
    
    % VT匹配
    [vt_id, ~] = visual_template(img, i);
    
    % 更新位姿（基于视觉里程计）
    current_theta = current_theta + vrot;
    current_x = current_x + vtrans * cos(current_theta);
    current_y = current_y + vtrans * sin(current_theta);
    
    % 记录视觉里程计轨迹
    odo_trajectory(i, :) = [current_x, current_y, current_theta];
    
    % 偏航-高度HDC更新
    yaw_height_hdc_iteration(vrot, vheight, vt_id);
    
    % 网格细胞更新
    gridcells_iteration(vtrans, vrot, vheight);
    
    % 经验地图更新
    [exp_id, ~] = experience_map_iteration(vt_id, vtrans, vrot, vheight);
    
    % 记录经验地图轨迹
    if exp_id > 0 && exp_id <= length(EXPERIENCES)
        exp_trajectory(i, :) = [EXPERIENCES(exp_id).x_m, ...
                                EXPERIENCES(exp_id).y_m, ...
                                EXPERIENCES(exp_id).th_rad];
    else
        exp_trajectory(i, :) = odo_trajectory(i, :);
    end
    
    % 进度显示
    if mod(i, progress_step) == 0
        fprintf('%d%% ', round(100*i/num_frames));
    end
end
fprintf('完成!\n');
processing_time = toc;

fprintf('\n处理完成:\n');
fprintf('  总帧数: %d\n', num_frames);
fprintf('  VT数量: %d\n', NUM_VT);
fprintf('  经验节点: %d\n', NUM_EXPS);
fprintf('  处理时间: %.1f秒 (%.2f fps)\n', processing_time, num_frames/processing_time);

%% 6. 可视化结果
fprintf('[6/7] 生成可视化结果...\n');

% 准备结果保存目录
result_path = fullfile(data_path, 'slam_results');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

% 绘制SLAM轨迹 - 专业设计
fig = figure('Position', [50, 50, 1400, 900]);
set(fig, 'Color', [0.96 0.96 0.98]);  % 专业灰蓝背景

% 视觉里程计轨迹 - 专业风格
subplot(2, 2, 1);
hold on;

% 专业网格
grid on;
set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
set(gca, 'Box', 'on', 'LineWidth', 1.2);
set(gca, 'Color', 'w');

% 专业轨迹线 - 深蓝
plot(odo_trajectory(:,1), odo_trajectory(:,2), '-', 'Color', [0.12 0.47 0.71], ...
    'LineWidth', 3.0, 'DisplayName', 'Visual Odometry');

% 专业起点/终点标记
scatter(odo_trajectory(1,1), odo_trajectory(1,2), 140, [0.17 0.63 0.17], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'Start', 'Marker', '^');
scatter(odo_trajectory(end,1), odo_trajectory(end,2), 140, [0.84 0.15 0.16], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'End', 'Marker', 'v');

axis equal;
xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
title('Visual Odometry Path', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

leg = legend('Location', 'northeast', 'FontSize', 11);
set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);

set(gca, 'FontSize', 11, 'FontName', 'Arial');
ax = gca;
ax.XAxis.Color = [0.2 0.2 0.2];
ax.YAxis.Color = [0.2 0.2 0.2];
ax.LineWidth = 1.2;

% 经验地图轨迹 - 专业红色
subplot(2, 2, 2);
hold on;

grid on;
set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
set(gca, 'Box', 'on', 'LineWidth', 1.2);
set(gca, 'Color', 'w');

% 专业轨迹线 - 深红色
plot(exp_trajectory(:,1), exp_trajectory(:,2), '-', 'Color', [0.84 0.15 0.16], ...
    'LineWidth', 3.0, 'DisplayName', 'Experience Map');

% 专业标记
scatter(exp_trajectory(1,1), exp_trajectory(1,2), 140, [0.17 0.63 0.17], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'Start', 'Marker', '^');
scatter(exp_trajectory(end,1), exp_trajectory(end,2), 140, [0.84 0.15 0.16], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'End', 'Marker', 'v');

axis equal;
xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
title('Experience Map Path', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

leg = legend('Location', 'northwest', 'FontSize', 11);
set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);

set(gca, 'FontSize', 11, 'FontName', 'Arial');
ax = gca;
ax.XAxis.Color = [0.2 0.2 0.2];
ax.YAxis.Color = [0.2 0.2 0.2];
ax.LineWidth = 1.2;

% 轨迹对比 - 专业重叠
subplot(2, 2, 3);
hold on;

grid on;
set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
set(gca, 'Box', 'on', 'LineWidth', 1.2);
set(gca, 'Color', 'w');

% 专业轨迹对比
plot(odo_trajectory(:,1), odo_trajectory(:,2), '-.', 'Color', [0.12 0.47 0.71], ...
    'LineWidth', 2.5, 'DisplayName', 'Visual Odometry');  % 深蓝
plot(exp_trajectory(:,1), exp_trajectory(:,2), '-', 'Color', [0.84 0.15 0.16], ...
    'LineWidth', 2.8, 'DisplayName', 'Experience Map');  % 深红

% 专业标记
scatter(odo_trajectory(1,1), odo_trajectory(1,2), 150, [0.17 0.63 0.17], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'Start', 'Marker', '^');
scatter(odo_trajectory(end,1), odo_trajectory(end,2), 150, [0.84 0.15 0.16], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, 'DisplayName', 'End', 'Marker', 'v');

axis equal;
xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
title('Real-World Path Comparison', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

leg = legend('Location', 'southwest', 'FontSize', 11);
set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);

set(gca, 'FontSize', 11, 'FontName', 'Arial');
ax = gca;
ax.XAxis.Color = [0.2 0.2 0.2];
ax.YAxis.Color = [0.2 0.2 0.2];
ax.LineWidth = 1.2;

% 统计信息 - 现代信息面板
subplot(2, 2, 4);
axis off;

% 标题
text(0.5, 0.95, 'Real-World SLAM Performance', ...
    'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'Color', [0.15 0.15 0.15]);

% 数据集信息区域
y_pos = 0.85;
text(0.05, y_pos, 'Dataset:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2 0.4 0.6]);
y_pos = y_pos - 0.08;
text(0.1, y_pos, sprintf('QUT Carpark - %d frames', num_frames), 'FontSize', 11);
y_pos = y_pos - 0.06;
text(0.1, y_pos, sprintf('Processing: %.1f s (%.2f fps)', processing_time, num_frames/processing_time), 'FontSize', 11);

% SLAM统计信息
y_pos = y_pos - 0.12;
text(0.05, y_pos, 'SLAM Statistics:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.6 0.3 0.5]);
y_pos = y_pos - 0.08;
text(0.1, y_pos, sprintf('Visual Templates: %d', NUM_VT), 'FontSize', 11);
y_pos = y_pos - 0.06;
text(0.1, y_pos, sprintf('Experience Nodes: %d', NUM_EXPS), 'FontSize', 11);
y_pos = y_pos - 0.06;
text(0.1, y_pos, sprintf('VT Threshold: %.3f', 0.06), 'FontSize', 11);

% 特征提取信息
y_pos = y_pos - 0.12;
text(0.05, y_pos, 'Feature Method:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2 0.5 0.3]);
y_pos = y_pos - 0.08;
text(0.1, y_pos, 'HART+Transformer', 'FontSize', 11);
y_pos = y_pos - 0.06;
text(0.1, y_pos, 'Plan B Configuration', 'FontSize', 11);

% 轨迹长度
y_pos = y_pos - 0.12;
text(0.05, y_pos, 'Path Length:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.5 0.3 0.2]);
y_pos = y_pos - 0.08;
odo_length = sum(sqrt(diff(odo_trajectory(:,1)).^2 + diff(odo_trajectory(:,2)).^2));
exp_length = sum(sqrt(diff(exp_trajectory(:,1)).^2 + diff(exp_trajectory(:,2)).^2));
text(0.1, y_pos, sprintf('Visual Odo: %.2f m', odo_length), 'FontSize', 11);
y_pos = y_pos - 0.06;
text(0.1, y_pos, sprintf('Exp Map: %.2f m', exp_length), 'FontSize', 11);

% 保存图表
saveas(gcf, fullfile(result_path, 'carpark_slam_trajectories.png'));
fprintf('  ✓ 轨迹图已保存\n');

%% 7. 保存结果
fprintf('[7/7] 保存结果...\n');

% 保存轨迹数据
save(fullfile(result_path, 'carpark_slam_results.mat'), ...
    'odo_trajectory', 'exp_trajectory', 'num_frames', 'processing_time');

% 保存轨迹文本文件
dlmwrite(fullfile(result_path, 'odo_trajectory.txt'), odo_trajectory, 'precision', 6);
dlmwrite(fullfile(result_path, 'exp_trajectory.txt'), exp_trajectory, 'precision', 6);

% 生成报告
report_file = fullfile(result_path, 'carpark_slam_report.txt');
fid = fopen(report_file, 'w');

fprintf(fid, '========================================\n');
fprintf(fid, 'QUT Carpark Real-World SLAM Test Report\n');
fprintf(fid, '========================================\n\n');

fprintf(fid, 'Dataset Information:\n');
fprintf(fid, '  Location: QUT Carpark (Real-World)\n');
fprintf(fid, '  Total Frames: %d\n', num_frames);
fprintf(fid, '  Image Size: %dx%d\n', size(img, 1), size(img, 2));
fprintf(fid, '\n');

fprintf(fid, 'Processing Performance:\n');
fprintf(fid, '  Total Time: %.2f seconds\n', processing_time);
fprintf(fid, '  FPS: %.2f\n', num_frames/processing_time);
fprintf(fid, '  Average Time per Frame: %.2f ms\n', 1000*processing_time/num_frames);
fprintf(fid, '\n');

fprintf(fid, 'SLAM Statistics:\n');
fprintf(fid, '  Visual Templates: %d\n', NUM_VT);
fprintf(fid, '  Experience Nodes: %d\n', NUM_EXPS);
fprintf(fid, '  VT/Frame Ratio: %.4f\n', NUM_VT/num_frames);
fprintf(fid, '  Exp/Frame Ratio: %.4f\n', NUM_EXPS/num_frames);
fprintf(fid, '\n');

fprintf(fid, 'Feature Extraction:\n');
fprintf(fid, '  Method: HART+Transformer\n');
fprintf(fid, '  Configuration: Plan B Optimal\n');
fprintf(fid, '  VT Threshold: %.3f\n', 0.06);
fprintf(fid, '  Global Modulation: 0.15\n');
fprintf(fid, '  Fusion Weights: [0.20, 0.30, 0.30, 0.20]\n');
fprintf(fid, '\n');

fprintf(fid, 'Trajectory Statistics:\n');
odo_length = sum(sqrt(diff(odo_trajectory(:,1)).^2 + diff(odo_trajectory(:,2)).^2));
exp_length = sum(sqrt(diff(exp_trajectory(:,1)).^2 + diff(exp_trajectory(:,2)).^2));
fprintf(fid, '  Visual Odometry Path Length: %.2f m\n', odo_length);
fprintf(fid, '  Experience Map Path Length: %.2f m\n', exp_length);
fprintf(fid, '  Drift (Difference): %.2f m\n', abs(odo_length - exp_length));
fprintf(fid, '\n');

fclose(fid);

fprintf('\n');
fprintf('========================================\n');
fprintf('✅ QUT Carpark SLAM测试完成！\n');
fprintf('========================================\n');
fprintf('结果保存在: %s\n', result_path);
fprintf('  - carpark_slam_trajectories.png (轨迹图)\n');
fprintf('  - carpark_slam_results.mat (MATLAB数据)\n');
fprintf('  - carpark_slam_report.txt (详细报告)\n');
fprintf('  - odo_trajectory.txt, exp_trajectory.txt (轨迹数据)\n');
fprintf('========================================\n\n');
