%% IMU-Visual Fusion SLAM Test Script
%  NeuroSLAM System Copyright (C) 2018-2019
%  IMU-Visual Integration Test (2024)
%
%  本脚本测试IMU-视觉融合的NeuroSLAM系统
%  对比纯视觉SLAM和IMU-视觉融合SLAM的性能

% 【重要】先检查是否为快速测试模式，避免被clear all清除
global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
fast_test_active = false;
fast_test_num_frames = 5000;  % 默认完整测试
dataset_name = 'Town01Data_IMU_Fusion';  % 默认Town01
try
    % 尝试读取全局变量
    if ~isempty(FAST_TEST_MODE) && FAST_TEST_MODE
        fast_test_active = true;
        if ~isempty(FAST_TEST_FRAMES)
            fast_test_num_frames = FAST_TEST_FRAMES;
        end
        fprintf('⚡ 检测到快速测试模式：%d帧\n', fast_test_num_frames);
    end
    % 检查是否设置了数据集名称
    if ~isempty(DATASET_NAME)
        dataset_name = DATASET_NAME;
        fprintf('📍 使用数据集: %s\n', dataset_name);
    end
catch
    % 如果读取失败，说明没有设置快速测试模式
    fast_test_active = false;
end

% 使用clearvars而不是clear all，保留快速测试标志和数据集名称
clearvars -except fast_test_active fast_test_num_frames dataset_name;
close all; clc;

% 恢复快速测试模式全局变量和数据集名称
if fast_test_active
    global FAST_TEST_MODE FAST_TEST_FRAMES;
    FAST_TEST_MODE = true;
    FAST_TEST_FRAMES = fast_test_num_frames;
end
global DATASET_NAME;
DATASET_NAME = dataset_name;

%% 1. 添加路径
fprintf('========== IMU-Visual Fusion SLAM Test ==========\n');
fprintf('数据集: %s\n', dataset_name);
fprintf('[1/9] 添加依赖路径...\n');
% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(rootDir, '09_vestibular'));
savepath;

%% 2. 初始化全局变量
fprintf('[2/9] 初始化全局变量...\n');
global PREV_VT_ID; PREV_VT_ID = -1;
global VT_TEMPLATES; VT_TEMPLATES = [];
global VT_ID_COUNT; VT_ID_COUNT = 0;
global NUM_VT; NUM_VT = 0;  % 增强VT方法使用NUM_VT
global VT; VT = [];  % VT数组
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
global YAW_HEIGHT_HDC_Y_TH_SIZE; YAW_HEIGHT_HDC_Y_TH_SIZE = 2*pi/36;  % 每个单元的角度大小

%% 3. 初始化各模块参数
fprintf('[3/9] 初始化模块参数...\n');

% 视觉里程计初始化（调整尺度参数以匹配IMU-Fusion）
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
    'ODO_TRANS_V_SCALE', 24, ...  % 从30降到24 (30 * 1630/2032 ≈ 24)
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

% 视觉模板初始化（HART+Transformer Plan B最优配置）
vt_image_initial('*.png', ...
    'VT_MATCH_THRESHOLD', 0.06, ...  % Plan B验证的最优阈值
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

fprintf('  VT方法: HART+Transformer Plan B最优 (阈值: %.3f，权重0.15，已验证)\n', 0.06);

% 偏航-高度头部朝向细胞初始化
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

% 3D网格细胞初始化（调整尺度以匹配IMU-Fusion）
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
    'GC_HORI_TRANS_V_SCALE', 0.8, ...  % 从1降到0.8 (1 * 1630/2032 ≈ 0.8)
    'GC_VERT_TRANS_V_SCALE', 0.8, ...  % 同步调整
    'GC_PACKET_SIZE', 4);

% 经验地图初始化（降低阈值以增加经验节点创建）
exp_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...  % 降低阈值从40到15
    'EXP_LOOPS', 1, ...
    'EXP_CORRECTION', 0.5);

fprintf('经验地图阈值: DELTA_EXP_GC_HDC_THRESHOLD = 15 (降低以创建更多经验节点)\n');

%% 4. 读取IMU-视觉融合数据
fprintf('[4/9] 读取IMU-视觉融合数据 (%s)...\n', dataset_name);
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets', dataset_name);

if ~exist(data_path, 'dir')
    error('数据路径不存在: %s\n请先运行Python脚本采集数据', data_path);
end

% 读取IMU数据
imu_data = read_imu_data(data_path);

% 读取融合位姿数据
fusion_data = read_fusion_pose(data_path);

% 读取Ground Truth数据（如果存在）
gt_file = fullfile(data_path, 'ground_truth.txt');
if exist(gt_file, 'file')
    gt_data = read_ground_truth(gt_file);
    has_ground_truth = true;
    fprintf('✓ 已加载Ground Truth数据\n');
else
    has_ground_truth = false;
    warning('⚠️  未找到Ground Truth文件: %s', gt_file);
    fprintf('   Ground Truth对比功能将不可用\n');
    fprintf('   建议：重新运行Python采集脚本生成ground_truth.txt\n');
    fprintf('   命令：cd ../../00_collect_data && python IMU_Vision_Fusion_EKF.py\n\n');
end

% 获取图像文件列表
img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到图像文件');
end
fprintf('找到 %d 张图像\n', length(img_files));

% 数据一致性检查
if length(img_files) ~= size(fusion_data.pos, 1)
    warning('⚠️  图像数量(%d) 与融合位姿数量(%d) 不匹配！', ...
        length(img_files), size(fusion_data.pos, 1));
    fprintf('   可能原因：\n');
    fprintf('   1. 数据采集未完成（按Ctrl+C中断）\n');
    fprintf('   2. fusion_pose.txt文件损坏\n');
    fprintf('   3. 重新运行Python采集脚本：python IMU_Vision_Fusion_EKF.py\n');
    fprintf('   将使用前 %d 帧进行处理\n', ...
        min(length(img_files), size(fusion_data.pos, 1)));
end

%% 5. 运行IMU-视觉融合SLAM
fprintf('[5/9] 开始运行IMU-Visual Fusion SLAM...\n');

% 支持快速测试模式（通过全局变量控制）
global FAST_TEST_MODE FAST_TEST_FRAMES;
if ~isempty(FAST_TEST_MODE) && FAST_TEST_MODE && ~isempty(FAST_TEST_FRAMES)
    num_frames = min([length(img_files), length(fusion_data.timestamp), FAST_TEST_FRAMES]);
    fprintf('⚡ 快速测试模式：处理 %d 帧（完整数据集有 %d 帧）\n', ...
        num_frames, min(length(img_files), length(fusion_data.timestamp)));
else
    num_frames = min(length(img_files), length(fusion_data.timestamp));
end
odo_trajectory = zeros(num_frames, 3);
exp_trajectory = zeros(num_frames, 3);
odo_x = 0; odo_y = 0; odo_z = 0;
odo_yaw = 0; odo_height = 0;

% 初始化HDC和GC
[curYawTheta, curHeightValue] = get_hdc_initial_value();
[gcX, gcY, gcZ] = get_gc_initial_pos();

% 处理每一帧
for frame_idx = 1:num_frames
    if mod(frame_idx, 50) == 0
        fprintf('处理进度: %d/%d (%.1f%%)\n', frame_idx, num_frames, ...
            frame_idx/num_frames*100);
    end
    
    % 读取图像
    img_path = fullfile(data_path, img_files(frame_idx).name);
    rawImg = imread(img_path);
    
    % 使用IMU辅助的视觉里程计
    [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx);
    
    % 更新里程计位置
    odo_yaw = odo_yaw + yawRotV * DEGREE_TO_RADIAN;
    odo_height = odo_height + heightV;
    odo_x = odo_x + transV * cos(odo_yaw);
    odo_y = odo_y + transV * sin(odo_yaw);
    odo_z = odo_height;
    
    odo_trajectory(frame_idx, :) = [odo_x, odo_y, odo_z];
    
    % 视觉模板匹配（使用正确的函数）
    % visual_template需要当前位置和姿态
    % 使用融合数据的当前帧（如果可用），否则使用里程计估计
    if frame_idx <= size(fusion_data.pos, 1)
        curr_x = fusion_data.pos(frame_idx, 1);
        curr_y = fusion_data.pos(frame_idx, 2);
        curr_z = fusion_data.pos(frame_idx, 3);
        curr_yaw = fusion_data.att(frame_idx, 3);  % degrees
        curr_height = curr_z;
    else
        % 如果融合数据不足，使用里程计位置
        curr_x = odo_x;
        curr_y = odo_y;
        curr_z = odo_z;
        curr_yaw = odo_yaw * 180 / pi;  % 转换为degrees
        curr_height = odo_z;
    end
    
    % 使用简化类脑特征提取（论文验证方法）
    vtId = visual_template_neuro_matlab_only(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);
    % 计算VT识别率
    if vtId > 0 && vtId == PREV_VT_ID
        vtRecog = 1;
    else
        vtRecog = 0;
    end
    
    % 更新HDC（修正参数顺序：vt_id, yawRotV, heightV）
    yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
    [curYawTheta, curHeightValue] = get_current_yaw_height_value();
    
    % 转换为弧度用于GC迭代
    curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
    
    % 更新3D网格细胞
    gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
    [gcX, gcY, gcZ] = get_gc_xyz();
    
    % 更新经验地图
    exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);
    
    % 使用全局变量CUR_EXP_ID获取当前经验节点
    if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0 && CUR_EXP_ID <= length(EXPERIENCES)
        exp_trajectory(frame_idx, :) = [EXPERIENCES(CUR_EXP_ID).x_exp, ...
                                         EXPERIENCES(CUR_EXP_ID).y_exp, ...
                                         EXPERIENCES(CUR_EXP_ID).z_exp];
    else
        exp_trajectory(frame_idx, :) = [0, 0, 0];
    end
    
    % 更新PREV_VT_ID
    PREV_VT_ID = vtId;
end

fprintf('[5/9] SLAM处理完成！\n');
fprintf('  经验地图节点数: %d\n', NUM_EXPS);
fprintf('  视觉模板数: %d\n', NUM_VT);  % 使用NUM_VT（增强方法）而不是VT_ID_COUNT
if NUM_EXPS < 10
    warning('经验地图节点数过少（%d个），可能导致轨迹异常！', NUM_EXPS);
    fprintf('  建议：降低DELTA_EXP_GC_HDC_THRESHOLD参数\n');
end

%% 6. 对比纯视觉和IMU-视觉融合结果
fprintf('[6/9] 生成对比可视化...\n');

% 准备结果保存目录
result_path = fullfile(data_path, 'slam_results');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

% 如果有Ground Truth，先进行轨迹对齐
if has_ground_truth
    fprintf('正在对齐轨迹到相同坐标系...\n');
    % 使用增强的simple方法（平移+尺度修正）
    [fusion_pos_aligned, gt_pos_aligned] = align_trajectories(fusion_data.pos, gt_data.pos, 'simple');
    [odo_traj_aligned, ~] = align_trajectories(odo_trajectory, gt_data.pos, 'simple');
    [exp_traj_aligned, ~] = align_trajectories(exp_trajectory, gt_data.pos, 'simple');
    
    % 创建对齐后的数据结构
    fusion_data_aligned = fusion_data;
    fusion_data_aligned.pos = fusion_pos_aligned;
    gt_data_aligned = gt_data;
    gt_data_aligned.pos = gt_pos_aligned;
    
    plot_imu_visual_comparison_with_gt(fusion_data_aligned, odo_traj_aligned, exp_traj_aligned, gt_data_aligned, result_path);
else
    plot_imu_visual_comparison(fusion_data, odo_trajectory, exp_trajectory, [], result_path);
end

%% 7. 精度评估
fprintf('[7/9] 评估轨迹精度...\n');

if has_ground_truth
    % 使用Ground Truth作为参考进行精度评估（使用前面已对齐的轨迹）
    fprintf('\n========== 相对于Ground Truth的精度评估 ==========\n\n');
    
    % 1. IMU-视觉融合 vs Ground Truth (对齐后)
    fprintf('\n--- IMU-视觉融合轨迹 vs Ground Truth (对齐后) ---\n');
    metrics_fusion_gt = evaluate_slam_accuracy(fusion_pos_aligned, gt_pos_aligned, result_path, 'imu_fusion');
    
    % 2. 经验地图 vs Ground Truth (对齐后)
    fprintf('\n--- 经验地图轨迹 vs Ground Truth (对齐后) ---\n');
    if size(exp_trajectory, 1) == size(gt_data.pos, 1)
        metrics_exp_gt = evaluate_slam_accuracy(exp_traj_aligned, gt_pos_aligned, result_path, 'experience_map');
    else
        fprintf('轨迹长度不匹配\n');
    end
    
    % 3. 视觉里程计 vs Ground Truth (对齐后)
    fprintf('\n--- 视觉里程计轨迹 vs Ground Truth (对齐后) ---\n');
    if size(odo_trajectory, 1) == size(gt_data.pos, 1)
        metrics_odo_gt = evaluate_slam_accuracy(odo_traj_aligned, gt_pos_aligned, result_path, 'visual_odometry');
    else
        fprintf('轨迹长度不匹配\n');
    end
else
    % 没有Ground Truth时，使用经验地图作为参考
    fprintf('\n--- IMU-视觉融合轨迹 vs 经验地图轨迹 ---\n');
    if size(exp_trajectory, 1) == size(fusion_data.pos, 1)
        metrics_fusion = evaluate_slam_accuracy(fusion_data.pos, exp_trajectory);
    else
        fprintf('轨迹长度不匹配,跳过精度评估\n');
    end
end

%% 8. 保存结果
fprintf('[8/9] 保存结果...\n');
result_path = fullfile(data_path, 'slam_results');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

% 保存轨迹数据
if has_ground_truth
    save(fullfile(result_path, 'trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory', 'imu_data', 'gt_data');
else
    save(fullfile(result_path, 'trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory', 'imu_data');
end

% 保存里程计轨迹
dlmwrite(fullfile(result_path, 'odo_trajectory.txt'), odo_trajectory, 'precision', 6);

% 保存经验地图轨迹
dlmwrite(fullfile(result_path, 'exp_trajectory.txt'), exp_trajectory, 'precision', 6);

% 如果有Ground Truth，也保存一份副本
if has_ground_truth
    dlmwrite(fullfile(result_path, 'ground_truth_backup.txt'), gt_data.pos, 'precision', 6);
end

fprintf('结果已保存到: %s\n', result_path);

%% 9. 生成对比报告
fprintf('[9/9] 生成性能对比报告...\n');
report_file = fullfile(result_path, 'performance_report.txt');
fid = fopen(report_file, 'w');

fprintf(fid, '========================================\n');
fprintf(fid, 'IMU-Visual Fusion SLAM Performance Report\n');
fprintf(fid, '========================================\n\n');

fprintf(fid, '数据集信息:\n');
fprintf(fid, '  路径: %s\n', data_path);
fprintf(fid, '  总帧数: %d\n', num_frames);
fprintf(fid, '  IMU采样点: %d\n', imu_data.count);
fprintf(fid, '\n');

fprintf(fid, '轨迹长度:\n');
fusion_length = sum(sqrt(sum(diff(fusion_data.pos).^2, 2)));
odo_length = sum(sqrt(sum(diff(odo_trajectory).^2, 2)));
exp_length = sum(sqrt(sum(diff(exp_trajectory).^2, 2)));
if has_ground_truth
    gt_length = sum(sqrt(sum(diff(gt_data.pos).^2, 2)));
    fprintf(fid, '  Ground Truth: %.2f m\n', gt_length);
end
fprintf(fid, '  IMU-视觉融合: %.2f m', fusion_length);
if has_ground_truth
    fprintf(fid, ' (误差: %.2f m, %.2f%%)\n', abs(fusion_length - gt_length), abs(fusion_length - gt_length)/gt_length*100);
else
    fprintf(fid, '\n');
end
fprintf(fid, '  视觉里程计: %.2f m', odo_length);
if has_ground_truth
    fprintf(fid, ' (误差: %.2f m, %.2f%%)\n', abs(odo_length - gt_length), abs(odo_length - gt_length)/gt_length*100);
else
    fprintf(fid, '\n');
end
fprintf(fid, '  经验地图: %.2f m', exp_length);
if has_ground_truth && exp_length > 0
    fprintf(fid, ' (误差: %.2f m, %.2f%%)\n', abs(exp_length - gt_length), abs(exp_length - gt_length)/gt_length*100);
else
    fprintf(fid, '\n');
end
fprintf(fid, '\n');

fprintf(fid, '平均位置不确定性:\n');
fprintf(fid, '  X: %.4f m\n', mean(fusion_data.uncertainty(:,1)));
fprintf(fid, '  Y: %.4f m\n', mean(fusion_data.uncertainty(:,2)));
fprintf(fid, '  Z: %.4f m\n', mean(fusion_data.uncertainty(:,3)));
fprintf(fid, '\n');

fprintf(fid, '改进效果:\n');
imu_drift = norm(fusion_data.imu_pos(end,:) - fusion_data.pos(end,:));
fprintf(fid, '  纯IMU漂移: %.2f m\n', imu_drift);
fprintf(fid, '  IMU漂移率: %.2f%%\n', (imu_drift/fusion_length)*100);
fprintf(fid, '\n');

% 如果有Ground Truth，添加精度评估摘要（使用对齐后的轨迹）
if has_ground_truth
    fprintf(fid, '相对于Ground Truth的精度评估(对齐后):\n');
    fprintf(fid, '----------------------------------------\n');
    fprintf(fid, '注：轨迹已对齐到相同坐标系以去除平移和旋转差异\n\n');
    
    % IMU-Visual Fusion
    fusion_error = sqrt(sum((fusion_pos_aligned - gt_pos_aligned).^2, 2));
    fprintf(fid, 'IMU-Visual Fusion:\n');
    fprintf(fid, '  平均位置误差: %.2f m\n', mean(fusion_error));
    fprintf(fid, '  RMSE: %.2f m\n', sqrt(mean(fusion_error.^2)));
    fprintf(fid, '  最大误差: %.2f m\n', max(fusion_error));
    fprintf(fid, '  终点误差: %.2f m\n', norm(fusion_pos_aligned(end,:) - gt_pos_aligned(end,:)));
    fprintf(fid, '\n');
    
    % Visual Odometry
    if size(odo_trajectory, 1) == size(gt_data.pos, 1)
        odo_error = sqrt(sum((odo_traj_aligned - gt_pos_aligned).^2, 2));
        fprintf(fid, 'Visual Odometry:\n');
        fprintf(fid, '  平均位置误差: %.2f m\n', mean(odo_error));
        fprintf(fid, '  RMSE: %.2f m\n', sqrt(mean(odo_error.^2)));
        fprintf(fid, '  最大误差: %.2f m\n', max(odo_error));
        fprintf(fid, '  终点误差: %.2f m\n', norm(odo_traj_aligned(end,:) - gt_pos_aligned(end,:)));
        fprintf(fid, '\n');
    end
    
    % Experience Map
    if size(exp_trajectory, 1) == size(gt_data.pos, 1) && any(exp_trajectory(:) ~= 0)
        exp_error = sqrt(sum((exp_traj_aligned - gt_pos_aligned).^2, 2));
        fprintf(fid, 'Experience Map:\n');
        fprintf(fid, '  平均位置误差: %.2f m\n', mean(exp_error));
        fprintf(fid, '  RMSE: %.2f m\n', sqrt(mean(exp_error.^2)));
        fprintf(fid, '  最大误差: %.2f m\n', max(exp_error));
        fprintf(fid, '  终点误差: %.2f m\n', norm(exp_traj_aligned(end,:) - gt_pos_aligned(end,:)));
        fprintf(fid, '\n');
    end
    
    fprintf(fid, '----------------------------------------\n');
end

fprintf(fid, '========================================\n');
fclose(fid);

fprintf('性能报告已保存: %s\n', report_file);

%% 完成
fprintf('\n========================================\n');
fprintf('IMU-Visual Fusion SLAM测试完成!\n');
fprintf('========================================\n');
fprintf('主要输出:\n');
fprintf('  1. 对比可视化图: imu_visual_slam_comparison.png\n');
fprintf('  2. 精度评估图: slam_accuracy_evaluation.png\n');
fprintf('  3. 轨迹数据: %s/trajectories.mat\n', result_path);
fprintf('  4. 性能报告: %s/performance_report.txt\n', result_path);
fprintf('========================================\n');
