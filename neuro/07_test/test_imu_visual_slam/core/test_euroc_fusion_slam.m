%% EuRoC IMU-Visual Fusion SLAM测试
%  测试EuRoC MAV Dataset（真实场景数据集）
%  支持MH_01_easy, MH_03_medium等序列
%
%  NeuroSLAM System Copyright (C) 2018-2019
%  EuRoC Support (2024)

% 清理工作空间（保留全局变量）
close all; clc;
clearvars -except EUROC_DATA_PATH;  % 清除所有变量，除了 EUROC_DATA_PATH

% 声明全局变量
global EUROC_DATA_PATH;

fprintf('\n========== EuRoC IMU-Visual Fusion SLAM Test ==========\n');

%% 1. 添加依赖路径
fprintf('[1/9] 添加依赖路径...\n');
% 使用脚本自身的位置计算相对路径（文件在core/子目录，需要回退3级）
scriptPath = fileparts(mfilename('fullpath'));  % 脚本所在目录
testDir = fileparts(scriptPath);  % test_imu_visual_slam目录
rootDir = fileparts(fileparts(fileparts(scriptPath)));  % 向上三级到neuro根目录
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(rootDir, '09_vestibular'));
addpath(fullfile(testDir, 'utils'));  % 添加utils目录
savepath;

%% 2. 初始化全局变量
fprintf('[2/9] 初始化全局变量...\n');
global POSECELL_X_SIZE;
global POSECELL_Y_SIZE;
global POSECELL_Z_SIZE;
global EXP_LOOPS;
global POSECELL;
global VT_ID;
global VT_HISTORY;
global NUM_EXPS;
global EXPS;
global VT_ID_COUNT;
global CUR_EXP_ID;
global EXPERIENCES;
global PREV_VT_ID; PREV_VT_ID = -1;
global DEGREE_TO_RADIAN; DEGREE_TO_RADIAN = pi / 180;
global YAW_HEIGHT_HDC_Y_TH_SIZE;

%% 3. 初始化模块参数
fprintf('[3/9] 初始化模块参数...\n');

% 视觉里程计初始化（EuRoC灰度图像，分辨率752x480）
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
    'ODO_TRANS_V_SCALE', 20, ...  % EuRoC调整
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

% 视觉模板初始化（EuRoC灰度图像，放宽阈值）
vt_image_initial('*.png', ...
    'VT_MATCH_THRESHOLD', 0.10, ...  % 0.10 for EuRoC grayscale
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
    'GC_HORI_TRANS_V_SCALE', 0.6, ...  % EuRoC调整
    'GC_VERT_TRANS_V_SCALE', 0.6, ...
    'GC_PACKET_SIZE', 4);

% 经验地图初始化
exp_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...
    'EXP_LOOPS', 1, ...
    'EXP_CORRECTION', 0.5);

fprintf('✓ 参数初始化完成（针对EuRoC优化）\n');

%% 4. 读取EuRoC数据
fprintf('[4/9] 读取EuRoC融合数据...\n');

% 设置EuRoC数据路径（使用全局变量）
global EUROC_DATA_PATH;
if isempty(EUROC_DATA_PATH)
    error('请先设置EUROC_DATA_PATH全局变量！例如：EUROC_DATA_PATH = ''/path/to/MH_01_easy'';');
end

fprintf('数据路径: %s\n', EUROC_DATA_PATH);

% 读取融合位姿数据（优先使用真正的VO+EKF融合文件）
fusion_file_vo_ekf = fullfile(EUROC_DATA_PATH, 'fusion_pose_vo_ekf.txt');
fusion_file_original = fullfile(EUROC_DATA_PATH, 'fusion_pose.txt');

if exist(fusion_file_vo_ekf, 'file')
    fprintf('使用VO+EKF融合数据（真实融合）...\n');
    fusion_file = fusion_file_vo_ekf;
elseif exist(fusion_file_original, 'file')
    fprintf('使用原始融合数据...\n');
    fusion_file = fusion_file_original;
else
    error('融合数据文件不存在');
end

fusion_data_raw = dlmread(fusion_file, ',', 1, 0);

fusion_data.timestamp = fusion_data_raw(:, 1);
fusion_data.pos = fusion_data_raw(:, 2:4);
fusion_data.att = fusion_data_raw(:, 5:7);
fusion_data.vel = fusion_data_raw(:, 8:10);
fusion_data.pos_uncertainty = fusion_data_raw(:, 11:13);
fprintf('✓ 融合位姿: %d 个\n', size(fusion_data.pos, 1));

% 读取Ground Truth
gt_file = fullfile(EUROC_DATA_PATH, 'ground_truth.txt');
if exist(gt_file, 'file')
    % 跳过第一行标题
    gt_data_raw = dlmread(gt_file, ',', 1, 0);
    gt_data.timestamp = gt_data_raw(:, 1);
    gt_data.pos = gt_data_raw(:, 2:4);
    gt_data.att = gt_data_raw(:, 5:7);
    has_ground_truth = true;
    fprintf('✓ Ground Truth: %d 个\n', size(gt_data.pos, 1));
else
    has_ground_truth = false;
    warning('未找到Ground Truth文件');
end

% 获取图像列表（支持两种目录结构）
img_dir = fullfile(EUROC_DATA_PATH, 'images');  % 转换后的结构
if ~exist(img_dir, 'dir')
    % 尝试原始EuRoC结构
    img_dir = fullfile(EUROC_DATA_PATH, 'cam0', 'data');
    if ~exist(img_dir, 'dir')
        error('图像目录不存在: %s\n请检查数据是否已正确处理', img_dir);
    end
end
img_files = dir(fullfile(img_dir, '*.png'));
num_images = length(img_files);
fprintf('✓ 图像: %d 张（目录: %s）\n', num_images, img_dir);

% 使用融合数据帧数
num_frames = size(fusion_data.pos, 1);

%% 5. 运行SLAM
fprintf('\n[5/9] 开始运行SLAM...\n');

% 初始化轨迹记录
odo_trajectory = zeros(num_frames, 3);
exp_trajectory = zeros(num_frames, 3);
odo_x = 0; odo_y = 0; odo_z = 0;
odo_yaw = 0; odo_height = 0;

% 初始化VT跟踪（用于统计唯一VT数量）
vt_ids_seen = [];  % 记录所有见过的VT ID

% 初始化HDC和GC
[curYawTheta, curHeightValue] = get_hdc_initial_value();
[gcX, gcY, gcZ] = get_gc_initial_pos();

% 处理每一帧
for frame_idx = 1:num_frames
    if mod(frame_idx, 200) == 0
        % 使用我们自己跟踪的VT数量（更可靠）
        vt_count = length(vt_ids_seen);
        % 如果VT_ID_COUNT可用，也显示它作为对比
        if ~isempty(VT_ID_COUNT) && VT_ID_COUNT > 0
            vt_count = VT_ID_COUNT;
        end
        fprintf('  进度: %d/%d (%.1f%%) | VT: %d | 经验: %d\n', ...
            frame_idx, num_frames, frame_idx/num_frames*100, vt_count, NUM_EXPS);
    end
    
    % 读取图像
    if frame_idx <= num_images
        img_path = fullfile(img_dir, img_files(frame_idx).name);
        rawImg = imread(img_path);
        if size(rawImg, 3) == 3
            rawImg = rgb2gray(rawImg);
        end
    else
        rawImg = imread(fullfile(img_dir, img_files(end).name));
        if size(rawImg, 3) == 3
            rawImg = rgb2gray(rawImg);
        end
    end
    
    % 使用融合数据获取当前位置和姿态
    if frame_idx <= size(fusion_data.pos, 1)
        curr_x = fusion_data.pos(frame_idx, 1);
        curr_y = fusion_data.pos(frame_idx, 2);
        curr_z = fusion_data.pos(frame_idx, 3);
        curr_yaw = fusion_data.att(frame_idx, 3);  % degrees
        curr_height = curr_z;
        
        % 计算运动增量（简化）
        if frame_idx > 1
            transV = norm(fusion_data.pos(frame_idx, 1:2) - fusion_data.pos(frame_idx-1, 1:2));
            yawRotV = fusion_data.att(frame_idx, 3) - fusion_data.att(frame_idx-1, 3);
            heightV = fusion_data.pos(frame_idx, 3) - fusion_data.pos(frame_idx-1, 3);
        else
            transV = 0;
            yawRotV = 0;
            heightV = 0;
        end
    else
        curr_x = odo_x;
        curr_y = odo_y;
        curr_z = odo_z;
        curr_yaw = odo_yaw * 180 / pi;
        curr_height = odo_z;
        transV = 0;
        yawRotV = 0;
        heightV = 0;
    end
    
    % 更新里程计位置
    odo_yaw = odo_yaw + yawRotV * pi/180;
    odo_height = odo_height + heightV;
    odo_x = odo_x + transV * cos(odo_yaw);
    odo_y = odo_y + transV * sin(odo_yaw);
    odo_z = odo_height;
    odo_trajectory(frame_idx, :) = [odo_x, odo_y, odo_z];
    
    % 视觉模板匹配
    vtId = visual_template(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);
    
    % 记录VT ID（用于统计）
    if vtId > 0 && ~ismember(vtId, vt_ids_seen)
        vt_ids_seen = [vt_ids_seen, vtId];
    end
    
    % 更新HDC
    yaw_height_hdc_iteration(vtId, yawRotV * pi/180, heightV);
    [curYawTheta, curHeightValue] = get_current_yaw_height_value();
    
    % 更新3D网格细胞
    global YAW_HEIGHT_HDC_Y_TH_SIZE;
    curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
    gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
    [gcX, gcY, gcZ] = get_gc_xyz();
    
    % 更新经验地图
    exp_map_iteration(vtId, transV, yawRotV * pi/180, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);
    
    % 记录经验地图轨迹
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

% 获取最终VT数量（优先使用我们跟踪的数量）
vt_final_count = length(vt_ids_seen);
if ~isempty(VT_ID_COUNT) && VT_ID_COUNT > 0
    vt_final_count = VT_ID_COUNT;
end
fprintf('\n[5/9] SLAM完成！VT: %d, 经验节点: %d\n', vt_final_count, NUM_EXPS);

%% 6. 轨迹对齐
fprintf('[6/9] 对齐轨迹...\n');

if has_ground_truth
    % 裁剪到最短长度
    min_len = min([size(fusion_data.pos, 1), size(gt_data.pos, 1), ...
                   size(odo_trajectory, 1), size(exp_trajectory, 1)]);
    
    fusion_trim = fusion_data.pos(1:min_len, :);
    gt_trim = gt_data.pos(1:min_len, :);
    odo_trim = odo_trajectory(1:min_len, :);
    exp_trim = exp_trajectory(1:min_len, :);
    
    % 对齐
    [fusion_aligned, gt_aligned] = align_trajectories(fusion_trim, gt_trim, 'simple');
    [odo_aligned, ~] = align_trajectories(odo_trim, gt_trim, 'simple');
    [exp_aligned, ~] = align_trajectories(exp_trim, gt_trim, 'simple');
end

%% 7. 生成可视化
fprintf('[7/9] 生成可视化...\n');

result_path = fullfile(EUROC_DATA_PATH, 'slam_results');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

if has_ground_truth
    % 创建对齐后的数据结构
    fusion_data_aligned.pos = fusion_aligned;
    gt_data_aligned.pos = gt_aligned;
    
    plot_imu_visual_comparison_with_gt(fusion_data_aligned, odo_aligned, exp_aligned, gt_data_aligned, result_path);
else
    plot_imu_visual_comparison(fusion_data, odo_trajectory, exp_trajectory, [], result_path);
end

%% 8. 精度评估
fprintf('[8/9] 精度评估...\n');

if has_ground_truth
    fprintf('\n========== 相对于Ground Truth的精度评估 ==========\n');
    
    fprintf('\n--- EKF Fusion (Input) vs Ground Truth ---\n');
    metrics_fusion = evaluate_slam_accuracy(fusion_aligned, gt_aligned, result_path, 'ekf_input');
    
    fprintf('\n--- Visual Odometry vs Ground Truth ---\n');
    metrics_odo = evaluate_slam_accuracy(odo_aligned, gt_aligned, result_path, 'visual_odometry');
    
    fprintf('\n--- Bio-inspired IMU-Visual Fusion (Ours) vs Ground Truth ---\n');
    metrics_exp = evaluate_slam_accuracy(exp_aligned, gt_aligned, result_path, 'bio_inspired_fusion');
    
    % 打印汇总
    fprintf('\n========== 精度评估汇总 ==========\n');
    traj_length = sum(sqrt(sum(diff(gt_aligned).^2, 2)));
    fprintf('Ground Truth行程: %.2f m\n\n', traj_length);
    
    fprintf('EKF Fusion (Input):\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_fusion.ate.rmse, metrics_fusion.ate.mean, metrics_fusion.final_error);
    
    fprintf('Visual Odometry:\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_odo.ate.rmse, metrics_odo.ate.mean, metrics_odo.final_error);
    
    fprintf('Bio-inspired Fusion (Ours):\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_exp.ate.rmse, metrics_exp.ate.mean, metrics_exp.final_error);
    
    fprintf('===================================\n\n');
end

%% 9. 保存结果
fprintf('[9/9] 保存结果...\n');

if has_ground_truth
    save(fullfile(result_path, 'euroc_trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory', 'gt_data');
else
    save(fullfile(result_path, 'euroc_trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory');
end

fprintf('\n✅ EuRoC SLAM测试完成！\n');
fprintf('结果保存在: %s\n', result_path);
