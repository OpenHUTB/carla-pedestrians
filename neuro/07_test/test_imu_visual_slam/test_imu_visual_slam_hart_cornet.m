%% IMU-Visual Fusion SLAM with HART+CORnet Feature Extractor
%  NeuroSLAM System Copyright (C) 2018-2019
%  IMU-Visual Integration with Enhanced Feature Extraction (2024)
%
%  本脚本使用HART+CORnet特征提取器测试IMU-视觉融合的NeuroSLAM系统
%
%  主要改进：
%    1. CORnet层次化特征提取 (V1->V2->V4->IT)
%    2. HART注意力机制和时序建模
%    3. 更鲁棒的场景识别能力

clear all; close all; clc;

%% 配置选项
USE_HART_CORNET = true;  % 尝试HART+CORnet深度特征，看能否改善区分度

%% 1. 添加路径
fprintf('========== IMU-Visual Fusion SLAM (HART+CORnet) ==========\n');
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

if USE_HART_CORNET
    fprintf('✓ 特征提取方法: HART+CORnet (层次化+注意力+时序)\n');
else
    fprintf('✓ 特征提取方法: 简单方法 (已验证: VT=299, RMSE=126m)\n');
end

%% 2. 初始化全局变量
fprintf('[2/9] 初始化全局变量...\n');
global PREV_VT_ID; PREV_VT_ID = -1;
global VT_TEMPLATES; VT_TEMPLATES = [];
global VT_ID_COUNT; VT_ID_COUNT = 0;
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

% VT相关全局变量
global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD;
VT = struct([]);
NUM_VT = 0;
VT_HISTORY = [];
VT_HISTORY_FIRST = [];
VT_HISTORY_OLD = [];

global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
global VT_PANORAMIC;

MIN_DIFF_CURR_IMG_VTS = [];
DIFFS_ALL_IMGS_VTS = [];
SUB_VT_IMG = [];

%% 3. 初始化各模块参数
fprintf('[3/9] 初始化模块参数...\n');

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

% 视觉模板初始化
VT_IMG_CROP_Y_RANGE = 1:120;
VT_IMG_CROP_X_RANGE = 1:160;
VT_IMG_RESIZE_X_RANGE = 16;
VT_IMG_RESIZE_Y_RANGE = 12;
VT_IMG_X_SHIFT = 5;
VT_IMG_Y_SHIFT = 3;
VT_IMG_HALF_OFFSET = floor(VT_IMG_RESIZE_X_RANGE / 2);
VT_MATCH_THRESHOLD = 0.05;  % Town10需要更多VT区分复杂场景（从0.07降低）
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 2.0;
VT_PANORAMIC = 0;

fprintf('✓ VT阈值: %.3f (已验证成功配置)\n', VT_MATCH_THRESHOLD);

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
    'GC_HORI_TRANS_V_SCALE', 0.8, ...
    'GC_VERT_TRANS_V_SCALE', 0.8, ...
    'GC_PACKET_SIZE', 4);

% 经验地图初始化
exp_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...
    'EXP_LOOPS', 1, ...
    'EXP_CORRECTION', 0.5);

fprintf('✓ 经验地图阈值: 15\n');

%% 4. 读取IMU-视觉融合数据
fprintf('[4/9] 读取IMU-视觉融合数据...\n');
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion');  % 切换到Town10

if ~exist(data_path, 'dir')
    error('数据路径不存在: %s\n请先运行Python脚本采集数据', data_path);
end

% 读取IMU数据
imu_data = read_imu_data(data_path);

% 读取融合位姿数据
fusion_data = read_fusion_pose(data_path);

% 读取Ground Truth数据
gt_file = fullfile(data_path, 'ground_truth.txt');
if exist(gt_file, 'file')
    gt_data = read_ground_truth(gt_file);
    has_ground_truth = true;
    fprintf('✓ 已加载Ground Truth数据\n');
else
    has_ground_truth = false;
    warning('⚠️  未找到Ground Truth文件');
end

% 获取图像文件列表
img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到图像文件');
end
fprintf('✓ 找到 %d 张图像\n', length(img_files));

%% 5. 运行SLAM
fprintf('[5/9] 开始运行SLAM (使用%s)...\n', ...
    iif(USE_HART_CORNET, 'HART+CORnet特征提取器', '简单特征提取'));

num_frames = min(length(img_files), length(fusion_data.timestamp));
odo_trajectory = zeros(num_frames, 3);
exp_trajectory = zeros(num_frames, 3);
odo_x = 0; odo_y = 0; odo_z = 0;
odo_yaw = 0; odo_height = 0;

% 初始化HDC和GC
[curYawTheta, curHeightValue] = get_hdc_initial_value();
[gcX, gcY, gcZ] = get_gc_initial_pos();

% 性能统计
feature_extraction_times = zeros(num_frames, 1);

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
    
    % 获取当前位置
    if frame_idx <= size(fusion_data.pos, 1)
        curr_x = fusion_data.pos(frame_idx, 1);
        curr_y = fusion_data.pos(frame_idx, 2);
        curr_z = fusion_data.pos(frame_idx, 3);
        curr_yaw = fusion_data.att(frame_idx, 3);
        curr_height = curr_z;
    else
        curr_x = odo_x;
        curr_y = odo_y;
        curr_z = odo_z;
        curr_yaw = odo_yaw * 180 / pi;
        curr_height = odo_z;
    end
    
    % 视觉模板匹配（内联特征提取，避免MATLAB缓存问题）
    tic;
    is_new_vt = false;  % 初始化标志变量
    if USE_HART_CORNET
        vtId = visual_template_hart_cornet(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);
    else
        % 内联简单特征提取（已验证成功：VT=299, RMSE=126m）
        subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
        vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
        
        % === 特征提取（完全内联，避免函数缓存） ===
        img = vtResizedImg;
        if size(img, 3) == 3
            img = rgb2gray(img);
        end
        img = double(img) / 255.0;
        
        % 1. 对比度增强
        img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
        
        % 2. 高斯平滑
        img_smoothed = imgaussfilt(img_enhanced, 0.5);
        
        % 3. 边缘检测
        [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
        edge_magnitude = sqrt(Gx.^2 + Gy.^2);
        
        % 4. 融合（仅强度+边缘，已验证配置）
        combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
        
        % 6. 归一化
        normVtImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
        
        % === VT匹配逻辑（内联） ===
        if NUM_VT < 5
            % 前5个直接添加
            NUM_VT = NUM_VT + 1;
            VT(NUM_VT).id = NUM_VT;
            VT(NUM_VT).template = normVtImg;
            VT(NUM_VT).decay = 0;
            VT(NUM_VT).gc_x = 0;  % 临时值，稍后用gc_iteration结果更新
            VT(NUM_VT).gc_y = 0;
            VT(NUM_VT).gc_z = 0;
            VT(NUM_VT).hdc_yaw = 0;
            VT(NUM_VT).hdc_height = 0;
            VT(NUM_VT).first = 1;
            VT(NUM_VT).numExp = 0;
            VT(NUM_VT).EXPERIENCES = [];
            vtId = NUM_VT;
            is_new_vt = true;
        else
            % 计算与所有VT的余弦距离
            min_diff = realmax;
            best_vt = -1;
            for k = 1:NUM_VT
                template = VT(k).template;
                f1 = normVtImg(:) / (norm(normVtImg(:)) + eps);
                f2 = template(:) / (norm(template(:)) + eps);
                diff = 1 - dot(f1, f2);
                if diff < min_diff
                    min_diff = diff;
                    best_vt = k;
                end
            end
            
            % 判断是否创建新VT
            if min_diff > VT_MATCH_THRESHOLD
                NUM_VT = NUM_VT + 1;
                VT(NUM_VT).id = NUM_VT;
                VT(NUM_VT).template = normVtImg;
                VT(NUM_VT).decay = 0;
                VT(NUM_VT).gc_x = 0;  % 临时值，稍后用gc_iteration结果更新
                VT(NUM_VT).gc_y = 0;
                VT(NUM_VT).gc_z = 0;
                VT(NUM_VT).hdc_yaw = 0;
                VT(NUM_VT).hdc_height = 0;
                VT(NUM_VT).first = 1;
                VT(NUM_VT).numExp = 0;
                VT(NUM_VT).EXPERIENCES = [];
                vtId = NUM_VT;
                is_new_vt = true;
            else
                % 匹配到已有VT
                vtId = best_vt;
                VT(vtId).first = 0;
            end
        end
        
        % 更新decay
        for k = 1:NUM_VT
            VT(k).decay = VT(k).decay + 0.01;
        end
        if vtId > 0
            VT(vtId).decay = 0;
        end
    end
    feature_extraction_times(frame_idx) = toc;
    
    % 更新HDC
    yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
    [curYawTheta, curHeightValue] = get_current_yaw_height_value();
    
    % 转换为弧度
    curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
    
    % 更新3D网格细胞
    gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
    [gcX, gcY, gcZ] = get_gc_xyz();
    
    % 如果是新创建的VT，更新其gc和hdc值（使用gc_iteration和hdc_iteration的结果）
    if is_new_vt
        VT(vtId).gc_x = gcX;
        VT(vtId).gc_y = gcY;
        VT(vtId).gc_z = gcZ;
        VT(vtId).hdc_yaw = curYawTheta;
        VT(vtId).hdc_height = curHeightValue;
    end
    
    % 更新经验地图
    exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);
    
    % 获取当前经验节点
    if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0 && CUR_EXP_ID <= length(EXPERIENCES)
        exp_trajectory(frame_idx, :) = [EXPERIENCES(CUR_EXP_ID).x_exp, ...
                                         EXPERIENCES(CUR_EXP_ID).y_exp, ...
                                         EXPERIENCES(CUR_EXP_ID).z_exp];
    else
        exp_trajectory(frame_idx, :) = [0, 0, 0];
    end
end

fprintf('[5/9] SLAM处理完成！\n');
fprintf('  经验地图节点数: %d\n', NUM_EXPS);
fprintf('  视觉模板数: %d\n', NUM_VT);
fprintf('  平均特征提取时间: %.4f秒/帧\n', mean(feature_extraction_times));
fprintf('  总特征提取时间: %.2f秒\n', sum(feature_extraction_times));

%% 6-9. 可视化、评估、保存（与原版本相同）
fprintf('[6/9] 生成对比可视化...\n');

result_path = fullfile(data_path, 'slam_results_hart_cornet');
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

if has_ground_truth
    fprintf('正在对齐轨迹（强制起点对齐）...\n');
    % 使用强制起点对齐，避免Umeyama的缩放问题
    fusion_pos_aligned = fusion_data.pos - fusion_data.pos(1,:) + gt_data.pos(1,:);
    odo_traj_aligned = odo_trajectory - odo_trajectory(1,:) + gt_data.pos(1,:);
    exp_traj_aligned = exp_trajectory - exp_trajectory(1,:) + gt_data.pos(1,:);
    gt_pos_aligned = gt_data.pos;
    
    fusion_data_aligned = fusion_data;
    fusion_data_aligned.pos = fusion_pos_aligned;
    gt_data_aligned = gt_data;
    gt_data_aligned.pos = gt_pos_aligned;
    
    plot_imu_visual_comparison_with_gt(fusion_data_aligned, odo_traj_aligned, exp_traj_aligned, gt_data_aligned, result_path);
else
    plot_imu_visual_comparison(fusion_data, odo_trajectory, exp_trajectory, [], result_path);
end

fprintf('[7/9] 评估轨迹精度...\n');
if has_ground_truth
    fprintf('\n========== 相对于Ground Truth的精度评估 ==========\n');
    % 评估SLAM输出（经验轨迹），而不是融合位姿输入！
    metrics_slam_gt = evaluate_slam_accuracy(exp_traj_aligned, gt_pos_aligned, result_path, 'slam_exp_trajectory');
end

fprintf('[8/9] 保存结果...\n');
if has_ground_truth
    save(fullfile(result_path, 'trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory', 'imu_data', 'gt_data');
else
    save(fullfile(result_path, 'trajectories.mat'), ...
        'fusion_data', 'odo_trajectory', 'exp_trajectory', 'imu_data');
end

fprintf('[9/9] 完成！\n');
fprintf('\n========================================\n');
fprintf('SLAM测试完成 (HART+CORnet)!\n');
fprintf('========================================\n');
fprintf('结果保存在: %s\n', result_path);
fprintf('========================================\n');


%% 辅助函数
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
