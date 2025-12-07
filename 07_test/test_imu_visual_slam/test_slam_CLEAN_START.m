%% 全新测试脚本 - 完全避免函数缓存问题
%
% 策略：直接内联所有代码，不调用可能被缓存的外部函数
%
% 成功配置（已验证）：
%   adapthisteq(0.02) -> imgaussfilt(0.5) -> Sobel -> 融合 -> 归一化
%   VT阈值：0.07
%   预期：VT ~299, RMSE ~126m

clear all; close all; clc;

fprintf('========================================\n');
fprintf('全新测试 - 避免缓存问题\n');
fprintf('========================================\n');
fprintf('使用直接内联的特征提取代码\n');
fprintf('预期：VT ~299个，RMSE ~126米\n');
fprintf('========================================\n\n');

%% 1. 添加路径
rootDir = '/home/dream/neuro_111111/carla-pedestrians/neuro';
addpath(genpath(fullfile(rootDir, '01_conjunctive_pose_cells_network')));
addpath(genpath(fullfile(rootDir, '03_visual_odometry')));
addpath(genpath(fullfile(rootDir, '04_visual_template')));
addpath(genpath(fullfile(rootDir, '02_multilayered_experience_map')));
addpath(genpath(fullfile(rootDir, '05_tookit')));
addpath(genpath(fullfile(rootDir, '09_vestibular')));
fprintf('✓ 路径已添加\n');

%% 2. 初始化全局变量
fprintf('[1/9] 初始化全局变量...\n');
global PREV_VT_ID; PREV_VT_ID = -1;
global VT_TEMPLATES; VT_TEMPLATES = [];
global NUM_VT; NUM_VT = 0;
global YAW_HEIGHT_HDC; YAW_HEIGHT_HDC = zeros(36, 36);
global GRIDCELLS; GRIDCELLS = zeros(36, 36, 36);
global EXPERIENCES; EXPERIENCES = [];
global NUM_EXPS; NUM_EXPS = 0;
global CUR_EXP_ID; CUR_EXP_ID = 0;
global PREV_EXP_ID; PREV_EXP_ID = 0;
global PREV_TRANS_V; PREV_TRANS_V = 0;
global PREV_YAW_ROT_V; PREV_YAW_ROT_V = 0;
global PREV_HEIGHT_V; PREV_HEIGHT_V = 0;
global YAW_HEIGHT_HDC_Y_TH_SIZE; YAW_HEIGHT_HDC_Y_TH_SIZE = 2*pi/36;

% 经验地图相关全局变量
global ACCUM_DELTA_X ACCUM_DELTA_Y ACCUM_DELTA_Z ACCUM_DELTA_YAW;
global DELTA_EM MIN_DELTA_EM;
global EXP_HISTORY;
DELTA_EM = [];
MIN_DELTA_EM = [];
EXP_HISTORY = [];

% VT相关全局变量（用于yaw_height_hdc_iteration）
global VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD;
VT = struct([]);
VT_HISTORY = [];
VT_HISTORY_FIRST = [];
VT_HISTORY_OLD = [];

% 初始化视觉模板参数
global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
global VT_PANORAMIC;

VT_IMG_CROP_Y_RANGE = [1:120];
VT_IMG_CROP_X_RANGE = [1:160];
VT_IMG_RESIZE_X_RANGE = 16;
VT_IMG_RESIZE_Y_RANGE = 12;
VT_IMG_X_SHIFT = 0;
VT_IMG_Y_SHIFT = 0;
VT_IMG_HALF_OFFSET = 8;
VT_MATCH_THRESHOLD = 0.07;  % 成功配置的阈值
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 1.0;
MIN_DIFF_CURR_IMG_VTS = realmax;
DIFFS_ALL_IMGS_VTS = [];
VT_PANORAMIC = 0;

% 常量
DEGREE_TO_RADIAN = pi / 180;
RADIAN_TO_DEGREE = 180 / pi;

%% 3. 初始化模块
fprintf('[2/9] 初始化SLAM模块...\n');
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
    'ODO_SHIFT_MATCH_VERT', 26, ...
    'ODO_SHIFT_MATCH_YAW_ROT', 21);

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

%% 4. 加载数据
fprintf('[3/9] 加载数据...\n');
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');

% 读取IMU数据
imu_data = read_imu_data(data_path);

% 读取融合位姿数据
fusion_data = read_fusion_pose(data_path);

% 读取Ground Truth数据
gt_file = fullfile(data_path, 'ground_truth.txt');
if exist(gt_file, 'file')
    gt_data = read_ground_truth(gt_file);
    fprintf('✓ 已加载Ground Truth数据\n');
else
    error('未找到Ground Truth文件');
end

% 获取图像文件列表
img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到图像文件');
end

num_frames = min(length(img_files), length(fusion_data.timestamp));
fprintf('✓ 找到 %d 张图像\n', num_frames);

%% 5. SLAM主循环
fprintf('[4/9] 开始SLAM处理...\n');

% 存储结果
exp_trajectory = zeros(num_frames, 3);
odo_trajectory = zeros(num_frames, 3);
feature_extraction_times = zeros(num_frames, 1);

% 初始化位置
odo_x = 0; odo_y = 0; odo_z = 0;
odo_yaw = 0; odo_height = 0;

% 初始化HDC和GC
[curYawTheta, curHeightValue] = get_hdc_initial_value();
[gcX, gcY, gcZ] = get_gc_initial_pos();

% 处理每一帧
for frame_idx = 1:num_frames
    if mod(frame_idx, 50) == 0
        fprintf('处理进度: %d/%d (%.1f%%)\n', frame_idx, num_frames, 100*frame_idx/num_frames);
    end
    
    % 读取图像
    img_file = fullfile(data_path, img_files(frame_idx).name);
    rawImg = imread(img_file);
    
    % 使用IMU辅助的视觉里程计
    [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx);
    
    % 更新里程计位置
    odo_yaw = odo_yaw + yawRotV * DEGREE_TO_RADIAN;
    odo_height = odo_height + heightV;
    odo_x = odo_x + transV * cos(odo_yaw);
    odo_y = odo_y + transV * sin(odo_yaw);
    odo_z = odo_height;
    
    odo_trajectory(frame_idx, :) = [odo_x, odo_y, odo_z];
    
    % 获取当前位置（使用融合位姿数据，不是里程计累积位置）
    if frame_idx <= size(fusion_data.pos, 1)
        curr_x = fusion_data.pos(frame_idx, 1);
        curr_y = fusion_data.pos(frame_idx, 2);
        curr_z = fusion_data.pos(frame_idx, 3);
    else
        curr_x = odo_x;
        curr_y = odo_y;
        curr_z = odo_z;
    end
    
    %% ========== 直接内联特征提取（避免缓存） ==========
    tic;
    
    % 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    
    % 特征提取（成功配置）
    img = vtResizedImg;
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    % 1. 对比度增强 (ClipLimit=0.02, 关键参数!)
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
    
    % 2. 高斯平滑 (sigma=0.5, 关键步骤!)
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    
    % 3. 边缘检测
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 4. 融合
    combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    
    % 5. 归一化
    normVtImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
    
    SUB_VT_IMG = normVtImg;
    
    %% VT匹配逻辑（内联）
    if NUM_VT < 5
        % 前5个直接添加
        if NUM_VT > 0
            for k = 1:NUM_VT
                VT_TEMPLATES(k).active_decay = VT_TEMPLATES(k).active_decay - VT_GLOBAL_DECAY;
            end
        end
        
        NUM_VT = NUM_VT + 1;
        VT_TEMPLATES(NUM_VT).template = normVtImg;
        VT_TEMPLATES(NUM_VT).active_decay = 1.0;
        VT_TEMPLATES(NUM_VT).x = curr_x;
        VT_TEMPLATES(NUM_VT).y = curr_y;
        VT_TEMPLATES(NUM_VT).z = curr_z;
        
        % 同步更新VT数组（用于SLAM所有模块）
        VT(NUM_VT).id = NUM_VT;
        VT(NUM_VT).template = normVtImg;
        VT(NUM_VT).decay = 0;
        VT(NUM_VT).gc_x = gcX;
        VT(NUM_VT).gc_y = gcY;
        VT(NUM_VT).gc_z = gcZ;
        VT(NUM_VT).hdc_yaw = curYawTheta;
        VT(NUM_VT).hdc_height = curHeightValue;
        VT(NUM_VT).first = 1;  % 第一次见到
        VT(NUM_VT).numExp = 0;  % 关联的经验数量
        VT(NUM_VT).EXPERIENCES = [];  % 关联的经验列表
        
        vtId = NUM_VT;
    else
        % 计算与所有VT的距离
        min_diff = realmax;
        best_vt = -1;
        
        for k = 1:NUM_VT
            template = VT_TEMPLATES(k).template;
            
            % 余弦距离
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
            % 创建新VT
            NUM_VT = NUM_VT + 1;
            VT_TEMPLATES(NUM_VT).template = normVtImg;
            VT_TEMPLATES(NUM_VT).active_decay = 1.0;
            VT_TEMPLATES(NUM_VT).x = curr_x;
            VT_TEMPLATES(NUM_VT).y = curr_y;
            VT_TEMPLATES(NUM_VT).z = curr_z;
            
            % 同步更新VT数组（用于SLAM所有模块）
            VT(NUM_VT).id = NUM_VT;
            VT(NUM_VT).template = normVtImg;
            VT(NUM_VT).decay = 0;
            VT(NUM_VT).gc_x = gcX;
            VT(NUM_VT).gc_y = gcY;
            VT(NUM_VT).gc_z = gcZ;
            VT(NUM_VT).hdc_yaw = curYawTheta;
            VT(NUM_VT).hdc_height = curHeightValue;
            VT(NUM_VT).first = 1;  % 第一次见到
            VT(NUM_VT).numExp = 0;  % 关联的经验数量
            VT(NUM_VT).EXPERIENCES = [];  % 关联的经验列表
            
            vtId = NUM_VT;
        else
            % 匹配已有VT
            vtId = best_vt;
            VT_TEMPLATES(vtId).active_decay = VT_ACTIVE_DECAY;
            
            % 更新VT数组中的first标志
            if VT(vtId).first == 1
                VT(vtId).first = 0;  % 不再是第一次见到
            end
        end
        
        % Decay所有VT
        for k = 1:NUM_VT
            VT_TEMPLATES(k).active_decay = VT_TEMPLATES(k).active_decay - VT_GLOBAL_DECAY;
            VT(k).decay = VT(k).decay + 0.01;  % 增加decay值（用于能量注入衰减）
        end
    end
    
    % 重置当前VT的decay
    if vtId > 0 && vtId <= NUM_VT
        VT(vtId).decay = 0;
    end
    
    feature_extraction_times(frame_idx) = toc;
    
    % 更新HDC
    yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
    [curYawTheta, curHeightValue] = get_current_yaw_height_value();
    
    % 转换为弧度
    curYawThetaInRadian = curYawTheta * (2 * pi / 36);  % YAW_HEIGHT_HDC_Y_TH_SIZE
    
    % 更新3D网格细胞
    gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
    [gcX, gcY, gcZ] = get_gc_xyz();
    
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

%% 6. 评估结果
fprintf('[6/9] 评估结果...\n');

% 确保长度匹配
gt_pos = gt_data.pos(1:num_frames, :);

% 对齐轨迹（强制起点对齐）
exp_traj_aligned = exp_trajectory - exp_trajectory(1,:) + gt_pos(1,:);
gt_pos_aligned = gt_pos;

% 计算RMSE
errors = sqrt(sum((exp_traj_aligned - gt_pos_aligned).^2, 2));
rmse = sqrt(mean(errors.^2));

fprintf('\n========== SLAM精度评估结果 ==========\n');
fprintf('绝对轨迹误差 (ATE):\n');
fprintf('  RMSE:     %.4f m\n', rmse);
fprintf('  平均值:   %.4f m\n', mean(errors));
fprintf('  中位数:   %.4f m\n', median(errors));
fprintf('  标准差:   %.4f m\n', std(errors));
fprintf('  最大值:   %.4f m\n', max(errors));
fprintf('  最小值:   %.4f m\n', min(errors));
fprintf('======================================\n\n');

%% 7. 绘制轨迹对比图
fprintf('[7/9] 生成轨迹对比图...\n');
save_dir = fullfile(data_path, 'slam_results_clean_start');
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 对齐所有轨迹
odo_traj_aligned = odo_trajectory - odo_trajectory(1,:) + gt_pos(1,:);
fusion_pos_aligned = fusion_data.pos(1:num_frames, :) - fusion_data.pos(1,:) + gt_pos(1,:);

% 创建对齐后的数据结构
fusion_data_aligned = struct();
fusion_data_aligned.pos = fusion_pos_aligned;
fusion_data_aligned.timestamp = fusion_data.timestamp(1:num_frames);

gt_data_aligned = struct();
gt_data_aligned.pos = gt_pos_aligned;

% 绘制轨迹对比图
plot_imu_visual_comparison_with_gt(fusion_data_aligned, odo_traj_aligned, exp_traj_aligned, gt_data_aligned, save_dir);

%% 8. 保存结果
fprintf('[8/9] 保存结果...\n');

save(fullfile(save_dir, 'slam_result_clean_start.mat'), ...
    'exp_trajectory', 'odo_trajectory', 'feature_extraction_times', ...
    'NUM_VT', 'NUM_EXPS', 'rmse');

% 保存对齐后的轨迹数据
save(fullfile(save_dir, 'trajectories.mat'), ...
    'fusion_data_aligned', 'odo_traj_aligned', 'exp_traj_aligned', 'gt_data_aligned');

fprintf('[9/9] 完成！\n\n');
fprintf('========================================\n');
fprintf('SLAM测试完成（全新脚本）!\n');
fprintf('========================================\n');
fprintf('结果：\n');
fprintf('  VT数量:    %d\n', NUM_VT);
fprintf('  经验节点:  %d\n', NUM_EXPS);
fprintf('  RMSE:      %.2f米\n', rmse);
fprintf('========================================\n');
fprintf('图像和结果保存在: %s\n', save_dir);
fprintf('========================================\n');

% 如果VT还是5个，打印诊断信息
if NUM_VT <= 10
    fprintf('\n⚠️  警告：VT数量异常少！\n');
    fprintf('诊断信息：\n');
    fprintf('  特征提取时间: %.4f秒（正常应该>0.03秒）\n', mean(feature_extraction_times));
    fprintf('  VT阈值: %.2f\n', VT_MATCH_THRESHOLD);
    fprintf('\n可能原因：\n');
    fprintf('  1. 特征仍然过于相似\n');
    fprintf('  2. 参数设置有问题\n');
    fprintf('  3. 需要调试特征提取过程\n');
end
