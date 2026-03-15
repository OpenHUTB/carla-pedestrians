%% EuRoC IMU-Visual Fusion SLAM测试
%  测试EuRoC MAV Dataset（真实场景数据集）
%  支持MH_01_easy, MH_03_medium序列
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

% 获取脚本所在目录的绝对路径
script_dir = fileparts(mfilename('fullpath'));

% 找到neuro根目录（假设脚本在neuro/07_test/test_imu_visual_slam/core/）
neuro_root = fileparts(fileparts(fileparts(script_dir)));

fprintf('✓ neuro根目录: %s\n', neuro_root);

% 添加所有需要的路径
addpath(genpath(fullfile(neuro_root, '00_tools')));
addpath(genpath(fullfile(neuro_root, '03_visual_odometry')));
addpath(genpath(fullfile(neuro_root, '04_visual_template')));
addpath(genpath(fullfile(neuro_root, '05_yaw_height_hdc')));
addpath(genpath(fullfile(neuro_root, '06_grid_cells')));
addpath(genpath(fullfile(neuro_root, '07_experience_map')));
addpath(genpath(fullfile(neuro_root, '08_visual_scene')));
addpath(genpath(fullfile(neuro_root, '09_vestibular')));  % IMU融合模块 + 报告生成函数
addpath(fullfile(neuro_root, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(neuro_root, '09_vestibular'));
addpath(fullfile(neuro_root, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(neuro_root, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(neuro_root, '04_visual_template'));
addpath(fullfile(neuro_root, '03_visual_odometry'));
addpath(fullfile(neuro_root, '02_multilayered_experience_map'));
addpath(fullfile(neuro_root, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(neuro_root, '09_vestibular'));
addpath(fullfile(neuro_root, '07_test/test_imu_visual_slam/utils'));  % 添加utils目录
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

%% 3. 应用参数覆盖（针对不同场景的优化）
global VT_MATCH_THRESHOLD_OVERRIDE;
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
global IMU_HEIGHT_WEIGHT_OVERRIDE;
global GC_VT_INJECT_ENERGY_OVERRIDE;

% 应用覆盖参数（如果存在）
VT_MATCH_THRESHOLD = 0.10;  % 默认值（EuRoC灰度图）
DELTA_EXP_GC_HDC_THRESHOLD = 20;
EXP_LOOPS = 8;
EXP_CORRECTION = 0.4;
GC_VT_INJECT_ENERGY = 0.5;

if ~isempty(VT_MATCH_THRESHOLD_OVERRIDE)
    VT_MATCH_THRESHOLD = VT_MATCH_THRESHOLD_OVERRIDE;
    fprintf('   ✓ VT阈值覆盖: %.3f\n', VT_MATCH_THRESHOLD);
end
if ~isempty(DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE)
    DELTA_EXP_GC_HDC_THRESHOLD = DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
    fprintf('   ✓ 经验地图阈值覆盖: %d\n', DELTA_EXP_GC_HDC_THRESHOLD);
end
if ~isempty(EXP_LOOPS_OVERRIDE)
    EXP_LOOPS = EXP_LOOPS_OVERRIDE;
    fprintf('   ✓ 地图松弛迭代覆盖: %d\n', EXP_LOOPS);
end
if ~isempty(EXP_CORRECTION_OVERRIDE)
    EXP_CORRECTION = EXP_CORRECTION_OVERRIDE;
    fprintf('   ✓ 修正力度覆盖: %.2f\n', EXP_CORRECTION);
end
if ~isempty(GC_VT_INJECT_ENERGY_OVERRIDE)
    GC_VT_INJECT_ENERGY = GC_VT_INJECT_ENERGY_OVERRIDE;
    fprintf('   ✓ GC VT注入能量覆盖: %.2f\n', GC_VT_INJECT_ENERGY);
end

%% 4. 初始化模块参数
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

% 视觉模板初始化（使用覆盖参数）
vt_image_initial('*.png', ...
    'VT_MATCH_THRESHOLD', VT_MATCH_THRESHOLD, ...
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
    'GC_VT_INJECT_ENERGY', GC_VT_INJECT_ENERGY, ...
    'GC_HORI_TRANS_V_SCALE', 0.6, ...  % EuRoC调整
    'GC_VERT_TRANS_V_SCALE', 0.6, ...
    'GC_PACKET_SIZE', 4);

% 经验地图初始化（使用覆盖参数）
exp_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', DELTA_EXP_GC_HDC_THRESHOLD, ...
    'EXP_LOOPS', EXP_LOOPS, ...
    'EXP_CORRECTION', EXP_CORRECTION);

fprintf('✓ 参数初始化完成\n');
fprintf('  VT阈值: %.3f, 经验阈值: %d, 松弛: %d次/%.2f\n', ...
    VT_MATCH_THRESHOLD, DELTA_EXP_GC_HDC_THRESHOLD, EXP_LOOPS, EXP_CORRECTION);

%% 4. 读取EuRoC数据
fprintf('[4/9] 读取EuRoC融合数据...\n');

% 设置EuRoC数据路径（使用全局变量）
global EUROC_DATA_PATH;
if isempty(EUROC_DATA_PATH)
    error('请先设置EUROC_DATA_PATH全局变量！例如：EUROC_DATA_PATH = ''/path/to/MH_01_easy'';');
end

fprintf('数据路径: %s\n', EUROC_DATA_PATH);

% 读取EKF融合位姿数据（作为对比baseline）
% fusion_data是外部Python脚本通过卡尔曼滤波生成的IMU-视觉融合结果
% 用于对比：Pure Visual < EKF Fusion < Bio-inspired SLAM
fusion_file_vo_ekf = fullfile(EUROC_DATA_PATH, 'fusion_pose_vo_ekf.txt');
fusion_file_original = fullfile(EUROC_DATA_PATH, 'fusion_pose.txt');

if exist(fusion_file_vo_ekf, 'file')
    fprintf('使用VO+EKF融合数据（卡尔曼滤波baseline）...\n');
    fusion_file = fusion_file_vo_ekf;
elseif exist(fusion_file_original, 'file')
    fprintf('使用融合数据（完整格式）...\n');
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
fprintf('✓ EKF融合位姿（对比baseline）: %d 个\n', size(fusion_data.pos, 1));

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

% 读取EuRoC原始IMU数据
fprintf('\n读取原始IMU数据...\n');
try
    imu_data_raw = read_euroc_imu_data(EUROC_DATA_PATH);
    has_imu_data = true;
    fprintf('✓ IMU数据: %d 条\n', imu_data_raw.count);
catch ME
    warning('无法读取IMU数据: %s\n将使用纯视觉模式', ME.message);
    has_imu_data = false;
    imu_data_raw = [];
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

% 提取图像时间戳并对齐IMU数据
if has_imu_data
    fprintf('\n对齐IMU数据到图像帧...\n');
    image_timestamps = get_euroc_image_timestamps(img_files);
    imu_data = align_imu_to_images(imu_data_raw, image_timestamps);
    fprintf('✓ IMU数据已对齐: %d 帧\n', imu_data.count);
else
    imu_data = [];
end

% 使用图像数量作为帧数
num_frames = num_images;

%% 5. 运行SLAM
fprintf('\n[5/9] 开始运行SLAM...\n');

% 初始化轨迹记录
pure_visual_traj = zeros(num_frames, 3);  % 纯视觉轨迹（baseline，不使用IMU）
imu_aided_traj = zeros(num_frames, 3);    % 时间戳对齐的惯视融合（Bio-inspired SLAM系统输入）
exp_trajectory = zeros(num_frames, 3);    % Bio-inspired SLAM输出（完整系统）

% 纯视觉里程计状态
pure_visual_x = 0; pure_visual_y = 0; pure_visual_z = 0;
pure_visual_yaw = 0; pure_visual_height = 0;

% IMU-aided里程计状态
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
    
    % 1. 计算纯视觉里程计- 独立计算
    [pure_transV, pure_yawRotV, pure_heightV] = visual_odometry(rawImg);
    
    % 更新纯视觉轨迹
    pure_visual_yaw = pure_visual_yaw + pure_yawRotV * DEGREE_TO_RADIAN;
    pure_visual_height = pure_visual_height + pure_heightV;
    pure_visual_x = pure_visual_x + pure_transV * cos(pure_visual_yaw);
    pure_visual_y = pure_visual_y + pure_transV * sin(pure_visual_yaw);
    pure_visual_z = pure_visual_height;
    
    pure_visual_traj(frame_idx, :) = [pure_visual_x, pure_visual_y, pure_visual_z];
    
    % 2. 计算时间戳对齐的IMU-视觉融合（生物启发惯视融合系统）
    if has_imu_data
        % 使用时间戳对齐融合IMU和视觉
        pure_visual_results = [pure_transV, pure_yawRotV, pure_heightV];
        [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx, pure_visual_results);
    else
        % 没有IMU数据，退化为纯视觉
        transV = pure_transV;
        yawRotV = pure_yawRotV;
        heightV = pure_heightV;
    end
    
    % 更新IMU-aided轨迹
    odo_yaw = odo_yaw + yawRotV * DEGREE_TO_RADIAN;
    odo_height = odo_height + heightV;
    odo_x = odo_x + transV * cos(odo_yaw);
    odo_y = odo_y + transV * sin(odo_yaw);
    odo_z = odo_height;
    
    imu_aided_traj(frame_idx, :) = [odo_x, odo_y, odo_z];
    
    % 3. Bio-inspired SLAM模块（使用imu_aided作为输入）
    % visual_template需要当前位置和姿态来存储VT的空间位置
    curr_x = odo_x;           % 使用时间戳对齐惯视融合的位置
    curr_y = odo_y;
    curr_z = odo_z;
    curr_yaw = odo_yaw * 180 / pi;
    curr_height = odo_z;
    
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
fprintf('\n[5/9] SLAM完成！\n');
fprintf('  VT数量: %d\n', vt_final_count);
fprintf('  经验节点: %d\n', NUM_EXPS);
if has_imu_data
    fprintf('  ✓ 使用真实IMU-视觉融合\n');
else
    fprintf('  ⚠️  纯视觉模式（未找到IMU数据）\n');
end

%% 6. 轨迹对齐
fprintf('[6/9] 对齐轨迹...\n');

if has_ground_truth
    % 裁剪到最短长度
    if has_imu_data
        min_len = min([size(fusion_data.pos, 1), size(gt_data.pos, 1), ...
                       size(pure_visual_traj, 1), size(imu_aided_traj, 1), size(exp_trajectory, 1)]);
    else
        min_len = min([size(fusion_data.pos, 1), size(gt_data.pos, 1), ...
                       size(pure_visual_traj, 1), size(exp_trajectory, 1)]);
    end
    
    fusion_trim = fusion_data.pos(1:min_len, :);
    gt_trim = gt_data.pos(1:min_len, :);
    pure_visual_trim = pure_visual_traj(1:min_len, :);
    exp_trim = exp_trajectory(1:min_len, :);
    
    % 对齐
    [fusion_aligned, gt_aligned] = align_trajectories(fusion_trim, gt_trim, 'simple');
    [pure_visual_aligned, ~] = align_trajectories(pure_visual_trim, gt_trim, 'simple');
    [exp_aligned, ~] = align_trajectories(exp_trim, gt_trim, 'simple');
    
    % 对齐IMU-aided轨迹（如果存在）
    if has_imu_data
        imu_aided_trim = imu_aided_traj(1:min_len, :);
        [imu_aided_aligned, ~] = align_trajectories(imu_aided_trim, gt_trim, 'simple');
    end
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
    
    plot_imu_visual_comparison_with_gt(fusion_data_aligned, pure_visual_aligned, exp_aligned, gt_data_aligned, result_path);
else
    plot_imu_visual_comparison(fusion_data, pure_visual_traj, exp_trajectory, [], result_path);
end

%% 8. 精度评估
fprintf('[8/9] 精度评估...\n');

if has_ground_truth
    fprintf('\n========== 相对于Ground Truth的精度评估 ==========\n');
    if has_imu_data
        fprintf('（使用真实IMU-视觉融合）\n');
    else
        fprintf('（纯视觉模式）\n');
    end
    
    fprintf('\n--- EKF Fusion (Baseline) vs Ground Truth ---\n');
    metrics_fusion = evaluate_slam_accuracy(fusion_aligned, gt_aligned, result_path, 'ekf_fusion_baseline');
    
    fprintf('\n--- Pure Visual Odometry (Baseline) vs Ground Truth ---\n');
    metrics_pure_visual = evaluate_slam_accuracy(pure_visual_aligned, gt_aligned, result_path, 'pure_visual_odometry');
    
    if has_imu_data
        fprintf('\n--- 时间戳对齐惯视融合（系统输入）vs Ground Truth ---\n');
        metrics_imu_aided = evaluate_slam_accuracy(imu_aided_aligned, gt_aligned, result_path, 'timestamp_aligned_fusion');
    end
    
    fprintf('\n--- Bio-inspired SLAM (完整系统) vs Ground Truth ---\n');
    metrics_exp = evaluate_slam_accuracy(exp_aligned, gt_aligned, result_path, 'bio_inspired_slam');
    
    % 打印汇总
    fprintf('\n========== 精度评估汇总 ==========\n');
    traj_length = sum(sqrt(sum(diff(gt_aligned).^2, 2)));
    fprintf('Ground Truth行程: %.2f m\n\n', traj_length);
    
    fprintf('EKF Fusion (Baseline):\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_fusion.ate.rmse, metrics_fusion.ate.mean, metrics_fusion.final_error);
    
    fprintf('Pure Visual Odometry (Baseline):\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_pure_visual.ate.rmse, metrics_pure_visual.ate.mean, metrics_pure_visual.final_error);
    
    if has_imu_data
        fprintf('时间戳对齐惯视融合（系统输入）:\n');
        fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
            metrics_imu_aided.ate.rmse, metrics_imu_aided.ate.mean, metrics_imu_aided.final_error);
        improvement = (metrics_pure_visual.ate.rmse - metrics_imu_aided.ate.rmse) / metrics_pure_visual.ate.rmse * 100;
        fprintf('  ✓ 相对纯视觉改进: %.1f%%\n', improvement);
    end
    
    fprintf('Bio-inspired SLAM (完整系统):\n');
    fprintf('  RMSE: %.3f m, Mean: %.3f m, End: %.3f m\n', ...
        metrics_exp.ate.rmse, metrics_exp.ate.mean, metrics_exp.final_error);
    
    fprintf('===================================\n\n');
end

%% 9. 保存结果和性能报告
fprintf('[9/9] 保存结果...\n');

if has_ground_truth && has_imu_data
    save(fullfile(result_path, 'euroc_trajectories.mat'), ...
        'fusion_data', 'pure_visual_traj', 'imu_aided_traj', 'exp_trajectory', 'gt_data', 'imu_data', ...
        'fusion_aligned', 'pure_visual_aligned', 'imu_aided_aligned', 'exp_aligned', 'gt_aligned', ...
        'has_imu_data');
elseif has_ground_truth
    save(fullfile(result_path, 'euroc_trajectories.mat'), ...
        'fusion_data', 'pure_visual_traj', 'imu_aided_traj', 'exp_trajectory', 'gt_data', ...
        'fusion_aligned', 'pure_visual_aligned', 'exp_aligned', 'gt_aligned', ...
        'has_imu_data');
else
    save(fullfile(result_path, 'euroc_trajectories.mat'), ...
        'fusion_data', 'pure_visual_traj', 'imu_aided_traj', 'exp_trajectory', ...
        'has_imu_data');
end

% 生成性能报告
if has_ground_truth && has_imu_data
    fprintf('生成性能对比报告...\n');
    report_file = fullfile(result_path, 'performance_report.txt');
    fid = fopen(report_file, 'w');
    
    fprintf(fid, '========================================\n');
    fprintf(fid, 'EuRoC IMU-Visual Fusion SLAM Performance Report\n');
    fprintf(fid, '========================================\n\n');
    
    fprintf(fid, '数据集信息:\n');
    fprintf(fid, '  数据集: %s\n', EUROC_DATA_PATH);
    fprintf(fid, '  总帧数: %d\n', num_frames);
    fprintf(fid, '  Ground Truth长度: %.2f m\n', sum(sqrt(sum(diff(gt_data.pos).^2, 2))));
    fprintf(fid, '\n');
    
    fprintf(fid, '精度评估 (RMSE):\n');
    fprintf(fid, '  EKF Fusion (Baseline): %.3f m\n', metrics_fusion.ate.rmse);
    fprintf(fid, '  Pure Visual Odometry: %.3f m\n', metrics_pure_visual.ate.rmse);
    fprintf(fid, '  IMU-aided VO (系统输入): %.3f m\n', metrics_imu_aided.ate.rmse);
    fprintf(fid, '  Bio-inspired SLAM: %.3f m\n', metrics_exp.ate.rmse);
    fprintf(fid, '\n');
    
    fprintf(fid, '终点误差:\n');
    fprintf(fid, '  EKF Fusion: %.3f m (漂移率: %.2f%%)\n', ...
        metrics_fusion.final_error, metrics_fusion.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100);
    fprintf(fid, '  Pure Visual: %.3f m (漂移率: %.2f%%)\n', ...
        metrics_pure_visual.final_error, metrics_pure_visual.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100);
    fprintf(fid, '  IMU-aided VO: %.3f m (漂移率: %.2f%%)\n', ...
        metrics_imu_aided.final_error, metrics_imu_aided.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100);
    fprintf(fid, '  Bio-inspired: %.3f m (漂移率: %.2f%%)\n', ...
        metrics_exp.final_error, metrics_exp.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100);
    fprintf(fid, '\n');
    
    fprintf(fid, '相对改进:\n');
    improvement_vs_visual = (metrics_pure_visual.ate.rmse - metrics_exp.ate.rmse) / metrics_pure_visual.ate.rmse * 100;
    improvement_vs_ekf = (metrics_fusion.ate.rmse - metrics_exp.ate.rmse) / metrics_fusion.ate.rmse * 100;
    fprintf(fid, '  Bio-SLAM vs Pure Visual: %.1f%%\n', improvement_vs_visual);
    fprintf(fid, '  Bio-SLAM vs EKF Fusion: %.1f%%\n', improvement_vs_ekf);
    fprintf(fid, '\n');
    
    fclose(fid);
    fprintf('性能报告已保存: %s\n', report_file);
end

%% 10. 生成综合对比报告（高级版）
if has_ground_truth && has_imu_data
    fprintf('[10/10] 生成综合对比报告...\n');
    
    % 调用高级对比报告生成函数
    try
        generate_comparison_report(result_path, 'EuRoC_MH');
        fprintf('✓ 综合对比报告已生成（6子图高级版）\n');
    catch ME
        fprintf('⚠️  高级报告生成失败: %s\n', ME.message);
        fprintf('   尝试生成简化版本...\n');
        
        % 回退到简化版本
        fig = figure('Position', [100, 100, 1600, 1200], 'Visible', 'off');
    
    % 子图1: 4条轨迹对比（XY平面）
    subplot(2,2,1);
    hold on;
    plot(gt_aligned(:,1), gt_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    plot(fusion_aligned(:,1), fusion_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF Fusion');
    plot(imu_aided_aligned(:,1), imu_aided_aligned(:,2), 'g--', 'LineWidth', 1.5, 'DisplayName', 'IMU-aided VO');
    plot(exp_aligned(:,1), exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Bio-inspired SLAM');
    hold off;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Trajectory Comparison (XY Plane)');
    legend('Location', 'best');
    grid on; axis equal;
    
    % 子图2: RMSE对比柱状图
    subplot(2,2,2);
    methods = {'EKF\nFusion', 'IMU-aided\nVO', 'Bio-inspired\nSLAM', 'Pure\nVisual'};
    rmse_values = [metrics_fusion.ate.rmse, metrics_imu_aided.ate.rmse, ...
                   metrics_exp.ate.rmse, metrics_pure_visual.ate.rmse];
    bar(rmse_values, 'FaceColor', [0.2 0.6 0.8]);
    set(gca, 'XTickLabel', methods);
    ylabel('RMSE (m)');
    title('Absolute Trajectory Error (RMSE)');
    grid on;
    for i = 1:length(rmse_values)
        text(i, rmse_values(i)+0.5, sprintf('%.2fm', rmse_values(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    % 子图3: 终点误差对比
    subplot(2,2,3);
    end_errors = [metrics_fusion.final_error, metrics_imu_aided.final_error, ...
                  metrics_exp.final_error, metrics_pure_visual.final_error];
    bar(end_errors, 'FaceColor', [0.8 0.4 0.2]);
    set(gca, 'XTickLabel', methods);
    ylabel('Final Error (m)');
    title('Final Position Error');
    grid on;
    for i = 1:length(end_errors)
        text(i, end_errors(i)+0.5, sprintf('%.2fm', end_errors(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    % 子图4: 性能总结表格
    subplot(2,2,4); axis off;
    summary_text = sprintf(['Performance Summary (EuRoC MH_01)\n\n' ...
        'Ground Truth Length: %.2f m\n\n' ...
        'EKF Fusion:\n  RMSE: %.2fm, End: %.2fm (%.2f%%)\n\n' ...
        'IMU-aided VO:\n  RMSE: %.2fm, End: %.2fm (%.2f%%) ✓Best\n\n' ...
        'Bio-inspired SLAM:\n  RMSE: %.2fm, End: %.2fm (%.2f%%)\n\n' ...
        'Pure Visual:\n  RMSE: %.2fm, End: %.2fm (%.2f%%)\n\n' ...
        'Improvements vs Pure Visual:\n' ...
        '  IMU-aided: %.1f%%\n' ...
        '  Bio-SLAM: %.1f%%'], ...
        sum(sqrt(sum(diff(gt_data.pos).^2, 2))), ...
        metrics_fusion.ate.rmse, metrics_fusion.final_error, ...
        metrics_fusion.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100, ...
        metrics_imu_aided.ate.rmse, metrics_imu_aided.final_error, ...
        metrics_imu_aided.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100, ...
        metrics_exp.ate.rmse, metrics_exp.final_error, ...
        metrics_exp.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100, ...
        metrics_pure_visual.ate.rmse, metrics_pure_visual.final_error, ...
        metrics_pure_visual.final_error/sum(sqrt(sum(diff(gt_data.pos).^2, 2)))*100, ...
        (metrics_pure_visual.ate.rmse - metrics_imu_aided.ate.rmse) / metrics_pure_visual.ate.rmse * 100, ...
        (metrics_pure_visual.ate.rmse - metrics_exp.ate.rmse) / metrics_pure_visual.ate.rmse * 100);
    
    text(0.1, 0.5, summary_text, 'FontSize', 10, 'FontName', 'FixedWidth', ...
        'VerticalAlignment', 'middle');
    
        % 保存图表（简化版）
        saveas(fig, fullfile(result_path, 'comprehensive_comparison_simple.png'));
        close(fig);
        fprintf('✓ 简化版对比图已保存: comprehensive_comparison_simple.png\n');
        
        % 生成文本报告
        report_txt = fullfile(result_path, 'comparison_report.txt');
        fid = fopen(report_txt, 'w');
        fprintf(fid, '%s\n', summary_text);
        fclose(fid);
        fprintf('✓ 文本报告已保存: comparison_report.txt\n');
    end
end

fprintf('\n✅ EuRoC SLAM测试完成！\n');
fprintf('结果保存在: %s\n', result_path);
if has_imu_data
    fprintf('✓ 系统对比：\n');
    fprintf('  - EKF Fusion (Baseline): fusion_data (外部Python生成)\n');
    fprintf('  - Pure Visual (Baseline): pure_visual_traj\n');
    fprintf('  - 时间戳对齐惯视融合（系统输入）: imu_aided_traj\n');
    fprintf('  - Bio-inspired SLAM（完整系统）: exp_trajectory\n');
else
    fprintf('⚠️  纯视觉模式（未找到IMU数据）\n');
    fprintf('  - Pure Visual: pure_visual_traj\n');
    fprintf('  - Bio-inspired SLAM: exp_trajectory\n');
end
