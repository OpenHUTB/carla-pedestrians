function result = run_single_ablation_experiment(exp_name, config)
%% 运行单个消融实验
% 输入:
%   exp_name - 实验名称
%   config - 实验配置结构体
% 输出:
%   result - 实验结果

%% 初始化路径
addpath(genpath('../../'));

%% 初始化全局变量
% 保存数据集名称
global DATASET_NAME;
saved_dataset = 'Town01Data_IMU_Fusion';  % 默认值
if ~isempty(DATASET_NAME)
    saved_dataset = DATASET_NAME;
end

% 清除之前的全局变量
clear global;

% 恢复数据集名称
global DATASET_NAME;
DATASET_NAME = saved_dataset;

% 直接初始化（不调用函数，避免依赖问题）
global GLOBAL_VARS;
GLOBAL_VARS = struct();

%% 根据配置设置参数
if config.full_feature
    % 使用完整HART+Transformer特征
    vt_threshold = 0.06;
else
    % 使用简化特征
    vt_threshold = 0.08;
end

%% 初始化各模块
% 视觉里程计
visual_odometry_initial( ...
    'MAX_YAW_ROT_V_THRESHOLD', 2.5, ...
    'MAX_HEIGHT_V_THRESHOLD', 0.45, ...
    'ODO_SHIFT_MATCH_HORI', 26, ...
    'ODO_SHIFT_MATCH_VERT', 20, ...
    'FOV_HORI_DEGREE', 81.5, ...
    'FOV_VERT_DEGREE', 50, ...
    'ODO_STEP', 1);

% 视觉模板（根据配置调整）
vt_image_initial('*.png', ...
    'VT_MATCH_THRESHOLD', vt_threshold, ...
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

% 设置特征提取配置
GLOBAL_VARS.ablation_config = config;

% 头部朝向细胞
yaw_height_hdc_initial( ...
    'YAW_HEIGHT_HDC_Y_DIM', 36, ...
    'YAW_HEIGHT_HDC_H_DIM', 36, ...
    'YAW_HEIGHT_HDC_EXCIT_Y_DIM', 8, ...
    'YAW_HEIGHT_HDC_EXCIT_H_DIM', 8, ...
    'YAW_HEIGHT_HDC_INHIB_Y_DIM', 5, ...
    'YAW_HEIGHT_HDC_INHIB_H_DIM', 5, ...
    'YAW_HEIGHT_HDC_EXCIT_Y_VAR', 1.9, ...
    'YAW_HEIGHT_HDC_EXCIT_H_VAR', 1.5, ...
    'YAW_HEIGHT_HDC_EXCIT_Y_WRAP', 1, ...
    'YAW_HEIGHT_HDC_EXCIT_H_WRAP', 0, ...
    'YAW_HEIGHT_HDC_Y_INHIB_WEIGHT', 0.15, ...
    'YAW_HEIGHT_HDC_H_INHIB_WEIGHT', 0.15);

% 经验地图
experience_map_initial( ...
    'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...
    'EXP_LOOPS', 50, ...
    'EXP_CORRECTION', 0.5, ...
    'MAX_GOALS', 5);

%% 读取数据
global DATASET_NAME;
data_dir = fullfile('..', '..', 'data', '01_NeuroSLAM_Datasets', DATASET_NAME);

% IMU数据
if config.imu
    imu_data = readtable(fullfile(data_dir, 'imu_data.csv'));
    fprintf('  加载IMU数据: %d条\n', height(imu_data));
else
    fprintf('  跳过IMU数据（纯视觉模式）\n');
    imu_data = [];
end

% 融合位姿
poses_data = readtable(fullfile(data_dir, 'fused_poses.csv'));
fprintf('  加载位姿数据: %d条\n', height(poses_data));

% Ground Truth
gt_data = readtable(fullfile(data_dir, 'ground_truth.csv'));
fprintf('  加载Ground Truth: %d条\n', height(gt_data));

% 图像列表
img_dir = fullfile(data_dir, 'images');
img_files = dir(fullfile(img_dir, '*.png'));
fprintf('  找到图像: %d张\n', length(img_files));

%% 运行SLAM
num_frames = min([height(poses_data), length(img_files), 5000]);
fprintf('  处理帧数: %d\n', num_frames);

% 初始化轨迹存储
slam_trajectory = zeros(num_frames, 3);
exp_trajectory = [];

for frame_idx = 1:num_frames
    % 显示进度
    if mod(frame_idx, 100) == 0
        fprintf('  进度: %d/%d (%.1f%%)\n', frame_idx, num_frames, frame_idx/num_frames*100);
    end
    
    % 读取图像
    img_path = fullfile(img_dir, img_files(frame_idx).name);
    img = imread(img_path);
    
    % 处理当前帧
    if config.imu && ~isempty(imu_data)
        % IMU-视觉融合模式
        imu_frame = imu_data(frame_idx, :);
        [vt_id, exp_id] = process_frame_with_imu(img, imu_frame, config);
    else
        % 纯视觉模式
        [vt_id, exp_id] = process_frame_visual_only(img, config);
    end
    
    % 记录位置
    if ~isempty(GLOBAL_VARS.CURRENT_EXP_ID) && GLOBAL_VARS.CURRENT_EXP_ID > 0
        exp_id = GLOBAL_VARS.CURRENT_EXP_ID;
        slam_trajectory(frame_idx, :) = [
            GLOBAL_VARS.EXP(exp_id).x_exp, ...
            GLOBAL_VARS.EXP(exp_id).y_exp, ...
            GLOBAL_VARS.EXP(exp_id).h_exp
        ];
    end
end

%% 提取经验地图轨迹
if isfield(GLOBAL_VARS, 'EXP') && ~isempty(GLOBAL_VARS.EXP)
    num_exp = length(GLOBAL_VARS.EXP);
    exp_trajectory = zeros(num_exp, 3);
    for i = 1:num_exp
        if isfield(GLOBAL_VARS.EXP(i), 'x_exp')
            exp_trajectory(i, :) = [
                GLOBAL_VARS.EXP(i).x_exp, ...
                GLOBAL_VARS.EXP(i).y_exp, ...
                GLOBAL_VARS.EXP(i).h_exp
            ];
        end
    end
end

%% 评估精度
gt_trajectory = [gt_data.x(1:num_frames), gt_data.y(1:num_frames), gt_data.z(1:num_frames)];

% 对齐轨迹
[aligned_exp, ~] = align_trajectories_simple(exp_trajectory(:, 1:2), gt_trajectory(:, 1:2));

% 计算误差
errors = sqrt(sum((aligned_exp - gt_trajectory(:, 1:2)).^2, 2));
rmse = sqrt(mean(errors.^2));

% 计算RPE（相对位姿误差）
rpe = calculate_rpe(aligned_exp, gt_trajectory(:, 1:2));

% 计算漂移率
trajectory_length = sum(sqrt(sum(diff(gt_trajectory(:, 1:2)).^2, 2)));
end_point_error = norm(aligned_exp(end, :) - gt_trajectory(end, 1:2));
drift_rate = (end_point_error / trajectory_length) * 100;

%% 输出结果
result = struct();
result.exp_name = exp_name;
result.config = config;
result.vt_count = length(GLOBAL_VARS.VT);
result.exp_count = length(GLOBAL_VARS.EXP);
result.rmse = rmse;
result.rpe = rpe;
result.drift_rate = drift_rate;
result.slam_trajectory = slam_trajectory;
result.exp_trajectory = exp_trajectory;
result.gt_trajectory = gt_trajectory;
result.aligned_trajectory = aligned_exp;

end

%% 辅助函数
function [vt_id, exp_id] = process_frame_with_imu(img, imu_frame, config)
    global GLOBAL_VARS;
    
    % 获取IMU数据
    accel = [imu_frame.accel_x, imu_frame.accel_y, imu_frame.accel_z];
    gyro = [imu_frame.gyro_x, imu_frame.gyro_y, imu_frame.gyro_z];
    
    % 更新位姿（简化）
    % 实际应该调用完整的IMU-视觉融合
    
    % 视觉处理
    [vt_id, exp_id] = visual_template_neuro_matlab_only(img);
    
    % 更新经验地图
    if vt_id > 0
        experience_map_iteration(vt_id);
    end
end

function [vt_id, exp_id] = process_frame_visual_only(img, config)
    global GLOBAL_VARS;
    
    % 纯视觉处理
    [vt_id, exp_id] = visual_template_neuro_matlab_only(img);
    
    % 更新经验地图
    if vt_id > 0
        experience_map_iteration(vt_id);
    end
end

function rpe = calculate_rpe(trajectory1, trajectory2)
    % 计算相对位姿误差
    if size(trajectory1, 1) ~= size(trajectory2, 1)
        min_len = min(size(trajectory1, 1), size(trajectory2, 1));
        trajectory1 = trajectory1(1:min_len, :);
        trajectory2 = trajectory2(1:min_len, :);
    end
    
    % 计算相邻帧之间的位移差异
    delta1 = diff(trajectory1);
    delta2 = diff(trajectory2);
    
    % RPE = 平均位移误差
    rpe_errors = sqrt(sum((delta1 - delta2).^2, 2));
    rpe = mean(rpe_errors);
end
