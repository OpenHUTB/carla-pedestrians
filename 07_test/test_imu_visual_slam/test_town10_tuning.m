%% Town10 SLAM参数调优脚本
% 测试不同VT阈值对Town10性能的影响

clear all
close all
clc

% 测试配置
test_configs = [
    % [VT_THRESHOLD, USE_HART, 描述]
    struct('threshold', 0.05, 'use_hart', true, 'name', 'HART_0.05');
    struct('threshold', 0.04, 'use_hart', true, 'name', 'HART_0.04');
    struct('threshold', 0.09, 'use_hart', false, 'name', 'Simple_0.09');
    struct('threshold', 0.085, 'use_hart', false, 'name', 'Simple_0.085');
];

results = cell(length(test_configs), 1);

for i = 1:length(test_configs)
    cfg = test_configs(i);
    fprintf('\n========================================\n');
    fprintf('测试配置 %d/%d: %s\n', i, length(test_configs), cfg.name);
    fprintf('  VT阈值: %.3f\n', cfg.threshold);
    fprintf('  特征方法: %s\n', iif(cfg.use_hart, 'HART+CORnet', '简单特征'));
    fprintf('========================================\n\n');
    
    try
        % 运行SLAM
        result = run_slam_town10(cfg.threshold, cfg.use_hart);
        results{i} = result;
        
        % 保存中间结果
        save(sprintf('town10_tuning_%s.mat', cfg.name), 'result', 'cfg');
        
        fprintf('\n结果摘要:\n');
        fprintf('  VT数量: %d\n', result.num_vt);
        fprintf('  经验节点: %d\n', result.num_exp);
        fprintf('  RMSE: %.2f m\n', result.rmse);
        fprintf('  轨迹完整性: %.1f%%\n', result.trajectory_completeness * 100);
        fprintf('  轨迹误差: %.2f%%\n', result.trajectory_error * 100);
        
    catch ME
        fprintf('❌ 配置失败: %s\n', ME.message);
        results{i} = struct('error', ME.message);
    end
end

%% 汇总对比
fprintf('\n\n========================================\n');
fprintf('Town10 参数调优汇总\n');
fprintf('========================================\n\n');
fprintf('%-15s | %5s | %5s | %8s | %8s | %8s\n', ...
    '配置', 'VT', '经验', 'RMSE(m)', '完整性', '误差(%)');
fprintf('%s\n', repmat('-', 1, 70));

for i = 1:length(results)
    if isfield(results{i}, 'error')
        fprintf('%-15s | ERROR\n', test_configs(i).name);
    else
        r = results{i};
        fprintf('%-15s | %5d | %5d | %8.2f | %7.1f%% | %7.2f%%\n', ...
            test_configs(i).name, ...
            r.num_vt, r.num_exp, r.rmse, ...
            r.trajectory_completeness * 100, ...
            r.trajectory_error * 100);
    end
end

fprintf('\n保存完整结果: town10_tuning_summary.mat\n');
save('town10_tuning_summary.mat', 'results', 'test_configs');


%% 辅助函数
function result = run_slam_town10(vt_threshold, use_hart)
    % 运行Town10 SLAM的核心函数
    
    % 添加路径
    rootDir = fileparts(fileparts(pwd));
    addpath(genpath(fullfile(rootDir, '00_tools')));
    addpath(genpath(fullfile(rootDir, '01_conjunctive_pose_cells_network')));
    addpath(genpath(fullfile(rootDir, '02_multilayered_experience_map')));
    addpath(genpath(fullfile(rootDir, '03_visual_odometry')));
    addpath(genpath(fullfile(rootDir, '04_visual_template')));
    addpath(genpath(fullfile(rootDir, '09_vestibular')));
    
    % 全局变量初始化
    global_initial();
    
    % VT参数
    global VT_MATCH_THRESHOLD;
    VT_MATCH_THRESHOLD = vt_threshold;
    
    % 特征提取方法
    global USE_HART_CORNET;
    USE_HART_CORNET = use_hart;
    
    % 其他参数（与原脚本一致）
    vt_initial( ...
        'VT_IMG_CROP_Y_RANGE', 1:120, ...
        'VT_IMG_CROP_X_RANGE', 1:160, ...
        'VT_IMG_RESIZE_X_RANGE', 16, ...
        'VT_IMG_RESIZE_Y_RANGE', 12, ...
        'VT_IMG_X_SHIFT', 5, ...
        'VT_IMG_Y_SHIFT', 3, ...
        'VT_IMG_HALF_OFFSET', 8, ...
        'VT_MATCH_THRESHOLD', vt_threshold, ...
        'VT_GLOBAL_DECAY', 0.1, ...
        'VT_ACTIVE_DECAY', 2.0, ...
        'VT_PANORAMIC', 0);
    
    yaw_height_hdc_initial( ...
        'YAW_HEIGHT_HDC_Y_DIM', 36, ...
        'YAW_HEIGHT_HDC_H_DIM', 36, ...
        'YAW_HEIGHT_HDC_Y_TH_SIZE', 2 * pi / 36, ...
        'YAW_HEIGHT_HDC_H_TH_SIZE', 2 * pi / 36);
    
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
    
    exp_initial( ...
        'DELTA_EXP_GC_HDC_THRESHOLD', 15, ...
        'EXP_LOOPS', 1, ...
        'EXP_CORRECTION', 0.5);
    
    % 读取数据
    data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion');
    imu_data = read_imu_data(data_path);
    fusion_data = read_fusion_pose(data_path);
    gt_file = fullfile(data_path, 'ground_truth.txt');
    gt_data = read_ground_truth(gt_file);
    
    % 获取图像列表
    img_files = dir(fullfile(data_path, '*.png'));
    num_frames = length(img_files);
    
    % 运行SLAM主循环
    odo_trajectory = zeros(num_frames, 3);
    feature_times = zeros(num_frames, 1);
    
    for frame_idx = 1:num_frames
        % 读取图像
        img_path = fullfile(data_path, img_files(frame_idx).name);
        rawImg = imread(img_path);
        
        % IMU辅助视觉里程计
        [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx);
        
        % 视觉模板匹配（计时特征提取）
        tic;
        if use_hart
            [curVTId, ~] = visual_template_hart_cornet(rawImg, transV);
        else
            [curVTId, ~] = visual_template_neuro_matlab_only(rawImg, transV);
        end
        feature_times(frame_idx) = toc;
        
        % 经验地图迭代
        [curExpId, ~] = exp_map_iteration(curVTId, transV, yawRotV, heightV);
        
        % 记录里程计轨迹
        odo_trajectory(frame_idx, :) = [fusion_data.pos(frame_idx, 1:2), 0];
    end
    
    % 提取经验轨迹
    global EXPERIENCES NUM_EXPS;
    exp_trajectory = zeros(NUM_EXPS, 3);
    for i = 1:NUM_EXPS
        exp_trajectory(i, :) = [EXPERIENCES(i).x_exp, EXPERIENCES(i).y_exp, EXPERIENCES(i).z_exp];
    end
    
    % 轨迹对齐
    [exp_traj_aligned, gt_pos_aligned] = align_trajectories_simple(exp_trajectory, gt_data.pos);
    
    % 评估指标
    metrics = evaluate_slam_metrics_simple(exp_traj_aligned, gt_pos_aligned, gt_data);
    
    % 整理结果
    global VISUAL_TEMPLATES;
    result.num_vt = length(VISUAL_TEMPLATES);
    result.num_exp = NUM_EXPS;
    result.rmse = metrics.rmse;
    result.trajectory_completeness = metrics.trajectory_length_est / metrics.trajectory_length_gt;
    result.trajectory_error = metrics.trajectory_length_error_pct / 100;
    result.drift_rate = metrics.drift_rate;
    result.avg_feature_time = mean(feature_times);
end

function metrics = evaluate_slam_metrics_simple(slam_traj, gt_traj, gt_data)
    % 简化的评估函数
    
    % ATE
    errors = sqrt(sum((slam_traj - gt_traj).^2, 2));
    metrics.rmse = sqrt(mean(errors.^2));
    
    % 轨迹长度
    slam_dist = sum(sqrt(sum(diff(slam_traj).^2, 2)));
    gt_dist = sum(sqrt(sum(diff(gt_traj).^2, 2)));
    
    metrics.trajectory_length_est = slam_dist;
    metrics.trajectory_length_gt = gt_dist;
    metrics.trajectory_length_error = abs(slam_dist - gt_dist);
    metrics.trajectory_length_error_pct = metrics.trajectory_length_error / gt_dist * 100;
    
    % 漂移率
    metrics.endpoint_error = norm(slam_traj(end, :) - gt_traj(end, :));
    metrics.drift_rate = metrics.endpoint_error / slam_dist * 100;
end

function out = iif(condition, true_val, false_val)
    if condition
        out = true_val;
    else
        out = false_val;
    end
end
