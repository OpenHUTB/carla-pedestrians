%% 公平的IMU融合参数网格搜索
%  关键修复：Baseline和Ours使用相同的旋转角度对齐到GT
%  
%  问题分析：
%    之前的方法让Baseline和Ours各自独立搜索最优旋转角度
%    这导致Ours可能找到一个"假的"最优对齐，产生不合理的改进
%  
%  解决方案：
%    1. 先用Baseline找到最优旋转角度
%    2. Ours使用相同的旋转角度
%    3. 这样才是公平对比

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     公平的IMU融合参数网格搜索 (统一旋转角度)                 ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
alpha_yaw_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
alpha_trans_values = [0.0];  % 简化：只搜索alpha_yaw
alpha_height_values = [0.0];

USE_FAST_MODE = true;
FAST_FRAMES = 1500;

dataset_name = 'Town01Data_IMU_Fusion';

%% 初始化路径
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
fprintf('[1/4] 加载数据...\n');
data_path = fullfile(rootDir, 'data', dataset_name);
if ~exist(data_path, 'dir')
    data_path = fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets', dataset_name);
end

imu_data = read_imu_data(data_path);
gt_file = fullfile(data_path, 'ground_truth.txt');
gt_data = read_ground_truth(gt_file);
img_files = dir(fullfile(data_path, '*.png'));

num_frames = min([length(img_files), FAST_FRAMES]);
fprintf('  帧数: %d\n', num_frames);

%% 运行Baseline并找到最优旋转角度
fprintf('\n[2/4] 运行Baseline并找最优旋转...\n');
[baseline_traj, ~] = run_slam_get_trajectory(data_path, img_files, [], num_frames, rootDir, 'baseline');

% 准备GT
gt_pos = gt_data.pos(1:num_frames, 1:2);

% 找Baseline的最优旋转角度和镜像
[best_angle, best_flip, baseline_ate, baseline_aligned] = find_best_rotation(baseline_traj, gt_pos);
fprintf('  Baseline最优: 旋转%d度, 镜像=%d, ATE=%.2fm\n', best_angle, best_flip, baseline_ate);

%% 网格搜索（使用固定的旋转角度）
fprintf('\n[3/4] 网格搜索 (使用Baseline的旋转角度)...\n');

all_results = [];
result_idx = 0;

for alpha_yaw = alpha_yaw_values
    result_idx = result_idx + 1;
    
    fprintf('\n--- 测试 [%d/%d] alpha_yaw=%.2f ---\n', ...
        result_idx, length(alpha_yaw_values), alpha_yaw);
    
    % 设置全局变量
    global IMU_YAW_WEIGHT_OVERRIDE IMU_TRANS_WEIGHT_OVERRIDE IMU_HEIGHT_WEIGHT_OVERRIDE;
    IMU_YAW_WEIGHT_OVERRIDE = alpha_yaw;
    IMU_TRANS_WEIGHT_OVERRIDE = 0;
    IMU_HEIGHT_WEIGHT_OVERRIDE = 0;
    
    % 运行Ours SLAM
    [ours_traj, ours_vt] = run_slam_get_trajectory(data_path, img_files, imu_data, num_frames, rootDir, 'ours');
    
    % 使用与Baseline相同的旋转角度计算ATE
    ours_ate = compute_ate_with_fixed_rotation(ours_traj, gt_pos, best_angle, best_flip);
    
    % 计算改进
    improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
    
    % 保存结果
    r = struct();
    r.alpha_yaw = alpha_yaw;
    r.ate = ours_ate;
    r.improvement = improvement;
    r.vt_count = ours_vt;
    
    if isempty(all_results)
        all_results = r;
    else
        all_results(end+1) = r;
    end
    
    fprintf('  ATE: %.2f m, 改进: %+.2f%%, VT数: %d\n', ours_ate, improvement, ours_vt);
end

%% 显示结果
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║              公平网格搜索结果 (统一旋转角度)               ║\n');
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  Baseline旋转: %d度, 镜像=%d                               ║\n', best_angle, best_flip);
fprintf('║  Baseline ATE: %.2f m                                      ║\n', baseline_ate);
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  alpha_yaw │   ATE (m)  │  改进 (%%)  │ VT数               ║\n');
fprintf('╠════════════════════════════════════════════════════════════╣\n');

best_improvement = -inf;
best_idx = 1;

for i = 1:length(all_results)
    r = all_results(i);
    fprintf('║    %.2f    │  %7.2f   │  %+7.2f   │ %4d               ║\n', ...
        r.alpha_yaw, r.ate, r.improvement, r.vt_count);
    
    if r.improvement > best_improvement
        best_improvement = r.improvement;
        best_idx = i;
    end
end

fprintf('╠════════════════════════════════════════════════════════════╣\n');
best = all_results(best_idx);
fprintf('║  最优: alpha_yaw=%.2f, ATE=%.2fm, 改进=%+.2f%%             ║\n', ...
    best.alpha_yaw, best.ate, best.improvement);
fprintf('╚════════════════════════════════════════════════════════════╝\n');

%% 保存结果
results = struct();
results.baseline_ate = baseline_ate;
results.baseline_rotation = best_angle;
results.baseline_flip = best_flip;
results.all_results = all_results;
results.best = best;
results.num_frames = num_frames;

save_path = fullfile(data_path, 'fair_grid_search_results.mat');
save(save_path, 'results');
fprintf('\n结果已保存到: %s\n', save_path);

fprintf('\n✓ 公平网格搜索完成！\n');

%% ========== 辅助函数 ==========

function [traj, vt_count] = run_slam_get_trajectory(data_path, img_files, imu_data, num_frames, rootDir, mode)
% 运行SLAM并返回轨迹

    global DEGREE_TO_RADIAN RADIAN_TO_DEGREE;
    DEGREE_TO_RADIAN = pi / 180;
    RADIAN_TO_DEGREE = 180 / pi;
    
    % 清除全局变量
    clear global PREV_VT_ID VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD;
    clear global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    clear global YAW_HEIGHT_HDC GRIDCELLS EXPERIENCES NUM_EXPS CUR_EXP_ID;
    clear global PREV_TRANS_V PREV_YAW_ROT_V PREV_HEIGHT_V;
    clear global ACCUM_DELTA_X ACCUM_DELTA_Y ACCUM_DELTA_Z;
    
    if strcmp(mode, 'ours')
        clear hart_transformer_extractor;
        clear hart_transformer_extractor_ablation;
        clear imu_aided_visual_odometry;
    end
    
    % 初始化全局变量
    global PREV_VT_ID; PREV_VT_ID = -1;
    global VT; VT = [];
    global NUM_VT; NUM_VT = 0;
    global VT_HISTORY; VT_HISTORY = [];
    global VT_HISTORY_FIRST; VT_HISTORY_FIRST = [];
    global VT_HISTORY_OLD; VT_HISTORY_OLD = [];
    global MIN_DIFF_CURR_IMG_VTS; MIN_DIFF_CURR_IMG_VTS = [];
    global DIFFS_ALL_IMGS_VTS; DIFFS_ALL_IMGS_VTS = [];
    global SUB_VT_IMG; SUB_VT_IMG = [];
    global YAW_HEIGHT_HDC; YAW_HEIGHT_HDC = zeros(36, 36);
    global GRIDCELLS; GRIDCELLS = zeros(36, 36, 36);
    global EXPERIENCES; EXPERIENCES = [];
    global NUM_EXPS; NUM_EXPS = 0;
    global CUR_EXP_ID; CUR_EXP_ID = 0;
    global PREV_TRANS_V; PREV_TRANS_V = 0;
    global PREV_YAW_ROT_V; PREV_YAW_ROT_V = 0;
    global PREV_HEIGHT_V; PREV_HEIGHT_V = 0;
    global YAW_HEIGHT_HDC_Y_TH_SIZE; YAW_HEIGHT_HDC_Y_TH_SIZE = 2*pi/36;
    
    if strcmp(mode, 'ours')
        global USE_TRANSFORMER_OVERRIDE USE_DUAL_STREAM_OVERRIDE;
        USE_TRANSFORMER_OVERRIDE = true;
        USE_DUAL_STREAM_OVERRIDE = true;
    end
    
    % 初始化模块
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
    
    vt_image_initial('*.png', ...
        'VT_MATCH_THRESHOLD', 0.06, ...
        'VT_IMG_CROP_Y_RANGE', 1:120, ...
        'VT_IMG_CROP_X_RANGE', 1:160, ...
        'VT_IMG_RESIZE_X_RANGE', 16, ...
        'VT_IMG_RESIZE_Y_RANGE', 12, ...
        'VT_IMG_X_SHIFT', 5, ...
        'VT_IMG_Y_SHIFT', 3, ...
        'VT_GLOBAL_DECAY', 0.08, ...
        'VT_ACTIVE_DECAY', 2.5, ...
        'PATCH_SIZE_Y_K', 5, ...
        'PATCH_SIZE_X_K', 5, ...
        'VT_PANORAMIC', 0, ...
        'VT_STEP', 1);
    
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
    
    gc_initial( ...
        'GC_X_DIM', 36, 'GC_Y_DIM', 36, 'GC_Z_DIM', 36, ...
        'GC_EXCIT_X_DIM', 7, 'GC_EXCIT_Y_DIM', 7, 'GC_EXCIT_Z_DIM', 7, ...
        'GC_INHIB_X_DIM', 5, 'GC_INHIB_Y_DIM', 5, 'GC_INHIB_Z_DIM', 5, ...
        'GC_EXCIT_X_VAR', 1.5, 'GC_EXCIT_Y_VAR', 1.5, 'GC_EXCIT_Z_VAR', 1.5, ...
        'GC_INHIB_X_VAR', 2, 'GC_INHIB_Y_VAR', 2, 'GC_INHIB_Z_VAR', 2, ...
        'GC_GLOBAL_INHIB', 0.0002, 'GC_VT_INJECT_ENERGY', 0.5, ...
        'GC_HORI_TRANS_V_SCALE', 0.8, 'GC_VERT_TRANS_V_SCALE', 0.8, ...
        'GC_PACKET_SIZE', 4);
    
    exp_initial( ...
        'DELTA_EXP_GC_HDC_THRESHOLD', 18, ...
        'EXP_LOOPS', 3, ...
        'EXP_CORRECTION', 0.12);
    
    % 运行SLAM
    exp_traj = zeros(num_frames, 3);
    odo_x = 0; odo_y = 0; odo_z = 0;
    odo_yaw = 0; odo_height = 0;
    bio_trans_gain = 1.2;
    
    [curYawTheta, curHeightValue] = get_hdc_initial_value();
    [gcX, gcY, gcZ] = get_gc_initial_pos();
    
    last_valid_img = [];
    
    for frame_idx = 1:num_frames
        if mod(frame_idx, 500) == 0
            fprintf('    %s: %d/%d\n', mode, frame_idx, num_frames);
        end
        
        img_path = fullfile(data_path, img_files(frame_idx).name);
        try
            rawImg = imread(img_path);
        catch
            if ~isempty(last_valid_img)
                rawImg = last_valid_img;
            else
                rawImg = uint8(zeros(240, 320));
            end
        end
        if size(rawImg, 3) == 3
            rawImg = rgb2gray(rawImg);
        end
        last_valid_img = rawImg;
        
        if strcmp(mode, 'baseline')
            [transV, yawRotV, heightV] = visual_odometry(rawImg);
        else
            [visual_transV, visual_yawRotV, visual_heightV] = visual_odometry(rawImg);
            [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx, ...
                visual_transV, visual_yawRotV, visual_heightV);
        end
        
        transV = transV * bio_trans_gain;
        
        odo_yaw = odo_yaw + yawRotV * DEGREE_TO_RADIAN;
        odo_height = odo_height + heightV;
        odo_x = odo_x + transV * cos(odo_yaw);
        odo_y = odo_y + transV * sin(odo_yaw);
        odo_z = odo_height;
        
        if strcmp(mode, 'baseline')
            vtId = visual_template_baseline(rawImg, gcX, gcY, gcZ, curYawTheta, curHeightValue);
        else
            vtId = visual_template_neuro_matlab_only(rawImg, gcX, gcY, gcZ, curYawTheta, curHeightValue);
        end
        
        yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
        [curYawTheta, curHeightValue] = get_current_yaw_height_value();
        curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
        
        gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
        [gcX, gcY, gcZ] = get_gc_xyz();
        
        exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, ...
                          gcX, gcY, gcZ, curYawTheta, curHeightValue);
        
        if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0 && CUR_EXP_ID <= length(EXPERIENCES)
            global ACCUM_DELTA_X ACCUM_DELTA_Y ACCUM_DELTA_Z;
            exp_base = [EXPERIENCES(CUR_EXP_ID).x_exp, EXPERIENCES(CUR_EXP_ID).y_exp, EXPERIENCES(CUR_EXP_ID).z_exp];
            if exist('ACCUM_DELTA_X', 'var') && ~isempty(ACCUM_DELTA_X)
                exp_traj(frame_idx, :) = exp_base + [ACCUM_DELTA_X, ACCUM_DELTA_Y, ACCUM_DELTA_Z];
            else
                exp_traj(frame_idx, :) = exp_base;
            end
        else
            exp_traj(frame_idx, :) = [odo_x, odo_y, odo_z];
        end
        
        PREV_VT_ID = vtId;
    end
    
    traj = exp_traj(:, 1:2);
    vt_count = NUM_VT;
end

function [best_angle, best_flip, best_ate, best_aligned] = find_best_rotation(traj, gt)
% 找最优旋转角度和镜像
    angles = 0:5:355;
    flips = [false, true];
    
    best_ate = inf;
    best_angle = 0;
    best_flip = false;
    best_aligned = traj;
    
    for do_flip = flips
        for angle = angles
            t = traj;
            
            if do_flip
                t(:,1) = -t(:,1);
            end
            
            theta = angle * pi / 180;
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            t = (R * t')';
            
            aligned = align_sim3(t, gt);
            
            errors = sqrt(sum((aligned - gt).^2, 2));
            ate = mean(errors);
            
            if ate < best_ate
                best_ate = ate;
                best_angle = angle;
                best_flip = do_flip;
                best_aligned = aligned;
            end
        end
    end
end

function ate = compute_ate_with_fixed_rotation(traj, gt, angle, do_flip)
% 使用固定旋转角度计算ATE
    t = traj;
    
    if do_flip
        t(:,1) = -t(:,1);
    end
    
    theta = angle * pi / 180;
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    t = (R * t')';
    
    aligned = align_sim3(t, gt);
    
    errors = sqrt(sum((aligned - gt).^2, 2));
    ate = mean(errors);
end

function aligned = align_sim3(traj, ref)
% Sim3对齐
    tc = mean(traj, 1);
    rc = mean(ref, 1);
    
    t_c = traj - tc;
    r_c = ref - rc;
    
    s = sqrt(sum(r_c(:).^2) / sum(t_c(:).^2));
    t_s = t_c * s;
    
    H = t_s' * r_c;
    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0
        V(:,end) = -V(:,end);
        R = V * U';
    end
    
    aligned = (R * t_s')' + rc;
end
