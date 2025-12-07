function [exp_trajectory, odo_trajectory, stats] = run_slam_with_vt_method(method, dataPath, imgFiles, num_frames, fusion_data, imu_data, rootDir)
%RUN_SLAM_WITH_VT_METHOD 使用指定VT方法运行SLAM
%   method: 'original' 或 'enhanced'
%   完全基于test_imu_visual_fusion_slam.m的逻辑

    %% 清理所有全局变量（关键！避免污染）
    clear global;
    
    %% 重新声明并初始化全局变量
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
    
    % 清理VO相关的全局变量
    global PREV_TRANS_V_IMG_X_SUMS; PREV_TRANS_V_IMG_X_SUMS = [];
    global PREV_TRANS_V_IMG_Y_SUMS; PREV_TRANS_V_IMG_Y_SUMS = [];
    global PREV_YAW_ROT_V_IMG_X_SUMS; PREV_YAW_ROT_V_IMG_X_SUMS = [];
    global PREV_YAW_ROT_V_IMG_Y_SUMS; PREV_YAW_ROT_V_IMG_Y_SUMS = [];
    global PREV_HEIGHT_V_IMG_X_SUMS; PREV_HEIGHT_V_IMG_X_SUMS = [];
    global PREV_HEIGHT_V_IMG_Y_SUMS; PREV_HEIGHT_V_IMG_Y_SUMS = [];
    
    % 清理VT相关的全局变量
    global VT_HISTORY; VT_HISTORY = [];
    global VT_HISTORY_FIRST; VT_HISTORY_FIRST = [];
    global VT_HISTORY_OLD; VT_HISTORY_OLD = [];
    global DIFFS_ALL_IMGS_VTS; DIFFS_ALL_IMGS_VTS = [];
    
    %% 初始化模块（与test_imu_visual_fusion_slam.m完全一致）
    fprintf('初始化模块参数...\n');
    
    % 视觉里程计
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
    
    % 视觉模板（根据方法调整阈值）
    if strcmp(method, 'enhanced')
        vt_threshold = 0.07;  % 手搓类脑模型最佳阈值（已验证）
        fprintf('  VT方法: 手搓类脑模型(对比度+边缘融合) (阈值: %.3f)\n', vt_threshold);
    else
        vt_threshold = 0.15;  % 原始patch normalization阈值
        fprintf('  VT方法: 原始patch normalization (阈值: %.3f)\n', vt_threshold);
    end
    
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
    
    % HDC
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
    
    % GC
    gc_initial( ...
        'GC_X_DIM', 36, 'GC_Y_DIM', 36, 'GC_Z_DIM', 36, ...
        'GC_EXCIT_X_DIM', 7, 'GC_EXCIT_Y_DIM', 7, 'GC_EXCIT_Z_DIM', 7, ...
        'GC_INHIB_X_DIM', 5, 'GC_INHIB_Y_DIM', 5, 'GC_INHIB_Z_DIM', 5, ...
        'GC_EXCIT_X_VAR', 1.5, 'GC_EXCIT_Y_VAR', 1.5, 'GC_EXCIT_Z_VAR', 1.5, ...
        'GC_INHIB_X_VAR', 2, 'GC_INHIB_Y_VAR', 2, 'GC_INHIB_Z_VAR', 2, ...
        'GC_GLOBAL_INHIB', 0.0002, ...
        'GC_VT_INJECT_ENERGY', 0.1, ...
        'GC_HORI_TRANS_V_SCALE', 0.8, ...
        'GC_VERT_TRANS_V_SCALE', 0.8, ...
        'GC_PACKET_SIZE', 4);
    
    % 经验地图
    exp_initial( ...
        'DELTA_EXP_GC_HDC_THRESHOLD', 10, ...  % 经验地图（优化参数）
        'EXP_LOOPS', 100, ...
        'EXP_CORRECTION', 0.5);
    
    %% 主处理循环（与test_imu_visual_fusion_slam.m完全一致）
    fprintf('开始处理 %d 帧...\n', num_frames);
    
    odo_trajectory = zeros(num_frames, 3);
    exp_trajectory = zeros(num_frames, 3);
    odo_x = 0; odo_y = 0; odo_z = 0;
    odo_yaw = 0; odo_height = 0;
    
    % 使用初始化函数（关键修复！）
    [curYawTheta, curHeightValue] = get_hdc_initial_value();
    [gcX, gcY, gcZ] = get_gc_initial_pos();
    
    % 调试：记录VT距离
    global DIFFS_ALL_IMGS_VTS;
    vt_diffs_log = [];
    
    tic;
    for frame_idx = 1:num_frames
        if mod(frame_idx, 500) == 1
            fprintf('  [%d/%d] VT:%d, 经验:%d\n', frame_idx, num_frames, NUM_VT, NUM_EXPS);
        end
        
        % 记录VT距离（每100帧）
        if mod(frame_idx, 100) == 0 && ~isempty(DIFFS_ALL_IMGS_VTS)
            vt_diffs_log = [vt_diffs_log; DIFFS_ALL_IMGS_VTS(end)];
        end
        
        % 读取图像
        img_path = fullfile(dataPath, imgFiles(frame_idx).name);
        rawImg = imread(img_path);
        
        % IMU辅助视觉里程计
        [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx);
        
        % 里程计累积
        odo_yaw = odo_yaw + yawRotV * DEGREE_TO_RADIAN;
        odo_height = odo_height + heightV;
        odo_x = odo_x + transV * cos(odo_yaw);
        odo_y = odo_y + transV * sin(odo_yaw);
        odo_z = odo_height;
        odo_trajectory(frame_idx, :) = [odo_x, odo_y, odo_z];
        
        % VT匹配（使用融合位姿）
        if frame_idx <= size(fusion_data.pos, 1)
            curr_x = fusion_data.pos(frame_idx, 1);
            curr_y = fusion_data.pos(frame_idx, 2);
            curr_z = fusion_data.pos(frame_idx, 3);
            curr_yaw = fusion_data.att(frame_idx, 3);
            curr_height = curr_z;
        else
            curr_x = gcX;
            curr_y = gcY;
            curr_z = gcZ;
            curr_yaw = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE * 180 / pi;
            curr_height = curHeightValue;
        end
        
        % 调用对应的VT函数
        if strcmp(method, 'enhanced')
            vtId = visual_template_neuro_matlab_only(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);
            if frame_idx == 1
                fprintf('  ✓ 使用增强VT: visual_template_neuro_matlab_only\n');
            end
        else
            vtId = visual_template(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);
            if frame_idx == 1
                fprintf('  ✓ 使用原始VT: visual_template\n');
            end
        end
        
        % HDC迭代
        yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
        [curYawTheta, curHeightValue] = get_current_yaw_height_value();
        
        % GC迭代
        curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
        gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
        [gcX, gcY, gcZ] = get_gc_xyz();
        
        % 经验地图更新
        exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);
        
        % 记录经验轨迹（每帧记录当前经验节点位置）
        if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0 && CUR_EXP_ID <= length(EXPERIENCES)
            exp_trajectory(frame_idx, :) = [EXPERIENCES(CUR_EXP_ID).x_exp, ...
                                             EXPERIENCES(CUR_EXP_ID).y_exp, ...
                                             EXPERIENCES(CUR_EXP_ID).z_exp];
        else
            exp_trajectory(frame_idx, :) = [0, 0, 0];
        end
        
        PREV_VT_ID = vtId;
    end
    elapsed_time = toc;
    
    %% 计算统计数据
    fprintf('处理完成！\n');
    fprintf('  VT数量: %d\n', NUM_VT);
    fprintf('  经验节点: %d\n', NUM_EXPS);
    fprintf('  处理时间: %.2f 秒\n', elapsed_time);
    
    % 调试：检查经验轨迹范围
    exp_valid = exp_trajectory(exp_trajectory(:,1) ~= 0 | exp_trajectory(:,2) ~= 0, :);
    if ~isempty(exp_valid)
        fprintf('  经验轨迹范围: X[%.2f, %.2f], Y[%.2f, %.2f]\n', ...
            min(exp_valid(:,1)), max(exp_valid(:,1)), ...
            min(exp_valid(:,2)), max(exp_valid(:,2)));
    end
    
    % 调试：VT距离统计
    if ~isempty(vt_diffs_log)
        fprintf('  VT距离统计: min=%.4f, max=%.4f, mean=%.4f\n', ...
            min(vt_diffs_log), max(vt_diffs_log), mean(vt_diffs_log));
        fprintf('  当前阈值: %.4f (%.1f%% 超过阈值)\n', vt_threshold, ...
            sum(vt_diffs_log > vt_threshold) / length(vt_diffs_log) * 100);
    end
    
    stats.num_vts = NUM_VT;
    stats.num_exps = NUM_EXPS;
    stats.time = elapsed_time;
    stats.rmse = NaN;  % RMSE在外部计算（对齐后）
end
