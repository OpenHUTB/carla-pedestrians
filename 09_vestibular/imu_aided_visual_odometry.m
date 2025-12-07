function [transV, yawRotV, heightV] = imu_aided_visual_odometry(rawImg, imu_data, frame_idx)
%IMU_AIDED_VISUAL_ODOMETRY IMU辅助的视觉里程计
%   使用IMU数据增强视觉里程计的精度
%   
%   输入:
%       rawImg - 原始图像
%       imu_data - IMU数据结构体(可选,如果为空则退化为纯视觉)
%       frame_idx - 当前帧索引
%   输出:
%       transV - 平移速度
%       yawRotV - 偏航旋转速度 (degrees)
%       heightV - 高度变化速度
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   IMU-Visual Odometry Integration (2024)

    persistent prev_frame_idx;
    persistent imu_yaw_vel;
    persistent imu_trans_vel;
    persistent imu_height_vel;
    
    % 初始化
    if isempty(prev_frame_idx)
        prev_frame_idx = 0;
        imu_yaw_vel = 0;
        imu_trans_vel = 0;
        imu_height_vel = 0;
    end
    
    % 首先调用原始视觉里程计
    [visual_transV, visual_yawRotV, visual_heightV] = visual_odometry(rawImg);
    
    % 如果没有IMU数据,直接返回视觉里程计结果
    if isempty(imu_data) || frame_idx > length(imu_data.timestamp)
        transV = visual_transV;
        yawRotV = visual_yawRotV;
        heightV = visual_heightV;
        return;
    end
    
    % 获取当前帧的IMU数据
    if frame_idx ~= prev_frame_idx && frame_idx <= imu_data.count
        % 提取陀螺仪数据 (rad/s)
        gyro = imu_data.gyro(frame_idx, :);
        accel = imu_data.accel(frame_idx, :);
        
        % 计算时间间隔
        if prev_frame_idx > 0 && prev_frame_idx < imu_data.count
            dt = imu_data.timestamp(frame_idx) - imu_data.timestamp(prev_frame_idx);
        else
            dt = 0.05;  % 默认20Hz
        end
        
        % 从陀螺仪计算偏航速度 (转换为degrees/s)
        imu_yaw_vel = gyro(3) * 180 / pi;  % rad/s -> deg/s
        
        % 从加速度计估计平移速度(简化版,需要积分)
        % 注意:这里使用加速度的模长作为速度变化的指示
        accel_magnitude = sqrt(sum(accel.^2));
        imu_trans_vel = max(0, (accel_magnitude - 9.81) * dt * 0.5);  % 简化的速度估计（修正系数）
        
        % 从加速度计Z轴估计高度变化
        imu_height_vel = (accel(3) - 9.81) * dt * 0.3;  % 修正高度系数
        
        prev_frame_idx = frame_idx;
    end
    
    % IMU-视觉融合策略
    % 使用互补滤波器融合IMU和视觉里程计
    
    % 偏航速度融合 (IMU更可靠用于旋转)
    alpha_yaw = 0.7;  % IMU权重
    if abs(visual_yawRotV) > 0.1  % 视觉检测到旋转
        yawRotV = alpha_yaw * imu_yaw_vel + (1 - alpha_yaw) * visual_yawRotV;
    else
        yawRotV = imu_yaw_vel;  % 视觉不可靠时完全依赖IMU
    end
    
    % 平移速度融合 (视觉更可靠用于平移)
    alpha_trans = 0.3;  % IMU权重较低
    if visual_transV > 0.01  % 视觉检测到运动
        transV = alpha_trans * imu_trans_vel + (1 - alpha_trans) * visual_transV;
    else
        transV = visual_transV;  % 平移主要依赖视觉
    end
    
    % 高度变化融合
    alpha_height = 0.5;  % 平衡权重
    if abs(visual_heightV) > 0.01
        heightV = alpha_height * imu_height_vel + (1 - alpha_height) * visual_heightV;
    else
        heightV = visual_heightV;
    end
    
    % 限制输出范围
    global MAX_TRANS_V_THRESHOLD;
    global MAX_YAW_ROT_V_THRESHOLD;
    global MAX_HEIGHT_V_THRESHOLD;
    
    if abs(yawRotV) > MAX_YAW_ROT_V_THRESHOLD
        yawRotV = sign(yawRotV) * MAX_YAW_ROT_V_THRESHOLD;
    end
    if transV > MAX_TRANS_V_THRESHOLD
        transV = MAX_TRANS_V_THRESHOLD;
    end
    if abs(heightV) > MAX_HEIGHT_V_THRESHOLD
        heightV = sign(heightV) * MAX_HEIGHT_V_THRESHOLD;
    end
end
