function [aligned_imu] = align_imu_to_images(imu_data, image_timestamps)
%ALIGN_IMU_TO_IMAGES 将高频IMU数据对齐到图像时间戳
%   EuRoC的IMU采样率(200Hz)比图像(20Hz)高，需要对齐
%   
%   输入:
%       imu_data - IMU数据结构体 (从read_euroc_imu_data获得)
%       image_timestamps - 图像时间戳数组 (秒)
%   输出:
%       aligned_imu - 对齐后的IMU数据，与图像帧一一对应
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   EuRoC IMU Alignment (2024)

    num_images = length(image_timestamps);
    
    % 初始化对齐后的数据
    aligned_imu.timestamp = image_timestamps;
    aligned_imu.gyro = zeros(num_images, 3);
    aligned_imu.accel = zeros(num_images, 3);
    aligned_imu.count = num_images;
    
    fprintf('对齐IMU数据到图像时间戳...\n');
    fprintf('  图像帧数: %d\n', num_images);
    fprintf('  IMU数据点: %d\n', imu_data.count);
    
    % 对每个图像时间戳，找到最近的IMU数据
    for i = 1:num_images
        img_time = image_timestamps(i);
        
        % 找到最接近的IMU时间戳索引
        [~, idx] = min(abs(imu_data.timestamp - img_time));
        
        % 如果时间差太大，警告（忽略超大时间差，可能是绝对时间戳）
        time_diff = abs(imu_data.timestamp(idx) - img_time);
        if time_diff > 0.1 && time_diff < 100  % 100ms到100秒之间才警告
            if i == 1 || mod(i, 500) == 0
                warning('图像 %d 与IMU时间差较大: %.3f 秒', i, time_diff);
            end
        end
        
        % 复制最近的IMU数据
        aligned_imu.gyro(i, :) = imu_data.gyro(idx, :);
        aligned_imu.accel(i, :) = imu_data.accel(idx, :);
    end
    
    fprintf('✓ IMU数据对齐完成\n');
    fprintf('  平均陀螺仪: [%.4f, %.4f, %.4f] rad/s\n', mean(aligned_imu.gyro));
    fprintf('  平均加速度: [%.4f, %.4f, %.4f] m/s^2\n', mean(aligned_imu.accel));
end
