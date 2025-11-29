function [imu_data] = read_imu_data(data_path)
%READ_IMU_DATA 读取IMU数据文件
%   从aligned_imu.txt读取时间戳、加速度计和陀螺仪数据
%   
%   输入:
%       data_path - 数据文件夹路径
%   输出:
%       imu_data - 结构体数组,包含:
%           timestamp - 时间戳
%           accel - 加速度 [ax, ay, az] (m/s^2)
%           gyro - 角速度 [gx, gy, gz] (rad/s)
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   IMU Integration Module (2024)

    imu_file = fullfile(data_path, 'aligned_imu.txt');
    
    if ~exist(imu_file, 'file')
        error('IMU数据文件不存在: %s', imu_file);
    end
    
    % 读取CSV格式数据 (timestamp,ax,ay,az,gx,gy,gz)
    try
        raw_data = dlmread(imu_file, ',');
        
        % 解析数据
        imu_data.timestamp = raw_data(:, 1);
        imu_data.accel = raw_data(:, 2:4);  % [ax, ay, az]
        imu_data.gyro = raw_data(:, 5:7);   % [gx, gy, gz]
        imu_data.count = size(raw_data, 1);
        
        fprintf('成功读取 %d 条IMU数据\n', imu_data.count);
        
        % 计算统计信息
        imu_data.stats.accel_mean = mean(imu_data.accel);
        imu_data.stats.accel_std = std(imu_data.accel);
        imu_data.stats.gyro_mean = mean(imu_data.gyro);
        imu_data.stats.gyro_std = std(imu_data.gyro);
        
        fprintf('加速度计均值: [%.3f, %.3f, %.3f] m/s^2\n', ...
            imu_data.stats.accel_mean(1), ...
            imu_data.stats.accel_mean(2), ...
            imu_data.stats.accel_mean(3));
        fprintf('陀螺仪均值: [%.3f, %.3f, %.3f] rad/s\n', ...
            imu_data.stats.gyro_mean(1), ...
            imu_data.stats.gyro_mean(2), ...
            imu_data.stats.gyro_mean(3));
    catch ME
        error('读取IMU数据失败: %s', ME.message);
    end
end
