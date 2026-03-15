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
        imu_candidates = dir(fullfile(data_path, 'aligned_imu*.txt'));
        if ~isempty(imu_candidates)
            [~, idx] = sort({imu_candidates.name});
            imu_file = fullfile(data_path, imu_candidates(idx(1)).name);
        else
            error('IMU数据文件不存在: %s', imu_file);
        end
    end
    
    % 读取CSV格式数据 (timestamp,ax,ay,az,gx,gy,gz)
    try
        % 手动检查并跳过CSV头部和注释行
        fid = fopen(imu_file, 'r');
        if fid == -1
            error('无法打开文件: %s', imu_file);
        end
        
        % 计算需要跳过的行数
        skip_lines = 0;
        while ~feof(fid)
            line = fgetl(fid);
            
            % 检查文件结束
            if ~ischar(line)
                break;
            end
            
            % 跳过空行
            if isempty(line)
                skip_lines = skip_lines + 1;
                continue;
            end
            
            % 去除首尾空格
            line_trimmed = strtrim(line);
            
            % 跳过空行
            if isempty(line_trimmed)
                skip_lines = skip_lines + 1;
                continue;
            end
            
            % 跳过注释行（以%或#开头）
            first_char = line_trimmed(1);
            if first_char == '%'
                skip_lines = skip_lines + 1;
                continue;
            end
            if first_char == '#'
                skip_lines = skip_lines + 1;
                continue;
            end
            
            % 跳过CSV头部（包含文本）
            line_lower = lower(line_trimmed);
            if ~isempty(strfind(line_lower, 'timestamp'))
                skip_lines = skip_lines + 1;
                continue;
            end
            if ~isempty(strfind(line_lower, 'ax'))
                skip_lines = skip_lines + 1;
                continue;
            end
            if ~isempty(strfind(line_lower, 'accel'))
                skip_lines = skip_lines + 1;
                continue;
            end
            
            % 找到第一行数据，停止
            break;
        end
        fclose(fid);
        
        % 读取数据，跳过头部行
        if skip_lines > 0
            raw_data = dlmread(imu_file, ',', skip_lines, 0);
        else
            raw_data = dlmread(imu_file, ',');
        end
        
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
