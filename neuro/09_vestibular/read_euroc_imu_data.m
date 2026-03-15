function [imu_data] = read_euroc_imu_data(data_path)
%READ_EUROC_IMU_DATA 读取EuRoC MAV数据集的IMU数据
%   从mav0/imu0/data.csv读取IMU原始数据
%   
%   输入:
%       data_path - EuRoC数据集根目录路径
%   输出:
%       imu_data - 结构体,包含:
%           timestamp - 时间戳 (秒，转换自纳秒)
%           gyro - 陀螺仪数据 [w_x, w_y, w_z] (rad/s)
%           accel - 加速度计数据 [a_x, a_y, a_z] (m/s^2)
%           count - 数据点数量
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   EuRoC IMU Support (2024)

    % EuRoC IMU数据文件路径
    imu_file = fullfile(data_path, 'mav0', 'imu0', 'data.csv');
    
    if ~exist(imu_file, 'file')
        % 尝试备用路径（如果数据已经预处理）
        imu_file_alt = fullfile(data_path, 'aligned_imu.txt');
        if exist(imu_file_alt, 'file')
            fprintf('使用预处理的IMU数据: %s\n', imu_file_alt);
            imu_data = read_aligned_imu(imu_file_alt);
            return;
        else
            error('IMU数据文件不存在: %s\n请检查EuRoC数据集是否完整', imu_file);
        end
    end
    
    fprintf('读取EuRoC原始IMU数据: %s\n', imu_file);
    
    % 读取CSV数据
    % 格式: #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
    %       a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
    try
        % 打开文件
        fid = fopen(imu_file, 'r');
        
        % 跳过第一行（标题）
        header = fgetl(fid);
        
        % 读取数据
        data = textscan(fid, '%f64 %f %f %f %f %f %f', 'Delimiter', ',');
        fclose(fid);
        
        % 提取数据
        timestamps_ns = data{1};  % 纳秒
        gyro_x = data{2};
        gyro_y = data{3};
        gyro_z = data{4};
        accel_x = data{5};
        accel_y = data{6};
        accel_z = data{7};
        
        % 转换时间戳从纳秒到秒
        imu_data.timestamp = timestamps_ns * 1e-9;
        
        % 陀螺仪数据 (rad/s)
        imu_data.gyro = [gyro_x, gyro_y, gyro_z];
        
        % 加速度计数据 (m/s^2)
        imu_data.accel = [accel_x, accel_y, accel_z];
        
        % 数据点数量
        imu_data.count = length(imu_data.timestamp);
        
        fprintf('✓ 成功读取 %d 条IMU数据\n', imu_data.count);
        fprintf('  采样频率: ~%.1f Hz\n', 1 / mean(diff(imu_data.timestamp)));
        fprintf('  时间范围: %.2f - %.2f 秒\n', ...
            imu_data.timestamp(1), imu_data.timestamp(end));
        
        % 计算基本统计信息
        gyro_mean = mean(imu_data.gyro);
        accel_mean = mean(imu_data.accel);
        fprintf('  陀螺仪均值: [%.4f, %.4f, %.4f] rad/s\n', gyro_mean);
        fprintf('  加速度计均值: [%.4f, %.4f, %.4f] m/s^2\n', accel_mean);
        
    catch ME
        error('读取EuRoC IMU数据失败: %s', ME.message);
    end
end


function [imu_data] = read_aligned_imu(imu_file)
%READ_ALIGNED_IMU 读取预处理的对齐IMU数据
%   向后兼容函数，读取aligned_imu.txt格式
    
    try
        fid = fopen(imu_file, 'r');
        
        % 跳过第一行标题（如果存在）
        first_line = fgetl(fid);
        fclose(fid);
        
        fid = fopen(imu_file, 'r');
        if contains(first_line, 'timestamp') || contains(first_line, 'gyro')
            fgetl(fid);  % 跳过标题
        end
        
        % 读取数据：timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        raw_data = textscan(fid, '%f %f %f %f %f %f %f', 'Delimiter', ',');
        fclose(fid);
        
        imu_data.timestamp = raw_data{1};
        imu_data.accel = [raw_data{2}, raw_data{3}, raw_data{4}];  % 加速度在前
        imu_data.gyro = [raw_data{5}, raw_data{6}, raw_data{7}];   % 陀螺仪在后
        imu_data.count = length(imu_data.timestamp);
        
        fprintf('✓ 读取预处理IMU数据: %d 条\n', imu_data.count);
        
    catch ME
        error('读取预处理IMU数据失败: %s', ME.message);
    end
end
