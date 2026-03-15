function [fusion_data] = read_fusion_pose(data_path)
%READ_FUSION_POSE 读取IMU-视觉融合位姿数据
%   从fusion_pose.txt读取EKF融合后的位姿信息
%   
%   输入:
%       data_path - 数据文件夹路径
%   输出:
%       fusion_data - 结构体,包含:
%           timestamp - 时间戳
%           pos - 融合位置 [x, y, z]
%           att - 融合姿态 [roll, pitch, yaw] (degrees)
%           imu_pos - 纯IMU积分位置 [x, y, z]
%           vel - 速度 [vx, vy, vz]
%           uncertainty - 位置不确定性 [ux, uy, uz]
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   IMU-Visual Fusion Module (2024)

    fusion_file = fullfile(data_path, 'fusion_pose.txt');
    
    if ~exist(fusion_file, 'file')
        fusion_candidates = dir(fullfile(data_path, 'fusion_pose*.txt'));
        if ~isempty(fusion_candidates)
            [~, idx] = sort({fusion_candidates.name});
            fusion_file = fullfile(data_path, fusion_candidates(idx(1)).name);
        else
            error('融合位姿文件不存在: %s', fusion_file);
        end
    end
    
    % 读取CSV格式数据
    % timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,imu_pos_x,imu_pos_y,imu_pos_z,vel_x,vel_y,vel_z,uncertainty_x,uncertainty_y,uncertainty_z
    try
        fid = fopen(fusion_file, 'r');
        % 检查第一行是否包含"timestamp"（表头标志）
        first_line = fgetl(fid);
        fclose(fid);
        
        % 打开文件准备读取
        fid = fopen(fusion_file, 'r');
        
        % 如果第一行包含"timestamp"，则跳过表头
        if contains(first_line, 'timestamp') || contains(first_line, 'pos_x')
            fgetl(fid);  % 跳过表头行
            fprintf('检测到表头，已跳过\n');
        else
            fprintf('未检测到表头，从第一行开始读取\n');
        end
        
        % 读取所有数据行（不在格式字符串中指定逗号，只用Delimiter参数）
        raw_data = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', ...
            'Delimiter', ',');
        fclose(fid);
        
        % 解析数据
        fusion_data.timestamp = raw_data{1};
        fusion_data.pos = [raw_data{2}, raw_data{3}, raw_data{4}];
        fusion_data.att = [raw_data{5}, raw_data{6}, raw_data{7}];  % degrees
        fusion_data.imu_pos = [raw_data{8}, raw_data{9}, raw_data{10}];
        fusion_data.vel = [raw_data{11}, raw_data{12}, raw_data{13}];
        fusion_data.uncertainty = [raw_data{14}, raw_data{15}, raw_data{16}];
        fusion_data.count = length(fusion_data.timestamp);
        
        fprintf('成功读取 %d 条融合位姿数据\n', fusion_data.count);
        
        % 计算轨迹统计
        trajectory_length = sum(sqrt(sum(diff(fusion_data.pos).^2, 2)));
        fprintf('总轨迹长度: %.2f 米\n', trajectory_length);
        fprintf('平均位置不确定性: [%.3f, %.3f, %.3f] 米\n', ...
            mean(fusion_data.uncertainty(:,1)), ...
            mean(fusion_data.uncertainty(:,2)), ...
            mean(fusion_data.uncertainty(:,3)));
        
    catch ME
        error('读取融合位姿数据失败: %s', ME.message);
    end
end
