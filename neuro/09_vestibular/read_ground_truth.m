function gt_data = read_ground_truth(gt_file)
    % READ_GROUND_TRUTH 读取CARLA车辆真实轨迹
    %
    % 输入:
    %   gt_file - ground truth文件路径
    %
    % 输出:
    %   gt_data - 结构体，包含以下字段:
    %     .timestamp - 时间戳向量
    %     .pos - [N×3] 位置矩阵 [x, y, z]
    %     .att - [N×3] 姿态矩阵 [roll, pitch, yaw] (degrees)
    %     .vel - [N×3] 速度矩阵 [vx, vy, vz]
    %     .count - 数据点数量
    
    if ~exist(gt_file, 'file')
        error('Ground truth文件不存在: %s', gt_file);
    end
    
    try
        fid = fopen(gt_file, 'r');
        
        % 读取第一行检查是否有表头
        first_line = fgetl(fid);
        fclose(fid);
        
        % 重新打开文件
        fid = fopen(gt_file, 'r');
        
        % 如果第一行包含"timestamp"，则跳过表头
        if contains(first_line, 'timestamp') || contains(first_line, 'pos_x')
            fgetl(fid);  % 跳过表头行
            fprintf('检测到表头，已跳过\n');
        else
            fprintf('未检测到表头，从第一行开始读取\n');
        end
        
        % 读取所有数据行
        % timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,vel_x,vel_y,vel_z
        raw_data = textscan(fid, '%f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',');
        fclose(fid);
        
        % 解析数据
        gt_data.timestamp = raw_data{1};
        gt_data.pos = [raw_data{2}, raw_data{3}, raw_data{4}];
        gt_data.att = [raw_data{5}, raw_data{6}, raw_data{7}];  % degrees
        gt_data.vel = [raw_data{8}, raw_data{9}, raw_data{10}];
        gt_data.count = length(gt_data.timestamp);
        
        % 计算轨迹长度
        if gt_data.count > 1
            diffs = diff(gt_data.pos);
            distances = sqrt(sum(diffs.^2, 2));
            gt_data.total_length = sum(distances);
        else
            gt_data.total_length = 0;
        end
        
        fprintf('成功读取 %d 条Ground Truth数据\n', gt_data.count);
        fprintf('真实轨迹长度: %.2f 米\n', gt_data.total_length);
        fprintf('位置范围: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]\n', ...
            min(gt_data.pos(:,1)), max(gt_data.pos(:,1)), ...
            min(gt_data.pos(:,2)), max(gt_data.pos(:,2)), ...
            min(gt_data.pos(:,3)), max(gt_data.pos(:,3)));
        
    catch ME
        error('读取Ground Truth失败: %s', ME.message);
    end
end
