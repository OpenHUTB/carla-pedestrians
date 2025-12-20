function [timestamps] = get_euroc_image_timestamps(img_files)
%GET_EUROC_IMAGE_TIMESTAMPS 从EuRoC图像文件名提取时间戳
%   EuRoC图像文件名格式: timestamp.png (纳秒)
%   
%   输入:
%       img_files - dir()返回的图像文件结构数组
%   输出:
%       timestamps - 时间戳数组 (秒)
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   EuRoC Timestamp Extraction (2024)

    num_images = length(img_files);
    timestamps = zeros(num_images, 1);
    
    for i = 1:num_images
        % 获取文件名（不含扩展名）
        [~, filename, ~] = fileparts(img_files(i).name);
        
        % 如果文件名是纳秒时间戳（EuRoC原始格式）
        if str2double(filename) > 1e15  % 纳秒级时间戳
            timestamps(i) = str2double(filename) * 1e-9;  % 转换为秒
        else
            % 如果是序号格式（0001.png等），使用帧率估计
            % 假设20Hz采样率
            timestamps(i) = (i - 1) * 0.05;  % 50ms间隔
        end
    end
    
    fprintf('提取图像时间戳:\n');
    fprintf('  图像数量: %d\n', num_images);
    fprintf('  时间范围: %.2f - %.2f 秒\n', timestamps(1), timestamps(end));
    fprintf('  平均帧率: %.1f Hz\n', 1 / mean(diff(timestamps)));
end
