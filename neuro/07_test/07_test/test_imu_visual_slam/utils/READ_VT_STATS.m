%% 读取所有数据集的VT统计数据
% 用于验证VT增长图的数据准确性

clear; clc;

fprintf('\n========================================\n');
fprintf('  读取所有数据集的VT统计数据\n');
fprintf('========================================\n\n');

% 数据路径
datasets = {
    'Town01Data_IMU_Fusion', 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\slam_results\vt_stats_Town01Data_IMU_Fusion.mat';
    'Town02Data_IMU_Fusion', 'E:\Neuro_end\neuro\data\Town02Data_IMU_Fusion\slam_results\vt_stats_Town02Data_IMU_Fusion.mat';
    'Town10Data_IMU_Fusion', 'E:\Neuro_end\neuro\data\Town10Data_IMU_Fusion\slam_results\vt_stats_Town10Data_IMU_Fusion.mat';
    'KITTI_07', 'E:\Neuro_end\neuro\data\KITTI_07\slam_results\vt_stats_KITTI_07.mat';
    'MH_01_easy', 'E:\Neuro_end\neuro\data\MH_01_easy\MH_01_easy\slam_results\vt_stats_MH_01_easy.mat';
    'MH_03_medium', 'E:\Neuro_end\neuro\data\MH_03_medium\MH_03_medium\slam_results\vt_stats_MH_03_medium.mat';
};

fprintf('%-20s | %-15s | %-10s | %-15s\n', 'Dataset', 'Total VT', 'Frames', 'File Date');
fprintf('%s\n', repmat('-', 1, 75));

for i = 1:size(datasets, 1)
    dataset_name = datasets{i, 1};
    file_path = datasets{i, 2};
    
    if exist(file_path, 'file')
        % 读取MAT文件
        data = load(file_path);
        
        % 获取VT数量
        if isfield(data, 'template_count')
            vt_count = data.template_count(end);  % 最后一个值
        elseif isfield(data, 'vt_history')
            vt_count = data.vt_history(end);
        else
            vt_count = NaN;
        end
        
        % 获取帧数
        if isfield(data, 'template_count')
            frames = length(data.template_count);
        elseif isfield(data, 'vt_history')
            frames = length(data.vt_history);
        else
            frames = NaN;
        end
        
        % 获取文件修改时间
        file_info = dir(file_path);
        file_date = datestr(file_info.date, 'yyyy-mm-dd HH:MM');
        
        fprintf('%-20s | %-15d | %-10d | %-15s\n', dataset_name, vt_count, frames, file_date);
        
        % 显示MAT文件中的所有字段
        fprintf('   字段: %s\n', strjoin(fieldnames(data), ', '));
        
    else
        fprintf('%-20s | %-15s | %-10s | %-15s\n', dataset_name, 'FILE NOT FOUND', '-', '-');
    end
    fprintf('\n');
end

fprintf('========================================\n');
fprintf('✅ 数据读取完成\n');
fprintf('========================================\n\n');

fprintf('📝 说明：\n');
fprintf('• 如果VT数量与论文不符，需要重新运行SLAM生成数据\n');
fprintf('• 如果文件不存在，需要先运行对应数据集的SLAM\n');
fprintf('• 论文中的数据（experience nodes）：\n');
fprintf('  - Town01: 242 nodes\n');
fprintf('  - Town02: 198 nodes\n');
fprintf('  - Town10: 195 nodes\n');
fprintf('  - KITTI07: 145 nodes\n');
fprintf('  - MH01: ~100 nodes\n');
fprintf('  - MH03: 162 nodes\n\n');
