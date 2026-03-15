%% 读取所有数据集的性能数据并汇总
% 用于验证论文中的runtime表格数据

clear; clc;

% 数据集路径
datasets = {
    'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\slam_results\performance_Town01Data_IMU_Fusion.mat', 'Town01';
    'E:\Neuro_end\neuro\data\Town02Data_IMU_Fusion\slam_results\performance_Town02Data_IMU_Fusion.mat', 'Town02';
    'E:\Neuro_end\neuro\data\Town10Data_IMU_Fusion\slam_results\performance_Town10Data_IMU_Fusion.mat', 'Town10';
};

fprintf('======================================================================\n');
fprintf('                    ALL DATASETS PERFORMANCE SUMMARY                   \n');
fprintf('======================================================================\n\n');

all_stats = {};
valid_count = 0;

for i = 1:size(datasets, 1)
    mat_file = datasets{i, 1};
    name = datasets{i, 2};
    
    if exist(mat_file, 'file')
        data = load(mat_file);
        valid_count = valid_count + 1;
        all_stats{valid_count} = struct('name', name, 'stats', data.stats);
        
        fprintf('=== %s ===\n', name);
        fprintf('%-25s %12s %12s\n', 'Component', 'Mean(ms)', 'Freq(Hz)');
        fprintf('------------------------------------------------------\n');
        
        component_names = fieldnames(data.stats);
        total_time = 0;
        
        for j = 1:length(component_names)
            comp = component_names{j};
            if isfield(data.stats.(comp), 'mean_ms')
                fprintf('%-25s %12.2f %12.2f\n', comp, data.stats.(comp).mean_ms, data.stats.(comp).freq_hz);
                if ~strcmp(comp, 'total_frame')
                    total_time = total_time + data.stats.(comp).mean_ms;
                end
            end
        end
        
        % 如果有total_frame，使用它
        if isfield(data.stats, 'total_frame')
            total_time = data.stats.total_frame.mean_ms;
        end
        
        fps = 1000.0 / total_time;
        fprintf('------------------------------------------------------\n');
        fprintf('%-25s %12.2f %12.2f\n', 'TOTAL', total_time, fps);
        fprintf('\n');
    else
        fprintf('=== %s === (文件不存在: %s)\n\n', name, mat_file);
    end
end

%% 计算平均值
if valid_count > 0
    fprintf('======================================================================\n');
    fprintf('                    AVERAGED ACROSS ALL DATASETS                       \n');
    fprintf('======================================================================\n');
    
    % 收集所有组件名
    all_components = {};
    for i = 1:valid_count
        comp_names = fieldnames(all_stats{i}.stats);
        all_components = union(all_components, comp_names);
    end
    
    fprintf('%-25s %12s %12s\n', 'Component', 'Mean(ms)', 'Freq(Hz)');
    fprintf('------------------------------------------------------\n');
    
    total_avg = 0;
    for j = 1:length(all_components)
        comp = all_components{j};
        values = [];
        for i = 1:valid_count
            if isfield(all_stats{i}.stats, comp) && isfield(all_stats{i}.stats.(comp), 'mean_ms')
                values = [values; all_stats{i}.stats.(comp).mean_ms];
            end
        end
        if ~isempty(values)
            avg_ms = mean(values);
            avg_hz = 1000.0 / avg_ms;
            fprintf('%-25s %12.2f %12.2f\n', comp, avg_ms, avg_hz);
            if ~strcmp(comp, 'total_frame')
                total_avg = total_avg + avg_ms;
            end
        end
    end
    
    % 使用total_frame的平均值
    total_frame_values = [];
    for i = 1:valid_count
        if isfield(all_stats{i}.stats, 'total_frame')
            total_frame_values = [total_frame_values; all_stats{i}.stats.total_frame.mean_ms];
        end
    end
    
    if ~isempty(total_frame_values)
        total_avg = mean(total_frame_values);
    end
    
    fps_avg = 1000.0 / total_avg;
    fprintf('------------------------------------------------------\n');
    fprintf('%-25s %12.2f %12.2f\n', 'TOTAL PIPELINE (avg)', total_avg, fps_avg);
    fprintf('======================================================================\n');
end
