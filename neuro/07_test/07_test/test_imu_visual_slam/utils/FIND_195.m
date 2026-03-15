%% 寻找195这个数字
clear; clc;

fprintf('\n寻找你记得的195个VT...\n\n');

% 检查所有Town数据集
datasets = {
    'Town01', 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\slam_results\vt_stats_Town01Data_IMU_Fusion.mat';
    'Town02', 'E:\Neuro_end\neuro\data\Town02Data_IMU_Fusion\slam_results\vt_stats_Town02Data_IMU_Fusion.mat';
    'Town10', 'E:\Neuro_end\neuro\data\Town10Data_IMU_Fusion\slam_results\vt_stats_Town10Data_IMU_Fusion.mat';
};

for i = 1:size(datasets, 1)
    name = datasets{i, 1};
    file = datasets{i, 2};
    
    fprintf('=== %s ===\n', name);
    
    if exist(file, 'file')
        data = load(file);
        
        % 检查所有字段
        fields = fieldnames(data);
        for j = 1:length(fields)
            field_name = fields{j};
            field_value = data.(field_name);
            
            if isnumeric(field_value)
                % 检查最后一个值
                last_val = field_value(end);
                % 检查最大值
                max_val = max(field_value(:));
                
                fprintf('  %s: 最后=%g, 最大=%g', field_name, last_val, max_val);
                
                if last_val == 195 || max_val == 195
                    fprintf(' ⭐ 找到195！');
                end
                fprintf('\n');
            elseif isstruct(field_value)
                % 检查结构体
                sub_fields = fieldnames(field_value);
                for k = 1:length(sub_fields)
                    sub_name = sub_fields{k};
                    sub_value = field_value.(sub_name);
                    if isnumeric(sub_value) && numel(sub_value) > 0
                        fprintf('  %s.%s: %g\n', field_name, sub_name, sub_value(end));
                        if sub_value(end) == 195
                            fprintf('    ⭐ 找到195！\n');
                        end
                    end
                end
            end
        end
    else
        fprintf('  文件不存在\n');
    end
    fprintf('\n');
end

fprintf('========================================\n');
fprintf('提示：如果没找到195，可能是：\n');
fprintf('1. 在其他文件中（不是vt_stats MAT文件）\n');
fprintf('2. 在论文中手动写的数字\n');
fprintf('3. 记忆有误，实际是其他数字\n');
fprintf('========================================\n');
