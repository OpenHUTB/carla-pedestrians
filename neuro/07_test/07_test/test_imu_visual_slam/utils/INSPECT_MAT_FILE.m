%% 详细检查MAT文件内容
% 找出Town10的195个VT在哪个字段

clear; clc;

fprintf('\n========================================\n');
fprintf('  详细检查MAT文件内容\n');
fprintf('========================================\n\n');

% 检查Town10（你记得是195）
town10_file = 'E:\Neuro_end\neuro\data\Town10Data_IMU_Fusion\slam_results\vt_stats_Town10Data_IMU_Fusion.mat';

if exist(town10_file, 'file')
    fprintf('正在检查Town10 MAT文件...\n');
    fprintf('文件路径: %s\n\n', town10_file);
    
    % 加载数据
    data = load(town10_file);
    
    % 显示所有字段
    fprintf('=== 所有字段 ===\n');
    fields = fieldnames(data);
    for i = 1:length(fields)
        field_name = fields{i};
        field_value = data.(field_name);
        
        fprintf('\n字段 %d: %s\n', i, field_name);
        fprintf('  类型: %s\n', class(field_value));
        fprintf('  大小: %s\n', mat2str(size(field_value)));
        
        % 如果是数组，显示一些统计信息
        if isnumeric(field_value)
            if numel(field_value) == 1
                fprintf('  值: %g\n', field_value);
            elseif numel(field_value) <= 10
                fprintf('  值: %s\n', mat2str(field_value));
            else
                fprintf('  最小值: %g\n', min(field_value(:)));
                fprintf('  最大值: %g\n', max(field_value(:)));
                fprintf('  平均值: %g\n', mean(field_value(:)));
                fprintf('  最后一个值: %g\n', field_value(end));
                
                % 检查是否包含195
                if any(field_value(:) == 195)
                    fprintf('  ⭐ 包含195！\n');
                end
                if any(field_value(:) == 125)
                    fprintf('  ⭐ 包含125！\n');
                end
            end
        elseif isstruct(field_value)
            fprintf('  结构体字段: %s\n', strjoin(fieldnames(field_value), ', '));
        end
    end
    
    fprintf('\n========================================\n');
    fprintf('详细检查完成\n');
    fprintf('========================================\n\n');
    
    % 尝试找195
    fprintf('🔍 寻找195这个数字...\n');
    found_195 = false;
    for i = 1:length(fields)
        field_name = fields{i};
        field_value = data.(field_name);
        
        if isnumeric(field_value) && any(field_value(:) == 195)
            fprintf('✅ 在字段 "%s" 中找到195！\n', field_name);
            fprintf('   位置: index %d\n', find(field_value == 195, 1));
            found_195 = true;
        end
    end
    
    if ~found_195
        fprintf('❌ 没有找到195\n');
        fprintf('   最大值是: %g (在字段 template_count 中)\n', max(data.template_count));
    end
    
else
    fprintf('❌ 文件不存在: %s\n', town10_file);
end

fprintf('\n');
