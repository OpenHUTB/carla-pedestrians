%% 检查comparison_results.mat的内容
% 诊断GT数据是否被保存

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  检查MAT文件内容                                             ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 加载MAT文件
mat_file = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\comparison_results\comparison_results.mat';

fprintf('[1/3] 检查文件...\n');
fprintf('  文件路径: %s\n', mat_file);

if ~exist(mat_file, 'file')
    fprintf('  ✗ 文件不存在!\n');
    return;
end

fprintf('  ✓ 文件存在\n');

% 获取文件信息
file_info = dir(mat_file);
fprintf('  文件大小: %.2f MB\n', file_info.bytes / 1024 / 1024);
fprintf('  修改时间: %s\n', file_info.date);

%% 2. 列出所有变量
fprintf('\n[2/3] 列出所有变量...\n');
vars = whos('-file', mat_file);

fprintf('  共有 %d 个变量:\n', length(vars));
for i = 1:length(vars)
    fprintf('    [%2d] %-30s %10s  %s\n', i, vars(i).name, mat2str(vars(i).size), vars(i).class);
end

%% 3. 检查关键变量
fprintf('\n[3/3] 检查关键变量...\n');

% 检查GT数据
has_gt_data = false;
for i = 1:length(vars)
    if strcmp(vars(i).name, 'gt_data')
        has_gt_data = true;
        break;
    end
end

if has_gt_data
    fprintf('  ✓ gt_data 存在\n');
    
    % 加载并检查gt_data
    load(mat_file, 'gt_data');
    fprintf('    gt_data类型: %s\n', class(gt_data));
    
    if isstruct(gt_data)
        fprintf('    gt_data字段:\n');
        fields = fieldnames(gt_data);
        for i = 1:length(fields)
            field_name = fields{i};
            field_value = gt_data.(field_name);
            fprintf('      - %s: %s (%s)\n', field_name, mat2str(size(field_value)), class(field_value));
        end
        
        if isfield(gt_data, 'pos')
            fprintf('    ✓ gt_data.pos 存在: %s\n', mat2str(size(gt_data.pos)));
        else
            fprintf('    ✗ gt_data.pos 不存在!\n');
        end
    else
        fprintf('    ⚠️ gt_data不是结构体!\n');
    end
else
    fprintf('  ✗ gt_data 不存在!\n');
    fprintf('\n  这就是为什么GT没有显示的原因!\n');
    fprintf('  需要重新运行实验或手动添加GT数据\n');
end

% 检查轨迹数据
fprintf('\n  检查轨迹数据:\n');
trajectory_vars = {'baseline_exp_traj', 'ours_exp_traj', 'baseline_odo_traj', 'ours_odo_traj'};
for i = 1:length(trajectory_vars)
    var_name = trajectory_vars{i};
    has_var = false;
    for j = 1:length(vars)
        if strcmp(vars(j).name, var_name)
            has_var = true;
            fprintf('    ✓ %-25s: %s\n', var_name, mat2str(vars(j).size));
            break;
        end
    end
    if ~has_var
        fprintf('    ✗ %-25s: 不存在\n', var_name);
    end
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  检查完成                                                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

if ~has_gt_data
    fprintf('\n');
    fprintf('【解决方案】\n');
    fprintf('1. 重新运行实验: test_imu_visual_fusion_slam2.m\n');
    fprintf('2. 或者手动添加GT数据到MAT文件\n');
    fprintf('3. 或者使用单次运行版本: test_imu_visual_fusion_slam.m\n');
end
