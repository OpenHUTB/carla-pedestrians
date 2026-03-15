%% 测试Ground Truth文件读取
% 验证ground_truth.txt是否能正确读取

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  测试GT文件读取                                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 检查文件
fprintf('[1/3] 检查文件...\n');

data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
gt_file = fullfile(data_path, 'ground_truth.txt');

fprintf('  文件路径: %s\n', gt_file);

if ~exist(gt_file, 'file')
    fprintf('  ✗ 文件不存在!\n');
    return;
end

fprintf('  ✓ 文件存在\n');

% 文件信息
file_info = dir(gt_file);
fprintf('  文件大小: %.2f KB\n', file_info.bytes / 1024);
fprintf('  修改时间: %s\n', file_info.date);

%% 2. 读取文件
fprintf('\n[2/3] 读取文件...\n');

try
    gt_table = readtable(gt_file);
    fprintf('  ✓ 读取成功\n');
    fprintf('  数据行数: %d\n', height(gt_table));
    fprintf('  数据列数: %d\n', width(gt_table));
catch ME
    fprintf('  ✗ 读取失败: %s\n', ME.message);
    return;
end

%% 3. 检查列名和数据
fprintf('\n[3/3] 检查数据结构...\n');

fprintf('  列名:\n');
col_names = gt_table.Properties.VariableNames;
for i = 1:length(col_names)
    fprintf('    [%d] %s\n', i, col_names{i});
end

% 检查必需的列
required_cols = {'timestamp', 'pos_x', 'pos_y', 'pos_z'};
fprintf('\n  检查必需列:\n');
for i = 1:length(required_cols)
    col_name = required_cols{i};
    if ismember(col_name, col_names)
        fprintf('    ✓ %s 存在\n', col_name);
    else
        fprintf('    ✗ %s 不存在!\n', col_name);
    end
end

% 显示前5行数据
fprintf('\n  前5行数据:\n');
if height(gt_table) >= 5
    disp(gt_table(1:5, :));
else
    disp(gt_table);
end

% 构建gt_data结构
fprintf('\n  构建gt_data结构...\n');
try
    gt_data = struct();
    gt_data.timestamp = gt_table.timestamp;
    gt_data.pos = [gt_table.pos_x, gt_table.pos_y, gt_table.pos_z];
    
    fprintf('  ✓ gt_data构建成功\n');
    fprintf('    gt_data.timestamp: %s\n', mat2str(size(gt_data.timestamp)));
    fprintf('    gt_data.pos: %s\n', mat2str(size(gt_data.pos)));
    
    % 计算轨迹长度
    traj_length = sum(sqrt(sum(diff(gt_data.pos).^2, 2)));
    fprintf('    轨迹长度: %.2f 米\n', traj_length);
    
    % 显示位置范围
    fprintf('    位置范围:\n');
    fprintf('      X: [%.2f, %.2f] 米\n', min(gt_data.pos(:,1)), max(gt_data.pos(:,1)));
    fprintf('      Y: [%.2f, %.2f] 米\n', min(gt_data.pos(:,2)), max(gt_data.pos(:,2)));
    fprintf('      Z: [%.2f, %.2f] 米\n', min(gt_data.pos(:,3)), max(gt_data.pos(:,3)));
    
catch ME
    fprintf('  ✗ 构建失败: %s\n', ME.message);
    return;
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  测试完成 - GT文件读取正常!                                  ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

fprintf('\n现在可以运行:\n');
fprintf('  1. 添加GT到MAT文件     - 将GT添加到comparison_results.mat\n');
fprintf('  2. 快速绘制带GT的轨迹  - 绘制轨迹对比图\n');
