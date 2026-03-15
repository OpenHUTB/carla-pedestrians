%% 手动添加GT数据到comparison_results.mat
% 解决GT没有显示的问题

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  添加GT数据到MAT文件                                         ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 读取GT数据
fprintf('[1/3] 读取Ground Truth数据...\n');

data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
gt_file = fullfile(data_path, 'ground_truth.txt');

if ~exist(gt_file, 'file')
    fprintf('  ✗ GT文件不存在: %s\n', gt_file);
    return;
end

% 读取TXT (CSV格式)
gt_table = readtable(gt_file);
fprintf('  ✓ 读取成功，共 %d 行\n', height(gt_table));

% 显示列名
fprintf('  列名: %s\n', strjoin(gt_table.Properties.VariableNames, ', '));

% 构建gt_data结构
gt_data = struct();
gt_data.timestamp = gt_table.timestamp;
% 注意：ground_truth.txt的列名是pos_x, pos_y, pos_z
gt_data.pos = [gt_table.pos_x, gt_table.pos_y, gt_table.pos_z];

fprintf('  ✓ GT数据读取成功\n');
fprintf('    数据点数: %d\n', size(gt_data.pos, 1));
fprintf('    轨迹长度: %.2f 米\n', sum(sqrt(sum(diff(gt_data.pos).^2, 2))));

%% 2. 加载现有MAT文件
fprintf('\n[2/3] 加载现有MAT文件...\n');

mat_file = fullfile(data_path, 'comparison_results', 'comparison_results.mat');

if ~exist(mat_file, 'file')
    fprintf('  ✗ MAT文件不存在: %s\n', mat_file);
    fprintf('  请先运行实验生成MAT文件\n');
    return;
end

fprintf('  ✓ MAT文件存在\n');

% 加载所有变量
load(mat_file);
fprintf('  ✓ MAT文件加载成功\n');

%% 3. 添加GT数据并保存
fprintf('\n[3/3] 添加GT数据并保存...\n');

% 检查是否已有gt_data
if exist('gt_data', 'var')
    fprintf('  ⚠️ MAT文件中已有gt_data，将被覆盖\n');
end

% 保存（覆盖模式）
try
    save(mat_file, 'gt_data', '-append');
    fprintf('  ✓ GT数据已添加到MAT文件\n');
catch ME
    fprintf('  ✗ 保存失败: %s\n', ME.message);
    return;
end

%% 4. 验证
fprintf('\n[4/4] 验证...\n');

% 重新加载验证
clear gt_data;
load(mat_file, 'gt_data');

if exist('gt_data', 'var') && isstruct(gt_data) && isfield(gt_data, 'pos')
    fprintf('  ✓ 验证成功!\n');
    fprintf('    gt_data.pos: %s\n', mat2str(size(gt_data.pos)));
    fprintf('    gt_data.timestamp: %s\n', mat2str(size(gt_data.timestamp)));
else
    fprintf('  ✗ 验证失败!\n');
    return;
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  完成!                                                       ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

fprintf('\n现在可以运行 快速绘制带GT的轨迹.m 来查看轨迹对比\n');
