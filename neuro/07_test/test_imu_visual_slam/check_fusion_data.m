%% 检查fusion_pose.txt数据完整性
% 用于诊断"只读取1条数据"的问题

clear; clc;

fprintf('========================================\n');
fprintf('Fusion Data 诊断工具\n');
fprintf('========================================\n\n');

% 数据路径
data_path = '../../data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion';
fusion_file = fullfile(data_path, 'fusion_pose.txt');

% 1. 检查文件是否存在
fprintf('[1/5] 检查文件存在性...\n');
if ~exist(fusion_file, 'file')
    error('❌ 文件不存在: %s', fusion_file);
end
fprintf('✓ 文件存在\n\n');

% 2. 检查文件大小
fprintf('[2/5] 检查文件大小...\n');
file_info = dir(fusion_file);
fprintf('文件大小: %.2f KB (%.2f MB)\n', file_info.bytes/1024, file_info.bytes/1024/1024);
if file_info.bytes < 1000
    warning('⚠️  文件太小，可能数据不完整！');
end
fprintf('\n');

% 3. 查看文件前几行
fprintf('[3/5] 查看文件前10行...\n');
fid = fopen(fusion_file, 'r');
for i = 1:10
    line = fgetl(fid);
    if line == -1
        fprintf('  [第%d行] (文件结束)\n', i);
        break;
    end
    fprintf('  [第%d行] %s\n', i, line);
end
fclose(fid);
fprintf('\n');

% 4. 统计总行数
fprintf('[4/5] 统计总行数...\n');
fid = fopen(fusion_file, 'r');
line_count = 0;
while ~feof(fid)
    fgetl(fid);
    line_count = line_count + 1;
end
fclose(fid);
fprintf('总行数: %d\n', line_count);

% 检查第一行是否是表头
fid = fopen(fusion_file, 'r');
first_line = fgetl(fid);
fclose(fid);
has_header = contains(first_line, 'timestamp') || contains(first_line, 'pos_x');
if has_header
    fprintf('检测到表头: 是\n');
    fprintf('数据行数: %d (总行数 - 1)\n', line_count - 1);
else
    fprintf('检测到表头: 否\n');
    fprintf('数据行数: %d\n', line_count);
end
fprintf('\n');

% 5. 尝试读取数据
fprintf('[5/5] 尝试读取所有数据...\n');
try
    fid = fopen(fusion_file, 'r');
    
    % 跳过表头（如果存在）
    first_line = fgetl(fid);
    if contains(first_line, 'timestamp') || contains(first_line, 'pos_x')
        fprintf('跳过表头行\n');
    else
        % 重新打开文件
        fclose(fid);
        fid = fopen(fusion_file, 'r');
    end
    
    % 读取所有数据（不在格式字符串中指定逗号）
    raw_data = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', ...
        'Delimiter', ',');
    fclose(fid);
    
    % 检查读取结果
    num_read = length(raw_data{1});
    fprintf('成功读取数据行数: %d\n', num_read);
    
    if num_read == 0
        error('❌ 读取数据失败！可能的原因：\n  1. 文件格式不正确\n  2. 文件为空或只有表头');
    elseif num_read == 1
        warning('⚠️  只读取到1行数据！');
        fprintf('\n可能原因：\n');
        fprintf('  1. Python脚本被提前终止（Ctrl+C）\n');
        fprintf('  2. 数据写入未完成\n');
        fprintf('  3. 文件被截断\n');
        fprintf('\n解决方案：\n');
        fprintf('  重新运行Python采集脚本完整采集数据：\n');
        fprintf('  cd ../../00_collect_data\n');
        fprintf('  python IMU_Vision_Fusion_EKF.py\n');
    else
        fprintf('✓ 数据读取正常\n');
        fprintf('\n数据统计：\n');
        fprintf('  时间戳范围: %.2f - %.2f 秒\n', min(raw_data{1}), max(raw_data{1}));
        fprintf('  位置范围:\n');
        fprintf('    X: %.2f - %.2f 米\n', min(raw_data{2}), max(raw_data{2}));
        fprintf('    Y: %.2f - %.2f 米\n', min(raw_data{3}), max(raw_data{3}));
        fprintf('    Z: %.2f - %.2f 米\n', min(raw_data{4}), max(raw_data{4}));
    end
    
catch ME
    fprintf('❌ 读取失败: %s\n', ME.message);
end

fprintf('\n========================================\n');
fprintf('诊断完成\n');
fprintf('========================================\n');
