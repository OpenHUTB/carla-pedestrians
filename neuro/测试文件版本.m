%% 测试test_imu_visual_fusion_slam2.m的版本
% 检查文件是否包含我们的修改

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  测试文件版本                                                ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

% 1. 检查文件路径
file_path = 'E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core\test_imu_visual_fusion_slam2.m';
fprintf('[1/3] 检查文件路径...\n');
fprintf('  文件: %s\n', file_path);

if exist(file_path, 'file')
    fprintf('  ✓ 文件存在\n');
else
    fprintf('  ✗ 文件不存在!\n');
    return;
end

% 2. 读取文件内容
fprintf('\n[2/3] 读取文件内容...\n');
fid = fopen(file_path, 'r');
if fid == -1
    fprintf('  ✗ 无法打开文件!\n');
    return;
end

content = fread(fid, '*char')';
fclose(fid);
fprintf('  ✓ 文件大小: %d 字节\n', length(content));

% 3. 检查关键修改
fprintf('\n[3/3] 检查关键修改...\n');

% 检查1: 文件开头的全局变量声明
if contains(content, 'global NUM_VT_BL NUM_EXPS_BL VT_BL EXPERIENCES_BL;')
    fprintf('  ✓ 找到全局变量声明 (NUM_VT_BL等)\n');
else
    fprintf('  ✗ 未找到全局变量声明!\n');
end

% 检查2: Ours结果保存
if contains(content, '正在保存Ours结果')
    fprintf('  ✓ 找到Ours结果保存代码\n');
else
    fprintf('  ✗ 未找到Ours结果保存代码!\n');
end

% 检查3: 调试信息输出
if contains(content, '=== 开始输出调试信息 ===')
    fprintf('  ✓ 找到调试信息输出代码\n');
else
    fprintf('  ✗ 未找到调试信息输出代码!\n');
end

% 检查4: 统计修改的行数
lines = strsplit(content, '\n');
fprintf('\n文件统计:\n');
fprintf('  总行数: %d\n', length(lines));

% 查找关键行
for i = 1:length(lines)
    if contains(lines{i}, '正在保存Ours结果')
        fprintf('  "正在保存Ours结果" 在第 %d 行\n', i);
    end
    if contains(lines{i}, '=== 开始输出调试信息 ===')
        fprintf('  "=== 开始输出调试信息 ===" 在第 %d 行\n', i);
    end
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  检查完成                                                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

fprintf('\n如果所有检查都通过(✓),说明文件已经修改\n');
fprintf('如果有失败(✗),说明文件没有保存或被覆盖\n');
