%% 一键修复GT显示问题
% 自动完成所有修复步骤

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  一键修复GT显示问题                                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 步骤1: 测试GT文件读取
fprintf('【步骤1/3】测试GT文件读取...\n\n');
try
    测试GT文件读取
    fprintf('\n✓ 步骤1完成\n');
catch ME
    fprintf('\n✗ 步骤1失败: %s\n', ME.message);
    return;
end

fprintf('\n按任意键继续到步骤2...\n');
pause;

%% 步骤2: 添加GT到MAT文件
fprintf('\n【步骤2/3】添加GT到MAT文件...\n\n');
try
    添加GT到MAT文件
    fprintf('\n✓ 步骤2完成\n');
catch ME
    fprintf('\n✗ 步骤2失败: %s\n', ME.message);
    return;
end

fprintf('\n按任意键继续到步骤3...\n');
pause;

%% 步骤3: 绘制带GT的轨迹
fprintf('\n【步骤3/3】绘制带GT的轨迹...\n\n');
try
    快速绘制带GT的轨迹
    fprintf('\n✓ 步骤3完成\n');
catch ME
    fprintf('\n✗ 步骤3失败: %s\n', ME.message);
    return;
end

%% 完成
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  全部完成!                                                   ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

fprintf('\n如果轨迹效果好，可以开始IMU融合参数网格搜索:\n');
fprintf('  快速网格搜索_等轨迹完成后运行\n');
