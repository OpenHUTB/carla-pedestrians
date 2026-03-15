%% 简单测试 - 验证代码是否能执行到Ours完成后
% 这个脚本模拟test_imu_visual_fusion_slam2.m的结构

fprintf('开始测试...\n');

% 模拟Baseline
fprintf('\n=== Baseline ===\n');
for i = 1:10
    % 模拟处理
end
fprintf('Baseline完成\n');

% 模拟Ours
fprintf('\n=== Ours ===\n');
for i = 1:10
    % 模拟处理
end
fprintf('Ours完成\n');

% ★★★ 关键测试点 ★★★
fprintf('\n=== 这里应该继续执行 ===\n');
fprintf('如果你看到这行,说明代码可以继续\n');
fprintf('如果看不到,说明有问题\n');

% 模拟后续处理
fprintf('\n=== 后续处理 ===\n');
fprintf('轨迹对齐...\n');
fprintf('可视化...\n');
fprintf('完成!\n');
