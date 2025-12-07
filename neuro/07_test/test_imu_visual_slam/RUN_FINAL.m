%% 最终运行脚本 - 使用已验证成功的方法
% 
% 配置：使用visual_template_neuro_matlab_only.m（成功方法）
% 预期结果：VT~299, 经验节点~426, RMSE~126m

clear all; close all; clc;

fprintf('========================================\n');
fprintf('最终测试 - 使用已验证成功的方法\n');
fprintf('========================================\n');
fprintf('预期结果：\n');
fprintf('  VT数量：~299个\n');
fprintf('  经验节点：~426个\n');
fprintf('  RMSE：~126米\n');
fprintf('========================================\n\n');

pause(2);

% 运行测试（USE_HART_CORNET已设为false）
test_imu_visual_slam_hart_cornet;
