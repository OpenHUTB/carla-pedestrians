%% 生成Town01卫星图轨迹可视化
% 直接运行此脚本，无需切换目录

clear; clc; close all;

fprintf('========================================\n');
fprintf('  Town01 卫星图轨迹可视化生成器\n');
fprintf('========================================\n\n');

%% 1. 获取当前脚本路径
script_path = fileparts(mfilename('fullpath'));
fprintf('当前脚本位置: %s\n', script_path);

%% 2. 添加必要路径
utils_path = fullfile(script_path, '07_test', '07_test', 'test_imu_visual_slam', 'utils');
if exist(utils_path, 'dir')
    addpath(utils_path);
    fprintf('✓ 已添加utils路径\n');
else
    error('❌ 找不到utils目录: %s', utils_path);
end

%% 3. 调用绘图函数
fprintf('\n开始生成卫星图轨迹...\n\n');

try
    % 基本调用（自动查找背景图）
    plot_trajectory_on_satellite_map('Town01Data_IMU_Fusion');
    
    fprintf('\n========================================\n');
    fprintf('✅ 生成完成！\n');
    fprintf('========================================\n');
    fprintf('\n查看结果:\n');
    fprintf('  neuro/data/Town01Data_IMU_Fusion/slam_results/\n');
    fprintf('  - trajectory_on_map_Town01Data_IMU_Fusion.png\n');
    fprintf('  - trajectory_on_map_Town01Data_IMU_Fusion.pdf\n\n');
    
catch ME
    fprintf('\n❌ 生成失败: %s\n', ME.message);
    fprintf('详细错误:\n');
    disp(ME);
end
