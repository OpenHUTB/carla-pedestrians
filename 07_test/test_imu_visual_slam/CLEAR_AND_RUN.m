%% 清除所有缓存并重新运行测试
% 解决MATLAB函数缓存导致修改不生效的问题

fprintf('========================================\n');
fprintf('清除MATLAB缓存并重新运行\n');
fprintf('========================================\n\n');

%% 1. 清除所有变量和函数
fprintf('[1/4] 清除所有变量和函数缓存...\n');
clear all;
close all;
clc;

%% 2. 清除特定函数缓存
fprintf('[2/4] 清除特征提取器缓存...\n');
clear hart_cornet_feature_extractor;
clear visual_template_hart_cornet;
clear visual_template_neuro_matlab_only;

%% 3. 重新加载函数（强制重新编译）
fprintf('[3/4] 重新加载函数...\n');
rehash toolboxcache;
rehash path;

%% 4. 运行测试
fprintf('[4/4] 开始运行测试...\n');
fprintf('========================================\n\n');

% 运行主测试脚本
test_imu_visual_slam_hart_cornet;
