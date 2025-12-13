%% 运行Town10 SLAM完整测试
%% 快速运行 Town10 SLAM测试

clear all; close all; clc;

% 添加core目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town10Data_IMU_Fusion';

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town10 SLAM测试                              ║\n');
fprintf('║   HART+Transformer (Plan B配置)                ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% 运行SLAM测试
test_imu_visual_fusion_slam;

fprintf('\n✅ Town10 SLAM测试完成！\n');
fprintf('📁 结果保存在: data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/slam_results/\n\n');
