%% 运行Town02 SLAM完整测试
%% 快速运行 Town02 SLAM测试

clear all; close all; clc;

% 添加core目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town02Data_IMU_Fusion';

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town02 SLAM测试                              ║\n');
fprintf('║   HART+Transformer (Plan B配置)                ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% 运行测试脚本
test_imu_visual_fusion_slam;

fprintf('\n✅ Town02 SLAM测试完成！\n');
fprintf('📁 结果保存在: data/01_NeuroSLAM_Datasets/Town02Data_IMU_Fusion/slam_results/\n\n');
