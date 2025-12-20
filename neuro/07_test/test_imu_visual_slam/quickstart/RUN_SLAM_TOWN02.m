%% 运行Town02 SLAM完整测试 


clear all; close all; clc;

% 添加core目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town02Data_IMU_Fusion';

%% ========== Town02 


% Visual Template 
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.055; 

% Experience Map 
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 15;  

% Map Relaxation 
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 10;        
EXP_CORRECTION_OVERRIDE = 0.5;  

% IMU融合权重 - 适当增强（v2优化）
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.65;  
IMU_TRANS_WEIGHT_OVERRIDE = 0.25; 

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town02 SLAM测试                              ║\n');
fprintf('║   场景: 紧凑型，中等闭环                        ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');



% 运行测试脚本
test_imu_visual_fusion_slam;

fprintf('\n✅ Town02 SLAM测试完成！\n');
fprintf('📁 结果保存在: data/01_NeuroSLAM_Datasets/Town02Data_IMU_Fusion/slam_results/\n\n');
