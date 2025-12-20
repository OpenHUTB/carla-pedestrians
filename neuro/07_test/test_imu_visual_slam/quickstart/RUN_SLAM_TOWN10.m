%% 运行Town10 SLAM完整测试 

clear all; close all; clc;

% 添加core目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));

global DATASET_NAME;
DATASET_NAME = 'Town10Data_IMU_Fusion';

% Visual Template 
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.040;  

% Experience Map 
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 18; 

% Map Relaxation 
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 12;       
EXP_CORRECTION_OVERRIDE = 0.5;   

% IMU融合 
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.70;  
IMU_TRANS_WEIGHT_OVERRIDE = 0.30; 
global IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0.6;  

% Grid Cell VT注入 -
global GC_VT_INJECT_ENERGY_OVERRIDE;
GC_VT_INJECT_ENERGY_OVERRIDE = 0.4;

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town10 SLAM测试                              ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');



% 运行SLAM测试
test_imu_visual_fusion_slam;

fprintf('\n✅ Town10 SLAM测试完成！\n');
fprintf('📁 结果保存在: data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/slam_results/\n\n');
