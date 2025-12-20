%% 运行EuRoC MH_03_medium SLAM测试
%  MH_03是中等难度序列，包含更快的运动和更多的动态变化
%
%  NeuroSLAM System Copyright (C) 2018-2019
%  EuRoC IMU Fusion Support (2024)

clear all; close all; clc;

% Visual Template 
global VT_MATCH_THRESHOLD_OVERRIDE;
VT_MATCH_THRESHOLD_OVERRIDE = 0.030;

% Experience Map 
global DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE;
DELTA_EXP_GC_HDC_THRESHOLD_OVERRIDE = 8;

% Map Relaxation 
global EXP_LOOPS_OVERRIDE;
global EXP_CORRECTION_OVERRIDE;
EXP_LOOPS_OVERRIDE = 20;
EXP_CORRECTION_OVERRIDE = 0.7;

% IMU融合
global IMU_YAW_WEIGHT_OVERRIDE;
global IMU_TRANS_WEIGHT_OVERRIDE;
global IMU_HEIGHT_WEIGHT_OVERRIDE;
IMU_YAW_WEIGHT_OVERRIDE = 0.90;
IMU_TRANS_WEIGHT_OVERRIDE = 0.70;
IMU_HEIGHT_WEIGHT_OVERRIDE = 0.75;

% Grid Cell VT注入 
global GC_VT_INJECT_ENERGY_OVERRIDE;
GC_VT_INJECT_ENERGY_OVERRIDE = 0.3;

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   EuRoC MH_03_medium SLAM测试       ║\n');
fprintf('║   惯视融合 + 类脑SLAM      ║\n');
fprintf('║   中等难度：快速运动 + 动态场景               ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');



%% 设置数据路径
% 请根据你的实际路径修改这里
global EUROC_DATA_PATH;

% 方法1：使用原始EuRoC数据集（推荐）
% EUROC_DATA_PATH = '/path/to/EuRoC/MH_03_medium';

% 方法2：使用neuro数据目录
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));
EUROC_DATA_PATH = fullfile(neuro_root, 'data', '02_EuRoc_Dataset', 'MH_03_medium');

% 方法3：手动指定（如果上面的路径不对）
if ~exist(EUROC_DATA_PATH, 'dir')
    fprintf('⚠️  数据路径不存在: %s\n\n', EUROC_DATA_PATH);
    fprintf('请设置正确的EUROC数据路径，例如：\n');
    fprintf('  EUROC_DATA_PATH = ''/home/user/EuRoC/MH_03_medium'';\n\n');
    
    % 提示用户输入
    user_path = input('请输入EuRoC MH_03_medium数据路径（留空取消）: ', 's');
    if ~isempty(user_path) && exist(user_path, 'dir')
        EUROC_DATA_PATH = user_path;
    else
        error('数据路径无效，测试取消');
    end
end

fprintf('数据路径: %s\n\n', EUROC_DATA_PATH);

%% 检查数据完整性
fprintf('检查数据完整性...\n');

% 检查图像
img_dir = fullfile(EUROC_DATA_PATH, 'cam0', 'data');
if ~exist(img_dir, 'dir')
    img_dir = fullfile(EUROC_DATA_PATH, 'images');
end
if exist(img_dir, 'dir')
    img_files = dir(fullfile(img_dir, '*.png'));
    fprintf('  ✓ 图像: %d 张\n', length(img_files));
else
    error('未找到图像目录');
end

% 检查IMU数据
imu_file = fullfile(EUROC_DATA_PATH, 'mav0', 'imu0', 'data.csv');
if exist(imu_file, 'file')
    fprintf('  ✓ IMU数据: %s\n', imu_file);
    has_imu = true;
else
    imu_file_alt = fullfile(EUROC_DATA_PATH, 'aligned_imu.txt');
    if exist(imu_file_alt, 'file')
        fprintf('  ✓ IMU数据（预处理）: %s\n', imu_file_alt);
        has_imu = true;
    else
        fprintf('  ⚠️  未找到IMU数据，将使用纯视觉模式\n');
        has_imu = false;
    end
end

% 检查Ground Truth
gt_file = fullfile(EUROC_DATA_PATH, 'mav0', 'state_groundtruth_estimate0', 'data.csv');
if exist(gt_file, 'file')
    fprintf('  ✓ Ground Truth: %s\n', gt_file);
elseif exist(fullfile(EUROC_DATA_PATH, 'ground_truth.txt'), 'file')
    fprintf('  ✓ Ground Truth (预处理): ground_truth.txt\n');
else
    fprintf('  ⚠️  未找到Ground Truth文件\n');
end

fprintf('\n');

%% 添加测试脚本路径
script_dir = fileparts(mfilename('fullpath'));
core_dir = fullfile(script_dir, '..', 'core');
addpath(core_dir);


% 运行测试脚本
test_euroc_fusion_slam;

% 重新检查has_imu（因为test_euroc_fusion_slam会清除变量）
imu_file = fullfile(EUROC_DATA_PATH, 'mav0', 'imu0', 'data.csv');
if exist(imu_file, 'file')
    has_imu = true;
else
    imu_file_alt = fullfile(EUROC_DATA_PATH, 'aligned_imu.txt');
    if exist(imu_file_alt, 'file')
        has_imu = true;
    else
        has_imu = false;
    end
end

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   MH_03测试完成！                             ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% 显示结果路径
result_path = fullfile(EUROC_DATA_PATH, 'slam_results');
fprintf('结果保存在:\n');
fprintf('  %s/\n', result_path);
fprintf('  - euroc_trajectories.mat (MATLAB数据)\n');
fprintf('  - imu_visual_slam_comparison.png (对比图)\n');
fprintf('  - slam_accuracy_evaluation.png (精度评估)\n\n');

