% QUICK_RUN - 快速运行NeuroSLAM（Town01数据）
%
% 已修复Ground Truth加载问题，现在可以直接运行

clear; clc;

fprintf('========================================\n');
fprintf(' 快速运行NeuroSLAM + 增强特征提取器\n');
fprintf('========================================\n\n');

%% Town01数据配置
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
visualDataFile = datasetPath;
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
expMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EXP_MAP.mat');
odoMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_ODO_MAP.mat');
vtHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_VT.mat');
emHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EM.mat');
gcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_GC_TRAJ.mat');
hdcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_HDC_TRAJ.mat');

%% 确认配置
fprintf('数据集: Town01 (5000张图像)\n');
fprintf('增强特征: 启用 (5.92x速度)\n');
fprintf('Ground Truth: 已修复加载问题\n');
fprintf('\n');

%% 运行NeuroSLAM
fprintf('开始运行...\n\n');

try
    main(visualDataFile, groundTruthFile, ...
         expMapHistoryFile, odoMapHistoryFile, ...
         vtHistoryFile, emHistoryFile, ...
         gcTrajFile, hdcTrajFile);
    
    fprintf('\n========================================\n');
    fprintf(' 运行完成！\n');
    fprintf('========================================\n\n');
    
    fprintf('结果保存在: %s/slam_results/\n', visualDataFile);
    fprintf('\n');
    
catch ME
    fprintf('\n========================================\n');
    fprintf(' 运行出错\n');
    fprintf('========================================\n\n');
    fprintf('错误: %s\n', ME.message);
    fprintf('位置: %s (第%d行)\n\n', ME.stack(1).name, ME.stack(1).line);
end
