% RUN_NEUROSLAM_EXAMPLE
% NeuroSLAM运行示例 - 使用增强视觉特征提取器
%
% 使用方法:
%   1. 修改下面的数据路径
%   2. 运行此脚本
%   3. 查看结果

clear; clc;

fprintf('========================================\n');
fprintf(' NeuroSLAM运行示例\n');
fprintf(' (增强视觉特征提取器已启用)\n');
fprintf('========================================\n\n');

%% 配置数据路径
% 请根据您的实际数据路径修改以下变量

% 动态获取neuro根目录（本脚本在neuro/06_main/下）
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);

% Town01示例配置（使用相对路径）
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
visualDataFile = datasetPath;
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
expMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EXP_MAP.mat');
odoMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_ODO_MAP.mat');
vtHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_VT.mat');
emHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EM.mat');
gcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_GC_TRAJ.mat');
hdcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_HDC_TRAJ.mat');

%% 检查文件是否存在
fprintf('检查数据文件...\n');

if ~exist(visualDataFile, 'dir')
    fprintf('❌ 视觉数据文件夹不存在: %s\n', visualDataFile);
    fprintf('\n请修改脚本中的数据路径，或使用以下方式：\n\n');
    fprintf('方法1: 指定完整路径\n');
    fprintf('  visualDataFile = ''/path/to/your/visual/data'';\n\n');
    fprintf('方法2: 使用相对路径\n');
    fprintf('  cd到数据目录，然后使用相对路径\n\n');
    fprintf('方法3: 手动调用main函数\n');
    fprintf('  main(visualDataFile, groundTruthFile, ...);\n\n');
    return;
end

fprintf('✓ 视觉数据文件夹: %s\n', visualDataFile);

if exist(groundTruthFile, 'file')
    fprintf('✓ Ground Truth文件: %s\n', groundTruthFile);
else
    fprintf('⚠  Ground Truth文件不存在，将跳过精度评估\n');
    groundTruthFile = '';  % 置空
end

fprintf('\n');

%% 确认增强特征提取器已启用
global USE_NEURO_FEATURE_EXTRACTOR;
if isempty(USE_NEURO_FEATURE_EXTRACTOR) || ~USE_NEURO_FEATURE_EXTRACTOR
    fprintf('启用增强视觉特征提取器...\n');
    USE_NEURO_FEATURE_EXTRACTOR = true;
end

%% 运行NeuroSLAM
fprintf('========================================\n');
fprintf(' 开始运行NeuroSLAM\n');
fprintf('========================================\n\n');

try
    main(visualDataFile, groundTruthFile, ...
         expMapHistoryFile, odoMapHistoryFile, ...
         vtHistoryFile, emHistoryFile, ...
         gcTrajFile, hdcTrajFile);
    
    fprintf('\n========================================\n');
    fprintf(' NeuroSLAM运行完成！\n');
    fprintf('========================================\n\n');
    
catch ME
    fprintf('\n========================================\n');
    fprintf(' 运行出错\n');
    fprintf('========================================\n\n');
    fprintf('错误信息: %s\n', ME.message);
    fprintf('错误位置: %s (第%d行)\n', ME.stack(1).file, ME.stack(1).line);
    fprintf('\n');
end
