% RUN_FRESH - 清除缓存后运行NeuroSLAM
%
% 强制MATLAB重新加载所有函数

fprintf('========================================\n');
fprintf(' 清除MATLAB缓存并重新运行\n');
fprintf('========================================\n\n');

%% 1. 彻底清除所有缓存
fprintf('清除缓存...\n');
clear all;
close all;
clc;

% 清除函数缓存
clear functions;
clear classes;
clear java;

% 重建函数路径缓存（多次执行确保生效）
rehash toolboxcache;
rehash pathreset;
rehash path;
pause(0.1);
rehash path;

fprintf('✓ 缓存已彻底清除\n\n');

%% 2. 验证关键函数已更新
fprintf('验证关键函数:\n');

% 检查get_cur_img_files_path_list.m
funcPath = which('get_cur_img_files_path_list');
fprintf('  get_cur_img_files_path_list: %s\n', funcPath);

% 检查sortObj.m
funcPath = which('sortObj');
fprintf('  sortObj: %s\n', funcPath);

% 检查load_ground_truth_data.m
funcPath = which('load_ground_truth_data');
fprintf('  load_ground_truth_data: %s\n\n', funcPath);

%% 3. 快速测试图像加载
fprintf('测试图像加载...\n');
% 动态获取数据路径
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
dataPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
imgType = '*.png';

% 使用fullfile（正确方法）
pattern = fullfile(dataPath, imgType);
fprintf('  搜索模式: %s\n', pattern);
files = dir(pattern);
fprintf('  找到图像: %d 张\n', length(files));

if length(files) == 5000
    fprintf('✓ 图像加载正常！\n\n');
else
    fprintf('❌ 图像加载异常！预期5000张，实际%d张\n\n', length(files));
    error('请检查数据路径');
end

%% 4. 运行NeuroSLAM
fprintf('========================================\n');
fprintf(' 启动NeuroSLAM (Town01 + 增强特征)\n');
fprintf('========================================\n\n');

% Town01数据配置（使用相对路径）
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
visualDataFile = datasetPath;
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
expMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EXP_MAP.mat');
odoMapHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_ODO_MAP.mat');
vtHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_VT.mat');
emHistoryFile = fullfile(datasetPath, 'slam_results/TOWN01_EM.mat');
gcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_GC_TRAJ.mat');
hdcTrajFile = fullfile(datasetPath, 'slam_results/TOWN01_HDC_TRAJ.mat');

try
    main(visualDataFile, groundTruthFile, ...
         expMapHistoryFile, odoMapHistoryFile, ...
         vtHistoryFile, emHistoryFile, ...
         gcTrajFile, hdcTrajFile);
    
    fprintf('\n========================================\n');
    fprintf(' ✓ 运行成功完成！\n');
    fprintf('========================================\n\n');
    
catch ME
    fprintf('\n========================================\n');
    fprintf(' ❌ 运行出错\n');
    fprintf('========================================\n\n');
    fprintf('错误: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('位置: %s (第%d行)\n\n', ME.stack(1).name, ME.stack(1).line);
    end
    rethrow(ME);
end
