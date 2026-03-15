% FIX_PATH_AND_RUN - 修复MATLAB路径冲突后运行
clear all; close all; clc;

fprintf('========================================\n');
fprintf(' 修复MATLAB路径冲突\n');
fprintf('========================================\n\n');

%% 1. 移除所有旧路径
fprintf('移除旧路径...\n');

% 移除可能的冲突路径
try
    % 尝试移除可能的旧路径（如果存在）
    oldPaths = {'/home/dream/Neuro_WS', 'D:\\work\\workspace'};
    for i = 1:length(oldPaths)
        if exist(oldPaths{i}, 'dir')
            rmpath(genpath(oldPaths{i}));
            fprintf('  ✓ 已移除 %s\n', oldPaths{i});
        end
    end
catch
    fprintf('  (旧路径不存在或已移除)\n');
end

% 清除所有缓存
clear all;
clear functions;
clear classes;
fprintf('  ✓ 已清除缓存\n\n');

%% 2. 添加正确路径（确保在最前面）
fprintf('添加正确路径...\n');
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
correctBasePath = fileparts(scriptDir);

% 按顺序添加（后添加的在前面，所以反向添加）
addpath(fullfile(correctBasePath, '08_plot'));
addpath(fullfile(correctBasePath, '05_tookit', 'sort_imge_path_list'));
addpath(fullfile(correctBasePath, '05_tookit', 'process_visual_data', 'process_images_data'));
addpath(fullfile(correctBasePath, '05_tookit', 'load_data'));
addpath(genpath(fullfile(correctBasePath, '05_tookit')));
addpath(genpath(fullfile(correctBasePath, '04_visual_template')));
addpath(genpath(fullfile(correctBasePath, '03_pose_cell')));
addpath(genpath(fullfile(correctBasePath, '02_experience_map')));
addpath(genpath(fullfile(correctBasePath, '01_hdc')));
addpath(fullfile(correctBasePath, '06_main'));

fprintf('  ✓ 已添加正确路径\n\n');

%% 3. 重建路径缓存
fprintf('重建路径缓存...\n');
rehash toolboxcache;
rehash path;
pause(0.2);
rehash path;
fprintf('  ✓ 路径缓存已重建\n\n');

%% 4. 验证关键函数路径
fprintf('验证关键函数:\n');

funcPath = which('get_cur_img_files_path_list');
fprintf('  get_cur_img_files_path_list:\n    %s\n', funcPath);
if contains(funcPath, 'neuro_111111')
    fprintf('    ✓ 路径正确！\n');
else
    fprintf('    ❌ 路径错误！仍然指向旧版本\n');
    error('路径修复失败，请手动检查MATLAB路径设置');
end

funcPath = which('sortObj');
fprintf('  sortObj:\n    %s\n', funcPath);

funcPath = which('load_ground_truth_data');
fprintf('  load_ground_truth_data:\n    %s\n\n', funcPath);

%% 5. 读取并验证get_cur_img_files_path_list.m内容
fprintf('检查 get_cur_img_files_path_list.m 是否为最新版本:\n');
funcPath = which('get_cur_img_files_path_list');
if ~isempty(funcPath)
    content = fileread(funcPath);
    if contains(content, 'sort({imgFilesPathList.name})')
        fprintf('  ✓ 已是最新版本（使用内置sort）\n\n');
    elseif contains(content, 'sortObj(imgFilesPathList)')
        fprintf('  ❌ 仍是旧版本（使用sortObj）\n');
        fprintf('  文件路径: %s\n', funcPath);
        error('函数文件不是最新版本！');
    else
        fprintf('  ⚠ 无法判断版本\n\n');
    end
end

%% 6. 测试图像加载
fprintf('测试图像加载...\n');
dataPath = fullfile(correctBasePath, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
imgType = '*.png';

pattern = fullfile(dataPath, imgType);
files = dir(pattern);
fprintf('  dir() 测试: %d 张图像\n', length(files));

if length(files) == 5000
    fprintf('  ✓ 图像加载正常！\n\n');
else
    fprintf('  ❌ 图像数量不对！\n\n');
end

%% 7. 运行NeuroSLAM
fprintf('========================================\n');
fprintf(' 启动NeuroSLAM (Town01 + 增强特征)\n');
fprintf('========================================\n\n');

datasetPath = fullfile(correctBasePath, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
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
