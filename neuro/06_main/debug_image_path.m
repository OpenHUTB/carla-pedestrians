% DEBUG_IMAGE_PATH - 直接测试get_cur_img_files_path_list函数
clear; clc;

fprintf('========================================\n');
fprintf(' 调试图像路径读取\n');
fprintf('========================================\n\n');

%% 添加路径
addpath(genpath('../05_tookit'));

%% 测试参数
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
subFoldersPathSet = {datasetPath};
imgType = '*.png';
iSubFolder = 1;

fprintf('输入参数:\n');
fprintf('  subFoldersPathSet{1}: %s\n', subFoldersPathSet{1});
fprintf('  imgType: %s\n', imgType);
fprintf('  iSubFolder: %d\n\n', iSubFolder);

%% 手动测试路径拼接
fprintf('手动测试:\n');
curPath = subFoldersPathSet{1};

% 方法1: strcat（错误）
pattern1 = strcat(curPath, imgType);
fprintf('  strcat: %s\n', pattern1);
files1 = dir(pattern1);
fprintf('    → 找到 %d 张图像\n', length(files1));

% 方法2: fullfile（正确）
pattern2 = fullfile(curPath, imgType);
fprintf('  fullfile: %s\n', pattern2);
files2 = dir(pattern2);
fprintf('    → 找到 %d 张图像\n\n', length(files2));

%% 调用函数测试
fprintf('调用 get_cur_img_files_path_list:\n');
try
    [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, imgType, iSubFolder);
    
    fprintf('  curFolderPath: %s\n', curFolderPath);
    fprintf('  numImgs: %d\n', numImgs);
    
    if numImgs > 0
        fprintf('\n前5个文件:\n');
        for i = 1:min(5, numImgs)
            fprintf('    %d. %s\n', i, imgFilesPathList(i).name);
        end
        fprintf('\n✓ 函数工作正常！\n');
    else
        fprintf('\n❌ 函数返回0张图像！\n');
        fprintf('\n请检查:\n');
        fprintf('  1. get_cur_img_files_path_list.m 是否已更新?\n');
        fprintf('  2. 是否需要清除MATLAB缓存? (clear functions; rehash path)\n');
    end
    
catch ME
    fprintf('❌ 函数调用失败！\n');
    fprintf('错误: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('位置: %s (第%d行)\n', ME.stack(1).file, ME.stack(1).line);
    end
end

fprintf('\n========================================\n');
fprintf(' 调试完成\n');
fprintf('========================================\n');
