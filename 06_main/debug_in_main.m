% DEBUG_IN_MAIN - 在main.m的上下文中调试
clear; clc;

fprintf('========================================\n');
fprintf(' 在main环境中调试图像加载\n');
fprintf('========================================\n\n');

%% 模拟main.m的环境
% 动态获取并切换到neuro/06_main目录
scriptDir = fileparts(mfilename('fullpath'));
cd(scriptDir);

% 添加路径（和main.m一样）
addpath(genpath('../01_hdc'));
addpath(genpath('../02_experience_map'));
addpath(genpath('../03_pose_cell'));
addpath(genpath('../04_visual_template'));
addpath(genpath('../05_tookit'));
addpath(genpath('../08_plot'));

% 设置全局变量
global IMG_TYPE;
IMG_TYPE = '*.png';

fprintf('全局变量 IMG_TYPE = %s\n\n', IMG_TYPE);

%% 模拟main.m中的路径设置
neuroRoot = fileparts(scriptDir);
visualDataFile = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');

% 检查目录
if exist(visualDataFile, 'dir')
    imageFiles = dir(fullfile(visualDataFile, '*.png'));
    fprintf('直接dir测试: %d 张图像\n', length(imageFiles));
    
    if ~isempty(imageFiles)
        subFoldersPathSet = {visualDataFile};
    else
        error('目录存在但无图像！');
    end
else
    error('目录不存在！');
end

numSubFolders = length(subFoldersPathSet);
fprintf('subFoldersPathSet 数量: %d\n', numSubFolders);
fprintf('subFoldersPathSet{1}: %s\n\n', subFoldersPathSet{1});

%% 调用get_cur_img_files_path_list（和main.m一样）
fprintf('调用 get_cur_img_files_path_list:\n');
iSubFolder = 1;

try
    fprintf('  参数:\n');
    fprintf('    subFoldersPathSet{1} = %s\n', subFoldersPathSet{1});
    fprintf('    IMG_TYPE = %s\n', IMG_TYPE);
    fprintf('    iSubFolder = %d\n', iSubFolder);
    
    [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, IMG_TYPE, iSubFolder);
    
    fprintf('  返回值:\n');
    fprintf('    curFolderPath = %s\n', curFolderPath);
    fprintf('    numImgs = %d\n', numImgs);
    fprintf('    length(imgFilesPathList) = %d\n\n', length(imgFilesPathList));
    
    if numImgs > 0
        fprintf('✓ 函数返回正常！前5个文件:\n');
        for i = 1:min(5, numImgs)
            fprintf('    %d. %s\n', i, imgFilesPathList(i).name);
        end
    else
        fprintf('❌ 函数返回0张图像！\n');
        fprintf('\n深入调试:\n');
        
        % 手动执行函数内部逻辑
        curPath = subFoldersPathSet{iSubFolder};
        fprintf('  curPath = %s\n', curPath);
        
        pattern = fullfile(curPath, IMG_TYPE);
        fprintf('  pattern = %s\n', pattern);
        
        files = dir(pattern);
        fprintf('  dir() 返回 %d 个文件\n', length(files));
        
        if ~isempty(files)
            fprintf('  dir() 前5个文件:\n');
            for i = 1:min(5, length(files))
                fprintf('    %d. %s\n', i, files(i).name);
            end
            
            % 测试sortObj
            fprintf('\n  测试 sortObj:\n');
            try
                sortedFiles = sortObj(files);
                fprintf('    sortObj 返回 %d 个文件\n', length(sortedFiles));
                if isempty(sortedFiles)
                    fprintf('    ❌ sortObj 返回空数组！\n');
                end
            catch ME2
                fprintf('    ❌ sortObj 出错: %s\n', ME2.message);
            end
        end
    end
    
catch ME
    fprintf('❌ 函数调用失败！\n');
    fprintf('错误: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('位置: %s (第%d行)\n', ME.stack(1).file, ME.stack(1).line);
        fprintf('\n完整堆栈:\n');
        for i = 1:length(ME.stack)
            fprintf('  %d. %s (第%d行)\n', i, ME.stack(i).file, ME.stack(i).line);
        end
    end
end

fprintf('\n========================================\n');
fprintf(' 调试完成\n');
fprintf('========================================\n');
