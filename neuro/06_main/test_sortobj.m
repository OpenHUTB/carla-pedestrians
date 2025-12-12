% 测试sortObj函数是否有问题
clear; clc;

fprintf('========================================\n');
fprintf(' 测试sortObj函数\n');
fprintf('========================================\n\n');

% 添加路径
addpath(genpath('../05_tookit'));

% 获取图像文件列表
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
dataPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
files = dir(fullfile(dataPath, '*.png'));

fprintf('dir() 返回: %d 个文件\n', length(files));

if isempty(files)
    error('dir() 未找到文件！');
end

fprintf('前3个文件:\n');
for i = 1:min(3, length(files))
    fprintf('  %d. %s\n', i, files(i).name);
end

% 测试sortObj
fprintf('\n调用 sortObj...\n');
try
    sortedFiles = sortObj(files);
    fprintf('sortObj 返回: %d 个文件\n', length(sortedFiles));
    
    if isempty(sortedFiles)
        fprintf('❌ sortObj 返回空数组！\n');
    else
        fprintf('✓ sortObj 成功！前3个文件:\n');
        for i = 1:min(3, length(sortedFiles))
            fprintf('  %d. %s\n', i, sortedFiles(i).name);
        end
    end
catch ME
    fprintf('❌ sortObj 出错: %s\n', ME.message);
    fprintf('堆栈:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (第%d行)\n', ME.stack(i).file, ME.stack(i).line);
    end
end

fprintf('\n========================================\n');
