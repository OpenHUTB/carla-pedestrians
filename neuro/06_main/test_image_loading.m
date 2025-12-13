% 测试图像加载是否正常
clear; clc;

fprintf('========================================\n');
fprintf(' 测试图像加载\n');
fprintf('========================================\n\n');

% 测试路径
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
dataPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
imgType = '*.png';

fprintf('数据路径: %s\n', dataPath);
fprintf('图像类型: %s\n\n', imgType);

% 方法1：使用strcat（旧方法，错误）
fprintf('方法1 - strcat (错误):\n');
pattern1 = strcat(dataPath, imgType);
fprintf('  搜索模式: %s\n', pattern1);
files1 = dir(pattern1);
fprintf('  找到图像: %d 张\n\n', length(files1));

% 方法2：使用fullfile（新方法，正确）
fprintf('方法2 - fullfile (正确):\n');
pattern2 = fullfile(dataPath, imgType);
fprintf('  搜索模式: %s\n', pattern2);
files2 = dir(pattern2);
fprintf('  找到图像: %d 张\n\n', length(files2));

% 显示前5个文件名
if ~isempty(files2)
    fprintf('前5个文件:\n');
    for i = 1:min(5, length(files2))
        fprintf('  %d. %s\n', i, files2(i).name);
    end
else
    fprintf('❌ 未找到任何图像文件！\n');
end

fprintf('\n========================================\n');
fprintf(' 测试完成\n');
fprintf('========================================\n');
