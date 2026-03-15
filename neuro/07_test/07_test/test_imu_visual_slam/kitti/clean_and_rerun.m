%% KITTI清理和重新运行脚本
%  清除之前的结果，重新运行实验

fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║     清理KITTI结果并重新运行实验               ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n');

% 计算数据路径
scriptPath = fileparts(mfilename('fullpath'));
testDir = fileparts(scriptPath);
test07Dir = fileparts(testDir);
neuroDir = fileparts(test07Dir);
data_path = fullfile(neuroDir, 'data', 'KITTI_07');
result_path = fullfile(data_path, 'slam_results');

% 1. 清理结果
fprintf('[1/2] 清理旧结果...\n');
if exist(result_path, 'dir')
    try
        rmdir(result_path, 's');
        fprintf('  ✓ 已删除: %s\n', result_path);
    catch ME
        warning('删除结果目录失败: %s', ME.message);
    end
else
    fprintf('  结果目录不存在，跳过清理\n');
end

% 清除MATLAB工作空间
clear all; close all; clc;

% 2. 重新运行
fprintf('\n[2/2] 重新运行实验...\n\n');

% 切换到kitti目录
cd(fileparts(mfilename('fullpath')));

% 运行实验
run_kitti_experiment;
