%% 检查trajectories.mat的数据格式

clear all; close all; clc;
%% 仓库根目录（NLM_ROOT 优先，否则由本文件位置上溯到含 kbs/run_all_checks.py 的目录）
rootDir = getenv('NLM_ROOT');
if isempty(rootDir)
    rootDir = fileparts(mfilename('fullpath'));
    while ~exist(fullfile(rootDir,'kbs','run_all_checks.py'),'file') && ...
            ~strcmp(fileparts(rootDir), rootDir)
        rootDir = fileparts(rootDir);
    end
end


fprintf('\n检查Town01数据格式...\n\n');

% 加载文件
data_file = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results/trajectories.mat');
load(data_file);

fprintf('=== 所有变量 ===\n');
whos

fprintf('\n=== gt_data 详情 ===\n');
fprintf('类型: %s\n', class(gt_data));
fprintf('大小: [%s]\n', num2str(size(gt_data)));
if isstruct(gt_data)
    fprintf('结构体字段:\n');
    disp(fieldnames(gt_data));
    if isfield(gt_data, 'x')
        fprintf('  x: [%s]\n', num2str(size(gt_data.x)));
    end
    if isfield(gt_data, 'y')
        fprintf('  y: [%s]\n', num2str(size(gt_data.y)));
    end
else
    fprintf('前5行数据:\n');
    disp(gt_data(1:min(5,end), :));
end

fprintf('\n=== fusion_data 详情 ===\n');
fprintf('类型: %s\n', class(fusion_data));
fprintf('大小: [%s]\n', num2str(size(fusion_data)));
if isstruct(fusion_data)
    fprintf('结构体字段:\n');
    disp(fieldnames(fusion_data));
else
    fprintf('前5行数据:\n');
    disp(fusion_data(1:min(5,end), :));
end

fprintf('\n=== odo_trajectory 详情 ===\n');
fprintf('类型: %s\n', class(odo_trajectory));
fprintf('大小: [%s]\n', num2str(size(odo_trajectory)));
if isstruct(odo_trajectory)
    fprintf('结构体字段:\n');
    disp(fieldnames(odo_trajectory));
else
    fprintf('前5行数据:\n');
    disp(odo_trajectory(1:min(5,end), :));
end

fprintf('\n=== exp_trajectory 详情 ===\n');
fprintf('类型: %s\n', class(exp_trajectory));
fprintf('大小: [%s]\n', num2str(size(exp_trajectory)));
if isstruct(exp_trajectory)
    fprintf('结构体字段:\n');
    disp(fieldnames(exp_trajectory));
else
    fprintf('前5行数据:\n');
    disp(exp_trajectory(1:min(5,end), :));
end

fprintf('\n完成！\n');
