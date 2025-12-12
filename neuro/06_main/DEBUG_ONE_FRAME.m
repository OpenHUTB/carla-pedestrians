% DEBUG_ONE_FRAME - 只处理1帧，显示详细调试信息
clear all; close all; clc;

fprintf('========================================\n');
fprintf(' 调试模式：只处理1帧\n');
fprintf('========================================\n\n');

% 彻底清除缓存
clear functions; clear classes;
rehash toolboxcache; rehash path;
fprintf('✓ 缓存已清除\n\n');

% 检查函数路径
fprintf('验证函数:\n');
fprintf('  get_cur_img_files_path_list: %s\n', which('get_cur_img_files_path_list'));
fprintf('  sortObj: %s\n', which('sortObj'));
fprintf('\n');

% 直接读取并显示get_cur_img_files_path_list.m的内容（确认是最新版本）
fprintf('检查 get_cur_img_files_path_list.m 的第36-43行:\n');
funcPath = which('get_cur_img_files_path_list');
if ~isempty(funcPath)
    fid = fopen(funcPath, 'r');
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid);
    end
    fclose(fid);
    
    if length(lines) >= 43
        for i = 36:43
            fprintf('  %3d: %s\n', i, lines{i});
        end
    end
    fprintf('\n');
end

% Town01数据配置
% 动态获取neuro根目录
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
visualDataFile = datasetPath;
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
expMapHistoryFile = tempname;
odoMapHistoryFile = tempname;
vtHistoryFile = tempname;
emHistoryFile = tempname;
gcTrajFile = tempname;
hdcTrajFile = tempname;

fprintf('========================================\n');
fprintf(' 启动main.m (将显示DEBUG输出)\n');
fprintf('========================================\n\n');

try
    main(visualDataFile, groundTruthFile, ...
         expMapHistoryFile, odoMapHistoryFile, ...
         vtHistoryFile, emHistoryFile, ...
         gcTrajFile, hdcTrajFile);
    
    fprintf('\n========================================\n');
    fprintf(' 调试运行完成\n');
    fprintf('========================================\n\n');
    
catch ME
    fprintf('\n========================================\n');
    fprintf(' 运行出错\n');
    fprintf('========================================\n\n');
    fprintf('错误: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('堆栈:\n');
        for i = 1:min(3, length(ME.stack))
            fprintf('  %s (第%d行)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
end
