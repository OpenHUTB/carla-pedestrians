% run_neuroslam.m - NeuroSLAM快速运行脚本
% 功能：一键启动优化版main函数，自动加载数据、运行SLAM并保存结果

% 获取当前脚本所在目录（动态计算路径，增强移植性）
currentDir = fileparts(mfilename('fullpath'));

% 配置相对路径（基于脚本所在目录计算）
visualDataFile = fullfile(currentDir, '../data/01_NeuroSLAM_Datasets/Town01Data');  % 向上1级到neuro目录，再进入data
groundTruthFile = fullfile(currentDir, '../../../../Dataset/02_NeuroSLAM_Groudtruth/01_SynPerData_GT.txt');  % 向上4级到Neuro_WS，再进入Dataset

% 结果保存路径（向上4级到Neuro_WS，创建results文件夹）
resultsDir = fullfile(currentDir, '../../../../results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
    disp(['已自动创建结果文件夹：', resultsDir]);
end

% 结果文件路径（保持相对结果目录）
expMapHistoryFile = fullfile(resultsDir, 'exp_map_history.mat');
odoMapHistoryFile = fullfile(resultsDir, 'odo_map_history.mat');
vtHistoryFile = fullfile(resultsDir, 'vt_history.mat');
emHistoryFile = fullfile(resultsDir, 'em_history.mat');
gcTrajFile = fullfile(resultsDir, 'gc_trajectory.mat');
hdcTrajFile = fullfile(resultsDir, 'hdc_trajectory.mat');

% 运行参数配置
renderRate = 10;  % 可视化帧率（每10帧更新一次，数值越大速度越快）
visualize = false;  % 调试时改为true开启可视化，默认关闭以加快运行

% 启动NeuroSLAM主程序
disp('===== 开始运行NeuroSLAM系统 =====');
main( ...
    visualDataFile, ...
    groundTruthFile, ...
    expMapHistoryFile, ...
    odoMapHistoryFile, ...
    vtHistoryFile, ...
    emHistoryFile, ...
    gcTrajFile, ...
    hdcTrajFile, ...
    'RENDER_RATE', renderRate, ...
    'VISUALIZE', visualize ...
);

% 运行结束提示
disp('===== 所有处理完成！结果已保存至：=====');
disp(resultsDir);
