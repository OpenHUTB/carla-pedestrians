%% 快速运行 EuRoC MH_03_medium SLAM测试
%  一键启动脚本
%
%  NeuroSLAM System Copyright (C) 2018-2019
%  EuRoC Support (2024)

clear; close all; clc;

% 设置EuRoC MH_03数据路径（根据实际情况修改）
global EUROC_DATA_PATH;
EUROC_DATA_PATH = '/home/dream/neuro_111111/datasets/euroc_converted/MH_03_medium';

% 检查数据路径是否存在
if ~exist(EUROC_DATA_PATH, 'dir')
    error(['数据目录不存在: ' EUROC_DATA_PATH '\n' ...
           '请先运行数据处理脚本:\n' ...
           'bash scripts/process_euroc_complete.sh MH_03_medium']);
end

fprintf('\n========== EuRoC MH_03_medium SLAM测试 ==========\n');
fprintf('数据路径: %s\n', EUROC_DATA_PATH);
fprintf('================================================\n\n');

% 运行SLAM测试
test_euroc_fusion_slam;
