%% 运行Town01消融实验
% 设置数据集为Town01并运行消融实验

clear all; close all; clc;

% 添加core和utils目录到路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'core'));
addpath(fullfile(script_dir, '..', 'utils'));

global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town01 消融实验                              ║\n');
fprintf('║   HART+Transformer Ablation Study              ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% 运行消融实验
ablation_study_main;
