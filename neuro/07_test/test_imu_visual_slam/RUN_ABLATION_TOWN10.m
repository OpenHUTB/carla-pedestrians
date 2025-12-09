%% 运行Town10消融实验
% 设置数据集为Town10并运行消融实验

clear all; close all; clc;

global DATASET_NAME;
DATASET_NAME = 'Town10Data_IMU_Fusion';

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   Town10 消融实验                              ║\n');
fprintf('║   HART+Transformer Ablation Study              ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% 运行消融实验
ablation_study_main;
