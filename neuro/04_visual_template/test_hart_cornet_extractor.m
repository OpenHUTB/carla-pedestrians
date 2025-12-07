%% HART+CORnet特征提取器测试脚本
% 测试新的特征提取器并与原有方法对比
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12

clear all; close all; clc;

fprintf('========== HART+CORnet特征提取器测试 ==========\n\n');

%% 1. 添加路径
% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(currentDir);
addpath(fullfile(rootDir, '04_visual_template'));
fprintf('[1/5] 已添加路径\n');

%% 2. 加载测试图像
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
if ~exist(data_path, 'dir')
    data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data');
end

img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到测试图像文件');
end

fprintf('[2/5] 找到 %d 张测试图像\n', length(img_files));

%% 3. 选择测试图像
test_indices = [1, 100, 200, 300, 400];  % 选择5张代表性图像
test_indices = test_indices(test_indices <= length(img_files));

fprintf('[3/5] 选择 %d 张图像进行测试\n', length(test_indices));

%% 4. 对比测试
fprintf('[4/5] 开始特征提取对比...\n\n');

figure('Position', [100, 100, 1400, 800]);

for i = 1:length(test_indices)
    idx = test_indices(i);
    img_path = fullfile(data_path, img_files(idx).name);
    rawImg = imread(img_path);
    
    % 预处理（统一尺寸）
    if size(rawImg, 1) > 120 || size(rawImg, 2) > 160
        testImg = rawImg(1:120, 1:160, :);
    else
        testImg = rawImg;
    end
    testImg = imresize(testImg, [12, 16]);  % VT标准尺寸
    
    fprintf('图像 %d (帧 %d):\n', i, idx);
    
    % === 方法1: 原始简单方法 ===
    tic;
    simple_features = extract_features_simple(testImg);
    time_simple = toc;
    fprintf('  简单方法: %.4f秒\n', time_simple);
    
    % === 方法2: HART+CORnet方法 ===
    tic;
    hart_cornet_features = hart_cornet_feature_extractor(testImg);
    time_hart_cornet = toc;
    fprintf('  HART+CORnet: %.4f秒 (%.2fx)\n', time_hart_cornet, time_hart_cornet/time_simple);
    
    % === 可视化对比 ===
    subplot(length(test_indices), 4, (i-1)*4 + 1);
    imshow(testImg, []);
    title(sprintf('原图 (帧%d)', idx));
    
    subplot(length(test_indices), 4, (i-1)*4 + 2);
    imshow(simple_features, []);
    title('简单方法');
    
    subplot(length(test_indices), 4, (i-1)*4 + 3);
    imshow(hart_cornet_features, []);
    title('HART+CORnet');
    
    subplot(length(test_indices), 4, (i-1)*4 + 4);
    imshow(abs(hart_cornet_features - simple_features), []);
    title('差异图');
    
    fprintf('\n');
end

sgtitle('HART+CORnet vs 简单方法 特征提取对比', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. 保存结果
fprintf('[5/5] 保存对比结果...\n');
result_path = fullfile(rootDir, '04_visual_template');
saveas(gcf, fullfile(result_path, 'hart_cornet_comparison.png'));
fprintf('对比图已保存: %s/hart_cornet_comparison.png\n', result_path);

%% 总结
fprintf('\n========================================\n');
fprintf('测试完成！\n');
fprintf('========================================\n');
fprintf('特点对比：\n');
fprintf('  简单方法: 快速，但特征表达能力有限\n');
fprintf('  HART+CORnet: 层次化特征，注意力机制，时序建模\n');
fprintf('    - V1: 多尺度边缘检测\n');
fprintf('    - V2: 局部特征池化\n');
fprintf('    - V4: 中层特征整合\n');
fprintf('    - IT: 高层语义特征\n');
fprintf('    - Attention: 空间注意力机制\n');
fprintf('    - Temporal: LSTM时序建模\n');
fprintf('========================================\n\n');


%% ========== 辅助函数 ==========

function normImg = extract_features_simple(img)
% 原始简单特征提取方法（用于对比）

    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    % 自适应对比度增强
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.03, 'NumTiles', [8 8]);
    
    % 边缘检测
    [Gx, Gy] = imgradientxy(img_enhanced, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 融合
    combined = 0.6 * img_enhanced + 0.4 * edge_magnitude;
    
    % 归一化
    normImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
end
