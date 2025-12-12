%% HART+CORnet 特征提取器快速启动脚本
% 
% 这个脚本演示如何使用HART+CORnet特征提取器
% 包含3个示例：
%   1. 单张图像特征提取
%   2. 多张图像对比
%   3. 简单的Visual Template测试
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12

clear all; close all; clc;

fprintf('========================================\n');
fprintf('HART+CORnet 特征提取器快速启动\n');
fprintf('========================================\n\n');

%% 设置路径
rootDir = '/home/dream/neuro_111111/carla-pedestrians/neuro';
addpath(fullfile(rootDir, '04_visual_template'));

%% 查找测试图像
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
if ~exist(data_path, 'dir')
    data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data');
end

if ~exist(data_path, 'dir')
    error('未找到数据目录！请检查路径: %s', data_path);
end

img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('未找到测试图像！');
end

fprintf('找到 %d 张测试图像\n', length(img_files));
fprintf('数据路径: %s\n\n', data_path);

%% ========== 示例1: 单张图像特征提取 ==========
fprintf('========== 示例1: 单张图像特征提取 ==========\n');

% 读取第一张图像
img_path = fullfile(data_path, img_files(1).name);
test_img = imread(img_path);
fprintf('加载图像: %s\n', img_files(1).name);

% 调整尺寸到VT标准尺寸
if size(test_img, 1) > 120 || size(test_img, 2) > 160
    test_img = test_img(1:120, 1:160, :);
end
test_img = imresize(test_img, [12, 16]);

% 提取特征
fprintf('正在提取特征...\n');
tic;
features = hart_cornet_feature_extractor(test_img);
t = toc;
fprintf('✓ 特征提取完成! 耗时: %.4f秒\n', t);
fprintf('  特征维度: %d × %d\n', size(features, 1), size(features, 2));
fprintf('  特征范围: [%.4f, %.4f]\n', min(features(:)), max(features(:)));

% 可视化
figure('Position', [100, 100, 1200, 400]);

subplot(1, 3, 1);
imshow(test_img, []);
title('原始图像');

subplot(1, 3, 2);
imshow(features, []);
title('HART+CORnet特征');
colormap('jet');
colorbar;

subplot(1, 3, 3);
imagesc(features);
title('特征热力图');
colormap('jet');
colorbar;

sgtitle('示例1: 单张图像特征提取', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n');

%% ========== 示例2: 多张图像对比 ==========
fprintf('========== 示例2: 多张图像特征对比 ==========\n');

% 选择3张有代表性的图像
test_indices = [1, 50, 100];
test_indices = test_indices(test_indices <= length(img_files));

fprintf('选择 %d 张图像进行对比\n', length(test_indices));

figure('Position', [100, 100, 1400, 600]);

for i = 1:length(test_indices)
    idx = test_indices(i);
    
    % 读取图像
    img_path = fullfile(data_path, img_files(idx).name);
    img = imread(img_path);
    
    % 预处理
    if size(img, 1) > 120 || size(img, 2) > 160
        img = img(1:120, 1:160, :);
    end
    img = imresize(img, [12, 16]);
    
    % 提取特征
    tic;
    features = hart_cornet_feature_extractor(img);
    t = toc;
    
    fprintf('  图像 %d (帧%d): %.4f秒\n', i, idx, t);
    
    % 可视化
    subplot(2, length(test_indices), i);
    imshow(img, []);
    title(sprintf('原图 - 帧%d', idx));
    
    subplot(2, length(test_indices), length(test_indices) + i);
    imshow(features, []);
    title(sprintf('特征 - 帧%d', idx));
end

sgtitle('示例2: 多张图像特征对比', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n');

%% ========== 示例3: Visual Template匹配测试 ==========
fprintf('========== 示例3: Visual Template匹配测试 ==========\n');

% 初始化全局变量
global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
global VT_PANORAMIC;

% 初始化
VT = struct([]);
NUM_VT = 0;
VT_HISTORY = [];
VT_HISTORY_FIRST = [];
VT_HISTORY_OLD = [];
PREV_VT_ID = -1;

MIN_DIFF_CURR_IMG_VTS = [];
DIFFS_ALL_IMGS_VTS = [];
SUB_VT_IMG = [];

% VT参数
VT_IMG_CROP_Y_RANGE = 1:120;
VT_IMG_CROP_X_RANGE = 1:160;
VT_IMG_RESIZE_X_RANGE = 16;
VT_IMG_RESIZE_Y_RANGE = 12;
VT_IMG_X_SHIFT = 5;
VT_IMG_Y_SHIFT = 3;
VT_IMG_HALF_OFFSET = floor(VT_IMG_RESIZE_X_RANGE / 2);
VT_MATCH_THRESHOLD = 0.15;
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 2.0;
VT_PANORAMIC = 0;

fprintf('VT参数配置:\n');
fprintf('  匹配阈值: %.3f\n', VT_MATCH_THRESHOLD);
fprintf('  图像尺寸: %d × %d\n', VT_IMG_RESIZE_Y_RANGE, VT_IMG_RESIZE_X_RANGE);

% 处理前10帧
num_test_frames = min(10, length(img_files));
fprintf('\n处理前 %d 帧图像...\n', num_test_frames);

vt_ids = zeros(num_test_frames, 1);

for i = 1:num_test_frames
    img_path = fullfile(data_path, img_files(i).name);
    rawImg = imread(img_path);
    
    % 调用visual_template_hart_cornet
    vt_id = visual_template_hart_cornet(rawImg, 0, 0, 0, 0, 0);
    vt_ids(i) = vt_id;
    
    if mod(i, 2) == 0
        fprintf('  帧 %d: VT ID = %d\n', i, vt_id);
    end
end

fprintf('\n结果统计:\n');
fprintf('  总VT数量: %d\n', NUM_VT);
fprintf('  新建VT: %d\n', length(VT_HISTORY_FIRST));
fprintf('  识别VT: %d\n', length(VT_HISTORY_OLD));
fprintf('  VT创建率: %.1f%%\n', (NUM_VT / num_test_frames) * 100);

% 可视化VT匹配结果
figure('Position', [100, 100, 1200, 400]);

subplot(1, 2, 1);
plot(1:num_test_frames, vt_ids, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('帧编号');
ylabel('VT ID');
title('VT匹配序列');
grid on;

subplot(1, 2, 2);
histogram(vt_ids, 'BinMethod', 'integers');
xlabel('VT ID');
ylabel('出现次数');
title('VT分布统计');
grid on;

sgtitle('示例3: Visual Template匹配测试', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n');

%% ========== 总结 ==========
fprintf('========================================\n');
fprintf('快速启动测试完成！\n');
fprintf('========================================\n');
fprintf('\n主要特点:\n');
fprintf('  ✓ V1层: 多尺度Gabor滤波 + Sobel边缘\n');
fprintf('  ✓ V2层: 局部池化 + 非线性激活\n');
fprintf('  ✓ V4层: 多尺度特征整合\n');
fprintf('  ✓ IT层: 高层语义特征\n');
fprintf('  ✓ 注意力: 层次化空间注意力\n');
fprintf('  ✓ 时序: LSTM递归建模\n');
fprintf('\n下一步:\n');
fprintf('  1. 运行完整SLAM测试: test_imu_visual_slam_hart_cornet.m\n');
fprintf('  2. 查看详细文档: README_HART_CORNET.md\n');
fprintf('  3. 调整参数优化性能\n');
fprintf('========================================\n');
