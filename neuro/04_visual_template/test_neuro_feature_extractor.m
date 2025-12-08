% TEST_NEURO_FEATURE_EXTRACTOR
% 测试增强的视觉特征提取器
%
% 使用方法:
%   1. 运行此脚本测试MATLAB版本
%   2. 设置 USE_PYTHON = true 测试Python版本

clear; clc; close all;

%% 配置
USE_PYTHON = false;  % 是否使用Python特征提取器
TEST_IMAGE_PATH = '';  % 留空则使用随机测试图像

%% 初始化全局变量
init_neuro_feature_config();

%% 测试1: 基础特征提取
fprintf('=== 测试1: 基础特征提取 ===\n');

% 创建或加载测试图像
if isempty(TEST_IMAGE_PATH)
    testImg = uint8(rand(120, 240) * 255);
    fprintf('使用随机测试图像: %dx%d\n', size(testImg, 1), size(testImg, 2));
else
    testImg = imread(TEST_IMAGE_PATH);
    fprintf('加载测试图像: %s\n', TEST_IMAGE_PATH);
end

% 提取特征
tic;
features = extract_features_matlab(testImg);
timeElapsed = toc;

fprintf('特征提取完成:\n');
fprintf('  特征维度: %dx%d\n', size(features, 1), size(features, 2));
fprintf('  特征范围: [%.3f, %.3f]\n', min(features(:)), max(features(:)));
fprintf('  耗时: %.3f秒\n', timeElapsed);

% 可视化
figure('Name', '测试1: 特征提取结果', 'Position', [100, 100, 1200, 400]);
subplot(1, 3, 1);
imshow(testImg, []);
title('原始图像');

subplot(1, 3, 2);
imshow(features, []);
title('提取的特征图');
colorbar;

subplot(1, 3, 3);
histogram(features(:), 50);
title('特征分布');
xlabel('特征值');
ylabel('频数');

%% 测试2: Gabor特征可视化
fprintf('\n=== 测试2: Gabor特征可视化 ===\n');

% 转换为灰度图
if size(testImg, 3) == 3
    grayImg = rgb2gray(testImg);
else
    grayImg = testImg;
end
grayImg = double(grayImg) / 255.0;

% 提取Gabor特征
gaborFeatures = extract_gabor_features(grayImg);
fprintf('Gabor特征维度: %dx%dx%d\n', ...
    size(gaborFeatures, 1), size(gaborFeatures, 2), size(gaborFeatures, 3));

% 可视化不同方向的Gabor响应
figure('Name', '测试2: Gabor特征 (8个方向)', 'Position', [100, 100, 1200, 800]);
for i = 1:8
    subplot(2, 4, i);
    imshow(gaborFeatures(:, :, i), []);
    title(sprintf('方向 %d (%.1f°)', i, (i-1)*22.5));
    colormap(jet);
    colorbar;
end

%% 测试3: 注意力机制
fprintf('\n=== 测试3: 注意力机制 ===\n');

% 计算注意力图
attentionMap = compute_attention_map(grayImg);
fprintf('注意力图范围: [%.3f, %.3f]\n', ...
    min(attentionMap(:)), max(attentionMap(:)));

% 应用注意力
attentionApplied = grayImg .* attentionMap;

% 可视化
figure('Name', '测试3: 注意力机制', 'Position', [100, 100, 1200, 400]);
subplot(1, 3, 1);
imshow(grayImg, []);
title('原始图像');

subplot(1, 3, 2);
imshow(attentionMap, []);
title('注意力图');
colormap(hot);
colorbar;

subplot(1, 3, 3);
imshow(attentionApplied, []);
title('应用注意力后');

%% 测试4: 特征比较
fprintf('\n=== 测试4: 特征比较 ===\n');

% 创建三张测试图像
img1 = testImg;
img2 = imnoise(testImg, 'gaussian', 0, 0.01);  % 添加轻微噪声
img3 = uint8(rand(size(testImg)) * 255);  % 完全随机图像

% 提取特征
feat1 = extract_features_matlab(img1);
feat2 = extract_features_matlab(img2);
feat3 = extract_features_matlab(img3);

% 计算相似度
sim12 = compute_feature_similarity(feat1, feat2);
sim13 = compute_feature_similarity(feat1, feat3);
sim23 = compute_feature_similarity(feat2, feat3);

fprintf('相似度结果:\n');
fprintf('  原图 vs 加噪图: %.4f (应该较高)\n', sim12);
fprintf('  原图 vs 随机图: %.4f (应该较低)\n', sim13);
fprintf('  加噪图 vs 随机图: %.4f (应该较低)\n', sim23);

% 可视化
figure('Name', '测试4: 特征比较', 'Position', [100, 100, 1200, 800]);
subplot(2, 3, 1);
imshow(img1, []);
title('图像1 (原图)');

subplot(2, 3, 2);
imshow(img2, []);
title('图像2 (加噪)');

subplot(2, 3, 3);
imshow(img3, []);
title('图像3 (随机)');

subplot(2, 3, 4);
imshow(feat1, []);
title('特征1');
colorbar;

subplot(2, 3, 5);
imshow(feat2, []);
title('特征2');
colorbar;

subplot(2, 3, 6);
imshow(feat3, []);
title('特征3');
colorbar;

%% 测试5: 性能基准测试
fprintf('\n=== 测试5: 性能基准测试 ===\n');

nTests = 50;
times = zeros(nTests, 1);

fprintf('运行%d次特征提取...\n', nTests);
for i = 1:nTests
    tic;
    _ = extract_features_matlab(testImg);
    times(i) = toc;
end

fprintf('性能统计:\n');
fprintf('  平均耗时: %.3f秒\n', mean(times));
fprintf('  最小耗时: %.3f秒\n', min(times));
fprintf('  最大耗时: %.3f秒\n', max(times));
fprintf('  标准差: %.3f秒\n', std(times));
fprintf('  处理速度: %.1f帧/秒\n', 1/mean(times));

% 可视化耗时分布
figure('Name', '测试5: 性能基准', 'Position', [100, 100, 800, 400]);
subplot(1, 2, 1);
plot(times, 'b.-');
xlabel('测试次数');
ylabel('耗时 (秒)');
title('特征提取耗时');
grid on;

subplot(1, 2, 2);
histogram(times, 20);
xlabel('耗时 (秒)');
ylabel('频数');
title('耗时分布');
grid on;

%% 测试6: 与原始方法对比
fprintf('\n=== 测试6: 与原始patch normalization对比 ===\n');

% 原始方法 (简单patch normalization)
tic;
origFeatures = simple_patch_normalization(testImg, 11);
timeOrig = toc;

% 新方法
tic;
neuroFeatures = extract_features_matlab(testImg);
timeNeuro = toc;

fprintf('原始方法:\n');
fprintf('  耗时: %.3f秒\n', timeOrig);
fprintf('  特征范围: [%.3f, %.3f]\n', min(origFeatures(:)), max(origFeatures(:)));

fprintf('新方法:\n');
fprintf('  耗时: %.3f秒\n', timeNeuro);
fprintf('  特征范围: [%.3f, %.3f]\n', min(neuroFeatures(:)), max(neuroFeatures(:)));
fprintf('  速度比: %.2fx\n', timeOrig/timeNeuro);

% 可视化对比
figure('Name', '测试6: 方法对比', 'Position', [100, 100, 1200, 400]);
subplot(1, 3, 1);
imshow(testImg, []);
title('原始图像');

subplot(1, 3, 2);
imshow(origFeatures, []);
title('原始Patch Normalization');
colorbar;

subplot(1, 3, 3);
imshow(neuroFeatures, []);
title('Neuro特征提取');
colorbar;

%% 完成
fprintf('\n=== 测试完成 ===\n');
fprintf('所有测试已完成。请查看生成的图表。\n');


%% ========== 辅助函数 ==========

function init_neuro_feature_config()
% 初始化特征提取所需的全局配置
    global NEURO_FEATURE_METHOD;
    NEURO_FEATURE_METHOD = 'matlab';
end


function similarity = compute_feature_similarity(feat1, feat2)
% 计算两个特征的余弦相似度
%
% 参数:
%   feat1, feat2 - 特征矩阵
% 返回:
%   similarity - 相似度 (0-1)

    vec1 = feat1(:);
    vec2 = feat2(:);
    
    % 归一化
    vec1 = vec1 / (norm(vec1) + eps);
    vec2 = vec2 / (norm(vec2) + eps);
    
    % 余弦相似度
    cosineSim = dot(vec1, vec2);
    
    % 映射到0-1
    similarity = (cosineSim + 1) / 2;
end


function normImg = simple_patch_normalization(img, patchSize)
% 原始的简单patch normalization (用于对比)
%
% 参数:
%   img - 输入图像
%   patchSize - Patch大小
% 返回:
%   normImg - 归一化图像

    % 转换为灰度图
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    img = double(img);
    [h, w] = size(img);
    
    % 扩展图像
    halfSize = floor(patchSize / 2);
    extImg = padarray(img, [halfSize, halfSize], 'replicate');
    
    normImg = zeros(h, w);
    
    % Patch normalization
    for i = 1:h
        for j = 1:w
            % 提取patch
            patch = extImg(i:i+patchSize-1, j:j+patchSize-1);
            
            % 计算均值和标准差
            patchMean = mean(patch(:));
            patchStd = std(patch(:));
            
            if patchStd < eps
                patchStd = 1;
            end
            
            % 归一化
            normImg(i, j) = (img(i, j) - patchMean) / patchStd / 255;
        end
    end
end
