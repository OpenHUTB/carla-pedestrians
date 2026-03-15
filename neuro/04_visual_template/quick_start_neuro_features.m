%% QUICK_START_NEURO_FEATURES
% 快速开始：增强的视觉特征提取器
%
% 一键测试新的特征提取器，无需修改现有代码
%
% 运行方法:
%   >> quick_start_neuro_features
%
% 或在MATLAB命令行直接运行各个section

clear; clc; close all;

fprintf('========================================\n');
fprintf(' NeuroSLAM增强视觉特征提取器\n');
fprintf(' 基于HART+CORnet的快速演示\n');
fprintf('========================================\n\n');

%% 1. 快速测试：单张图像
fprintf('[快速测试] 单张图像特征提取\n');
fprintf('----------------------------------------\n');

% 创建测试图像
testImg = uint8(rand(120, 240) * 255);

% 使用新方法提取特征
fprintf('提取特征中...\n');
tic;
features = extract_features_matlab(testImg);
timeElapsed = toc;

fprintf('完成!\n');
fprintf('  输入图像: %dx%d\n', size(testImg, 1), size(testImg, 2));
fprintf('  输出特征: %dx%d\n', size(features, 1), size(features, 2));
fprintf('  耗时: %.3f秒 (%.1f FPS)\n\n', timeElapsed, 1/timeElapsed);

% 可视化
figure('Name', '快速测试', 'Position', [100, 100, 1000, 300]);
subplot(1, 3, 1);
imshow(testImg, []);
title('原始图像');

subplot(1, 3, 2);
imshow(features, []);
title('提取的特征');
colormap(jet);
colorbar;

subplot(1, 3, 3);
histogram(features(:), 50);
title('特征分布');
xlabel('特征值');
ylabel('频数');


%% 2. 对比测试：新方法 vs 原始方法
fprintf('[对比测试] 新方法 vs 原始方法\n');
fprintf('----------------------------------------\n');

% 原始方法 (简单patch normalization)
fprintf('运行原始方法...\n');
tic;
origFeatures = simple_patch_norm(testImg, 11);
timeOrig = toc;
fprintf('  耗时: %.3f秒\n', timeOrig);

% 新方法
fprintf('运行新方法...\n');
tic;
neuroFeatures = extract_features_matlab(testImg);
timeNeuro = toc;
fprintf('  耗时: %.3f秒\n', timeNeuro);

speedRatio = abs(timeOrig/timeNeuro);
if timeNeuro < timeOrig
    speedText = '(更快)';
else
    speedText = '(更慢)';
end
fprintf('  速度对比: %.2fx %s\n\n', speedRatio, speedText);

% 可视化对比
figure('Name', '方法对比', 'Position', [100, 100, 1200, 400]);
subplot(1, 3, 1);
imshow(testImg, []);
title('原始图像');

subplot(1, 3, 2);
imshow(origFeatures, []);
title(sprintf('原始方法 (%.3fs)', timeOrig));
colormap(jet);
colorbar;

subplot(1, 3, 3);
imshow(neuroFeatures, []);
title(sprintf('新方法 (%.3fs)', timeNeuro));
colormap(jet);
colorbar;


%% 3. 鲁棒性测试：不同条件下的表现
fprintf('[鲁棒性测试] 不同条件下的表现\n');
fprintf('----------------------------------------\n');

% 创建不同条件的测试图像
baseImg = uint8(rand(120, 240) * 255);

% 条件1: 添加噪声
noisyImg = imnoise(baseImg, 'gaussian', 0, 0.02);

% 条件2: 改变亮度
brightImg = imadjust(baseImg, [0; 1], [0.3; 1]);

% 条件3: 模糊
blurredImg = imgaussfilt(baseImg, 2);

% 提取特征
feat_base = extract_features_matlab(baseImg);
feat_noisy = extract_features_matlab(noisyImg);
feat_bright = extract_features_matlab(brightImg);
feat_blur = extract_features_matlab(blurredImg);

% 计算相似度
sim_noisy = compute_cosine_similarity(feat_base, feat_noisy);
sim_bright = compute_cosine_similarity(feat_base, feat_bright);
sim_blur = compute_cosine_similarity(feat_base, feat_blur);

fprintf('相似度分析 (与原图对比):\n');
fprintf('  添加噪声: %.3f\n', sim_noisy);
fprintf('  改变亮度: %.3f\n', sim_bright);
fprintf('  图像模糊: %.3f\n\n', sim_blur);

% 可视化
figure('Name', '鲁棒性测试', 'Position', [100, 100, 1200, 600]);
for i = 1:4
    subplot(2, 4, i);
    switch i
        case 1
            imshow(baseImg, []);
            title('原图');
        case 2
            imshow(noisyImg, []);
            title(sprintf('加噪 (相似度: %.3f)', sim_noisy));
        case 3
            imshow(brightImg, []);
            title(sprintf('变亮 (相似度: %.3f)', sim_bright));
        case 4
            imshow(blurredImg, []);
            title(sprintf('模糊 (相似度: %.3f)', sim_blur));
    end
    
    subplot(2, 4, i+4);
    switch i
        case 1
            imshow(feat_base, []);
            title('原图特征');
        case 2
            imshow(feat_noisy, []);
        case 3
            imshow(feat_bright, []);
        case 4
            imshow(feat_blur, []);
    end
    colormap(jet);
end


%% 4. 实际应用：视觉模板匹配
fprintf('[实际应用] 视觉模板匹配演示\n');
fprintf('----------------------------------------\n');

% 初始化视觉模板系统
global VT NUM_VT VT_HISTORY;

% 初始化第一个VT（避免字段未定义错误）
VT(1).id = 1;
VT(1).template = [];
VT(1).decay = 0;
VT(1).gc_x = 0;
VT(1).gc_y = 0;
VT(1).gc_z = 0;
VT(1).hdc_yaw = 0;
VT(1).hdc_height = 0;
VT(1).first = 1;

NUM_VT = 1;
VT_HISTORY = [];

% 初始化其他必需的全局变量
init_vt_globals();

% 创建测试图像序列 (模拟机器人移动)
n_frames = 20;
fprintf('处理 %d 帧图像序列...\n', n_frames);

for frame = 1:n_frames
    % 创建测试图像 (添加一些相关性)
    if frame == 1
        testImg = uint8(rand(120, 240) * 255);
    else
        % 添加小的变化
        testImg = testImg + uint8(randn(size(testImg)) * 10);
    end
    
    % 模拟位置信息
    x = frame * 0.1;
    y = frame * 0.05;
    z = 0;
    yaw = frame * 0.1;
    height = 0;
    
    % 调用简化的视觉模板匹配 (不依赖Python)
    [vt_id] = simple_vt_matching(testImg, x, y, z, yaw, height);
    
    if mod(frame, 5) == 0
        fprintf('  处理进度: %d/%d, 当前VT数量: %d\n', frame, n_frames, NUM_VT);
    end
end

fprintf('完成!\n');
fprintf('  总帧数: %d\n', n_frames);
fprintf('  创建的VT数: %d\n', NUM_VT);
fprintf('  平均重用率: %.1f%%\n\n', (n_frames - NUM_VT)/n_frames * 100);

% 可视化部分视觉模板
if NUM_VT > 1
    figure('Name', '视觉模板', 'Position', [100, 100, 1200, 300]);
    n_show = min(NUM_VT - 1, 6);
    for i = 1:n_show
        subplot(1, n_show, i);
        if i+1 <= NUM_VT
            imshow(VT(i+1).template, []);
            title(sprintf('VT %d', i+1));
            colormap(jet);
        end
    end
end


%% 5. 性能建议
fprintf('[性能建议]\n');
fprintf('----------------------------------------\n');
fprintf('根据测试结果，建议配置:\n\n');

fprintf('✓ 如果注重速度:\n');
fprintf('  - 使用MATLAB实现\n');
fprintf('  - 关闭注意力机制\n');
fprintf('  - 减小特征维度\n\n');

fprintf('✓ 如果注重精度:\n');
fprintf('  - 使用Python实现 (如果可用)\n');
fprintf('  - 开启注意力机制\n');
fprintf('  - 使用256维特征\n\n');

fprintf('✓ 如果处理视频流:\n');
fprintf('  - 开启时序整合\n');
fprintf('  - 适当调高VT_MATCH_THRESHOLD\n\n');


%% 完成
fprintf('========================================\n');
fprintf(' 快速演示完成！\n');
fprintf('========================================\n\n');

fprintf('下一步:\n');
fprintf('  1. 查看生成的图表，了解特征提取效果\n');
fprintf('  2. 运行 test_neuro_feature_extractor.m 进行完整测试\n');
fprintf('  3. 参考 integrate_neuro_features_example.m 进行集成\n\n');


%% ========== 辅助函数 ==========

function normImg = simple_patch_norm(img, patchSize)
% 简单的patch normalization (用于对比)

    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    img = double(img);
    [h, w] = size(img);
    
    halfSize = floor(patchSize / 2);
    extImg = padarray(img, [halfSize, halfSize], 'replicate');
    
    normImg = zeros(h, w);
    
    for i = 1:h
        for j = 1:w
            patch = extImg(i:i+patchSize-1, j:j+patchSize-1);
            patchMean = mean(patch(:));
            patchStd = std(patch(:));
            
            if patchStd < eps
                patchStd = 1;
            end
            
            normImg(i, j) = (img(i, j) - patchMean) / patchStd / 255;
        end
    end
end


function similarity = compute_cosine_similarity(feat1, feat2)
% 计算余弦相似度

    vec1 = feat1(:);
    vec2 = feat2(:);
    
    vec1 = vec1 / (norm(vec1) + eps);
    vec2 = vec2 / (norm(vec2) + eps);
    
    cosineSim = dot(vec1, vec2);
    similarity = (cosineSim + 1) / 2;  % 映射到0-1
end


function init_vt_globals()
% 初始化视觉模板所需的全局变量

    global VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
    global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    global PATCH_SIZE_Y_K PATCH_SIZE_X_K;
    global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
    global VT_PANORAMIC;
    global NEURO_FEATURE_METHOD;
    
    % 基础参数
    VT_IMG_CROP_Y_RANGE = 1:120;
    VT_IMG_CROP_X_RANGE = 1:240;
    VT_IMG_RESIZE_X_RANGE = 64;
    VT_IMG_RESIZE_Y_RANGE = 32;
    
    % 匹配参数
    VT_IMG_X_SHIFT = 10;
    VT_IMG_Y_SHIFT = 5;
    VT_IMG_HALF_OFFSET = 0;
    VT_MATCH_THRESHOLD = 0.3;
    
    % 衰减参数
    VT_GLOBAL_DECAY = 0.1;
    VT_ACTIVE_DECAY = 1.0;
    
    % Patch参数
    PATCH_SIZE_Y_K = 11;
    PATCH_SIZE_X_K = 11;
    
    % 全景模式
    VT_PANORAMIC = false;
    
    % 历史记录
    VT_HISTORY_FIRST = [];
    VT_HISTORY_OLD = [];
    PREV_VT_ID = 0;
    
    % 误差记录
    MIN_DIFF_CURR_IMG_VTS = [];
    DIFFS_ALL_IMGS_VTS = [];
    SUB_VT_IMG = [];
    
    % 特征提取方法
    NEURO_FEATURE_METHOD = 'matlab';
end


%% ========== 核心特征提取函数 ==========

function normImg = extract_features_matlab(img)
% MATLAB本地实现的类脑特征提取
% 简化版的HART+CORnet特征提取

    % 转换为灰度图
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    % 归一化到0-1
    img = double(img) / 255.0;
    
    % V1层：Gabor特征提取
    v1Features = extract_gabor_features(img);
    
    % 注意力机制
    attentionMap = compute_attention_map(img);
    
    % 应用注意力权重
    for i = 1:size(v1Features, 3)
        v1Features(:, :, i) = v1Features(:, :, i) .* attentionMap;
    end
    
    % V2层：特征池化和组合
    v2Features = pool_and_combine_features(v1Features);
    
    % 最终归一化
    normImg = normalize_features(v2Features);
end


function gaborFeatures = extract_gabor_features(img)
% 提取Gabor特征 (V1简单细胞)

    % Gabor参数
    nOrientations = 8;
    wavelength = 4;
    sigmaX = 3;
    sigmaY = 3;
    
    [h, w] = size(img);
    gaborFeatures = zeros(h, w, nOrientations);
    
    % 对每个方向创建Gabor滤波器
    for i = 1:nOrientations
        theta = (i-1) * pi / nOrientations;
        gaborKernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, 0, 0.5);
        response = imfilter(img, gaborKernel, 'same', 'replicate');
        gaborFeatures(:, :, i) = max(response, 0);
    end
end


function kernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, offset, aspect)
% 创建Gabor核

    kernelSize = round(3 * max(sigmaX, sigmaY));
    if mod(kernelSize, 2) == 0
        kernelSize = kernelSize + 1;
    end
    
    [x, y] = meshgrid(-(kernelSize-1)/2:(kernelSize-1)/2, ...
                      -(kernelSize-1)/2:(kernelSize-1)/2);
    
    xTheta = x * cos(theta) + y * sin(theta);
    yTheta = -x * sin(theta) + y * cos(theta);
    
    gaussian = exp(-(xTheta.^2 / (2*sigmaX^2) + yTheta.^2 / (2*sigmaY^2)));
    sinusoid = cos(2 * pi * xTheta / wavelength + offset);
    
    kernel = gaussian .* sinusoid;
    kernel = kernel / sum(abs(kernel(:)));
end


function attentionMap = compute_attention_map(img)
% 计算注意力图

    blurred = imgaussfilt(img, 3);
    intensityContrast = abs(img - blurred);
    
    edges = edge(img, 'canny');
    edges = double(edges);
    edges = imdilate(edges, strel('disk', 2));
    
    attentionMap = 0.7 * intensityContrast + 0.3 * edges;
    attentionMap = attentionMap / (max(attentionMap(:)) + eps);
    attentionMap = imgaussfilt(attentionMap, 2);
end


function pooledFeatures = pool_and_combine_features(features)
% V2层：特征池化和组合

    [h, w, c] = size(features);
    poolSize = 2;
    
    maxPooled = zeros(floor(h/poolSize), floor(w/poolSize), c);
    for ch = 1:c
        maxPooled(:, :, ch) = imresize(features(:, :, ch), ...
                                       1/poolSize, 'Method', 'bilinear');
    end
    
    pooledFeatures = mean(maxPooled, 3);
end


function normFeatures = normalize_features(features)
% 特征归一化

    meanVal = mean(features(:));
    stdVal = std(features(:));
    
    if stdVal < eps
        stdVal = 1;
    end
    
    normFeatures = (features - meanVal) / stdVal;
    normFeatures = max(min(normFeatures, 3), -3);
    normFeatures = (normFeatures + 3) / 6;
end


%% ========== 简化的视觉模板匹配 ==========

function [vt_id] = simple_vt_matching(rawImg, x, y, z, yaw, height)
% 简化的视觉模板匹配 (不依赖Python环境)
% 使用增强的特征提取 + 余弦相似度匹配

    global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS;
    
    % 提取特征
    normVtImg = extract_features_matlab(rawImg);
    
    % 前5个模板直接添加
    if NUM_VT < 5
        VT(NUM_VT).decay = VT(NUM_VT).decay - VT_GLOBAL_DECAY;
        if VT(NUM_VT).decay < 0
            VT(NUM_VT).decay = 0;
        end
        
        NUM_VT = NUM_VT + 1;
        VT(NUM_VT).id = NUM_VT;
        VT(NUM_VT).template = normVtImg;
        VT(NUM_VT).decay = VT_ACTIVE_DECAY;
        VT(NUM_VT).gc_x = x;
        VT(NUM_VT).gc_y = y;
        VT(NUM_VT).gc_z = z;
        VT(NUM_VT).hdc_yaw = yaw;
        VT(NUM_VT).hdc_height = height;
        VT(NUM_VT).first = 1;
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        % 与现有模板比较
        MIN_DIFF_CURR_IMG_VTS = inf(NUM_VT, 1);
        
        for k = 2:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            % 计算余弦相似度
            currentVec = normVtImg(:);
            templateVec = VT(k).template(:);
            
            currentVec = currentVec / (norm(currentVec) + eps);
            templateVec = templateVec / (norm(templateVec) + eps);
            
            cosineSim = dot(currentVec, templateVec);
            MIN_DIFF_CURR_IMG_VTS(k) = 1 - cosineSim;  % 余弦距离
        end
        
        [diff, diff_id] = min(MIN_DIFF_CURR_IMG_VTS);
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; diff];
        
        % 判断是否创建新模板
        if (diff > VT_MATCH_THRESHOLD)
            NUM_VT = NUM_VT + 1;
            VT(NUM_VT).id = NUM_VT;
            VT(NUM_VT).template = normVtImg;
            VT(NUM_VT).decay = VT_ACTIVE_DECAY;
            VT(NUM_VT).gc_x = x;
            VT(NUM_VT).gc_y = y;
            VT(NUM_VT).gc_z = z;
            VT(NUM_VT).hdc_yaw = yaw;
            VT(NUM_VT).hdc_height = height;
            VT(NUM_VT).first = 1;
            vt_id = NUM_VT;
            VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
        else
            vt_id = diff_id;
            VT(vt_id).decay = VT(vt_id).decay + VT_ACTIVE_DECAY;
            if PREV_VT_ID ~= vt_id
                VT(vt_id).first = 0;
            end
            VT_HISTORY_OLD = [VT_HISTORY_OLD; vt_id];
        end
    end
    
    VT_HISTORY = [VT_HISTORY; vt_id];
    PREV_VT_ID = vt_id;
end
