% TEST_SIMPLE - 最简单的测试，验证所有功能
clear; clc; close all;

fprintf('=== 简单测试 ===\n\n');

%% 1. 单张图像特征提取
fprintf('[1] 特征提取测试...\n');
img = uint8(rand(120, 240) * 255);
features = extract_features_matlab(img);
fprintf('    ✓ 特征尺寸: %dx%d\n', size(features, 1), size(features, 2));

%% 2. 视觉模板匹配测试
fprintf('[2] 视觉模板匹配测试...\n');

% 初始化全局变量
global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS;

% 初始化第一个VT
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
VT_HISTORY_FIRST = [];
VT_HISTORY_OLD = [];
PREV_VT_ID = 0;

% 参数
VT_MATCH_THRESHOLD = 0.3;
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 1.0;
MIN_DIFF_CURR_IMG_VTS = [];
DIFFS_ALL_IMGS_VTS = [];

% 测试5帧
for i = 1:5
    test_img = uint8(rand(120, 240) * 255);
    vt_id = simple_vt_matching(test_img, i*0.1, i*0.05, 0, i*0.1, 0);
    fprintf('    帧%d: VT_ID=%d\n', i, vt_id);
end

fprintf('    ✓ 创建了%d个视觉模板\n', NUM_VT);

fprintf('\n=== 所有测试通过! ===\n');


%% 函数定义

function normImg = extract_features_matlab(img)
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    v1Features = extract_gabor_features(img);
    attentionMap = compute_attention_map(img);
    
    for i = 1:size(v1Features, 3)
        v1Features(:, :, i) = v1Features(:, :, i) .* attentionMap;
    end
    
    v2Features = pool_and_combine_features(v1Features);
    normImg = normalize_features(v2Features);
end

function gaborFeatures = extract_gabor_features(img)
    nOrientations = 8;
    wavelength = 4;
    sigmaX = 3;
    sigmaY = 3;
    
    [h, w] = size(img);
    gaborFeatures = zeros(h, w, nOrientations);
    
    for i = 1:nOrientations
        theta = (i-1) * pi / nOrientations;
        gaborKernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, 0, 0.5);
        response = imfilter(img, gaborKernel, 'same', 'replicate');
        gaborFeatures(:, :, i) = max(response, 0);
    end
end

function kernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, offset, aspect)
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
    [h, w, c] = size(features);
    poolSize = 2;
    
    maxPooled = zeros(floor(h/poolSize), floor(w/poolSize), c);
    for ch = 1:c
        maxPooled(:, :, ch) = imresize(features(:, :, ch), 1/poolSize, 'Method', 'bilinear');
    end
    
    pooledFeatures = mean(maxPooled, 3);
end

function normFeatures = normalize_features(features)
    meanVal = mean(features(:));
    stdVal = std(features(:));
    
    if stdVal < eps
        stdVal = 1;
    end
    
    normFeatures = (features - meanVal) / stdVal;
    normFeatures = max(min(normFeatures, 3), -3);
    normFeatures = (normFeatures + 3) / 6;
end

function [vt_id] = simple_vt_matching(rawImg, x, y, z, yaw, height)
    global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS;
    
    normVtImg = extract_features_matlab(rawImg);
    
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
        MIN_DIFF_CURR_IMG_VTS = inf(NUM_VT, 1);
        
        for k = 2:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            currentVec = normVtImg(:);
            templateVec = VT(k).template(:);
            
            currentVec = currentVec / (norm(currentVec) + eps);
            templateVec = templateVec / (norm(templateVec) + eps);
            
            cosineSim = dot(currentVec, templateVec);
            MIN_DIFF_CURR_IMG_VTS(k) = 1 - cosineSim;
        end
        
        [diff, diff_id] = min(MIN_DIFF_CURR_IMG_VTS);
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; diff];
        
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
