% TEST_QUICK - 快速测试增强特征提取器
% 最简单的测试，确保代码可以运行

clear; clc;

fprintf('快速测试增强特征提取器...\n');

% 创建测试图像
testImg = uint8(rand(120, 240) * 255);
fprintf('创建测试图像: %dx%d\n', size(testImg, 1), size(testImg, 2));

% 提取特征
fprintf('提取特征...\n');
tic;
features = extract_features_matlab(testImg);
t = toc;

fprintf('完成!\n');
fprintf('  特征尺寸: %dx%d\n', size(features, 1), size(features, 2));
fprintf('  耗时: %.3f秒\n', t);
fprintf('  速度: %.1f FPS\n', 1/t);

% 显示结果
figure;
subplot(1, 2, 1);
imshow(testImg);
title('原始图像');

subplot(1, 2, 2);
imshow(features, []);
title('提取的特征');
colorbar;

fprintf('\n测试成功! ✓\n');


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
