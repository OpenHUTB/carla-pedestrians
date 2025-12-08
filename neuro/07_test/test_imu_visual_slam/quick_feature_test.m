%% 快速特征诊断 - 测试2张图像的特征距离
clear all; close all; clc;

fprintf('========================================\n');
fprintf('快速特征诊断\n');
fprintf('========================================\n\n');

%% 加载2张图像
% 动态获取数据路径
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));

% 测试第100和200帧（应该是不同场景）
img1 = imread(fullfile(data_path, img_files(100).name));
img2 = imread(fullfile(data_path, img_files(200).name));

fprintf('测试图像：\n');
fprintf('  图像1: %s\n', img_files(100).name);
fprintf('  图像2: %s\n', img_files(200).name);
fprintf('\n');

%% 预处理
crop_y = 1:120;
crop_x = 1:160;
resize_y = 12;
resize_x = 16;

img1_crop = img1(crop_y, crop_x);
img1_resize = imresize(img1_crop, [resize_y, resize_x]);

img2_crop = img2(crop_y, crop_x);
img2_resize = imresize(img2_crop, [resize_y, resize_x]);

%% 测试方法1: 当前配置（可能被缓存破坏）
fprintf('[测试1] 当前配置检查\n');
fprintf('--------------------\n');

% 方法1特征提取
feature1_v1 = extract_test_features(img1_resize, 'current');
feature2_v1 = extract_test_features(img2_resize, 'current');

% 计算距离
f1 = feature1_v1(:) / (norm(feature1_v1(:)) + eps);
f2 = feature2_v1(:) / (norm(feature2_v1(:)) + eps);
dist_v1 = 1 - dot(f1, f2);

fprintf('  余弦距离: %.6f\n', dist_v1);
if dist_v1 < 0.01
    fprintf('  ❌ 特征过于相似（<0.01）\n');
elseif dist_v1 < 0.07
    fprintf('  ⚠️  距离小于阈值0.07\n');
else
    fprintf('  ✓ 距离正常（>0.07）\n');
end
fprintf('\n');

%% 测试方法2: 强制使用成功配置
fprintf('[测试2] 强制使用成功配置\n');
fprintf('--------------------\n');

feature1_v2 = extract_test_features(img1_resize, 'success');
feature2_v2 = extract_test_features(img2_resize, 'success');

f1 = feature1_v2(:) / (norm(feature1_v2(:)) + eps);
f2 = feature2_v2(:) / (norm(feature2_v2(:)) + eps);
dist_v2 = 1 - dot(f1, f2);

fprintf('  余弦距离: %.6f\n', dist_v2);
if dist_v2 < 0.01
    fprintf('  ❌ 特征仍然过于相似！\n');
elseif dist_v2 < 0.07
    fprintf('  ⚠️  距离小于阈值，但接近\n');
else
    fprintf('  ✓ 距离正常！应该能创建足够VT\n');
end
fprintf('\n');

%% 对比分析
fprintf('[对比分析]\n');
fprintf('--------------------\n');
fprintf('  方法1距离: %.6f\n', dist_v1);
fprintf('  方法2距离: %.6f\n', dist_v2);
fprintf('  差异: %.6f\n', abs(dist_v2 - dist_v1));
fprintf('\n');

if dist_v2 > 0.03 && dist_v2 < 0.08
    fprintf('✓ 成功配置特征正常！\n');
    fprintf('  建议：运行 test_slam_CLEAN_START\n');
else
    fprintf('⚠️  问题可能更深层\n');
    fprintf('  建议：\n');
    fprintf('    1. 检查图像数据\n');
    fprintf('    2. 尝试不同阈值\n');
    fprintf('    3. 查看特征可视化\n');
end

fprintf('\n========================================\n');

%% 可视化
figure('Name', '特征对比');

subplot(2,3,1); imshow(img1_resize); title('原图1');
subplot(2,3,2); imshow(feature1_v1); title('方法1特征1');
subplot(2,3,3); imshow(feature1_v2); title('方法2特征1');

subplot(2,3,4); imshow(img2_resize); title('原图2');
subplot(2,3,5); imshow(feature2_v1); title('方法1特征2');
subplot(2,3,6); imshow(feature2_v2); title('方法2特征2');

fprintf('特征可视化窗口已打开\n');
fprintf('========================================\n');

%% 辅助函数
function feature = extract_test_features(img, method)
    % 转灰度
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    switch method
        case 'current'
            % 当前配置（可能被破坏）
            img_enhanced = adapthisteq(img, 'ClipLimit', 0.03, 'NumTiles', [8 8]);
            [Gx, Gy] = imgradientxy(img_enhanced, 'sobel');
            edge_magnitude = sqrt(Gx.^2 + Gy.^2);
            combined = 0.6 * img_enhanced + 0.4 * edge_magnitude;
            
        case 'success'
            % 成功配置（ClipLimit=0.02 + imgaussfilt）
            img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
            img_smoothed = imgaussfilt(img_enhanced, 0.5);
            [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
            edge_magnitude = sqrt(Gx.^2 + Gy.^2);
            combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    end
    
    % 归一化
    feature = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
end
