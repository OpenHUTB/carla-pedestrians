% VISUALIZE_HART_CORNET_FEATURES 可视化HART+CORnet特征提取效果
%
% 用于对比原始图像、简单特征和HART+CORnet特征
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12-02

clear all; close all; clc;

%% 添加路径
% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
addpath(fullfile(rootDir, '04_visual_template'));

%% 读取测试图像
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));

% 选择几帧有代表性的图像
test_frames = [100, 500, 1000, 2000, 3000];
n_frames = length(test_frames);

%% 提取特征并可视化
figure('Name', 'HART+CORnet 特征提取对比', 'Position', [100, 100, 1400, 800]);

for i = 1:n_frames
    frame_idx = test_frames(i);
    
    % 读取图像
    img_path = fullfile(data_path, img_files(frame_idx).name);
    img = imread(img_path);
    
    % 裁剪和缩放（与VT处理一致）
    subImg = img(1:120, 1:160, :);
    vtResizedImg = imresize(subImg, [12, 16]);
    
    % 提取特征
    features_hart_cornet = extract_features_hart_cornet(vtResizedImg);
    features_simple = extract_simple_features(vtResizedImg);
    
    % 显示
    subplot(n_frames, 4, (i-1)*4 + 1);
    imshow(subImg);
    title(sprintf('原始图像 (帧%d)', frame_idx));
    
    subplot(n_frames, 4, (i-1)*4 + 2);
    imshow(vtResizedImg);
    title('缩放后 (12x16)');
    
    subplot(n_frames, 4, (i-1)*4 + 3);
    imagesc(features_simple);
    colorbar; axis image;
    title('简单特征');
    
    subplot(n_frames, 4, (i-1)*4 + 4);
    imagesc(features_hart_cornet);
    colorbar; axis image;
    title('HART+CORnet特征');
end

sgtitle('HART+CORnet vs 简单特征提取对比', 'FontSize', 14, 'FontWeight', 'bold');

%% 保存可视化
result_dir = fullfile(data_path, 'slam_results_hart_cornet');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end
saveas(gcf, fullfile(result_dir, 'feature_comparison_v2.png'));
fprintf('特征对比图已保存: feature_comparison_v2.png\n');

%% 计算特征距离分布
fprintf('\n计算特征距离分布...\n');

sample_frames = 1:50:min(500, length(img_files));
n_samples = length(sample_frames);
features_collection = cell(n_samples, 1);

for i = 1:n_samples
    frame_idx = sample_frames(i);
    img_path = fullfile(data_path, img_files(frame_idx).name);
    img = imread(img_path);
    subImg = img(1:120, 1:160, :);
    vtResizedImg = imresize(subImg, [12, 16]);
    features_collection{i} = extract_features_hart_cornet(vtResizedImg);
end

% 计算所有帧对之间的余弦距离
distances = [];
for i = 1:n_samples
    for j = i+1:n_samples
        feat1 = features_collection{i}(:);
        feat2 = features_collection{j}(:);
        
        % 余弦距离
        cosine_sim = dot(feat1, feat2) / (norm(feat1) * norm(feat2) + eps);
        cosine_dist = 1 - cosine_sim;
        
        distances = [distances; cosine_dist];
    end
end

% 可视化距离分布
figure('Name', '特征距离分布', 'Position', [100, 100, 800, 600]);
histogram(distances, 50, 'Normalization', 'probability');
hold on;
xline(0.09, 'r--', 'LineWidth', 2, 'Label', '阈值 0.09');
xline(median(distances), 'g--', 'LineWidth', 2, 'Label', sprintf('中位数 %.3f', median(distances)));
hold off;
xlabel('余弦距离');
ylabel('概率');
title('HART+CORnet特征距离分布');
grid on;

fprintf('距离统计:\n');
fprintf('  最小值: %.4f\n', min(distances));
fprintf('  最大值: %.4f\n', max(distances));
fprintf('  平均值: %.4f\n', mean(distances));
fprintf('  中位数: %.4f\n', median(distances));
fprintf('  标准差: %.4f\n', std(distances));
fprintf('  阈值0.09以下: %.1f%%\n', sum(distances < 0.09) / length(distances) * 100);

%% 保存距离分布图
saveas(gcf, fullfile(result_dir, 'feature_distance_distribution_v2.png'));
fprintf('距离分布图已保存: feature_distance_distribution_v2.png\n');
fprintf('\n改进说明:\n');
fprintf('  - 简化IT层归一化（移除tanh压缩）\n');
fprintf('  - 减弱注意力强度（70%%原始+30%%加权）\n');
fprintf('  - 降低Softmax集中度（系数5→2）\n');
fprintf('  - 目标: 提高特征区分度，减少误匹配\n');


%% 辅助函数：简单特征提取
function features = extract_simple_features(img)
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    % 简单方法：adapthisteq + Sobel
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    features = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
end
