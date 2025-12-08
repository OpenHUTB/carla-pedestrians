% 分析HART+CORnet特征距离分布
% 帮助理解为什么VT数量只有5个

clear all; close all; clc;

%% 配置
% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
data_dir = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_dir = fullfile(data_dir, 'images');
num_samples = 200;  % 采样图像数量

% 添加路径
addpath(fullfile(rootDir, '04_visual_template'));

%% 提取特征
fprintf('提取 %d 张图像的HART+CORnet特征...\n', num_samples);
features = cell(num_samples, 1);
img_files = dir(fullfile(img_dir, '*.png'));

for i = 1:num_samples
    if mod(i, 20) == 0
        fprintf('  进度: %d/%d\n', i, num_samples);
    end
    
    img_path = fullfile(img_dir, img_files(i*25).name);  % 每25帧采样一次
    img = imread(img_path);
    
    % 裁剪和缩放
    subImg = img(1:120, 1:160);
    resized = imresize(subImg, [12 16]);
    
    % HART+CORnet特征提取
    features{i} = extract_features_hart_cornet(resized);
end

%% 计算距离矩阵
fprintf('计算特征距离矩阵...\n');
distances = zeros(num_samples);

for i = 1:num_samples
    for j = i+1:num_samples
        % 余弦距离
        f1 = features{i}(:);
        f2 = features{j}(:);
        cosine_dist = 1 - (f1' * f2) / (norm(f1) * norm(f2) + eps);
        distances(i,j) = cosine_dist;
        distances(j,i) = cosine_dist;
    end
end

%% 统计分析
dist_vector = distances(triu(true(num_samples), 1));

fprintf('\n========== HART+CORnet特征距离分布 ==========\n');
fprintf('样本数量: %d\n', num_samples);
fprintf('距离对数: %d\n', length(dist_vector));
fprintf('平均距离: %.4f\n', mean(dist_vector));
fprintf('中位数:   %.4f\n', median(dist_vector));
fprintf('标准差:   %.4f\n', std(dist_vector));
fprintf('最小值:   %.4f\n', min(dist_vector));
fprintf('最大值:   %.4f\n', max(dist_vector));
fprintf('\n');
fprintf('阈值分析:\n');
fprintf('  距离 < 0.03: %.1f%%\n', 100 * sum(dist_vector < 0.03) / length(dist_vector));
fprintf('  距离 < 0.05: %.1f%%\n', 100 * sum(dist_vector < 0.05) / length(dist_vector));
fprintf('  距离 < 0.07: %.1f%%\n', 100 * sum(dist_vector < 0.07) / length(dist_vector));
fprintf('  距离 < 0.10: %.1f%%\n', 100 * sum(dist_vector < 0.10) / length(dist_vector));
fprintf('======================================\n\n');

%% 可视化
figure('Position', [100, 100, 1200, 400]);

% 子图1: 距离直方图
subplot(1, 3, 1);
histogram(dist_vector, 50, 'Normalization', 'probability');
hold on;
xline(0.05, 'r--', 'LineWidth', 2, 'Label', '阈值0.05');
xline(0.07, 'g--', 'LineWidth', 2, 'Label', '阈值0.07');
xlabel('余弦距离');
ylabel('概率');
title('HART+CORnet特征距离分布');
grid on;

% 子图2: 累积分布
subplot(1, 3, 2);
sorted_dist = sort(dist_vector);
plot(sorted_dist, (1:length(sorted_dist))/length(sorted_dist), 'b-', 'LineWidth', 2);
hold on;
xline(0.05, 'r--', 'LineWidth', 2);
xline(0.07, 'g--', 'LineWidth', 2);
xlabel('余弦距离');
ylabel('累积概率');
title('累积分布函数');
grid on;

% 子图3: 距离矩阵热图
subplot(1, 3, 3);
imagesc(distances);
colorbar;
colormap('hot');
title('特征距离矩阵');
xlabel('图像索引');
ylabel('图像索引');

% 保存
save_path = fullfile(data_dir, 'slam_results_hart_cornet', 'hart_feature_distance_analysis.png');
saveas(gcf, save_path);
fprintf('分析结果已保存: %s\n', save_path);
