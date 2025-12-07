%% 验证特征提取并运行测试
% 确保修改生效

clear all; close all; clc;

fprintf('========================================\n');
fprintf('验证特征提取修复\n');
fprintf('========================================\n\n');

%% 添加路径
rootDir = '/home/dream/neuro_111111/carla-pedestrians/neuro';
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(rootDir, '09_vestibular'));

%% 步骤1: 快速验证特征提取
fprintf('[1/3] 快速验证特征提取...\n');

data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));

% 测试2张图像
features_list = cell(2, 1);
for i = 1:2
    img_path = fullfile(data_path, img_files(i*100).name);
    img = imread(img_path);
    
    % 调整尺寸
    if size(img, 1) > 120 || size(img, 2) > 160
        img = img(1:120, 1:160, :);
    end
    img = imresize(img, [12, 16]);
    
    % 使用visual_template_hart_cornet的内联特征提取
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    normVtImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
    
    features_list{i} = normVtImg;
    fprintf('  图像%d: 范围[%.4f, %.4f], 均值%.4f\n', ...
        i, min(normVtImg(:)), max(normVtImg(:)), mean(normVtImg(:)));
end

% 计算余弦距离
f1 = features_list{1}(:) / (norm(features_list{1}(:)) + eps);
f2 = features_list{2}(:) / (norm(features_list{2}(:)) + eps);
cosine_dist = 1 - dot(f1, f2);

fprintf('  两张图像余弦距离: %.6f\n', cosine_dist);
if cosine_dist < 0.07
    fprintf('  ⚠️  警告: 距离小于阈值0.07，可能仍会创建VT太少\n');
else
    fprintf('  ✓ 距离大于阈值，应该能创建足够的VT\n');
end

fprintf('\n');

%% 步骤2: 清除所有缓存
fprintf('[2/3] 清除所有缓存...\n');
clear all; close all; clc;

% 重新添加路径
rootDir = '/home/dream/neuro_111111/carla-pedestrians/neuro';
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(rootDir, '09_vestibular'));

rehash toolboxcache;
rehash path;

fprintf('  ✓ 缓存已清除\n\n');

%% 步骤3: 运行测试
fprintf('[3/3] 开始运行SLAM测试...\n');
fprintf('========================================\n\n');

test_imu_visual_slam_hart_cornet;
