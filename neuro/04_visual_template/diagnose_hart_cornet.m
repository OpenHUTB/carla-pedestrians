%% HART+CORnet特征提取器诊断脚本
% 检查特征提取器是否正常工作

clear all; close all; clc;

fprintf('========== HART+CORnet诊断 ==========\n\n');

%% 添加路径
% 动态获取neuro根目录
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(currentDir);
addpath(fullfile(rootDir, '04_visual_template'));

%% 加载测试图像
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));

if isempty(img_files)
    error('未找到图像文件');
end

%% 测试特征提取
fprintf('测试1: 提取10张图像的特征\n');
num_test = min(10, length(img_files));
features_list = cell(num_test, 1);

for i = 1:num_test
    img_path = fullfile(data_path, img_files(i).name);
    img = imread(img_path);
    
    % 调整尺寸
    if size(img, 1) > 120 || size(img, 2) > 160
        img = img(1:120, 1:160, :);
    end
    img = imresize(img, [12, 16]);
    
    % 清除持久化状态（每次重新开始）
    if i == 1
        clear hart_cornet_feature_extractor;
    end
    
    % 提取特征
    features = hart_cornet_feature_extractor(img);
    features_list{i} = features;
    
    fprintf('  图像%d: 特征范围[%.4f, %.4f], 均值%.4f, 标准差%.4f\n', ...
        i, min(features(:)), max(features(:)), mean(features(:)), std(features(:)));
end

%% 测试2: 计算特征之间的余弦距离
fprintf('\n测试2: 计算特征间余弦距离\n');
distances = zeros(num_test, num_test);

for i = 1:num_test
    for j = 1:num_test
        f1 = features_list{i}(:);
        f2 = features_list{j}(:);
        
        % 归一化
        f1 = f1 / (norm(f1) + eps);
        f2 = f2 / (norm(f2) + eps);
        
        % 余弦距离
        cosine_sim = dot(f1, f2);
        distances(i, j) = 1 - cosine_sim;
    end
end

fprintf('\n余弦距离矩阵:\n');
disp(distances);

fprintf('\n距离统计:\n');
off_diag = distances(~eye(num_test));
fprintf('  最小距离: %.6f\n', min(off_diag));
fprintf('  最大距离: %.6f\n', max(off_diag));
fprintf('  平均距离: %.6f\n', mean(off_diag));
fprintf('  标准差: %.6f\n', std(off_diag));

%% 测试3: 检查VT阈值
fprintf('\n测试3: VT阈值分析\n');
thresholds = [0.01, 0.05, 0.07, 0.10, 0.15, 0.20];
for thresh = thresholds
    num_vt = sum(any(distances > thresh, 1));
    fprintf('  阈值%.3f: 预计创建%d个VT\n', thresh, num_vt);
end

%% 测试4: 可视化特征
fprintf('\n测试4: 可视化前3张图像的特征\n');
figure('Position', [100, 100, 1200, 400]);
for i = 1:min(3, num_test)
    subplot(2, 3, i);
    img_path = fullfile(data_path, img_files(i).name);
    img = imread(img_path);
    if size(img, 1) > 120 || size(img, 2) > 160
        img = img(1:120, 1:160, :);
    end
    img = imresize(img, [12, 16]);
    imshow(img, []);
    title(sprintf('原图 %d', i));
    
    subplot(2, 3, i+3);
    imshow(features_list{i}, []);
    title(sprintf('特征 %d', i));
    colorbar;
end
sgtitle('HART+CORnet特征可视化');

%% 诊断结论
fprintf('\n========================================\n');
fprintf('诊断结论:\n');
if mean(off_diag) < 0.05
    fprintf('  ⚠️  特征过于相似！平均距离仅%.6f\n', mean(off_diag));
    fprintf('  原因: HART+CORnet可能过度归一化\n');
    fprintf('  建议: 减少归一化强度或调整特征提取参数\n');
elseif mean(off_diag) > 0.5
    fprintf('  ⚠️  特征差异过大！平均距离%.6f\n', mean(off_diag));
    fprintf('  原因: 可能存在数值问题\n');
    fprintf('  建议: 检查特征提取流程\n');
else
    fprintf('  ✓ 特征距离正常，平均%.6f\n', mean(off_diag));
end
fprintf('========================================\n');
