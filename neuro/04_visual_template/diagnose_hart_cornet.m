%% HART+CORnet特征提取器诊断脚本
% 检查特征提取器是否正常工作
% 无硬编码路径 | 可跨平台运行 | 可直接提交

clear all; close all; clc;
fprintf('========== HART+CORnet 诊断 ==========\n\n');

%% ====================== 修复：自动获取当前路径 ======================
rootDir = fileparts(mfilename('fullpath'));  % 自动获取脚本所在目录
addpath(fullfile(rootDir, '04_visual_template'));

%% 加载测试图像（自动寻找同目录下 data 文件夹）
data_path = fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets', 'Town01Data_IMU_Fusion');

if ~exist(data_path, 'dir')
    error('❌ 数据文件夹不存在: %s', data_path);
end

img_files = dir(fullfile(data_path, '*.png'));
if isempty(img_files)
    error('❌ 未找到任何 PNG 图像');
end

%% 测试1: 提取10张图像的特征
fprintf('✅ 测试1: 提取10张图像特征\n');
num_test = min(10, length(img_files));
features_list = cell(num_test, 1);

for i = 1:num_test
    img_path = fullfile(data_path, img_files(i).name);
    img = imread(img_path);
    
    % 标准VT尺寸
    img = imresize(rgb2gray(img), [12, 16]);
    
    % 重置LSTM状态
    if i == 1
        clear hart_cornet_feature_extractor;
    end
    
    % 提取特征
    features = hart_cornet_feature_extractor(img);
    features_list{i} = features;
    
    fprintf('  图像%d | 范围[%.4f, %.4f] | 均值%.4f | 标准差%.4f\n', ...
        i, min(features(:)), max(features(:)), mean(features(:)), std(features(:)));
end

%% 测试2: 余弦距离
fprintf('\n✅ 测试2: 特征余弦距离\n');
distances = zeros(num_test, num_test);

for i = 1:num_test
    for j = 1:num_test
        f1 = features_list{i}(:) / (norm(features_list{i}(:)) + eps);
        f2 = features_list{j}(:) / (norm(features_list{j}(:)) + eps);
        distances(i,j) = 1 - dot(f1, f2);
    end
end

fprintf('距离矩阵:\n');
disp(distances);

off_diag = distances(~eye(num_test));
fprintf('\n📊 距离统计:\n');
fprintf('  最小: %.6f\n', min(off_diag));
fprintf('  最大: %.6f\n', max(off_diag));
fprintf('  平均: %.6f\n', mean(off_diag));
fprintf('  标准差: %.6f\n', std(off_diag));

%% 测试3: VT阈值预测
fprintf('\n✅ 测试3: VT阈值预测\n');
thresholds = [0.01, 0.05, 0.07, 0.10, 0.15, 0.20];
for thresh = thresholds
    num_vt = sum(any(distances > thresh, 1));
    fprintf('  阈值 %.3f → 预计VT数: %d\n', thresh, num_vt);
end

%% 测试4: 可视化
fprintf('\n✅ 测试4: 特征可视化\n');
figure('Position', [100, 100, 1200, 400]);
for i = 1:min(3, num_test)
    % 原图
    subplot(2,3,i);
    img = imread(fullfile(data_path, img_files(i).name));
    imshow(imresize(img, [120, 160]));
    title(['原图 ' num2str(i)]);
    
    % 特征图
    subplot(2,3,i+3);
    imshow(features_list{i}, []);
    colorbar;
    title(['HART+CORnet 特征 ' num2str(i)]);
end
sgtitle('HART+CORnet 特征提取诊断');

%% 最终结论
fprintf('\n========================================\n');
fprintf('📌 诊断结论:\n');
avg_dist = mean(off_diag);

if avg_dist < 0.05
    fprintf('⚠️  特征过于相似 → 可能会生成过多VT\n');
elseif avg_dist > 0.5
    fprintf('⚠️  特征差异过大 → 可能不稳定\n');
else
    fprintf('✅ 特征正常 | 平均距离 = %.6f\n', avg_dist);
    fprintf('✅ HART+CORnet 工作正常！\n');
end
fprintf('========================================\n\n');
