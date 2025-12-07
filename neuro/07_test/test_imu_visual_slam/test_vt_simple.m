%% 简化VT测试 - 只测试特征提取和VT创建
% 不需要任何.mat文件，只用图像

clear all; close all; clc;

fprintf('========================================\n');
fprintf('简化VT测试 - 只测试特征提取\n');
fprintf('========================================\n');
fprintf('配置：ClipLimit=0.02 + imgaussfilt(0.5)\n');
fprintf('预期：VT数量应该 >200\n');
fprintf('========================================\n\n');

%% 配置
VT_MATCH_THRESHOLD = 0.07;
crop_y = 1:120;
crop_x = 1:160;
resize_y = 12;
resize_x = 16;

%% 加载图像
% 动态获取数据路径
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));
num_frames = min(length(img_files), 5000);

fprintf('[1/3] 找到%d张图像\n\n', num_frames);

%% 初始化VT
VT_TEMPLATES = [];
VT_COUNT = 0;
distances = [];

%% 处理图像
fprintf('[2/3] 处理图像并创建VT...\n');

for frame_idx = 1:num_frames
    if mod(frame_idx, 250) == 0
        fprintf('  进度: %d/%d (%.1f%%) - VT数量: %d\n', ...
            frame_idx, num_frames, 100*frame_idx/num_frames, VT_COUNT);
    end
    
    % 读取图像
    img_file = fullfile(data_path, img_files(frame_idx).name);
    rawImg = imread(img_file);
    
    % 预处理
    subImg = rawImg(crop_y, crop_x, :);
    vtResizedImg = imresize(subImg, [resize_y, resize_x]);
    
    %% ========== 特征提取（成功配置） ==========
    % 灰度化
    if size(vtResizedImg, 3) == 3
        img = rgb2gray(vtResizedImg);
    else
        img = vtResizedImg;
    end
    img = double(img) / 255.0;
    
    % 1. 对比度增强 (ClipLimit=0.02)
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
    
    % 2. 高斯平滑 (sigma=0.5)
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    
    % 3. 边缘检测
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 4. 融合
    combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    
    % 5. 归一化
    normVtImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
    
    %% VT匹配
    if VT_COUNT < 5
        % 前5个直接添加
        VT_COUNT = VT_COUNT + 1;
        VT_TEMPLATES{VT_COUNT} = normVtImg;
    else
        % 与所有VT比较
        min_diff = inf;
        
        for k = 1:VT_COUNT
            template = VT_TEMPLATES{k};
            
            % 余弦距离
            f1 = normVtImg(:) / (norm(normVtImg(:)) + eps);
            f2 = template(:) / (norm(template(:)) + eps);
            diff = 1 - dot(f1, f2);
            
            if diff < min_diff
                min_diff = diff;
            end
        end
        
        distances(end+1) = min_diff;
        
        % 创建新VT？
        if min_diff > VT_MATCH_THRESHOLD
            VT_COUNT = VT_COUNT + 1;
            VT_TEMPLATES{VT_COUNT} = normVtImg;
        end
    end
end

fprintf('\n');

%% 结果
fprintf('[3/3] 测试结果\n');
fprintf('========================================\n');
fprintf('VT数量: %d\n', VT_COUNT);
fprintf('VT距离统计:\n');
if ~isempty(distances)
    fprintf('  平均: %.6f\n', mean(distances));
    fprintf('  中位数: %.6f\n', median(distances));
    fprintf('  最小值: %.6f\n', min(distances));
    fprintf('  最大值: %.6f\n', max(distances));
    fprintf('  标准差: %.6f\n', std(distances));
    fprintf('  >阈值(%.2f)比例: %.1f%%\n', VT_MATCH_THRESHOLD, ...
        100*sum(distances>VT_MATCH_THRESHOLD)/length(distances));
end
fprintf('========================================\n\n');

%% 诊断
if VT_COUNT <= 10
    fprintf('❌ VT数量异常少！\n');
    fprintf('问题分析：\n');
    fprintf('  1. 平均距离: %.6f (应该>0.03)\n', mean(distances));
    fprintf('  2. 如果<0.01，说明特征过于相似\n');
    fprintf('  3. 可能需要降低阈值或检查特征提取\n');
elseif VT_COUNT < 100
    fprintf('⚠️  VT数量偏少\n');
    fprintf('可能原因：\n');
    fprintf('  1. 阈值偏高（当前%.2f）\n', VT_MATCH_THRESHOLD);
    fprintf('  2. 建议尝试0.05或0.06\n');
elseif VT_COUNT > 200
    fprintf('✅ VT数量正常！特征提取成功！\n');
    fprintf('说明：\n');
    fprintf('  1. 特征提取配置正确\n');
    fprintf('  2. 可以运行完整SLAM测试\n');
    fprintf('  3. 预期RMSE应该 <150米\n');
else
    fprintf('VT数量适中（%d个）\n', VT_COUNT);
end

fprintf('\n========================================\n');
fprintf('测试完成！\n');
fprintf('========================================\n');

%% 可视化
if VT_COUNT > 5 && ~isempty(distances)
    figure('Name', 'VT距离分布');
    
    subplot(2,1,1);
    histogram(distances, 50);
    hold on;
    xline(VT_MATCH_THRESHOLD, 'r--', 'LineWidth', 2, ...
        'Label', sprintf('阈值=%.2f', VT_MATCH_THRESHOLD));
    xlabel('余弦距离');
    ylabel('频数');
    title(sprintf('VT距离分布（总VT=%d）', VT_COUNT));
    grid on;
    
    subplot(2,1,2);
    plot(distances);
    hold on;
    yline(VT_MATCH_THRESHOLD, 'r--', 'LineWidth', 2);
    xlabel('帧数');
    ylabel('最小距离');
    title('每帧与最近VT的距离');
    grid on;
    
    fprintf('✓ 可视化完成\n');
end
