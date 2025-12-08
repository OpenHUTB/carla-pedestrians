%% 只测试VT创建 - 完全内联特征提取
% 这个脚本只测试VT创建逻辑，不涉及其他SLAM组件
% 完全绕过函数缓存问题

clear all; close all; clc;

fprintf('========================================\n');
fprintf('VT创建测试 - 内联特征提取\n');
fprintf('========================================\n');
fprintf('成功配置：ClipLimit=0.02 + imgaussfilt(0.5)\n');
fprintf('预期：VT数量 ~299个\n');
fprintf('========================================\n\n');

%% 配置参数
VT_MATCH_THRESHOLD = 0.07;
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 1.0;

% 图像尺寸
crop_y = 1:120;
crop_x = 1:160;
resize_y = 12;
resize_x = 16;

%% 加载数据
fprintf('[1/4] 加载数据...\n');
% 动态获取数据路径
currentDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(currentDir));
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
img_files = dir(fullfile(data_path, '*.png'));
num_frames = length(img_files);

% 加载位置信息
fusion_data = load(fullfile(data_path, 'slam_result_fusion.mat'));

fprintf('  总帧数: %d\n\n', num_frames);

%% 初始化VT存储
VT_TEMPLATES = struct('template', {}, 'active_decay', {}, 'x', {}, 'y', {}, 'z', {});
VT_ID_COUNT = 0;

%% 处理所有帧
fprintf('[2/4] 处理帧并创建VT...\n');
vt_creation_log = [];
feature_times = zeros(num_frames, 1);

for frame_idx = 1:num_frames
    if mod(frame_idx, 50) == 0
        fprintf('  进度: %d/%d (%.1f%%) - 当前VT数: %d\n', ...
            frame_idx, num_frames, 100*frame_idx/num_frames, VT_ID_COUNT);
    end
    
    % 读取图像
    img_file = fullfile(data_path, img_files(frame_idx).name);
    rawImg = imread(img_file);
    
    % 获取位置
    curr_x = fusion_data.pos(frame_idx, 1);
    curr_y = fusion_data.pos(frame_idx, 2);
    curr_z = fusion_data.pos(frame_idx, 3);
    
    %% ========== 内联特征提取（成功配置） ==========
    tic;
    
    % 预处理
    subImg = rawImg(crop_y, crop_x);
    vtResizedImg = imresize(subImg, [resize_y, resize_x]);
    
    % 转灰度
    if size(vtResizedImg, 3) == 3
        img = rgb2gray(vtResizedImg);
    else
        img = vtResizedImg;
    end
    img = double(img) / 255.0;
    
    % 【关键】成功配置的特征提取
    % 1. 对比度增强 (ClipLimit=0.02)
    img_enhanced = adapthisteq(img, 'ClipLimit', 0.02, 'NumTiles', [8 8]);
    
    % 2. 高斯平滑 (sigma=0.5) - 关键步骤！
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    
    % 3. 边缘检测
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 4. 融合
    combined = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    
    % 5. 归一化
    normVtImg = (combined - min(combined(:))) / (max(combined(:)) - min(combined(:)) + eps);
    
    feature_times(frame_idx) = toc;
    
    %% VT匹配逻辑
    NUM_VT = VT_ID_COUNT;
    
    if NUM_VT < 5
        % 前5个直接添加
        if NUM_VT > 0
            for k = 1:NUM_VT
                VT_TEMPLATES(k).active_decay = VT_TEMPLATES(k).active_decay - VT_GLOBAL_DECAY;
            end
        end
        
        VT_ID_COUNT = VT_ID_COUNT + 1;
        VT_TEMPLATES(VT_ID_COUNT).template = normVtImg;
        VT_TEMPLATES(VT_ID_COUNT).active_decay = 1.0;
        VT_TEMPLATES(VT_ID_COUNT).x = curr_x;
        VT_TEMPLATES(VT_ID_COUNT).y = curr_y;
        VT_TEMPLATES(VT_ID_COUNT).z = curr_z;
        
        vtId = VT_ID_COUNT;
        vt_creation_log(end+1) = frame_idx;
        
    else
        % 计算与所有VT的距离
        min_diff = realmax;
        best_vt = -1;
        
        for k = 1:NUM_VT
            template = VT_TEMPLATES(k).template;
            
            % 余弦距离
            f1 = normVtImg(:) / (norm(normVtImg(:)) + eps);
            f2 = template(:) / (norm(template(:)) + eps);
            diff = 1 - dot(f1, f2);
            
            if diff < min_diff
                min_diff = diff;
                best_vt = k;
            end
        end
        
        % 判断是否创建新VT
        if min_diff > VT_MATCH_THRESHOLD
            % 创建新VT
            VT_ID_COUNT = VT_ID_COUNT + 1;
            VT_TEMPLATES(VT_ID_COUNT).template = normVtImg;
            VT_TEMPLATES(VT_ID_COUNT).active_decay = 1.0;
            VT_TEMPLATES(VT_ID_COUNT).x = curr_x;
            VT_TEMPLATES(VT_ID_COUNT).y = curr_y;
            VT_TEMPLATES(VT_ID_COUNT).z = curr_z;
            
            vtId = VT_ID_COUNT;
            vt_creation_log(end+1) = frame_idx;
        else
            % 匹配已有VT
            vtId = best_vt;
            VT_TEMPLATES(vtId).active_decay = VT_ACTIVE_DECAY;
        end
        
        % Decay所有VT
        for k = 1:NUM_VT
            VT_TEMPLATES(k).active_decay = VT_TEMPLATES(k).active_decay - VT_GLOBAL_DECAY;
        end
    end
end

fprintf('\n');

%% 结果统计
fprintf('[3/4] 统计结果...\n');
fprintf('========================================\n');
fprintf('VT创建结果：\n');
fprintf('  总VT数量:          %d\n', VT_ID_COUNT);
fprintf('  平均特征提取时间:  %.4f秒/帧\n', mean(feature_times));
fprintf('  总特征提取时间:    %.2f秒\n', sum(feature_times));
fprintf('  VT创建帧数:        %d\n', length(vt_creation_log));
fprintf('========================================\n\n');

%% 诊断信息
fprintf('[4/4] 诊断分析...\n');
fprintf('========================================\n');

if VT_ID_COUNT <= 10
    fprintf('⚠️  警告：VT数量异常少（%d个）\n\n', VT_ID_COUNT);
    fprintf('可能原因：\n');
    fprintf('  1. 特征提取参数有问题\n');
    fprintf('  2. 图像数据本身相似度太高\n');
    fprintf('  3. VT阈值设置不合适\n\n');
    
    fprintf('建议操作：\n');
    fprintf('  1. 运行 quick_feature_test 查看特征距离\n');
    fprintf('  2. 尝试降低VT阈值（0.07 -> 0.05）\n');
    fprintf('  3. 检查图像数据是否正常\n');
    
elseif VT_ID_COUNT < 100
    fprintf('⚠️  VT数量偏少（%d个）\n\n', VT_ID_COUNT);
    fprintf('可能原因：\n');
    fprintf('  1. VT阈值偏高\n');
    fprintf('  2. 特征差异性不够\n\n');
    
    fprintf('建议操作：\n');
    fprintf('  1. 尝试降低VT阈值（0.07 -> 0.05）\n');
    fprintf('  2. 运行 quick_feature_test 验证特征\n');
    
elseif VT_ID_COUNT > 200
    fprintf('✓ VT数量正常（%d个）！\n\n', VT_ID_COUNT);
    fprintf('特征提取配置成功！\n');
    fprintf('现在可以运行完整SLAM测试。\n');
    
else
    fprintf('VT数量适中（%d个）\n\n', VT_ID_COUNT);
    fprintf('可能需要微调阈值以优化性能。\n');
end

fprintf('========================================\n\n');

%% VT创建分布可视化
if VT_ID_COUNT > 5
    fprintf('生成VT创建分布图...\n');
    
    figure('Name', 'VT创建分析');
    
    % VT创建时间分布
    subplot(2,1,1);
    histogram(vt_creation_log, 50);
    xlabel('帧数');
    ylabel('VT创建次数');
    title(sprintf('VT创建分布（总计%d个VT）', VT_ID_COUNT));
    grid on;
    
    % 特征提取时间
    subplot(2,1,2);
    plot(feature_times * 1000);
    xlabel('帧数');
    ylabel('时间 (ms)');
    title(sprintf('特征提取时间（平均%.2fms）', mean(feature_times)*1000));
    grid on;
    
    fprintf('✓ 可视化完成\n');
end

fprintf('\n========================================\n');
fprintf('测试完成！\n');
fprintf('========================================\n');
