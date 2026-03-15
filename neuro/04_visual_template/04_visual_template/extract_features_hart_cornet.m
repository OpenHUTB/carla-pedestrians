function features = extract_features_hart_cornet(img)
% EXTRACT_FEATURES_HART_CORNET 基于HART+CORnet的特征提取器
%
% 参考文献:
%   [1] HART: Hierarchical Attentive Recurrent Tracking (NIPS 2017)
%       - 空间注意力机制
%       - 层次化特征提取
%   [2] CORnet: Brain-Like Object Recognition (NeurIPS 2018)
%       - V1 → V2 → V4 → IT 视觉皮层层次结构
%       - 每层: Conv → Norm → ReLU
%
% 设计思路:
%   1. CORnet层次化结构提取多尺度特征
%   2. HART注意力机制聚焦显著区域
%   3. 纯MATLAB实现,无需Python/PyTorch
%   4. 快速高效,适合SLAM实时性要求
%
% 输入:
%   img - 输入图像 [H x W] or [H x W x 3]
%
% 输出:
%   features - 归一化特征图 [H' x W']
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12-02

    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% ========== CORnet 层次化特征提取 ==========
    
    % === V1层: 早期视觉处理（边缘、方向） ===
    % 模拟简单细胞和复杂细胞
    v1_features = cornet_v1_layer(img);
    
    % === V2层: 中层特征整合（纹理、形状） ===
    v2_features = cornet_v2_layer(v1_features, img);
    
    % === V4层: 高层形状和对象部分 ===
    v4_features = cornet_v4_layer(v2_features);
    
    % === IT层: 不变性对象表示 ===
    it_features = cornet_it_layer(v4_features);
    
    %% ========== HART 注意力机制 ==========
    
    % 计算空间注意力图
    % 聚焦于场景中的显著区域（道路、建筑物边缘等）
    attention_map = hart_spatial_attention(v1_features, v4_features, it_features);
    
    % 应用注意力加权（减弱注意力强度，保留更多原始信息）
    attended_features = 0.7 * it_features + 0.3 * (it_features .* attention_map);
    
    %% ========== 后处理 ==========
    
    % 保留更多动态范围，避免过度归一化
    % 使用标准化而不是Min-Max归一化
    feat_mean = mean(attended_features(:));
    feat_std = std(attended_features(:));
    features = (attended_features - feat_mean) / (feat_std + eps);
    
    % 裁剪极值并映射到[0,1]
    features = max(min(features, 3), -3);  % Clip to [-3σ, 3σ]
    features = (features + 3) / 6;  % Map to [0, 1]
end


%% ========== CORnet V1层: 简单细胞（多方向边缘检测） ==========
function v1_out = cornet_v1_layer(img)
    % V1层模拟初级视觉皮层的简单细胞和复杂细胞
    % 功能: 检测多方向、多尺度的边缘
    
    % 多方向Gabor滤波器（模拟简单细胞）
    orientations = 4;  % 0°, 45°, 90°, 135°
    scales = 2;  % 2个尺度
    
    responses = zeros(size(img, 1), size(img, 2), orientations * scales);
    idx = 1;
    
    for scale = [3, 5]  % 小尺度和中尺度
        for ori = 0:(orientations-1)
            theta = ori * pi / orientations;
            % 创建Gabor滤波器
            gabor_filter = create_simple_gabor(scale, theta);
            % 卷积
            response = imfilter(img, gabor_filter, 'same', 'replicate');
            responses(:, :, idx) = abs(response);  % 复杂细胞的不变性
            idx = idx + 1;
        end
    end
    
    % 池化多方向响应（模拟复杂细胞）
    v1_out = max(responses, [], 3);  % Max pooling across orientations
end


%% ========== CORnet V2层: 复杂特征整合 ==========
function v2_out = cornet_v2_layer(v1_features, original_img)
    % V2层整合V1的局部特征，形成更复杂的纹理和形状表示
    
    % 1. 局部对比度归一化（模拟侧抑制）
    lcn_features = local_contrast_normalization(v1_features, 5);
    
    % 2. 融合原始强度信息和边缘信息
    % 这样既保留了场景的整体结构，也强调了边缘细节
    img_normalized = (original_img - min(original_img(:))) / (max(original_img(:)) - min(original_img(:)) + eps);
    
    v2_out = 0.6 * img_normalized + 0.4 * lcn_features;
end


%% ========== CORnet V4层: 中层特征池化 ==========
function v4_out = cornet_v4_layer(v2_features)
    % V4层进行空间池化，增加感受野，形成更抽象的特征
    
    % 多尺度池化
    pool1 = imresize(v2_features, 1.0);  % 原始尺度
    pool2 = imresize(imgaussfilt(v2_features, 1.0), 1.0);  % 小尺度平滑
    pool3 = imresize(imgaussfilt(v2_features, 2.0), 1.0);  % 大尺度平滑
    
    % 融合多尺度特征
    v4_out = 0.5 * pool1 + 0.3 * pool2 + 0.2 * pool3;
end


%% ========== CORnet IT层: 高层不变性表示 ==========
function it_out = cornet_it_layer(v4_features)
    % IT层产生对位置、尺度部分不变的高层语义特征
    % 简化版本：减少过度归一化，保留场景差异
    
    % 1. 轻度平滑（增加一点不变性）
    it_out = imgaussfilt(v4_features, 0.3);
    
    % 2. 不再做归一化，保留原始动态范围
    % 让后续的标准化处理这个问题
end


%% ========== HART 空间注意力 ==========
function attention_map = hart_spatial_attention(v1_features, v4_features, it_features)
    % HART的空间注意力机制
    % 功能: 识别场景中的显著区域
    
    % 1. 底层显著性（边缘强度）
    low_level_saliency = v1_features;
    
    % 2. 中层显著性（纹理对比）
    mid_level_saliency = imgradient(v4_features);
    
    % 3. 高层显著性（语义重要性）
    high_level_saliency = abs(it_features - mean(it_features(:)));
    
    % 4. 融合多层显著性
    % 底层权重高（边缘对SLAM很重要），高层权重低（语义不太重要）
    combined_saliency = 0.5 * low_level_saliency + ...
                       0.3 * mid_level_saliency + ...
                       0.2 * high_level_saliency;
    
    % 5. 轻度Softmax归一化（避免过度集中）
    attention_map = exp(combined_saliency * 2);  % 降低放大系数（原5→2）
    attention_map = attention_map / (sum(attention_map(:)) + eps);
    attention_map = attention_map * numel(attention_map);  % 恢复尺度
    
    % 6. 限制注意力范围（更温和的范围）
    attention_map = min(attention_map, 1.5);  % 最大1.5倍增强（原2.0）
    attention_map = max(attention_map, 0.7);  % 最小0.7倍抑制（原0.5）
end


%% ========== 辅助函数 ==========

function gabor = create_simple_gabor(sigma, theta)
    % 创建简化的Gabor滤波器
    kernel_size = ceil(sigma * 3);
    if mod(kernel_size, 2) == 0
        kernel_size = kernel_size + 1;
    end
    
    [x, y] = meshgrid(-floor(kernel_size/2):floor(kernel_size/2), ...
                      -floor(kernel_size/2):floor(kernel_size/2));
    
    % 旋转坐标
    x_theta = x * cos(theta) + y * sin(theta);
    y_theta = -x * sin(theta) + y * cos(theta);
    
    % Gabor函数
    wavelength = sigma;
    gaussian = exp(-(x_theta.^2 + y_theta.^2) / (2 * sigma^2));
    sinusoid = cos(2 * pi * x_theta / wavelength);
    
    gabor = gaussian .* sinusoid;
    gabor = gabor / (sum(abs(gabor(:))) + eps);
end


function lcn = local_contrast_normalization(img, window_size)
    % 局部对比度归一化（模拟侧抑制）
    % 增强局部差异，抑制全局偏差
    
    local_mean = imboxfilt(img, window_size);
    local_std = stdfilt(img, ones(window_size));
    
    lcn = (img - local_mean) ./ (local_std + 0.01);
    
    % 限制范围
    lcn = max(min(lcn, 3), -3);  % Clip to [-3, 3]
    lcn = (lcn + 3) / 6;  % Normalize to [0, 1]
end
