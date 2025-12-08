function normImg = extract_features_hart_cornet_v2(img)
% EXTRACT_FEATURES_HART_CORNET_V2 HART+CORnet层次化特征提取（简化版V2）
%
% 实现CORnet层次化特征提取 + HART注意力机制
% 
% 核心思想：
% 1. V1层: 多方向Gabor滤波（模拟简单细胞）
% 2. V2层: 空间池化（模拟复杂细胞）
% 3. V4层: 多尺度融合 + CLAHE增强
% 4. IT层: 语义级特征表示
% 5. HART注意力: 空间注意力调制
% 
% 参数:
%   img - 输入图像 [H×W×3] 或 [H×W]
%
% 返回:
%   normImg - 归一化特征图 [H×W]

    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    
    %% V1层：多方向边缘检测（简化的Gabor）
    % 使用4个方向的梯度
    [Gx, Gy] = imgradientxy(img, 'sobel');
    
    % 0度方向（水平）
    orient_0 = abs(Gx);
    
    % 90度方向（垂直）
    orient_90 = abs(Gy);
    
    % 45度方向（对角线）
    orient_45 = abs(Gx + Gy) / sqrt(2);
    
    % 135度方向（对角线）
    orient_135 = abs(Gx - Gy) / sqrt(2);
    
    % 边缘强度（综合所有方向）
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % V1层输出：多方向特征融合
    v1_output = 0.3 * orient_0 + 0.3 * orient_90 + 0.2 * orient_45 + 0.2 * orient_135;
    
    %% V2层：空间池化和局部不变性
    % 轻度高斯平滑（模拟复杂细胞的空间整合）
    v2_output = imgaussfilt(v1_output, 0.8);
    
    %% V4层：多尺度特征融合 + CLAHE
    % CLAHE增强（模拟视觉皮层的对比度适应）
    img_clahe = adapthisteq(img, 'ClipLimit', 0.03, 'NumTiles', [4 4]);
    
    % 多尺度高斯金字塔（3个尺度）
    scale1 = imgaussfilt(img_clahe, 0.5);  % 精细尺度
    scale2 = imgaussfilt(img_clahe, 1.0);  % 中间尺度
    scale3 = imgaussfilt(img_clahe, 1.5);  % 粗糙尺度
    
    % 融合多尺度特征
    multiscale_features = 0.5 * scale1 + 0.3 * scale2 + 0.2 * scale3;
    
    % V4层输出：边缘特征 + 多尺度特征
    v4_output = 0.4 * v2_output + 0.6 * multiscale_features;
    
    %% IT层：高级语义特征
    % 使用Tanh激活函数（模拟神经元非线性）
    it_features = tanh(2 * v4_output);
    
    %% HART注意力机制
    % 计算3层注意力图
    
    % 底层注意力（边缘显著性）
    low_level_attention = edge_magnitude / (max(edge_magnitude(:)) + eps);
    
    % 中层注意力（强度对比）
    intensity_contrast = abs(img - imgaussfilt(img, 2.0));
    mid_level_attention = intensity_contrast / (max(intensity_contrast(:)) + eps);
    
    % 高层注意力（语义重要性，基于IT特征）
    high_level_attention = abs(it_features) / (max(abs(it_features(:))) + eps);
    
    % 融合三层注意力（30% 底层 + 40% 中层 + 30% 高层）
    attention_map = 0.30 * low_level_attention + ...
                    0.40 * mid_level_attention + ...
                    0.30 * high_level_attention;
    
    % 平滑注意力图
    attention_map = imgaussfilt(attention_map, 1.0);
    
    % 注意力调制
    attended_features = it_features .* (1.0 + attention_map);
    
    %% 时序建模（简化的LSTM效果）
    % 使用全局特征的加权平均模拟时序记忆
    persistent prev_features;
    
    if isempty(prev_features) || any(size(prev_features) ~= size(attended_features))
        % 第一帧或尺寸变化，直接使用当前特征
        temporal_features = attended_features;
    else
        % 融合历史特征（70%历史 + 30%当前）
        forget_gate = 0.7;
        input_gate = 0.3;
        temporal_features = forget_gate * prev_features + input_gate * attended_features;
    end
    
    % 更新历史特征
    prev_features = temporal_features;
    
    %% 最终特征融合
    % 60% 空间特征 + 40% 时序特征
    final_features = 0.6 * attended_features + 0.4 * temporal_features;
    
    %% 归一化输出
    normImg = (final_features - min(final_features(:))) / ...
              (max(final_features(:)) - min(final_features(:)) + eps);
end
