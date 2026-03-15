function normImg = hart_cornet_feature_extractor(img, prev_state)
% HART_CORNET_FEATURE_EXTRACTOR 结合HART和CORnet的图像特征提取器
%
% 参考：
%   1. HART: Hierarchical Attentive Recurrent Tracking (主要)
%      - 层次化注意力机制
%      - 递归时序建模
%      - 动态特征提取
%   
%   2. CORnet: Brain-Like Object Recognition (次要)
%      - V1 -> V2 -> V4 -> IT 层次化结构
%      - 模拟视觉皮层处理流程
%
% 输入:
%   img        - 输入图像 [H x W x C] 或 [H x W]
%   prev_state - 上一帧的隐藏状态（可选，用于时序建模）
%
% 输出:
%   normImg    - 归一化特征图 [H' x W']
%
% 架构设计:
%   V1: 多尺度边缘检测（简单细胞）
%   V2: 局部特征池化（复杂细胞）
%   V4: 中层特征整合
%   IT: 高层特征表示
%   Attention: 空间注意力机制
%   Temporal: 递归状态更新
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12

    %% 持久化变量用于时序建模（模拟LSTM隐藏状态）
    persistent lstm_hidden_state;
    persistent lstm_cell_state;
    persistent frame_count;
    
    % 初始化持久化状态
    if isempty(frame_count)
        frame_count = 0;
        lstm_hidden_state = [];
        lstm_cell_state = [];
    end
    frame_count = frame_count + 1;
    
    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% ========== 简化特征提取（使用成功方法） ==========
    % 修复: 原层次化方法过度归一化导致特征相似
    % 改用: 简单但有效的特征提取
    
    % 自适应对比度增强（与成功版本完全一致）
    % ★★★ 修复: 使用简单对比度增强替代adapthisteq，避免coder.isColumnMajor错误 ★★★
    img_enhanced = simple_contrast_enhance_cornet(img);
    
    % 高斯平滑（与成功版本一致）
    img_smoothed = imgaussfilt(img_enhanced, 0.5);
    
    % 边缘检测
    [Gx, Gy] = imgradientxy(img_smoothed, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 融合强度和边缘
    base_features = 0.6 * img_smoothed + 0.4 * edge_magnitude;
    
    % 用base_features作为it_features（保持接口兼容）
    it_features = base_features;
    v1_features = edge_magnitude;  % 用于注意力计算
    v4_features = img_enhanced;    % 用于注意力计算
    
    %% ========== HART时序跟踪（主要） + CORnet特征（辅助） ==========
    % 重新启用HART：动态场景跟踪的关键
    % SLAM是连续帧处理，HART的时序建模非常重要
    
    % 1. HART空间注意力（层次化注意力机制）
    attention_map = compute_spatial_attention(img, v1_features, v4_features);
    attended_features = it_features .* (1.0 + 0.5 * attention_map);  % 注意力调制
    
    % 2. HART时序建模（LSTM递归状态更新）
    if isempty(lstm_hidden_state) || any(size(lstm_hidden_state) ~= size(attended_features))
        % 初始化LSTM状态
        lstm_hidden_state = attended_features;
        lstm_cell_state = attended_features;
        temporal_features = attended_features;
    else
        % LSTM更新
        [lstm_hidden_state, lstm_cell_state, temporal_features] = ...
            lstm_update(attended_features, lstm_hidden_state, lstm_cell_state);
    end
    
    % 3. 融合空间和时序特征（更平衡的配置，避免过度平滑）
    fused_features = 0.5 * temporal_features + 0.5 * attended_features;
    
    % 4. 归一化输出
    normImg = normalize_features(fused_features);
end


%% ========== V1层：简单细胞（多尺度边缘检测） ==========
function v1_out = v1_simple_cells(img)
    % 模拟V1简单细胞：对不同方向的边缘敏感
    % 使用多个Gabor滤波器和Sobel算子
    
    % 方法1: Gabor滤波器（多方向）
    orientations = [0, 45, 90, 135];  % 4个主要方向
    wavelengths = [4, 8];  % 2个尺度
    n_filters = length(orientations) * length(wavelengths);
    
    gabor_responses = zeros([size(img), n_filters]);
    idx = 1;
    for lambda = wavelengths
        for theta_deg = orientations
            theta = theta_deg * pi / 180;
            gabor_kernel = create_gabor_kernel(lambda, theta, 3, 3);
            gabor_responses(:, :, idx) = abs(imfilter(img, gabor_kernel, 'same', 'replicate'));
            idx = idx + 1;
        end
    end
    
    % 方法2: Sobel边缘（快速近似）
    [Gx, Gy] = imgradientxy(img, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 融合Gabor和Sobel特征
    gabor_pooled = mean(gabor_responses, 3);  % 池化所有Gabor响应
    v1_out = 0.5 * gabor_pooled + 0.5 * edge_magnitude;
end


%% ========== V2层：复杂细胞（局部池化） ==========
function v2_out = v2_complex_cells(v1_features)
    % 模拟V2复杂细胞：对位置有一定不变性
    % 使用局部最大池化和非线性变换
    
    % 局部最大池化（2x2）
    pooled = imerode(v1_features, strel('square', 2));  % 近似max pooling
    
    % 非线性激活（类似ReLU）
    v2_out = max(pooled, 0);
    
    % 轻度平滑
    v2_out = imgaussfilt(v2_out, 1.0);
end


%% ========== V4层：中层特征整合 ==========
function v4_out = v4_intermediate_features(v2_features)
    % 模拟V4：整合局部特征，形成中层表示
    % V4对形状、纹理等中层特征敏感
    
    % 多尺度特征提取
    scale1 = imgaussfilt(v2_features, 1.5);  % 细节
    scale2 = imgaussfilt(v2_features, 3.0);  % 中等
    scale3 = imgaussfilt(v2_features, 5.0);  % 粗略
    
    % 融合多尺度特征
    v4_out = 0.5 * scale1 + 0.3 * scale2 + 0.2 * scale3;
    
    % 对比度增强
    % ★★★ 修复: 使用简单对比度增强替代adapthisteq，避免coder.isColumnMajor错误 ★★★
    v4_out = simple_contrast_enhance_cornet(v4_out);
end


%% ========== IT层：高层特征 ==========
function it_out = it_high_level_features(v4_features)
    % 模拟IT（Inferotemporal）：高层语义特征
    % IT对物体类别和身份敏感
    
    % 全局统计特征
    global_mean = mean(v4_features(:));
    global_std = std(v4_features(:));
    
    % 标准化
    it_out = (v4_features - global_mean) / (global_std + eps);
    
    % 非线性变换（tanh-like）
    it_out = tanh(it_out);
    
    % 平滑整合
    it_out = imgaussfilt(it_out, 2.0);
end


%% ========== HART注意力机制 ==========
function attention = compute_spatial_attention(img, v1_features, v4_features)
    % HART的层次化空间注意力
    % 结合低层（V1）和中层（V4）特征来计算显著性
    
    % === 底层注意力：基于边缘和对比度 ===
    low_level_saliency = v1_features;
    
    % === 中层注意力：基于纹理和结构 ===
    mid_level_saliency = v4_features;
    
    % === 高层注意力：基于局部对比 ===
    % 计算局部对比度
    img_blurred = imgaussfilt(img, 5);
    high_level_saliency = abs(img - img_blurred);
    
    % 归一化各层注意力
    low_level_saliency = normalize_map(low_level_saliency);
    mid_level_saliency = normalize_map(mid_level_saliency);
    high_level_saliency = normalize_map(high_level_saliency);
    
    % 层次化融合（类似HART的多层注意力）
    % 30% 底层 + 40% 中层 + 30% 高层
    attention = 0.3 * low_level_saliency + ...
                0.4 * mid_level_saliency + ...
                0.3 * high_level_saliency;
    
    % 平滑注意力图
    attention = imgaussfilt(attention, 2.0);
    
    % 归一化到[0, 1]
    attention = normalize_map(attention);
    
    % 确保最小注意力值（避免完全抑制）
    attention = max(attention, 0.1);
end


%% ========== HART时序建模（简化LSTM） ==========
function [h_new, c_new, output] = lstm_update(x, h_prev, c_prev)
    % 简化的LSTM单元，用于时序建模
    % 参考HART的递归结构
    
    % LSTM门控参数（调整为更敏感的配置）
    % 降低历史保留，增加对新场景的响应
    forget_rate = 0.4;   % 遗忘门（保留40%历史）
    input_rate = 0.6;    % 输入门（接受60%新信息，增强场景区分）
    output_rate = 0.9;   % 输出门（控制特征输出强度）
    
    % === 遗忘门：决定保留多少历史信息 ===
    c_forgotten = forget_rate * c_prev;
    
    % === 输入门：决定接受多少新信息 ===
    c_input = input_rate * tanh(x);
    
    % === 更新细胞状态 ===
    c_new = c_forgotten + c_input;
    
    % === 输出门：决定输出什么 ===
    h_new = output_rate * tanh(c_new);
    
    % === 输出特征 ===
    output = h_new;
end


%% ========== 辅助函数 ==========

function kernel = create_gabor_kernel(wavelength, theta, sigma_x, sigma_y)
    % 创建Gabor滤波器核
    kernel_size = round(3 * max(sigma_x, sigma_y));
    if mod(kernel_size, 2) == 0
        kernel_size = kernel_size + 1;
    end
    
    [x, y] = meshgrid(-(kernel_size-1)/2:(kernel_size-1)/2, ...
                      -(kernel_size-1)/2:(kernel_size-1)/2);
    
    % 旋转坐标系
    x_theta = x * cos(theta) + y * sin(theta);
    y_theta = -x * sin(theta) + y * cos(theta);
    
    % Gabor函数
    gaussian = exp(-(x_theta.^2 / (2*sigma_x^2) + y_theta.^2 / (2*sigma_y^2)));
    sinusoid = cos(2 * pi * x_theta / wavelength);
    
    kernel = gaussian .* sinusoid;
    kernel = kernel / sum(abs(kernel(:)));  % 归一化
end

function normalized = normalize_map(map)
    % 归一化到[0, 1]
    min_val = min(map(:));
    max_val = max(map(:));
    if max_val - min_val < eps
        normalized = zeros(size(map));
    else
        normalized = (map - min_val) / (max_val - min_val);
    end
end

function normalized = normalize_features(features)
    % 特征归一化 - 简单Min-Max归一化
    % 与成功版本保持一致
    min_val = min(features(:));
    max_val = max(features(:));
    if abs(max_val - min_val) > eps
        normalized = (features - min_val) / (max_val - min_val);
    else
        normalized = zeros(size(features));
    end
end


function enhanced = simple_contrast_enhance_cornet(img)
% SIMPLE_CONTRAST_ENHANCE_CORNET 简单对比度增强（替代adapthisteq）
% 避免使用mean2函数，兼容所有MATLAB版本
    img = double(img);
    [H, W] = size(img);
    tile_h = max(1, floor(H / 8));  % 模拟NumTiles=[8,8]
    tile_w = max(1, floor(W / 8));
    
    win_size = max(tile_h, tile_w);
    if mod(win_size, 2) == 0
        win_size = win_size + 1;
    end
    
    h = ones(win_size, win_size) / (win_size * win_size);
    local_mean = imfilter(img, h, 'replicate');
    local_mean_sq = imfilter(img.^2, h, 'replicate');
    local_var = max(local_mean_sq - local_mean.^2, 0);
    local_std = sqrt(local_var) + eps;
    
    enhanced = (img - local_mean) ./ local_std;
    clip_limit = 3.0;
    enhanced = max(-clip_limit, min(clip_limit, enhanced));
    enhanced = (enhanced - min(enhanced(:))) / (max(enhanced(:)) - min(enhanced(:)) + eps);
end
