function normImg = hart_transformer_extractor(img)
% HART_TRANSFORMER_EXTRACTOR HART + Transformer混合特征提取器
%
% 架构设计：
%   - HART核心：Spatial Attention + 双流架构
%   - Transformer创新：Self-Attention替换部分LSTM
%   - 保持时序建模能力，增强特征交互
%
% 创新点：
%   1. 空间注意力（HART）+ 特征自注意力（Transformer）
%   2. 双流特征融合 + 多头注意力
%   3. 时序建模：LSTM门控 + Self-Attention
%
% 参数:
%   img - 输入图像 [H×W×C] 或 [H×W]
%
% 返回:
%   normImg - 归一化特征图 [H×W]

    % 持久化变量（时序状态）
    persistent lstm_h lstm_c attention_center feature_history frame_count;
    
    % 初始化
    if isempty(frame_count)
        frame_count = 0;
        lstm_h = [];
        lstm_c = [];
        attention_center = [0.5, 0.5];
        feature_history = [];  % 用于self-attention
    end
    frame_count = frame_count + 1;
    
    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% ========== 步骤1: Spatial Attention（HART空间注意力） ==========
    
    % 高斯注意力窗口
    uy = attention_center(1) * H;
    ux = attention_center(2) * W;
    sy = 0.25 * H;
    sx = 0.25 * W;
    
    [X, Y] = meshgrid(1:W, 1:H);
    attention_map = exp(-((X-ux).^2 / (2*sx^2) + (Y-uy).^2 / (2*sy^2)));
    attention_map = attention_map / (sum(attention_map(:)) + eps);
    
    % 应用空间注意力
    attended_img = img .* (1.0 + 2.0 * attention_map);
    
    %% ========== 步骤2: V1层（Gabor滤波器） ==========
    
    orientations = [0, 45, 90, 135];
    wavelength = 4;
    gabor_responses = zeros(H, W, length(orientations));
    
    for i = 1:length(orientations)
        theta = orientations(i) * pi / 180;
        sigma = wavelength / pi;
        kernel_size = 2 * ceil(3 * sigma) + 1;
        gabor_kernel = create_gabor_filter(kernel_size, theta, wavelength, sigma);
        gabor_responses(:,:,i) = abs(imfilter(attended_img, gabor_kernel, 'same', 'replicate'));
    end
    
    v1_output = mean(gabor_responses, 3);
    
    %% ========== 步骤3: 双流架构（Dorsal + Ventral） ==========
    
    % Dorsal Stream（位置）
    [Gx, Gy] = imgradientxy(attended_img, 'sobel');
    gradient_magnitude = sqrt(Gx.^2 + Gy.^2);
    dorsal_features = 0.6 * gradient_magnitude + 0.4 * v1_output;
    
    % Ventral Stream（特征）
    win = ones(3,3);
    local_mean = imfilter(attended_img, win, 'same', 'replicate') ./ numel(win);
    local_mean2 = imfilter(attended_img.^2, win, 'same', 'replicate') ./ numel(win);
    local_var = max(local_mean2 - local_mean.^2, 0);
    local_std = sqrt(local_var);
    % ★★★ 修复: 使用简单对比度增强替代adapthisteq，避免coder.isColumnMajor错误 ★★★
    ventral_enhanced = simple_contrast_enhance(attended_img);
    ventral_features = 0.4 * v1_output + 0.3 * local_std + 0.3 * ventral_enhanced;
    
    %% ========== 步骤4: Transformer-Inspired Feature Enhancement（创新点！） ==========
    
    % 简化的Self-Attention机制：全局-局部特征交互
    % 不做完整的QKV矩阵运算，而是用全局统计增强局部特征
    
    % 全局特征统计（模拟Query-Key匹配）
    dorsal_mean = mean(dorsal_features(:));
    dorsal_std = std(dorsal_features(:));
    ventral_mean = mean(ventral_features(:));
    ventral_std = std(ventral_features(:));
    
    % 全局上下文向量（模拟attention的全局信息）
    global_context = [dorsal_mean, dorsal_std, ventral_mean, ventral_std];
    global_weight = mean(global_context);  % 归一化权重
    
    % Self-Attention效果：用全局信息调制局部特征（保守优化）
    % Head 1: Dorsal特征增强
    dorsal_normalized = (dorsal_features - dorsal_mean) / (dorsal_std + eps);
    dorsal_enhanced = dorsal_features + 0.17 * global_weight * dorsal_normalized;  % 0.15→0.17 微调Transformer
    
    % Head 2: Ventral特征增强
    ventral_normalized = (ventral_features - ventral_mean) / (ventral_std + eps);
    ventral_enhanced = ventral_features + 0.17 * global_weight * ventral_normalized;  % 0.15→0.17
    
    % 多头融合（Transformer的multi-head概念）
    transformer_features = 0.5 * dorsal_enhanced + 0.5 * ventral_enhanced;
    
    %% ========== 步骤5: 历史特征存储（用于跨帧attention） ==========
    
    % 保存最近N帧的特征用于temporal attention
    max_history = 4;  % 保存4帧历史（适度增强）
    current_feature_vector = [mean(dorsal_features(:)); mean(ventral_features(:))];
    
    if isempty(feature_history)
        feature_history = current_feature_vector;
    else
        feature_history = [feature_history, current_feature_vector];
        if size(feature_history, 2) > max_history
            feature_history = feature_history(:, end-max_history+1:end);
        end
    end
    
    %% ========== 步骤6: LSTM时序建模（保留HART核心） ==========
    
    % 初始化LSTM状态
    if isempty(lstm_h) || any(size(lstm_h) ~= size(ventral_features))
        lstm_h = ventral_features;
        lstm_c = ventral_features;
    end
    
    % LSTM门控（网格搜索优化后的最优参数）
    input_gate = 0.50;    % 输入门（网格搜索最优值）
    forget_gate = 0.30;   % 遗忘门（网格搜索最优值）
    output_gate = 0.95;   % 输出门（网格搜索最优值）
    
    % 输入调制（ventral + transformer）
    combined_input = 0.6 * ventral_features + 0.4 * transformer_features;
    input_modulated = tanh(combined_input);
    
    % 更新LSTM状态
    lstm_c = forget_gate * lstm_c + input_gate * input_modulated;
    lstm_h = output_gate * tanh(lstm_c);
    
    %% ========== 步骤7: 特征融合（保守优化） ==========
    % 融合所有层次的特征（加权平均）
    fused_features = 0.18 * dorsal_features + ...      % 位置信息
                     0.32 * lstm_h + ...               % 时序记忆（微增）
                     0.32 * transformer_features + ... % 全局上下文（微增）
                     0.18 * v1_output;                 % 基础特征
    
    % 再次应用空间注意力调制
    final_features = fused_features .* (1.0 + 0.5 * attention_map);
    
    %% ========== 步骤8: 更新注意力中心（预测性注意力） ==========
    
    [~, max_idx] = max(final_features(:));
    [max_y, max_x] = ind2sub(size(final_features), max_idx);
    
    % 平滑更新（适度增强稳定性）
    attention_center(1) = 0.72 * attention_center(1) + 0.28 * (max_y / H);  % 0.7→0.72 微调
    attention_center(2) = 0.72 * attention_center(2) + 0.28 * (max_x / W);  % 微调
    attention_center = max(0.2, min(0.8, attention_center));
    
    %% ========== 步骤9: 归一化输出 ==========
    normImg = (final_features - min(final_features(:))) / ...
              (max(final_features(:)) - min(final_features(:)) + eps);
end


%% ========== 辅助函数 ==========

function kernel = create_gabor_filter(size, theta, wavelength, sigma)
    if mod(size, 2) == 0
        size = size + 1;
    end
    
    half_size = (size - 1) / 2;
    [x, y] = meshgrid(-half_size:half_size, -half_size:half_size);
    
    x_theta = x * cos(theta) + y * sin(theta);
    y_theta = -x * sin(theta) + y * cos(theta);
    
    gaussian = exp(-(x_theta.^2 + y_theta.^2) / (2 * sigma^2));
    sinusoid = cos(2 * pi * x_theta / wavelength);
    
    kernel = gaussian .* sinusoid;
    kernel = kernel / sum(abs(kernel(:)));
end

function y = softmax_1d(x)
    % 简化的1D softmax
    exp_x = exp(x - max(x));  % 数值稳定性
    y = exp_x / sum(exp_x);
end

function enhanced = simple_contrast_enhance(img)
% SIMPLE_CONTRAST_ENHANCE 简单对比度增强（替代adapthisteq）
% 避免使用mean2函数，兼容所有MATLAB版本

    img = double(img);
    [H, W] = size(img);
    tile_h = max(1, floor(H / 4));
    tile_w = max(1, floor(W / 4));
    
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
