function normImg = hart_transformer_extractor_full_parameterized(img, transformer_enhance, fusion_weights)
% HART_TRANSFORMER_EXTRACTOR_FULL_PARAMETERIZED 完全参数化版本
%
% 用于网格搜索Transformer增强权重和特征融合权重
%
% 参数:
%   img - 输入图像 [H×W×C] 或 [H×W]
%   transformer_enhance - Self-Attention增强权重 (默认0.17)
%   fusion_weights - 特征融合权重 [dorsal, lstm, transformer, v1] (默认[0.18,0.32,0.32,0.18])
%
% 返回:
%   normImg - 归一化特征图 [H×W]

    % 默认参数
    if nargin < 2 || isempty(transformer_enhance)
        transformer_enhance = 0.17;
    end
    if nargin < 3 || isempty(fusion_weights)
        fusion_weights = [0.18, 0.32, 0.32, 0.18];
    end

    % 持久化变量
    persistent lstm_h lstm_c attention_center feature_history frame_count;
    
    if isempty(frame_count)
        frame_count = 0;
        lstm_h = [];
        lstm_c = [];
        attention_center = [0.5, 0.5];
        feature_history = [];
    end
    frame_count = frame_count + 1;
    
    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% 步骤1: Spatial Attention
    uy = attention_center(1) * H;
    ux = attention_center(2) * W;
    sy = 0.25 * H;
    sx = 0.25 * W;
    
    [X, Y] = meshgrid(1:W, 1:H);
    attention_map = exp(-((X-ux).^2 / (2*sx^2) + (Y-uy).^2 / (2*sy^2)));
    attention_map = attention_map / (sum(attention_map(:)) + eps);
    
    attended_img = img .* (1.0 + 2.0 * attention_map);
    
    %% 步骤2: V1层（Gabor滤波器）
    orientations = [0, 45, 90, 135];
    wavelength = 4;
    gabor_responses = zeros(H, W, length(orientations));
    
    for i = 1:length(orientations)
        theta = orientations(i) * pi / 180;
        sigma = wavelength / pi;
        kernel_size = 2 * ceil(3 * sigma) + 1;
        gabor_kernel = create_gabor_filter_full(kernel_size, theta, wavelength, sigma);
        gabor_responses(:,:,i) = abs(imfilter(attended_img, gabor_kernel, 'same', 'replicate'));
    end
    
    v1_output = mean(gabor_responses, 3);
    
    %% 步骤3: 双流架构
    % Dorsal Stream
    [Gx, Gy] = imgradientxy(attended_img, 'sobel');
    gradient_magnitude = sqrt(Gx.^2 + Gy.^2);
    dorsal_features = 0.6 * gradient_magnitude + 0.4 * v1_output;
    
    % Ventral Stream
    win = ones(3,3);
    local_mean = imfilter(attended_img, win, 'same', 'replicate') ./ numel(win);
    local_mean2 = imfilter(attended_img.^2, win, 'same', 'replicate') ./ numel(win);
    local_var = max(local_mean2 - local_mean.^2, 0);
    local_std = sqrt(local_var);
    % ★★★ 修复: 使用简单对比度增强替代adapthisteq，避免coder.isColumnMajor错误 ★★★
    ventral_enhanced = simple_contrast_enhance_full(attended_img);
    ventral_features = 0.4 * v1_output + 0.3 * local_std + 0.3 * ventral_enhanced;
    
    %% 步骤4: Transformer增强（使用参数化权重）
    dorsal_mean = mean(dorsal_features(:));
    dorsal_std = std(dorsal_features(:));
    ventral_mean = mean(ventral_features(:));
    ventral_std = std(ventral_features(:));
    
    global_context = [dorsal_mean, dorsal_std, ventral_mean, ventral_std];
    global_weight = mean(global_context);
    
    % ★★★ 使用参数化的Transformer增强权重 ★★★
    dorsal_normalized = (dorsal_features - dorsal_mean) / (dorsal_std + eps);
    dorsal_enhanced = dorsal_features + transformer_enhance * global_weight * dorsal_normalized;
    
    ventral_normalized = (ventral_features - ventral_mean) / (ventral_std + eps);
    ventral_enhanced = ventral_features + transformer_enhance * global_weight * ventral_normalized;
    
    transformer_features = 0.5 * dorsal_enhanced + 0.5 * ventral_enhanced;
    
    %% 步骤5: 历史特征存储
    max_history = 4;
    current_feature_vector = [mean(dorsal_features(:)); mean(ventral_features(:))];
    
    if isempty(feature_history)
        feature_history = current_feature_vector;
    else
        feature_history = [feature_history, current_feature_vector];
        if size(feature_history, 2) > max_history
            feature_history = feature_history(:, end-max_history+1:end);
        end
    end
    
    %% 步骤6: LSTM时序建模
    if isempty(lstm_h) || any(size(lstm_h) ~= size(ventral_features))
        lstm_h = ventral_features;
        lstm_c = ventral_features;
    end
    
    input_gate = 0.55;
    forget_gate = 0.6;
    output_gate = 0.92;
    
    combined_input = 0.6 * ventral_features + 0.4 * transformer_features;
    input_modulated = tanh(combined_input);
    
    lstm_c = forget_gate * lstm_c + input_gate * input_modulated;
    lstm_h = output_gate * tanh(lstm_c);
    
    %% 步骤7: 特征融合（使用参数化权重）
    % ★★★ 使用参数化的融合权重 ★★★
    fused_features = fusion_weights(1) * dorsal_features + ...
                     fusion_weights(2) * lstm_h + ...
                     fusion_weights(3) * transformer_features + ...
                     fusion_weights(4) * v1_output;
    
    final_features = fused_features .* (1.0 + 0.5 * attention_map);
    
    %% 步骤8: 更新注意力中心
    [~, max_idx] = max(final_features(:));
    [max_y, max_x] = ind2sub(size(final_features), max_idx);
    
    attention_center(1) = 0.72 * attention_center(1) + 0.28 * (max_y / H);
    attention_center(2) = 0.72 * attention_center(2) + 0.28 * (max_x / W);
    attention_center = max(0.2, min(0.8, attention_center));
    
    %% 步骤9: 归一化输出
    normImg = (final_features - min(final_features(:))) / ...
              (max(final_features(:)) - min(final_features(:)) + eps);
end

function kernel = create_gabor_filter_full(size, theta, wavelength, sigma)
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


function enhanced = simple_contrast_enhance_full(img)
% SIMPLE_CONTRAST_ENHANCE_FULL 简单对比度增强（替代adapthisteq）
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
