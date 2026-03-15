function normImg = hart_transformer_extractor_parameterized(img, input_gate_param, forget_gate_param, output_gate_param)
% HART_TRANSFORMER_EXTRACTOR_PARAMETERIZED 参数化版本的HART+Transformer特征提取器
%
% 用于网格搜索LSTM门控参数
%
% 参数:
%   img - 输入图像 [H×W×C] 或 [H×W]
%         特殊值: 如果img是字符串'reset'，则重置所有持久化变量
%   input_gate_param - 输入门参数 (默认0.55)
%   forget_gate_param - 遗忘门参数 (默认0.6)
%   output_gate_param - 输出门参数 (默认0.92)
%
% 返回:
%   normImg - 归一化特征图 [H×W]
%
% 重置持久化变量:
%   hart_transformer_extractor_parameterized('reset');

    % 持久化变量（时序状态）
    persistent lstm_h lstm_c attention_center feature_history frame_count;
    
    % ★★★ 支持重置持久化变量 ★★★
    if ischar(img) && strcmpi(img, 'reset')
        lstm_h = [];
        lstm_c = [];
        attention_center = [];
        feature_history = [];
        frame_count = [];
        normImg = [];
        return;
    end

    % 默认参数
    if nargin < 2 || isempty(input_gate_param)
        input_gate_param = 0.55;
    end
    if nargin < 3 || isempty(forget_gate_param)
        forget_gate_param = 0.6;
    end
    if nargin < 4 || isempty(output_gate_param)
        output_gate_param = 0.92;
    end
    
    % 初始化
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
        gabor_kernel = create_gabor_filter_param(kernel_size, theta, wavelength, sigma);
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
    % adapthisteq内部使用mean2，在某些MATLAB版本会报错
    ventral_enhanced = simple_contrast_enhance(attended_img);
    ventral_features = 0.4 * v1_output + 0.3 * local_std + 0.3 * ventral_enhanced;
    
    %% ========== 步骤4: Transformer-Inspired Feature Enhancement ==========
    
    % 全局特征统计
    dorsal_mean = mean(dorsal_features(:));
    dorsal_std = std(dorsal_features(:));
    ventral_mean = mean(ventral_features(:));
    ventral_std = std(ventral_features(:));
    
    global_context = [dorsal_mean, dorsal_std, ventral_mean, ventral_std];
    global_weight = mean(global_context);
    
    % Self-Attention效果
    dorsal_normalized = (dorsal_features - dorsal_mean) / (dorsal_std + eps);
    dorsal_enhanced = dorsal_features + 0.17 * global_weight * dorsal_normalized;
    
    ventral_normalized = (ventral_features - ventral_mean) / (ventral_std + eps);
    ventral_enhanced = ventral_features + 0.17 * global_weight * ventral_normalized;
    
    % 多头融合
    transformer_features = 0.5 * dorsal_enhanced + 0.5 * ventral_enhanced;
    
    %% ========== 步骤5: 历史特征存储 ==========
    
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
    
    %% ========== 步骤6: LSTM时序建模（使用参数化门控） ==========
    
    % 初始化LSTM状态
    if isempty(lstm_h) || any(size(lstm_h) ~= size(ventral_features))
        lstm_h = ventral_features;
        lstm_c = ventral_features;
    end
    
    % ★★★ 使用参数化的LSTM门控 ★★★
    input_gate = input_gate_param;
    forget_gate = forget_gate_param;
    output_gate = output_gate_param;
    
    % 输入调制
    combined_input = 0.6 * ventral_features + 0.4 * transformer_features;
    input_modulated = tanh(combined_input);
    
    % 更新LSTM状态
    lstm_c = forget_gate * lstm_c + input_gate * input_modulated;
    lstm_h = output_gate * tanh(lstm_c);
    
    %% ========== 步骤7: 特征融合 ==========
    fused_features = 0.18 * dorsal_features + ...
                     0.32 * lstm_h + ...
                     0.32 * transformer_features + ...
                     0.18 * v1_output;
    
    % 再次应用空间注意力调制
    final_features = fused_features .* (1.0 + 0.5 * attention_map);
    
    %% ========== 步骤8: 更新注意力中心 ==========
    
    [~, max_idx] = max(final_features(:));
    [max_y, max_x] = ind2sub(size(final_features), max_idx);
    
    attention_center(1) = 0.72 * attention_center(1) + 0.28 * (max_y / H);
    attention_center(2) = 0.72 * attention_center(2) + 0.28 * (max_x / W);
    attention_center = max(0.2, min(0.8, attention_center));
    
    %% ========== 步骤9: 归一化输出 ==========
    normImg = (final_features - min(final_features(:))) / ...
              (max(final_features(:)) - min(final_features(:)) + eps);
end


%% ========== 辅助函数 ==========

function kernel = create_gabor_filter_param(size, theta, wavelength, sigma)
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

function enhanced = simple_contrast_enhance(img)
% SIMPLE_CONTRAST_ENHANCE 简单对比度增强（替代adapthisteq）
% 避免使用mean2函数，兼容所有MATLAB版本
%
% 使用局部对比度增强方法：
%   1. 计算局部均值和标准差
%   2. 进行局部归一化
%   3. 限制对比度增强幅度

    % 确保输入是double类型
    img = double(img);
    
    % 局部窗口大小 (模拟adapthisteq的NumTiles=[4,4])
    [H, W] = size(img);
    tile_h = max(1, floor(H / 4));
    tile_w = max(1, floor(W / 4));
    
    % 使用滑动窗口计算局部统计量
    win_size = max(tile_h, tile_w);
    if mod(win_size, 2) == 0
        win_size = win_size + 1;
    end
    
    % 创建均值滤波器
    h = ones(win_size, win_size) / (win_size * win_size);
    
    % 计算局部均值
    local_mean = imfilter(img, h, 'replicate');
    
    % 计算局部方差
    local_mean_sq = imfilter(img.^2, h, 'replicate');
    local_var = max(local_mean_sq - local_mean.^2, 0);
    local_std = sqrt(local_var) + eps;
    
    % 局部对比度增强
    enhanced = (img - local_mean) ./ local_std;
    
    % 限制对比度 (模拟ClipLimit=0.02)
    clip_limit = 3.0;  % 约等于ClipLimit=0.02的效果
    enhanced = max(-clip_limit, min(clip_limit, enhanced));
    
    % 归一化到[0,1]
    enhanced = (enhanced - min(enhanced(:))) / (max(enhanced(:)) - min(enhanced(:)) + eps);
end
