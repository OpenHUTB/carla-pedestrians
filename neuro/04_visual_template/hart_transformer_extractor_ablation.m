function features = hart_transformer_extractor_ablation(img, config)
%% HART+Transformer特征提取器 - 消融实验版本
% 支持动态开关各个组件
% 
% 输入:
%   img - 输入图像 [H x W]
%   config - 配置结构体
%       .attention - 是否使用空间注意力
%       .dual_stream - 是否使用双流架构
%       .transformer - 是否使用Transformer
%       .lstm - 是否使用LSTM
%       .full_feature - 是否使用完整特征 (false则使用简化版)
%
% 输出:
%   features - 提取的特征 [H x W]

persistent lstm_state attention_center feature_history

%% 检查配置
if nargin < 2
    % 默认配置：完整系统
    config = struct();
    config.attention = true;
    config.dual_stream = true;
    config.transformer = true;
    config.lstm = true;
    config.full_feature = true;
end

%% 如果使用简化特征，直接返回基础处理
if ~config.full_feature
    % 简化特征提取（对比baseline）
    img_gray = double(img) / 255;
    img_enhanced = adapthisteq(img_gray, 'ClipLimit', 0.02);
    img_smooth = imgaussfilt(img_enhanced, 0.5);
    features = (img_smooth - min(img_smooth(:))) / (max(img_smooth(:)) - min(img_smooth(:)) + eps);
    return;
end

%% 初始化
if isempty(lstm_state)
    lstm_state = struct();
    lstm_state.h = zeros(size(img));
    lstm_state.c = zeros(size(img));
end

if isempty(attention_center)
    attention_center = [size(img, 1)/2, size(img, 2)/2];
end

if isempty(feature_history)
    feature_history = {};
end

%% 归一化输入
img = double(img);
if max(img(:)) > 1
    img = img / 255;
end

%% ========== 步骤1: 空间注意力（可选） ==========
if config.attention
    % 使用空间注意力
    [h, w] = size(img);
    [X, Y] = meshgrid(1:w, 1:h);
    
    % 高斯注意力窗口
    sigma_y = h / 3;
    sigma_x = w / 3;
    attention_map = exp(-((Y - attention_center(1)).^2 / (2*sigma_y^2) + ...
                         (X - attention_center(2)).^2 / (2*sigma_x^2)));
    attention_map = attention_map / max(attention_map(:));
    
    % 应用注意力
    attended_img = img .* (1 + 2 * attention_map);
    attended_img = attended_img / max(attended_img(:));
else
    % 跳过注意力
    attended_img = img;
end

%% ========== 步骤2: V1层 - Gabor滤波器 ==========
orientations = [0, 45, 90, 135];
v1_responses = zeros(size(img, 1), size(img, 2), length(orientations));

for i = 1:length(orientations)
    gabor_kernel = create_gabor_filter(5, orientations(i), 0.5, 0.5, 0);
    v1_responses(:, :, i) = abs(conv2(attended_img, gabor_kernel, 'same'));
end

v1_output = mean(v1_responses, 3);
v1_output = v1_output / (max(v1_output(:)) + eps);

%% ========== 步骤3: 双流架构（可选） ==========
if config.dual_stream
    % Dorsal Stream (位置/"在哪里")
    [Gx, Gy] = gradient(v1_output);
    gradient_magnitude = sqrt(Gx.^2 + Gy.^2);
    gradient_magnitude = gradient_magnitude / (max(gradient_magnitude(:)) + eps);
    
    dorsal_features = 0.6 * gradient_magnitude + 0.4 * v1_output;
    
    % Ventral Stream (特征/"是什么")
    texture_features = stdfilt(v1_output, ones(3, 3));
    texture_features = texture_features / (max(texture_features(:)) + eps);
    
    enhanced_features = adapthisteq(v1_output, 'ClipLimit', 0.02);
    
    ventral_features = 0.4 * v1_output + 0.3 * texture_features + 0.3 * enhanced_features;
else
    % 单流模式：只使用V1输出
    dorsal_features = v1_output;
    ventral_features = v1_output;
end

%% ========== 步骤4: Transformer全局-局部交互（可选） ==========
if config.transformer
    % 计算全局统计
    dorsal_mean = mean(dorsal_features(:));
    dorsal_std = std(dorsal_features(:));
    ventral_mean = mean(ventral_features(:));
    ventral_std = std(ventral_features(:));
    
    global_context = [dorsal_mean, dorsal_std, ventral_mean, ventral_std];
    global_weight = mean(global_context);
    
    % 简化的Self-Attention效果
    dorsal_normalized = (dorsal_features - dorsal_mean) / (dorsal_std + eps);
    dorsal_enhanced = dorsal_features + 0.15 * global_weight * dorsal_normalized;
    
    ventral_normalized = (ventral_features - ventral_mean) / (ventral_std + eps);
    ventral_enhanced = ventral_features + 0.15 * global_weight * ventral_normalized;
    
    transformer_features = 0.5 * dorsal_enhanced + 0.5 * ventral_enhanced;
else
    % 跳过Transformer：直接融合
    transformer_features = 0.5 * dorsal_features + 0.5 * ventral_features;
end

%% ========== 步骤5: LSTM时序建模（可选） ==========
if config.lstm
    % LSTM前向传播
    combined_input = 0.6 * transformer_features + 0.4 * ventral_features;
    
    % 输入门
    input_gate = sigmoid(combined_input);
    
    % 遗忘门
    forget_gate = sigmoid(0.5 + 0.3 * combined_input);
    
    % 输出门
    output_gate = sigmoid(0.7 + 0.2 * combined_input);
    
    % 更新细胞状态
    cell_input = tanh(combined_input);
    lstm_state.c = forget_gate .* lstm_state.c + input_gate .* cell_input;
    
    % 计算输出
    lstm_h = output_gate .* tanh(lstm_state.c);
    lstm_state.h = lstm_h;
else
    % 跳过LSTM
    lstm_h = transformer_features;
end

%% ========== 步骤6: 特征融合 ==========
if config.lstm && config.transformer && config.dual_stream
    % 完整融合
    fused_features = 0.20 * dorsal_features + ...
                     0.30 * lstm_h + ...
                     0.30 * transformer_features + ...
                     0.20 * v1_output;
elseif config.transformer && config.dual_stream
    % 无LSTM
    fused_features = 0.30 * dorsal_features + ...
                     0.40 * transformer_features + ...
                     0.30 * v1_output;
elseif config.dual_stream
    % 只有双流
    fused_features = 0.50 * dorsal_features + ...
                     0.50 * ventral_features;
else
    % 最小配置
    fused_features = v1_output;
end

%% 归一化输出
features = (fused_features - min(fused_features(:))) / ...
           (max(fused_features(:)) - min(fused_features(:)) + eps);

%% ========== 步骤7: 更新注意力中心（如果使用注意力） ==========
if config.attention
    % 找到当前特征响应最强的位置
    [max_val, max_idx] = max(features(:));
    [max_y, max_x] = ind2sub(size(features), max_idx);
    
    % 平滑更新注意力中心
    attention_center = 0.7 * attention_center + 0.3 * [max_y, max_x];
end

end

%% ========== 辅助函数 ==========

function kernel = create_gabor_filter(size, orientation, wavelength, sigma, phase)
    % 创建Gabor滤波器
    half_size = floor(size / 2);
    [x, y] = meshgrid(-half_size:half_size, -half_size:half_size);
    
    % 旋转坐标
    theta = orientation * pi / 180;
    x_theta = x * cos(theta) + y * sin(theta);
    y_theta = -x * sin(theta) + y * cos(theta);
    
    % Gabor函数
    gaussian = exp(-(x_theta.^2 + y_theta.^2) / (2 * sigma^2));
    sinusoid = cos(2 * pi * x_theta / wavelength + phase);
    
    kernel = gaussian .* sinusoid;
    kernel = kernel / sum(abs(kernel(:)));
end

function y = sigmoid(x)
    % Sigmoid激活函数
    y = 1 ./ (1 + exp(-x));
end
