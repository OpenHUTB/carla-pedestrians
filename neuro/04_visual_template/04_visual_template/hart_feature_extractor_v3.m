function normImg = hart_feature_extractor_v3(img)
% HART_FEATURE_EXTRACTOR_V3 HART模型特征提取器（根据论文架构手搓）
%
% 参考架构（来自Hierarchical Attentive Recurrent Tracking论文）:
%   x_t → Spatial Attention → g_t → V1
%         ├─ Dorsal Stream (位置信息) → s_t
%         └─ Ventral Stream (特征信息) → v_t → LSTM → o_t
%
% 实现细节：
%   1. Spatial Attention: 高斯注意力机制提取glimpse
%   2. V1层: Gabor滤波器组（模拟简单细胞）
%   3. Dorsal Stream: 位置和运动信息处理
%   4. Ventral Stream: 特征和物体信息处理
%   5. LSTM: 时序递归建模
%
% 参数:
%   img - 输入图像 [H×W×C] 或 [H×W]
%
% 返回:
%   normImg - 归一化特征图 [H×W]

    % 持久化变量（模拟LSTM状态）
    persistent lstm_h lstm_c frame_count attention_center;
    
    % 初始化
    if isempty(frame_count)
        frame_count = 0;
        lstm_h = [];
        lstm_c = [];
        attention_center = [0.5, 0.5];  % 图像中心
    end
    frame_count = frame_count + 1;
    
    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% ========== 步骤1: Spatial Attention（空间注意力） ==========
    % 根据HART论文，使用高斯注意力窗口提取glimpse
    
    % 注意力参数 [uy, sy, dy, ux, sx, dx]
    % u - 中心位置, s - 标准差（尺度）, d - 步长
    uy = attention_center(1) * H;
    ux = attention_center(2) * W;
    sy = 0.25 * H;  % 注意力窗口大小
    sx = 0.25 * W;
    
    % 生成高斯注意力图
    [X, Y] = meshgrid(1:W, 1:H);
    attention_map = exp(-((X-ux).^2 / (2*sx^2) + (Y-uy).^2 / (2*sy^2)));
    attention_map = attention_map / (sum(attention_map(:)) + eps);
    
    % 应用注意力（加权）
    attended_img = img .* (1.0 + 2.0 * attention_map);  % 增强注意力区域
    
    %% ========== 步骤2: V1层（简单细胞 - Gabor滤波器组） ==========
    % V1层使用多方向Gabor滤波器模拟简单细胞
    
    orientations = [0, 45, 90, 135];  % 4个主要方向
    wavelength = 4;  % Gabor波长
    gabor_responses = zeros(H, W, length(orientations));
    
    for i = 1:length(orientations)
        theta = orientations(i) * pi / 180;
        
        % 创建Gabor滤波器
        sigma = wavelength / pi;
        kernel_size = 2 * ceil(3 * sigma) + 1;
        gabor_kernel = create_gabor_filter(kernel_size, theta, wavelength, sigma);
        
        % 应用Gabor滤波
        gabor_responses(:,:,i) = abs(imfilter(attended_img, gabor_kernel, 'same', 'replicate'));
    end
    
    % V1层输出（所有方向的综合）
    v1_output = mean(gabor_responses, 3);
    
    %% ========== 步骤3: 双流架构（Dorsal + Ventral） ==========
    
    % === Dorsal Stream（背侧流）: 处理"where"信息 ===
    % 关注位置、运动、空间关系
    
    % 位置特征：图像梯度（运动和边界）
    [Gx, Gy] = imgradientxy(attended_img, 'sobel');
    gradient_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 方向特征
    gradient_direction = atan2(Gy, Gx);
    
    % Dorsal特征：位置 + 梯度
    dorsal_features = 0.6 * gradient_magnitude + 0.4 * v1_output;
    
    % === Ventral Stream（腹侧流）: 处理"what"信息 ===
    % 关注物体、纹理、特征
    
    % 纹理特征：局部方差
    % ★★★ 修复: 使用简单方法替代stdfilt，避免兼容性问题 ★★★
    win = ones(5,5);
    local_mean_v3 = imfilter(attended_img, win, 'same', 'replicate') ./ numel(win);
    local_mean2_v3 = imfilter(attended_img.^2, win, 'same', 'replicate') ./ numel(win);
    local_var_v3 = max(local_mean2_v3 - local_mean_v3.^2, 0);
    local_std = sqrt(local_var_v3);
    
    % CLAHE增强（增强对比度）
    % ★★★ 修复: 使用简单对比度增强替代adapthisteq，避免coder.isColumnMajor错误 ★★★
    ventral_enhanced = simple_contrast_enhance_v3(attended_img);
    
    % Ventral特征：V1 + 纹理 + 增强
    ventral_features = 0.4 * v1_output + 0.3 * local_std + 0.3 * ventral_enhanced;
    
    %% ========== 步骤4: LSTM时序递归建模 ==========
    % 根据HART架构，Ventral特征输入LSTM
    
    % 初始化LSTM状态
    if isempty(lstm_h) || any(size(lstm_h) ~= size(ventral_features))
        lstm_h = ventral_features;  % 隐藏状态
        lstm_c = ventral_features;  % 细胞状态
    end
    
    % LSTM门控（简化版）
    % 参考HART论文的递归结构
    input_gate = 0.5;      % 输入门（控制新信息）
    forget_gate = 0.5;     % 遗忘门（控制历史信息）
    output_gate = 0.9;     % 输出门（控制输出）
    
    % 输入调制
    ventral_modulated = tanh(ventral_features);
    
    % 更新细胞状态
    lstm_c = forget_gate * lstm_c + input_gate * ventral_modulated;
    
    % 更新隐藏状态
    lstm_h = output_gate * tanh(lstm_c);
    
    %% ========== 步骤5: 融合双流和时序特征 ==========
    
    % 融合策略：
    % - Dorsal提供空间信息
    % - LSTM输出提供时序一致性
    % - 注意力图提供聚焦
    
    % 权重：30% Dorsal + 50% LSTM + 20% V1
    fused_features = 0.30 * dorsal_features + ...
                     0.50 * lstm_h + ...
                     0.20 * v1_output;
    
    % 注意力调制（再次应用注意力）
    final_features = fused_features .* (1.0 + 0.5 * attention_map);
    
    %% ========== 步骤6: 更新注意力中心 ==========
    % 根据当前特征更新下一帧的注意力中心
    % 这模拟了HART的预测性注意力
    
    % 找到最显著的区域
    [max_val, max_idx] = max(final_features(:));
    [max_y, max_x] = ind2sub(size(final_features), max_idx);
    
    % 平滑更新注意力中心（70%历史 + 30%新位置）
    attention_center(1) = 0.7 * attention_center(1) + 0.3 * (max_y / H);
    attention_center(2) = 0.7 * attention_center(2) + 0.3 * (max_x / W);
    
    % 限制在图像范围内
    attention_center = max(0.2, min(0.8, attention_center));
    
    %% ========== 步骤7: 归一化输出 ==========
    normImg = (final_features - min(final_features(:))) / ...
              (max(final_features(:)) - min(final_features(:)) + eps);
end


%% ========== 辅助函数 ==========

function kernel = create_gabor_filter(size, theta, wavelength, sigma)
    % 创建Gabor滤波器
    % size - 滤波器大小
    % theta - 方向（弧度）
    % wavelength - 波长
    % sigma - 高斯标准差
    
    if mod(size, 2) == 0
        size = size + 1;
    end
    
    half_size = (size - 1) / 2;
    [x, y] = meshgrid(-half_size:half_size, -half_size:half_size);
    
    % 旋转坐标系
    x_theta = x * cos(theta) + y * sin(theta);
    y_theta = -x * sin(theta) + y * cos(theta);
    
    % Gabor函数
    gaussian = exp(-(x_theta.^2 + y_theta.^2) / (2 * sigma^2));
    sinusoid = cos(2 * pi * x_theta / wavelength);
    
    kernel = gaussian .* sinusoid;
    kernel = kernel / sum(abs(kernel(:)));  % 归一化
end


function enhanced = simple_contrast_enhance_v3(img)
% SIMPLE_CONTRAST_ENHANCE_V3 简单对比度增强（替代adapthisteq）
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
