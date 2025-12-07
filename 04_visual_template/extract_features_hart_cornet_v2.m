function features = extract_features_hart_cornet_v2(img)
%EXTRACT_FEATURES_HART_CORNET_V2 简化版HART+CORnet特征提取
%   移除过度平滑和归一化，保留更多场景差异
%
% 输入:
%   img - 输入图像 [H x W] or [H x W x 3]
%
% 输出:
%   features - 归一化特征图 [H' x W']

    %% 预处理
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    img = double(img) / 255.0;
    [H, W] = size(img);
    
    %% ========== 简化的CORnet特征提取 ==========
    
    % === V1层: 多方向Gabor滤波 ===
    orientations = 4;
    v1_responses = zeros(H, W, orientations);
    
    for i = 1:orientations
        theta = (i-1) * pi / orientations;
        gabor = create_simple_gabor(2.0, theta);
        v1_responses(:,:,i) = abs(imfilter(img, gabor, 'replicate'));
    end
    
    % 取最大响应（复杂细胞）
    v1_features = max(v1_responses, [], 3);
    
    % === V2层: 轻度局部对比度归一化 ===
    % 不做过度归一化，只做轻度增强
    v2_features = adapthisteq(v1_features, 'ClipLimit', 0.02);
    
    % === V4层: 中等尺度边缘整合 ===
    % 使用Sobel检测边缘方向
    [Gx, Gy] = imgradientxy(v2_features, 'sobel');
    edge_magnitude = sqrt(Gx.^2 + Gy.^2);
    
    % 轻度平滑
    v4_features = imgaussfilt(edge_magnitude, 0.5);
    
    % === IT层: 保留原始信息 ===
    % 不做额外平滑，直接使用V4特征
    it_features = v4_features;
    
    %% ========== 简化的HART注意力 ==========
    
    % 只使用底层边缘显著性
    attention_map = v1_features;
    
    % 轻度注意力加权（保留大部分原始信息）
    attended_features = 0.8 * it_features + 0.2 * (it_features .* attention_map);
    
    %% ========== 最小化的归一化 ==========
    
    % 使用adapthisteq而不是标准化（保留局部对比度）
    features = adapthisteq(attended_features, 'ClipLimit', 0.03);
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
