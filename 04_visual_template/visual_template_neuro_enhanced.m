function [vt_id] = visual_template_neuro_enhanced(rawImg, x, y, z, yaw, height)
% VISUAL_TEMPLATE_NEURO_ENHANCED 增强的视觉模板匹配
%
% 基于HART (Hierarchical Attentive Recurrent Tracking)和CORnet的特征提取
% 
% 主要改进:
%   1. 类脑层次化特征提取 (V1->V2->V4->IT)
%   2. 注意力机制 (Saliency-based)
%   3. 更鲁棒的特征表示
%
% 使用方法:
%   方法1: 使用Python特征提取器 (推荐，更强大)
%   方法2: 使用MATLAB本地实现 (轻量级)
%
% 参数:
%   rawImg - 原始图像
%   x, y, z - Grid Cell位置
%   yaw, height - Head Direction信息
%
% 返回:
%   vt_id - 视觉模板ID

    %% 定义全局变量 (与原始visual_template.m保持一致)
    global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
    global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
    global VT_PANORAMIC;
    
    % 新增: 特征提取方法选择
    global NEURO_FEATURE_METHOD;  % 'python' 或 'matlab'
    if isempty(NEURO_FEATURE_METHOD) || ~ischar(NEURO_FEATURE_METHOD)
        NEURO_FEATURE_METHOD = 'matlab';  % 默认使用MATLAB实现
    end
    % 确保是小写
    NEURO_FEATURE_METHOD = lower(NEURO_FEATURE_METHOD);
    
    %% 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    
    %% 增强特征提取
    % 优先使用MATLAB实现，避免Python环境问题
    if strcmp(NEURO_FEATURE_METHOD, 'python')
        % 方法1: 调用Python特征提取器 (更强大)
        % 注意：需要配置Python环境
        try
            normVtImg = extract_features_python(vtResizedImg);
        catch ME
            warning('Python特征提取失败，回退到MATLAB实现: %s', ME.message);
            normVtImg = extract_features_matlab(vtResizedImg);
            % 自动切换到MATLAB模式，避免重复警告
            NEURO_FEATURE_METHOD = 'matlab';
        end
    else
        % 方法2: 使用MATLAB本地实现 (轻量级，推荐)
        normVtImg = extract_features_matlab(vtResizedImg);
    end
    
    % 保存用于显示
    SUB_VT_IMG = normVtImg;
    
    %% 视觉模板匹配逻辑 (与原始版本一致)
    if NUM_VT < 5
        % 前5个模板直接添加
        VT(NUM_VT).decay = VT(NUM_VT).decay - VT_GLOBAL_DECAY;
        if VT(NUM_VT).decay < 0
            VT(NUM_VT).decay = 0;
        end
        
        NUM_VT = NUM_VT + 1;
        VT(NUM_VT).id = NUM_VT;
        VT(NUM_VT).template = normVtImg;
        VT(NUM_VT).decay = VT_ACTIVE_DECAY;
        VT(NUM_VT).gc_x = x;
        VT(NUM_VT).gc_y = y;
        VT(NUM_VT).gc_z = z;
        VT(NUM_VT).hdc_yaw = yaw;
        VT(NUM_VT).hdc_height = height;
        VT(NUM_VT).first = 1;
        VT(NUM_VT).numExp = 0;
        VT(NUM_VT).exps = [];
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        % 与现有模板比较
        for k = 2:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            % 使用增强的比较函数
            [minOffsetY(k), minOffsetX(k), MIN_DIFF_CURR_IMG_VTS(k)] = ...
                vt_compare_neuro_enhanced(normVtImg, VT(k).template, ...
                VT_PANORAMIC, VT_IMG_HALF_OFFSET, VT_IMG_Y_SHIFT, ...
                VT_IMG_X_SHIFT, size(normVtImg, 1), size(normVtImg, 2));
        end
        
        [diff, diff_id] = min(MIN_DIFF_CURR_IMG_VTS);
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; diff];
        
        % 判断是否创建新模板
        if (diff > VT_MATCH_THRESHOLD)
            NUM_VT = NUM_VT + 1;
            VT(NUM_VT).id = NUM_VT;
            VT(NUM_VT).template = normVtImg;
            VT(NUM_VT).decay = VT_ACTIVE_DECAY;
            VT(NUM_VT).gc_x = x;
            VT(NUM_VT).gc_y = y;
            VT(NUM_VT).gc_z = z;
            VT(NUM_VT).hdc_yaw = yaw;
            VT(NUM_VT).hdc_height = height;
            VT(NUM_VT).first = 1;
            VT(NUM_VT).numExp = 0;
            VT(NUM_VT).exps = [];
            vt_id = NUM_VT;
            VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
        else
            vt_id = diff_id;
            VT(vt_id).decay = VT(vt_id).decay + VT_ACTIVE_DECAY;
            if PREV_VT_ID ~= vt_id
                VT(vt_id).first = 0;
            end
            VT_HISTORY_OLD = [VT_HISTORY_OLD; vt_id];
        end
    end
    
    VT_HISTORY = [VT_HISTORY; vt_id];
end


%% ========== 辅助函数：Python特征提取接口 ==========

function normImg = extract_features_python(img)
% 调用Python的NeuroVisualFeatureExtractor
%
% 需要MATLAB与Python环境配置正确
%
% 如果Python环境不可用，自动回退到MATLAB实现

    % 首先检查是否存在pyenv函数（MATLAB R2019b+才有）
    if ~exist('pyenv', 'builtin')
        warning('当前MATLAB版本不支持Python集成，使用MATLAB实现');
        normImg = extract_features_matlab(img);
        return;
    end
    
    try
        % 检查Python环境状态
        pe = pyenv;
        
        % 如果Python未加载，尝试初始化
        if strcmp(pe.Status, 'NotLoaded')
            % Python可用但未加载，直接使用
        elseif strcmp(pe.Status, 'Loaded')
            % Python已加载，正常使用
        else
            % 其他状态，回退到MATLAB
            error('Python环境状态异常');
        end
        
        % 添加Python模块路径
        scriptPath = fileparts(mfilename('fullpath'));
        if count(py.sys.path, scriptPath) == 0
            insert(py.sys.path, int32(0), scriptPath);
        end
        
        % 转换MATLAB图像到Python numpy数组
        pyImg = py.numpy.array(img);
        
        % 调用Python特征提取函数
        pyModule = py.importlib.import_module('neuro_visual_feature_extractor');
        pyFeatures = pyModule.neuro_patch_normalization(pyImg);
        
        % 转换回MATLAB
        normImg = double(pyFeatures);
        
    catch ME
        % Python调用失败，回退到MATLAB实现
        warning('Python特征提取失败，使用MATLAB实现: %s', ME.message);
        normImg = extract_features_matlab(img);
    end
end


%% ========== 辅助函数：MATLAB本地特征提取 ==========

function normImg = extract_features_matlab(img)
% MATLAB本地实现的类脑特征提取
% 简化版的HART+CORnet特征提取
%
% 实现:
%   1. V1层：Gabor滤波器 (多方向边缘检测)
%   2. V2层：复杂特征组合
%   3. 注意力机制：Saliency map
%   4. 特征归一化

    % 转换为灰度图
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    % 归一化到0-1
    img = double(img) / 255.0;
    
    %% V1层：Gabor特征提取 (模拟简单细胞)
    v1Features = extract_gabor_features(img);
    
    %% 注意力机制
    attentionMap = compute_attention_map(img);
    
    % 应用注意力权重
    for i = 1:size(v1Features, 3)
        v1Features(:, :, i) = v1Features(:, :, i) .* attentionMap;
    end
    
    %% V2层：特征池化和组合
    v2Features = pool_and_combine_features(v1Features);
    
    %% 最终归一化
    normImg = normalize_features(v2Features);
end


function gaborFeatures = extract_gabor_features(img)
% 提取Gabor特征 (V1简单细胞)
% 
% 参数:
%   img - 输入图像 [H, W]
% 返回:
%   gaborFeatures - Gabor特征图 [H, W, N_orientations]

    % Gabor参数
    nOrientations = 8;  % 8个方向
    wavelength = 4;     % 波长
    sigmaX = 3;         % X方向标准差
    sigmaY = 3;         % Y方向标准差
    
    [h, w] = size(img);
    gaborFeatures = zeros(h, w, nOrientations);
    
    % 对每个方向创建Gabor滤波器
    for i = 1:nOrientations
        theta = (i-1) * pi / nOrientations;
        
        % 创建Gabor滤波器
        gaborKernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, 0, 0.5);
        
        % 卷积
        response = imfilter(img, gaborKernel, 'same', 'replicate');
        
        % ReLU激活
        gaborFeatures(:, :, i) = max(response, 0);
    end
end


function kernel = create_gabor_kernel(wavelength, theta, sigmaX, sigmaY, offset, aspect)
% 创建Gabor核
% 
% 参数:
%   wavelength - 波长
%   theta - 方向角度
%   sigmaX, sigmaY - 标准差
%   offset - 相位偏移
%   aspect - 长宽比
% 返回:
%   kernel - Gabor核

    % 核大小
    kernelSize = round(3 * max(sigmaX, sigmaY));
    if mod(kernelSize, 2) == 0
        kernelSize = kernelSize + 1;
    end
    
    % 生成网格
    [x, y] = meshgrid(-(kernelSize-1)/2:(kernelSize-1)/2, ...
                      -(kernelSize-1)/2:(kernelSize-1)/2);
    
    % 旋转坐标
    xTheta = x * cos(theta) + y * sin(theta);
    yTheta = -x * sin(theta) + y * cos(theta);
    
    % Gabor函数
    gaussian = exp(-(xTheta.^2 / (2*sigmaX^2) + yTheta.^2 / (2*sigmaY^2)));
    sinusoid = cos(2 * pi * xTheta / wavelength + offset);
    
    kernel = gaussian .* sinusoid;
    
    % 归一化
    kernel = kernel / sum(abs(kernel(:)));
end


function attentionMap = compute_attention_map(img)
% 计算注意力图 (Saliency detection)
%
% 参数:
%   img - 输入图像
% 返回:
%   attentionMap - 注意力权重图

    % 方法1: 基于强度对比
    blurred = imgaussfilt(img, 3);
    intensityContrast = abs(img - blurred);
    
    % 方法2: 边缘检测
    edges = edge(img, 'canny');
    edges = double(edges);
    edges = imdilate(edges, strel('disk', 2));
    
    % 组合
    attentionMap = 0.7 * intensityContrast + 0.3 * edges;
    
    % 归一化
    attentionMap = attentionMap / (max(attentionMap(:)) + eps);
    
    % 平滑
    attentionMap = imgaussfilt(attentionMap, 2);
end


function pooledFeatures = pool_and_combine_features(features)
% V2层：特征池化和组合
%
% 参数:
%   features - 输入特征 [H, W, C]
% 返回:
%   pooledFeatures - 池化后的特征 [H', W']

    [h, w, c] = size(features);
    poolSize = 2;
    
    % 对所有通道做max pooling
    maxPooled = zeros(floor(h/poolSize), floor(w/poolSize), c);
    for ch = 1:c
        maxPooled(:, :, ch) = imresize(features(:, :, ch), ...
                                       1/poolSize, 'Method', 'bilinear');
    end
    
    % 跨通道组合（求平均）
    pooledFeatures = mean(maxPooled, 3);
end


function normFeatures = normalize_features(features)
% 特征归一化
%
% 参数:
%   features - 输入特征
% 返回:
%   normFeatures - 归一化特征

    % Z-score归一化
    meanVal = mean(features(:));
    stdVal = std(features(:));
    
    if stdVal < eps
        stdVal = 1;
    end
    
    normFeatures = (features - meanVal) / stdVal;
    
    % 限制范围
    normFeatures = max(min(normFeatures, 3), -3);
    
    % 缩放到0-1
    normFeatures = (normFeatures + 3) / 6;
end


%% ========== 辅助函数：增强的模板比较 ==========

function [minOffsetY, minOffsetX, minDiff] = vt_compare_neuro_enhanced(...
    currentImg, templateImg, panoramic, halfOffset, yShift, xShift, ySize, xSize)
% 增强的视觉模板比较函数
%
% 使用余弦相似度代替简单的像素差异

    minDiff = inf;
    minOffsetY = 0;
    minOffsetX = 0;
    
    % 遍历可能的偏移
    for y = -yShift:yShift
        for x = -xShift:xShift
            % 计算偏移后的比较
            if panoramic
                % 全景模式：支持循环偏移
                shiftedImg = circshift(templateImg, [y, x]);
            else
                % 普通模式：裁剪偏移区域
                yStart = max(1, 1 + y);
                yEnd = min(ySize, ySize + y);
                xStart = max(1, 1 + x);
                xEnd = min(xSize, xSize + x);
                
                if yStart > yEnd || xStart > xEnd
                    continue;
                end
                
                currentPatch = currentImg(yStart:yEnd, xStart:xEnd);
                templatePatch = templateImg(yStart:yEnd, xStart:xEnd);
                
                % 计算余弦相似度
                currentVec = currentPatch(:);
                templateVec = templatePatch(:);
                
                % 归一化
                currentVec = currentVec / (norm(currentVec) + eps);
                templateVec = templateVec / (norm(templateVec) + eps);
                
                % 余弦距离 (1 - 余弦相似度)
                cosineSim = dot(currentVec, templateVec);
                diff = 1 - cosineSim;
                
                if diff < minDiff
                    minDiff = diff;
                    minOffsetY = y;
                    minOffsetX = x;
                end
                
                continue;
            end
            
            % 全景模式的比较
            currentVec = currentImg(:);
            shiftedVec = shiftedImg(:);
            
            currentVec = currentVec / (norm(currentVec) + eps);
            shiftedVec = shiftedVec / (norm(shiftedVec) + eps);
            
            cosineSim = dot(currentVec, shiftedVec);
            diff = 1 - cosineSim;
            
            if diff < minDiff
                minDiff = diff;
                minOffsetY = y;
                minOffsetX = x;
            end
        end
    end
end
