function vt_id = visual_template_baseline(rawImg, x, y, z, yaw, height)
% VISUAL_TEMPLATE_BASELINE 原始NeuroSLAM视觉模板匹配（Baseline方法）
%
% 这是原始NeuroSLAM的Patch Normalization方法，用于对比实验的Baseline
% 
% 特征提取流程：
%   1. 图像裁剪和缩放
%   2. Patch Normalization（局部均值/标准差归一化）
%   3. SAD距离匹配（通过NCC实现）
%
% 与Ours (HART+Transformer)的区别：
%   - Baseline: Patch Normalization + SAD
%   - Ours: HART+Transformer双流 + LSTM门控 + Self-Attention
%
% 参数:
%   rawImg - 原始图像 [H×W×3] 或 [H×W]
%   x, y, z - 位置坐标
%   yaw - 偏航角（度）
%   height - 高度
%
% 返回:
%   vt_id - 视觉模板ID

    global NUM_VT VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
    global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
    global PATCH_SIZE_Y_K PATCH_SIZE_X_K;
    global VT_PANORAMIC;
    
    %% 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    vtResizedImg = double(vtResizedImg);
    
    % 获取图像尺寸
    ySizeVtImg = VT_IMG_RESIZE_Y_RANGE;
    xSizeVtImg = VT_IMG_RESIZE_X_RANGE;
    ySizeNormImg = ySizeVtImg;
    
    %% ========== 原始Patch Normalization（Baseline核心方法） ==========
    % 这是原始NeuroSLAM的特征提取方法
    % 通过局部patch的均值和标准差进行归一化，补偿光照变化
    
    % 扩展图像边界用于patch处理
    extVtImg = zeros(ySizeVtImg + PATCH_SIZE_Y_K - 1, xSizeVtImg + PATCH_SIZE_X_K - 1);
    extVtImg(fix((PATCH_SIZE_Y_K + 1)/2) : fix((PATCH_SIZE_Y_K + 1)/2) + ySizeNormImg - 1, ...
             fix((PATCH_SIZE_X_K + 1)/2) : fix((PATCH_SIZE_X_K + 1)/2) + xSizeVtImg - 1) = vtResizedImg;
    
    % Patch Normalization
    normVtImg = zeros(ySizeNormImg, xSizeVtImg);
    for v = 1:ySizeNormImg
        for u = 1:xSizeVtImg
            % 获取patch
            patchImg = extVtImg(v : v + PATCH_SIZE_Y_K - 1, u : u + PATCH_SIZE_X_K - 1);
            
            % 计算patch均值（替代mean2，兼容性更好）
            meanPatchImg = mean(patchImg(:));
            
            % 计算patch标准差（替代std2，兼容性更好）
            stdPatchImg = std(patchImg(:));
            
            % 归一化
            if stdPatchImg > 1e-6
                normVtImg(v, u) = (vtResizedImg(v, u) - meanPatchImg) / stdPatchImg / 255;
            else
                normVtImg(v, u) = 0;
            end
        end
    end
    
    % 保存用于显示
    SUB_VT_IMG = normVtImg;
    
    %% 视觉模板匹配逻辑（与原始NeuroSLAM相同）
    if NUM_VT < 5
        % 前5个模板直接添加
        if NUM_VT > 0
            for k = 1:NUM_VT
                VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
                if VT(k).decay < 0
                    VT(k).decay = 0;
                end
            end
        end
        
        % 创建新的VT
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
        VT(NUM_VT).EXPERIENCES = [];
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        % ★★★ Bug修复：初始化MIN_DIFF_CURR_IMG_VTS数组 ★★★
        MIN_DIFF_CURR_IMG_VTS = inf(1, NUM_VT);  % 初始化为无穷大
        
        % 与现有模板比较（★★★ Bug修复：从k=1开始，不跳过第一个VT ★★★）
        for k = 1:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            % 使用NCC进行匹配（与HART版本相同的匹配方式，确保公平）
            MIN_DIFF_CURR_IMG_VTS(k) = vt_compare_ncc(normVtImg, VT(k).template);
        end
        
        % 找到最匹配的VT（★★★ Bug修复：搜索所有VT，不跳过第一个 ★★★）
        [min_diff, min_id] = min(MIN_DIFF_CURR_IMG_VTS);
        
        % 保存用于显示
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; min_diff];
        
        if min_diff > VT_MATCH_THRESHOLD
            % 创建新VT
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
            VT(NUM_VT).EXPERIENCES = [];
            vt_id = NUM_VT;
            VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
        else
            % 匹配到现有VT
            VT(min_id).decay = VT_ACTIVE_DECAY;
            VT(min_id).first = 0;
            vt_id = min_id;
            VT_HISTORY = [VT_HISTORY; vt_id];
        end
    end
end
