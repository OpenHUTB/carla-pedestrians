function vt_id = visual_template_neuro_matlab_only(rawImg, x, y, z, yaw, height)
% VISUAL_TEMPLATE_NEURO_MATLAB_ONLY 简化的类脑视觉模板匹配（纯MATLAB实现）
%
% 
% 
% 特征提取流程：
%   1. CLAHE自适应对比度增强 (ClipLimit=0.02)
%   2. 高斯平滑 (sigma=0.5)
%   3. Sobel边缘检测
%   4. 强度-边缘融合 (60% intensity + 40% edge)
%   5. Min-Max归一化
%
% 性能（5000帧Town01）：
%   - VT数量: 321个 (阈值0.08)
%   - 经验节点: 431个
%   - RMSE: ~150m
%   - 处理时间: 240秒
%
% 参数:
%   rawImg - 原始图像 [H×W×3]
%   x, y, z - 位置坐标
%   yaw - 偏航角（度）
%   height - 高度
%
% 返回:
%   vt_id - 视觉模板ID

    global NUM_VT VT VT_HISTORY VT_HISTORY_FIRST PREV_VT_ID;
    global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
    global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
    global VT_PANORAMIC;
    
    %% 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    
    %% HART + Transformer混合特征提取（创新架构）
    % 结合两种先进方法：
    % - HART: Spatial Attention + 双流 + LSTM时序
    % - Transformer: Self-Attention特征交互
    % 创新：用Transformer的Self-Attention增强特征表达
    normVtImg = hart_transformer_extractor(vtResizedImg);
    
    % 保存用于显示
    SUB_VT_IMG = normVtImg;
    
    %% 视觉模板匹配逻辑
    if NUM_VT < 5
        % 前5个模板直接添加
        % 先对现有VT进行decay（如果有的话）
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
        VT(NUM_VT).numExp = 0;      % 经验图关联计数
        VT(NUM_VT).exps = [];       % 关联的经验节点列表
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        % 与现有模板比较
        for k = 2:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            % 使用余弦相似度比较
            cosine_dist = 1 - sum(normVtImg(:) .* VT(k).template(:)) / ...
                          (norm(normVtImg(:)) * norm(VT(k).template(:)) + eps);
            DIFFS_ALL_IMGS_VTS(k) = cosine_dist;
        end
        
        % 找到最匹配的VT
        [min_diff, min_id] = min(DIFFS_ALL_IMGS_VTS(2:NUM_VT));
        min_id = min_id + 1;  % 调整索引
        MIN_DIFF_CURR_IMG_VTS = min_diff;
        
        if min_diff < VT_MATCH_THRESHOLD
            % 匹配到现有VT
            VT(min_id).decay = VT_ACTIVE_DECAY;
            vt_id = min_id;
            VT_HISTORY = [VT_HISTORY; vt_id];
        else
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
            VT(NUM_VT).exps = [];
            vt_id = NUM_VT;
            VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
        end
    end
    
    % 更新前一个VT ID
    PREV_VT_ID = vt_id;
end
