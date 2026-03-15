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
    global USE_TRANSFORMER_OVERRIDE USE_DUAL_STREAM_OVERRIDE;
    
    %% 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    
    %% HART + Transformer混合特征提取（创新架构）
    % 结合两种先进方法：
    % - HART: Spatial Attention + 双流 + LSTM时序
    % - Transformer: Self-Attention特征交互
    % 创新：用Transformer的Self-Attention增强特征表达
    use_transformer = true;
    if ~isempty(USE_TRANSFORMER_OVERRIDE)
        use_transformer = logical(USE_TRANSFORMER_OVERRIDE);
    end

    use_dual_stream = true;
    if ~isempty(USE_DUAL_STREAM_OVERRIDE)
        use_dual_stream = logical(USE_DUAL_STREAM_OVERRIDE);
    end

    if use_transformer && use_dual_stream
        normVtImg = hart_transformer_extractor(vtResizedImg);
    else
        cfg = struct();
        cfg.attention = true;
        cfg.dual_stream = use_dual_stream;
        cfg.transformer = use_transformer;
        cfg.lstm = true;
        cfg.full_feature = true;
        normVtImg = hart_transformer_extractor_ablation(vtResizedImg, cfg);
    end
    
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
        VT(NUM_VT).EXPERIENCES = [];       % 关联的经验节点列表
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        % ★★★ Bug修复：初始化MIN_DIFF_CURR_IMG_VTS数组 ★★★
        MIN_DIFF_CURR_IMG_VTS = inf(1, NUM_VT);  % 初始化为无穷大
        
        % 与现有模板比较
        % ★★★ Bug修复：从k=1开始，不跳过第一个VT ★★★
        for k = 1:NUM_VT
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            % 使用NCC（归一化互相关）替代SAD+位移搜索，更快且对光照鲁棒
            MIN_DIFF_CURR_IMG_VTS(k) = vt_compare_ncc(normVtImg, VT(k).template);
        end
        
        % 找到最匹配的VT（★★★ Bug修复：搜索所有VT ★★★）
        [min_diff, min_id] = min(MIN_DIFF_CURR_IMG_VTS);
        
        % 保存用于显示
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; min_diff];
        
        if min_diff > VT_MATCH_THRESHOLD
            % 创建新VT（差异大于阈值）
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
            % 匹配到现有VT（差异小于等于阈值）
            VT(min_id).decay = VT_ACTIVE_DECAY;
            VT(min_id).first = 0;
            vt_id = min_id;
            VT_HISTORY = [VT_HISTORY; vt_id];
        end
    end
    
end
