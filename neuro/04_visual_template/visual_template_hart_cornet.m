function [vt_id] = visual_template_hart_cornet(rawImg, x, y, z, yaw, height)
% VISUAL_TEMPLATE_HART_CORNET 基于HART+CORnet的视觉模板匹配
%
% 使用HART和CORnet思想的增强特征提取器
% 
% 主要特点:
%   1. CORnet层次化特征提取 (V1->V2->V4->IT)
%   2. HART注意力机制和时序建模
%   3. 更鲁棒的场景识别能力
%   4. 纯MATLAB实现，易于集成
%
% 参数:
%   rawImg - 原始图像
%   x, y, z - Grid Cell位置
%   yaw, height - Head Direction信息
%
% 返回:
%   vt_id - 视觉模板ID
%
% 参考文献:
%   [1] HART: Hierarchical Attentive Recurrent Tracking
%       https://github.com/akosiorek/hart
%   [2] CORnet: Brain-Like Object Recognition
%       https://github.com/dicarlolab/CORnet
%
% 作者: Neuro-SLAM Team
% 日期: 2024-12

    %% 定义全局变量
    global VT NUM_VT VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD PREV_VT_ID;
    global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
    global VT_IMG_X_SHIFT VT_IMG_Y_SHIFT VT_IMG_HALF_OFFSET;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
    global VT_IMG_RESIZE_X_RANGE VT_IMG_RESIZE_Y_RANGE;
    global VT_PANORAMIC;
    
    %% 图像预处理
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);
    
    %% HART+CORnet特征提取（HART为主，CORnet为辅）
    % HART: 动态场景跟踪（时序建模 + 注意力机制）
    % CORnet: 静态特征提取（层次化特征）
    normVtImg = hart_cornet_feature_extractor(vtResizedImg);
    
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
            [minOffsetY(k), minOffsetX(k), MIN_DIFF_CURR_IMG_VTS(k)] = ...
                vt_compare_cosine(normVtImg, VT(k).template, ...
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
            VT(NUM_VT).numExp = 0;      % 经验图关联计数
            VT(NUM_VT).exps = [];       % 关联的经验节点列表
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
    PREV_VT_ID = vt_id;
end


%% ========== 模板比较函数 ==========

function [minOffsetY, minOffsetX, minDiff] = vt_compare_cosine(...
    currentImg, templateImg, panoramic, halfOffset, yShift, xShift, ySize, xSize)
% 使用余弦相似度的模板比较

    minDiff = inf;
    minOffsetY = 0;
    minOffsetX = 0;
    
    for y = -yShift:yShift
        for x = -xShift:xShift
            if panoramic
                shiftedImg = circshift(templateImg, [y, x]);
                currentVec = currentImg(:);
                shiftedVec = shiftedImg(:);
                
                currentVec = currentVec / (norm(currentVec) + eps);
                shiftedVec = shiftedVec / (norm(shiftedVec) + eps);
                
                cosineSim = dot(currentVec, shiftedVec);
                diff = 1 - cosineSim;
            else
                yStart = max(1, 1 + y);
                yEnd = min(ySize, ySize + y);
                xStart = max(1, 1 + x);
                xEnd = min(xSize, xSize + x);
                
                if yStart > yEnd || xStart > xEnd
                    continue;
                end
                
                currentPatch = currentImg(yStart:yEnd, xStart:xEnd);
                templatePatch = templateImg(yStart:yEnd, xStart:xEnd);
                
                currentVec = currentPatch(:);
                templateVec = templatePatch(:);
                
                currentVec = currentVec / (norm(currentVec) + eps);
                templateVec = templateVec / (norm(templateVec) + eps);
                
                cosineSim = dot(currentVec, templateVec);
                diff = 1 - cosineSim;
            end
            
            if diff < minDiff
                minDiff = diff;
                minOffsetY = y;
                minOffsetX = x;
            end
        end
    end
end
