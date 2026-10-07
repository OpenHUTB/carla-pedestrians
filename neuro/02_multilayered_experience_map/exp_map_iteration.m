function exp_map_iteration(vt_id, transV, yawRotV, heightV, xGc, yGc, zGc, curYawHdc, curHeight)
%     NeuroSLAM System Copyright (C) 2018-2019 
%     NeuroSLAM: A Brain inspired SLAM System for 3D Environments
%
%     Fangwen Yu (www.yufangwen.com), Jianga Shang, Youjian Hu, Michael Milford(www.michaelmilford.com) 
%
%     The NeuroSLAM V1.0 (MATLAB) was developed based on the OpenRatSLAM (David et al. 2013). 
%     The RatSLAM V0.3 (MATLAB) developed by David Ball, Michael Milford and Gordon Wyeth in 2008.
% 
%     Reference:
%     Ball, David, Scott Heath, Janet Wiles, Gordon Wyeth, Peter Corke, and Michael Milford.
%     "OpenRatSLAM: an open source brain-based SLAM system." Autonomous Robots 34, no. 3 (2013): 149-176.
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

    %% define some variables 
    
    %%% some variables of em
    
    % define a variable of experiences
    global EXPERIENCES; 
     
    % current experience id
    global CUR_EXP_ID;
   
    % the number of experiences
    global NUM_EXPS;
    
    % the min change of em between previous e with current e
    global MIN_DELTA_EM;

    % the experience delta threshold 
    global DELTA_EXP_GC_HDC_THRESHOLD;
    
    
    
    %%% some variables of vt
    
    % define a variable of visual template
    global VT; 
        
    % the previous visual template id
    global PREV_VT_ID; 
 
    
    %%% some variables of 3D GC
    
    % The x, y, z dimension of 3D Grid Cells Model (3D CAN)
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;

    
    %%% some variables of 3D HDC
    
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;
    
    
    %%% computing delta x,y,z, yaw, height
    
    % integrate the delta x, y, z, yaw, height
    % Rad radian  accumulative
    global ACCUM_DELTA_X; 
    global ACCUM_DELTA_Y; 
    global ACCUM_DELTA_Z;  
    global ACCUM_DELTA_YAW;   % accum_delta_facing
%     global ACCUM_DELTA_HEIGHT; 

    global EXP_LOOP_CLOSURE_LINKS;
    global EXP_CONSTRAINT_ERR_MAX;
    global EXP_MATCH_RATIO_THRESHOLD;
    global EXP_MAX_LOOP_CE;

    if isempty(EXP_LOOP_CLOSURE_LINKS)
        EXP_LOOP_CLOSURE_LINKS = 0;
    end
    if isempty(EXP_CONSTRAINT_ERR_MAX)
        EXP_CONSTRAINT_ERR_MAX = 0;
    end
    if isempty(EXP_MATCH_RATIO_THRESHOLD)
        EXP_MATCH_RATIO_THRESHOLD = 0.7;
    end
    if isempty(EXP_MAX_LOOP_CE)
        EXP_MAX_LOOP_CE = 20;
    end

    % [B-fix] 闭环修正传播(单向累计修正, 替代被禁用的每帧松弛循环):
    % 真闭环被接受(CE门+朝向门双验证)时, 匹配节点位移β·Δ并累计进NLM_LOOP_CORR,
    % 后续新节点经create_new_exp继承(EKF-odo钉扎=锚点+NLM_LOOP_CORR)→修正粘住整条后缀。
    % 根因: 原EKF-odo模式新节点钉扎原始EKF坐标, 闭环修正一离开匹配节点即被抹掉
    % (CE扫描: 10HD CE=200仅-2.1m; 闭环上限模拟10HD可达3.48m, 差距即修正不传播)。
    global NLM_LOOP_BETA NLM_LOOP_CE_MIN NLM_LOOP_STEP_MAX NLM_LOOP_YAW_GATE NLM_LOOP_CORR;
    global NLM_LOOP_FIX_ENABLE;
    if isempty(NLM_LOOP_FIX_ENABLE), NLM_LOOP_FIX_ENABLE = false; end % 持续性修正开关: 默认关=纯重锚定(基线行为), 开=朝向门+β·r后缀传播
    if isempty(NLM_LOOP_BETA), NLM_LOOP_BETA = 1.0; end               % 修正力度β
    if isempty(NLM_LOOP_CE_MIN), NLM_LOOP_CE_MIN = 3; end             % CE<3m=近邻重访: 纯跳转不修正
    if isempty(NLM_LOOP_STEP_MAX), NLM_LOOP_STEP_MAX = 25; end        % 单步修正钳位(m)
    if isempty(NLM_LOOP_YAW_GATE), NLM_LOOP_YAW_GATE = 60 * pi / 180; end % 朝向门: 同向或反向容差
    if isempty(NLM_LOOP_CORR), NLM_LOOP_CORR = [0, 0, 0]; end
    % 注: 跨run重置由驱动脚本在调用core前显式 NLM_LOOP_CORR=[0,0,0] 完成
    % (此处不能依赖EXP_HISTORY, 其global声明在函数后部, 提前引用不解析)

    %% [DIAG] 闭环诊断计数器（论文重跑用，最终会移除）
    global DIAG_VT_REVISIT DIAG_BELOW_THR DIAG_MAX_DELTA DIAG_MULTI_CAND DIAG_MULTI_REJECT ...
           DIAG_SINGLE_BELOW DIAG_SINGLE_MATCH DIAG_CE_REJECT DIAG_CE_MAX ...
           DIAG_LOOP_ACCEPT DIAG_YAW_REJECT;
    if isempty(DIAG_VT_REVISIT), DIAG_VT_REVISIT = 0; end
    if isempty(DIAG_BELOW_THR), DIAG_BELOW_THR = 0; end
    if isempty(DIAG_MAX_DELTA), DIAG_MAX_DELTA = 0; end
    if isempty(DIAG_MULTI_CAND), DIAG_MULTI_CAND = 0; end
    if isempty(DIAG_MULTI_REJECT), DIAG_MULTI_REJECT = 0; end
    if isempty(DIAG_SINGLE_BELOW), DIAG_SINGLE_BELOW = 0; end
    if isempty(DIAG_SINGLE_MATCH), DIAG_SINGLE_MATCH = 0; end
    if isempty(DIAG_CE_REJECT), DIAG_CE_REJECT = 0; end
    if isempty(DIAG_CE_MAX), DIAG_CE_MAX = 0; end
    if isempty(DIAG_LOOP_ACCEPT), DIAG_LOOP_ACCEPT = 0; end
    if isempty(DIAG_YAW_REJECT), DIAG_YAW_REJECT = 0; end
    global DIAG_FUNNEL_LOG;
    if isempty(DIAG_FUNNEL_LOG), DIAG_FUNNEL_LOG = zeros(0, 10); end
    
    ACCUM_DELTA_YAW = clip_radian_180(ACCUM_DELTA_YAW + yawRotV);
%     ACCUM_DELTA_HEIGHT = mod(ACCUM_DELTA_HEIGHT + heightV, YAW_HEIGHT_HDC_H_DIM);
%     
    ACCUM_DELTA_X = ACCUM_DELTA_X + transV * cos(ACCUM_DELTA_YAW);
    ACCUM_DELTA_Y = ACCUM_DELTA_Y + transV * sin(ACCUM_DELTA_YAW);
    ACCUM_DELTA_Z = ACCUM_DELTA_Z + heightV;
    
    % trajectory of delta of em
    global DELTA_EM;
    
    minDeltaX = get_min_delta(EXPERIENCES(CUR_EXP_ID).x_gc, xGc, GC_X_DIM);
    minDeltaY = get_min_delta(EXPERIENCES(CUR_EXP_ID).y_gc, yGc, GC_Y_DIM);
    minDeltaZ = get_min_delta(EXPERIENCES(CUR_EXP_ID).z_gc, zGc, GC_Z_DIM);
    
    minDeltaYaw = get_min_delta(EXPERIENCES(CUR_EXP_ID).yaw_hdc, curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
    minDeltaHeight = get_min_delta(EXPERIENCES(CUR_EXP_ID).height_hdc, curHeight, YAW_HEIGHT_HDC_H_DIM);
    
    minDeltaYawReversed = get_min_delta(EXPERIENCES(CUR_EXP_ID).yaw_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
    minDeltaYaw = min(minDeltaYaw, minDeltaYawReversed);
    
%     minDeltaHeightReversed = get_min_delta(EXPERIENCES(CUR_EXP_ID).height_hdc, (YAW_HEIGHT_HDC_H_DIM /2) - curHeight, YAW_HEIGHT_HDC_H_DIM);
%     minDeltaHeight = min(minDeltaHeight, minDeltaHeightReversed);
    
    delta_em = sqrt((minDeltaX).^2 + (minDeltaY).^2 + (minDeltaZ).^2 + (minDeltaYaw).^2 + (minDeltaHeight).^2);
    DELTA_EM = [DELTA_EM delta_em];
    
    % 确保delta_em是标量（处理可能的数组情况）
    if numel(delta_em) > 1
        delta_em = max(delta_em);  % 取最大值作为判断依据
    end
    
    % If the visual template is new, create a new experience.
    % Otherwise, prefer matching an existing experience for this VT before creating a new one
    % (important when delta_em is large due to drift but VT repeats).
    if VT(vt_id).numExp > 0
        DIAG_VT_REVISIT = DIAG_VT_REVISIT + 1;  % [DIAG]
    end
    if VT(vt_id).numExp == 0
        log_funnel(0, inf, inf, 0, 0, 0, vt_id);  % [DIAG] 漏斗: VT新建节点
        NUM_EXPS = NUM_EXPS + 1;
        create_new_exp(CUR_EXP_ID, NUM_EXPS, vt_id, xGc, yGc, zGc, curYawHdc, curHeight);

        PREV_EXP_ID = CUR_EXP_ID;
        CUR_EXP_ID = NUM_EXPS;

        ACCUM_DELTA_X = 0;
        ACCUM_DELTA_Y = 0;
        ACCUM_DELTA_Z = 0;
        
        ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
%         ACCUM_DELTA_HEIGHT = EXPERIENCES(CUR_EXP_ID).height_hdc;

        % if the visual template has changed (but isn't new) search for the matching experience
    elseif vt_id ~= PREV_VT_ID

        % find the experience associated with the current visual template and that is under the
        % threshold distance to the centre of grid cell and head direction cell activity
        % if multiple experiences are under the threshold then don't match (to reduce hash collisions)
        matched_exp_id = 0;
        matched_exp_count = 0;

        delta_em = zeros(1, VT(vt_id).numExp);

        for search_id = 1:VT(vt_id).numExp
             
            minDeltaYaw = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).yaw_hdc, curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
%             minDeltaYawReversed = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).yaw_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
%             minDeltaYaw = min(minDeltaYaw, minDeltaYawReversed);
    
            minDeltaHeight = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).height_hdc, curHeight, YAW_HEIGHT_HDC_H_DIM);
%             minDeltaHeightReversed = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).height_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curHeight, YAW_HEIGHT_HDC_H_DIM);
%             minDeltaHeight = min(minDeltaHeight, minDeltaHeightReversed);
            
            delta_em(search_id) = sqrt(get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).x_gc, xGc, GC_X_DIM)^2 ...
            + get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).y_gc, yGc, GC_Y_DIM)^2 ...
            + get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).z_gc, zGc, GC_Z_DIM)^2 ...
            + minDeltaYaw^2 + minDeltaHeight^2) ;
            
            
            if delta_em(search_id) < DELTA_EXP_GC_HDC_THRESHOLD
               matched_exp_count = matched_exp_count + 1; 
            end
        end

        if matched_exp_count > 1
            % this means we aren't sure about which experience is a match due to hash table collision
            % instead of a false posivitive which may create blunder links in
            % the experience map keep the previous experience
            [vals, ids] = sort(delta_em);
            if numel(vals) >= 2
                ratio = vals(1) / (vals(2) + eps);
            else
                ratio = 0;
            end

            if vals(1) < DELTA_EXP_GC_HDC_THRESHOLD
                DIAG_MULTI_CAND = DIAG_MULTI_CAND + 1;  % [DIAG]
            end
            if vals(1) < DELTA_EXP_GC_HDC_THRESHOLD && ratio < EXP_MATCH_RATIO_THRESHOLD
                matched_exp_id = VT(vt_id).EXPERIENCES(ids(1)).id;
            else
                DIAG_MULTI_REJECT = DIAG_MULTI_REJECT + 1;  % [DIAG]
                log_funnel(2, vals(1), ratio, 0, 0, 0, vt_id);  % [DIAG] 漏斗: 多候选ratio门拒绝(哈希碰撞)
            end

            if matched_exp_id ~= 0
                heading_yaw_exp_rad_tmp = get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X));
                d_xy_tmp = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
                lx_tmp = EXPERIENCES(CUR_EXP_ID).x_exp + d_xy_tmp * cos(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad + heading_yaw_exp_rad_tmp);
                ly_tmp = EXPERIENCES(CUR_EXP_ID).y_exp + d_xy_tmp * sin(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad + heading_yaw_exp_rad_tmp);
                lz_tmp = EXPERIENCES(CUR_EXP_ID).z_exp + ACCUM_DELTA_Z;
                ce_tmp = sqrt((EXPERIENCES(matched_exp_id).x_exp - lx_tmp)^2 + (EXPERIENCES(matched_exp_id).y_exp - ly_tmp)^2 + (EXPERIENCES(matched_exp_id).z_exp - lz_tmp)^2);
                if ~isfinite(ce_tmp) || ce_tmp > EXP_MAX_LOOP_CE
                    log_funnel(3, vals(1), ratio, matched_exp_id, ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 多候选CE门拒绝
                    matched_exp_id = 0;
                    DIAG_CE_REJECT = DIAG_CE_REJECT + 1;  % [DIAG]
                    if isfinite(ce_tmp) && ce_tmp > DIAG_CE_MAX, DIAG_CE_MAX = ce_tmp; end
                end
            end

            if matched_exp_id ~= 0
                log_exp_match(ce_tmp, matched_exp_id);  % [DIAG]
                matched_exp_id = nl_apply_loop_fix(matched_exp_id, ce_tmp);  % [B-fix] 朝向门+修正传播(0=拒绝, 落到建新节点)
                if matched_exp_id ~= 0
                    log_funnel(4, vals(1), ratio, matched_exp_id, ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 多候选闭环接受
                else
                    log_funnel(8, vals(1), ratio, ids(1), ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 朝向门拒绝
                end
            end
            if matched_exp_id ~= 0
                link_exists = 0;
                for link_id = 1 : EXPERIENCES(CUR_EXP_ID).numlinks
                    if EXPERIENCES(CUR_EXP_ID).links(link_id).exp_id == matched_exp_id
                        link_exists = 1;
                        break;
                    end
                end

                if link_exists == 0
                    EXPERIENCES(CUR_EXP_ID).numlinks = EXPERIENCES(CUR_EXP_ID).numlinks + 1;
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).exp_id = matched_exp_id;
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_z = ACCUM_DELTA_Z;
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).heading_yaw_exp_rad = ...
                        get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X));
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).facing_yaw_exp_rad = ...
                        get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, ACCUM_DELTA_YAW);

                    try
                        if matched_exp_id > 0 && matched_exp_id < CUR_EXP_ID
                            EXP_LOOP_CLOSURE_LINKS = EXP_LOOP_CLOSURE_LINKS + 1;
                        end
                    catch
                    end
                end

                PREV_EXP_ID = CUR_EXP_ID;
                CUR_EXP_ID = matched_exp_id;

                ACCUM_DELTA_X = 0;
                ACCUM_DELTA_Y = 0;
                ACCUM_DELTA_Z = 0;
                ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
            end
        else
            [vals, ids] = sort(delta_em);
            if ~isempty(vals)
                min_delta = vals(1);
                min_delta_id = ids(1);
            else
                min_delta = inf;
                min_delta_id = 1;
            end

            MIN_DELTA_EM = [MIN_DELTA_EM; min_delta];
            if min_delta < DELTA_EXP_GC_HDC_THRESHOLD
                DIAG_SINGLE_BELOW = DIAG_SINGLE_BELOW + 1;  % [DIAG]
            end
            if min_delta > DIAG_MAX_DELTA
                DIAG_MAX_DELTA = min_delta;  % [DIAG]
            end
            ratio = 0;
            if numel(vals) >= 2
                ratio = vals(1) / (vals(2) + eps);
            end
            if min_delta < DELTA_EXP_GC_HDC_THRESHOLD && (numel(vals) < 2 || ratio < EXP_MATCH_RATIO_THRESHOLD)

                matched_exp_id = VT(vt_id).EXPERIENCES(min_delta_id).id;

                if matched_exp_id <= 0 || matched_exp_id > NUM_EXPS
                    matched_exp_id = 0;
                end

                if matched_exp_id ~= 0
                    heading_yaw_exp_rad_tmp = get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X));
                    d_xy_tmp = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
                    lx_tmp = EXPERIENCES(CUR_EXP_ID).x_exp + d_xy_tmp * cos(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad + heading_yaw_exp_rad_tmp);
                    ly_tmp = EXPERIENCES(CUR_EXP_ID).y_exp + d_xy_tmp * sin(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad + heading_yaw_exp_rad_tmp);
                    lz_tmp = EXPERIENCES(CUR_EXP_ID).z_exp + ACCUM_DELTA_Z;
                    ce_tmp = sqrt((EXPERIENCES(matched_exp_id).x_exp - lx_tmp)^2 + (EXPERIENCES(matched_exp_id).y_exp - ly_tmp)^2 + (EXPERIENCES(matched_exp_id).z_exp - lz_tmp)^2);
                    if ~isfinite(ce_tmp) || ce_tmp > EXP_MAX_LOOP_CE
                        log_funnel(6, min_delta, ratio, matched_exp_id, ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 单候选CE门拒绝
                        matched_exp_id = 0;
                        DIAG_CE_REJECT = DIAG_CE_REJECT + 1;  % [DIAG]
                        if isfinite(ce_tmp) && ce_tmp > DIAG_CE_MAX, DIAG_CE_MAX = ce_tmp; end
                    end
                end

                % [DIAG/FIX] 对照: 禁用重锚定(永不匹配旧经验), 隔离纯DR基底质量
                global EXP_DISABLE_REANCHOR;
                if ~isempty(EXP_DISABLE_REANCHOR) && EXP_DISABLE_REANCHOR
                    matched_exp_id = 0;
                end

                if matched_exp_id ~= 0
                    matched_exp_id = nl_apply_loop_fix(matched_exp_id, ce_tmp);  % [B-fix] 朝向门+修正传播(0=拒绝, 落到建新节点)
                    if matched_exp_id ~= 0
                        log_funnel(7, min_delta, ratio, matched_exp_id, ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 单候选闭环接受
                    else
                        log_funnel(8, min_delta, ratio, ids(1), ce_tmp, d_xy_tmp, vt_id);  % [DIAG] 漏斗: 朝向门拒绝
                    end
                end

                if matched_exp_id ~= 0
                    DIAG_SINGLE_MATCH = DIAG_SINGLE_MATCH + 1;  % [DIAG]
                    log_exp_match(ce_tmp, matched_exp_id);  % [DIAG]
                    % see if the previous experience already has a link to the current experience
                    link_exists = 0;
                    for link_id = 1 : EXPERIENCES(CUR_EXP_ID).numlinks
                        if EXPERIENCES(CUR_EXP_ID).links(link_id).exp_id == matched_exp_id
                            link_exists = 1;
                            break;
                        end
                    end

                    % if we didn't find a link then create the link between current
                    % experience and the experience for the current visual template
                    if link_exists == 0
                        EXPERIENCES(CUR_EXP_ID).numlinks = EXPERIENCES(CUR_EXP_ID).numlinks + 1;
                        EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).exp_id = matched_exp_id;
                        %  EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2 + ACCUM_DELTA_Z^2);
                        EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
                        EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_z = ACCUM_DELTA_Z;
                        
                        EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).heading_yaw_exp_rad = ...
                            get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X)); % heading is the delta angle between current pose of exp and previous pose of exp
                       
                        EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).facing_yaw_exp_rad = ... % facing is the direction of each exp
                            get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, ACCUM_DELTA_YAW);

                        try
                            if matched_exp_id > 0 && matched_exp_id < CUR_EXP_ID
                                EXP_LOOP_CLOSURE_LINKS = EXP_LOOP_CLOSURE_LINKS + 1;
                            end
                        catch
                        end
                                        
                     end
                end

            else
                log_funnel(5, min_delta, ratio, 0, 0, 0, vt_id);  % [DIAG] 漏斗: 单候选无阈值内候选
            end

            % if there wasn't an experience with the current visual template and grid cell (x y z) and head direction cell (yaw, height)
            % then create a new experience
            if matched_exp_id == 0
                log_funnel(10, NaN, NaN, 0, NaN, NaN, vt_id);  % [DIAG] 漏斗: VT已知但无有效匹配→重复建节点
                NUM_EXPS = NUM_EXPS + 1;
                create_new_exp(CUR_EXP_ID, NUM_EXPS, vt_id, xGc, yGc, zGc, curYawHdc, curHeight);
                matched_exp_id = NUM_EXPS;
            end

            PREV_EXP_ID = CUR_EXP_ID;
            CUR_EXP_ID = matched_exp_id;

            ACCUM_DELTA_X = 0;
            ACCUM_DELTA_Y = 0;
            ACCUM_DELTA_Z = 0;
            ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
%             ACCUM_DELTA_HEIGHT = EXPERIENCES(CUR_EXP_ID).height_hdc;
        end

    elseif delta_em > DELTA_EXP_GC_HDC_THRESHOLD

        % VT did not change, but the pose-cell state drifted enough: create a new experience.
        log_funnel(1, delta_em, 0, 0, 0, 0, vt_id);  % [DIAG] 漏斗: VT未变位姿漂移新建
        NUM_EXPS = NUM_EXPS + 1;
        create_new_exp(CUR_EXP_ID, NUM_EXPS, vt_id, xGc, yGc, zGc, curYawHdc, curHeight);

        PREV_EXP_ID = CUR_EXP_ID;
        CUR_EXP_ID = NUM_EXPS;

        ACCUM_DELTA_X = 0;
        ACCUM_DELTA_Y = 0;
        ACCUM_DELTA_Z = 0;
        ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
%         ACCUM_DELTA_HEIGHT = EXPERIENCES(CUR_EXP_ID).height_hdc;

    else
        log_funnel(9, delta_em, 0, CUR_EXP_ID, 0, 0, vt_id);  % [DIAG] 漏斗: VT不变静默停留(不评估闭环)
    end

    global EXP_CORRECTION;
    global EXP_LOOPS;


    % do the experience map correction interatively for all the links in all the experiences
    for i = 1:EXP_LOOPS

        for exp_id = 1:NUM_EXPS

            for link_id=1:EXPERIENCES(exp_id).numlinks

                % experience 0 has a link to experience 1
                e0 = exp_id;
                e1 = EXPERIENCES(exp_id).links(link_id).exp_id;

                if isempty(e1) || e1 < 1 || e1 > NUM_EXPS
                    continue;
                end

                % work out where e0 thinks e1 (x,y) should be based on the stored link information
                lx = EXPERIENCES(e0).x_exp + EXPERIENCES(e0).links(link_id).d_xy * cos(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
                ly = EXPERIENCES(e0).y_exp + EXPERIENCES(e0).links(link_id).d_xy * sin(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
                lz = EXPERIENCES(e0).z_exp + EXPERIENCES(e0).links(link_id).d_z;  % 

                try
                    ce = sqrt((EXPERIENCES(e1).x_exp - lx)^2 + (EXPERIENCES(e1).y_exp - ly)^2 + (EXPERIENCES(e1).z_exp - lz)^2);
                    if ce > EXP_CONSTRAINT_ERR_MAX
                        EXP_CONSTRAINT_ERR_MAX = ce;
                    end
                catch
                end

                % correct e0 and e1 (x,y) by equal but opposite amounts
                % a 0.5 correction parameter means that e0 and e1 will be fully
                % corrected based on e0's link information
                EXPERIENCES(e0).x_exp = EXPERIENCES(e0).x_exp + (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
                EXPERIENCES(e0).y_exp = EXPERIENCES(e0).y_exp + (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
                EXPERIENCES(e0).z_exp = EXPERIENCES(e0).z_exp + (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;
                
                EXPERIENCES(e1).x_exp = EXPERIENCES(e1).x_exp - (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
                EXPERIENCES(e1).y_exp = EXPERIENCES(e1).y_exp - (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
                EXPERIENCES(e1).z_exp = EXPERIENCES(e1).z_exp - (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;

                % determine the angle between where e0 thinks e1's facing
                % should be based on the link information
                TempDeltaYawFacing = get_signed_delta_radian((EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).facing_yaw_exp_rad), EXPERIENCES(e1).yaw_exp_rad);

                % correct e0 and e1 facing by equal but opposite amounts
                % a 0.5 correction parameter means that e0 and e1 will be fully
                % corrected based on e0's link information           
                EXPERIENCES(e0).yaw_exp_rad = clip_radian_180(EXPERIENCES(e0).yaw_exp_rad + TempDeltaYawFacing * EXP_CORRECTION);
                EXPERIENCES(e1).yaw_exp_rad = clip_radian_180(EXPERIENCES(e1).yaw_exp_rad - TempDeltaYawFacing * EXP_CORRECTION);
                
            end
        end

    end

    % keep a frame by frame history of which experience was currently active
    global EXP_HISTORY;
    EXP_HISTORY = [EXP_HISTORY; CUR_EXP_ID];
end

%% [B-fix] 真闭环修正传播: 朝向一致性门验证 + 残差β·r累进NLM_LOOP_CORR
%  入参: matched_exp_id(通过CE门的匹配节点), ce_tmp(约束误差m)
%  返回: 0=拒绝(该次匹配作废, 上游落到建新节点路径); 否则=accepted的matched_exp_id
%  原理: matched是早期节点(EKF早期漂移小, 位置相对可信), 残差主要来自后缀累积漂移。
%  故不动节点, 只把后缀累计修正量 NLM_LOOP_CORR -= β·r, 其中
%  r = (CUR.pos + ACCUM_DELTA) - matched.pos 与上游CE门同一公式(ce_tmp=|r|)。
%  后续新节点经create_new_exp继承(钉扎=EKF锚点+NLM_LOOP_CORR)→闭环修正传播到整条后缀。
function out_id = nl_apply_loop_fix(matched_exp_id, ce_tmp)
    global EXPERIENCES CUR_EXP_ID NLM_LOOP_BETA NLM_LOOP_CE_MIN NLM_LOOP_STEP_MAX ...
           NLM_LOOP_YAW_GATE NLM_LOOP_CORR;
    global ACCUM_DELTA_X ACCUM_DELTA_Y ACCUM_DELTA_Z;
    global DIAG_LOOP_ACCEPT DIAG_YAW_REJECT;
    out_id = matched_exp_id;
    if isempty(matched_exp_id) || matched_exp_id <= 0
        return;
    end
    % 持续性修正总开关: 关=仅重锚定(纯jump, 基线行为), 开=朝向门+后缀传播
    global NLM_LOOP_FIX_ENABLE;
    if ~NLM_LOOP_FIX_ENABLE
        return;
    end
    % 仅对真闭环(远距重访)做修正; 近邻重访CE<CE_MIN=纯重锚定跳转(现状行为)
    if ce_tmp < NLM_LOOP_CE_MIN
        return;
    end
    % 朝向一致性门: 同向或反向(掉头重访)容差内才接受, 过滤GT无关的VT哈希碰撞
    % (get_signed_delta_radian 返回 [-pi,pi] 内从CUR到matched的有符号最短角差)
    d_yaw = get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, EXPERIENCES(matched_exp_id).yaw_exp_rad);
    if abs(d_yaw) > NLM_LOOP_YAW_GATE && abs(abs(d_yaw) - pi) > NLM_LOOP_YAW_GATE
        DIAG_YAW_REJECT = DIAG_YAW_REJECT + 1;
        out_id = 0;
        return;
    end
    % 残差r: 与CE门同一公式 — 当前节点沿ACCUM_DELTA方向推算的"matched应在位置"减matched现位置
    % (上游调用本函数时ACCUM_DELTA尚未清零, 可用)
    r = [EXPERIENCES(CUR_EXP_ID).x_exp + ACCUM_DELTA_X - EXPERIENCES(matched_exp_id).x_exp, ...
         EXPERIENCES(CUR_EXP_ID).y_exp + ACCUM_DELTA_Y - EXPERIENCES(matched_exp_id).y_exp, ...
         EXPERIENCES(CUR_EXP_ID).z_exp + ACCUM_DELTA_Z - EXPERIENCES(matched_exp_id).z_exp];
    % 单步钳位: |r|上限=STEP_MAX(默认25m), 按比例缩放, 防错配把后缀甩飞
    norm_r = norm(r);
    if norm_r < 1e-6
        return;
    end
    step = -NLM_LOOP_BETA * min(1, NLM_LOOP_STEP_MAX / norm_r) * r;
    % [A-fix] 约束模式: 只记录事件, 不动NLM_LOOP_CORR/节点(零地图反馈, 避免B-fix式
    % 错误闭环放大); 轨迹结束后核心脚本把事件松弛成闭环约束漂移场注入输出。
    % 非约束模式: 原B-fix后缀平移(NLM_LOOP_CORR), 保留作A/B基线。
    global NLM_LOOP_CONSTRAINT NLM_LOOP_EVENTS;
    if ~isempty(NLM_LOOP_CONSTRAINT) && NLM_LOOP_CONSTRAINT
        % 注意: 不加NLM_DC_CMAX幅值门 — 此处 norm_r ≡ ce_tmp(与CE门同一公式),
        % CE门已把残差限到 ≤EXP_MAX_LOOP_CE; 而DC逐集CMAX(2~6m)是跳变幅值门,
        % 套在闭环残差上会误杀所有真闭环(残差=区间累积漂移, 可达10~80m)。
        if isempty(NLM_LOOP_EVENTS), NLM_LOOP_EVENTS = zeros(0, 5); end
        t_old = 0;
        if isfield(EXPERIENCES(matched_exp_id), 'born_frame')
            t_old = EXPERIENCES(matched_exp_id).born_frame;
        end
        global NLM_FRAME_IDX;
        if isempty(NLM_FRAME_IDX), NLM_FRAME_IDX = 0; end
        % [A-fix去抖] MIN_GAP门: 真闭环=长间隔重访(绕一圈回来, 间隔数千帧);
        % 近邻/中程重访(gap<MIN_GAP)是路线自交/抖动误检(开放路线上全是这类假
        % 闭环), 不记约束事件 → 开放路线上约束模式回退中性(=基线), 真闭环路线
        % (折返/绕环)间隔≫MIN_GAP 正常记录。Town05实测假闭环间隔均<286帧。
        global NLM_LOOP_MIN_GAP;
        if isempty(NLM_LOOP_MIN_GAP), NLM_LOOP_MIN_GAP = 300; end
        if (NLM_FRAME_IDX - t_old) >= NLM_LOOP_MIN_GAP
            NLM_LOOP_EVENTS(end+1, :) = [NLM_FRAME_IDX, t_old, step(1), step(2), step(3)];
        end
    else
        % 累计修正量(后缀平移量): 后续新节点经create_new_exp继承
        NLM_LOOP_CORR = NLM_LOOP_CORR + step;
    end
    DIAG_LOOP_ACCEPT = DIAG_LOOP_ACCEPT + 1;
end

%% [DIAG] 记录每次成功匹配(重锚定事件), 供离线分解 teleport 损伤来源
%  字段: frame from_id to_id age_diff(from-to,正=匹配到更老节点)
%       from_x from_y to_x to_y ce(约束误差m)
function log_exp_match(ce_val, to_id)
    global DIAG_MATCH_LOG;
    global CUR_EXP_ID;
    global EXPERIENCES;
    global EXP_HISTORY;
    if isempty(DIAG_MATCH_LOG), DIAG_MATCH_LOG = zeros(0, 9); end
    % frame号=EXP_HISTORY已追加帧数+1(本帧在函数末尾才追加); EXP_HISTORY(end)是活跃经验ID, 不是帧号
    if isempty(EXP_HISTORY), cur_frame = 1; else, cur_frame = length(EXP_HISTORY) + 1; end
    row = [cur_frame, CUR_EXP_ID, to_id, CUR_EXP_ID - to_id, ...
           EXPERIENCES(CUR_EXP_ID).x_exp, EXPERIENCES(CUR_EXP_ID).y_exp, ...
           EXPERIENCES(to_id).x_exp, EXPERIENCES(to_id).y_exp, ce_val];
    DIAG_MATCH_LOG(end+1, :) = row;
end

%% [DIAG] 闭环漏斗日志: 逐帧记录每个VT决策点, 定位真闭环episode死在哪一级
%  行: [frame branch min_delta ratio match_id ce age d_xy vt_id num_exp]
%  branch: 0=VT新建 1=位姿漂移新建 2=多候选ratio拒 3=多候选CE拒
%          4=多候选闭环接受 5=无阈值内候选 6=单候选CE拒 7=单候选闭环接受 8=朝向门拒
%          9=VT不变静默停留(不评估闭环) 10=VT已知但无有效匹配→重复建节点
%  global FUNNEL_LOG_PATH 非空时追加写文件(默认关, 零开销)
function log_funnel(branch, min_delta, ratio, match_id, ce_val, d_xy, vt_id)
    global DIAG_FUNNEL_LOG;
    global CUR_EXP_ID;
    global EXP_HISTORY;
    global FUNNEL_LOG_PATH;
    global VT;
    if isempty(DIAG_FUNNEL_LOG), DIAG_FUNNEL_LOG = zeros(0, 10); end
    % frame号=EXP_HISTORY已追加帧数+1(本帧在函数末尾才追加); EXP_HISTORY(end)是活跃经验ID, 不是帧号
    if isempty(EXP_HISTORY), cur_frame = 1; else, cur_frame = length(EXP_HISTORY) + 1; end
    age = 0;
    if match_id > 0, age = CUR_EXP_ID - match_id; end
    num_exp = VT(vt_id).numExp;
    row = [cur_frame, branch, min_delta, ratio, match_id, ce_val, age, d_xy, vt_id, num_exp];
    DIAG_FUNNEL_LOG(end+1, :) = row;
    if ~isempty(FUNNEL_LOG_PATH) && ~isempty(FUNNEL_LOG_PATH{1})
        try
            fid = fopen(FUNNEL_LOG_PATH{1}, 'a');
            if fid > 0
                fprintf(fid, '%d %d %.4f %.4f %d %.3f %d %.3f %d %d\n', row);
                fclose(fid);
            end
        catch
        end
    end
end

