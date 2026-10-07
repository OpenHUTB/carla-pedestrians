function create_new_exp(curExpId, newExpId, vt_id, xGc, yGc, zGc, curYawHdc, curHeight)
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

    % create a new experience and current experience to it
    global VT;
    global EXPERIENCES;
    global ACCUM_DELTA_X;
    global ACCUM_DELTA_Y;
    global ACCUM_DELTA_Z;
    global ACCUM_DELTA_YAW; 
%     global ACCUM_DELTA_HEIGHT; 
    
    % add link information to the current experience for the new experience
    % including the experience_id, odo distance to the experience, 
    % odo heading (relative to the current experience's facing) to the experience, 
    % odo delta facing (relative to the current expereience's facing).
    
    EXPERIENCES(curExpId).numlinks = EXPERIENCES(curExpId).numlinks + 1;
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).exp_id = newExpId;
    
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).d_z = ACCUM_DELTA_Z;
    
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).heading_yaw_exp_rad = ... 
        get_signed_delta_radian(EXPERIENCES(curExpId).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X)); % heading is the delta angle between current pose of exp and previous pose of exp
        
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).facing_yaw_exp_rad = ... % facing is the direction of each exp
        get_signed_delta_radian(EXPERIENCES(curExpId).yaw_exp_rad, ACCUM_DELTA_YAW);
    
    % [A-fix] 记录节点创建帧(闭环约束区间衰减用; 驱动脚本逐帧注入 NLM_FRAME_IDX)
    global NLM_FRAME_IDX;
    if isempty(NLM_FRAME_IDX), NLM_FRAME_IDX = 0; end
    EXPERIENCES(newExpId).born_frame = NLM_FRAME_IDX;

    % create the new experience which will have no links to being with
    % associate with 3d gc
    EXPERIENCES(newExpId).x_gc = xGc;
    EXPERIENCES(newExpId).y_gc = yGc;
    EXPERIENCES(newExpId).z_gc = zGc;
    
    % associate with hdc
    EXPERIENCES(newExpId).yaw_hdc = curYawHdc;
    EXPERIENCES(newExpId).height_hdc = curHeight;
    
    % associate with vt
    EXPERIENCES(newExpId).vt_id = vt_id;
    
    % update the coordinate of em (x_exp, y_exp, z_exp, yaw_exp_rad, height_exp)
    % 2026-10-02 EKF-odo节点坐标钉扎: 若 NLM_EKF_ANCHOR_MAP 已设置(EKF-odo模式),
    % 新节点坐标直接取创建帧的EKF地图系位置(真实米), 消除DR累积在GC钳位下的
    % 欠积分压缩(地图架系scale 0.38-1.6); 非EKF-odo路径该全局为空, 行为不变
    global NLM_EKF_ANCHOR_MAP;
    global NLM_LOOP_CORR;  % [B-fix] 闭环累计修正量
    if ~isempty(NLM_EKF_ANCHOR_MAP)
        % [B-fix] 钉扎坐标继承闭环累计修正: 新节点=EKF锚点+NLM_LOOP_CORR,
        % 使已接受的闭环修正传播到后续整条后缀(否则闭环一离开匹配节点即被抹掉)
        if isempty(NLM_LOOP_CORR), NLM_LOOP_CORR = [0, 0, 0]; end
        EXPERIENCES(newExpId).x_exp = NLM_EKF_ANCHOR_MAP(1) + NLM_LOOP_CORR(1);
        EXPERIENCES(newExpId).y_exp = NLM_EKF_ANCHOR_MAP(2) + NLM_LOOP_CORR(2);
        EXPERIENCES(newExpId).z_exp = NLM_EKF_ANCHOR_MAP(3) + NLM_LOOP_CORR(3);
    else
        EXPERIENCES(newExpId).x_exp = EXPERIENCES(curExpId).x_exp + ACCUM_DELTA_X;
        EXPERIENCES(newExpId).y_exp = EXPERIENCES(curExpId).y_exp + ACCUM_DELTA_Y;
        EXPERIENCES(newExpId).z_exp = EXPERIENCES(curExpId).z_exp + ACCUM_DELTA_Z;
    end
    
    EXPERIENCES(newExpId).yaw_exp_rad = clip_radian_180(ACCUM_DELTA_YAW);
%     EXPERIENCES(newExpId).height_exp = ACCUM_DELTA_HEIGHT;
    
    EXPERIENCES(newExpId).numlinks = 0;
    EXPERIENCES(newExpId).links = [];

    % add this experience id to the vt for efficient lookup
    VT(vt_id).numExp = VT(vt_id).numExp + 1;
    VT(vt_id).EXPERIENCES(VT(vt_id).numExp).id = newExpId;
end

