function yaw_height_hdc_iteration(vt_id, yawRotV, heightV)
%     hdc: head direction cell
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
    
    % 姿态细胞（Pose cell）更新步
    % 1. 添加视角模板能量（view template energy）
    % 2. 局部激活
    % 3. 局部抑制
    % 4. 全局抑制
    % 5. 正则化
    % 6. 路径积分 (yawRotV 然后 heightV)

   
    % 偏航和高度（俯仰）结合的头部朝向细胞。The HD (head directional) cells of yaw and height conjunctively
    % head directional conjunctively
    global YAW_HEIGHT_HDC;
    
    % 视觉模板 visual templete
    global VT;
    
    % yaw_height_hdc 网络中偏航的维度
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % yaw_height_hdc 网络中高度的维度
    global YAW_HEIGHT_HDC_H_DIM;
    
    % 偏航的 局部激活权重矩阵（local excitation weight matrix）维度
    global YAW_HEIGHT_HDC_EXCIT_Y_DIM;
    
    % 高度的 局部激活权重矩阵（local excitation weight matrix）维度
    global YAW_HEIGHT_HDC_EXCIT_H_DIM;
    
    % 偏航的 局部抑制权重矩阵（local excitation weight matrix）维度
    global YAW_HEIGHT_HDC_INHIB_Y_DIM;
    
    % 高度的 局部抑制权重矩阵（local excitation weight matrix）维度
    global YAW_HEIGHT_HDC_INHIB_H_DIM;
    
    % 全局抑制值
    global YAW_HEIGHT_HDC_GLOBAL_INHIB;
    
    
    % 重新看到视角模板时注入的能量
    global YAW_HEIGHT_HDC_VT_INJECT_ENERGY; 
    
    % yaw_height_hdc 网络中偏航的激活包络
    global YAW_HEIGHT_HDC_EXCIT_Y_WRAP;
    
    % yaw_height_hdc 网络中高度的激活包络
    global YAW_HEIGHT_HDC_EXCIT_H_WRAP;
    
    % yaw_height_hdc 网络中偏航的抑制包络
    global YAW_HEIGHT_HDC_INHIB_Y_WRAP;
    
    % yaw_height_hdc 网络中高度的抑制包络
    global YAW_HEIGHT_HDC_INHIB_H_WRAP;
    
    % yaw_height_hdc 网络中激活的权重
    global YAW_HEIGHT_HDC_EXCIT_WEIGHT;
    
    % yaw_height_hdc 网络中抑制的权重
    global YAW_HEIGHT_HDC_INHIB_WEIGHT;
    
    % 每个单元偏航 theta 的大小（以弧度为单位），2*pi/ YAW_HEIGHT_HDC_Y_DIM
    % 弧度 e.g. 2*pi/360 = 0.0175
    global YAW_HEIGHT_HDC_Y_TH_SIZE;   
    global YAW_HEIGHT_HDC_H_SIZE;     


    % 如果这不是一个新的视觉模板，则在其关联的 姿态细胞posecell 位置添加能量
    if VT(vt_id).first ~= 1
        act_yaw = min([max([round(VT(vt_id).hdc_yaw), 1]), YAW_HEIGHT_HDC_Y_DIM]);
        act_height = min([max([round(VT(vt_id).hdc_height), 1]), YAW_HEIGHT_HDC_H_DIM]);
        
        % 这些衰减注入视觉模板姿势单元位置的能量，
        % 这很重要，因为 姿态细胞 Posecells 会因长时间发生的不良视觉模板匹配而错误地捕捉（例如，在代理静止时发生的不良匹配）。
        % 这意味着需要识别多个视觉模板才能发生捕捉
        energy = YAW_HEIGHT_HDC_VT_INJECT_ENERGY * 1/30 * (30 - exp(1.2 * VT(vt_id).decay));
        if energy > 0
            YAW_HEIGHT_HDC(act_yaw, act_height) = YAW_HEIGHT_HDC(act_yaw, act_height) + energy;
        end
    end


    % Local excitation: yaw_height_hdc_local_excitation = yaw_height_hdc elements * yaw_height_hdc weights
    yaw_height_hdc_local_excit_new = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);
    for h = 1 : YAW_HEIGHT_HDC_H_DIM
        for y = 1 : YAW_HEIGHT_HDC_Y_DIM
            if YAW_HEIGHT_HDC(y, h) ~= 0
                yaw_height_hdc_local_excit_new(YAW_HEIGHT_HDC_EXCIT_Y_WRAP(y : y + YAW_HEIGHT_HDC_EXCIT_Y_DIM - 1),YAW_HEIGHT_HDC_EXCIT_H_WRAP(h : h + YAW_HEIGHT_HDC_EXCIT_H_DIM - 1)) = ...
                    yaw_height_hdc_local_excit_new(YAW_HEIGHT_HDC_EXCIT_Y_WRAP(y : y + YAW_HEIGHT_HDC_EXCIT_Y_DIM - 1),YAW_HEIGHT_HDC_EXCIT_H_WRAP(h : h + YAW_HEIGHT_HDC_EXCIT_H_DIM - 1)) ...
                        + YAW_HEIGHT_HDC(y,h) .* YAW_HEIGHT_HDC_EXCIT_WEIGHT;
            end    
        end
    end
    YAW_HEIGHT_HDC = yaw_height_hdc_local_excit_new;

    % 局部抑制：yaw_height_hdc_local_inhibition = hdc - hdc elements * hdc_inhib weights
    yaw_height_hdc_local_inhib_new = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);  
    for h = 1 : YAW_HEIGHT_HDC_H_DIM
        for y = 1 : YAW_HEIGHT_HDC_Y_DIM
            if YAW_HEIGHT_HDC(y, h) ~= 0
                yaw_height_hdc_local_inhib_new(YAW_HEIGHT_HDC_INHIB_Y_WRAP(y : y + YAW_HEIGHT_HDC_INHIB_Y_DIM - 1),YAW_HEIGHT_HDC_INHIB_H_WRAP(h : h + YAW_HEIGHT_HDC_INHIB_H_DIM - 1)) = ...
                    yaw_height_hdc_local_inhib_new(YAW_HEIGHT_HDC_INHIB_Y_WRAP(y : y + YAW_HEIGHT_HDC_INHIB_Y_DIM - 1),YAW_HEIGHT_HDC_INHIB_H_WRAP(h : h + YAW_HEIGHT_HDC_INHIB_H_DIM - 1)) ...
                    + YAW_HEIGHT_HDC(y, h) .* YAW_HEIGHT_HDC_INHIB_WEIGHT;
            end
        end
    end
    YAW_HEIGHT_HDC = YAW_HEIGHT_HDC - yaw_height_hdc_local_inhib_new;

    % global inhibition - PC_gi = PC_li elements - inhibition
    YAW_HEIGHT_HDC = (YAW_HEIGHT_HDC >= YAW_HEIGHT_HDC_GLOBAL_INHIB) .* (YAW_HEIGHT_HDC - YAW_HEIGHT_HDC_GLOBAL_INHIB);
    
    % 正则化
    total = sum(sum(YAW_HEIGHT_HDC));
    YAW_HEIGHT_HDC = YAW_HEIGHT_HDC./total;
    
    if yawRotV ~= 0
        % mod to work out the partial shift amount
        weight = mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, 1);
        if weight == 0
            weight = 1.0;
        end
        YAW_HEIGHT_HDC = circshift(YAW_HEIGHT_HDC, ...
            [sign(yawRotV) * floor(mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, YAW_HEIGHT_HDC_Y_DIM)) 0]) * (1.0 - weight) ...
            + circshift(YAW_HEIGHT_HDC, ...
            [sign(yawRotV) * ceil(mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, YAW_HEIGHT_HDC_Y_DIM)) 0]) * (weight);
    end
    
    if heightV ~= 0
        % mod to work out the partial shift amount
        weight = mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, 1);
        if weight == 0
            weight = 1.0;
        end
        YAW_HEIGHT_HDC = circshift(YAW_HEIGHT_HDC, ...
            [0 sign(heightV) * floor(mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, YAW_HEIGHT_HDC_H_DIM))]) * (1.0 - weight) ...
            + circshift(YAW_HEIGHT_HDC, ...
            [0 sign(heightV) * ceil(mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, YAW_HEIGHT_HDC_H_DIM))]) * (weight);
    end

end






