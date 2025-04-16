function gc_iteration(vt_id, transV, curYawThetaInRadian,heightV)
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
    
    % 3D 网格细胞更新步
    % 1. 添加视图模板能量
    % 2. 局部激活
    % 3. 局部抑制
    % 4. 全局抑制
    % 5. 正则化
    % 6. 路径积分 (视图平移？ vtrans，然后 视图高度 vheight)
    
    %% define some variables of 3d gc
    % The 3D Grid Cells Network
    global GRIDCELLS;
    
    % The visual template
    global VT;

    % The x, y, z dimension of 3D Grid Cells Model (3D CAN) 
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;
    
    % The dimension of local excitation weight matrix for x, y, z
    global GC_EXCIT_X_DIM;
    global GC_EXCIT_Y_DIM;
    global GC_EXCIT_Z_DIM;
    
    % The dimension of local excitation weight matrix for x, y, z
    global GC_INHIB_X_DIM;
    global GC_INHIB_Y_DIM;
    global GC_INHIB_Z_DIM;
    
    % The global inhibition value
    global GC_GLOBAL_INHIB;   
    
    % The amount of energy injected when a view template is re-seen
    global GC_VT_INJECT_ENERGY;
        
    % The excit wrap of x,y,z in 3D grid cell network
    global GC_EXCIT_X_WRAP;
    global GC_EXCIT_Y_WRAP;
    global GC_EXCIT_Z_WRAP;
    
    % The inhibit wrap of x,y,z in 3D grid cell network
    global GC_INHIB_X_WRAP;
    global GC_INHIB_Y_WRAP;
    global GC_INHIB_Z_WRAP;
    
    % The weight of excitation in 3D grid cell network
    global GC_EXCIT_WEIGHT;
    
    % The weight of inhibition in 3D grid cell network
    global GC_INHIB_WEIGHT;
    
    % The x, y, z cell size of each unit in meter or unit
    global GC_X_TH_SIZE;   
    global GC_Y_TH_SIZE;   
    global GC_Z_TH_SIZE;   
 
    
    %% 视图模板 (view template, vt) 添加能量
    % 如果这不是一个新的视觉模板，则在其关联的网格单元位置添加能量
    if VT(vt_id).first ~= 1
        actX = min([max([round(VT(vt_id).gc_x), 1]), GC_X_DIM]);
        actY = min([max([round(VT(vt_id).gc_y), 1]), GC_Y_DIM]);
        actZ = min([max([round(VT(vt_id).gc_z), 1]), GC_Z_DIM]);

        % 这会衰减注入视觉模板网格单元位置的能量，
        % 这很重要，因为网格单元会因长时间发生的不良视觉模板匹配而错误地捕捉
        % （例如，在代理静止时发生的不良匹配）。
        % 这意味着需要识别多个视觉模板才能发生捕捉
        energy = GC_VT_INJECT_ENERGY * 1/30 * (30 - exp(1.2 * VT(vt_id).decay));
        if energy > 0
            GRIDCELLS(actX, actY, actZ) = GRIDCELLS(actX, actY, actZ) + energy;
        end
    end

    %% 局部抑制
    % 局部抑制 GC_local_excitation = GC elements * GC weights
    gridcell_local_excit_new = zeros(GC_X_DIM, GC_X_DIM, GC_Z_DIM);
    for z = 1:GC_Z_DIM
        for x = 1:GC_X_DIM
            for y = 1:GC_Y_DIM
                if GRIDCELLS(x,y,z) ~= 0
                    gridcell_local_excit_new(GC_EXCIT_X_WRAP(x : x + GC_EXCIT_X_DIM - 1), ...
                        GC_EXCIT_Y_WRAP(y : y + GC_EXCIT_Y_DIM - 1), ...
                        GC_EXCIT_Z_WRAP(z : z + GC_EXCIT_Z_DIM - 1)) = ...
                        gridcell_local_excit_new(GC_EXCIT_X_WRAP(x : x + GC_EXCIT_X_DIM - 1), ...
                        GC_EXCIT_X_WRAP(y : y + GC_EXCIT_Y_DIM - 1), ...
                        GC_EXCIT_Z_WRAP(z : z + GC_EXCIT_Z_DIM - 1)) ...
                        + GRIDCELLS(x,y,z) .* GC_EXCIT_WEIGHT;
                end
            end
        end
    end
    GRIDCELLS = gridcell_local_excit_new;

    %% local inhibition
    % local inhibition - GC_li = GC_le - GC_le elements * GC weights
    gridcell_local_inhib_new = zeros(GC_X_DIM, GC_X_DIM,GC_Z_DIM);  
    for z=1:GC_Z_DIM
        for x=1:GC_X_DIM
            for y=1:GC_Y_DIM
                if GRIDCELLS(x,y,z) ~= 0
                    gridcell_local_inhib_new(GC_INHIB_X_WRAP(x : x + GC_INHIB_X_DIM - 1), ...
                        GC_INHIB_Y_WRAP(y : y + GC_INHIB_Y_DIM - 1), ...
                        GC_INHIB_Z_WRAP(z : z + GC_INHIB_Z_DIM - 1)) = ...
                        gridcell_local_inhib_new(GC_INHIB_X_WRAP(x : x + GC_INHIB_X_DIM - 1), ...
                        GC_INHIB_Y_WRAP(y : y + GC_INHIB_Y_DIM - 1), ...
                        GC_INHIB_Z_WRAP(z : z + GC_INHIB_Z_DIM - 1)) ...
                        + GRIDCELLS(x,y,z) .* GC_INHIB_WEIGHT;
                end
            end
        end
    end
    GRIDCELLS = GRIDCELLS - gridcell_local_inhib_new;

    %% 全局抑制
    % global inhibition - gc_gi = GC_li elements - inhibition
    GRIDCELLS = (GRIDCELLS >= GC_GLOBAL_INHIB) .* (GRIDCELLS - GC_GLOBAL_INHIB);

    %% 正则化
    % normalisation
    total = sum(sum(sum(GRIDCELLS)));
    GRIDCELLS = GRIDCELLS./total;

    
    %% 路径积分
    
    % pi in x-y plane
    for indZ = 1 : GC_Z_DIM
        if curYawThetaInRadian == 0
            GRIDCELLS(:,:,indZ) = GRIDCELLS(:,:,indZ).*(1.0 - transV)  ...
                + circshift(GRIDCELLS(:,:,indZ), [0 1]).* transV;
        elseif curYawThetaInRadian == pi/2
            GRIDCELLS(:,:,indZ) = GRIDCELLS(:,:,indZ).*(1.0 - transV)  ...
                + circshift(GRIDCELLS(:,:,indZ), [1 0]).* transV;
        elseif curYawThetaInRadian == pi
            GRIDCELLS(:,:,indZ) = GRIDCELLS(:,:,indZ).*(1.0 - transV) ...
                + circshift(GRIDCELLS(:,:,indZ), [0 -1]).* transV;
        elseif curYawThetaInRadian == 3*pi/2
            GRIDCELLS(:,:,indZ) = GRIDCELLS(:,:,indZ).*(1.0 - transV)  ...
                + circshift(GRIDCELLS(:,:,indZ), [-1 0]).* transV;   
        else
            
            % rotate the GRIDCELLS instead of implementing for four quadrants
            % floor  rounded down integer offsets
            gcInZPlane90 = rot90(GRIDCELLS(:,:,indZ), floor(curYawThetaInRadian *2/pi));

            % residue 
            dir90 = curYawThetaInRadian - floor(curYawThetaInRadian *2/pi)* pi/2;

            gcInZPlaneNew = zeros(GC_X_DIM + 2, GC_Y_DIM + 2);            
            gcInZPlaneNew(2:end-1,2:end-1) = gcInZPlane90;

            weight_sw = transV^2 * cos(dir90) * sin(dir90);
            weight_se = transV * sin(dir90) - transV^2 * cos(dir90) * sin(dir90);
            weight_nw = transV * cos(dir90) - transV^2 * cos(dir90) * sin(dir90);
            weight_ne = 1.0 - weight_sw - weight_se - weight_nw;

            gcInZPlaneNew = gcInZPlaneNew.*weight_ne + circshift(gcInZPlaneNew, [0 1]).*weight_nw + circshift(gcInZPlaneNew, [1 0]).*weight_se + circshift(gcInZPlaneNew, [1 1]).*weight_sw;

            gcInZPlane90 = gcInZPlaneNew(2:end-1,2:end-1);
            gcInZPlane90(2:end,1) = gcInZPlane90(2:end,1) + gcInZPlaneNew(3:end-1,end);
            gcInZPlane90(1,2:end) = gcInZPlane90(1,2:end) + gcInZPlaneNew(end,3:end-1);
            gcInZPlane90(1,1) = gcInZPlane90(1,1) + gcInZPlaneNew(end:end);

            GRIDCELLS(:,:,indZ) = rot90(gcInZPlane90, 4 - floor(curYawThetaInRadian * 2/pi));
        end
    end
    
    % pi in z axis
    if heightV ~= 0
        % mod to work out the partial shift amount
        weight = mod(abs(heightV) / GC_Z_TH_SIZE, 1);
        if weight == 0
            weight = 1.0;
        end
        GRIDCELLS = circshift(GRIDCELLS, [0 0 sign(heightV)* floor(mod(abs(heightV) / GC_Z_TH_SIZE, GC_Z_DIM))]) * (1.0 - weight) ...
            + circshift(GRIDCELLS, [0 0 sign(heightV) * ceil(mod(abs(heightV) / GC_Z_TH_SIZE, GC_Z_DIM))]) * (weight);
    end

end