function [transV, yawRotV, heightV] = visual_odometry(rawImg)
%     NeuroSLAM System Copyright (C) 2018-2019 
%     NeuroSLAM: A Brain inspired SLAM System for 3D Environments
%     视觉里程计用IMU里程计代替
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
   
    % 简单的视觉里程计与扫描线强度轮廓 scanline intensity profile 算法
    % 输入是原始图像
    % 输出包括水平平移速度、旋转速度、垂直平移速度（垂直）
    % 尝试用加速度（前庭系统）、速度（速度细胞）、位置（位置细胞）


    %% 开始设置视觉里程计

    % 定义用于绘制估计平移、旋转和俯仰速度的子图像的变量
    global SUB_TRANS_IMG;
    global SUB_YAW_ROT_IMG;
    global SUB_HEIGHT_V_IMG;

          
    % 定义输入图像的 Y（垂直）范围，包括平移速度图像、旋转速度图像和高度变化速度图像
    global ODO_IMG_HEIGHT_V_Y_RANGE;
    global ODO_IMG_YAW_ROT_Y_RANGE;

    % 定义用于里程计的图像的 X（水平）范围
    global ODO_IMG_YAW_ROT_X_RANGE;
    global ODO_IMG_HEIGHT_V_X_RANGE;

    global ODO_IMG_TRANS_Y_RANGE;
    global ODO_IMG_TRANS_X_RANGE;
    
    % 为 里程计odo 定义调整大小的图像的大小
    global ODO_IMG_YAW_ROT_RESIZE_RANGE;
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE;
    global ODO_IMG_TRANS_RESIZE_RANGE;
   
    % 定义平移速度、旋转速度和俯仰速度的比例
    global ODO_TRANS_V_SCALE;
    
    global ODO_YAW_ROT_V_SCALE;
    global ODO_HEIGHT_V_SCALE;
     
    % 定义平移速度、旋转速度和俯仰速度的最大阈值
    global MAX_TRANS_V_THRESHOLD;
    global MAX_YAW_ROT_V_THRESHOLD;
    global MAX_HEIGHT_V_THRESHOLD;
       
    % 定义视觉里程计在垂直和水平方向上位移匹配的变量
    global ODO_SHIFT_MATCH_VERT;
    global ODO_SHIFT_MATCH_HORI;
    
    % 在水平和垂直方向上定义视场的角度。视场（Field of View, FOV），度（Degree, DEG）所有FOV的水平，垂直和对角度
    global FOV_HORI_DEGREE;
    global FOV_VERT_DEGREE;
          
    % x_sum，将当前图像和前一图像中的每列强度分别相加。
    % 形成一维向量, 1*N array
    global PREV_TRANS_V_IMG_X_SUMS;
    global PREV_YAW_ROT_V_IMG_X_SUMS;
    global PREV_HEIGHT_V_IMG_Y_SUMS;
    
    % define the previous velocity for keeping stable speed
    global PREV_TRANS_V;
    global PREV_YAW_ROT_V;
    global PREV_HEIGHT_V;
    
    global DEGREE_TO_RADIAN;
    
    %%% End up for setting up the visual odometry 
    
    global OFFSET_YAW_ROT;
    global OFFSET_HEIGHT_V;
    
    %% 开始计算水平旋转速度（偏航）

    % 从具有范围限制的原始图像中获取旋转速度的sub_image
    subRawImg = rawImg(ODO_IMG_YAW_ROT_Y_RANGE, ODO_IMG_YAW_ROT_X_RANGE);
    subRawImg = imresize(subRawImg, ODO_IMG_YAW_ROT_RESIZE_RANGE); 
    horiDegPerPixel = FOV_HORI_DEGREE / size(subRawImg, 2);
    
    SUB_YAW_ROT_IMG = subRawImg;
    SUB_TRANS_IMG = subRawImg;
    
%     % 获取调整大小后的模板图像的大小
%     ySizeODOImg = ODO_IMG_YAW_ROT_RESIZE_RANGE(1);
%     xSizeODOImg = ODO_IMG_YAW_ROT_RESIZE_RANGE(2);
%     ySizeNormImg = ySizeODOImg;
%     
%     PATCH_SIZE_Y_K = 5;
%     PATCH_SIZE_X_K = 5;
%     
%     % define a temp variable for patch normalization
%     % extent the dimension of raw image for patch normalization (extODOImg, extension sub image of vtResizedImg)
%     extODOImg = zeros(ySizeODOImg + PATCH_SIZE_Y_K - 1, xSizeODOImg + PATCH_SIZE_X_K - 1);
%     extODOImg(fix((PATCH_SIZE_Y_K + 1 )/2) : fix((PATCH_SIZE_Y_K + 1 )/2) ...
%         + ySizeNormImg - 1 ,  fix((PATCH_SIZE_X_K + 1 )/2) : fix((PATCH_SIZE_X_K + 1 )/2) ...
%         + xSizeODOImg - 1 ) = subRawImg;
%     
%     %% patch normalisation is applied to compensate for changes in lighting condition
%     for v = 1: ySizeNormImg 
%         for u = 1 : xSizeODOImg 
%             % get patch image
%             patchImg = extODOImg(v : v + PATCH_SIZE_Y_K - 1, u : u + PATCH_SIZE_X_K -1);        
% 
%             % Find the average of the matrix patch image
%             meanPatchImg = mean2(patchImg);
% 
%             % Find the standard deviation of the matrix patch image
%             stdPatchIMG = std2(patchImg);
%             
%             % Normalize the sub raw image
%             % normODOImg(v,u) = 11 * 11 * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
%             % normODOImg(v,u) =  PATCH_SIZE_Y_K * PATCH_SIZE_X_K * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
%             normODOImg(v,u) =  (subRawImg(v,u) - meanPatchImg ) / stdPatchIMG/ 255;
%             
%         end
%     end
%     
%     SUB_YAW_ROT_IMG = normODOImg;
%     SUB_TRANS_IMG = normODOImg;
    
    % 得到图像每列平均总和强度值的x_sum
    imgXSums = sum(subRawImg);
    avgIntensity = sum(imgXSums) / size(imgXSums, 2);
    imgXSums = imgXSums / avgIntensity;

    % 将当前图像与之前的图像进行比较
    % 得到两幅图像之间最小的偏移量和最小的强度差
    [minOffsetYawRot, minDiffIntensityRot] = compare_segments(imgXSums, PREV_YAW_ROT_V_IMG_X_SUMS, ODO_SHIFT_MATCH_HORI, size(imgXSums, 2));  

    OFFSET_YAW_ROT = minOffsetYawRot;
    yawRotV = ODO_YAW_ROT_V_SCALE * minOffsetYawRot * horiDegPerPixel;  % in deg

    if abs(yawRotV) > MAX_YAW_ROT_V_THRESHOLD
        yawRotV = PREV_YAW_ROT_V;
    else
        PREV_YAW_ROT_V = yawRotV;
    end

    PREV_YAW_ROT_V_IMG_X_SUMS = imgXSums;
    PREV_TRANS_V_IMG_X_SUMS = imgXSums;
    %%% end up to compute the translational velocity
    
   
    %% 开始计算总平移速度（前庭系统怎么做？）

    % 原理
    % 速度是根据图像变化的速率估计的。
    % the speed measure v is obtained from the filtered average absolute
    % intensity difference between consecutive scanline intensity profiles at
    % the best match for rotation with best offest in yaw and pitch shift
 
    transV = minDiffIntensityRot * ODO_TRANS_V_SCALE;
    
    
    if transV > MAX_TRANS_V_THRESHOLD
       transV = PREV_TRANS_V;
    else
       PREV_TRANS_V = transV;
    end
    
    
    %% 开始计算高度改变速度

    % 从具有范围约束的原始图像中获取 俯仰 速度的子图像
    subRawImg = rawImg(ODO_IMG_HEIGHT_V_Y_RANGE, ODO_IMG_HEIGHT_V_X_RANGE);
    subRawImg = imresize(subRawImg, ODO_IMG_HEIGHT_V_RESIZE_RANGE); 
    vertDegPerPixel = FOV_VERT_DEGREE / size(subRawImg, 1);
    

    if minOffsetYawRot > 0
        subRawImg = subRawImg(:, minOffsetYawRot + 1 : end);
    else
        subRawImg = subRawImg(:, 1 : end -(-minOffsetYawRot));
    end

    SUB_HEIGHT_V_IMG = subRawImg;

    imageYSums = sum(subRawImg,2);
    avgIntensity = sum(imageYSums) / size(imageYSums, 1);
    imageYSums = imageYSums / avgIntensity; 

    [minOffsetHeightV, minDiffIntensityHeight] = compare_segments(imageYSums, PREV_HEIGHT_V_IMG_Y_SUMS, ODO_SHIFT_MATCH_VERT, size(imageYSums, 1));
    
    
    if minOffsetHeightV < 0
        minDiffIntensityHeight = - minDiffIntensityHeight;
    end
    
    OFFSET_HEIGHT_V = minOffsetHeightV;

    % 使用经验确定的常数 TRANSLATIONAL_VELOCITY_SCALE 将感知速度转换为物理速度

%     if abs(minOffsetYawRot) < 2
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%     else 
%         heightV = 0;
%     end


%     if abs(minOffsetYawRot) < 2
%         heightV = minOffsetHeightV * 0.1;
%     else 
%         heightV = 0;
%     end
%     
%     if abs(minOffsetHeightV) < 2
%         heightV = 0;
%     end
    if minOffsetHeightV > 3
        heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
    else
        heightV = 0;
    end
    heightV = 0;
%   if abs(minOffsetHeightV) > 1
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%             
%     else 
%         heightV = 0;
%   end
%     
%     if abs(minOffsetHeightV) > 1 && abs(minOffsetYawRot) < 2
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%             
%     else 
%         heightV = 0;
%     end
    
    % 为了检测过大的平移速度，阈值 Vmax 确保不会使用虚假的高图像差异。
    % 突然的照明变化（例如上坡时直面太阳）可能会导致较大的图像差异。

    % 根据平均运动速度定义最大速度阈值

    if abs(heightV) > MAX_HEIGHT_V_THRESHOLD
        heightV = PREV_HEIGHT_V;
    else
        PREV_HEIGHT_V = heightV;
    end
    
%     if abs(minOffsetHeightV) >3  
%         transV = 0;
%     end
    
    PREV_HEIGHT_V_IMG_Y_SUMS = imageYSums;
    %%% end up to compute the pitch velocity
    
end
