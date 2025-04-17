function visual_odo_initial(varargin)
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

    %% 初始化一些全局变量

    % 定义里程计图像的 Y（垂直）范围，包括平移速度图像、旋转速度图像和俯仰速度图像

    % 里程计图像平移范围（x, y）
    global ODO_IMG_TRANS_Y_RANGE;
    global ODO_IMG_TRANS_X_RANGE;
        
    % 里程计图像高度（俯仰）垂直范围(x, y)
    global ODO_IMG_HEIGHT_V_Y_RANGE;
    global ODO_IMG_HEIGHT_V_X_RANGE;  
    
    % 里程计图像偏航（旋转）范围(x, y)
    global ODO_IMG_YAW_ROT_Y_RANGE;
    global ODO_IMG_YAW_ROT_X_RANGE;
   
    % 定义里程计调整大小后的图像尺寸
    global ODO_IMG_TRANS_RESIZE_RANGE;
    global ODO_IMG_YAW_ROT_RESIZE_RANGE;
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE;
   
    % 定义平移速度、旋转速度和俯仰速度的尺度
    global ODO_TRANS_V_SCALE;
    global ODO_YAW_ROT_V_SCALE;
    global ODO_HEIGHT_V_SCALE;

    % 定义平移速度、旋转速度和俯仰速度的最大阈值
    global MAX_TRANS_V_THRESHOLD;
    global MAX_YAW_ROT_V_THRESHOLD;
    global MAX_HEIGHT_V_THRESHOLD;
    
    % 定义垂直和水平方向的视觉里程计位移匹配变量
    global ODO_SHIFT_MATCH_VERT;
    global ODO_SHIFT_MATCH_HORI;

    % 定义水平和垂直方向的视野角度。
    % 视野 (Field of View, FOV)，角度 (degree, DEG) 所有 FOV 的水平、垂直和对角线角度
    global FOV_HORI_DEGREE;
    global FOV_VERT_DEGREE;
     
    global KEY_POINT_SET;
    global ODO_STEP;

    %%% 处理参数

    for i=1:(nargin-1)
        if ischar(varargin{i})
            switch varargin{i}
                case 'ODO_IMG_TRANS_Y_RANGE', ODO_IMG_TRANS_Y_RANGE = varargin{i+1};
                case 'ODO_IMG_TRANS_X_RANGE', ODO_IMG_TRANS_X_RANGE = varargin{i+1};              
                case 'ODO_IMG_HEIGHT_V_Y_RANGE', ODO_IMG_HEIGHT_V_Y_RANGE = varargin{i+1}; 
                case 'ODO_IMG_HEIGHT_V_X_RANGE', ODO_IMG_HEIGHT_V_X_RANGE = varargin{i+1};     
                case 'ODO_IMG_YAW_ROT_Y_RANGE', ODO_IMG_YAW_ROT_Y_RANGE = varargin{i+1};
                case 'ODO_IMG_YAW_ROT_X_RANGE', ODO_IMG_YAW_ROT_X_RANGE = varargin{i+1};
                
                case 'ODO_IMG_TRANS_RESIZE_RANGE', ODO_IMG_TRANS_RESIZE_RANGE = varargin{i+1};
                case 'ODO_IMG_YAW_ROT_RESIZE_RANGE', ODO_IMG_YAW_ROT_RESIZE_RANGE = varargin{i+1};
                case 'ODO_IMG_HEIGHT_V_RESIZE_RANGE', ODO_IMG_HEIGHT_V_RESIZE_RANGE = varargin{i+1};
                      
                case 'ODO_TRANS_V_SCALE', ODO_TRANS_V_SCALE = varargin{i+1};
                case 'ODO_YAW_ROT_V_SCALE', ODO_YAW_ROT_V_SCALE = varargin{i+1};
                case 'ODO_HEIGHT_V_SCALE', ODO_HEIGHT_V_SCALE = varargin{i+1};
                    
                case 'MAX_TRANS_V_THRESHOLD', MAX_TRANS_V_THRESHOLD = varargin{i+1};         
                case 'MAX_YAW_ROT_V_THRESHOLD', MAX_YAW_ROT_V_THRESHOLD = varargin{i+1};
                case 'MAX_HEIGHT_V_THRESHOLD', MAX_HEIGHT_V_THRESHOLD = varargin{i+1};
                 
                case 'ODO_SHIFT_MATCH_HORI', ODO_SHIFT_MATCH_HORI = varargin{i+1};
                case 'ODO_SHIFT_MATCH_VERT', ODO_SHIFT_MATCH_VERT = varargin{i+1};
 
                case 'FOV_HORI_DEGREE', FOV_HORI_DEGREE = varargin{i+1};
                case 'FOV_VERT_DEGREE', FOV_VERT_DEGREE = varargin{i+1};
               
                case 'KEY_POINT_SET', KEY_POINT_SET = varargin{i+1};
                case 'ODO_STEP', ODO_STEP = varargin{i+1};
                    
            end
        end
    end
    
    % x_sums, 分别对当前图像和前一图像中每一列强度求和。形成一维向量 1*N 数组

    global PREV_TRANS_V_IMG_X_SUMS;
    global PREV_YAW_ROT_V_IMG_X_SUMS;
    global PREV_HEIGHT_V_IMG_Y_SUMS;
    
    PREV_YAW_ROT_V_IMG_X_SUMS = zeros(1,ODO_IMG_TRANS_RESIZE_RANGE(2));
    PREV_HEIGHT_V_IMG_Y_SUMS = zeros(ODO_IMG_HEIGHT_V_RESIZE_RANGE(1), 1);
    PREV_TRANS_V_IMG_X_SUMS = zeros(1,ODO_IMG_TRANS_RESIZE_RANGE(2) - ODO_SHIFT_MATCH_HORI);
    
    % 定义先前的速率以保持稳定的速度
    global PREV_TRANS_V;
    global PREV_YAW_ROT_V;
    global PREV_HEIGHT_V;
    PREV_TRANS_V = 0.025;    % 0.03
    PREV_YAW_ROT_V = 0;
    PREV_HEIGHT_V = 0;
    
    %%% 完成视觉里程计的设置
   
end