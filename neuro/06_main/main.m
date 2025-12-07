
function main(visualDataFile, groundTruthFile, expMapHistoryFile, odoMapHistoryFile, vtHistoryFile, emHistoryFile, gcTrajFile, hdcTrajFile,varargin)
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
    
    %% 日志：启动函数
    disp('='*50);
    disp('=== NeuroSLAM 优化版主程序启动 ===');
    disp('='*50);

    clear EXP_NODES_LINKS
    %% 1. 添加所有依赖路径（确保能找到辅助函数）
    disp('[1/12] 开始添加依赖路径...');
    rootDir = '/home/dream/Neuro_WS/carla-pedestrians/neuro';
    addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
    addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
    addpath(fullfile(rootDir, '04_visual_template'));
    addpath(fullfile(rootDir, '02_multilayered_experience_map'));
    addpath(fullfile(rootDir, '06_main'));
    addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
    savepath;
    disp('[1/12] 依赖路径添加完成！');

    %% 2. 补全所有全局变量（无重复，全初始化）
    disp('[2/12] 开始初始化全局变量...');
    % 视觉模板相关
    global PREV_VT_ID; PREV_VT_ID = -1;
    global SUB_VT_IMG; SUB_VT_IMG = [];
    global VT_HISTORY; VT_HISTORY = [];
    global IMG_TYPE; IMG_TYPE = '*.png';
    global VT_STEP; VT_STEP = 1;
    global VT_TEMPLATES; VT_TEMPLATES = [];
    global VT_ID_COUNT; VT_ID_COUNT = 0;
    % 视觉模板裁剪/缩放（适配120x160图像）
    global VT_IMG_Y_RANGE; VT_IMG_Y_RANGE = 20:100;
    global VT_IMG_X_RANGE; VT_IMG_X_RANGE = 30:130;
    global VT_IMG_RESIZE_Y_RANGE; VT_IMG_RESIZE_Y_RANGE = 32;
    global VT_IMG_RESIZE_X_RANGE; VT_IMG_RESIZE_X_RANGE = 32;

    % HDC（偏航-高度细胞）相关
    global VT; VT = [];
    global HDC_CELLS; HDC_CELLS = zeros(36, 36);
    global HDC_LEARNING_RATE; HDC_LEARNING_RATE = 0.1;
    global HDC_THRESHOLD; HDC_THRESHOLD = 0.3;
    global YAW_HEIGHT_HDC; YAW_HEIGHT_HDC = zeros(36, 36);
    global YAW_HEIGHT_HDC_Y_DIM; YAW_HEIGHT_HDC_Y_DIM = 36;
    global YAW_HEIGHT_HDC_H_DIM; YAW_HEIGHT_HDC_H_DIM = 36;
    global YAW_HEIGHT_HDC_Y_TH_SIZE; YAW_HEIGHT_HDC_Y_TH_SIZE = 2*pi/36;
    global MAX_ACTIVE_YAW_HEIGHT_HIS_PATH; MAX_ACTIVE_YAW_HEIGHT_HIS_PATH = [];

    % 3D网格细胞相关
    global GRIDCELLS; GRIDCELLS = zeros(36, 36, 36);
    global GC_X_DIM; GC_X_DIM = 36;
    global GC_Y_DIM; GC_Y_DIM = 36;
    global GC_Z_DIM; GC_Z_DIM = 36;
    global GC_LEARNING_RATE; GC_LEARNING_RATE = 0.1;
    global GC_THRESHOLD; GC_THRESHOLD = 0.3;
    global gcX; gcX = 18;
    global gcY; gcY = 18;
    global gcZ; gcZ = 18;
    global MAX_ACTIVE_XYZ_PATH; MAX_ACTIVE_XYZ_PATH = [];

    % 经验图相关
    global EXPERIENCES; EXPERIENCES = [];
    global EXP_HISTORY; EXP_HISTORY = [];
    global NUM_EXPS; NUM_EXPS = 0;
    global EXP_CORRECTION; EXP_CORRECTION = [];
    global EXP_LOOPS; EXP_LOOPS = [];
    global DELTA_EM; DELTA_EM = 0;

    % 视觉里程计（VO）相关
    global ODO_IMG_YAW_ROT_Y_RANGE; ODO_IMG_YAW_ROT_Y_RANGE = 20:100;
    global ODO_IMG_YAW_ROT_X_RANGE; ODO_IMG_YAW_ROT_X_RANGE = 30:130;
    global ODO_IMG_HEIGHT_V_Y_RANGE; ODO_IMG_HEIGHT_V_Y_RANGE = 20:100;
    global ODO_IMG_HEIGHT_V_X_RANGE; ODO_IMG_HEIGHT_V_X_RANGE = 30:130;
    global ODO_IMG_TRANS_Y_RANGE; ODO_IMG_TRANS_Y_RANGE = 20:100;
    global ODO_IMG_TRANS_X_RANGE; ODO_IMG_TRANS_X_RANGE = 30:130;
    global ODO_IMG_YAW_ROT_RESIZE_RANGE; ODO_IMG_YAW_ROT_RESIZE_RANGE = [32 32];
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE; ODO_IMG_HEIGHT_V_RESIZE_RANGE = [64 64];
    global ODO_IMG_TRANS_RESIZE_RANGE; ODO_IMG_TRANS_RESIZE_RANGE = [64 64];
    global ODO_TRANS_V_SCALE; ODO_TRANS_V_SCALE = 0.1;
    global ODO_YAW_ROT_V_SCALE; ODO_YAW_ROT_V_SCALE = 0.1;
    global ODO_HEIGHT_V_SCALE; ODO_HEIGHT_V_SCALE = 0.1;
    global MAX_TRANS_V_THRESHOLD; MAX_TRANS_V_THRESHOLD = 5;
    global MAX_YAW_ROT_V_THRESHOLD; MAX_YAW_ROT_V_THRESHOLD = 10;
    global MAX_HEIGHT_V_THRESHOLD; MAX_HEIGHT_V_THRESHOLD = 5;
    global ODO_SHIFT_MATCH_VERT; ODO_SHIFT_MATCH_VERT = -5:5;
    global ODO_SHIFT_MATCH_HORI; ODO_SHIFT_MATCH_HORI = -5:5;
    global FOV_HORI_DEGREE; FOV_HORI_DEGREE = 90;
    global FOV_VERT_DEGREE; FOV_VERT_DEGREE = 60;
    global PREV_TRANS_V_IMG_X_SUMS; PREV_TRANS_V_IMG_X_SUMS = zeros(1,64);
    global PREV_YAW_ROT_V_IMG_X_SUMS; PREV_YAW_ROT_V_IMG_X_SUMS = zeros(1,64);
    global PREV_HEIGHT_V_IMG_Y_SUMS; PREV_HEIGHT_V_IMG_Y_SUMS = zeros(64,1);
    global PREV_TRANS_V; PREV_TRANS_V = 0;
    global PREV_YAW_ROT_V; PREV_YAW_ROT_V = 0;
    global PREV_HEIGHT_V; PREV_HEIGHT_V = 0;
    global OFFSET_YAW_ROT; OFFSET_YAW_ROT = 0;
    global OFFSET_HEIGHT_V; OFFSET_HEIGHT_V = 0;

    % 其他参数
    global BLOCK_READ; BLOCK_READ = 500;
    global RENDER_RATE; RENDER_RATE = 1;
    global GT_ODO_X_SCALING; GT_ODO_X_SCALING = 1;
    global GT_ODO_Y_SCALING; GT_ODO_Y_SCALING = 1;
    global GT_ODO_Z_SCALING; GT_ODO_Z_SCALING = 1;
    global GT_EXP_X_SCALING; GT_EXP_X_SCALING = 1;
    global GT_EXP_Y_SCALING; GT_EXP_Y_SCALING = 1;
    global GT_EXP_Z_SCALING; GT_EXP_Z_SCALING = 1;
    global ODO_MAP_X_SCALING; ODO_MAP_X_SCALING = 1;
    global ODO_MAP_Y_SCALING; ODO_MAP_Y_SCALING = 1;
    global ODO_MAP_Z_SCALING; ODO_MAP_Z_SCALING = 1;
    global EXP_MAP_X_SCALING; EXP_MAP_X_SCALING = 1;
    global EXP_MAP_Y_SCALING; EXP_MAP_Y_SCALING = 1;
    global EXP_MAP_Z_SCALING; EXP_MAP_Z_SCALING = 1;
    global KEY_POINT_SET; KEY_POINT_SET = [];
    global ODO_STEP; ODO_STEP = 1;
    global DEGREE_TO_RADIAN; DEGREE_TO_RADIAN = pi / 180;
    global RADIAN_TO_DEGREE; RADIAN_TO_DEGREE = 180 / pi;  
    disp('[2/12] 全局变量初始化完成！');

    %% 3. 处理输入参数（支持可视化开关）
    disp('[3/12] 开始处理输入参数...');
    visualizate = false;  % 默认关闭可视化
    for i=1:(nargin - 8)
        if ischar(varargin{i})
            switch varargin{i}
                case 'BLOCK_READ', BLOCK_READ = varargin{i+1}; disp(['[3/12] BLOCK_READ设置为：', num2str(BLOCK_READ)]);
                case 'RENDER_RATE', RENDER_RATE = varargin{i+1}; disp(['[3/12] RENDER_RATE设置为：', num2str(RENDER_RATE)]);
                case 'VISUALIZE', visualizate = varargin{i+1}; disp(['[3/12] 可视化开关：', num2str(visualizate)]);
                case 'GT_ODO_X_SCALING', GT_ODO_X_SCALING = varargin{i+1};
                case 'GT_ODO_Y_SCALING', GT_ODO_Y_SCALING = varargin{i+1};
                case 'GT_ODO_Z_SCALING', GT_ODO_Z_SCALING = varargin{i+1};
                case 'GT_EXP_X_SCALING', GT_EXP_X_SCALING = varargin{i+1};
                case 'GT_EXP_Y_SCALING', GT_EXP_Y_SCALING = varargin{i+1};
                case 'GT_EXP_Z_SCALING', GT_EXP_Z_SCALING = varargin{i+1};
                case 'ODO_MAP_X_SCALING', ODO_MAP_X_SCALING = varargin{i+1};
                case 'ODO_MAP_Y_SCALING', ODO_MAP_Y_SCALING = varargin{i+1};
                case 'ODO_MAP_Z_SCALING', ODO_MAP_Z_SCALING = varargin{i+1};
                case 'EXP_MAP_X_SCALING', EXP_MAP_X_SCALING = varargin{i+1};
                case 'EXP_MAP_Y_SCALING', EXP_MAP_Y_SCALING = varargin{i+1};
                case 'EXP_MAP_Z_SCALING', EXP_MAP_Z_SCALING = varargin{i+1};
            end
        end
    end
    disp('[3/12] 输入参数处理完成！');

    %% 4. 初始化HDC和3D网格细胞
    disp('[4/12] 初始化HDC模块...');
    [curYawTheta, curHeightValue] = get_hdc_initial_value();
    disp(['[4/12] HDC初始化结果：curYawTheta=', num2str(curYawTheta), ', curHeightValue=', num2str(curHeightValue)]);

    disp('[5/12] 初始化3D网格细胞模块...');
    [gcX, gcY, gcZ] = get_gc_initial_pos();
    disp(['[5/12] 网格细胞初始位置：gcX=', num2str(gcX), ', gcY=', num2str(gcY), ', gcZ=', num2str(gcZ)]);

    %% 6. 读取视觉数据（手动指定路径，兼容自动解析）
    disp('[6/12] 读取视觉数据文件夹...');
    subFoldersPathSet = {
        %'/home/dream/Neuro_WS/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town01Data',
        '/home/dream/Neuro_WS/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town10Data'
    };
    % 验证路径有效性
    numSubFolders = length(subFoldersPathSet);
    for i = 1:numSubFolders
        if ~exist(subFoldersPathSet{i}, 'dir')
            warning('子文件夹不存在：%s，尝试自动解析', subFoldersPathSet{i});
            [subFoldersPathSet, numSubFolders] = get_images_data_info(visualDataFile);
            break;
        end
    end
    disp(['[6/12] 共识别 ', num2str(numSubFolders), ' 个子文件夹，准备处理图像！']);

    %% 7. 初始化绘图和轨迹变量
    disp('[7/12] 初始化绘图和轨迹变量...');
    odoHeightValue = [0 0 0];
    Theta = [0 0 0 0 0 0];
    Rho = [0 0 0 0 0 0];
    startpoint =[0 0];
    endpoint = [0.8 0.8];
    odoYawTheta = [0 0 0];
    hdcYawTheta = [0 0 0];
    hdcYawTheta(1,3)= 0;
    zAxisHDC = 1:2:36;
    expTrajectory(1,1) = 0;
    expTrajectory(1,2) = 0;
    expTrajectory(1,3) = 0;
    expTrajectory(1,4) = 0;
    odoMapTrajectory(1,1) = 0;
    odoMapTrajectory(1,2) = 0;
    odoMapTrajectory(1,3) = 0;
    odoMapTrajectory(1,4) = 0;
    gtHasValue = 0;
    curFrame = 0;
    vtcurFrame = 1;
    preImg = 0;
    % 初始化经验图节点连接（矩阵类型，兼容旧脚本）
    EXP_NODES_LINKS.nodes(1) = 1;
    EXP_NODES_LINKS.numlinks(1) = 0;
    EXP_NODES_LINKS.linknodes = zeros(1,1);  % 明确为矩阵
    EXP_NODES_LINKS.linknodes(1,1) = 0;
    global DELTA_EM; DELTA_EM = 0;
    disp('[7/12] 绘图和轨迹变量初始化完成！');

    %% 8. 加载真值数据（可选）
    disp('[8/12] 加载真值数据...');
    if ~isempty(groundTruthFile) && exist(groundTruthFile, 'file')
        [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);
        gtHasValue = 1;
        disp(['[8/12] 真值数据加载成功，共 ', num2str(length(frameId)), ' 帧']);
    else
        disp('[8/12] 未找到真值文件，跳过加载');
    end

    %% 9. 主处理循环（子文件夹+图像帧）
    disp('[9/12] 开始主处理循环...');
    startFrame = 1;
    endFrame = 956;  % 可改为numImgs处理全部帧
    for iSubFolder = 1 : numSubFolders
        disp(['='*30]);
        disp(['[9/12] 处理第 ', num2str(iSubFolder), '/', num2str(numSubFolders), ' 个子文件夹']);
        
        [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, IMG_TYPE, iSubFolder);
        disp(['[9/12] 子文件夹路径：', curFolderPath]);
        disp(['[9/12] 子文件夹包含 ', num2str(numImgs), ' 张图像']);
        
        if numImgs > 0
            for indFrame = startFrame : ODO_STEP : min(numImgs-1, endFrame)
                disp(['[10/12] 处理第 ', num2str(indFrame), '/', num2str(numImgs), ' 帧']);
                
                % 读取并预处理图像
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, indFrame);
                curGrayImg = rgb2gray(curImg);
                curGrayImg = im2double(curGrayImg);
                disp(['[10/12] 图像预处理完成，尺寸：', num2str(size(curGrayImg))]);

                %% 10. 视觉里程计（VO）计算
                disp('[10/12] 调用visual_odometry计算里程计...');
                if length(KEY_POINT_SET) == 2
                    if indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2)
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                    transV = 2;
                else
                    if isempty(KEY_POINT_SET) || indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2) 
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(3)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(4)
                        [transV, yawRotV, heightV] = visual_odometry_down(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                end
                yawRotV = yawRotV * DEGREE_TO_RADIAN;
                disp(['[10/12] 里程计结果：transV=', num2str(transV), ', yawRotV=', num2str(yawRotV), ', heightV=', num2str(heightV)]);

                %% 11. 视觉模板（VT）处理
                disp('[11/12] 处理视觉模板...');
                curFrame = curFrame + 1;
                if VT_STEP == 1
                    vtcurGrayImg = curGrayImg; 
                else
                    if mod(curFrame, VT_STEP) == 1 
                        vtcurGrayImg = curGrayImg; 
                        preImg = vtcurGrayImg;
                    else
                        vtcurGrayImg = preImg;
                    end
                end
                [vt_id] = visual_template(vtcurGrayImg, gcX, gcY, gcZ, curYawTheta, curHeightValue);
                disp(['[11/12] 视觉模板ID：vt_id=', num2str(vt_id)]);

                %% 12. HDC迭代
                disp('[12/12] HDC模块迭代...');
                yaw_height_hdc_iteration(vt_id, yawRotV, heightV);
                [curYawTheta, curHeightValue] = get_current_yaw_height_value();
                curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
                MAX_ACTIVE_YAW_HEIGHT_HIS_PATH = [MAX_ACTIVE_YAW_HEIGHT_HIS_PATH; curYawTheta curHeightValue];
                disp(['[12/12] HDC更新结果：yaw=', num2str(curYawThetaInRadian), ', height=', num2str(curHeightValue)]);

                %% 13. 3D网格细胞迭代
                disp('[13/12] 3D网格细胞迭代...');
                gc_iteration(vt_id, transV, curYawThetaInRadian, heightV);
                [gcX, gcY, gcZ] = get_gc_xyz();
                MAX_ACTIVE_XYZ_PATH = [MAX_ACTIVE_XYZ_PATH; gcX gcY gcZ];  
                disp(['[13/12] 网格细胞位置：(', num2str(gcX), ',', num2str(gcY), ',', num2str(gcZ), ')']);

                %% 14. 经验图迭代
                disp('[14/12] 经验图迭代...');
                exp_map_iteration(vt_id, transV, yawRotV, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);
                disp(['[14/12] 经验图当前节点数：', num2str(NUM_EXPS)]);

                %% 15. 更新轨迹数据（修复版：矩阵存储连接关系）
                disp('[15/12] 更新轨迹数据...');
                % 里程计轨迹
                odoYawTheta(curFrame + 1, 1) = cos(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 2) = sin(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 3) = odoYawTheta(curFrame, 3) + yawRotV;

                odoMapTrajectory(curFrame + 1,1) = odoMapTrajectory(curFrame,1) + transV * cos(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,2) = odoMapTrajectory(curFrame,2) + transV * sin(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,3) = odoMapTrajectory(curFrame,3) + heightV;
                odoMapTrajectory(curFrame + 1,4) = odoYawTheta(curFrame + 1, 3);

                % HDC偏航角轨迹
                hdcYawTheta(curFrame, 1) = cos(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 2) = sin(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 3) = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;

                % 经验图轨迹（使用实际字段gcX/gcY/gcZ/yaw）
                for ind = 1 : NUM_EXPS 
                    expTrajectory(ind,1) = EXPERIENCES(ind).gcX;
                    expTrajectory(ind,2) = EXPERIENCES(ind).gcY;
                    expTrajectory(ind,3) = EXPERIENCES(ind).gcZ;
                    expTrajectory(ind,4) = EXPERIENCES(ind).yaw;
                end

                % 经验图节点连接关系（矩阵存储，无类型错误）
                for ind_exps = 1 : NUM_EXPS
                    % 节点ID
                    EXP_NODES_LINKS.nodes(ind_exps) = ind_exps;
                    
                    % 连接数量：强制设为0（兼容无numlinks字段）
                    EXP_NODES_LINKS.numlinks(ind_exps) = 0;
                    
                    % 连接节点：矩阵动态扩展，无连接填0
                    if isfield(EXPERIENCES(ind_exps), 'links') && ~isempty(EXPERIENCES(ind_exps).links)
                        link_count = length(EXPERIENCES(ind_exps).links);
                        % 扩展矩阵列数避免索引错误
                        if link_count > size(EXP_NODES_LINKS.linknodes, 2)
                            add_cols = link_count - size(EXP_NODES_LINKS.linknodes, 2);
                            EXP_NODES_LINKS.linknodes = [EXP_NODES_LINKS.linknodes, zeros(size(EXP_NODES_LINKS.linknodes, 1), add_cols)];
                        end
                        % 填充连接ID
                        for link_id = 1 : link_count
                            if isfield(EXPERIENCES(ind_exps).links(link_id), 'exp_id')
                                EXP_NODES_LINKS.linknodes(ind_exps, link_id) = EXPERIENCES(ind_exps).links(link_id).exp_id;
                            else
                                EXP_NODES_LINKS.linknodes(ind_exps, link_id) = 0;
                            end
                        end
                    else
                        % 无连接时填0
                        if size(EXP_NODES_LINKS.linknodes, 1) < ind_exps
                            % 扩展矩阵行数
                            EXP_NODES_LINKS.linknodes(ind_exps, :) = 0;
                        else
                            EXP_NODES_LINKS.linknodes(ind_exps, :) = 0;
                        end
                    end
                end

                %% 16. 可视化（按需开启）
                if visualizate && mod(curFrame, RENDER_RATE) == 1
                    disp('[16/12] 绘制可视化结果...');
                    figure('Position', [100, 100, 1200, 800]);
                    
                    % 1. 3D网格细胞活性图
                    subplot(2,3,1);
                    gridSlice = GRIDCELLS(:,:,gcZ);
                    imagesc(gridSlice); colormap(jet); colorbar;
                    hold on; plot(gcY, gcX, 'ro', 'MarkerSize', 5); hold off;
                    xlabel('GC Y'); ylabel('GC X'); title(['3D网格细胞活性（Z=', num2str(gcZ), '）']);
                    axis equal;
                    
                    % 2. HDC热力图
                    subplot(2,3,2);
                    imagesc(HDC_CELLS); colormap(jet); colorbar;
                    hold on; plot(curHeightValue, curYawTheta, 'ro', 'MarkerSize', 5); hold off;
                    xlabel('Height Index'); ylabel('Yaw Index'); title('HDC偏航-高度活性热力图');
                    axis equal;
                    
                    % 3. 经验图3D轨迹
                    subplot(2,3,3);
                    if NUM_EXPS > 1
                        plot3(expTrajectory(:,1), expTrajectory(:,2), expTrajectory(:,3), 'b-o', 'LineWidth', 1);
                    else
                        plot3(expTrajectory(:,1), expTrajectory(:,2), expTrajectory(:,3), 'bo', 'MarkerSize', 5);
                    end
                    xlabel('GC X'); ylabel('GC Y'); zlabel('GC Z'); title('经验图3D轨迹'); grid on;
                    
                    % 4. 里程计2D轨迹
                    subplot(2,3,4);
                    if curFrame > 1
                        plot(odoMapTrajectory(:,1), odoMapTrajectory(:,2), 'r-', 'LineWidth', 1.5);
                    else
                        plot(odoMapTrajectory(:,1), odoMapTrajectory(:,2), 'ro', 'MarkerSize', 5);
                    end
                    xlabel('X Position'); ylabel('Y Position'); title('里程计轨迹（X-Y平面）'); grid on;
                    
                    % 5. 原始图像
                    subplot(2,3,5);
                    imshow(curImg); title(['当前帧：', num2str(indFrame), '（模板ID=', num2str(vt_id), '）']);
                    
                    % 6. 视觉模板ID历史
                    subplot(2,3,6);
                    plot(VT_HISTORY, 'g-', 'LineWidth', 1.2);
                    xlabel('Frame'); ylabel('Visual Template ID'); title('视觉模板ID变化'); grid on;
                    
                    drawnow;
                    disp('[16/12] 可视化完成');
                end

                disp(['[10/12] 第 ', num2str(indFrame), ' 帧处理结束']);
            end
        else
            disp('[9/12] 当前子文件夹无图像，跳过');
        end
    end

    %% 17. 自动保存结果
    %% 17. 自动保存结果（修改后：添加GRIDCELLS和HDC_CELLS）
    disp('[17/12] 开始保存结果...');
    if ~isempty(expMapHistoryFile)
        save(expMapHistoryFile, 'expTrajectory');
        disp(['[17/12] 经验图轨迹保存至：', expMapHistoryFile]);
    end
    if ~isempty(odoMapHistoryFile)
        save(odoMapHistoryFile, 'odoMapTrajectory');
        disp(['[17/12] 里程计轨迹保存至：', odoMapHistoryFile]);
    end
    if ~isempty(vtHistoryFile)
        save(vtHistoryFile, 'VT_HISTORY');
        disp(['[17/12] 视觉模板历史保存至：', vtHistoryFile]);
    end
    if ~isempty(emHistoryFile)
    save(emHistoryFile, 'EXP_HISTORY', 'NUM_EXPS');  % 同时保存NUM_EXPS
    disp(['[17/12] 经验图历史保存至：', emHistoryFile]);
    end
    if ~isempty(gcTrajFile) && ~isempty(hdcTrajFile)
        % 保存GRIDCELLS到gc_trajectory.mat
        save(gcTrajFile, 'MAX_ACTIVE_XYZ_PATH', 'GRIDCELLS');
        % 保存HDC_CELLS到hdc_trajectory.mat
        save(hdcTrajFile, 'MAX_ACTIVE_YAW_HEIGHT_HIS_PATH', 'HDC_CELLS', 'curYawTheta', 'curHeightValue');
        disp(['[17/12] 网格细胞和HDC轨迹保存完成']);
    end
