
% add all function path
addpath(genpath('.'))

% 数据目录
global data_dir;
data_dir = 'D:\work\workspace\carla-pedestrians\neuro\data';


%% 测试3D建图 test_3d_mapping

% CarlaData：Carla 的 Town10 场景中
test_mapping_CarlaData

% QUTCarparkData：QUT 停车场数据
% 480 x 270
test_mapping_QUTCarparkData

% 全景(Panoramic)相机：SynPanData
test_mapping_SynPanData

% 透视(Perspective)相机：SynPerData
test_mapping_SynPerData


%% 3D 视觉里程计 aidvo ：test_aidvo
% QUTCarparkData
test_vo_QUTCarparkData

% SynPanData
test_vo_ov_SynPanData

% 在透视相机上测试视觉里程计 SynPerData
test_vo_SynPerData


%% 测试视觉模板 test_vt
test_vt_SynPanData


