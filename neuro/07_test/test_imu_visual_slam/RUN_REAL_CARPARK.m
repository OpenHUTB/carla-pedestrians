%% 运行QUT Carpark真实场景SLAM测试
% 一键启动真实停车场场景的SLAM测试

clear all; close all; clc;

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║   QUT Carpark真实场景SLAM测试                  ║\n');
fprintf('║   HART+Transformer (Plan B配置)                ║\n');
fprintf('║   纯视觉SLAM（无IMU）                          ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

fprintf('📍 数据集: QUT Carpark (真实停车场场景)\n');
fprintf('📊 图像数量: 10935张\n');
fprintf('🔧 特征提取: HART+Transformer\n');
fprintf('⏱️  预计时间: 15-20分钟\n\n');

% 运行测试
test_real_carpark_slam;

fprintf('\n✅ QUT Carpark SLAM测试完成！\n');
fprintf('📁 结果保存在: DATASETS/01_NeuroSLAM_Datasets/03_QUTCarparkData/slam_results/\n\n');
