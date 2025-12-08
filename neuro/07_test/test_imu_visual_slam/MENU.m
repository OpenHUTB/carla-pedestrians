%% SLAM系统测试菜单
% 快捷启动各种测试

clc;
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║         HART+Transformer SLAM 测试系统                    ║\n');
fprintf('║              快捷启动菜单                                 ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('📋 SLAM完整测试（仿真场景）：\n');
fprintf('  [1] RUN_SLAM_TOWN01    - Town01 SLAM测试\n');
fprintf('  [2] RUN_SLAM_TOWN10    - Town10 SLAM测试\n');
fprintf('\n');

fprintf('🏢 SLAM真实场景测试：\n');
fprintf('  [3] RUN_REAL_CARPARK   - QUT Carpark停车场（10935帧）\n');
fprintf('\n');

fprintf('🔬 消融实验：\n');
fprintf('  [4] RUN_ABLATION_TOWN01 - Town01 消融实验\n');
fprintf('  [5] RUN_ABLATION_TOWN10 - Town10 消融实验\n');
fprintf('\n');

fprintf('⚡ 其他：\n');
fprintf('  [6] RUN_FAST_TEST      - 快速测试（500帧）\n');
fprintf('\n');

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('💡 使用方法：直接在命令窗口输入脚本名称，例如：RUN_SLAM_TOWN01\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('📊 输出位置：\n');
fprintf('  Town01: data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/\n');
fprintf('  Town10: data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/\n');
fprintf('  Carpark: DATASETS/01_NeuroSLAM_Datasets/03_QUTCarparkData/\n');
fprintf('\n');
