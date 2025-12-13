%% 从现有测试结果提取真实消融实验数据
%
% 本脚本从Town01的完整测试结果中提取真实的RMSE数据
% 包括：融合轨迹、纯视觉、经验地图
%
% 这些才是真实的消融实验数据（不是硬编码估算）

clear all; close all; clc;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   提取真实消融实验数据                                   ║\n');
fprintf('║   从Town01完整测试结果中获取                             ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 加载Town01测试结果
script_dir = fileparts(mfilename('fullpath'));
% 从ablation目录回到neuro根目录
neuro_root = fileparts(fileparts(fileparts(script_dir)));
data_file = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'Town01Data_IMU_Fusion', 'slam_results', 'trajectories.mat');

if ~exist(data_file, 'file')
    error('❌ 找不到测试结果文件: %s\n请先运行 RUN_SLAM_TOWN01', data_file);
end

fprintf('📁 加载数据: %s\n', data_file);
load(data_file);

%% 计算各轨迹的RMSE

% 提取坐标数据
% gt_data和fusion_data是结构体，需要访问.pos字段
% odo_trajectory和exp_trajectory是直接矩阵
gt_xyz = gt_data.pos(:, 1:3);      % pos字段包含[x, y, z]
fusion_xyz = fusion_data.pos(:, 1:3);
odo_xyz = odo_trajectory(:, 1:3);  % 直接是[x, y, z]
exp_xyz = exp_trajectory(:, 1:3);

% 确保长度一致
min_len = min([size(gt_xyz,1), size(fusion_xyz,1), size(odo_xyz,1), size(exp_xyz,1)]);
gt_xyz = gt_xyz(1:min_len, :);
fusion_xyz = fusion_xyz(1:min_len, :);
odo_xyz = odo_xyz(1:min_len, :);
exp_xyz = exp_xyz(1:min_len, :);

% Ground Truth轨迹长度
traj_length = sum(sqrt(sum(diff(gt_xyz).^2, 2)));
fprintf('✓ Ground Truth轨迹长度: %.1f米 (使用%d帧)\n', traj_length, min_len);

%% 1. 完整系统（融合轨迹）
fprintf('\n【1/3】完整系统 - IMU-Visual融合轨迹\n');
fusion_errors = sqrt(sum((fusion_xyz - gt_xyz).^2, 2));
fusion_rmse = sqrt(mean(fusion_errors.^2));
fusion_mean = mean(fusion_errors);
fusion_final = fusion_errors(end);
fusion_drift = (fusion_rmse / traj_length) * 100;

fprintf('  RMSE: %.2f m\n', fusion_rmse);
fprintf('  平均误差: %.2f m\n', fusion_mean);
fprintf('  终点误差: %.2f m\n', fusion_final);
fprintf('  漂移率: %.2f%%\n', fusion_drift);

%% 2. 去掉IMU（纯视觉里程计）
fprintf('\n【2/3】去掉IMU - 纯视觉里程计\n');
odo_errors = sqrt(sum((odo_xyz - gt_xyz).^2, 2));
odo_rmse = sqrt(mean(odo_errors.^2));
odo_mean = mean(odo_errors);
odo_final = odo_errors(end);
odo_drift = (odo_rmse / traj_length) * 100;

fprintf('  RMSE: %.2f m\n', odo_rmse);
fprintf('  平均误差: %.2f m\n', odo_mean);
fprintf('  终点误差: %.2f m\n', odo_final);
fprintf('  漂移率: %.2f%%\n', odo_drift);

%% 3. 经验地图（对比基准）
fprintf('\n【3/3】经验地图轨迹（对比）\n');
exp_errors = sqrt(sum((exp_xyz - gt_xyz).^2, 2));
exp_rmse = sqrt(mean(exp_errors.^2));
exp_mean = mean(exp_errors);
exp_final = exp_errors(end);
exp_drift = (exp_rmse / traj_length) * 100;

fprintf('  RMSE: %.2f m\n', exp_rmse);
fprintf('  平均误差: %.2f m\n', exp_mean);
fprintf('  终点误差: %.2f m\n', exp_final);
fprintf('  漂移率: %.2f%%\n', exp_drift);

%% 生成对比表格
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   真实消融实验结果（Town01, 1802m轨迹）                  ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('| 配置 | RMSE (m) | 漂移率 (%%) | vs Baseline |\n');
fprintf('|------|----------|------------|-------------|\n');
fprintf('| 完整系统（融合） | %.2f | %.2f | Baseline |\n', fusion_rmse, fusion_drift);
fprintf('| 去掉IMU（视觉） | %.2f | %.2f | +%.0f%% (%.1f倍) |\n', ...
    odo_rmse, odo_drift, ...
    ((odo_rmse - fusion_rmse) / fusion_rmse) * 100, ...
    odo_rmse / fusion_rmse);
fprintf('| 经验地图（对比） | %.2f | %.2f | +%.0f%% (%.1f倍) |\n', ...
    exp_rmse, exp_drift, ...
    ((exp_rmse - fusion_rmse) / fusion_rmse) * 100, ...
    exp_rmse / fusion_rmse);

%% 关键发现
fprintf('\n═══════════════════════════════════════════════════════════\n');
fprintf('关键发现:\n');
fprintf('═══════════════════════════════════════════════════════════\n');

fprintf('\n1. IMU融合的作用:\n');
improvement_vs_visual = odo_rmse / fusion_rmse;
fprintf('   - 融合 vs 纯视觉: %.1f倍改进 🚀\n', improvement_vs_visual);
fprintf('   - RMSE: %.2fm → %.2fm\n', odo_rmse, fusion_rmse);
fprintf('   - 这证明了IMU-视觉融合的关键作用\n');

fprintf('\n2. 融合 vs 经验地图:\n');
improvement_vs_exp = exp_rmse / fusion_rmse;
fprintf('   - 融合比经验地图好: %.1f倍 ✅\n', improvement_vs_exp);
fprintf('   - 融合轨迹是最准确的估计\n');

fprintf('\n3. 论文应该用的数据:\n');
fprintf('   - Baseline（完整系统）: %.2fm\n', fusion_rmse);
fprintf('   - 去掉IMU（主要消融）: %.2fm (+%.0f%%)\n', ...
    odo_rmse, ((odo_rmse - fusion_rmse) / fusion_rmse) * 100);

%% 保存结果
results = struct();
results.config_names = {'Full_System', 'No_IMU', 'Experience_Map'};
results.descriptions = {'完整系统（融合）', '去掉IMU（纯视觉）', '经验地图（对比）'};
results.rmse_values = [fusion_rmse, odo_rmse, exp_rmse];
results.mean_errors = [fusion_mean, odo_mean, exp_mean];
results.final_errors = [fusion_final, odo_final, exp_final];
results.drift_rates = [fusion_drift, odo_drift, exp_drift];

% 从全局变量获取VT和节点数（如果可用）
try
    global VT_ID_COUNT NUM_EXPS;
    results.vt_count = VT_ID_COUNT;
    results.exp_count = NUM_EXPS;
    fprintf('\n   VT数量: %d, 经验节点: %d\n', VT_ID_COUNT, NUM_EXPS);
catch
    fprintf('\n   (VT和节点数未加载，请运行RUN_SLAM_TOWN01获取)\n');
end

results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'Town01Data_IMU_Fusion', 'ablation_results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'real_ablation_results_extracted.mat'), 'results');

% 生成CSV
csvfile = fullfile(results_dir, 'real_ablation_results_extracted.csv');
fid = fopen(csvfile, 'w');
fprintf(fid, 'Configuration,Description,RMSE_m,Mean_Error_m,Final_Error_m,Drift_Rate_percent\n');
for i = 1:length(results.config_names)
    fprintf(fid, '%s,%s,%.2f,%.2f,%.2f,%.2f\n', ...
        results.config_names{i}, ...
        results.descriptions{i}, ...
        results.rmse_values(i), ...
        results.mean_errors(i), ...
        results.final_errors(i), ...
        results.drift_rates(i));
end
fclose(fid);

fprintf('\n✅ 结果已保存:\n');
fprintf('   MAT: %s\n', fullfile(results_dir, 'real_ablation_results_extracted.mat'));
fprintf('   CSV: %s\n', csvfile);

fprintf('\n🎉 真实数据提取完成！\n');
fprintf('💡 这些数据可以直接用于论文中的消融实验部分\n\n');
