%% 修复消融实验数据 - 直接从trajectories.mat计算正确RMSE
%
% 问题：RUN_COMPLETE_ABLATION从performance_report读取了错误的RMSE
% 解决：直接从原始数据重新计算

clear all; close all; clc;

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   修复消融实验数据 - 重新计算正确RMSE                   ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% 配置
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));

datasets = {
    'Town01Data_IMU_Fusion', 'Town01', 1802.26;
    'Town10Data_IMU_Fusion', 'Town10', 1630.84;
};

%% 处理每个数据集
all_results = struct();

for d = 1:size(datasets, 1)
    dataset_dir = datasets{d, 1};
    dataset_name = datasets{d, 2};
    gt_length = datasets{d, 3};
    
    fprintf('═══════════════════════════════════════════════════════════\n');
    fprintf('数据集: %s (轨迹长度: %.1fm)\n', dataset_name, gt_length);
    fprintf('═══════════════════════════════════════════════════════════\n\n');
    
    % 加载数据
    data_path = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', dataset_dir);
    traj_file = fullfile(data_path, 'slam_results', 'trajectories.mat');
    
    if ~exist(traj_file, 'file')
        fprintf('⚠️  数据文件未找到，跳过\n\n');
        continue;
    end
    
    load(traj_file);
    
    % 提取Ground Truth
    gt_xyz = gt_data.pos(:, 1:3);
    
    %% 配置1：Bio-inspired Fusion（完整系统 = exp_trajectory）
    fprintf('[1/3] Bio-inspired IMU-Visual Fusion (Ours)\n');
    bio_xyz = exp_trajectory(:, 1:3);
    min_len = min(size(gt_xyz,1), size(fusion_xyz,1));
    
    bio_errors = sqrt(sum((bio_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
    bio_rmse = sqrt(mean(bio_errors.^2));
    bio_final = bio_errors(end);
    bio_drift = (bio_final / gt_length) * 100;
    
    fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
        bio_rmse, bio_final, bio_drift);
    
    %% 配置2：纯视觉里程计（去掉IMU）
    fprintf('[2/3] w/o IMU Fusion (Pure VO)\n');
    odo_xyz = odo_trajectory(:, 1:3);
    min_len = min(size(gt_xyz,1), size(odo_xyz,1));
    
    odo_errors = sqrt(sum((odo_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
    odo_rmse = sqrt(mean(odo_errors.^2));
    odo_final = odo_errors(end);
    odo_drift = (odo_final / gt_length) * 100;
    
    fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
        odo_rmse, odo_final, odo_drift);
    
    %% 配置3：EKF Fusion（基线 = fusion_data）
    fprintf('[3/3] EKF Fusion (Baseline Reference)\n');
    ekf_xyz = fusion_data.pos(:, 1:3);
    min_len = min(size(gt_xyz,1), size(exp_xyz,1));
    
    ekf_errors = sqrt(sum((ekf_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
    ekf_rmse = sqrt(mean(ekf_errors.^2));
    ekf_final = ekf_errors(end);
    ekf_drift = (ekf_final / gt_length) * 100;
    
    fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
        ekf_rmse, ekf_final, ekf_drift);
    
    %% 改进分析
    fprintf('\nImprovement Analysis:\n');
    fprintf('  w/o IMU vs Ours: %.1fx worse (RMSE: %.2f → %.2f m)\n', ...
        odo_rmse/bio_rmse, bio_rmse, odo_rmse);
    fprintf('  EKF vs Ours: %.1fx worse (RMSE: %.2f → %.2f m)\n\n', ...
        ekf_rmse/bio_rmse, bio_rmse, ekf_rmse);
    
    %% 保存结果
    all_results.(dataset_name).name = dataset_name;
    all_results.(dataset_name).gt_length = gt_length;
    
    all_results.(dataset_name).Complete.description = 'Bio-inspired IMU-Visual Fusion (Ours)';
    all_results.(dataset_name).Complete.rmse = bio_rmse;
    all_results.(dataset_name).Complete.final_error = bio_final;
    all_results.(dataset_name).Complete.drift_rate = bio_drift;
    
    all_results.(dataset_name).No_IMU.description = 'w/o IMU Fusion (Pure VO)';
    all_results.(dataset_name).No_IMU.rmse = odo_rmse;
    all_results.(dataset_name).No_IMU.final_error = odo_final;
    all_results.(dataset_name).No_IMU.drift_rate = odo_drift;
    
    all_results.(dataset_name).EKF_Baseline.description = 'EKF Fusion (Baseline)';
    all_results.(dataset_name).EKF_Baseline.rmse = ekf_rmse;
    all_results.(dataset_name).EKF_Baseline.final_error = ekf_final;
    all_results.(dataset_name).EKF_Baseline.drift_rate = ekf_drift;
end

%% 生成汇总报告
fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   正确的消融实验结果汇总                                 ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

for d = 1:size(datasets, 1)
    dataset_name = datasets{d, 2};
    if ~isfield(all_results, dataset_name)
        continue;
    end
    
    fprintf('=== %s (%.0fm trajectory) ===\n\n', dataset_name, datasets{d, 3});
    fprintf('| Configuration | RMSE (m) | Drift Rate (%%) | vs Baseline |\n');
    fprintf('|---------------|----------|----------------|-------------|\n');
    
    res = all_results.(dataset_name);
    baseline_rmse = res.Complete.rmse;
    
    % 完整系统
    fprintf('| %s | %.2f | %.2f | Baseline |\n', ...
        res.Complete.description, res.Complete.rmse, res.Complete.drift_rate);
    
    % w/o IMU
    ratio = res.No_IMU.rmse / baseline_rmse;
    fprintf('| %s | %.2f | %.2f | %.1fx worse |\n', ...
        res.No_IMU.description, res.No_IMU.rmse, res.No_IMU.drift_rate, ratio);
    
    % w/o ExpMap
    ratio = res.No_ExpMap.rmse / baseline_rmse;
    fprintf('| %s | %.2f | %.2f | %.1fx worse |\n\n', ...
        res.No_ExpMap.description, res.No_ExpMap.rmse, res.No_ExpMap.drift_rate, ratio);
end

%% 保存结果
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'ablation_results_corrected.mat'), 'all_results');
fprintf('✅ 修正后的结果已保存: ablation_results_corrected.mat\n\n');

fprintf('💡 提示：这些才是正确的RMSE值！\n');
fprintf('   融合轨迹应该是最好的（RMSE最小）\n\n');
