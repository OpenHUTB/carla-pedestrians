%% 纯消融实验（Ablation Study Only）
%
% 测试系统组件贡献（3配置）：
%   1. Complete System (Ours) - exp_trajectory
%   2. w/o Experience Map - imu_aided_traj
%   3. w/o IMU Fusion - pure_visual_traj
%
% ✅ 关键改进：在计算RMSE前先对齐轨迹（Procrustes）

clear all; close all; clc;

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   纯消融实验 - Town01 & MH_03                            ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% 配置
script_dir = fileparts(mfilename('fullpath'));
neuro_root = fileparts(fileparts(fileparts(script_dir)));

datasets = {
    'Town01Data_IMU_Fusion', 'Town01', 'Town', 1802.26;
    'MH_03_medium', 'MH_03', 'EuRoC', 127.07;
};

%% 处理每个数据集
all_results = struct();

for d = 1:size(datasets, 1)
    dataset_dir = datasets{d, 1};
    dataset_name = datasets{d, 2};
    dataset_type = datasets{d, 3};
    gt_length = datasets{d, 4};
    
    fprintf('═══════════════════════════════════════════════════════════\n');
    fprintf('Dataset: %s (Trajectory Length: %.1fm)\n', dataset_name, gt_length);
    fprintf('═══════════════════════════════════════════════════════════\n\n');
    
    % 加载数据
    if strcmp(dataset_type, 'Town')
        data_path = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', dataset_dir);
        traj_file = fullfile(data_path, 'slam_results', 'trajectories.mat');
    else  % EuRoC
        data_path = fullfile(neuro_root, 'data', '02_EuRoc_Dataset', dataset_dir);
        traj_file = fullfile(data_path, 'slam_results', 'euroc_trajectories.mat');
    end
    
    if ~exist(traj_file, 'file')
        fprintf('⚠️  数据文件未找到，跳过\n\n');
        continue;
    end
    
    load(traj_file);
    
    % 提取Ground Truth
    gt_xyz = gt_data.pos(:, 1:3);
    
    fprintf('════ 消融实验：组件贡献分析 ════\n\n');
    
    %% 配置1：Complete System (Ours)
    fprintf('[1/3] Complete System (Ours)\n');
    [exp_rmse, exp_final, exp_drift, exp_xyz_aligned] = ...
        compute_metrics_with_alignment(exp_trajectory(:,1:3), gt_xyz, gt_length);
    fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
        exp_rmse, exp_final, exp_drift);
    
    %% 配置2：w/o Experience Map
    fprintf('[2/3] w/o Experience Map (IMU-aided VO)\n');
    if exist('imu_aided_traj', 'var')
        [imu_rmse, imu_final, imu_drift, imu_xyz_aligned] = ...
            compute_metrics_with_alignment(imu_aided_traj(:,1:3), gt_xyz, gt_length);
        fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n', ...
            imu_rmse, imu_final, imu_drift);
    else
        fprintf('  ⚠️  未找到imu_aided_traj\n');
        imu_rmse = NaN;
        imu_final = NaN;
        imu_drift = NaN;
    end
    
    %% 配置3：w/o IMU Fusion (Pure Visual)
    fprintf('[3/3] w/o IMU Fusion (Pure Visual VO)\n');
    if exist('pure_visual_traj', 'var')
        vo_xyz = pure_visual_traj(:, 1:3);
    elseif exist('odo_trajectory', 'var')
        vo_xyz = odo_trajectory(:, 1:3);
        fprintf('  (使用旧数据：odo_trajectory)\n');
    else
        error('未找到纯视觉轨迹数据（pure_visual_traj）');
    end
    [vo_rmse, vo_final, vo_drift, vo_xyz_aligned] = ...
        compute_metrics_with_alignment(vo_xyz, gt_xyz, gt_length);
    fprintf('  RMSE: %.2f m, End Error: %.2f m, Drift Rate: %.2f%%\n\n', ...
        vo_rmse, vo_final, vo_drift);
    
    %% 组件贡献分析
    fprintf('\n─── 消融实验分析 ───\n');
    if ~isnan(imu_rmse)
        exp_contrib = (imu_rmse - exp_rmse) / imu_rmse * 100;
        fprintf('  Experience Map贡献: %.0f%% (%.2f → %.2f m)\n', ...
            exp_contrib, imu_rmse, exp_rmse);
    end
    if ~isnan(imu_rmse)
        imu_contrib = (vo_rmse - imu_rmse) / vo_rmse * 100;
        fprintf('  IMU Fusion贡献: %.0f%% (%.2f → %.2f m)\n', ...
            imu_contrib, vo_rmse, imu_rmse);
    end
    total_contrib = (vo_rmse - exp_rmse) / vo_rmse * 100;
    fprintf('  总体改进: %.0f%% (%.2f → %.2f m)\n', ...
        total_contrib, vo_rmse, exp_rmse);
    
    
    %% 保存结果
    all_results.(dataset_name).Complete.description = 'Complete System (Ours)';
    all_results.(dataset_name).Complete.rmse = exp_rmse;
    all_results.(dataset_name).Complete.final_error = exp_final;
    all_results.(dataset_name).Complete.drift_rate = exp_drift;
    
    all_results.(dataset_name).No_ExpMap.description = 'w/o Experience Map';
    all_results.(dataset_name).No_ExpMap.rmse = imu_rmse;
    all_results.(dataset_name).No_ExpMap.final_error = imu_final;
    all_results.(dataset_name).No_ExpMap.drift_rate = imu_drift;
    
    all_results.(dataset_name).No_IMU.description = 'w/o IMU Fusion';
    all_results.(dataset_name).No_IMU.rmse = vo_rmse;
    all_results.(dataset_name).No_IMU.final_error = vo_final;
    all_results.(dataset_name).No_IMU.drift_rate = vo_drift;
end

%% 生成汇总报告
fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   消融实验结果汇总 - Town01 & MH_03                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

for d = 1:size(datasets, 1)
    dataset_name = datasets{d, 2};
    if ~isfield(all_results, dataset_name)
        continue;
    end
    
    fprintf('═══ %s (%.0fm trajectory) ═══\n\n', dataset_name, datasets{d, 4});
    
    %% 消融实验结果表
    fprintf('【消融实验结果】\n');
    fprintf('| Configuration | RMSE (m) | Drift (%%) | Degradation |\n');
    fprintf('|---------------|----------|-----------|-------------|\n');
    
    res = all_results.(dataset_name);
    baseline_rmse = res.Complete.rmse;
    
    % Complete
    fprintf('| %s | %.2f | %.2f | Baseline |\n', ...
        res.Complete.description, res.Complete.rmse, res.Complete.drift_rate);
    
    % w/o ExpMap
    if ~isnan(res.No_ExpMap.rmse)
        deg_pct = (res.No_ExpMap.rmse - baseline_rmse) / baseline_rmse * 100;
        fprintf('| %s | %.2f | %.2f | +%.0f%% |\n', ...
            res.No_ExpMap.description, res.No_ExpMap.rmse, res.No_ExpMap.drift_rate, deg_pct);
    end
    
    % w/o IMU
    deg_pct = (res.No_IMU.rmse - baseline_rmse) / baseline_rmse * 100;
    fprintf('| %s | %.2f | %.2f | +%.0f%% |\n', ...
        res.No_IMU.description, res.No_IMU.rmse, res.No_IMU.drift_rate, deg_pct);
    
    fprintf('\n');
end  % 关闭156行的for循环

%% 保存结果
results_dir = fullfile(neuro_root, 'data', '01_NeuroSLAM_Datasets', 'ablation_results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'ablation_results_aligned.mat'), 'all_results');

% CSV
csvfile = fullfile(results_dir, 'ablation_results_aligned.csv');
fid = fopen(csvfile, 'w');
fprintf(fid, 'Dataset,Configuration,RMSE_m,Final_Error_m,Drift_Rate_pct\n');

for d = 1:size(datasets, 1)
    dataset_name = datasets{d, 2};
    if ~isfield(all_results, dataset_name)
        continue;
    end
    
    res = all_results.(dataset_name);
    configs = {'Complete', 'No_ExpMap', 'No_IMU'};
    
    for c = 1:length(configs)
        cfg = configs{c};
        if ~isnan(res.(cfg).rmse)
            fprintf(fid, '%s,%s,%.2f,%.2f,%.2f\n', ...
                dataset_name, res.(cfg).description, ...
                res.(cfg).rmse, res.(cfg).final_error, res.(cfg).drift_rate);
        end
    end
end
fclose(fid);

fprintf('✅ 结果已保存:\n');
fprintf('   MAT: ablation_results_aligned.mat\n');
fprintf('   CSV: ablation_results_aligned.csv\n\n');

fprintf('💡 消融实验完成！\n');
fprintf('   ✅ 3个配置：Complete, w/o ExpMap, w/o IMU\n');
fprintf('   ✅ 所有轨迹已用Procrustes对齐到GT坐标系\n');
fprintf('   ✅ 数据集：Town01 (长距离) & MH_03 (短距离)\n');
fprintf('   ✅ 指标：RMSE, End Error, Drift Rate\n\n');
fprintf('📊 下一步：运行 GENERATE_ABLATION_ONLY 生成图表\n\n');
