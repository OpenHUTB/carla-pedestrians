%% 手动完成可视化
% 如果实验已运行但可视化未完成,使用此脚本手动完成

fprintf('========== 手动完成可视化 ==========\n\n');

%% 检查必需变量
required_vars = {'baseline_odo_traj', 'baseline_exp_traj', 'ours_odo_traj', 'ours_exp_traj', ...
                 'NUM_VT_BL', 'NUM_EXPS_BL', 'NUM_VT_OURS', 'NUM_EXPS_OURS'};

missing_vars = {};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        missing_vars{end+1} = required_vars{i};
    end
end

if ~isempty(missing_vars)
    fprintf('✗ 缺少必需变量:\n');
    for i = 1:length(missing_vars)
        fprintf('  - %s\n', missing_vars{i});
    end
    fprintf('\n请先运行实验:\n');
    fprintf('  >> cd E:\\Neuro_end\\neuro\\07_test\\07_test\\test_imu_visual_slam\\core\n');
    fprintf('  >> test_imu_visual_fusion_slam2\n');
    return;
end

fprintf('✓ 所有必需变量存在\n');

%% 设置参数
dataset_name = 'Town01Data_IMU_Fusion';
num_frames = size(baseline_odo_traj, 1);
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
result_path = fullfile(data_path, 'comparison_results');

if ~exist(result_path, 'dir')
    mkdir(result_path);
    fprintf('✓ 创建结果目录: %s\n', result_path);
end

%% 读取Ground Truth
fprintf('\n[1] 读取Ground Truth...\n');
gt_file = fullfile(data_path, 'ground_truth.txt');
if exist(gt_file, 'file')
    % 读取GT
    fid = fopen(gt_file, 'r');
    header = fgetl(fid);
    data = textscan(fid, '%f %f %f %f %f %f %f %f', 'Delimiter', ',');
    fclose(fid);
    
    gt_data = struct();
    gt_data.timestamp = data{1};
    gt_data.pos = [data{2}, data{3}, data{4}];
    gt_data.quat = [data{5}, data{6}, data{7}, data{8}];
    
    % 裁剪到相同长度
    min_len = min([size(gt_data.pos, 1), num_frames]);
    gt_pos_trim = gt_data.pos(1:min_len, :);
    baseline_odo_trim = baseline_odo_traj(1:min_len, :);
    baseline_exp_trim = baseline_exp_traj(1:min_len, :);
    ours_odo_trim = ours_odo_traj(1:min_len, :);
    ours_exp_trim = ours_exp_traj(1:min_len, :);
    
    % Town数据集: 2D对齐
    gt_pos_trim(:, 3) = 0;
    baseline_odo_trim(:, 3) = 0;
    baseline_exp_trim(:, 3) = 0;
    ours_odo_trim(:, 3) = 0;
    ours_exp_trim(:, 3) = 0;
    
    % 添加路径
    addpath('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\utils');
    
    % 轴匹配
    n_map = min(min_len, 1000);
    idx_map = unique(round(linspace(1, min_len, n_map)));
    gt_xy = gt_pos_trim(idx_map, 1:2);
    
    % Baseline轴匹配
    try
        [swap_bl, sx_bl, sy_bl] = best_axis_match_xy(baseline_exp_trim(idx_map, 1:2), gt_xy);
        bl_xy = baseline_exp_trim(:, 1:2);
        if swap_bl, bl_xy = [bl_xy(:,2), bl_xy(:,1)]; end
        bl_xy = [sx_bl * bl_xy(:,1), sy_bl * bl_xy(:,2)];
        baseline_exp_trim(:, 1:2) = bl_xy;
        
        bl_odo_xy = baseline_odo_trim(:, 1:2);
        if swap_bl, bl_odo_xy = [bl_odo_xy(:,2), bl_odo_xy(:,1)]; end
        bl_odo_xy = [sx_bl * bl_odo_xy(:,1), sy_bl * bl_odo_xy(:,2)];
        baseline_odo_trim(:, 1:2) = bl_odo_xy;
    catch
    end
    
    % Ours轴匹配
    try
        [swap_ours, sx_ours, sy_ours] = best_axis_match_xy(ours_exp_trim(idx_map, 1:2), gt_xy);
        ours_xy = ours_exp_trim(:, 1:2);
        if swap_ours, ours_xy = [ours_xy(:,2), ours_xy(:,1)]; end
        ours_xy = [sx_ours * ours_xy(:,1), sy_ours * ours_xy(:,2)];
        ours_exp_trim(:, 1:2) = ours_xy;
        
        ours_odo_xy = ours_odo_trim(:, 1:2);
        if swap_ours, ours_odo_xy = [ours_odo_xy(:,2), ours_odo_xy(:,1)]; end
        ours_odo_xy = [sx_ours * ours_odo_xy(:,1), sy_ours * ours_odo_xy(:,2)];
        ours_odo_trim(:, 1:2) = ours_odo_xy;
    catch
    end
    
    % Sim(3)对齐
    fit_n = min(1000, min_len);
    fit_idx = unique(round(linspace(1, min_len, fit_n)));
    
    try
        [~, ~, R_bl, t_bl, s_bl] = align_trajectories(baseline_exp_trim(fit_idx, :), gt_pos_trim(fit_idx, :), 'rigid_2d');
        baseline_exp_aligned = (s_bl * R_bl * baseline_exp_trim')' + repmat(t_bl', min_len, 1);
        [~, ~, R_bl_odo, t_bl_odo, s_bl_odo] = align_trajectories(baseline_odo_trim(fit_idx, :), gt_pos_trim(fit_idx, :), 'rigid_2d');
        baseline_odo_aligned = (s_bl_odo * R_bl_odo * baseline_odo_trim')' + repmat(t_bl_odo', min_len, 1);
    catch
        baseline_exp_aligned = baseline_exp_trim;
        baseline_odo_aligned = baseline_odo_trim;
    end
    
    try
        [~, ~, R_ours, t_ours, s_ours] = align_trajectories(ours_exp_trim(fit_idx, :), gt_pos_trim(fit_idx, :), 'rigid_2d');
        ours_exp_aligned = (s_ours * R_ours * ours_exp_trim')' + repmat(t_ours', min_len, 1);
        [~, ~, R_ours_odo, t_ours_odo, s_ours_odo] = align_trajectories(ours_odo_trim(fit_idx, :), gt_pos_trim(fit_idx, :), 'rigid_2d');
        ours_odo_aligned = (s_ours_odo * R_ours_odo * ours_odo_trim')' + repmat(t_ours_odo', min_len, 1);
    catch
        ours_exp_aligned = ours_exp_trim;
        ours_odo_aligned = ours_odo_trim;
    end
    
    gt_pos_aligned = gt_pos_trim;
    gt_pos_aligned(:, 3) = 0;
    baseline_exp_aligned(:, 3) = 0;
    baseline_odo_aligned(:, 3) = 0;
    ours_exp_aligned(:, 3) = 0;
    ours_odo_aligned(:, 3) = 0;
    
    has_ground_truth = true;
    fprintf('✓ Ground Truth对齐完成\n');
else
    fprintf('⚠️ 未找到Ground Truth\n');
    has_ground_truth = false;
    baseline_exp_aligned = baseline_exp_traj;
    baseline_odo_aligned = baseline_odo_traj;
    ours_exp_aligned = ours_exp_traj;
    ours_odo_aligned = ours_odo_traj;
end

%% 计算精度
fprintf('\n[2] 计算精度指标...\n');
if has_ground_truth
    bl_exp_error = sqrt(sum((baseline_exp_aligned - gt_pos_aligned).^2, 2));
    bl_odo_error = sqrt(sum((baseline_odo_aligned - gt_pos_aligned).^2, 2));
    ours_exp_error = sqrt(sum((ours_exp_aligned - gt_pos_aligned).^2, 2));
    ours_odo_error = sqrt(sum((ours_odo_aligned - gt_pos_aligned).^2, 2));
    
    bl_exp_ate = mean(bl_exp_error);
    bl_exp_rmse = sqrt(mean(bl_exp_error.^2));
    bl_exp_max = max(bl_exp_error);
    bl_odo_ate = mean(bl_odo_error);
    bl_odo_rmse = sqrt(mean(bl_odo_error.^2));
    
    ours_exp_ate = mean(ours_exp_error);
    ours_exp_rmse = sqrt(mean(ours_exp_error.^2));
    ours_exp_max = max(ours_exp_error);
    ours_odo_ate = mean(ours_odo_error);
    ours_odo_rmse = sqrt(mean(ours_odo_error.^2));
    
    ate_improvement = (bl_exp_ate - ours_exp_ate) / bl_exp_ate * 100;
    rmse_improvement = (bl_exp_rmse - ours_exp_rmse) / bl_exp_rmse * 100;
    
    fprintf('✓ 精度计算完成\n');
    fprintf('  ATE改善: %.1f%%\n', ate_improvement);
    fprintf('  RMSE改善: %.1f%%\n', rmse_improvement);
end

%% 保存数据
fprintf('\n[3] 保存数据...\n');
save(fullfile(result_path, 'comparison_results.mat'), ...
    'baseline_odo_traj', 'baseline_exp_traj', ...
    'ours_odo_traj', 'ours_exp_traj', ...
    'NUM_VT_BL', 'NUM_EXPS_BL', 'NUM_VT_OURS', 'NUM_EXPS_OURS', ...
    'dataset_name', 'num_frames');

if has_ground_truth
    save(fullfile(result_path, 'comparison_results.mat'), ...
        'gt_data', 'gt_pos_aligned', ...
        'baseline_exp_aligned', 'baseline_odo_aligned', ...
        'ours_exp_aligned', 'ours_odo_aligned', ...
        'bl_exp_ate', 'bl_exp_rmse', 'bl_odo_ate', 'bl_odo_rmse', ...
        'ours_exp_ate', 'ours_exp_rmse', 'ours_odo_ate', 'ours_odo_rmse', ...
        'ate_improvement', 'rmse_improvement', ...
        '-append');
end

fprintf('✓ 数据已保存: %s\n', fullfile(result_path, 'comparison_results.mat'));

%% 生成可视化
fprintf('\n[4] 生成可视化...\n');
fprintf('运行: QUICK_VIEW_RESULTS\n');
QUICK_VIEW_RESULTS;

fprintf('\n========== 完成！ ==========\n');
fprintf('结果保存在: %s\n', result_path);
