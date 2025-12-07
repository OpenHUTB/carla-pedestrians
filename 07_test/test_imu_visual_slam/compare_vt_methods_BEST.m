%% 对比原始VT和增强VT的NeuroSLAM性能
%  完全基于test_imu_visual_fusion_slam.m的逻辑
%  只替换VT特征提取方法

clear all; close all; clc;

fprintf('========================================\n');
fprintf('  原始VT vs 增强VT 对比测试\n');
fprintf('========================================\n\n');

%% 公共设置
rootDir = '/home/dream/neuro_111111/carla-pedestrians/neuro';
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/3d_grid_cells_network'));
addpath(fullfile(rootDir, '01_conjunctive_pose_cells_network/yaw_height_hdc_network'));
addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '03_visual_odometry'));
addpath(fullfile(rootDir, '02_multilayered_experience_map'));
addpath(fullfile(rootDir, '05_tookit/process_visual_data/process_images_data'));
addpath(fullfile(rootDir, '09_vestibular'));

dataPath = '/home/dream/neuro_111111/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion';

% 读取Ground Truth和IMU数据（只读一次）
fprintf('读取Ground Truth和IMU数据...\n');
gt_data = read_ground_truth(fullfile(dataPath, 'ground_truth.txt'));
fusion_data = read_fusion_pose(dataPath);
imu_data = read_imu_data(dataPath);

imgFiles = dir(fullfile(dataPath, '*.png'));
[~, sortIdx] = sort({imgFiles.name});
imgFiles = imgFiles(sortIdx);
num_frames = min([length(imgFiles), gt_data.count, 5000]);
fprintf('  将处理 %d 帧图像\n', num_frames);

%% ========== 运行1: 原始VT方法 ==========
fprintf('\n========== [1/2] 运行原始patch normalization方法 ==========\n');
[exp_traj_original, odo_traj_original, stats_original] = run_slam_with_vt_method('original', ...
    dataPath, imgFiles, num_frames, fusion_data, imu_data, rootDir);

%% ========== 运行2: 增强VT方法 ==========
fprintf('\n========== [2/2] 运行增强HART+CORnet方法 ==========\n');
[exp_traj_enhanced, odo_traj_enhanced, stats_enhanced] = run_slam_with_vt_method('enhanced', ...
    dataPath, imgFiles, num_frames, fusion_data, imu_data, rootDir);

%% ========== 对比可视化 ==========
fprintf('\n========== 生成对比图表 ==========\n');

% 对齐轨迹
fprintf('对齐轨迹到相同坐标系...\n');
gt_pos = gt_data.pos(1:num_frames, :);
[fusion_pos_aligned, gt_aligned] = align_trajectories(fusion_data.pos, gt_pos, 'simple');
[odo_original_aligned, ~] = align_trajectories(odo_traj_original, gt_pos, 'simple');
[exp_original_aligned, ~] = align_trajectories(exp_traj_original, gt_pos, 'simple');
[odo_enhanced_aligned, ~] = align_trajectories(odo_traj_enhanced, gt_pos, 'simple');
[exp_enhanced_aligned, ~] = align_trajectories(exp_traj_enhanced, gt_pos, 'simple');

% 计算RMSE（使用GT作为参考）
errors_fusion = sqrt(sum((fusion_pos_aligned - gt_aligned).^2, 2));
errors_original = sqrt(sum((exp_original_aligned - gt_aligned).^2, 2));
errors_enhanced = sqrt(sum((exp_enhanced_aligned - gt_aligned).^2, 2));

stats_original.rmse = mean(errors_original);
stats_enhanced.rmse = mean(errors_enhanced);

% 调试：检查对齐后的范围
fprintf('\n对齐后的轨迹范围:\n');
fprintf('  GT: X[%.2f, %.2f], Y[%.2f, %.2f]\n', ...
    min(gt_aligned(:,1)), max(gt_aligned(:,1)), min(gt_aligned(:,2)), max(gt_aligned(:,2)));
fprintf('  Fusion: X[%.2f, %.2f], Y[%.2f, %.2f]\n', ...
    min(fusion_pos_aligned(:,1)), max(fusion_pos_aligned(:,1)), ...
    min(fusion_pos_aligned(:,2)), max(fusion_pos_aligned(:,2)));
fprintf('  原始SLAM: X[%.2f, %.2f], Y[%.2f, %.2f]\n', ...
    min(exp_original_aligned(:,1)), max(exp_original_aligned(:,1)), ...
    min(exp_original_aligned(:,2)), max(exp_original_aligned(:,2)));
fprintf('  增强SLAM: X[%.2f, %.2f], Y[%.2f, %.2f]\n', ...
    min(exp_enhanced_aligned(:,1)), max(exp_enhanced_aligned(:,1)), ...
    min(exp_enhanced_aligned(:,2)), max(exp_enhanced_aligned(:,2)));

fprintf('\nRMSE结果:\n');
fprintf('  融合位姿 RMSE: %.2f m\n', mean(errors_fusion));
fprintf('  原始VT RMSE: %.2f m\n', stats_original.rmse);
fprintf('  增强VT RMSE: %.2f m\n', stats_enhanced.rmse);

% 创建对比图（使用原始脚本风格）
figure('Name', '原始VT vs 增强VT vs Fusion 对比', 'Position', [100 100 1600 900]);

% 左侧：原始VT方法
subplot(2, 3, 1);
hold on; grid on;
plot(gt_aligned(:,1), gt_aligned(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
plot(fusion_pos_aligned(:,1), fusion_pos_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Fusion');
plot(odo_original_aligned(:,1), odo_original_aligned(:,2), 'b--', 'LineWidth', 1, 'DisplayName', '里程计');
plot(exp_original_aligned(:,1), exp_original_aligned(:,2), 'g:', 'LineWidth', 1.5, 'DisplayName', 'SLAM');
xlabel('X (m)'); ylabel('Y (m)');
title('原始VT: patch normalization');
legend('Location', 'best');
axis equal;

% 中间：增强VT方法
subplot(2, 3, 2);
hold on; grid on;
plot(gt_aligned(:,1), gt_aligned(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
plot(fusion_pos_aligned(:,1), fusion_pos_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Fusion');
plot(odo_enhanced_aligned(:,1), odo_enhanced_aligned(:,2), 'b--', 'LineWidth', 1, 'DisplayName', '里程计');
plot(exp_enhanced_aligned(:,1), exp_enhanced_aligned(:,2), 'g:', 'LineWidth', 1.5, 'DisplayName', 'SLAM');
xlabel('X (m)'); ylabel('Y (m)');
title('增强VT: HART+CORnet');
legend('Location', 'best');
axis equal;

% 右侧：仅SLAM轨迹对比
subplot(2, 3, 3);
hold on; grid on;
plot(gt_aligned(:,1), gt_aligned(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
plot(exp_original_aligned(:,1), exp_original_aligned(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '原始VT');
plot(exp_enhanced_aligned(:,1), exp_enhanced_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', '增强VT');
xlabel('X (m)'); ylabel('Y (m)');
title('SLAM轨迹直接对比');
legend('Location', 'best');
axis equal;

% 下方：误差对比
subplot(2, 3, 4:6);
hold on; grid on;
frames = 1:length(errors_fusion);
plot(frames, errors_fusion, 'r-', 'LineWidth', 1.5, 'DisplayName', sprintf('Fusion (%.2fm)', mean(errors_fusion)));
plot(frames, errors_original, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('原始VT (%.2fm)', stats_original.rmse));
plot(frames, errors_enhanced, 'g-', 'LineWidth', 1.5, 'DisplayName', sprintf('增强VT (%.2fm)', stats_enhanced.rmse));
xlabel('帧数'); ylabel('位置误差 (m)');
title('位置误差随时间变化（相对于GT）');
legend('Location', 'best');

%% ========== 性能对比表 ==========
fprintf('\n========================================\n');
fprintf('             性能对比\n');
fprintf('========================================\n');
fprintf('指标              | 原始VT    | 增强VT    | 改善\n');
fprintf('------------------+-----------+-----------+------\n');
fprintf('VT数量           | %9d | %9d | %s\n', stats_original.num_vts, stats_enhanced.num_vts, ...
    get_change_str(stats_original.num_vts, stats_enhanced.num_vts));
fprintf('经验节点数       | %9d | %9d | %s\n', stats_original.num_exps, stats_enhanced.num_exps, ...
    get_change_str(stats_original.num_exps, stats_enhanced.num_exps));
fprintf('处理时间(秒)     | %9.2f | %9.2f | %s\n', stats_original.time, stats_enhanced.time, ...
    get_speedup_str(stats_original.time, stats_enhanced.time));
fprintf('平均帧时间(ms)   | %9.2f | %9.2f | %s\n', stats_original.time/num_frames*1000, ...
    stats_enhanced.time/num_frames*1000, get_speedup_str(stats_original.time, stats_enhanced.time));
fprintf('RMSE (m)         | %9.2f | %9.2f | %s\n', stats_original.rmse, stats_enhanced.rmse, ...
    get_improvement_str(stats_original.rmse, stats_enhanced.rmse));
fprintf('========================================\n');

%% 辅助函数
function str = get_change_str(old_val, new_val)
    if new_val > old_val
        str = sprintf('+%.0f%%', (new_val/old_val - 1) * 100);
    elseif new_val < old_val
        str = sprintf('%.0f%%', (new_val/old_val - 1) * 100);
    else
        str = '=';
    end
end

function str = get_speedup_str(old_time, new_time)
    speedup = old_time / new_time;
    if speedup > 1
        str = sprintf('%.2fx', speedup);
    else
        str = sprintf('%.2fx', speedup);
    end
end

function str = get_improvement_str(old_val, new_val)
    improvement = (old_val - new_val) / old_val * 100;
    if improvement > 0
        str = sprintf('↓%.0f%%', improvement);
    else
        str = sprintf('↑%.0f%%', -improvement);
    end
end
