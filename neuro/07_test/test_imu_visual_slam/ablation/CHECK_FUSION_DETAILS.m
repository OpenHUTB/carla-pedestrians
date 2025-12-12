%% 检查fusion_data的详细内容

clear all; close all; clc;

fprintf('\n检查fusion_data详细信息...\n\n');

% 加载文件
data_file = '/home/dream/neuro_111111/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results/trajectories.mat';
load(data_file);

fprintf('=== fusion_data 所有字段 ===\n');
disp(fieldnames(fusion_data));

fprintf('\n=== 各字段大小 ===\n');
fields = fieldnames(fusion_data);
for i = 1:length(fields)
    f = fields{i};
    fprintf('  %s: [%s]\n', f, num2str(size(fusion_data.(f))));
end

fprintf('\n=== fusion_data.pos 前5行 ===\n');
disp(fusion_data.pos(1:5, :));

fprintf('\n=== gt_data.pos 前5行 ===\n');
disp(gt_data.pos(1:5, :));

fprintf('\n=== 计算初始误差 ===\n');
initial_error = sqrt(sum((fusion_data.pos(1, 1:3) - gt_data.pos(1, 1:3)).^2));
fprintf('第1帧误差: %.2f m\n', initial_error);

middle_idx = round(size(fusion_data.pos,1)/2);
middle_error = sqrt(sum((fusion_data.pos(middle_idx, 1:3) - gt_data.pos(middle_idx, 1:3)).^2));
fprintf('中间帧误差: %.2f m\n', middle_error);

final_error = sqrt(sum((fusion_data.pos(end, 1:3) - gt_data.pos(end, 1:3)).^2));
fprintf('最后帧误差: %.2f m\n', final_error);

fprintf('\n=== 检查是否已经对齐 ===\n');
if initial_error < 1.0
    fprintf('⚠️  第1帧误差很小(%.2fm)，fusion_data.pos可能已经对齐过了\n', initial_error);
    fprintf('    应该使用原始的imu_pos而不是pos\n');
else
    fprintf('✓ 第1帧误差较大(%.2fm)，数据未对齐\n', initial_error);
end

% 检查是否有imu_pos字段
if isfield(fusion_data, 'imu_pos')
    fprintf('\n=== fusion_data.imu_pos 前5行 ===\n');
    disp(fusion_data.imu_pos(1:5, :));
    
    fprintf('\n=== 使用imu_pos计算RMSE ===\n');
    imu_xyz = fusion_data.imu_pos(:, 1:3);
    gt_xyz = gt_data.pos(:, 1:3);
    min_len = min(size(imu_xyz,1), size(gt_xyz,1));
    
    errors = sqrt(sum((imu_xyz(1:min_len,:) - gt_xyz(1:min_len,:)).^2, 2));
    rmse = sqrt(mean(errors.^2));
    fprintf('使用imu_pos的RMSE: %.2f m\n', rmse);
end

fprintf('\n完成！\n');
