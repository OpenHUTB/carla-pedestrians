function visualize_experience_map(dataset_names)
% VISUALIZE_EXPERIENCE_MAP 可视化经验地图拓扑结构
%
% 参数:
%   dataset_names - 数据集名称数组，如 {'Town01Data_IMU_Fusion', 'Town02Data_IMU_Fusion'}
%
% 示例:
%   visualize_experience_map({'Town01Data_IMU_Fusion', 'Town02Data_IMU_Fusion', 'Town10Data_IMU_Fusion'});

if nargin < 1
    dataset_names = {'Town01Data_IMU_Fusion', 'Town02Data_IMU_Fusion', 'Town10Data_IMU_Fusion'};
end

% 获取neuro根目录
% utils/ -> test_imu_visual_slam/ -> 07_test/ -> neuro/
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));

% 创建大图
figure('Name', 'Experience Map Comparison', 'Position', [100, 100, 1800, 600]);

num_datasets = length(dataset_names);
colors = {'b', 'r', 'g', 'm', 'c'};

for idx = 1:num_datasets
    dataset_name = dataset_names{idx};
    data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets', dataset_name);
    mat_file = fullfile(data_path, 'slam_results', 'trajectories.mat');
    
    if ~exist(mat_file, 'file')
        warning('Data file not found: %s', mat_file);
        continue;
    end
    
    % 加载数据
    data = load(mat_file);
    
    % 提取经验地图轨迹
    if isfield(data, 'exp_trajectory')
        exp_traj = data.exp_trajectory;
    else
        warning('No exp_trajectory field in data file: %s', dataset_name);
        continue;
    end
    
    % 子图1: XY平面轨迹
    subplot(1, 3, 1);
    hold on;
    plot(exp_traj(:,1), exp_traj(:,2), '-', 'Color', colors{idx}, 'LineWidth', 1.5, ...
        'DisplayName', strrep(dataset_name, 'Data_IMU_Fusion', ''));
    plot(exp_traj(1,1), exp_traj(1,2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors{idx}, ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    plot(exp_traj(end,1), exp_traj(end,2), 's', 'MarkerSize', 10, 'MarkerFaceColor', colors{idx}, ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    
    % 子图2: XZ平面轨迹
    subplot(1, 3, 2);
    hold on;
    plot(exp_traj(:,1), exp_traj(:,3), '-', 'Color', colors{idx}, 'LineWidth', 1.5, ...
        'DisplayName', strrep(dataset_name, 'Data_IMU_Fusion', ''));
    plot(exp_traj(1,1), exp_traj(1,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors{idx}, ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    plot(exp_traj(end,1), exp_traj(end,3), 's', 'MarkerSize', 10, 'MarkerFaceColor', colors{idx}, ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    
    % 子图3: 3D轨迹
    subplot(1, 3, 3);
    hold on;
    plot3(exp_traj(:,1), exp_traj(:,2), exp_traj(:,3), '-', 'Color', colors{idx}, 'LineWidth', 1.5, ...
        'DisplayName', strrep(dataset_name, 'Data_IMU_Fusion', ''));
    plot3(exp_traj(1,1), exp_traj(1,2), exp_traj(1,3), 'o', 'MarkerSize', 10, ...
        'MarkerFaceColor', colors{idx}, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    plot3(exp_traj(end,1), exp_traj(end,2), exp_traj(end,3), 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', colors{idx}, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    
    % Analyze trajectory characteristics
    fprintf('\n=== %s Experience Map Analysis ===\n', dataset_name);
    fprintf('Total nodes: %d\n', size(exp_traj, 1));
    
    % Calculate distances between adjacent nodes
    if size(exp_traj, 1) > 1
        dists = sqrt(sum(diff(exp_traj).^2, 2));
        fprintf('Average node spacing: %.2f m\n', mean(dists));
        fprintf('Maximum node spacing: %.2f m\n', max(dists));
        fprintf('Minimum node spacing: %.2f m\n', min(dists));
        
        % Detect possible loop closures (sudden distance changes)
        jump_threshold = mean(dists) * 3;
        large_jumps = find(dists > jump_threshold);
        fprintf('Large jumps (>3x average): %d\n', length(large_jumps));
    end
    
    % Calculate total trajectory length
    if size(exp_traj, 1) > 1
        total_length = sum(sqrt(sum(diff(exp_traj).^2, 2)));
        fprintf('Total trajectory length: %.2f m\n', total_length);
    end
    
    % Calculate trajectory bounds
    fprintf('X range: [%.2f, %.2f] (%.2f m)\n', min(exp_traj(:,1)), max(exp_traj(:,1)), ...
        max(exp_traj(:,1)) - min(exp_traj(:,1)));
    fprintf('Y range: [%.2f, %.2f] (%.2f m)\n', min(exp_traj(:,2)), max(exp_traj(:,2)), ...
        max(exp_traj(:,2)) - min(exp_traj(:,2)));
    fprintf('Z range: [%.2f, %.2f] (%.2f m)\n', min(exp_traj(:,3)), max(exp_traj(:,3)), ...
        max(exp_traj(:,3)) - min(exp_traj(:,3)));
end

% Set subplot properties
subplot(1, 3, 1);
xlabel('X (m)');
ylabel('Y (m)');
title('Experience Map - XY Plane');
legend('Location', 'best');
grid on;
axis equal;

subplot(1, 3, 2);
xlabel('X (m)');
ylabel('Z (m)');
title('Experience Map - XZ Plane');
legend('Location', 'best');
grid on;
axis equal;

subplot(1, 3, 3);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Experience Map - 3D View');
legend('Location', 'best');
grid on;
axis equal;
view(45, 30);
rotate3d on;

fprintf('\nLegend: ○ Start, □ End\n');
fprintf('Note: Town01 with good loop closure should show trajectory returning near start\n\n');

end
