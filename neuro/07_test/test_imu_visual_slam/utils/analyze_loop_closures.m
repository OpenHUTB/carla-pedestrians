function analyze_loop_closures(dataset_name)
% ANALYZE_LOOP_CLOSURES 分析经验地图的闭环检测情况
%
% 参数:
%   dataset_name - 数据集名称，如 'Town01Data_IMU_Fusion'
%
% 示例:
%   analyze_loop_closures('Town01Data_IMU_Fusion');

if nargin < 1
    dataset_name = 'Town01Data_IMU_Fusion';
end

% 获取neuro根目录
% utils/ -> test_imu_visual_slam/ -> 07_test/ -> neuro/
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets', dataset_name);
mat_file = fullfile(data_path, 'slam_results', 'trajectories.mat');

if ~exist(mat_file, 'file')
    error('Data file not found: %s', mat_file);
end

% Load data
fprintf('Loading data: %s\n', dataset_name);
data = load(mat_file);

if ~isfield(data, 'exp_trajectory')
    error('No exp_trajectory field in data file');
end

exp_traj = data.exp_trajectory;
num_nodes = size(exp_traj, 1);

fprintf('\n=== Experience Map Basic Info ===\n');
fprintf('Total nodes: %d\n', num_nodes);

% Calculate distance matrix between all nodes
fprintf('\nCalculating node distance matrix...\n');
dist_matrix = zeros(num_nodes, num_nodes);
for i = 1:num_nodes
    for j = i+1:num_nodes
        dist = norm(exp_traj(i,:) - exp_traj(j,:));
        dist_matrix(i,j) = dist;
        dist_matrix(j,i) = dist;
    end
end

% Detect potential loop closures
% Loop closure condition: nodes far apart in time but close in space
loop_threshold = 5.0;  % Within 5m considered potential loop closure
time_gap_threshold = 50;  % At least 50 frames apart

potential_loops = [];
for i = 1:num_nodes
    for j = i+time_gap_threshold:num_nodes
        if dist_matrix(i,j) < loop_threshold
            potential_loops = [potential_loops; i, j, dist_matrix(i,j)];
        end
    end
end

fprintf('\n=== Loop Closure Detection Analysis ===\n');
fprintf('Loop threshold: %.2f m\n', loop_threshold);
fprintf('Time gap threshold: %d frames\n', time_gap_threshold);
fprintf('Detected potential loops: %d\n', size(potential_loops, 1));

if size(potential_loops, 1) > 0
    fprintf('\nFirst 10 loop closures:\n');
    fprintf('  Node1  Node2   Dist(m)  Time Gap(frames)\n');
    for i = 1:min(10, size(potential_loops, 1))
        fprintf('  %5d  %5d   %6.2f     %4d\n', ...
            potential_loops(i,1), potential_loops(i,2), ...
            potential_loops(i,3), potential_loops(i,2) - potential_loops(i,1));
    end
end

% Visualization
figure('Name', sprintf('%s Loop Closure Analysis', dataset_name), 'Position', [100, 100, 1600, 800]);

% Subplot 1: Trajectory with loop closure markers
subplot(2, 3, 1);
plot(exp_traj(:,1), exp_traj(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(exp_traj(1,1), exp_traj(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(exp_traj(end,1), exp_traj(end,2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'End');

% Mark loop closures
if size(potential_loops, 1) > 0
    for i = 1:size(potential_loops, 1)
        idx1 = potential_loops(i,1);
        idx2 = potential_loops(i,2);
        plot([exp_traj(idx1,1), exp_traj(idx2,1)], ...
             [exp_traj(idx1,2), exp_traj(idx2,2)], ...
             'r--', 'LineWidth', 0.5);
    end
end
xlabel('X (m)');
ylabel('Y (m)');
title('Trajectory with Loop Closures');
legend('Location', 'best');
grid on;
axis equal;

% Subplot 2: Distance matrix heatmap
subplot(2, 3, 2);
imagesc(dist_matrix);
colorbar;
xlabel('Node Index');
ylabel('Node Index');
title('Node Distance Matrix');
colormap(jet);

% Subplot 3: Node spacing statistics
subplot(2, 3, 3);
if num_nodes > 1
    sequential_dists = sqrt(sum(diff(exp_traj).^2, 2));
    histogram(sequential_dists, 50);
    xlabel('Distance (m)');
    ylabel('Count');
    title('Sequential Node Distance Distribution');
    grid on;
    
    fprintf('\n=== Node Spacing Statistics ===\n');
    fprintf('Mean: %.2f m\n', mean(sequential_dists));
    fprintf('Median: %.2f m\n', median(sequential_dists));
    fprintf('Std Dev: %.2f m\n', std(sequential_dists));
end

% Subplot 4: 3D trajectory view
subplot(2, 3, 4);
plot3(exp_traj(:,1), exp_traj(:,2), exp_traj(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot3(exp_traj(1,1), exp_traj(1,2), exp_traj(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(exp_traj(end,1), exp_traj(end,2), exp_traj(end,3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory');
grid on;
axis equal;
view(45, 30);

% Subplot 5: Loop closure distance distribution
subplot(2, 3, 5);
if size(potential_loops, 1) > 0
    histogram(potential_loops(:,3), 30);
    xlabel('Loop Distance (m)');
    ylabel('Count');
    title('Loop Closure Distance Distribution');
    grid on;
else
    text(0.5, 0.5, 'No Loop Closures Detected', 'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
end

% Subplot 6: Loop closure time gap distribution
subplot(2, 3, 6);
if size(potential_loops, 1) > 0
    time_gaps = potential_loops(:,2) - potential_loops(:,1);
    histogram(time_gaps, 30);
    xlabel('Time Gap (frames)');
    ylabel('Count');
    title('Loop Closure Time Gap Distribution');
    grid on;
else
    text(0.5, 0.5, 'No Loop Closures Detected', 'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
end

% Calculate loop closure rate
loop_closure_rate = size(potential_loops, 1) / num_nodes * 100;
fprintf('\n=== Loop Closure Summary ===\n');
fprintf('Loop closure rate: %.2f%% (%d/%d)\n', loop_closure_rate, size(potential_loops, 1), num_nodes);

if loop_closure_rate > 10
    fprintf('✅ Good loop closure detection! This explains why Bio-inspired SLAM performs well.\n');
elseif loop_closure_rate > 5
    fprintf('⚠️  Moderate loop closure detection. Can be improved by lowering VT matching threshold.\n');
else
    fprintf('❌ Very few loop closures! This explains why Bio-inspired SLAM underperforms.\n');
    fprintf('Recommendations:\n');
    fprintf('  1. Lower VT matching threshold (current: 0.080)\n');
    fprintf('  2. Check if scene has loop paths\n');
    fprintf('  3. Increase experience map correction iterations\n');
end

% Save analysis results
result_path = fullfile(data_path, 'slam_results');
save_path = fullfile(result_path, 'loop_closure_analysis.mat');
save(save_path, 'potential_loops', 'dist_matrix', 'loop_closure_rate', 'num_nodes');
fprintf('\nAnalysis results saved: %s\n', save_path);

end
