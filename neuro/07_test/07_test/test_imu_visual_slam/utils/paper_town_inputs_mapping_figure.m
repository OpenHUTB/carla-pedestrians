function out_files = paper_town_inputs_mapping_figure(dataset_name)
%PAPER_TOWN_INPUTS_MAPPING_FIGURE 生成论文风格的输入与建图可视化图
% 左侧：连续帧图像序列（带方向标注）
% 中间：IMU数据
% 右侧：3D轨迹对比
%
% 改进：
% 1. 使用Town01连续帧序列，选择有转弯的关键帧
% 2. 3D轨迹可视化
% 3. 优化布局减少空白
% 4. 改善轨迹对齐

if nargin < 1 || isempty(dataset_name)
    dataset_name = 'Town01Data_IMU_Fusion';
end

this_dir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(fileparts(fileparts(this_dir))));

out_dir = fullfile(rootDir, 'data', 'paper_figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

data_path = resolve_dataset_path(rootDir, dataset_name);
result_path = resolve_results_path(data_path);

fprintf('Processing: %s\n', dataset_name);

% 加载轨迹数据
traj_file = fullfile(result_path, 'trajectories.mat');
S = [];
if exist(traj_file, 'file')
    S = load(traj_file);
end

% 加载IMU数据
imu = read_imu_data(data_path);

% 获取图像文件列表
img_files = dir(fullfile(data_path, '*.png'));
[~, ord] = sort({img_files.name});
img_files = img_files(ord);

%% 选择关键帧（有转弯的帧）
key_frames = select_key_frames_with_turns(imu, numel(img_files));
fprintf('Selected key frames: %s\n', mat2str(key_frames));

%% 创建图形 - 紧凑布局
fig = figure('Color', 'white', 'Position', [50, 50, 1400, 500]);

% ========== 左侧：连续帧图像序列（2x3网格）==========
% 6张关键帧图像，带方向箭头
for i = 1:min(6, numel(key_frames))
    row = ceil(i / 3);
    col = mod(i - 1, 3) + 1;
    
    % 计算子图位置
    x_pos = 0.02 + (col - 1) * 0.14;
    y_pos = 0.52 - (row - 1) * 0.48;
    
    ax = axes('Parent', fig, 'Position', [x_pos, y_pos, 0.13, 0.42]);
    
    frame_idx = key_frames(i);
    if frame_idx <= numel(img_files)
        img = imread(fullfile(data_path, img_files(frame_idx).name));
        imshow(img, 'Parent', ax);
        hold(ax, 'on');
        
        % 计算并绘制运动方向箭头
        if ~isempty(imu) && frame_idx < numel(imu.gyro)
            draw_direction_arrow_on_image(ax, img, imu, frame_idx);
        end
        
        % 帧号标签
        text(ax, 5, 20, sprintf('Frame %d', frame_idx), ...
            'Color', 'white', 'FontSize', 9, 'FontWeight', 'bold', ...
            'BackgroundColor', [0 0 0 0.5]);
    end
    axis(ax, 'off');
end

% 添加"Image Sequence"标题
annotation(fig, 'textbox', [0.02 0.94 0.42 0.05], ...
    'String', 'Input: RGB Image Sequence with Motion Direction', ...
    'FontSize', 11, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center');

% ========== 中间：IMU数据 ==========
ax_imu = axes('Parent', fig, 'Position', [0.48, 0.12, 0.18, 0.78]);
plot_imu_data_compact(ax_imu, imu, key_frames);

% ========== 右侧：3D轨迹对比 ==========
ax_3d = axes('Parent', fig, 'Position', [0.70, 0.10, 0.28, 0.82]);
if ~isempty(S)
    plot_3d_trajectory(ax_3d, S, dataset_name);
else
    text(ax_3d, 0.5, 0.5, 0.5, 'No trajectory data', 'HorizontalAlignment', 'center');
    axis(ax_3d, 'off');
end

%% 保存图片
out_base = fullfile(out_dir, sprintf('paper_%s_inputs_mapping', lower(extractBefore(dataset_name, 'Data'))));
out_png = [out_base '.png'];
out_pdf = [out_base '.pdf'];
out_eps = [out_base '.eps'];
out_files = {out_png, out_pdf, out_eps};

try
    exportgraphics(fig, out_png, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', out_png);
catch
    print(fig, out_png, '-dpng', '-r300');
end
try
    exportgraphics(fig, out_pdf, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', out_pdf);
catch
end
try
    print(fig, out_eps, '-depsc2', '-painters');
    fprintf('Saved: %s\n', out_eps);
catch
end

fprintf('\n✅ 论文图已生成: %s\n', out_png);
end


%% ==================== 选择有转弯的关键帧 ====================
function key_frames = select_key_frames_with_turns(imu, total_frames)
% 选择6个关键帧：包含直行和转弯
if isempty(imu) || total_frames < 100
    % 默认均匀分布
    key_frames = round(linspace(100, total_frames - 100, 6));
    return;
end

% 计算角速度绝对值
gyro_z = abs(imu.gyro(:, 3));
n = min(numel(gyro_z), total_frames);
gyro_z = gyro_z(1:n);

% 平滑处理
window = 50;
gyro_smooth = movmean(gyro_z, window);

% 找到转弯峰值（角速度大的位置）
[~, turn_locs] = findpeaks(gyro_smooth, 'MinPeakHeight', 0.3, ...
    'MinPeakDistance', 200);

% 选择帧：2个直行 + 4个转弯
key_frames = zeros(1, 6);

% 选择转弯帧
if numel(turn_locs) >= 4
    % 均匀选择4个转弯
    turn_idx = round(linspace(1, numel(turn_locs), 4));
    key_frames(3:6) = turn_locs(turn_idx);
else
    % 转弯不够，用均匀分布补充
    key_frames(3:6) = round(linspace(n*0.3, n*0.9, 4));
end

% 选择直行帧（角速度小的位置）
straight_mask = gyro_smooth < 0.1;
straight_idx = find(straight_mask);
if numel(straight_idx) >= 2
    key_frames(1) = straight_idx(round(numel(straight_idx) * 0.2));
    key_frames(2) = straight_idx(round(numel(straight_idx) * 0.5));
else
    key_frames(1) = round(n * 0.1);
    key_frames(2) = round(n * 0.2);
end

% 排序并确保有效
key_frames = sort(key_frames);
key_frames = max(1, min(total_frames, key_frames));
end


%% ==================== 在图像上绘制方向箭头 ====================
function draw_direction_arrow_on_image(ax, img, imu, frame_idx)
[h, w, ~] = size(img);

% 获取当前帧的角速度（yaw rate）
gyro_z = imu.gyro(frame_idx, 3);

% 箭头起点（图像中心偏下）
cx = w / 2;
cy = h * 0.7;

% 根据角速度计算箭头方向
% 正角速度 = 左转，负角速度 = 右转
base_angle = -90;  % 默认向前（向上）
turn_angle = gyro_z * 50;  % 放大角速度效果
arrow_angle = base_angle + turn_angle;

% 箭头长度
arrow_len = min(w, h) * 0.25;

% 计算箭头终点
dx = arrow_len * cosd(arrow_angle);
dy = arrow_len * sind(arrow_angle);

% 绘制箭头
quiver(ax, cx, cy, dx, dy, 0, ...
    'Color', [0.2 0.7 0.3], 'LineWidth', 3, ...
    'MaxHeadSize', 2, 'AutoScale', 'off');

% 添加方向文字
if abs(gyro_z) > 0.3
    if gyro_z > 0
        dir_text = 'Turning Left';
    else
        dir_text = 'Turning Right';
    end
else
    dir_text = 'Straight';
end
text(ax, cx, cy + 30, dir_text, 'Color', [0.2 0.7 0.3], ...
    'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'BackgroundColor', [1 1 1 0.7]);
end


%% ==================== IMU数据绘制（紧凑版）====================
function plot_imu_data_compact(ax, imu, key_frames)
if isempty(imu)
    text(ax, 0.5, 0.5, 'IMU not found', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

n = numel(imu.acc_norm);
t = (1:n) / 10;  % 假设10Hz，转换为秒

hold(ax, 'on');

% 加速度
yyaxis(ax, 'left');
plot(ax, t, imu.acc_norm, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
ylabel(ax, 'Accel (m/s²)', 'FontSize', 9);
set(ax, 'YColor', [0.85 0.33 0.10]);

% 角速度
yyaxis(ax, 'right');
plot(ax, t, imu.gyro(:, 3), '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.0);
ylabel(ax, 'Gyro Z (rad/s)', 'FontSize', 9);
set(ax, 'YColor', [0.0 0.45 0.74]);

% 标记关键帧位置
for i = 1:numel(key_frames)
    frame_t = key_frames(i) / 10;
    xline(ax, frame_t, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
end

xlabel(ax, 'Time (s)', 'FontSize', 9);
title(ax, 'IMU Data', 'FontSize', 11, 'FontWeight', 'bold');
grid(ax, 'on');
set(ax, 'GridAlpha', 0.15, 'Box', 'on', 'FontSize', 8);
legend(ax, {'Accel', 'Gyro Z'}, 'Location', 'northeast', 'FontSize', 7);
end


%% ==================== 3D轨迹绘制 ====================
function plot_3d_trajectory(ax, S, dataset_name)
gt_pos = get_traj(S, {'gt_pos', 'gt_pos_aligned'});
bio_pos = get_traj(S, {'exp_trajectory', 'exp_traj', 'exp_traj_aligned'});

if isempty(gt_pos) || isempty(bio_pos)
    text(ax, 0.5, 0.5, 0.5, 'Missing trajectories', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 确保有3列
if size(gt_pos, 2) < 3
    gt_pos(:, 3) = 0;
end
if size(bio_pos, 2) < 3
    bio_pos(:, 3) = 0;
end

% 对齐长度
n = min(size(gt_pos, 1), size(bio_pos, 1));
gt_pos = gt_pos(1:n, :);
bio_pos = bio_pos(1:n, :);

% 对齐轨迹（Procrustes对齐）
[bio_aligned, ~] = align_trajectory_procrustes(bio_pos, gt_pos);

hold(ax, 'on');

% Ground Truth - 黑色实线
plot3(ax, gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), '-', ...
    'Color', [0.1 0.1 0.1], 'LineWidth', 2.0, 'DisplayName', 'Ground Truth');

% Bio-inspired结果 - 红色线
plot3(ax, bio_aligned(:,1), bio_aligned(:,2), bio_aligned(:,3), '-', ...
    'Color', [0.85 0.2 0.2], 'LineWidth', 1.8, 'DisplayName', 'NeuroLocMap');

% 起点和终点标记
scatter3(ax, gt_pos(1,1), gt_pos(1,2), gt_pos(1,3), 120, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
scatter3(ax, gt_pos(end,1), gt_pos(end,2), gt_pos(end,3), 120, 'r', 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'End');

% 添加方向箭头
add_3d_direction_arrows(ax, gt_pos, [0.1 0.1 0.1]);
add_3d_direction_arrows(ax, bio_aligned, [0.85 0.2 0.2]);

% 设置3D视角
view(ax, 45, 25);
grid(ax, 'on');
set(ax, 'GridAlpha', 0.2, 'Box', 'on', 'FontSize', 9);
xlabel(ax, 'X (m)', 'FontSize', 10);
ylabel(ax, 'Y (m)', 'FontSize', 10);
zlabel(ax, 'Z (m)', 'FontSize', 10);

% 简化标题
town_name = extractBefore(dataset_name, 'Data');
if isempty(town_name)
    town_name = dataset_name;
end
title(ax, sprintf('%s 3D Trajectory', town_name), ...
    'FontSize', 11, 'FontWeight', 'bold');

legend(ax, 'Location', 'best', 'FontSize', 8);
axis(ax, 'equal');

% 调整Z轴范围（如果太小则放大显示）
z_range = max(gt_pos(:,3)) - min(gt_pos(:,3));
if z_range < 5
    z_center = mean(gt_pos(:,3));
    zlim(ax, [z_center - 10, z_center + 10]);
end
end


%% ==================== 3D方向箭头 ====================
function add_3d_direction_arrows(ax, traj, color)
n = size(traj, 1);
num_arrows = 4;
indices = round(linspace(n*0.15, n*0.85, num_arrows));

for i = 1:numel(indices)
    idx = indices(i);
    step = min(20, n - idx);
    if idx + step <= n
        dx = traj(idx+step, 1) - traj(idx, 1);
        dy = traj(idx+step, 2) - traj(idx, 2);
        dz = traj(idx+step, 3) - traj(idx, 3);
        
        % 归一化并放大
        len = sqrt(dx^2 + dy^2 + dz^2);
        if len > 0.1
            scale = 15 / len;
            quiver3(ax, traj(idx,1), traj(idx,2), traj(idx,3), ...
                dx*scale, dy*scale, dz*scale, 0, ...
                'Color', color, 'LineWidth', 1.5, 'MaxHeadSize', 0.8, ...
                'HandleVisibility', 'off');
        end
    end
end
end


%% ==================== Procrustes轨迹对齐 ====================
function [aligned, transform] = align_trajectory_procrustes(source, target)
% 使用Procrustes分析对齐轨迹
% 只使用前N帧进行对齐估计

n_align = min(500, min(size(source, 1), size(target, 1)));

src = source(1:n_align, :);
tgt = target(1:n_align, :);

% 中心化
src_mean = mean(src, 1);
tgt_mean = mean(tgt, 1);
src_centered = src - src_mean;
tgt_centered = tgt - tgt_mean;

% 计算最优旋转（SVD）
[U, ~, V] = svd(src_centered' * tgt_centered);
R = V * U';

% 确保是正交旋转（det = 1）
if det(R) < 0
    V(:, end) = -V(:, end);
    R = V * U';
end

% 计算缩放因子
scale = norm(tgt_centered, 'fro') / norm(src_centered, 'fro');

% 应用变换到整个轨迹
aligned = (source - src_mean) * R' * scale + tgt_mean;

transform.R = R;
transform.scale = scale;
transform.src_mean = src_mean;
transform.tgt_mean = tgt_mean;
end


%% ==================== 辅助函数 ====================
function imu = read_imu_data(data_path)
imu = [];
imu_file = fullfile(data_path, 'aligned_imu.txt');
if ~exist(imu_file, 'file')
    candidates = dir(fullfile(data_path, 'aligned_imu*.txt'));
    if isempty(candidates)
        return;
    end
    imu_file = fullfile(data_path, candidates(1).name);
end
try
    raw = dlmread(imu_file, ',');
    imu.timestamp = raw(:, 1);
    imu.accel = raw(:, 2:4);
    imu.gyro = raw(:, 5:7);
    imu.acc_norm = sqrt(sum(imu.accel.^2, 2));
catch
    imu = [];
end
end

function traj = get_traj(S, keys)
traj = [];
for i = 1:numel(keys)
    k = keys{i};
    if isfield(S, k)
        v = S.(k);
        if isnumeric(v) && size(v, 2) >= 2
            traj = v;
            return;
        end
    end
end
end

function data_path = resolve_dataset_path(rootDir, dataset_name)
candidates = {
    fullfile(rootDir, 'data', dataset_name), ...
    fullfile(rootDir, 'data', '01_NeuroSLAM_Datasets', dataset_name)
};
for i = 1:numel(candidates)
    if exist(candidates{i}, 'dir')
        data_path = candidates{i};
        return;
    end
end
error('Dataset not found: %s', dataset_name);
end

function result_path = resolve_results_path(data_path)
result_path = fullfile(data_path, 'slam_results');
if ~exist(result_path, 'dir')
    result_path = data_path;
end
end
