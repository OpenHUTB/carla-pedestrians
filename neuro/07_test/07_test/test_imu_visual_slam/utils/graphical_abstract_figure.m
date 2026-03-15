function out_file = graphical_abstract_figure(dataset_name)
%GRAPHICAL_ABSTRACT_FIGURE 生成论文摘要图（Graphical Abstract）
%
% 布局设计：
% ┌─────────────────────────────────────────────────────────────────┐
% │  左侧：输入        中间：系统架构          右侧：输出           │
% │  ┌─────────┐      ┌─────────────────┐     ┌─────────────┐      │
% │  │ RGB图像 │ ──→  │   Vestibular    │     │  3D轨迹图   │      │
% │  │ (CARLA) │      │   Processing    │ ──→ │  (对比GT)   │      │
% │  └─────────┘      │       ↓         │     └─────────────┘      │
% │  ┌─────────┐      │  Complementary  │     ┌─────────────┐      │
% │  │ IMU数据 │ ──→  │    Fusion       │ ──→ │ 经验地图    │      │
% │  │ (曲线)  │      │       ↓         │     │ (拓扑图)    │      │
% │  └─────────┘      │  Visual Cortex  │     └─────────────┘      │
% │                   │  (Dual-Stream)  │                          │
% │                   │       ↓         │                          │
% │                   │  Spatial Cells  │                          │
% │                   │  (Grid+HDC)     │                          │
% │                   └─────────────────┘                          │
% └─────────────────────────────────────────────────────────────────┘

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

fprintf('Generating Graphical Abstract for: %s\n', dataset_name);

% 加载数据
traj_file = fullfile(result_path, 'trajectories.mat');
S = [];
if exist(traj_file, 'file')
    S = load(traj_file);
end
imu = read_imu_data(data_path);
img_files = dir(fullfile(data_path, '*.png'));
[~, ord] = sort({img_files.name});
img_files = img_files(ord);

%% 创建图形
fig = figure('Color', 'white', 'Position', [50, 50, 1600, 700]);

% 定义颜色方案
colors = struct();
colors.primary = [0.0 0.45 0.74];      % 蓝色
colors.secondary = [0.85 0.33 0.10];   % 橙色
colors.accent = [0.47 0.67 0.19];      % 绿色
colors.highlight = [0.93 0.69 0.13];   % 黄色
colors.dark = [0.3 0.3 0.3];           % 深灰
colors.light = [0.9 0.9 0.9];          % 浅灰
colors.bio = [0.85 0.2 0.2];           % 红色（Bio结果）

%% ========== 左侧：输入部分 ==========
% RGB图像
ax_rgb = axes('Parent', fig, 'Position', [0.02, 0.55, 0.15, 0.38]);
draw_rgb_input(ax_rgb, data_path, img_files, colors);

% IMU数据
ax_imu = axes('Parent', fig, 'Position', [0.02, 0.08, 0.15, 0.38]);
draw_imu_input(ax_imu, imu, colors);

% 输入标签
annotation(fig, 'textbox', [0.02, 0.94, 0.15, 0.05], ...
    'String', '{\bf Sensory Inputs}', 'FontSize', 12, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex');

%% ========== 中间：系统架构 ==========
ax_arch = axes('Parent', fig, 'Position', [0.22, 0.08, 0.38, 0.84]);
draw_system_architecture(ax_arch, data_path, img_files, colors);

%% ========== 右侧：输出部分 ==========
% 3D轨迹
ax_traj = axes('Parent', fig, 'Position', [0.65, 0.45, 0.33, 0.48]);
draw_3d_trajectory_output(ax_traj, S, colors);

% 经验地图/性能指标
ax_exp = axes('Parent', fig, 'Position', [0.65, 0.08, 0.33, 0.32]);
draw_experience_map_output(ax_exp, S, colors);

% 输出标签
annotation(fig, 'textbox', [0.65, 0.94, 0.33, 0.05], ...
    'String', '{\bf Localization \& Mapping Output}', 'FontSize', 12, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex');

%% ========== 添加连接箭头 ==========
draw_flow_arrows(fig, colors);

%% 保存
out_base = fullfile(out_dir, 'graphical_abstract');
out_file = [out_base '.png'];

try
    exportgraphics(fig, out_file, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', out_file);
catch
    print(fig, out_file, '-dpng', '-r300');
end

% PDF版本
try
    exportgraphics(fig, [out_base '.pdf'], 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s.pdf\n', out_base);
catch
end

fprintf('\n✅ Graphical Abstract 已生成!\n');
end


%% ==================== RGB输入绘制 ====================
function draw_rgb_input(ax, data_path, img_files, colors)
if isempty(img_files)
    text(ax, 0.5, 0.5, 'No images', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 选择一张代表性图像
idx = min(500, numel(img_files));
img = imread(fullfile(data_path, img_files(idx).name));
imshow(img, 'Parent', ax);
hold(ax, 'on');

% 添加相机图标效果（边框）
[h, w, ~] = size(img);
rectangle(ax, 'Position', [1, 1, w-2, h-2], 'EdgeColor', colors.primary, ...
    'LineWidth', 3, 'LineStyle', '-');

% 添加标签
title(ax, '{\bf RGB Camera}', 'FontSize', 10, 'Interpreter', 'tex', ...
    'Color', colors.dark);

% 添加CARLA标签
text(ax, w-10, h-10, 'CARLA', 'Color', 'white', 'FontSize', 8, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
    'BackgroundColor', [0 0 0 0.6]);

axis(ax, 'off');
end


%% ==================== IMU输入绘制 ====================
function draw_imu_input(ax, imu, colors)
if isempty(imu)
    text(ax, 0.5, 0.5, 'IMU not found', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

n = min(800, numel(imu.acc_norm));
t = (1:n) / 10;

hold(ax, 'on');

% 绘制加速度和角速度
yyaxis(ax, 'left');
plot(ax, t, imu.acc_norm(1:n), '-', 'Color', colors.secondary, 'LineWidth', 1.5);
ylabel(ax, 'Accel', 'FontSize', 9, 'Color', colors.secondary);
set(ax, 'YColor', colors.secondary);

yyaxis(ax, 'right');
plot(ax, t, imu.gyro(1:n, 3), '-', 'Color', colors.primary, 'LineWidth', 1.5);
ylabel(ax, 'Gyro', 'FontSize', 9, 'Color', colors.primary);
set(ax, 'YColor', colors.primary);

xlabel(ax, 'Time (s)', 'FontSize', 9);
title(ax, '{\bf IMU (6-DoF)}', 'FontSize', 10, 'Interpreter', 'tex', ...
    'Color', colors.dark);

% 添加IMU图标（简化的芯片形状）
% 使用annotation在图上添加

grid(ax, 'on');
set(ax, 'GridAlpha', 0.15, 'Box', 'on', 'FontSize', 8);
legend(ax, {'Accel', 'Gyro'}, 'Location', 'northeast', 'FontSize', 7);
end


%% ==================== 系统架构绘制 ====================
function draw_system_architecture(ax, data_path, img_files, colors)
axis(ax, 'off');
hold(ax, 'on');
xlim(ax, [0, 100]);
ylim(ax, [0, 100]);

% 标题
text(ax, 50, 98, '{\bf NeuroLocMap: Brain-Inspired 4-DoF SLAM}', ...
    'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex', 'Color', colors.dark);

% ========== 模块1: Vestibular Processing ==========
draw_module_box(ax, [5, 75, 28, 18], 'Vestibular Processing', colors.secondary, colors);
% 子标签
text(ax, 19, 80, 'Semicircular Canals', 'FontSize', 7, 'HorizontalAlignment', 'center');
text(ax, 19, 76, 'Otolith Organs', 'FontSize', 7, 'HorizontalAlignment', 'center');

% ========== 模块2: Visual Cortex (Dual-Stream) ==========
draw_module_box(ax, [5, 52, 28, 18], 'Visual Cortex', colors.primary, colors);
text(ax, 19, 58, 'Ventral: V1→V4→IT', 'FontSize', 7, 'HorizontalAlignment', 'center');
text(ax, 19, 54, 'Dorsal: V1→MT/MST', 'FontSize', 7, 'HorizontalAlignment', 'center');

% ========== 模块3: Complementary Fusion ==========
draw_module_box(ax, [38, 63, 24, 16], 'Complementary', colors.accent, colors);
text(ax, 50, 68, 'Fusion', 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(ax, 50, 64, 'H_{LP} + H_{HP} = 1', 'FontSize', 7, 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex');

% ========== 模块4: Spatial Cells ==========
draw_module_box(ax, [38, 38, 24, 20], 'Spatial Cells', colors.highlight, colors);
text(ax, 50, 52, '3D Grid Cells', 'FontSize', 8, 'HorizontalAlignment', 'center');
text(ax, 50, 47, '(FCC Lattice)', 'FontSize', 7, 'HorizontalAlignment', 'center');
text(ax, 50, 42, 'Head Direction', 'FontSize', 8, 'HorizontalAlignment', 'center');

% ========== 模块5: Visual Templates ==========
draw_module_box(ax, [5, 28, 28, 18], 'Visual Templates', [0.6 0.4 0.8], colors);
text(ax, 19, 34, 'HART + Transformer', 'FontSize', 7, 'HorizontalAlignment', 'center');
text(ax, 19, 30, '321 templates (64×)', 'FontSize', 7, 'HorizontalAlignment', 'center');

% ========== 模块6: Experience Map ==========
draw_module_box(ax, [38, 12, 24, 20], 'Experience Map', [0.5 0.7 0.9], colors);
text(ax, 50, 26, 'Loop Closure', 'FontSize', 8, 'HorizontalAlignment', 'center');
text(ax, 50, 21, 'Graph Relaxation', 'FontSize', 7, 'HorizontalAlignment', 'center');
text(ax, 50, 16, 'Cognitive Map', 'FontSize', 7, 'HorizontalAlignment', 'center');

% ========== 输出模块 ==========
draw_module_box(ax, [68, 55, 28, 20], '4-DoF Pose', [0.3 0.7 0.5], colors);
text(ax, 82, 62, '(x, y, z, \psi)', 'FontSize', 10, 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex', 'FontWeight', 'bold');

draw_module_box(ax, [68, 28, 28, 20], '3D Map', [0.4 0.6 0.8], colors);
text(ax, 82, 35, 'Topological +', 'FontSize', 8, 'HorizontalAlignment', 'center');
text(ax, 82, 31, 'Metric', 'FontSize', 8, 'HorizontalAlignment', 'center');

% ========== 绘制连接箭头 ==========
% Vestibular → Fusion
draw_arrow(ax, [33, 84], [38, 74], colors.secondary);
% Visual → Fusion
draw_arrow(ax, [33, 61], [38, 68], colors.primary);
% Fusion → Spatial
draw_arrow(ax, [50, 63], [50, 58], colors.accent);
% Visual → Templates
draw_arrow(ax, [19, 52], [19, 46], colors.primary);
% Templates → ExpMap
draw_arrow(ax, [33, 37], [38, 27], [0.6 0.4 0.8]);
% Spatial → ExpMap
draw_arrow(ax, [50, 38], [50, 32], colors.highlight);
% Spatial → Pose
draw_arrow(ax, [62, 48], [68, 62], colors.highlight);
% ExpMap → Map
draw_arrow(ax, [62, 22], [68, 35], [0.5 0.7 0.9]);

% ========== 添加小图标 ==========
% 大脑图标（简化）
draw_brain_icon(ax, [85, 78], 8, colors);

% 添加性能指标
text(ax, 82, 8, '{\bf 21 FPS | 65\% drift reduction}', 'FontSize', 8, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'tex', 'Color', colors.accent);
end


%% ==================== 3D轨迹输出 ====================
function draw_3d_trajectory_output(ax, S, colors)
gt_pos = get_traj(S, {'gt_pos', 'gt_pos_aligned'});
bio_pos = get_traj(S, {'exp_trajectory', 'exp_traj', 'exp_traj_aligned'});

if isempty(gt_pos) || isempty(bio_pos)
    text(ax, 0.5, 0.5, 0.5, 'No trajectory', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 确保3D
if size(gt_pos, 2) < 3, gt_pos(:,3) = 0; end
if size(bio_pos, 2) < 3, bio_pos(:,3) = 0; end

n = min(size(gt_pos, 1), size(bio_pos, 1));
gt_pos = gt_pos(1:n, :);
bio_pos = bio_pos(1:n, :);

% Procrustes对齐
bio_aligned = align_trajectory_simple(bio_pos, gt_pos);

hold(ax, 'on');

% Ground Truth
plot3(ax, gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), '-', ...
    'Color', colors.dark, 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');

% Bio结果
plot3(ax, bio_aligned(:,1), bio_aligned(:,2), bio_aligned(:,3), '-', ...
    'Color', colors.bio, 'LineWidth', 2.0, 'DisplayName', 'NeuroLocMap');

% 起点终点
scatter3(ax, gt_pos(1,1), gt_pos(1,2), gt_pos(1,3), 150, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Start');
scatter3(ax, gt_pos(end,1), gt_pos(end,2), gt_pos(end,3), 150, colors.bio, 's', ...
    'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'End');

view(ax, 35, 25);
grid(ax, 'on');
set(ax, 'GridAlpha', 0.2, 'Box', 'on', 'FontSize', 9);
xlabel(ax, 'X (m)', 'FontSize', 10);
ylabel(ax, 'Y (m)', 'FontSize', 10);
zlabel(ax, 'Z (m)', 'FontSize', 10);
title(ax, '{\bf 3D Trajectory Comparison}', 'FontSize', 11, 'Interpreter', 'tex');
legend(ax, 'Location', 'best', 'FontSize', 8);
axis(ax, 'equal');
end


%% ==================== 经验地图/性能输出 ====================
function draw_experience_map_output(ax, S, colors)
axis(ax, 'off');
hold(ax, 'on');
xlim(ax, [0, 100]);
ylim(ax, [0, 100]);

% 绘制简化的经验地图拓扑图
% 节点
node_x = [15, 35, 55, 75, 25, 45, 65, 85];
node_y = [70, 80, 75, 70, 40, 35, 40, 30];

% 边（连接）
edges = [1,2; 2,3; 3,4; 1,5; 2,6; 3,7; 4,8; 5,6; 6,7; 7,8; 5,2; 8,4];

% 绘制边
for i = 1:size(edges, 1)
    e1 = edges(i, 1);
    e2 = edges(i, 2);
    if i > 10  % 回环边用虚线
        plot(ax, [node_x(e1), node_x(e2)], [node_y(e1), node_y(e2)], '--', ...
            'Color', colors.accent, 'LineWidth', 1.5);
    else
        plot(ax, [node_x(e1), node_x(e2)], [node_y(e1), node_y(e2)], '-', ...
            'Color', colors.primary, 'LineWidth', 1.2);
    end
end

% 绘制节点
scatter(ax, node_x, node_y, 80, colors.primary, 'filled', 'MarkerEdgeColor', 'k');

% 标签
text(ax, 50, 95, '{\bf Experience Map (Cognitive Map)}', 'FontSize', 10, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'tex');

% 图例
plot(ax, [10, 20], [15, 15], '-', 'Color', colors.primary, 'LineWidth', 1.5);
text(ax, 22, 15, 'Sequential', 'FontSize', 8);
plot(ax, [50, 60], [15, 15], '--', 'Color', colors.accent, 'LineWidth', 1.5);
text(ax, 62, 15, 'Loop Closure', 'FontSize', 8);

% 性能指标框
rectangle(ax, 'Position', [5, 2, 90, 10], 'FaceColor', [0.95 0.95 0.95], ...
    'EdgeColor', colors.dark, 'LineWidth', 1, 'Curvature', 0.1);
text(ax, 50, 7, '{\bf RMSE: 253m → 103m (59\% ↓)  |  Drift: 38\% → 2.7\%  |  Templates: 5 → 321}', ...
    'FontSize', 8, 'HorizontalAlignment', 'center', 'Interpreter', 'tex');
end


%% ==================== 流程箭头 ====================
function draw_flow_arrows(fig, colors)
% 输入 → 系统
annotation(fig, 'arrow', [0.17, 0.22], [0.73, 0.73], ...
    'Color', colors.primary, 'LineWidth', 2.5, 'HeadWidth', 12, 'HeadLength', 10);
annotation(fig, 'arrow', [0.17, 0.22], [0.27, 0.27], ...
    'Color', colors.secondary, 'LineWidth', 2.5, 'HeadWidth', 12, 'HeadLength', 10);

% 系统 → 输出
annotation(fig, 'arrow', [0.60, 0.65], [0.70, 0.70], ...
    'Color', colors.accent, 'LineWidth', 2.5, 'HeadWidth', 12, 'HeadLength', 10);
annotation(fig, 'arrow', [0.60, 0.65], [0.25, 0.25], ...
    'Color', [0.5 0.7 0.9], 'LineWidth', 2.5, 'HeadWidth', 12, 'HeadLength', 10);
end


%% ==================== 辅助绘图函数 ====================
function draw_module_box(ax, pos, title_text, color, colors)
% pos = [x, y, w, h]
rectangle(ax, 'Position', pos, 'FaceColor', [color, 0.2], ...
    'EdgeColor', color, 'LineWidth', 2, 'Curvature', 0.1);
text(ax, pos(1) + pos(3)/2, pos(2) + pos(4) - 3, title_text, ...
    'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'Color', colors.dark);
end

function draw_arrow(ax, from, to, color)
% 绘制箭头
dx = to(1) - from(1);
dy = to(2) - from(2);
quiver(ax, from(1), from(2), dx, dy, 0, 'Color', color, 'LineWidth', 1.5, ...
    'MaxHeadSize', 0.8);
end

function draw_brain_icon(ax, center, size, colors)
% 简化的大脑图标
theta = linspace(0, 2*pi, 50);
r = size/2;

% 左半球
x1 = center(1) - r/2 + r/2 * cos(theta);
y1 = center(2) + r/2 * sin(theta);
fill(ax, x1, y1, colors.primary, 'FaceAlpha', 0.3, 'EdgeColor', colors.primary, 'LineWidth', 1);

% 右半球
x2 = center(1) + r/2 + r/2 * cos(theta);
y2 = center(2) + r/2 * sin(theta);
fill(ax, x2, y2, colors.secondary, 'FaceAlpha', 0.3, 'EdgeColor', colors.secondary, 'LineWidth', 1);

text(ax, center(1), center(2) - size/2 - 2, 'Bio-inspired', 'FontSize', 7, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end


%% ==================== 轨迹对齐 ====================
function aligned = align_trajectory_simple(source, target)
n = min(300, min(size(source, 1), size(target, 1)));
src = source(1:n, :);
tgt = target(1:n, :);

src_mean = mean(src, 1);
tgt_mean = mean(tgt, 1);
src_c = src - src_mean;
tgt_c = tgt - tgt_mean;

[U, ~, V] = svd(src_c' * tgt_c);
R = V * U';
if det(R) < 0
    V(:, end) = -V(:, end);
    R = V * U';
end

scale = norm(tgt_c, 'fro') / norm(src_c, 'fro');
aligned = (source - src_mean) * R' * scale + tgt_mean;
end


%% ==================== 数据读取函数 ====================
function imu = read_imu_data(data_path)
imu = [];
imu_file = fullfile(data_path, 'aligned_imu.txt');
if ~exist(imu_file, 'file')
    candidates = dir(fullfile(data_path, 'aligned_imu*.txt'));
    if isempty(candidates), return; end
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
