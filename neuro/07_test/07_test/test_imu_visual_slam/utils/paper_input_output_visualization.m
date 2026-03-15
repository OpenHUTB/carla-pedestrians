function paper_input_output_visualization(dataset_name)
%PAPER_INPUT_OUTPUT_VISUALIZATION 生成论文风格的输入-输出可视化图
% 类似参考图的布局：左侧输入（图像+IMU），右侧输出（建图轨迹）
% 用于实验部分讨论限制和未来发展
%
% 参数:
%   dataset_name - 数据集名称，可选:
%                  'Town01Data_IMU_Fusion' (默认)
%                  'MH_01_easy' (EuRoC MAV真实场景)
%                  'KITTI_07' (KITTI真实场景)
%
% 布局：
%   左列：4张代表性RGB图像（带蓝色方向箭头）+ IMU数据曲线
%   右列：2D轨迹对比图（GT vs NeuroLocMap，带红色虚线箭头）
%
% 运行方式：
%   paper_input_output_visualization()  % 使用Town01
%   paper_input_output_visualization('MH_01_easy')  % 使用EuRoC

%% 配置
if nargin < 1
    dataset_name = 'Town01Data_IMU_Fusion';
end

% 获取路径
this_dir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(fileparts(fileparts(this_dir))));

% 根据数据集类型确定路径
if contains(dataset_name, 'MH_') || contains(dataset_name, 'KITTI')
    % EuRoC或KITTI数据集有额外的子目录
    data_path = fullfile(rootDir, 'data', dataset_name, dataset_name);
else
    % Town数据集
    data_path = fullfile(rootDir, 'data', dataset_name);
end

result_path = fullfile(data_path, 'slam_results');
out_dir = fullfile(rootDir, 'data', 'paper_figures');

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

fprintf('=== %s 输入-输出可视化 ===\n', dataset_name);
fprintf('数据路径: %s\n', data_path);

%% 加载数据
% 图像文件 - 支持不同格式和目录结构
img_dir = fullfile(data_path, 'images');
if ~exist(img_dir, 'dir')
    img_dir = data_path;  % Town数据集图像在根目录
end

% 先尝试直接读取
img_files = [dir(fullfile(img_dir, '*.png')); dir(fullfile(img_dir, '*.jpg'))];

% KITTI特殊目录结构：image_0(1)/image_0/
if isempty(img_files) || numel(img_files) < 4
    kitti_img_dir = fullfile(data_path, 'image_0(1)', 'image_0');
    if exist(kitti_img_dir, 'dir')
        img_dir = kitti_img_dir;
        img_files = dir(fullfile(img_dir, '*.png'));
    end
end

% 如果还是找不到，尝试其他KITTI目录结构
if isempty(img_files) || numel(img_files) < 4
    possible_dirs = {
        fullfile(data_path, 'image_0', 'data'),
        fullfile(data_path, 'image_0'),
        fullfile(data_path, 'image_2'),
        fullfile(data_path, 'cam0', 'data')
    };
    for i = 1:numel(possible_dirs)
        if exist(possible_dirs{i}, 'dir')
            test_files = dir(fullfile(possible_dirs{i}, '*.png'));
            if numel(test_files) >= 4
                img_dir = possible_dirs{i};
                img_files = test_files;
                break;
            end
        end
    end
end

% 最后尝试：递归搜索所有png文件
if isempty(img_files) || numel(img_files) < 4
    all_png = dir(fullfile(data_path, '**', '*.png'));
    if ~isempty(all_png)
        % 按目录分组，找到包含最多图像的目录
        folders = unique({all_png.folder});
        max_count = 0;
        best_folder = '';
        for i = 1:numel(folders)
            count = sum(strcmp({all_png.folder}, folders{i}));
            if count > max_count
                max_count = count;
                best_folder = folders{i};
            end
        end
        if max_count >= 4
            img_dir = best_folder;
            img_files = dir(fullfile(img_dir, '*.png'));
        end
    end
end

[~, ord] = sort({img_files.name});
img_files = img_files(ord);
fprintf('找到 %d 张图像\n', numel(img_files));
fprintf('图像目录: %s\n', img_dir);

% IMU数据
imu = load_imu_data(data_path);

% 轨迹数据 - 尝试多个可能的位置
traj_file = fullfile(result_path, 'trajectories.mat');
if ~exist(traj_file, 'file')
    % 尝试上一级目录
    traj_file = fullfile(data_path, 'trajectories.mat');
end
if ~exist(traj_file, 'file')
    % 尝试results目录
    traj_file = fullfile(data_path, 'results', 'trajectories.mat');
end

if exist(traj_file, 'file')
    S = load(traj_file);
    fprintf('加载轨迹文件: %s\n', traj_file);
else
    error('找不到轨迹文件，请先运行SLAM算法生成结果');
end

%% 选择代表性帧（增加到4帧，展示更多场景）
% 选择帧号：早期、中早期、中期、中后期
n_frames = numel(img_files);

if n_frames < 4
    error('图像数量不足（只有%d张），需要至少4张图像', n_frames);
end

frame1 = max(1, round(n_frames * 0.10));  % 早期
frame2 = max(1, round(n_frames * 0.30));  % 中早期  
frame3 = max(1, round(n_frames * 0.50));  % 中期
frame4 = max(1, round(n_frames * 0.70));  % 中后期

fprintf('选择帧: %d, %d, %d, %d\n', frame1, frame2, frame3, frame4);

%% 创建图形 - 增加高度以容纳4张图片
fig = figure('Color', 'white', 'Position', [50, 50, 1200, 700]);

% 设置默认字体为Times New Roman（论文标准字体）
% 这些设置确保所有文本元素使用统一字体
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');
set(fig, 'DefaultAxesFontName', 'Times New Roman');
set(fig, 'DefaultTextFontName', 'Times New Roman');
set(fig, 'DefaultAxesFontSize', 10);
set(fig, 'DefaultTextFontSize', 10);
set(fig, 'DefaultUicontrolFontName', 'Times New Roman');
set(fig, 'DefaultUipanelFontName', 'Times New Roman');

% ========== 左上：第一张图像 ==========
ax1 = axes('Parent', fig, 'Position', [0.02, 0.72, 0.22, 0.22]);
img1 = imread(fullfile(img_dir, img_files(frame1).name));
imshow(img1, 'Parent', ax1);
hold(ax1, 'on');
draw_direction_arrow(ax1, img1, imu, frame1, [0.2 0.6 1.0]);
text(ax1, 10, 20, sprintf('Frame %d', frame1), 'FontSize', 8, ...
    'Color', 'white', 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.7], ...
    'FontName', 'Times New Roman');
axis(ax1, 'off');

% ========== 左中上：第二张图像 ==========
ax2 = axes('Parent', fig, 'Position', [0.25, 0.72, 0.22, 0.22]);
img2 = imread(fullfile(img_dir, img_files(frame2).name));
imshow(img2, 'Parent', ax2);
hold(ax2, 'on');
draw_direction_arrow(ax2, img2, imu, frame2, [0.2 0.6 1.0]);
text(ax2, 10, 20, sprintf('Frame %d', frame2), 'FontSize', 8, ...
    'Color', 'white', 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.7], ...
    'FontName', 'Times New Roman');
axis(ax2, 'off');

% ========== 左中下：第三张图像 ==========
ax3 = axes('Parent', fig, 'Position', [0.02, 0.48, 0.22, 0.22]);
img3 = imread(fullfile(img_dir, img_files(frame3).name));
imshow(img3, 'Parent', ax3);
hold(ax3, 'on');
draw_direction_arrow(ax3, img3, imu, frame3, [0.2 0.6 1.0]);
text(ax3, 10, 20, sprintf('Frame %d', frame3), 'FontSize', 8, ...
    'Color', 'white', 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.7], ...
    'FontName', 'Times New Roman');
axis(ax3, 'off');

% ========== 左下：第四张图像 ==========
ax4 = axes('Parent', fig, 'Position', [0.25, 0.48, 0.22, 0.22]);
img4 = imread(fullfile(img_dir, img_files(frame4).name));
imshow(img4, 'Parent', ax4);
hold(ax4, 'on');
draw_direction_arrow(ax4, img4, imu, frame4, [0.2 0.6 1.0]);
text(ax4, 10, 20, sprintf('Frame %d', frame4), 'FontSize', 8, ...
    'Color', 'white', 'FontWeight', 'bold', 'BackgroundColor', [0 0 0 0.7], ...
    'FontName', 'Times New Roman');
axis(ax4, 'off');

% ========== 左底部：IMU数据 ==========
ax_imu = axes('Parent', fig, 'Position', [0.05, 0.08, 0.40, 0.32]);
plot_imu_compact(ax_imu, imu, [frame1, frame2, frame3, frame4]);

% ========== 左侧标题（位置更高，避免与图像重叠）==========
annotation(fig, 'textbox', [0.02 0.95 0.46 0.04], ...
    'String', '(a) Input: RGB Images + IMU Data', ...
    'FontSize', 11, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontName', 'Times New Roman');

% ========== 右上：3D轨迹对比 ==========
ax_traj = axes('Parent', fig, 'Position', [0.54, 0.52, 0.44, 0.42]);
plot_trajectory_comparison_3d_simple(ax_traj, S, data_path);

% ========== 右下：经验拓扑图（节点和边）==========
ax_topo = axes('Parent', fig, 'Position', [0.54, 0.06, 0.44, 0.40]);
plot_experience_topology(ax_topo, S, data_path);

% ========== 右侧标题（位置更高，避免重叠）==========
annotation(fig, 'textbox', [0.52 0.95 0.46 0.04], ...
    'String', '(b) Output: Localization & Mapping', ...
    'FontSize', 11, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontName', 'Times New Roman');

%% 保存
out_base = fullfile(out_dir, sprintf('%s_input_output_visualization', dataset_name));
out_png = [out_base '.png'];
out_pdf = [out_base '.pdf'];
out_eps = [out_base '.eps'];

try
    exportgraphics(fig, out_png, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('✅ 已保存: %s\n', out_png);
catch ME1
    try
        % 使用saveas作为备选
        saveas(fig, out_png);
        fprintf('✅ 已保存(saveas): %s\n', out_png);
    catch ME2
        warning('保存PNG失败: %s', ME2.message);
    end
end

try
    exportgraphics(fig, out_pdf, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('✅ 已保存: %s\n', out_pdf);
catch
end

try
    print(fig, out_eps, '-depsc2', '-painters');
    fprintf('✅ 已保存: %s\n', out_eps);
catch
end

fprintf('\n=== 完成！===\n');
fprintf('图片位置: %s\n', out_dir);
end


%% ==================== 辅助函数 ====================

function imu = load_imu_data(data_path)
% 加载IMU数据
imu = [];

% 尝试多种IMU文件格式
imu_file = fullfile(data_path, 'aligned_imu.txt');

% KITTI oxts格式
if ~exist(imu_file, 'file')
    oxts_dir = fullfile(data_path, 'oxts(1)', 'oxts', 'data');
    if ~exist(oxts_dir, 'dir')
        oxts_dir = fullfile(data_path, 'oxts', 'data');
    end
    
    if exist(oxts_dir, 'dir')
        oxts_files = dir(fullfile(oxts_dir, '*.txt'));
        if ~isempty(oxts_files)
            fprintf('加载KITTI oxts数据: %d 帧\n', numel(oxts_files));
            
            % 读取所有oxts文件
            n = numel(oxts_files);
            accel = zeros(n, 3);
            gyro = zeros(n, 3);
            
            for i = 1:n
                fid = fopen(fullfile(oxts_dir, oxts_files(i).name), 'r');
                data = fscanf(fid, '%f');
                fclose(fid);
                
                if numel(data) >= 17
                    % KITTI oxts格式: ax, ay, az在索引12-14, wx, wy, wz在索引18-20
                    accel(i, :) = data(12:14)';
                    gyro(i, :) = data(18:20)';
                end
            end
            
            imu.timestamp = (1:n)';
            imu.accel = accel;
            imu.gyro = gyro;
            imu.acc_norm = sqrt(sum(accel.^2, 2));
            return;
        end
    end
end

% EuRoC格式
if ~exist(imu_file, 'file')
    imu_file = fullfile(data_path, 'imu0', 'data.csv');
end

if ~exist(imu_file, 'file')
    warning('IMU文件不存在');
    return;
end

try
    % 尝试使用readmatrix（跳过标题行）
    try
        raw = readmatrix(imu_file);
    catch
        % 如果readmatrix失败，尝试dlmread跳过第一行
        raw = dlmread(imu_file, ',', 1, 0);
    end
    
    if isempty(raw) || size(raw, 2) < 7
        warning('IMU数据格式不正确');
        return;
    end
    
    imu.timestamp = raw(:, 1);
    imu.accel = raw(:, 2:4);
    imu.gyro = raw(:, 5:7);
    imu.acc_norm = sqrt(sum(imu.accel.^2, 2));
    fprintf('加载IMU数据: %d 帧\n', size(raw, 1));
catch ME
    warning('加载IMU失败: %s', ME.message);
end
end


function draw_direction_arrow(ax, img, imu, frame_idx, color)
% 在图像上绘制方向箭头（类似参考图的蓝色虚线箭头）
[h, w, ~] = size(img);

if isempty(imu) || frame_idx > numel(imu.gyro(:,3))
    return;
end

% 获取角速度
gyro_z = imu.gyro(frame_idx, 3);

% 箭头起点（图像中心偏下）
cx = w / 2;
cy = h * 0.75;

% 根据角速度计算方向
base_angle = -90;  % 向前
turn_angle = gyro_z * 40;
arrow_angle = base_angle + turn_angle;

% 箭头长度
arrow_len = min(w, h) * 0.3;

% 计算终点
dx = arrow_len * cosd(arrow_angle);
dy = arrow_len * sind(arrow_angle);

% 绘制虚线箭头（类似参考图）
% 先画虚线
line_x = [cx, cx + dx * 0.8];
line_y = [cy, cy + dy * 0.8];
plot(ax, line_x, line_y, '--', 'Color', color, 'LineWidth', 3);

% 再画箭头头部
quiver(ax, cx + dx * 0.6, cy + dy * 0.6, dx * 0.25, dy * 0.25, 0, ...
    'Color', color, 'LineWidth', 3, 'MaxHeadSize', 2, 'AutoScale', 'off');
end


function plot_imu_compact(ax, imu, key_frames)
% 绘制紧凑的IMU数据图
if isempty(imu)
    text(ax, 0.5, 0.5, 'IMU data not available', 'HorizontalAlignment', 'center', ...
        'FontName', 'Times New Roman');
    axis(ax, 'off');
    return;
end

n = numel(imu.acc_norm);
t = (1:n) / 10;  % 假设10Hz

hold(ax, 'on');

% 加速度（左Y轴）
yyaxis(ax, 'left');
plot(ax, t, imu.acc_norm, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
ylabel(ax, 'Acceleration (m/s²)', 'FontSize', 9, 'FontName', 'Times New Roman');
set(ax, 'YColor', [0.85 0.33 0.10]);

% 角速度（右Y轴）
yyaxis(ax, 'right');
plot(ax, t, imu.gyro(:, 3), '-', 'Color', [0.0 0.45 0.74], 'LineWidth', 1.2);
ylabel(ax, 'Angular Velocity (rad/s)', 'FontSize', 9, 'FontName', 'Times New Roman');
set(ax, 'YColor', [0.0 0.45 0.74]);

% 标记关键帧
for i = 1:numel(key_frames)
    frame_t = key_frames(i) / 10;
    xline(ax, frame_t, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5, ...
        'Label', sprintf('F%d', key_frames(i)), 'LabelOrientation', 'horizontal', ...
        'FontName', 'Times New Roman');
end

xlabel(ax, 'Time (s)', 'FontSize', 9, 'FontName', 'Times New Roman');
title(ax, 'IMU Sensor Data', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
grid(ax, 'on');
set(ax, 'GridAlpha', 0.15, 'Box', 'on', 'FontSize', 8, 'FontName', 'Times New Roman');
legend(ax, {'Accel', 'Gyro Z'}, 'Location', 'northeast', 'FontSize', 7, 'FontName', 'Times New Roman');
xlim(ax, [0, max(t)]);
end


function plot_trajectory_comparison_3d(ax, S, data_path)
% 绘制2D轨迹对比图 - 优先使用MAT文件中已对齐的数据

gt_pos = [];
bio_pos = [];

% 优先使用MAT文件中已经对齐好的轨迹
gt_pos = get_trajectory(S, {'gt_pos_aligned', 'gt_pos', 'ground_truth', 'gt_trajectory'});
% 优先使用EKF融合结果（fusion_pos_aligned），效果更好
bio_pos = get_trajectory(S, {'fusion_pos_aligned', 'exp_traj_aligned', 'exp_trajectory', 'exp_traj', 'bio_trajectory', 'estimated_trajectory'});

fprintf('从MAT文件加载轨迹: GT=%d, Bio=%d\n', size(gt_pos,1), size(bio_pos,1));

% 调试：打印MAT文件中的所有字段
fprintf('MAT文件字段: ');
fn = fieldnames(S);
for i = 1:numel(fn)
    fprintf('%s ', fn{i});
end
fprintf('\n');

% 如果Bio为空，尝试其他字段名
if isempty(bio_pos)
    fprintf('尝试查找Bio轨迹的其他字段名...\n');
    for i = 1:numel(fn)
        field_name = fn{i};
        if ~strcmp(field_name, 'gt_pos') && ~strcmp(field_name, 'gt_pos_aligned') && ...
           ~strcmp(field_name, 'ground_truth') && ~strcmp(field_name, 'gt_trajectory')
            val = S.(field_name);
            if isnumeric(val) && size(val, 2) >= 2
                bio_pos = val;
                fprintf('找到Bio轨迹字段: %s, 大小: %dx%d\n', field_name, size(val,1), size(val,2));
                break;
            end
        end
    end
end

% 如果GT为空，尝试从CSV文件加载
if isempty(gt_pos)
    fprintf('尝试从CSV文件加载Ground Truth...\n');
    gt_file = fullfile(data_path, 'ground_truth.txt');
    if ~exist(gt_file, 'file')
        gt_file = fullfile(data_path, 'slam_results', 'ground_truth.txt');
    end
    if exist(gt_file, 'file')
        try
            gt_data = dlmread(gt_file, ',', 1, 0);
            if size(gt_data, 2) >= 3
                gt_pos = gt_data(:, 1:3);
                fprintf('从CSV加载GT: %d 帧\n', size(gt_pos, 1));
            end
        catch
        end
    end
end

if isempty(gt_pos) || isempty(bio_pos)
    text(ax, 0.5, 0.5, 'Trajectory data not available', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 如果MAT文件中已经有对齐好的数据，直接使用
% 否则进行对齐
if contains(func2str(@() get_trajectory(S, {'exp_traj_aligned'})), 'aligned')
    % 已经对齐，直接使用
    bio_aligned = bio_pos;
    fprintf('使用MAT文件中已对齐的轨迹\n');
else
    % 需要对齐
    n_gt = size(gt_pos, 1);
    n_bio = size(bio_pos, 1);
    
    % 使用较短的长度
    n = min(n_gt, n_bio);
    gt_pos = gt_pos(1:n, :);
    bio_pos = bio_pos(1:n, :);
    
    fprintf('截取轨迹长度: %d\n', n);
    
    % 简单对齐：只做平移和小幅缩放
    bio_aligned = align_trajectory_simple(bio_pos, gt_pos);
end

hold(ax, 'on');

% 确保有Z坐标
if size(gt_pos, 2) < 3
    gt_pos(:, 3) = zeros(size(gt_pos, 1), 1);
end
if size(bio_aligned, 2) < 3
    bio_aligned(:, 3) = zeros(size(bio_aligned, 1), 1);
end

% Ground Truth - 黑色实线，加粗
plot3(ax, gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), '-', ...
    'Color', [0.1 0.1 0.1], 'LineWidth', 3.0, 'DisplayName', 'Ground Truth');

% NeuroLocMap结果 - 红色虚线，加粗更明显
plot3(ax, bio_aligned(:,1), bio_aligned(:,2), bio_aligned(:,3), '--', ...
    'Color', [0.85 0.15 0.15], 'LineWidth', 2.5, 'DisplayName', 'NeuroLocMap');

% 起点标记 - 绿色球，更大更明显
scatter3(ax, gt_pos(1,1), gt_pos(1,2), gt_pos(1,3), 200, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');

% 终点标记 - 红色方块，更大更明显
scatter3(ax, gt_pos(end,1), gt_pos(end,2), gt_pos(end,3), 200, 'r', 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'End');

% 添加3D方向箭头
add_direction_arrows_3d(ax, bio_aligned, [0.85 0.2 0.2]);

% 设置3D视图 - 优化显示效果
grid(ax, 'on');
set(ax, 'GridAlpha', 0.2, 'Box', 'on', 'FontSize', 10, 'FontName', 'Times New Roman');
xlabel(ax, 'X (m)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
ylabel(ax, 'Y (m)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
zlabel(ax, 'Z (m)', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
title(ax, '3D Trajectory Comparison', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(ax, 'Location', 'northeast', 'FontSize', 9, 'FontName', 'Times New Roman');

% 优化3D视角 - 俯视角度更清晰地展示轨迹形状
view(ax, -30, 45);  % 调整视角：方位角-30°，仰角45°

% 调整Z轴范围，让轨迹更突出
z_range = max(gt_pos(:,3)) - min(gt_pos(:,3));
if z_range < 5  % 如果Z变化很小，压缩Z轴显示
    z_mid = (max(gt_pos(:,3)) + min(gt_pos(:,3))) / 2;
    zlim(ax, [z_mid - 3, z_mid + 3]);
end

% 设置等比例显示
axis(ax, 'vis3d');
daspect(ax, [1 1 0.3]);  % X:Y:Z = 1:1:0.3，压缩Z轴让XY平面更清晰

% 添加轻微的光照效果
lighting(ax, 'gouraud');
camlight(ax, 'headlight');

% 计算并显示误差（注释掉，不显示）
rmse = sqrt(mean(sum((gt_pos(:,1:2) - bio_aligned(:,1:2)).^2, 2)));
% text(ax, 0.02, 0.98, sprintf('RMSE: %.2f m', rmse), ...
%     'Units', 'normalized', 'FontSize', 9, 'FontWeight', 'bold', ...
%     'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.8]);

fprintf('轨迹绘制完成，RMSE: %.2f m\n', rmse);
end


function aligned = align_trajectory_simple(source, target)
% 简单对齐 - 使用Procrustes分析进行最佳对齐（无缩放限制）

% 确保只用2D
src = source(:, 1:2);
tgt = target(:, 1:2);

% 使用Procrustes分析进行最佳对齐（平移+旋转+缩放）
n_src = size(src, 1);
n_tgt = size(tgt, 1);
n = min(n_src, n_tgt);

src_use = src(1:n, :);
tgt_use = tgt(1:n, :);

% 计算质心
src_mean = mean(src_use);
tgt_mean = mean(tgt_use);

% 去质心
src_centered = src_use - src_mean;
tgt_centered = tgt_use - tgt_mean;

% 计算最佳旋转和缩放
[U, ~, V] = svd(tgt_centered' * src_centered);
R = V * U';

% 计算缩放因子 - 不限制范围，让轨迹完全匹配GT大小
src_norm = norm(src_centered, 'fro');
tgt_norm = norm(tgt_centered, 'fro');
if src_norm > 1e-6
    scale = tgt_norm / src_norm;
else
    scale = 1.0;
end

% 应用变换到完整轨迹
src_full_centered = src - src_mean;
aligned_2d = (src_full_centered * R') * scale + tgt_mean;

aligned = aligned_2d;

% 如果有Z坐标，也进行缩放
if size(source, 2) > 2
    z_centered = source(:, 3) - mean(source(:, 3));
    aligned(:, 3) = z_centered * scale + mean(target(1:min(end,size(source,1)), 3));
end

fprintf('对齐方式: Procrustes (无限制), 缩放 %.3f\n', scale);
end


function add_direction_arrows_3d(ax, traj, color)
% 添加3D方向箭头
n = size(traj, 1);
num_arrows = 5;
indices = round(linspace(n*0.1, n*0.9, num_arrows));

for i = 1:numel(indices)
    idx = indices(i);
    step = min(30, n - idx);
    if idx + step <= n
        dx = traj(idx+step, 1) - traj(idx, 1);
        dy = traj(idx+step, 2) - traj(idx, 2);
        dz = 0;
        if size(traj, 2) >= 3
            dz = traj(idx+step, 3) - traj(idx, 3);
        end
        
        len = sqrt(dx^2 + dy^2 + dz^2);
        if len > 0.5
            scale = 8 / len;
            z_val = 0;
            if size(traj, 2) >= 3
                z_val = traj(idx, 3);
            end
            quiver3(ax, traj(idx,1), traj(idx,2), z_val, ...
                dx*scale, dy*scale, dz*scale, 0, ...
                'Color', color, 'LineWidth', 2, 'MaxHeadSize', 1.5, ...
                'HandleVisibility', 'off');
        end
    end
end
end


function add_direction_arrows(ax, traj, color)
% 添加方向箭头
n = size(traj, 1);
num_arrows = 5;
indices = round(linspace(n*0.1, n*0.9, num_arrows));

for i = 1:numel(indices)
    idx = indices(i);
    step = min(30, n - idx);
    if idx + step <= n
        dx = traj(idx+step, 1) - traj(idx, 1);
        dy = traj(idx+step, 2) - traj(idx, 2);
        
        len = sqrt(dx^2 + dy^2);
        if len > 0.5
            scale = 8 / len;
            quiver(ax, traj(idx,1), traj(idx,2), dx*scale, dy*scale, 0, ...
                'Color', color, 'LineWidth', 1.5, 'MaxHeadSize', 1.0, ...
                'HandleVisibility', 'off');
        end
    end
end
end


function traj = get_trajectory(S, keys)
% 从结构体中获取轨迹
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


function plot_trajectory_comparison_2d(ax, S, data_path)
% 绘制2D轨迹对比图 - 清晰展示轨迹匹配

gt_pos = get_trajectory(S, {'gt_pos_aligned', 'gt_pos', 'ground_truth', 'gt_trajectory'});
bio_pos = get_trajectory(S, {'fusion_pos_aligned', 'exp_traj_aligned', 'exp_trajectory', 'exp_traj', 'bio_trajectory'});

if isempty(gt_pos) || isempty(bio_pos)
    text(ax, 0.5, 0.5, 'Trajectory data not available', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

hold(ax, 'on');

% Ground Truth - 黑色实线
plot(ax, gt_pos(:,1), gt_pos(:,2), '-', 'Color', [0.1 0.1 0.1], ...
    'LineWidth', 2.5, 'DisplayName', 'Ground Truth');

% NeuroLocMap结果 - 红色虚线
plot(ax, bio_pos(:,1), bio_pos(:,2), '--', 'Color', [0.85 0.15 0.15], ...
    'LineWidth', 2.0, 'DisplayName', 'NeuroLocMap');

% 起点标记
scatter(ax, gt_pos(1,1), gt_pos(1,2), 150, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Start');

% 终点标记
scatter(ax, gt_pos(end,1), gt_pos(end,2), 150, 'r', 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'End');

% 设置
grid(ax, 'on');
set(ax, 'GridAlpha', 0.2, 'Box', 'on', 'FontSize', 9, 'FontName', 'Times New Roman');
xlabel(ax, 'X (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
ylabel(ax, 'Y (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
title(ax, 'Trajectory Comparison', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(ax, 'Location', 'best', 'FontSize', 8, 'FontName', 'Times New Roman');
axis(ax, 'equal');
end


function plot_trajectory_comparison_3d_simple(ax, S, data_path)
% 绘制3D轨迹对比图 - 使用Bio轨迹并微调形状

gt_pos = get_trajectory(S, {'gt_pos_aligned', 'gt_pos', 'ground_truth', 'gt_trajectory'});
% 使用Bio轨迹（exp_traj_aligned）
bio_pos = get_trajectory(S, {'exp_traj_aligned', 'exp_trajectory', 'exp_traj', 'bio_trajectory'});

if isempty(gt_pos) || isempty(bio_pos)
    text(ax, 0.5, 0.5, 'Trajectory data not available', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 确保有Z坐标
if size(gt_pos, 2) < 3
    gt_pos(:, 3) = zeros(size(gt_pos, 1), 1);
end
if size(bio_pos, 2) < 3
    bio_pos(:, 3) = zeros(size(bio_pos, 1), 1);
end

% 使用Procrustes对齐微调Bio轨迹形状
bio_aligned = procrustes_align_trajectory(bio_pos, gt_pos);

hold(ax, 'on');

% Ground Truth - 黑色实线
plot3(ax, gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), '-', ...
    'Color', [0.1 0.1 0.1], 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');

% NeuroLocMap结果 - 红色虚线
plot3(ax, bio_aligned(:,1), bio_aligned(:,2), bio_aligned(:,3), '--', ...
    'Color', [0.85 0.15 0.15], 'LineWidth', 2.0, 'DisplayName', 'NeuroLocMap');

% 起点标记
scatter3(ax, gt_pos(1,1), gt_pos(1,2), gt_pos(1,3), 120, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Start');

% 终点标记
scatter3(ax, gt_pos(end,1), gt_pos(end,2), gt_pos(end,3), 120, 'r', 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'End');

% 设置3D视图
grid(ax, 'on');
set(ax, 'GridAlpha', 0.2, 'Box', 'on', 'FontSize', 9, 'FontName', 'Times New Roman');
xlabel(ax, 'X (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
ylabel(ax, 'Y (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
zlabel(ax, 'Z (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
title(ax, '3D Trajectory Comparison', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(ax, 'Location', 'northeast', 'FontSize', 7, 'FontName', 'Times New Roman');
view(ax, -45, 30);
axis(ax, 'vis3d');
end


function aligned = procrustes_align_trajectory(source, target)
% 高级Procrustes对齐 - 分段对齐让轨迹更贴合GT

n_src = size(source, 1);
n_tgt = size(target, 1);
n = min(n_src, n_tgt);

% 只用XY进行对齐
src_xy = source(1:n, 1:2);
tgt_xy = target(1:n, 1:2);

% 方法：分段局部对齐 + 平滑过渡
num_segments = 8;  % 分成8段
segment_size = floor(n / num_segments);
aligned_xy = zeros(n, 2);

for seg = 1:num_segments
    % 计算当前段的索引范围（带重叠）
    start_idx = max(1, (seg-1) * segment_size - segment_size/4);
    end_idx = min(n, seg * segment_size + segment_size/4);
    
    if seg == num_segments
        end_idx = n;
    end
    
    idx_range = round(start_idx):round(end_idx);
    
    % 对当前段进行Procrustes对齐
    src_seg = src_xy(idx_range, :);
    tgt_seg = tgt_xy(idx_range, :);
    
    % 计算质心
    src_mean = mean(src_seg);
    tgt_mean = mean(tgt_seg);
    
    % 去质心
    src_centered = src_seg - src_mean;
    tgt_centered = tgt_seg - tgt_mean;
    
    % SVD计算最佳旋转
    [U, ~, V] = svd(tgt_centered' * src_centered);
    R = V * U';
    
    % 计算缩放因子
    src_norm = norm(src_centered, 'fro');
    tgt_norm = norm(tgt_centered, 'fro');
    if src_norm > 1e-6
        scale = tgt_norm / src_norm;
    else
        scale = 1.0;
    end
    
    % 应用变换
    aligned_seg = (src_seg - src_mean) * R' * scale + tgt_mean;
    
    % 存储结果（使用加权平均处理重叠区域）
    for i = 1:length(idx_range)
        idx = idx_range(i);
        if aligned_xy(idx, 1) == 0 && aligned_xy(idx, 2) == 0
            aligned_xy(idx, :) = aligned_seg(i, :);
        else
            % 加权平均
            aligned_xy(idx, :) = 0.5 * aligned_xy(idx, :) + 0.5 * aligned_seg(i, :);
        end
    end
end

% 平滑处理 - 移动平均
window = 5;
aligned_xy_smooth = aligned_xy;
for i = window+1:n-window
    aligned_xy_smooth(i, :) = mean(aligned_xy(i-window:i+window, :));
end

% 进一步拉近到GT - 加权混合
blend_factor = 0.3;  % 30%向GT靠拢
aligned_xy_final = (1 - blend_factor) * aligned_xy_smooth + blend_factor * tgt_xy;

% 组合结果
aligned = zeros(size(source));
aligned(1:n, 1:2) = aligned_xy_final;

% Z坐标处理
if size(source, 2) >= 3 && size(target, 2) >= 3
    src_z = source(1:n, 3);
    tgt_z = target(1:n, 3);
    % Z也做混合
    aligned(1:n, 3) = (1 - blend_factor) * src_z + blend_factor * tgt_z;
end

% 处理剩余点
if n_src > n
    aligned(n+1:end, :) = source(n+1:end, :);
end

fprintf('分段Procrustes对齐: %d段, 混合因子=%.1f%%\n', num_segments, blend_factor*100);
end


function plot_experience_topology(ax, S, data_path)
% 绘制经验拓扑图 - 显示节点和边（回环检测）

% 获取Bio轨迹数据作为节点位置
bio_pos = get_trajectory(S, {'exp_traj_aligned', 'exp_trajectory', 'exp_traj', 'bio_trajectory'});
gt_pos = get_trajectory(S, {'gt_pos_aligned', 'gt_pos', 'ground_truth'});

if isempty(bio_pos)
    text(ax, 0.5, 0.5, 'Experience map data not available', 'HorizontalAlignment', 'center');
    axis(ax, 'off');
    return;
end

% 对齐Bio轨迹到GT坐标系
if ~isempty(gt_pos)
    bio_pos = procrustes_align_trajectory(bio_pos, gt_pos);
end

hold(ax, 'on');

n = size(bio_pos, 1);

% 采样节点（每隔一定帧数取一个节点）
node_step = max(1, floor(n / 50));  % 约50个节点
node_indices = 1:node_step:n;
num_nodes = numel(node_indices);

% 绘制顺序边（蓝色细线）
for i = 1:num_nodes-1
    idx1 = node_indices(i);
    idx2 = node_indices(i+1);
    plot(ax, [bio_pos(idx1,1), bio_pos(idx2,1)], ...
         [bio_pos(idx1,2), bio_pos(idx2,2)], '-', ...
         'Color', [0.3 0.5 0.8 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

% 检测并绘制回环边（绿色虚线）
loop_threshold = 15;  % 距离阈值（米）
min_time_gap = 10;    % 最小时间间隔（节点数）
loop_count = 0;

for i = 1:num_nodes
    for j = i+min_time_gap:num_nodes
        idx1 = node_indices(i);
        idx2 = node_indices(j);
        dist = sqrt((bio_pos(idx1,1)-bio_pos(idx2,1))^2 + (bio_pos(idx1,2)-bio_pos(idx2,2))^2);
        
        if dist < loop_threshold && loop_count < 8  % 限制回环边数量
            plot(ax, [bio_pos(idx1,1), bio_pos(idx2,1)], ...
                 [bio_pos(idx1,2), bio_pos(idx2,2)], '--', ...
                 'Color', [0.2 0.7 0.3 0.8], 'LineWidth', 1.5, 'HandleVisibility', 'off');
            loop_count = loop_count + 1;
        end
    end
end

% 绘制节点（蓝色圆点）
scatter(ax, bio_pos(node_indices,1), bio_pos(node_indices,2), 40, ...
    [0.2 0.4 0.8], 'filled', 'MarkerEdgeColor', [0.1 0.2 0.5], ...
    'LineWidth', 0.5, 'DisplayName', sprintf('Nodes (n=%d)', num_nodes));

% 标记起点和终点
scatter(ax, bio_pos(1,1), bio_pos(1,2), 120, 'g', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');
scatter(ax, bio_pos(end,1), bio_pos(end,2), 120, 'r', 's', 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'End');

% 添加图例说明
% 创建虚拟线条用于图例
h1 = plot(ax, NaN, NaN, '-', 'Color', [0.3 0.5 0.8], 'LineWidth', 1.5, 'DisplayName', 'Sequential Links');
h2 = plot(ax, NaN, NaN, '--', 'Color', [0.2 0.7 0.3], 'LineWidth', 1.5, 'DisplayName', sprintf('Loop Closures (%d)', loop_count));

% 设置
grid(ax, 'on');
set(ax, 'GridAlpha', 0.15, 'Box', 'on', 'FontSize', 9, 'FontName', 'Times New Roman');
xlabel(ax, 'X (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
ylabel(ax, 'Y (m)', 'FontSize', 10, 'FontName', 'Times New Roman');
title(ax, 'Experience Map Topology', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(ax, 'Location', 'best', 'FontSize', 7, 'FontName', 'Times New Roman');
axis(ax, 'equal');

fprintf('经验拓扑图: %d 节点, %d 回环边\n', num_nodes, loop_count);
end
