%% Generate Professional 3D Grid Cell FCC Lattice Figure
% 为顶级期刊设计的专业3D Grid Cell可视化
% 作者：NeuroLocMap Team
% 日期：2026-01-11

clear; close all; clc;

%% 配置参数
% 图片尺寸（单位：厘米）- 增大尺寸避免压缩
fig_width_cm = 24;  % 总宽度（增大）
fig_height_cm = 8;  % 总高度（增大）

% 配色方案（统一）
color_activity = [0, 0.7, 0.8];      % 青色 - Activity packet
color_peak = [0.86, 0.2, 0.2];       % 红色 - Peak marker
color_lattice = [0.6, 0.6, 0.6];     % 灰色 - Lattice nodes
color_fcc_vertex = [0.2, 0.4, 0.7];  % 深蓝 - FCC顶点
color_fcc_face = [0.4, 0.6, 0.9];    % 浅蓝 - FCC面心
color_hdc = [0.2, 0.4, 0.7];         % 深蓝 - HDC方向

% 字体设置（按用户要求）
font_name = 'Times New Roman';
font_size_subfig = 24;   % (a)(b)(c)子图标题 - 24pt
font_size_title = 20;    % 大标题 - 20pt
font_size_label = 18;    % 坐标轴标签 - 18pt
font_size_text = 16;     % 普通文字 - 16pt

%% 创建图形窗口
fig = figure('Position', [100, 100, fig_width_cm*37.8, fig_height_cm*37.8], ...
    'Color', 'w', 'PaperPositionMode', 'auto');

%% ========== 子图(a): 3D Activity Packet ==========
subplot(1, 3, 1);
hold on; axis equal; grid off; box on;

% 1. 绘制FCC晶格结构（背景）
lattice_size = 60;
lattice_step = 4;
[X_lat, Y_lat, Z_lat] = meshgrid(0:lattice_step:lattice_size, ...
                                   0:lattice_step:lattice_size, ...
                                   0:lattice_step:20);
% 绘制节点
scatter3(X_lat(:), Y_lat(:), Z_lat(:), 8, color_lattice, 'filled', ...
    'MarkerFaceAlpha', 0.2, 'MarkerEdgeAlpha', 0);

% 2. 绘制3D高斯Activity Packet
sigma = 5;  % 标准差
center = [30, 30, 10];  % 中心位置

% 创建3D网格
[x, y, z] = meshgrid(10:0.8:50, 10:0.8:50, 2:0.8:18);

% 计算3D高斯分布
activity = exp(-((x-center(1)).^2 + (y-center(2)).^2 + (z-center(3)).^2)/(2*sigma^2));

% 绘制等值面（多层）
fv1 = isosurface(x, y, z, activity, 0.5);
patch(fv1, 'FaceColor', color_activity, 'FaceAlpha', 0.15, 'EdgeColor', 'none');

fv2 = isosurface(x, y, z, activity, 0.3);
patch(fv2, 'FaceColor', color_activity, 'FaceAlpha', 0.25, 'EdgeColor', 'none');

fv3 = isosurface(x, y, z, activity, 0.1);
patch(fv3, 'FaceColor', color_activity, 'FaceAlpha', 0.35, 'EdgeColor', 'none');

% 3. 绘制峰值标记
scatter3(center(1), center(2), center(3), 150, color_peak, 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.5);

% 4. 设置坐标轴
xlabel('X (cells)', 'FontName', font_name, 'FontSize', font_size_label);
ylabel('Y (cells)', 'FontName', font_name, 'FontSize', font_size_label);
zlabel('Z (cells)', 'FontName', font_name, 'FontSize', font_size_label);
set(gca, 'FontName', font_name, 'FontSize', font_size_text);
xlim([0 60]); ylim([0 60]); zlim([0 20]);
view(45, 25);  % 设置视角

% 5. 添加光照
camlight('headlight');
lighting gouraud;
material dull;

% 6. 标题和标注
title('(a) 3D Activity Packet', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold');
text(50, 5, 2, '$\sigma_x = \sigma_y = \sigma_z = 5$ cells', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'right', 'Interpreter', 'latex');

%% ========== 子图(b): FCC Lattice Structure ==========
subplot(1, 3, 2);
hold on; axis equal; grid off; box on;

% 1. 绘制FCC单元格
a = 4;  % 晶格常数

% 顶点原子（8个）
vertices = [0 0 0; a 0 0; a a 0; 0 a 0; ...
            0 0 a; a 0 a; a a a; 0 a a];
scatter3(vertices(:,1), vertices(:,2), vertices(:,3), 120, ...
    color_fcc_vertex, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);

% 面心原子（6个）
face_centers = [a/2 a/2 0; a/2 a/2 a; ...  % 上下面
                a/2 0 a/2; a/2 a a/2; ...  % 前后面
                0 a/2 a/2; a a/2 a/2];     % 左右面
scatter3(face_centers(:,1), face_centers(:,2), face_centers(:,3), 100, ...
    color_fcc_face, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);

% 2. 绘制立方体边框
edges = [1 2; 2 3; 3 4; 4 1; ...  % 底面
         5 6; 6 7; 7 8; 8 5; ...  % 顶面
         1 5; 2 6; 3 7; 4 8];     % 竖边
for i = 1:size(edges, 1)
    plot3(vertices(edges(i,:), 1), vertices(edges(i,:), 2), ...
          vertices(edges(i,:), 3), 'k-', 'LineWidth', 1.2);
end

% 3. 绘制最近邻连接（部分，避免太乱）
% 只连接面心到顶点
for i = 1:size(face_centers, 1)
    fc = face_centers(i, :);
    % 找最近的4个顶点
    distances = sqrt(sum((vertices - fc).^2, 2));
    [~, nearest_idx] = sort(distances);
    for j = 1:4
        plot3([fc(1) vertices(nearest_idx(j),1)], ...
              [fc(2) vertices(nearest_idx(j),2)], ...
              [fc(3) vertices(nearest_idx(j),3)], ...
              'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'LineStyle', ':');
    end
end

% 4. 添加六边形层示意图（右侧）
% 绘制一个小的六边形网格
hex_center_x = 6;
hex_center_y = 2;
hex_center_z = a/2;
hex_radius = 0.6;
angles = (0:60:300) * pi/180;
for i = 1:6
    x_hex = hex_center_x + hex_radius * cos(angles(i));
    y_hex = hex_center_y + hex_radius * sin(angles(i));
    scatter3(x_hex, y_hex, hex_center_z, 40, color_fcc_face, 'filled');
end

% 5. 标注
xlabel('X', 'FontName', font_name, 'FontSize', font_size_label);
ylabel('Y', 'FontName', font_name, 'FontSize', font_size_label);
zlabel('Z', 'FontName', font_name, 'FontSize', font_size_label);
set(gca, 'FontName', font_name, 'FontSize', font_size_text);
xlim([-0.5 7]); ylim([-0.5 5]); zlim([-0.5 a+0.5]);
view(45, 25);

% 6. 标题
title('(b) FCC Lattice Structure', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold');

% 7. 添加说明文字
text(6, 4.5, a+0.3, 'FCC: 12 neighbors', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'right');
text(6, 4.5, a-0.3, 'Simple Cubic: 6 neighbors', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'right', 'Color', [0.5 0.5 0.5]);

% 8. 光照
camlight('headlight');
lighting gouraud;

%% ========== 子图(c): 4-DoF Encoding Scheme ==========
subplot(1, 3, 3);
axis off; hold on;

% 设置绘图区域
xlim([0 10]); ylim([0 10]);

% 1. 输入部分（顶部）
% 左侧：Grid Cell
rectangle('Position', [0.5, 7.5, 3.5, 2], 'FaceColor', [color_activity 0.2], ...
    'EdgeColor', color_activity, 'LineWidth', 1.5, 'Curvature', 0.1);
text(2.25, 8.8, 'Grid Cell', 'FontName', font_name, 'FontSize', font_size_title, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(2.25, 8.3, '$A^g \in \mathbb{R}^{N_x \times N_y \times N_z}$', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');

% 右侧：Head Direction Cell
rectangle('Position', [6, 7.5, 3.5, 2], 'FaceColor', [color_hdc 0.2], ...
    'EdgeColor', color_hdc, 'LineWidth', 1.5, 'Curvature', 0.1);
text(7.75, 8.8, 'Head Direction Cell', 'FontName', font_name, ...
    'FontSize', font_size_title, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(7.75, 8.3, '$A^h \in \mathbb{R}^{N_\psi \times N_h}$', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');

% 绘制HDC罗盘（简化）
compass_center = [7.75, 8.5];
compass_radius = 0.4;
angles_compass = (0:45:315) * pi/180;
for i = 1:length(angles_compass)
    x_end = compass_center(1) + compass_radius * cos(angles_compass(i));
    y_end = compass_center(2) + compass_radius * sin(angles_compass(i));
    if i == 1  % 高亮一个方向
        arrow([compass_center(1), compass_center(2)], [x_end, y_end], ...
            'Color', color_hdc, 'LineWidth', 2);
    else
        plot([compass_center(1) x_end], [compass_center(2) y_end], ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
    end
end

% 2. 融合箭头
annotation('arrow', [0.72 0.72], [0.58 0.52], 'LineWidth', 2.5, ...
    'HeadStyle', 'vback3', 'HeadLength', 8, 'HeadWidth', 8);
text(5, 6.8, 'Concatenation', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');
text(5, 6.4, '$[g_{xyz} ; h_v]$', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center', ...
    'Interpreter', 'latex');

% 3. 解码过程
rectangle('Position', [1, 4, 8, 2], 'FaceColor', [0.94 0.94 0.94], ...
    'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.2, 'Curvature', 0.1);
text(5, 5.6, 'Decoding', 'FontName', font_name, 'FontSize', font_size_title, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 解码公式（分行显示）
text(5, 5.1, '$\hat{x} = \sum A^g_{i,j,k} \cdot x_{i,j,k}$', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');
text(5, 4.7, '$\hat{y} = \sum A^g_{i,j,k} \cdot y_{i,j,k}$', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');
text(5, 4.3, '$\hat{\psi} = \mathrm{atan2}(\sum A^h_u \sin\theta_u, \sum A^h_u \cos\theta_u)$', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'latex');

% 4. 输出箭头
annotation('arrow', [0.72 0.72], [0.38 0.32], 'LineWidth', 2.5, ...
    'HeadStyle', 'vback3', 'HeadLength', 8, 'HeadWidth', 8);

% 5. 输出部分
rectangle('Position', [2, 1, 6, 1.5], 'FaceColor', [0.86 0.92 1], ...
    'EdgeColor', [0.2 0.4 0.7], 'LineWidth', 1.5, 'Curvature', 0.1);
text(5, 2.1, '4-DoF Pose Output', 'FontName', font_name, ...
    'FontSize', font_size_title, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');
text(5, 1.6, '$(\hat{x}, \hat{y}, \hat{z}, \hat{\psi})$', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center', ...
    'Interpreter', 'latex');

% 6. 标题
text(5, 9.8, '(c) 4-DoF Encoding & Decoding', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

%% 调整子图间距
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [fig_width_cm fig_height_cm]);
set(gcf, 'PaperPosition', [0 0 fig_width_cm fig_height_cm]);

%% 保存图片
% 保存到kbs_1/fig目录
output_dir = 'kbs/kbs_1/fig';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% 保存为PDF（矢量图）
output_file = fullfile(output_dir, '3d_grid_cell_fcc_lattice.pdf');
print(fig, output_file, '-dpdf', '-r300', '-painters');

fprintf('✅ 图片已生成并保存到：\n');
fprintf('   PDF: %s\n', output_file);

%% 辅助函数：绘制箭头
function arrow(start_point, end_point, varargin)
    % 简单的箭头绘制函数
    p = inputParser;
    addParameter(p, 'Color', 'k', @(x) true);
    addParameter(p, 'LineWidth', 1, @isnumeric);
    parse(p, varargin{:});
    
    % 绘制线段
    plot([start_point(1) end_point(1)], [start_point(2) end_point(2)], ...
        'Color', p.Results.Color, 'LineWidth', p.Results.LineWidth);
    
    % 绘制箭头头部（简化）
    arrow_length = 0.15;
    arrow_angle = 25 * pi/180;
    dx = end_point(1) - start_point(1);
    dy = end_point(2) - start_point(2);
    angle = atan2(dy, dx);
    
    x1 = end_point(1) - arrow_length * cos(angle - arrow_angle);
    y1 = end_point(2) - arrow_length * sin(angle - arrow_angle);
    x2 = end_point(1) - arrow_length * cos(angle + arrow_angle);
    y2 = end_point(2) - arrow_length * sin(angle + arrow_angle);
    
    plot([end_point(1) x1], [end_point(2) y1], ...
        'Color', p.Results.Color, 'LineWidth', p.Results.LineWidth);
    plot([end_point(1) x2], [end_point(2) y2], ...
        'Color', p.Results.Color, 'LineWidth', p.Results.LineWidth);
end
