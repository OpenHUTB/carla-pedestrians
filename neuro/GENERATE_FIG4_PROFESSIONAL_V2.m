%% Generate Professional 3D Grid Cell Figure - Version 2
% 顶刊级别专业绘图 - 优化布局，避免压缩
% 作者：Professional Illustration Master
% 日期：2026-01-12

clear; close all; clc;

%% 配置参数
% 图片尺寸（单位：厘米）- 大幅增加，给足空间
fig_width_cm = 20;   % 增加宽度
fig_height_cm = 24;  % 大幅增加高度，避免任何压缩

% 配色方案（统一、专业）
color_activity = [0, 0.71, 0.78];    % 青色 - Activity packet
color_peak = [0.86, 0.2, 0.2];       % 红色 - Peak marker
color_lattice = [0.65, 0.65, 0.65];  % 灰色 - Lattice nodes
color_fcc_vertex = [0.2, 0.4, 0.7];  % 深蓝 - FCC顶点
color_fcc_face = [0.4, 0.6, 0.9];    % 浅蓝 - FCC面心
color_hdc = [0.2, 0.4, 0.7];         % 深蓝 - HDC方向
color_box = [0.95, 0.95, 0.95];      % 浅灰 - 背景框

% 字体设置（严格按用户要求）
font_name = 'Times New Roman';
font_size_subfig = 24;   % (a)(b)(c)子图标题 - 24pt
font_size_title = 20;    % 大标题 - 20pt
font_size_label = 18;    % 坐标轴标签 - 18pt
font_size_text = 16;     % 普通文字 - 16pt

%% 创建图形窗口
fig = figure('Position', [100, 100, fig_width_cm*37.8, fig_height_cm*37.8], ...
    'Color', 'w', 'PaperPositionMode', 'auto');

%% ========== 子图(a): 3D Activity Packet - 左上 ==========
% 增加边距，避免裁剪
subplot('Position', [0.10, 0.62, 0.35, 0.32]);  % [left, bottom, width, height]
hold on; axis equal; grid off; box on;

% 1. 绘制FCC晶格结构（背景，稀疏一些）
lattice_size = 60;
lattice_step = 5;  % 增大步长，减少节点数
[X_lat, Y_lat, Z_lat] = meshgrid(0:lattice_step:lattice_size, ...
                                   0:lattice_step:lattice_size, ...
                                   0:lattice_step:20);
scatter3(X_lat(:), Y_lat(:), Z_lat(:), 12, color_lattice, 'filled', ...
    'MarkerFaceAlpha', 0.25, 'MarkerEdgeAlpha', 0);

% 2. 绘制3D高斯Activity Packet
sigma = 6;  % 稍微增大
center = [30, 30, 10];

[x, y, z] = meshgrid(10:1:50, 10:1:50, 2:1:18);
activity = exp(-((x-center(1)).^2 + (y-center(2)).^2 + (z-center(3)).^2)/(2*sigma^2));

% 绘制等值面（三层）
fv1 = isosurface(x, y, z, activity, 0.5);
patch(fv1, 'FaceColor', color_activity, 'FaceAlpha', 0.2, 'EdgeColor', 'none');

fv2 = isosurface(x, y, z, activity, 0.3);
patch(fv2, 'FaceColor', color_activity, 'FaceAlpha', 0.3, 'EdgeColor', 'none');

fv3 = isosurface(x, y, z, activity, 0.1);
patch(fv3, 'FaceColor', color_activity, 'FaceAlpha', 0.4, 'EdgeColor', 'none');

% 3. 绘制峰值标记
scatter3(center(1), center(2), center(3), 200, color_peak, 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% 4. 坐标轴
xlabel('X (cells)', 'FontName', font_name, 'FontSize', font_size_label);
ylabel('Y (cells)', 'FontName', font_name, 'FontSize', font_size_label);
zlabel('Z (cells)', 'FontName', font_name, 'FontSize', font_size_label);
set(gca, 'FontName', font_name, 'FontSize', font_size_text);
xlim([0 60]); ylim([0 60]); zlim([0 20]);
view(45, 25);

% 5. 光照
camlight('headlight');
lighting gouraud;
material dull;

% 6. 标题
title('(a) 3D Activity Packet', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold');

% 7. 标注（简化，不用公式）
text(52, 5, 2, 'Gaussian spread: \sigma = 6 cells', ...
    'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'right');

%% ========== 子图(b): FCC Lattice Structure - 右上 ==========
% 增加边距，避免裁剪
subplot('Position', [0.55, 0.62, 0.35, 0.32]);
hold on; axis equal; grid off; box on;

% 1. 绘制FCC单元格
a = 4;  % 晶格常数

% 顶点原子（8个）
vertices = [0 0 0; a 0 0; a a 0; 0 a 0; ...
            0 0 a; a 0 a; a a a; 0 a a];
scatter3(vertices(:,1), vertices(:,2), vertices(:,3), 180, ...
    color_fcc_vertex, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

% 面心原子（6个）
face_centers = [a/2 a/2 0; a/2 a/2 a; ...
                a/2 0 a/2; a/2 a a/2; ...
                0 a/2 a/2; a a/2 a/2];
scatter3(face_centers(:,1), face_centers(:,2), face_centers(:,3), 150, ...
    color_fcc_face, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

% 2. 绘制立方体边框
edges = [1 2; 2 3; 3 4; 4 1; ...
         5 6; 6 7; 7 8; 8 5; ...
         1 5; 2 6; 3 7; 4 8];
for i = 1:size(edges, 1)
    plot3(vertices(edges(i,:), 1), vertices(edges(i,:), 2), ...
          vertices(edges(i,:), 3), 'k-', 'LineWidth', 1.5);
end

% 3. 绘制部分连接线（避免太乱）
for i = 1:size(face_centers, 1)
    fc = face_centers(i, :);
    distances = sqrt(sum((vertices - fc).^2, 2));
    [~, nearest_idx] = sort(distances);
    for j = 1:4
        plot3([fc(1) vertices(nearest_idx(j),1)], ...
              [fc(2) vertices(nearest_idx(j),2)], ...
              [fc(3) vertices(nearest_idx(j),3)], ...
              'Color', [0.7 0.7 0.7], 'LineWidth', 0.8, 'LineStyle', ':');
    end
end

% 4. 坐标轴
xlabel('X', 'FontName', font_name, 'FontSize', font_size_label);
ylabel('Y', 'FontName', font_name, 'FontSize', font_size_label);
zlabel('Z', 'FontName', font_name, 'FontSize', font_size_label);
set(gca, 'FontName', font_name, 'FontSize', font_size_text);
xlim([-0.5 a+0.5]); ylim([-0.5 a+0.5]); zlim([-0.5 a+0.5]);
view(45, 25);

% 5. 光照
camlight('headlight');
lighting gouraud;

% 6. 标题
title('(b) FCC Lattice Structure', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold');

% 7. 标注（简化）
text(a+0.3, a+0.3, a+0.3, 'FCC: 12 neighbors', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'left');
text(a+0.3, a+0.3, a-0.5, 'Simple Cubic: 6 neighbors', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'left', 'Color', [0.5 0.5 0.5]);

%% ========== 子图(c): 4-DoF Encoding - 下方，更大空间 ==========
% 增加边距和高度，避免文字溢出
subplot('Position', [0.10, 0.08, 0.80, 0.48]);  % 增加高度和边距
axis off; hold on;
xlim([0 10]); ylim([0 10]);

% === 输入部分（顶部） ===
% 左侧：Grid Cell - 调整位置，避免溢出
y_top = 8.8;  % 降低一点
rectangle('Position', [0.5, y_top-1.3, 3.2, 1.6], 'FaceColor', [color_activity 0.15], ...
    'EdgeColor', color_activity, 'LineWidth', 2, 'Curvature', 0.1);
text(2, y_top-0.2, 'Grid Cell Network', 'FontName', font_name, 'FontSize', font_size_title, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(2, y_top-0.7, '3D Position (x, y, z)', 'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center');

% 右侧：Head Direction Cell - 调整位置
rectangle('Position', [6.3, y_top-1.3, 3.2, 1.6], 'FaceColor', [color_hdc 0.15], ...
    'EdgeColor', color_hdc, 'LineWidth', 2, 'Curvature', 0.1);
text(8, y_top-0.2, 'Head Direction Cell', 'FontName', font_name, 'FontSize', font_size_title, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(8, y_top-0.7, 'Heading \psi', 'FontName', font_name, 'FontSize', font_size_text, ...
    'HorizontalAlignment', 'center');

% 绘制HDC罗盘（简化）
compass_center = [8, y_top-0.45];
compass_radius = 0.35;
angles_compass = (0:45:315) * pi/180;
for i = 1:length(angles_compass)
    x_end = compass_center(1) + compass_radius * cos(angles_compass(i));
    y_end = compass_center(2) + compass_radius * sin(angles_compass(i));
    if i == 1
        annotation('arrow', [(compass_center(1))/10, x_end/10], ...
            [(compass_center(2))/10, y_end/10], 'Color', color_hdc, 'LineWidth', 2.5);
    else
        plot([compass_center(1) x_end], [compass_center(2) y_end], ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 1);
    end
end

% === 融合箭头 ===
y_mid = 6;
annotation('arrow', [0.5, 0.5], [0.58, 0.48], 'LineWidth', 3, ...
    'HeadStyle', 'vback3', 'HeadLength', 10, 'HeadWidth', 10, 'Color', 'k');
text(5, y_mid+0.3, 'Concatenate', 'FontName', font_name, ...
    'FontSize', font_size_title, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(5, y_mid-0.1, '[Position ; Heading]', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');

% === 解码过程 - 增大框，避免文字溢出 ===
y_decode = 4.2;  % 调整位置
rectangle('Position', [1.2, y_decode-1.0, 7.6, 2.2], 'FaceColor', color_box, ...
    'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.5, 'Curvature', 0.1);
text(5, y_decode+0.7, 'Decoding Process', 'FontName', font_name, 'FontSize', font_size_title, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 解码说明（简化，不用复杂公式）
text(5, y_decode+0.2, 'Population Vector Decoding:', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');
text(5, y_decode-0.2, 'Position: Weighted sum of active cells', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');
text(5, y_decode-0.6, 'Heading: Circular mean of HDC activity', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');

% === 输出箭头 ===
y_out = 1.8;
annotation('arrow', [0.5, 0.5], [0.32, 0.22], 'LineWidth', 3, ...
    'HeadStyle', 'vback3', 'HeadLength', 10, 'HeadWidth', 10, 'Color', 'k');

% === 输出部分 - 调整位置 ===
y_out = 1.5;  % 降低位置
rectangle('Position', [2.5, y_out-0.8, 5, 1.0], 'FaceColor', [0.86 0.92 1], ...
    'EdgeColor', [0.2 0.4 0.7], 'LineWidth', 2, 'Curvature', 0.1);
text(5, y_out-0.2, '4-DoF Pose Estimate', 'FontName', font_name, ...
    'FontSize', font_size_title, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');
text(5, y_out-0.6, '(x, y, z, \psi)', 'FontName', font_name, ...
    'FontSize', font_size_text, 'HorizontalAlignment', 'center');

% === 子图标题 - 调整位置 ===
text(5, 9.7, '(c) 4-DoF Encoding & Decoding', 'FontName', font_name, ...
    'FontSize', font_size_subfig, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

%% 调整整体布局
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [fig_width_cm fig_height_cm]);
set(gcf, 'PaperPosition', [0 0 fig_width_cm fig_height_cm]);

%% 保存图片
output_dir = 'kbs/kbs_1/fig';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% 保存为PDF（矢量图）
output_file = fullfile(output_dir, '3d_grid_cell_fcc_lattice_v4.pdf');
print(fig, output_file, '-dpdf', '-r300', '-painters');

fprintf('✅ Figure 4 V4已生成（修复裁剪和溢出问题）！\n');
fprintf('   位置: %s\n', output_file);
fprintf('   尺寸: %.1f × %.1f cm\n', fig_width_cm, fig_height_cm);
fprintf('   改进: 增加边距，避免裁剪；增大框，避免文字溢出\n');
fprintf('   尺寸: %.1f × %.1f cm\n', fig_width_cm, fig_height_cm);
fprintf('   布局: 上下布局，避免压缩\n');
fprintf('   字体: Times New Roman, 24/20/18/16pt\n');
fprintf('   风格: 顶刊级别，无复杂公式\n');
