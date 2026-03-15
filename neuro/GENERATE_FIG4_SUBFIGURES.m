%% 生成Figure 4的三个子图 - 分别独立保存
% 输出路径: E:\Neuro_end\neuro\kbs\kbs_1\fig\
% 文件名: 1.pdf, 2.pdf, 3.pdf

clear; close all;

output_dir = 'E:\Neuro_end\neuro\kbs\kbs_1\fig\';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ========== 子图1: 3D Activity Packet ==========
fprintf('生成子图1: 3D Activity Packet...\n');

fig1 = figure('Position', [100, 100, 800, 600], 'Color', 'w');
set(fig1, 'PaperPositionMode', 'auto');

% 创建3D高斯分布
[X, Y, Z] = meshgrid(-3:0.2:3, -3:0.2:3, -3:0.2:3);
center = [0, 0, 0];
sigma = 0.8;
G = exp(-((X-center(1)).^2 + (Y-center(2)).^2 + (Z-center(3)).^2)/(2*sigma^2));

% 绘制多层等值面
hold on;
levels = [0.3, 0.5, 0.7];
colors = [0.4 0.7 0.9; 0.3 0.6 0.85; 0.2 0.5 0.8];

for i = 1:length(levels)
    p = patch(isosurface(X, Y, Z, G, levels(i)));
    set(p, 'FaceColor', colors(i,:), 'EdgeColor', 'none', ...
        'FaceAlpha', 0.4, 'FaceLighting', 'gouraud');
end

% 坐标轴设置
xlabel('X (cells)', 'FontName', 'Times New Roman', 'FontSize', 18);
ylabel('Y (cells)', 'FontName', 'Times New Roman', 'FontSize', 18);
zlabel('Z (cells)', 'FontName', 'Times New Roman', 'FontSize', 18);
title('(a) 3D Activity Packet', 'FontName', 'Times New Roman', ...
    'FontSize', 24, 'FontWeight', 'bold');

% 添加说明文字
text(0, -3.5, -3.5, 'Gaussian spread: \sigma = 6 cells', ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'HorizontalAlignment', 'center');

grid on;
axis equal;
view(45, 30);
lighting gouraud;
camlight('headlight');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 16);
box on;

% 保存
print(fig1, fullfile(output_dir, '1.pdf'), '-dpdf', '-r300');
fprintf('✓ 子图1保存完成: %s\n', fullfile(output_dir, '1.pdf'));

%% ========== 子图2: FCC Lattice Structure ==========
fprintf('生成子图2: FCC Lattice Structure...\n');

fig2 = figure('Position', [100, 100, 900, 900], 'Color', 'w');
set(fig2, 'PaperPositionMode', 'auto');
set(fig2, 'PaperUnits', 'inches');
set(fig2, 'PaperSize', [9, 9]);

% 创建主绘图区域（上方65%）
ax_main = axes('Position', [0.1, 0.38, 0.85, 0.57]);

% FCC晶格的三层结构 - 使用更清晰的布局
a = 1; % 晶格常数

% 第一层 (z=0) - 深蓝色
layer1_x = [0, a, 2*a, 0, a, 2*a, 0, a, 2*a];
layer1_y = [0, 0, 0, a, a, a, 2*a, 2*a, 2*a];
layer1_z = zeros(size(layer1_x));

% 第二层 (z=a/2) - 橙色（插入到第一层的间隙中）
layer2_x = [a/2, 3*a/2, a/2, 3*a/2];
layer2_y = [a/2, a/2, 3*a/2, 3*a/2];
layer2_z = ones(size(layer2_x)) * a/2;

% 第三层 (z=a) - 绿色（与第一层对齐）
layer3_x = layer1_x;
layer3_y = layer1_y;
layer3_z = ones(size(layer1_x)) * a;

hold(ax_main, 'on');

% 绘制连接线（先画线，后画点，这样点在上层）
line_color = [0.6 0.6 0.6];
line_width = 1.5;

% 层内连接 - Layer 1
for i = 1:length(layer1_x)
    for j = i+1:length(layer1_x)
        dist = sqrt((layer1_x(i)-layer1_x(j))^2 + (layer1_y(i)-layer1_y(j))^2);
        if abs(dist - a) < 0.1
            plot3(ax_main, [layer1_x(i), layer1_x(j)], [layer1_y(i), layer1_y(j)], ...
                [layer1_z(i), layer1_z(j)], 'Color', line_color, 'LineWidth', line_width);
        end
    end
end

% 层间连接 - 展示FCC的12邻居特性
% 选择中心点展示连接
center_idx = 5; % layer1中心点
for j = 1:length(layer2_x)
    dist = sqrt((layer1_x(center_idx)-layer2_x(j))^2 + ...
                (layer1_y(center_idx)-layer2_y(j))^2 + ...
                (layer1_z(center_idx)-layer2_z(j))^2);
    if dist < a*0.9
        plot3(ax_main, [layer1_x(center_idx), layer2_x(j)], ...
            [layer1_y(center_idx), layer2_y(j)], ...
            [layer1_z(center_idx), layer2_z(j)], ...
            'Color', [0.8 0.3 0.3], 'LineWidth', 2, 'LineStyle', '--');
    end
end

% 绘制三层原子（大球体，清晰可见）
scatter3(ax_main, layer1_x, layer1_y, layer1_z, 300, [0.2 0.4 0.8], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2);
scatter3(ax_main, layer2_x, layer2_y, layer2_z, 300, [0.9 0.6 0.2], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2);
scatter3(ax_main, layer3_x, layer3_y, layer3_z, 300, [0.3 0.7 0.3], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2);

% 标注中心原子
plot3(ax_main, layer1_x(center_idx), layer1_y(center_idx), layer1_z(center_idx), ...
    'ro', 'MarkerSize', 15, 'LineWidth', 3);

% 坐标轴设置
xlabel(ax_main, 'X', 'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'bold');
ylabel(ax_main, 'Y', 'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'bold');
zlabel(ax_main, 'Z', 'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'bold');
title(ax_main, '(b) FCC Lattice Structure', 'FontName', 'Times New Roman', ...
    'FontSize', 24, 'FontWeight', 'bold');

% 添加图例（简洁版）
legend(ax_main, {'Layer 1 (z=0)', 'Layer 2 (z=a/2)', 'Layer 3 (z=a)'}, ...
    'FontName', 'Times New Roman', 'FontSize', 16, 'Location', 'northeast', ...
    'Box', 'on');

grid(ax_main, 'on');
axis(ax_main, 'equal');
view(ax_main, 45, 25);
set(ax_main, 'FontName', 'Times New Roman', 'FontSize', 16);
box(ax_main, 'on');
xlim(ax_main, [-0.5, 2.5]);
ylim(ax_main, [-0.5, 2.5]);
zlim(ax_main, [-0.3, 1.3]);

% 添加光照效果
lighting(ax_main, 'gouraud');
camlight(ax_main, 'headlight');

% 文字说明区域（下方33%）
ax_text = axes('Position', [0.05, 0.02, 0.9, 0.33], 'Visible', 'off');
hold(ax_text, 'on');

% 添加说明文字（分行，清晰易读，字号稍小）
text_y = 0.95;
text(ax_text, 0.02, text_y, '\bf{Key Features:}', ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'Units', 'normalized', 'VerticalAlignment', 'top');

text_y = text_y - 0.22;
text(ax_text, 0.02, text_y, '\bullet Hexagonal pattern in 2D projection', ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'Units', 'normalized', 'VerticalAlignment', 'top');

text_y = text_y - 0.22;
text(ax_text, 0.02, text_y, '\bullet Three layers offset by a/2 for optimal packing', ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'Units', 'normalized', 'VerticalAlignment', 'top');

text_y = text_y - 0.22;
text(ax_text, 0.02, text_y, '\bullet 12 nearest neighbors (red dashed lines)', ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'Units', 'normalized', 'VerticalAlignment', 'top');

text_y = text_y - 0.22;
text(ax_text, 0.02, text_y, '\bullet Packing efficiency: 74% vs. 52% (cubic)', ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'Units', 'normalized', 'VerticalAlignment', 'top');

% 保存
print(fig2, fullfile(output_dir, '2.pdf'), '-dpdf', '-r300');
fprintf('✓ 子图2保存完成: %s\n', fullfile(output_dir, '2.pdf'));

%% ========== 子图3: 4-DoF Encoding Scheme ==========
fprintf('生成子图3: 4-DoF Encoding Scheme...\n');

fig3 = figure('Position', [100, 100, 1200, 500], 'Color', 'w');
set(fig3, 'PaperPositionMode', 'auto');
set(fig3, 'PaperUnits', 'inches');
set(fig3, 'PaperSize', [12, 5]);

% 使用annotation绘制流程图（调整位置，留出更多空间）
% 标题
annotation('textbox', [0.35, 0.85, 0.3, 0.12], ...
    'String', '(c) 4-DoF Encoding Scheme', ...
    'FontName', 'Times New Roman', 'FontSize', 24, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold');

% 输入框 - Position
annotation('rectangle', [0.05, 0.45, 0.12, 0.18], 'LineWidth', 2, ...
    'FaceColor', [0.95 0.95 0.95]);
annotation('textbox', [0.05, 0.45, 0.12, 0.18], ...
    'String', {'Position', '(x, y, z)'}, ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold');

% 箭头1
annotation('arrow', [0.17, 0.26], [0.54, 0.54], 'LineWidth', 2);

% 3D Grid Cell Network
annotation('rectangle', [0.26, 0.42, 0.16, 0.24], 'LineWidth', 2, ...
    'FaceColor', [0.8 0.9 1]);
annotation('textbox', [0.26, 0.42, 0.16, 0.24], ...
    'String', {'3D Grid Cell', 'Network', '(FCC)'}, ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold');

% 箭头2
annotation('arrow', [0.42, 0.50], [0.54, 0.54], 'LineWidth', 2);

% g_xyz
annotation('rectangle', [0.50, 0.47, 0.09, 0.14], 'LineWidth', 2, ...
    'FaceColor', [0.95 0.95 0.95]);
annotation('textbox', [0.50, 0.47, 0.09, 0.14], ...
    'String', 'g_{xyz}', ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'Interpreter', 'tex');

% Heading输入
annotation('rectangle', [0.05, 0.15, 0.12, 0.18], 'LineWidth', 2, ...
    'FaceColor', [0.95 0.95 0.95]);
annotation('textbox', [0.05, 0.15, 0.12, 0.18], ...
    'String', {'Heading', '\psi'}, ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold', 'Interpreter', 'tex');

% 箭头3
annotation('arrow', [0.17, 0.26], [0.24, 0.24], 'LineWidth', 2);

% Head Direction Cell
annotation('rectangle', [0.26, 0.12, 0.16, 0.24], 'LineWidth', 2, ...
    'FaceColor', [1 0.9 0.8]);
annotation('textbox', [0.26, 0.12, 0.16, 0.24], ...
    'String', {'Head Direction', 'Cell Network'}, ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold');

% 箭头4
annotation('arrow', [0.42, 0.50], [0.24, 0.24], 'LineWidth', 2);

% h_psi
annotation('rectangle', [0.50, 0.17, 0.09, 0.14], 'LineWidth', 2, ...
    'FaceColor', [0.95 0.95 0.95]);
annotation('textbox', [0.50, 0.17, 0.09, 0.14], ...
    'String', 'h_{\psi}', ...
    'FontName', 'Times New Roman', 'FontSize', 16, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'Interpreter', 'tex');

% 合并箭头
annotation('arrow', [0.59, 0.68], [0.54, 0.42], 'LineWidth', 2);
annotation('arrow', [0.59, 0.68], [0.24, 0.36], 'LineWidth', 2);

% Concatenation
annotation('rectangle', [0.68, 0.30, 0.12, 0.18], 'LineWidth', 2, ...
    'FaceColor', [0.9 1 0.9]);
annotation('textbox', [0.68, 0.30, 0.12, 0.18], ...
    'String', {'Concatenate', '[g; h]'}, ...
    'FontName', 'Times New Roman', 'FontSize', 14, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'FontWeight', 'bold');

% 箭头5
annotation('arrow', [0.80, 0.88], [0.39, 0.39], 'LineWidth', 2);

% 输出
annotation('rectangle', [0.88, 0.32, 0.09, 0.14], 'LineWidth', 3, ...
    'FaceColor', [0.7 0.9 1]);
annotation('textbox', [0.88, 0.32, 0.09, 0.14], ...
    'String', 's_{4D}', ...
    'FontName', 'Times New Roman', 'FontSize', 18, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'EdgeColor', 'none', 'Interpreter', 'tex', 'FontWeight', 'bold');

% 保存
print(fig3, fullfile(output_dir, '3.pdf'), '-dpdf', '-r300');
fprintf('✓ 子图3保存完成: %s\n', fullfile(output_dir, '3.pdf'));

%% 完成
fprintf('\n========================================\n');
fprintf('✓ 所有子图生成完成！\n');
fprintf('输出目录: %s\n', output_dir);
fprintf('文件列表:\n');
fprintf('  - 1.pdf (3D Activity Packet)\n');
fprintf('  - 2.pdf (FCC Lattice Structure)\n');
fprintf('  - 3.pdf (4-DoF Encoding Scheme)\n');
fprintf('========================================\n');

close all;
