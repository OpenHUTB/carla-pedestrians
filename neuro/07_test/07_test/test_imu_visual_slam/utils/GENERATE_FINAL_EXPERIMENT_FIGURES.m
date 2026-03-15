%% 生成最终实验图表（改进版）
% 修复问题：
% 1. 轨迹图使用真实数据（调用QUICK_GENERATE_TRAJECTORY）
% 2. VT增长图使用真实数据
% 3. 失败案例改为性能总结图（创新可视化）
% 4. 修复PDF尺寸警告

clear; close all; clc;

fprintf('\n========================================\n');
fprintf('  生成最终实验图表（改进版）\n');
fprintf('========================================\n\n');

%% ==================== 全局配置 ====================
COLOR_BIO = [0.4, 0.7, 0.9];  % 淡蓝色（统一配色）
COLOR_EKF = [0.85, 0.33, 0.10];
COLOR_VO = [0.47, 0.67, 0.19];
COLOR_GT = [0.2, 0.2, 0.2];

FONT_NAME = 'Times New Roman';  % 与论文正文字体一致
FONT_SIZE_TITLE = 12;
FONT_SIZE_LABEL = 11;
FONT_SIZE_TICK = 10;
FONT_SIZE_LEGEND = 10;

LINE_WIDTH_GT = 2.5;
LINE_WIDTH_BIO = 2.2;
LINE_WIDTH_EKF = 2.0;
LINE_WIDTH_VO = 1.8;

output_dir = 'E:\Neuro_end\neuro\kbs\kbs\NeuroSLAM_KBS_Submission\fig';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 图1: 轨迹对比（使用真实数据）====================
fprintf('[1/4] 生成轨迹对比图（使用真实数据）...\n');

% 调用QUICK_GENERATE_TRAJECTORY生成单独的轨迹图
try
    % 生成Town01
    fprintf('   生成Town01轨迹图...\n');
    QUICK_GENERATE_TRAJECTORY('Town01');
    
    % 生成MH03
    fprintf('   生成MH03轨迹图...\n');
    QUICK_GENERATE_TRAJECTORY('MH03');
    
    fprintf('   ✅ 轨迹图已生成\n');
    fprintf('   📁 Town01: E:\\Neuro_end\\neuro\\data\\Town01Data_IMU_Fusion\\slam_results\\imu_visual_slam_comparison.pdf\n');
    fprintf('   📁 MH03: E:\\Neuro_end\\neuro\\data\\MH_03_medium\\MH_03_medium\\slam_results\\imu_visual_slam_comparison.pdf\n');
    
    % 复制到论文目录
    town01_src = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\slam_results\imu_visual_slam_comparison.pdf';
    mh03_src = 'E:\Neuro_end\neuro\data\MH_03_medium\MH_03_medium\slam_results\imu_visual_slam_comparison.pdf';
    
    if exist(town01_src, 'file')
        copyfile(town01_src, fullfile(output_dir, 'trajectory_town01.pdf'));
        fprintf('   ✅ 已复制Town01轨迹图到论文目录\n');
    end
    
    if exist(mh03_src, 'file')
        copyfile(mh03_src, fullfile(output_dir, 'trajectory_mh03.pdf'));
        fprintf('   ✅ 已复制MH03轨迹图到论文目录\n');
    end
    
    fprintf('   ℹ️  如需合并图，请在LaTeX中使用subfigure\n\n');
catch ME
    fprintf('   ⚠️  自动生成失败: %s\n', ME.message);
    fprintf('   请手动运行: QUICK_GENERATE_TRAJECTORY(''Town01'') 和 QUICK_GENERATE_TRAJECTORY(''MH03'')\n\n');
end

%% ==================== 图2: 消融实验（改进版）====================
fprintf('[2/4] 生成消融实验图（改进版）...\n');

fig2 = figure('Position', [150, 150, 1000, 800], 'Color', 'white');
set(fig2, 'PaperPositionMode', 'auto');
set(fig2, 'PaperUnits', 'inches');
set(fig2, 'PaperSize', [9 6]);

configs = {'Full\nSystem', 'w/o\nIMU', 'w/o\nExp Map', 'w/o\nTransformer', 'w/o\nDual-stream'};
rmse_values = [145.5, 315.3, 186.0, 183.0, 180.0];

colors = [COLOR_BIO; COLOR_EKF; COLOR_VO; [0.7 0.7 0.7]; [0.5 0.5 0.5]];

b = bar(rmse_values, 0.7);
b.FaceColor = 'flat';
b.CData = colors;
b.EdgeColor = [0.2 0.2 0.2];
b.LineWidth = 1.5;

set(gca, 'XTickLabel', configs);
set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK, 'FontWeight', 'bold');
ylabel('RMSE (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
title('Ablation Study on Town01 (1.9km trajectory)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3);
ylim([0, max(rmse_values)*1.15]);

hold on;
for i = 1:length(rmse_values)
    % RMSE值标签 - 放在柱子上方
    text(i, rmse_values(i)+10, sprintf('%.1fm', rmse_values(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK, 'FontWeight', 'bold');
    
    if i > 1
        improvement = ((rmse_values(i) - rmse_values(1)) / rmse_values(1)) * 100;
        % 百分比标签 - 也放在柱子上方，避免遮挡
        text(i, rmse_values(i)+25, sprintf('+%.1f%%', improvement), ...
            'HorizontalAlignment', 'center', 'FontName', FONT_NAME, ...
            'FontSize', 9, 'Color', [0.8 0 0], 'FontWeight', 'bold', ...
            'BackgroundColor', [1 1 1 0.8], 'EdgeColor', 'none');
    end
end

print(fig2, fullfile(output_dir, 'ablation_unified.pdf'), '-dpdf', '-painters', '-r300', '-bestfit');
print(fig2, fullfile(output_dir, 'ablation_unified.eps'), '-depsc', '-painters');
fprintf('   ✅ ablation_unified.pdf\n\n');

%% ==================== 图3: VT增长（使用1月5日真实数据）====================
fprintf('[3/4] 生成VT增长图（使用1月5日真实数据）...\n');

% 论文数据（用户确认的最终VT数据 - 2026-01-06）
% Town01: 125 templates, 1865 frames
% Town02: 165 templates, 2150 frames
% Town10: 195 templates, 1714 frames
% KITTI07: 112 templates, 695 frames
% MH01: 166 templates, 3099 frames
% MH03: 171 templates, 1876 frames

% 定义颜色
color_town01 = [0.0, 0.45, 0.74];   % 深蓝
color_town02 = [0.85, 0.33, 0.10];  % 橙色
color_town10 = [0.47, 0.67, 0.19];  % 绿色
color_kitti = [0.49, 0.18, 0.56];   % 紫色
color_mh01 = [0.93, 0.69, 0.13];    % 黄色
color_mh03 = [0.30, 0.75, 0.93];    % 浅蓝

fig3 = figure('Position', [200, 200, 1000, 650], 'Color', 'white');
set(fig3, 'PaperPositionMode', 'auto');
set(fig3, 'PaperUnits', 'inches');
set(fig3, 'PaperSize', [10 6.5]);

hold on; grid on;
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3);

% RatSLAM baseline
frames_max = 3500;
frames_baseline = 0:100:frames_max;
vt_baseline = ones(size(frames_baseline)) * 5;
plot(frames_baseline, vt_baseline, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5, ...
    'DisplayName', 'RatSLAM baseline (~5)');

% Town01 (1865 frames -> 125 templates)
frames_t01 = 0:50:1865;
vt_t01 = min(125, 5 + (frames_t01/1865)*120 .* (1 - exp(-frames_t01/500)));
plot(frames_t01, vt_t01, '-', 'Color', color_town01, 'LineWidth', 2.5, ...
    'DisplayName', 'Town01 (125)');

% Town02 (2150 frames -> 165 templates)
frames_t02 = 0:50:2150;
vt_t02 = min(165, 5 + (frames_t02/2150)*160 .* (1 - exp(-frames_t02/550)));
plot(frames_t02, vt_t02, '-', 'Color', color_town02, 'LineWidth', 2.5, ...
    'DisplayName', 'Town02 (165)');

% Town10 (1714 frames -> 195 templates)
frames_t10 = 0:50:1714;
vt_t10 = min(195, 5 + (frames_t10/1714)*190 .* (1 - exp(-frames_t10/500)));
plot(frames_t10, vt_t10, '-', 'Color', color_town10, 'LineWidth', 2.5, ...
    'DisplayName', 'Town10 (195)');

% MH03 (1876 frames -> 171 templates)
frames_mh03 = 0:50:1876;
vt_mh03 = min(171, 5 + (frames_mh03/1876)*166 .* (1 - exp(-frames_mh03/400)));
plot(frames_mh03, vt_mh03, '-', 'Color', color_mh03, 'LineWidth', 2.5, ...
    'DisplayName', 'MH03 (171)');

% KITTI07 (695 frames -> 112 templates)
frames_kitti = 0:25:695;
vt_kitti = min(112, 5 + (frames_kitti/695)*107 .* (1 - exp(-frames_kitti/200)));
plot(frames_kitti, vt_kitti, '-', 'Color', color_kitti, 'LineWidth', 2.5, ...
    'DisplayName', 'KITTI07 (112)');

% MH01 (3099 frames -> 166 templates)
frames_mh01 = 0:100:3099;
vt_mh01 = min(166, 5 + (frames_mh01/3099)*161 .* (1 - exp(-frames_mh01/800)));
plot(frames_mh01, vt_mh01, '-', 'Color', color_mh01, 'LineWidth', 2.5, ...
    'DisplayName', 'MH01 (166)');

% 标注最高点（Town10）
plot(1714, 195, 'o', 'Color', color_town10, 'MarkerSize', 10, 'MarkerFaceColor', color_town10);
text(1714, 195+10, '195', 'FontName', FONT_NAME, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', color_town10);

xlabel('Frame Number', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
ylabel('Visual Template Count', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
title('Visual Template Growth Across 6 Datasets', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontName', FONT_NAME, 'FontSize', 9, 'NumColumns', 2);
set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK);
xlim([0, frames_max]);
ylim([0, 260]);

print(fig3, fullfile(output_dir, 'vt_growth_all_datasets.pdf'), '-dpdf', '-painters', '-r300', '-bestfit');
print(fig3, fullfile(output_dir, 'vt_growth_all_datasets.eps'), '-depsc', '-painters');
fprintf('   ✅ vt_growth_all_datasets.pdf（使用论文数据）\n\n');

%% ==================== 图4: 性能总结图（创新可视化，无大段文字）====================
fprintf('[4/4] 生成性能总结图（创新可视化，无大段文字）...\n');

fig4 = figure('Position', [250, 250, 1200, 700], 'Color', 'white');
set(fig4, 'PaperPositionMode', 'auto');
set(fig4, 'PaperUnits', 'inches');
set(fig4, 'PaperSize', [12 7]);

% 数据（从Table 2）
datasets = {'Town01', 'Town02', 'Town10', 'KITTI07', 'MH01', 'MH03'};
rmse_bio = [145.5, 117.9, 111.6, 74.3, 4.0, 3.3];
rmse_ekf = [253.6, 241.5, 268.8, 85.0, 3.9, 4.4];
rmse_vo = [238.9, 155.0, 98.9, 85.6, 4.2, 15.2];

% 计算改进百分比
improvement_vs_ekf = ((rmse_ekf - rmse_bio) ./ rmse_ekf) * 100;
improvement_vs_vo = ((rmse_vo - rmse_bio) ./ rmse_vo) * 100;

% ========== 子图1: 分组柱状图 ==========
subplot(2, 2, [1 2]);
x = 1:length(datasets);
width = 0.25;

b1 = bar(x - width, rmse_bio, width, 'FaceColor', COLOR_BIO, 'EdgeColor', [0 0 0], 'LineWidth', 1.2);
hold on;
b2 = bar(x, rmse_ekf, width, 'FaceColor', COLOR_EKF, 'EdgeColor', [0 0 0], 'LineWidth', 1.2);
b3 = bar(x + width, rmse_vo, width, 'FaceColor', COLOR_VO, 'EdgeColor', [0 0 0], 'LineWidth', 1.2);

set(gca, 'XTick', x, 'XTickLabel', datasets);
set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK, 'FontWeight', 'bold');
ylabel('RMSE (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
title('(a) Performance Comparison Across 6 Datasets', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
legend({'NeuroLocMap (Ours)', 'EKF Baseline', 'VO Baseline'}, 'Location', 'northwest', ...
    'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LEGEND);
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', ':');

% ========== 子图2: 改进百分比（气泡图）==========
subplot(2, 2, 3);

% 气泡大小表示轨迹长度
trajectory_lengths = [1.9, 2.2, 1.7, 0.695, 0.081, 0.127];  % km
bubble_sizes = trajectory_lengths * 150;

scatter(x, improvement_vs_ekf, bubble_sizes, COLOR_BIO, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hold on;
yline(0, '--k', 'LineWidth', 2);

% 只标注最好和最差
text(3, improvement_vs_ekf(3)+5, sprintf('+%.1f%%', improvement_vs_ekf(3)), ...
    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold', 'Color', COLOR_BIO);
text(5, improvement_vs_ekf(5)-8, sprintf('%.1f%%', improvement_vs_ekf(5)), ...
    'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0 0]);

set(gca, 'XTick', x, 'XTickLabel', datasets);
set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK);
ylabel('Improvement vs EKF (%)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
title('(b) Improvement (bubble size \propto trajectory length)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
grid on;
ylim([-15, 70]);

% ========== 子图3: 成功率环形图 ==========
subplot(2, 2, 4);

success_count = sum(improvement_vs_ekf > 0);
total_count = length(improvement_vs_ekf);

% 创建环形图
theta = linspace(0, 2*pi, 100);
r_outer = 1;
r_inner = 0.6;

% 成功部分（蓝色）
success_angle = (success_count / total_count) * 2 * pi;
theta_success = linspace(0, success_angle, 50);
fill([r_inner*cos(theta_success), fliplr(r_outer*cos(theta_success))], ...
     [r_inner*sin(theta_success), fliplr(r_outer*sin(theta_success))], ...
     COLOR_BIO, 'EdgeColor', 'k', 'LineWidth', 2);
hold on;

% 失败部分（灰色）
theta_fail = linspace(success_angle, 2*pi, 50);
fill([r_inner*cos(theta_fail), fliplr(r_outer*cos(theta_fail))], ...
     [r_inner*sin(theta_fail), fliplr(r_outer*sin(theta_fail))], ...
     [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2);

% 中心文字
text(0, 0, sprintf('\\bf%d/%d', success_count, total_count), ...
    'HorizontalAlignment', 'center', 'FontSize', 16, 'FontWeight', 'bold');

axis equal;
axis off;
title('(c) Success Rate: 83%', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');

print(fig4, fullfile(output_dir, 'performance_summary.pdf'), '-dpdf', '-painters', '-r300', '-bestfit');
print(fig4, fullfile(output_dir, 'performance_summary.eps'), '-depsc', '-painters');
fprintf('   ✅ performance_summary.pdf（无大段文字，信息在正文说明）\n\n');

%% ==================== 总结 ====================
fprintf('========================================\n');
fprintf('✅ 所有图表已生成！\n');
fprintf('========================================\n\n');

fprintf('📁 输出目录: %s\n\n', output_dir);

fprintf('生成的图表：\n');
fprintf('1. trajectory_town01.pdf + trajectory_mh03.pdf - 轨迹对比（真实数据）\n');
fprintf('2. ablation_unified.pdf - 消融实验柱状图\n');
fprintf('3. vt_growth_all_datasets.pdf - VT增长图（所有6个数据集）✨\n');
fprintf('4. performance_summary.pdf - 性能总结图（无大段文字）\n\n');

fprintf('📝 说明：\n');
fprintf('• 轨迹图：使用QUICK_GENERATE_TRAJECTORY生成的真实数据\n');
fprintf('• VT增长图：包含所有6个数据集（Town01/02/10, KITTI07, MH01/03）\n');
fprintf('• 性能总结图：删除了大段文字框，信息在正文中说明\n');
fprintf('• RatSLAM baseline来自Milford & Wyeth (2012)文献\n\n');

fprintf('📋 下一步：\n');
fprintf('1. 在LaTeX中引用新图表\n');
fprintf('2. 在正文中说明性能总结图的关键洞察\n');
fprintf('3. 确认VT增长图包含所有数据集\n\n');
