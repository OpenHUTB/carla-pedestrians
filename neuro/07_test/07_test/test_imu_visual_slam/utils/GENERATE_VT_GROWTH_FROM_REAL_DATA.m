%% 使用1月5日真实数据生成VT增长图
% 从MAT文件读取实际的VT增长数据

clear; close all; clc;

fprintf('\n========================================\n');
fprintf('  使用真实数据生成VT增长图\n');
fprintf('========================================\n\n');

%% ==================== 全局配置 ====================
COLOR_BIO = [0.0, 0.45, 0.74];
FONT_NAME = 'Arial';
FONT_SIZE_TITLE = 12;
FONT_SIZE_LABEL = 11;
FONT_SIZE_TICK = 10;
FONT_SIZE_LEGEND = 10;

output_dir = 'E:\Neuro_end\neuro\kbs\kbs\NeuroSLAM_KBS_Submission\fig';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% ==================== 读取真实数据 ====================
fprintf('[1/2] 读取1月5日的真实VT数据...\n');

% 数据路径
datasets = {
    'Town01', 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\slam_results\vt_stats_Town01Data_IMU_Fusion.mat', [0.0, 0.45, 0.74];
    'Town02', 'E:\Neuro_end\neuro\data\Town02Data_IMU_Fusion\slam_results\vt_stats_Town02Data_IMU_Fusion.mat', [0.85, 0.33, 0.10];
    'Town10', 'E:\Neuro_end\neuro\data\Town10Data_IMU_Fusion\slam_results\vt_stats_Town10Data_IMU_Fusion.mat', [0.47, 0.67, 0.19];
    'KITTI07', 'E:\Neuro_end\neuro\data\KITTI_07\slam_results\vt_stats_KITTI_07.mat', [0.49, 0.18, 0.56];
    'MH01', 'E:\Neuro_end\neuro\data\MH_01_easy\MH_01_easy\slam_results\vt_stats_MH_01_easy.mat', [0.93, 0.69, 0.13];
    'MH03', 'E:\Neuro_end\neuro\data\MH_03_medium\MH_03_medium\slam_results\vt_stats_MH_03_medium.mat', [0.30, 0.75, 0.93];
};

vt_data = cell(size(datasets, 1), 1);
max_frames = 0;

for i = 1:size(datasets, 1)
    dataset_name = datasets{i, 1};
    file_path = datasets{i, 2};
    
    if exist(file_path, 'file')
        data = load(file_path);
        
        if isfield(data, 'template_count')
            vt_data{i}.name = dataset_name;
            vt_data{i}.vt_growth = data.template_count;
            vt_data{i}.frames = length(data.template_count);
            vt_data{i}.final_vt = data.template_count(end);
            vt_data{i}.color = datasets{i, 3};
            
            max_frames = max(max_frames, vt_data{i}.frames);
            
            fprintf('   ✓ %s: %d templates (%d frames)\n', dataset_name, vt_data{i}.final_vt, vt_data{i}.frames);
        else
            fprintf('   ✗ %s: 字段不正确\n', dataset_name);
            vt_data{i} = [];
        end
    else
        fprintf('   ✗ %s: 文件不存在\n', dataset_name);
        vt_data{i} = [];
    end
end

fprintf('\n');

%% ==================== 生成VT增长图 ====================
fprintf('[2/2] 生成VT增长图...\n');

fig = figure('Position', [200, 200, 1000, 650], 'Color', 'white');
set(fig, 'PaperPositionMode', 'auto');
set(fig, 'PaperUnits', 'inches');
set(fig, 'PaperSize', [10 6.5]);

hold on; grid on;
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3);

% RatSLAM baseline
frames_baseline = 0:100:max_frames;
vt_baseline = ones(size(frames_baseline)) * 5;
plot(frames_baseline, vt_baseline, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5, ...
    'DisplayName', 'RatSLAM baseline (~5)');

% 绘制所有数据集
max_vt = 0;
for i = 1:length(vt_data)
    if ~isempty(vt_data{i})
        frames = 0:vt_data{i}.frames-1;
        vt_growth = vt_data{i}.vt_growth;
        
        plot(frames, vt_growth, '-', 'Color', vt_data{i}.color, 'LineWidth', 2.5, ...
            'DisplayName', sprintf('%s (%d)', vt_data{i}.name, vt_data{i}.final_vt));
        
        max_vt = max(max_vt, vt_data{i}.final_vt);
    end
end

% 标注最高点
for i = 1:length(vt_data)
    if ~isempty(vt_data{i}) && vt_data{i}.final_vt == max_vt
        plot(vt_data{i}.frames-1, vt_data{i}.final_vt, 'o', ...
            'Color', vt_data{i}.color, 'MarkerSize', 10, 'MarkerFaceColor', vt_data{i}.color);
        text(vt_data{i}.frames-1, vt_data{i}.final_vt+10, sprintf('%d', vt_data{i}.final_vt), ...
            'FontName', FONT_NAME, 'FontSize', 9, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', vt_data{i}.color);
        break;
    end
end

xlabel('Frame Number', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
ylabel('Visual Template Count', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL, 'FontWeight', 'bold');
title('Visual Template Growth Across 6 Datasets (Jan 5, 2026 Data)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontName', FONT_NAME, 'FontSize', 9, 'NumColumns', 2);
set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TICK);
xlim([0, max_frames]);
ylim([0, max_vt*1.1]);

print(fig, fullfile(output_dir, 'vt_growth_all_datasets_real.pdf'), '-dpdf', '-painters', '-r300', '-bestfit');
print(fig, fullfile(output_dir, 'vt_growth_all_datasets_real.eps'), '-depsc', '-painters');

fprintf('   ✅ vt_growth_all_datasets_real.pdf\n\n');

%% ==================== 总结 ====================
fprintf('========================================\n');
fprintf('✅ VT增长图生成完成（使用真实数据）\n');
fprintf('========================================\n\n');

fprintf('📊 数据来源：1月5日生成的MAT文件\n');
fprintf('📁 输出位置：%s\n\n', output_dir);

fprintf('📝 各数据集VT数量：\n');
for i = 1:length(vt_data)
    if ~isempty(vt_data{i})
        fprintf('  • %s: %d templates (%d frames)\n', vt_data{i}.name, vt_data{i}.final_vt, vt_data{i}.frames);
    end
end
fprintf('\n');

fprintf('💡 提示：\n');
fprintf('• 这是使用1月5日真实数据生成的VT增长图\n');
fprintf('• 如果某些数据集缺失，请先运行对应的SLAM\n');
fprintf('• 图表文件名：vt_growth_all_datasets_real.pdf\n\n');
