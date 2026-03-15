function out_files = paper_town02_visualization(dataset_name)
%PAPER_TOWN02_VISUALIZATION 生成论文风格的Town02可视化图
% 左侧：输入场景图片（带轨迹箭头）+ IMU数据
% 右侧：建图效果（轨迹对比）
%
% 风格参考：Real scene -> Digital twin 的对比图

if nargin < 1 || isempty(dataset_name)
    dataset_name = 'Town02Data_IMU_Fusion';
end

this_dir = fileparts(mfilename('fullpath'));
rootDir = fileparts(fileparts(fileparts(fileparts(this_dir))));

out_dir = fullfile(rootDir, 'data', 'paper_figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% 解析数据路径
data_path = resolve_dataset_path(rootDir, dataset_name);
result_path = resolve_results_path(data_path);

fprintf('Dataset: %s\n', data_path);
fprintf('Results: %s\n', result_path);

% 加载轨迹数据
traj_file = fullfile(result_path, 'trajectories.mat');
S = [];
if exist(traj_file, 'file')
    S = load(traj_file);
end

%% 创建2x2布局图
fig = figure('Color', 'white', 'Position', [50, 50, 1600, 900]);

% ========== 第一行：场景1 ==========
% 左上：输入图片（带轨迹箭头）
ax1 = subplot(2, 2, 1);
show_scene_with_trajectory(ax1, data_path, 100, [0.2 0.5 0.9], 'Input Scene 1');

% 右上：对应的建图效果
ax2 = subplot(2, 2, 2);
if ~isempty(S)
    plot_mapping_result(ax2, S, 'Mapping Result', [0.85 0.2 0.2]);
else
    text(ax2, 0.5, 0.5, 'No trajectory data', 'HorizontalAlignment', 'center');
    axis(ax2, 'off');
end

% ========== 第二行：场景2 ==========
% 左下：另一个输入图片（带轨迹箭头）
ax3 = subplot(2, 2, 3);
show_scene_with_trajectory(ax3, data_path, 2500, [0.2 0.5 0.9], 'Input Scene 2');

% 右下：IMU数据可视化
ax4 = subplot(2, 2, 4);
plot_imu_visualization(ax4, data_path);

% 添加箭头连接
add_connection_arrows(fig);

% 添加总标题
sgtitle(sprintf('%s - Inputs and Mapping Visualization', dataset_name), ...
    'FontSize', 16, 'FontWeight', 'bold', 'Interpreter', 'none');

%% 保存图片
out_base = fullfile(out_dir, sprintf('paper_%s_visualization', dataset_name));
out_png = [out_base '.png'];
out_pdf = [out_base '.pdf'];

out_files = {};
try
    exportgraphics(fig, out_png, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', out_png);
    out_files{end+1} = out_png;
catch
    print(fig, out_png, '-dpng', '-r300');
    fprintf('Saved (fallback): %s\n', out_png);
end

try
    exportgraphics(fig, out_pdf, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', out_pdf);
    out_files{end+1} = out_pdf;
catch
end

close(fig);
end
