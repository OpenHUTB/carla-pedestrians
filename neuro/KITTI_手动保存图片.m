%% KITTI测试结果 - 手动保存图片
% 如果图片保存失败，运行此脚本手动保存当前显示的图形

% 结果保存路径
result_path = 'E:\Neuro_end\neuro\data\KITTI_07\KITTI_07\comparison_results';

% 确保目录存在
if ~exist(result_path, 'dir')
    mkdir(result_path);
end

% 获取所有打开的图形窗口
figs = findall(0, 'Type', 'figure');

fprintf('找到 %d 个打开的图形窗口\n', length(figs));

% 保存每个图形
for i = 1:length(figs)
    fig = figs(i);
    fig_name = get(fig, 'Name');
    
    if isempty(fig_name)
        fig_name = sprintf('figure_%d', i);
    end
    
    % 清理文件名（移除特殊字符）
    fig_name = strrep(fig_name, ' ', '_');
    fig_name = strrep(fig_name, ':', '');
    fig_name = strrep(fig_name, '/', '_');
    
    % 保存为PNG（使用print避免saveas的bug）
    output_file = fullfile(result_path, [fig_name, '.png']);
    try
        print(fig, output_file, '-dpng', '-r150');
        fprintf('✓ 已保存: %s\n', output_file);
    catch ME
        warning('保存失败: %s - %s', fig_name, ME.message);
    end
end

fprintf('\n所有图片已保存到: %s\n', result_path);
