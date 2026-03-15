classdef matlab_template_statistics < handle
    % 模板统计收集类
    % 用于统计visual template数量和距离分布
    %
    % 使用方法：
    %   vt_stats = matlab_template_statistics();
    %   vt_stats.update(vt_id, num_templates);  % 每帧更新
    %   vt_stats.print_report('Town01');
    
    properties
        vt_history      % VT历史记录
        template_count  % 模板数量历史
        frame_count     % 帧计数
    end
    
    methods
        function obj = matlab_template_statistics()
            % 构造函数
            obj.vt_history = [];
            obj.template_count = [];
            obj.frame_count = 0;
        end
        
        function update(obj, vt_id, num_templates)
            % 更新统计
            obj.frame_count = obj.frame_count + 1;
            obj.vt_history = [obj.vt_history; vt_id];
            obj.template_count = [obj.template_count; num_templates];
        end
        
        function stats = get_statistics(obj)
            % 获取统计信息
            stats = struct();
            stats.total_frames = obj.frame_count;
            stats.final_template_count = obj.template_count(end);
            stats.unique_vt_count = length(unique(obj.vt_history));
            
            % 计算模板增长率
            if length(obj.template_count) > 1
                stats.template_growth = obj.template_count(end) - obj.template_count(1);
            else
                stats.template_growth = 0;
            end
        end
        
        function print_report(obj, dataset_name)
            % 打印报告
            if nargin < 2
                dataset_name = 'Unknown';
            end
            
            stats = obj.get_statistics();
            
            fprintf('\n');
            fprintf('========== Template Statistics: %s ==========\n', dataset_name);
            fprintf('Total Frames:          %d\n', stats.total_frames);
            fprintf('Final Template Count:  %d\n', stats.final_template_count);
            fprintf('Unique VT Count:       %d\n', stats.unique_vt_count);
            fprintf('Template Growth:       +%d\n', stats.template_growth);
            fprintf('=====================================================\n\n');
        end
        
        function plot_growth(obj)
            % 绘制模板增长曲线
            figure('Name', 'Template Growth');
            plot(obj.template_count, 'LineWidth', 2);
            xlabel('Frame Index');
            ylabel('Number of Templates');
            title('Visual Template Growth Over Time');
            grid on;
        end
        
        function save_statistics(obj, filename)
            % 保存统计数据
            stats = obj.get_statistics();
            vt_history = obj.vt_history;
            template_count = obj.template_count;
            save(filename, 'stats', 'vt_history', 'template_count');
            fprintf('Template statistics saved to %s\n', filename);
        end
    end
end
