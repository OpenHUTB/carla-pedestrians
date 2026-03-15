classdef matlab_performance_monitor < handle
    % 性能监控类 - 用于测量各模块运行时间
    % 解决论文中21 FPS数据来源问题
    %
    % 使用方法：
    %   perf = matlab_performance_monitor();
    %   perf.start('visual_processing');
    %   % ... 你的代码 ...
    %   perf.stop('visual_processing');
    %   perf.print_report();
    %   perf.save_report('performance_Town01.mat');
    
    properties
        timings  % 存储计时数据
        starts   % 存储开始时间
    end
    
    methods
        function obj = matlab_performance_monitor()
            % 构造函数
            obj.timings = containers.Map();
            obj.starts = containers.Map();
        end
        
        function start(obj, component_name)
            % 开始计时
            obj.starts(component_name) = tic;
        end
        
        function stop(obj, component_name)
            % 结束计时并记录
            if isKey(obj.starts, component_name)
                elapsed = toc(obj.starts(component_name));
                
                if ~isKey(obj.timings, component_name)
                    obj.timings(component_name) = [];
                end
                obj.timings(component_name) = [obj.timings(component_name); elapsed * 1000];  % 转换为ms
            else
                warning('Component %s not started!', component_name);
            end
        end
        
        function stats = get_stats(obj)
            % 获取统计信息
            stats = struct();
            component_names = keys(obj.timings);
            
            for i = 1:length(component_names)
                name = component_names{i};
                times = obj.timings(name);
                
                if ~isempty(times)
                    stats.(name).mean_ms = mean(times);
                    stats.(name).std_ms = std(times);
                    stats.(name).min_ms = min(times);
                    stats.(name).max_ms = max(times);
                    stats.(name).freq_hz = 1000.0 / mean(times);
                    stats.(name).count = length(times);
                end
            end
        end
        
        function print_report(obj)
            % 打印性能报告
            fprintf('\n');
            fprintf('======================================================================\n');
            fprintf('Performance Report\n');
            fprintf('======================================================================\n');
            fprintf('%-30s %12s %12s %12s\n', 'Component', 'Mean(ms)', 'Std(ms)', 'Freq(Hz)');
            fprintf('----------------------------------------------------------------------\n');
            
            stats = obj.get_stats();
            component_names = fieldnames(stats);
            total_time = 0;
            
            for i = 1:length(component_names)
                name = component_names{i};
                data = stats.(name);
                fprintf('%-30s %12.2f %12.2f %12.2f\n', ...
                    name, data.mean_ms, data.std_ms, data.freq_hz);
                
                if ~strcmp(name, 'total')
                    total_time = total_time + data.mean_ms;
                end
            end
            
            fprintf('----------------------------------------------------------------------\n');
            if total_time > 0
                fps = 1000.0 / total_time;
                fprintf('%-30s %12.2f %12s %12.2f\n', 'TOTAL PIPELINE', total_time, '---', fps);
            end
            fprintf('======================================================================\n\n');
        end
        
        function save_report(obj, filename)
            % 保存报告到MAT文件
            stats = obj.get_stats();
            save(filename, 'stats');
            fprintf('Performance report saved to %s\n', filename);
        end
        
        function latex_table = generate_latex_table(obj)
            % 生成LaTeX表格代码
            stats = obj.get_stats();
            component_names = fieldnames(stats);
            
            latex_table = sprintf('\\begin{table}[h]\n');
            latex_table = [latex_table sprintf('\\centering\n')];
            latex_table = [latex_table sprintf('\\caption{Runtime Performance Analysis}\n')];
            latex_table = [latex_table sprintf('\\begin{tabular}{lccc}\n')];
            latex_table = [latex_table sprintf('\\toprule\n')];
            latex_table = [latex_table sprintf('Component & Mean (ms) & Std (ms) & Freq (Hz) \\\\\n')];
            latex_table = [latex_table sprintf('\\midrule\n')];
            
            total_time = 0;
            for i = 1:length(component_names)
                name = component_names{i};
                data = stats.(name);
                latex_name = strrep(name, '_', '\\_');  % 转义下划线
                latex_table = [latex_table sprintf('%s & %.1f & %.1f & %.1f \\\\\n', ...
                    latex_name, data.mean_ms, data.std_ms, data.freq_hz)];
                
                if ~strcmp(name, 'total')
                    total_time = total_time + data.mean_ms;
                end
            end
            
            latex_table = [latex_table sprintf('\\midrule\n')];
            fps = 1000.0 / total_time;
            latex_table = [latex_table sprintf('\\textbf{Total Pipeline} & \\textbf{%.1f} & \\textbf{---} & \\textbf{%.1f} \\\\\n', total_time, fps)];
            latex_table = [latex_table sprintf('\\bottomrule\n')];
            latex_table = [latex_table sprintf('\\end{tabular}\n')];
            latex_table = [latex_table sprintf('\\end{table}\n')];
            
            fprintf('\n%s\n', latex_table);
        end
    end
end
