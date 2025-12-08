function generate_ablation_tables(results, output_dir)
%% 生成消融实验对比表格

fprintf('正在生成对比表格...\n');

%% 提取数据
exp_labels = {
    '完整系统 (Baseline)';
    '去掉IMU';
    '去掉LSTM';
    '去掉Transformer';
    '去掉双流';
    '去掉注意力';
    '简化特征'
};

vt_counts = results.vt_counts;
exp_counts = results.exp_counts;
rmse_values = results.rmse_values;
rpe_values = results.rpe_values;
drift_rates = results.drift_rates;
times = results.processing_times;

%% 计算相对变化
baseline_idx = 1;
rel_rmse = ((rmse_values - rmse_values(baseline_idx)) / rmse_values(baseline_idx)) * 100;
rel_vt = ((vt_counts - vt_counts(baseline_idx)) / vt_counts(baseline_idx)) * 100;
rel_exp = ((exp_counts - exp_counts(baseline_idx)) / exp_counts(baseline_idx)) * 100;

%% 生成Markdown表格
fid = fopen(fullfile(output_dir, 'ablation_results_table.md'), 'w');

fprintf(fid, '# 消融实验结果对比表\n\n');
fprintf(fid, '## 1. 完整性能指标\n\n');
fprintf(fid, '| 实验配置 | VT数量 | 经验节点 | RMSE (m) | RPE (m) | 漂移率 (%%) | 时间 (s) |\n');
fprintf(fid, '|---------|--------|----------|----------|---------|-----------|----------|\n');

for i = 1:length(exp_labels)
    fprintf(fid, '| %s | %d | %d | %.2f | %.4f | %.2f | %.1f |\n', ...
        exp_labels{i}, vt_counts(i), exp_counts(i), rmse_values(i), ...
        rpe_values(i), drift_rates(i), times(i));
end

fprintf(fid, '\n## 2. 相对Baseline的变化\n\n');
fprintf(fid, '| 实验配置 | VT变化 (%%) | 节点变化 (%%) | RMSE变化 (%%) |\n');
fprintf(fid, '|---------|-----------|-------------|-------------|\n');

for i = 1:length(exp_labels)
    if i == 1
        fprintf(fid, '| %s | - | - | - |\n', exp_labels{i});
    else
        fprintf(fid, '| %s | %+.1f | %+.1f | %+.1f |\n', ...
            exp_labels{i}, rel_vt(i), rel_exp(i), rel_rmse(i));
    end
end

fprintf(fid, '\n## 3. 性能排名\n\n');

% RMSE排名
[sorted_rmse, rmse_rank] = sort(rmse_values);
fprintf(fid, '### 3.1 定位精度排名 (RMSE)\n\n');
fprintf(fid, '| 排名 | 配置 | RMSE (m) |\n');
fprintf(fid, '|------|------|----------|\n');
for i = 1:length(rmse_rank)
    idx = rmse_rank(i);
    fprintf(fid, '| %d | %s | %.2f |\n', i, exp_labels{idx}, sorted_rmse(i));
end

% VT数量排名
[sorted_vt, vt_rank] = sort(vt_counts, 'descend');
fprintf(fid, '\n### 3.2 VT数量排名\n\n');
fprintf(fid, '| 排名 | 配置 | VT数量 |\n');
fprintf(fid, '|------|------|--------|\n');
for i = 1:length(vt_rank)
    idx = vt_rank(i);
    fprintf(fid, '| %d | %s | %d |\n', i, exp_labels{idx}, sorted_vt(i));
end

fprintf(fid, '\n## 4. 关键发现\n\n');
fprintf(fid, '### 4.1 最重要的组件\n\n');

% 找出对RMSE影响最大的组件
rmse_impacts = rmse_values(2:end) - rmse_values(1);
[max_impact, max_idx] = max(rmse_impacts);
fprintf(fid, '- **影响最大**: %s (RMSE增加 %.2f m)\n', exp_labels{max_idx + 1}, max_impact);

[min_impact, min_idx] = min(rmse_impacts);
fprintf(fid, '- **影响最小**: %s (RMSE增加 %.2f m)\n', exp_labels{min_idx + 1}, min_impact);

fprintf(fid, '\n### 4.2 组件重要性排序\n\n');
[sorted_impacts, impact_rank] = sort(rmse_impacts, 'descend');
for i = 1:length(impact_rank)
    idx = impact_rank(i) + 1;
    fprintf(fid, '%d. %s: +%.2f m\n', i, exp_labels{idx}, sorted_impacts(i));
end

fclose(fid);
fprintf('  ✓ Markdown表格已保存\n');

%% 生成HTML表格
fid = fopen(fullfile(output_dir, 'ablation_results_table.html'), 'w');

fprintf(fid, '<!DOCTYPE html>\n<html>\n<head>\n');
fprintf(fid, '<meta charset="UTF-8">\n');
fprintf(fid, '<title>消融实验结果对比</title>\n');
fprintf(fid, '<style>\n');
fprintf(fid, 'body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }\n');
fprintf(fid, 'h1 { color: #333; text-align: center; }\n');
fprintf(fid, 'h2 { color: #555; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; }\n');
fprintf(fid, 'table { border-collapse: collapse; width: 100%%; margin: 20px 0; background-color: white; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }\n');
fprintf(fid, 'th { background-color: #4CAF50; color: white; padding: 12px; text-align: left; }\n');
fprintf(fid, 'td { padding: 10px; border-bottom: 1px solid #ddd; }\n');
fprintf(fid, 'tr:hover { background-color: #f5f5f5; }\n');
fprintf(fid, '.baseline { background-color: #e8f5e9; font-weight: bold; }\n');
fprintf(fid, '.best { color: #4CAF50; font-weight: bold; }\n');
fprintf(fid, '.worst { color: #f44336; }\n');
fprintf(fid, '.positive { color: #f44336; }\n');
fprintf(fid, '.negative { color: #4CAF50; }\n');
fprintf(fid, '</style>\n');
fprintf(fid, '</head>\n<body>\n');

fprintf(fid, '<h1>🔬 消融实验结果对比</h1>\n');

% 表格1: 完整指标
fprintf(fid, '<h2>1. 完整性能指标</h2>\n');
fprintf(fid, '<table>\n');
fprintf(fid, '<tr><th>实验配置</th><th>VT数量</th><th>经验节点</th><th>RMSE (m)</th><th>RPE (m)</th><th>漂移率 (%%)</th><th>时间 (s)</th></tr>\n');

for i = 1:length(exp_labels)
    if i == 1
        row_class = ' class="baseline"';
    else
        row_class = '';
    end
    
    % 高亮最佳值
    rmse_class = '';
    if rmse_values(i) == min(rmse_values)
        rmse_class = ' class="best"';
    end
    
    vt_class = '';
    if vt_counts(i) == max(vt_counts)
        vt_class = ' class="best"';
    end
    
    fprintf(fid, '<tr%s><td>%s</td><td%s>%d</td><td>%d</td><td%s>%.2f</td><td>%.4f</td><td>%.2f</td><td>%.1f</td></tr>\n', ...
        row_class, exp_labels{i}, vt_class, vt_counts(i), exp_counts(i), ...
        rmse_class, rmse_values(i), rpe_values(i), drift_rates(i), times(i));
end

fprintf(fid, '</table>\n');

% 表格2: 相对变化
fprintf(fid, '<h2>2. 相对Baseline的性能变化</h2>\n');
fprintf(fid, '<table>\n');
fprintf(fid, '<tr><th>实验配置</th><th>VT变化 (%%)</th><th>节点变化 (%%)</th><th>RMSE变化 (%%)</th></tr>\n');

for i = 1:length(exp_labels)
    if i == 1
        fprintf(fid, '<tr class="baseline"><td>%s</td><td>-</td><td>-</td><td>-</td></tr>\n', exp_labels{i});
    else
        % 正负值着色
        vt_class = '';
        if rel_vt(i) < 0
            vt_class = ' class="negative"';
        elseif rel_vt(i) > 0
            vt_class = ' class="positive"';
        end
        
        exp_class = '';
        if rel_exp(i) < 0
            exp_class = ' class="negative"';
        elseif rel_exp(i) > 0
            exp_class = ' class="positive"';
        end
        
        rmse_class = '';
        if rel_rmse(i) < 0
            rmse_class = ' class="negative"';
        elseif rel_rmse(i) > 0
            rmse_class = ' class="positive"';
        end
        
        fprintf(fid, '<tr><td>%s</td><td%s>%+.1f</td><td%s>%+.1f</td><td%s>%+.1f</td></tr>\n', ...
            exp_labels{i}, vt_class, rel_vt(i), exp_class, rel_exp(i), rmse_class, rel_rmse(i));
    end
end

fprintf(fid, '</table>\n');

% 关键发现
fprintf(fid, '<h2>3. 关键发现</h2>\n');
fprintf(fid, '<ul>\n');
fprintf(fid, '<li><strong>最佳配置:</strong> %s (RMSE = %.2f m)</li>\n', ...
    exp_labels{rmse_rank(1)}, sorted_rmse(1));
fprintf(fid, '<li><strong>影响最大的组件:</strong> %s (移除后RMSE增加 %.2f m)</li>\n', ...
    exp_labels{max_idx + 1}, max_impact);
fprintf(fid, '<li><strong>VT数量最多:</strong> %s (%d个)</li>\n', ...
    exp_labels{vt_rank(1)}, sorted_vt(1));
fprintf(fid, '</ul>\n');

fprintf(fid, '</body>\n</html>\n');
fclose(fid);
fprintf('  ✓ HTML表格已保存\n');

%% 生成CSV文件
T = table(exp_labels, vt_counts, exp_counts, rmse_values, rpe_values, drift_rates, times, ...
    'VariableNames', {'Configuration', 'VT_Count', 'Exp_Count', 'RMSE', 'RPE', 'Drift_Rate', 'Time'});
writetable(T, fullfile(output_dir, 'ablation_results.csv'));
fprintf('  ✓ CSV文件已保存\n');

fprintf('所有表格生成完成！\n');

end
