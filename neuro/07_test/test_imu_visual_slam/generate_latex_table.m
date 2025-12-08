function generate_latex_table(results, output_dir)
%% 生成LaTeX格式表格（用于论文）

fprintf('正在生成LaTeX表格...\n');

%% 提取数据
exp_labels = {
    'Full System (Baseline)';
    'w/o IMU';
    'w/o LSTM';
    'w/o Transformer';
    'w/o Dual-Stream';
    'w/o Attention';
    'Simplified Feature'
};

vt_counts = results.vt_counts;
exp_counts = results.exp_counts;
rmse_values = results.rmse_values;
rpe_values = results.rpe_values;
drift_rates = results.drift_rates;

%% 计算相对变化
baseline_idx = 1;
rel_rmse = ((rmse_values - rmse_values(baseline_idx)) / rmse_values(baseline_idx)) * 100;

%% 生成主表格
fid = fopen(fullfile(output_dir, 'ablation_results_latex.tex'), 'w');

fprintf(fid, '%% 消融实验结果表格 - LaTeX格式\n');
fprintf(fid, '%% 使用方法: 复制到论文的table环境中\n\n');

% 表格1: 完整结果
fprintf(fid, '%% ===== 表格1: 完整性能对比 =====\n');
fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Ablation Study Results on Town01 Dataset (5000 frames)}\n');
fprintf(fid, '\\label{tab:ablation_full}\n');
fprintf(fid, '\\begin{tabular}{l|ccc|cc}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, '\\textbf{Configuration} & \\textbf{VT} & \\textbf{Exp.} & \\textbf{RMSE} & \\textbf{RPE} & \\textbf{Drift} \\\\\n');
fprintf(fid, '                       & \\textbf{Count} & \\textbf{Nodes} & \\textbf{(m)} & \\textbf{(m)} & \\textbf{Rate (\\%%)} \\\\\n');
fprintf(fid, '\\hline\n');

for i = 1:length(exp_labels)
    if i == 1
        % Baseline加粗
        fprintf(fid, '\\textbf{%s} & ', exp_labels{i});
    else
        fprintf(fid, '%s & ', exp_labels{i});
    end
    
    % 高亮最佳值
    if rmse_values(i) == min(rmse_values)
        fprintf(fid, '%d & %d & \\textbf{%.2f} & %.4f & %.2f \\\\\n', ...
            vt_counts(i), exp_counts(i), rmse_values(i), rpe_values(i), drift_rates(i));
    else
        fprintf(fid, '%d & %d & %.2f & %.4f & %.2f \\\\\n', ...
            vt_counts(i), exp_counts(i), rmse_values(i), rpe_values(i), drift_rates(i));
    end
end

fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n\n');

% 表格2: 相对性能变化
fprintf(fid, '%% ===== 表格2: 相对Baseline的性能变化 =====\n');
fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Performance Changes Relative to Full System}\n');
fprintf(fid, '\\label{tab:ablation_relative}\n');
fprintf(fid, '\\begin{tabular}{l|ccc}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, '\\textbf{Configuration} & \\textbf{VT} & \\textbf{Exp.} & \\textbf{RMSE} \\\\\n');
fprintf(fid, '                       & \\textbf{Change (\\%%)} & \\textbf{Change (\\%%)} & \\textbf{Change (\\%%)} \\\\\n');
fprintf(fid, '\\hline\n');

for i = 1:length(exp_labels)
    if i == 1
        fprintf(fid, '\\textbf{%s} & - & - & - \\\\\n', exp_labels{i});
    else
        rel_vt_val = ((vt_counts(i) - vt_counts(1)) / vt_counts(1)) * 100;
        rel_exp_val = ((exp_counts(i) - exp_counts(1)) / exp_counts(1)) * 100;
        
        % 负值用绿色，正值用红色
        if rel_rmse(i) > 0
            rmse_str = sprintf('\\textcolor{red}{+%.1f}', rel_rmse(i));
        else
            rmse_str = sprintf('\\textcolor{green}{%.1f}', rel_rmse(i));
        end
        
        fprintf(fid, '%s & %+.1f & %+.1f & %s \\\\\n', ...
            exp_labels{i}, rel_vt_val, rel_exp_val, rmse_str);
    end
end

fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n\n');

% 表格3: 组件重要性排名
fprintf(fid, '%% ===== 表格3: 组件重要性排名 =====\n');
fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Component Importance Ranking}\n');
fprintf(fid, '\\label{tab:component_importance}\n');
fprintf(fid, '\\begin{tabular}{c|l|c}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, '\\textbf{Rank} & \\textbf{Component} & \\textbf{RMSE Increase (m)} \\\\\n');
fprintf(fid, '\\hline\n');

% 计算组件影响并排序
component_names = {'IMU Fusion', 'LSTM Memory', 'Transformer', 'Dual-Stream', 'Attention', 'Full Features'};
rmse_impacts = rmse_values(2:7) - rmse_values(1);
[sorted_impacts, sort_idx] = sort(rmse_impacts, 'descend');

for i = 1:length(sorted_impacts)
    fprintf(fid, '%d & %s & +%.2f \\\\\n', ...
        i, component_names{sort_idx(i)}, sorted_impacts(i));
end

fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n\n');

% 简化版表格（用于space-limited论文）
fprintf(fid, '%% ===== 简化表格（节省空间） =====\n');
fprintf(fid, '\\begin{table}[htbp]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Ablation Study (Compact)}\n');
fprintf(fid, '\\label{tab:ablation_compact}\n');
fprintf(fid, '\\small\n');  % 使用小字体
fprintf(fid, '\\begin{tabular}{l|cc|c}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, '\\textbf{Config.} & \\textbf{VT} & \\textbf{Nodes} & \\textbf{RMSE (m)} \\\\\n');
fprintf(fid, '\\hline\n');

% 只显示关键配置
key_indices = [1, 2, 3, 7];  % 完整、无IMU、无LSTM、简化
key_labels = {'Full', 'w/o IMU', 'w/o LSTM', 'Simple'};

for i = 1:length(key_indices)
    idx = key_indices(i);
    if idx == 1
        fprintf(fid, '\\textbf{%s} & ', key_labels{i});
    else
        fprintf(fid, '%s & ', key_labels{i});
    end
    
    if rmse_values(idx) == min(rmse_values(key_indices))
        fprintf(fid, '%d & %d & \\textbf{%.2f} \\\\\n', ...
            vt_counts(idx), exp_counts(idx), rmse_values(idx));
    else
        fprintf(fid, '%d & %d & %.2f \\\\\n', ...
            vt_counts(idx), exp_counts(idx), rmse_values(idx));
    end
end

fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n\n');

% 添加使用说明
fprintf(fid, '%% ===== 使用说明 =====\n');
fprintf(fid, '%% 1. 确保导言区包含: \\usepackage{xcolor}\n');
fprintf(fid, '%% 2. 完整表格适用于详细分析章节\n');
fprintf(fid, '%% 3. 简化表格适用于空间受限的论文\n');
fprintf(fid, '%% 4. 根据需要调整表格宽度和字体大小\n');

fclose(fid);
fprintf('  ✓ LaTeX表格已保存\n');

%% 生成论文用的文字描述
fid = fopen(fullfile(output_dir, 'ablation_description.tex'), 'w');

fprintf(fid, '%% 消融实验结果描述（可直接用于论文）\n\n');

fprintf(fid, '\\subsection{Ablation Study}\n\n');

fprintf(fid, 'To validate the contribution of each component in our proposed system, ');
fprintf(fid, 'we conduct a comprehensive ablation study on the Town01 dataset (5000 frames). ');
fprintf(fid, 'Table~\\ref{tab:ablation_full} presents the complete results.\n\n');

% 找出最重要的组件
rmse_impacts = rmse_values(2:7) - rmse_values(1);
[max_impact, max_idx] = max(rmse_impacts);
component_names = {'IMU fusion', 'LSTM memory', 'Transformer', 'dual-stream architecture', 'spatial attention', 'full feature extraction'};

fprintf(fid, '\\textbf{Key Findings:} ');
fprintf(fid, '(1) The full system achieves an RMSE of %.2f~m with %d visual templates and %d experience map nodes. ', ...
    rmse_values(1), vt_counts(1), exp_counts(1));
fprintf(fid, '(2) Removing %s causes the largest performance degradation (RMSE increases by %.2f~m), ', ...
    component_names{max_idx}, max_impact);
fprintf(fid, 'indicating its critical role in the system. ');

% 找出最小影响的组件
[min_impact, min_idx] = min(rmse_impacts);
fprintf(fid, '(3) Removing %s has the smallest impact (RMSE increases by %.2f~m), ', ...
    component_names{min_idx}, min_impact);
fprintf(fid, 'suggesting relative redundancy or compensatory mechanisms. ');

fprintf(fid, '(4) The full system outperforms the simplified baseline by %.2f~m in RMSE, ', ...
    abs(rmse_values(1) - rmse_values(7)));
fprintf(fid, 'demonstrating the effectiveness of our bio-inspired design.\n\n');

fclose(fid);
fprintf('  ✓ 论文描述文本已保存\n');

fprintf('LaTeX表格生成完成！\n');

end
