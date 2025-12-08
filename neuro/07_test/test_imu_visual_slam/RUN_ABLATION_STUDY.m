%% 快速运行消融实验
% 这是简化的入口脚本，方便快速执行消融实验
%
% 使用方法：
%   1. 直接运行此脚本
%   2. 或在命令行中输入: RUN_ABLATION_STUDY
%
% 输出：
%   - 6种精美对比图表
%   - 3种格式表格（Markdown, HTML, CSV）
%   - LaTeX论文表格
%   - 所有结果保存在 ablation_results 文件夹

clear all; close all; clc;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                                                              ║\n');
fprintf('║      HART+Transformer SLAM 消融实验                          ║\n');
fprintf('║      Ablation Study Framework                                ║\n');
fprintf('║                                                              ║\n');
fprintf('║  本实验将验证系统各组件的贡献                                  ║\n');
fprintf('║  预计运行时间: 约30-45分钟                                    ║\n');
fprintf('║                                                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 确认运行
fprintf('实验将运行以下7个配置:\n');
fprintf('  1. 完整系统 (Baseline)\n');
fprintf('  2. 去掉IMU (纯视觉)\n');
fprintf('  3. 去掉LSTM (无时序记忆)\n');
fprintf('  4. 去掉Transformer (无全局上下文)\n');
fprintf('  5. 去掉双流 (单特征流)\n');
fprintf('  6. 去掉注意力 (无空间注意力)\n');
fprintf('  7. 简化特征 (基础方法对比)\n');
fprintf('\n');

response = input('确认开始? (y/n): ', 's');
if ~strcmpi(response, 'y')
    fprintf('已取消实验。\n');
    return;
end

%% 运行主实验
fprintf('\n开始运行消融实验...\n\n');

try
    ablation_study_main();
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║                   ✓ 实验成功完成！                            ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');
    
    % 显示结果位置
    results_dir = '../../../data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/ablation_results';
    fprintf('结果文件:\n');
    fprintf('  📊 图表: %s\n', fullfile(results_dir, 'ablation_*.png'));
    fprintf('  📋 表格: %s\n', fullfile(results_dir, 'ablation_results_table.*'));
    fprintf('  📄 LaTeX: %s\n', fullfile(results_dir, 'ablation_results_latex.tex'));
    fprintf('\n');
    
    % 打开结果文件夹
    if ismac
        system(sprintf('open "%s"', results_dir));
    elseif isunix
        system(sprintf('xdg-open "%s"', results_dir));
    elseif ispc
        winopen(results_dir);
    end
    
catch ME
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║                   ✗ 实验出错                                  ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('\n错误信息: %s\n', ME.message);
    fprintf('错误位置: %s (第%d行)\n', ME.stack(1).name, ME.stack(1).line);
    fprintf('\n');
    rethrow(ME);
end
