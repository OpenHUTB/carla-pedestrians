%% 测试单个消融配置 - 快速Demo
% 用于快速测试特定的消融配置，无需运行完整的7组实验
%
% 使用方法：
%   修改下面的config配置，然后运行此脚本
%
% 预计时间：4-6分钟

clear all; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════\n');
fprintf('   单个消融配置快速测试\n');
fprintf('═══════════════════════════════════════════════════\n');
fprintf('\n');

%% ========== 配置区（修改这里） ==========

% 实验名称
exp_name = 'Test_Config';

% 组件开关
config = struct();
config.imu = true;          % 是否使用IMU融合
config.lstm = true;         % 是否使用LSTM时序建模
config.transformer = true;  % 是否使用Transformer全局上下文
config.dual_stream = true;  % 是否使用双流架构
config.attention = true;    % 是否使用空间注意力
config.full_feature = true; % 是否使用完整特征（false则用简化版）

%% ========================================

% 显示配置
fprintf('当前测试配置:\n');
fprintf('  - IMU融合:        %s\n', bool2str(config.imu));
fprintf('  - LSTM时序建模:   %s\n', bool2str(config.lstm));
fprintf('  - Transformer:    %s\n', bool2str(config.transformer));
fprintf('  - 双流架构:       %s\n', bool2str(config.dual_stream));
fprintf('  - 空间注意力:     %s\n', bool2str(config.attention));
fprintf('  - 完整特征:       %s\n', bool2str(config.full_feature));
fprintf('\n');

%% 运行测试
fprintf('开始运行...\n');
tic;

try
    result = run_single_ablation_experiment(exp_name, config);
    
    elapsed_time = toc;
    
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('   ✓ 测试完成！\n');
    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('\n');
    
    % 显示结果
    fprintf('性能指标:\n');
    fprintf('  VT数量:      %d\n', result.vt_count);
    fprintf('  经验节点:    %d\n', result.exp_count);
    fprintf('  RMSE:        %.2f m\n', result.rmse);
    fprintf('  RPE:         %.4f m\n', result.rpe);
    fprintf('  漂移率:      %.2f%%\n', result.drift_rate);
    fprintf('  处理时间:    %.1f 秒 (%.1f 分钟)\n', elapsed_time, elapsed_time/60);
    fprintf('\n');
    
    % 生成简单对比图
    generate_single_result_plot(result, exp_name);
    
catch ME
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('   ✗ 测试出错\n');
    fprintf('═══════════════════════════════════════════════════\n');
    fprintf('\n');
    fprintf('错误信息: %s\n', ME.message);
    fprintf('错误位置: %s (第%d行)\n', ME.stack(1).name, ME.stack(1).line);
    fprintf('\n');
end

%% 辅助函数
function str = bool2str(val)
    if val
        str = '✓ 启用';
    else
        str = '✗ 禁用';
    end
end

function generate_single_result_plot(result, exp_name)
    % 生成单个结果的可视化
    
    fig = figure('Position', [100, 100, 1200, 400]);
    set(fig, 'Color', 'w');
    
    % 子图1：VT和经验节点
    subplot(1, 3, 1);
    categories = {'VT数量', '经验节点'};
    values = [result.vt_count, result.exp_count];
    bar(values, 'FaceColor', [0.4, 0.7, 0.9]);
    set(gca, 'XTickLabel', categories);
    ylabel('数量', 'FontSize', 12, 'FontWeight', 'bold');
    title('地图统计', 'FontSize', 13, 'FontWeight', 'bold');
    grid on;
    for i = 1:length(values)
        text(i, values(i) + max(values)*0.05, sprintf('%d', values(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    end
    
    % 子图2：误差指标
    subplot(1, 3, 2);
    categories = {'RMSE (m)', 'RPE (m)', '漂移率 (%)'};
    values = [result.rmse, result.rpe * 100, result.drift_rate];
    bar(values, 'FaceColor', [0.8, 0.4, 0.4]);
    set(gca, 'XTickLabel', categories);
    ylabel('误差值', 'FontSize', 12, 'FontWeight', 'bold');
    title('定位精度', 'FontSize', 13, 'FontWeight', 'bold');
    grid on;
    for i = 1:length(values)
        text(i, values(i) + max(values)*0.05, sprintf('%.2f', values(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    end
    
    % 子图3：轨迹对比
    subplot(1, 3, 3);
    hold on;
    plot(result.gt_trajectory(:, 1), result.gt_trajectory(:, 2), ...
        'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    plot(result.aligned_trajectory(:, 1), result.aligned_trajectory(:, 2), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', '估计轨迹');
    xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title('轨迹对比', 'FontSize', 13, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    axis equal;
    
    sgtitle(sprintf('配置: %s', exp_name), 'FontSize', 15, 'FontWeight', 'bold');
    
    % 保存
    results_dir = '../../../data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/ablation_results';
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    saveas(fig, fullfile(results_dir, sprintf('%s_result.png', exp_name)));
    fprintf('结果图已保存: %s\n', fullfile(results_dir, sprintf('%s_result.png', exp_name)));
end
