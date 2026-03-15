%% 诊断test_imu_visual_fusion_slam2.m轨迹对齐问题
% 此脚本帮助诊断为什么轨迹看起来不像

fprintf('========== 轨迹对齐问题诊断 ==========\n\n');

% 1. 检查comparison_results.mat是否存在
result_file = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\comparison_results\comparison_results.mat';

if ~exist(result_file, 'file')
    error('未找到结果文件: %s\n请先运行test_imu_visual_fusion_slam2.m', result_file);
end

% 2. 加载数据
fprintf('[1/5] 加载对比结果...\n');
load(result_file);

% 3. 检查轨迹尺度
fprintf('\n[2/5] 检查轨迹尺度...\n');
fprintf('----------------------------------------\n');

if exist('gt_data', 'var') && isstruct(gt_data)
    gt_len = sum(sqrt(sum(diff(gt_data.pos).^2, 2)));
    fprintf('Ground Truth总长度: %.2f m\n', gt_len);
end

bl_odo_len = sum(sqrt(sum(diff(baseline_odo_traj).^2, 2)));
bl_exp_len = sum(sqrt(sum(diff(baseline_exp_traj).^2, 2)));
fprintf('Baseline里程计总长度: %.2f m\n', bl_odo_len);
fprintf('Baseline经验地图总长度: %.2f m\n', bl_exp_len);

ours_odo_len = sum(sqrt(sum(diff(ours_odo_traj).^2, 2)));
ours_exp_len = sum(sqrt(sum(diff(ours_exp_traj).^2, 2)));
fprintf('Ours里程计总长度: %.2f m\n', ours_odo_len);
fprintf('Ours经验地图总长度: %.2f m\n', ours_exp_len);

if exist('gt_len', 'var')
    fprintf('\n尺度比例:\n');
    fprintf('  Baseline/GT: %.2f%%\n', bl_exp_len/gt_len*100);
    fprintf('  Ours/GT: %.2f%%\n', ours_exp_len/gt_len*100);
    
    if ours_exp_len/gt_len > 1.3 || ours_exp_len/gt_len < 0.7
        fprintf('⚠️  警告: Ours轨迹尺度异常 (%.1f%% of GT)\n', ours_exp_len/gt_len*100);
        fprintf('   建议: 检查ODO_TRANS_V_SCALE参数\n');
    end
end

% 4. 检查轨迹形状相似度
fprintf('\n[3/5] 检查轨迹形状相似度...\n');
fprintf('----------------------------------------\n');

% 归一化轨迹（去除尺度影响）
bl_exp_norm = baseline_exp_traj - mean(baseline_exp_traj, 1);
bl_exp_norm = bl_exp_norm / norm(bl_exp_norm(:));

ours_exp_norm = ours_exp_traj - mean(ours_exp_traj, 1);
ours_exp_norm = ours_exp_norm / norm(ours_exp_norm(:));

if exist('gt_data', 'var')
    gt_norm = gt_data.pos - mean(gt_data.pos, 1);
    gt_norm = gt_norm / norm(gt_norm(:));
    
    % 计算形状相似度（余弦相似度）
    min_len = min([size(gt_norm,1), size(bl_exp_norm,1), size(ours_exp_norm,1)]);
    
    bl_similarity = sum(sum(gt_norm(1:min_len,:) .* bl_exp_norm(1:min_len,:)));
    ours_similarity = sum(sum(gt_norm(1:min_len,:) .* ours_exp_norm(1:min_len,:)));
    
    fprintf('形状相似度 (归一化后):\n');
    fprintf('  Baseline vs GT: %.4f\n', bl_similarity);
    fprintf('  Ours vs GT: %.4f\n', ours_similarity);
    
    if ours_similarity < 0.5
        fprintf('⚠️  警告: Ours轨迹形状与GT差异很大\n');
        fprintf('   可能原因:\n');
        fprintf('   1. 轴匹配错误（XY轴交换或翻转）\n');
        fprintf('   2. 对齐算法失败\n');
        fprintf('   3. 经验地图回环闭合异常\n');
    end
end

% 5. 检查对齐后的轨迹
fprintf('\n[4/5] 检查对齐后的轨迹...\n');
fprintf('----------------------------------------\n');

if exist('baseline_exp_aligned', 'var') && exist('ours_exp_aligned', 'var')
    fprintf('对齐后轨迹范围:\n');
    fprintf('  Baseline X: [%.2f, %.2f]\n', min(baseline_exp_aligned(:,1)), max(baseline_exp_aligned(:,1)));
    fprintf('  Baseline Y: [%.2f, %.2f]\n', min(baseline_exp_aligned(:,2)), max(baseline_exp_aligned(:,2)));
    fprintf('  Ours X: [%.2f, %.2f]\n', min(ours_exp_aligned(:,1)), max(ours_exp_aligned(:,1)));
    fprintf('  Ours Y: [%.2f, %.2f]\n', min(ours_exp_aligned(:,2)), max(ours_exp_aligned(:,2)));
    
    if exist('gt_pos_aligned', 'var')
        fprintf('  GT X: [%.2f, %.2f]\n', min(gt_pos_aligned(:,1)), max(gt_pos_aligned(:,1)));
        fprintf('  GT Y: [%.2f, %.2f]\n', min(gt_pos_aligned(:,2)), max(gt_pos_aligned(:,2)));
    end
end

% 6. 可视化诊断
fprintf('\n[5/5] 生成诊断可视化...\n');
fprintf('----------------------------------------\n');

figure('Name', '轨迹对齐诊断', 'Position', [100, 100, 1600, 800]);

% 子图1: 原始轨迹（未对齐）
subplot(2, 3, 1);
hold on;
if exist('gt_data', 'var')
    plot(gt_data.pos(:,1), gt_data.pos(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
end
plot(baseline_exp_traj(:,1), baseline_exp_traj(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
plot(ours_exp_traj(:,1), ours_exp_traj(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
hold off;
xlabel('X'); ylabel('Y');
title('原始轨迹（未对齐）');
legend('Location', 'best');
grid on; axis equal;

% 子图2: 对齐后轨迹
subplot(2, 3, 2);
hold on;
if exist('gt_pos_aligned', 'var')
    plot(gt_pos_aligned(:,1), gt_pos_aligned(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
end
if exist('baseline_exp_aligned', 'var')
    plot(baseline_exp_aligned(:,1), baseline_exp_aligned(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
end
if exist('ours_exp_aligned', 'var')
    plot(ours_exp_aligned(:,1), ours_exp_aligned(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
end
hold off;
xlabel('X'); ylabel('Y');
title('对齐后轨迹');
legend('Location', 'best');
grid on; axis equal;

% 子图3: 归一化形状对比
subplot(2, 3, 3);
hold on;
if exist('gt_norm', 'var')
    plot(gt_norm(:,1), gt_norm(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'GT');
end
plot(bl_exp_norm(:,1), bl_exp_norm(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
plot(ours_exp_norm(:,1), ours_exp_norm(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours');
hold off;
xlabel('X (normalized)'); ylabel('Y (normalized)');
title('归一化形状对比');
legend('Location', 'best');
grid on; axis equal;

% 子图4: Baseline里程计 vs 经验地图
subplot(2, 3, 4);
hold on;
plot(baseline_odo_traj(:,1), baseline_odo_traj(:,2), 'c--', 'LineWidth', 1, 'DisplayName', 'Baseline Odo');
plot(baseline_exp_traj(:,1), baseline_exp_traj(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Baseline Exp');
hold off;
xlabel('X'); ylabel('Y');
title('Baseline: 里程计 vs 经验地图');
legend('Location', 'best');
grid on; axis equal;

% 子图5: Ours里程计 vs 经验地图
subplot(2, 3, 5);
hold on;
plot(ours_odo_traj(:,1), ours_odo_traj(:,2), 'm--', 'LineWidth', 1, 'DisplayName', 'Ours Odo');
plot(ours_exp_traj(:,1), ours_exp_traj(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ours Exp');
hold off;
xlabel('X'); ylabel('Y');
title('Ours: 里程计 vs 经验地图');
legend('Location', 'best');
grid on; axis equal;

% 子图6: 轨迹长度对比
subplot(2, 3, 6);
lengths = [];
labels = {};
if exist('gt_len', 'var')
    lengths(end+1) = gt_len;
    labels{end+1} = 'GT';
end
lengths(end+1) = bl_exp_len;
labels{end+1} = 'Baseline';
lengths(end+1) = ours_exp_len;
labels{end+1} = 'Ours';

bar(lengths);
set(gca, 'XTickLabel', labels);
ylabel('Total Length (m)');
title('轨迹总长度对比');
grid on;

% 添加数值标签
for i = 1:length(lengths)
    text(i, lengths(i), sprintf('%.1fm', lengths(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

sgtitle('轨迹对齐诊断报告', 'FontSize', 16, 'FontWeight', 'bold');

% 保存诊断图
saveas(gcf, fullfile(fileparts(result_file), 'alignment_diagnosis.png'));
fprintf('✓ 诊断图已保存: alignment_diagnosis.png\n');

% 7. 输出建议
fprintf('\n========== 诊断建议 ==========\n');
if exist('ours_exp_len', 'var') && exist('gt_len', 'var')
    ratio = ours_exp_len / gt_len;
    if ratio > 1.3
        fprintf('❌ Ours轨迹过大 (%.1f%% of GT)\n', ratio*100);
        fprintf('   解决方案:\n');
        fprintf('   1. 增大ODO_TRANS_V_SCALE参数 (当前建议: 35 -> 40)\n');
        fprintf('   2. 检查bio_trans_gain是否过大\n');
    elseif ratio < 0.7
        fprintf('❌ Ours轨迹过小 (%.1f%% of GT)\n', ratio*100);
        fprintf('   解决方案:\n');
        fprintf('   1. 减小ODO_TRANS_V_SCALE参数 (当前建议: 35 -> 24)\n');
        fprintf('   2. 检查IMU融合权重\n');
    else
        fprintf('✓ Ours轨迹尺度正常 (%.1f%% of GT)\n', ratio*100);
    end
end

if exist('ours_similarity', 'var') && ours_similarity < 0.5
    fprintf('\n❌ Ours轨迹形状异常 (相似度: %.3f)\n', ours_similarity);
    fprintf('   可能原因:\n');
    fprintf('   1. 轴匹配错误 - 检查best_axis_match_xy函数\n');
    fprintf('   2. 对齐算法失败 - 尝试使用更多采样点\n');
    fprintf('   3. 经验地图回环异常 - 检查EXP_LOOPS和EXP_CORRECTION参数\n');
    fprintf('\n   建议操作:\n');
    fprintf('   1. 在test_imu_visual_fusion_slam2.m中添加调试输出\n');
    fprintf('   2. 检查Baseline和Ours是否使用了相同的轴匹配结果\n');
    fprintf('   3. 尝试手动指定轴匹配参数\n');
end

fprintf('\n========== 诊断完成 ==========\n');
