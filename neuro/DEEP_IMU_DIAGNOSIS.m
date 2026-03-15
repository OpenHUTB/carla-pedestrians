%% 深度IMU诊断 - 确定根本原因
%  检查：1. 单位问题  2. 坐标系问题  3. 噪声问题  4. 时间同步问题

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║              深度IMU诊断 - 根本原因分析                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';

%% 1. 读取数据
fprintf('[1/6] 读取数据...\n');
gt_file = fullfile(data_path, 'ground_truth.txt');
imu_file = fullfile(data_path, 'aligned_imu.txt');

% 读取GT（跳过表头）
fid = fopen(gt_file, 'r');
header = fgetl(fid);
gt_data = textscan(fid, '%f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',');
fclose(fid);
gt_timestamp = gt_data{1};
gt_x = gt_data{2};
gt_y = gt_data{3};

% 读取IMU
imu_data = load(imu_file);
imu_timestamp = imu_data(:, 1);
imu_gyro_x = imu_data(:, 5);
imu_gyro_y = imu_data(:, 6);
imu_gyro_z = imu_data(:, 7);

fprintf('  GT数据: %d 帧\n', length(gt_x));
fprintf('  IMU数据: %d 帧\n', length(imu_timestamp));

%% 2. 从GT位置计算真实yaw角和yaw角速度
fprintf('\n[2/6] 从GT位置计算真实yaw角速度...\n');

% 从位置变化计算yaw角（运动方向）
gt_yaw = zeros(length(gt_x), 1);
for i = 2:length(gt_x)
    dx = gt_x(i) - gt_x(i-1);
    dy = gt_y(i) - gt_y(i-1);
    if sqrt(dx^2 + dy^2) > 0.01  % 只有移动距离>1cm才更新yaw
        gt_yaw(i) = atan2(dy, dx);
    else
        gt_yaw(i) = gt_yaw(i-1);  % 静止时保持上一帧的yaw
    end
end

% 计算yaw角速度
gt_yaw_rate = zeros(length(gt_yaw), 1);
for i = 2:length(gt_yaw)
    dt = gt_timestamp(i) - gt_timestamp(i-1);
    if dt > 0
        dyaw = gt_yaw(i) - gt_yaw(i-1);
        % 处理角度跳变
        if dyaw > pi, dyaw = dyaw - 2*pi; end
        if dyaw < -pi, dyaw = dyaw + 2*pi; end
        gt_yaw_rate(i) = dyaw / dt;
    end
end

fprintf('  GT yaw rate: 均值=%.4f, 标准差=%.4f, 最大值=%.4f rad/s\n', ...
    mean(abs(gt_yaw_rate)), std(gt_yaw_rate), max(abs(gt_yaw_rate)));

%% 3. 对齐时间戳
fprintf('\n[3/6] 对齐时间戳...\n');
t_start = max(gt_timestamp(1), imu_timestamp(1));
t_end = min(gt_timestamp(end), imu_timestamp(end));
idx_gt = find(gt_timestamp >= t_start & gt_timestamp <= t_end);
t_common = gt_timestamp(idx_gt);

% 插值IMU到GT时间戳
gyro_x_interp = interp1(imu_timestamp, imu_gyro_x, t_common, 'linear');
gyro_y_interp = interp1(imu_timestamp, imu_gyro_y, t_common, 'linear');
gyro_z_interp = interp1(imu_timestamp, imu_gyro_z, t_common, 'linear');
gt_yaw_rate_aligned = gt_yaw_rate(idx_gt);

% 只看运动段
gt_speed = sqrt(diff(gt_x).^2 + diff(gt_y).^2) ./ diff(gt_timestamp);
gt_speed = [0; gt_speed];
moving_idx = find(gt_speed(idx_gt) > 0.5);
fprintf('  运动段: %d / %d 帧 (%.1f%%)\n', length(moving_idx), length(t_common), ...
    length(moving_idx)/length(t_common)*100);

%% 4. 测试所有可能的配置
fprintf('\n[4/6] 测试所有可能的配置...\n');
fprintf('%-30s | %-12s | %-12s | %-12s\n', '配置', '相关系数', '幅度比', '判断');
fprintf('--------------------------------------------------------------------------------\n');

configs = {
    % [轴, 符号, 缩放因子, 描述]
    {gyro_z_interp, 1, 1, 'gyro_z (原始)'}
    {gyro_z_interp, -1, 1, '-gyro_z (反转)'}
    {gyro_z_interp, 1, 50, 'gyro_z × 50'}
    {gyro_z_interp, -1, 50, '-gyro_z × 50'}
    {gyro_z_interp, 1, 57.3, 'gyro_z × 57.3 (deg→rad)'}
    {gyro_z_interp, -1, 57.3, '-gyro_z × 57.3 (deg→rad)'}
    {gyro_x_interp, 1, 1, 'gyro_x (原始)'}
    {gyro_x_interp, -1, 1, '-gyro_x (反转)'}
    {gyro_y_interp, 1, 1, 'gyro_y (原始)'}
    {gyro_y_interp, -1, 1, '-gyro_y (反转)'}
};

best_corr = -inf;
best_config = '';
best_signal = [];

for i = 1:length(configs)
    cfg = configs{i};
    signal = cfg{1}(moving_idx) * cfg{2} * cfg{3};
    gt_sig = gt_yaw_rate_aligned(moving_idx);
    
    % 去除NaN
    valid = ~isnan(signal) & ~isnan(gt_sig) & ~isinf(signal) & ~isinf(gt_sig);
    
    if sum(valid) > 10
        corr_mat = corrcoef(signal(valid), gt_sig(valid));
        corr_val = corr_mat(1,2);
        amplitude_ratio = std(signal(valid)) / std(gt_sig(valid));
        
        % 判断
        if abs(corr_val) > 0.7 && amplitude_ratio > 0.8 && amplitude_ratio < 1.2
            judgment = '✓ 完美';
        elseif abs(corr_val) > 0.7
            judgment = '✓ 相关性好';
        elseif amplitude_ratio > 0.8 && amplitude_ratio < 1.2
            judgment = '⚠ 幅度匹配';
        else
            judgment = '✗ 不匹配';
        end
        
        fprintf('%-30s | %12.4f | %12.2f | %s\n', cfg{4}, corr_val, amplitude_ratio, judgment);
        
        if abs(corr_val) > abs(best_corr)
            best_corr = corr_val;
            best_config = cfg{4};
            best_signal = signal;
        end
    end
end

fprintf('--------------------------------------------------------------------------------\n');
fprintf('✓ 最佳配置: %s (相关系数=%.4f)\n', best_config, best_corr);

%% 5. 噪声分析
fprintf('\n[5/6] 噪声分析...\n');
imu_noise = std(gyro_z_interp(moving_idx));
gt_noise = std(gt_yaw_rate_aligned(moving_idx));
snr = mean(abs(gt_yaw_rate_aligned(moving_idx))) / std(gt_yaw_rate_aligned(moving_idx));

fprintf('  IMU噪声水平: %.6f rad/s\n', imu_noise);
fprintf('  GT信号标准差: %.4f rad/s\n', gt_noise);
fprintf('  信噪比: %.2f\n', snr);

if imu_noise / gt_noise < 0.1
    fprintf('  → IMU噪声相对很小，数据质量应该不错\n');
else
    fprintf('  → IMU噪声较大，可能影响精度\n');
end

%% 6. 根本原因诊断
fprintf('\n[6/6] 根本原因诊断...\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    诊断结论                                  ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

if abs(best_corr) > 0.7
    fprintf('║  ✓ IMU数据可用！                                            ║\n');
    fprintf('║                                                              ║\n');
    fprintf('║  最佳配置: %s\n', best_config);
    fprintf('║  相关系数: %.4f                                             ║\n', best_corr);
    fprintf('║                                                              ║\n');
    if contains(best_config, '× 50') || contains(best_config, '× 57.3')
        fprintf('║  ⚠️  问题: 幅度缩放错误                                     ║\n');
        fprintf('║  原因: CARLA IMU输出单位可能是deg/s但被当成rad/s处理       ║\n');
        fprintf('║  解决: 需要在数据读取时乘以缩放因子                        ║\n');
    end
    if contains(best_config, '-')
        fprintf('║  ⚠️  问题: 坐标系符号反转                                   ║\n');
        fprintf('║  原因: CARLA坐标系与NeuroSLAM坐标系Z轴方向相反             ║\n');
        fprintf('║  解决: 需要在融合时取反                                    ║\n');
    end
    if contains(best_config, 'gyro_x') || contains(best_config, 'gyro_y')
        fprintf('║  ⚠️  问题: 坐标轴混淆                                       ║\n');
        fprintf('║  原因: CARLA坐标系与预期不同                               ║\n');
        fprintf('║  解决: 使用正确的轴                                        ║\n');
    end
else
    fprintf('║  ✗ IMU数据不可用                                            ║\n');
    fprintf('║                                                              ║\n');
    fprintf('║  最佳相关系数: %.4f (< 0.7)                                 ║\n', best_corr);
    fprintf('║                                                              ║\n');
    fprintf('║  可能原因:                                                   ║\n');
    fprintf('║  1. 时间戳严重不同步                                        ║\n');
    fprintf('║  2. IMU传感器配置错误                                       ║\n');
    fprintf('║  3. 数据采集过程有问题                                      ║\n');
    fprintf('║  4. 坐标系完全不匹配（不只是简单的符号/缩放）              ║\n');
    fprintf('║                                                              ║\n');
    fprintf('║  建议:                                                       ║\n');
    fprintf('║  1. 检查CARLA IMU传感器配置                                 ║\n');
    fprintf('║  2. 重新采集数据                                            ║\n');
    fprintf('║  3. 使用其他数据集（KITTI, EuRoC）                          ║\n');
    fprintf('║  4. 暂时禁用IMU融合                                         ║\n');
end

fprintf('╚══════════════════════════════════════════════════════════════╝\n');

%% 7. 可视化
fprintf('\n生成可视化图表...\n');
figure('Position', [50, 50, 1600, 900]);

% 子图1: GT轨迹
subplot(2, 3, 1);
plot(gt_x, gt_y, 'k-', 'LineWidth', 1.5);
hold on;
plot(gt_x(1), gt_y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(gt_x(end), gt_y(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Ground Truth轨迹');
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;

% 子图2: GT yaw角速度
subplot(2, 3, 2);
plot(gt_timestamp, gt_yaw_rate * 180/pi, 'b-', 'LineWidth', 1);
title('GT Yaw角速度');
xlabel('时间 (s)'); ylabel('Yaw Rate (deg/s)');
grid on;

% 子图3: IMU三轴陀螺仪原始数据
subplot(2, 3, 3);
plot(imu_timestamp, imu_gyro_x * 180/pi, 'r-', 'DisplayName', 'Gyro X'); hold on;
plot(imu_timestamp, imu_gyro_y * 180/pi, 'g-', 'DisplayName', 'Gyro Y');
plot(imu_timestamp, imu_gyro_z * 180/pi, 'b-', 'DisplayName', 'Gyro Z');
title('IMU三轴陀螺仪（原始）');
xlabel('时间 (s)'); ylabel('角速度 (deg/s)');
legend; grid on;

% 子图4: 最佳配置对比
subplot(2, 3, 4);
plot(t_common(moving_idx), gt_yaw_rate_aligned(moving_idx) * 180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'GT'); hold on;
if ~isempty(best_signal)
    plot(t_common(moving_idx), best_signal * 180/pi, 'r--', 'LineWidth', 1, 'DisplayName', best_config);
end
title(sprintf('最佳配置对比 (corr=%.3f)', best_corr));
xlabel('时间 (s)'); ylabel('Yaw Rate (deg/s)');
legend; grid on;

% 子图5: 散点图
subplot(2, 3, 5);
if ~isempty(best_signal)
    scatter(gt_yaw_rate_aligned(moving_idx) * 180/pi, best_signal * 180/pi, 20, 'filled', 'MarkerFaceAlpha', 0.3);
    hold on;
    lim = max(abs([xlim, ylim]));
    plot([-lim, lim], [-lim, lim], 'k--', 'LineWidth', 1);
    title('散点图: 最佳配置');
    xlabel('GT Yaw Rate (deg/s)'); ylabel('IMU (deg/s)');
    axis equal; grid on;
end

% 子图6: 幅度对比
subplot(2, 3, 6);
bar([std(gt_yaw_rate_aligned(moving_idx)), std(gyro_z_interp(moving_idx)), std(gyro_z_interp(moving_idx))*50]);
set(gca, 'XTickLabel', {'GT', 'IMU原始', 'IMU×50'});
ylabel('标准差 (rad/s)');
title('幅度对比');
grid on;

sgtitle(sprintf('深度IMU诊断 | 最佳配置: %s (相关系数=%.4f)', best_config, best_corr), ...
    'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, fullfile(data_path, 'deep_imu_diagnosis.png'));
fprintf('✓ 图表已保存\n');
