%% 分析IMU数据与Ground Truth的相关性
%  通过对比GT的运动方向和IMU陀螺仪读数，确定坐标系是否匹配

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║        IMU数据与Ground Truth相关性分析                       ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. 读取数据
data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';

% 读取Ground Truth
fprintf('[1/5] 读取Ground Truth...\n');
gt_file = fullfile(data_path, 'ground_truth.txt');

% 尝试直接加载（无表头）
try
    gt_data = load(gt_file);
    gt_timestamp = gt_data(:, 1);
    gt_x = gt_data(:, 2);
    gt_y = gt_data(:, 3);
catch
    % 有表头，使用textscan跳过
    fid = fopen(gt_file, 'r');
    header = fgetl(fid);  % 跳过表头
    gt_data = textscan(fid, '%f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',');
    fclose(fid);
    
    gt_timestamp = gt_data{1};
    gt_x = gt_data{2};
    gt_y = gt_data{3};
end

fprintf('  GT数据: %d 帧\n', length(gt_x));

% 读取IMU数据
fprintf('[2/5] 读取IMU数据...\n');
imu_file = fullfile(data_path, 'aligned_imu.txt');
imu_data = load(imu_file);
imu_timestamp = imu_data(:, 1);
imu_accel_x = imu_data(:, 2);
imu_accel_y = imu_data(:, 3);
imu_accel_z = imu_data(:, 4);
imu_gyro_x = imu_data(:, 5);
imu_gyro_y = imu_data(:, 6);
imu_gyro_z = imu_data(:, 7);
fprintf('  IMU数据: %d 帧\n', length(imu_timestamp));

%% 2. 从GT计算真实的yaw角速度
fprintf('[3/5] 从GT计算真实运动...\n');

% 计算GT的yaw角（从位置变化推算）
gt_yaw = zeros(length(gt_x), 1);
for i = 2:length(gt_x)
    dx = gt_x(i) - gt_x(i-1);
    dy = gt_y(i) - gt_y(i-1);
    gt_yaw(i) = atan2(dy, dx);  % 运动方向的yaw角
end

% 计算GT的yaw角速度（rad/s）
gt_yaw_rate = zeros(length(gt_x), 1);
for i = 2:length(gt_x)
    dt = gt_timestamp(i) - gt_timestamp(i-1);
    if dt > 0
        dyaw = gt_yaw(i) - gt_yaw(i-1);
        % 处理角度跳变（-π到π）
        if dyaw > pi
            dyaw = dyaw - 2*pi;
        elseif dyaw < -pi
            dyaw = dyaw + 2*pi;
        end
        gt_yaw_rate(i) = dyaw / dt;
    end
end

% 计算GT的速度（m/s）
gt_speed = zeros(length(gt_x), 1);
for i = 2:length(gt_x)
    dx = gt_x(i) - gt_x(i-1);
    dy = gt_y(i) - gt_y(i-1);
    dt = gt_timestamp(i) - gt_timestamp(i-1);
    if dt > 0
        gt_speed(i) = sqrt(dx^2 + dy^2) / dt;
    end
end

fprintf('  GT yaw角速度范围: [%.4f, %.4f] rad/s\n', min(gt_yaw_rate), max(gt_yaw_rate));
fprintf('  GT速度范围: [%.2f, %.2f] m/s\n', min(gt_speed), max(gt_speed));

%% 3. 对齐时间戳并计算相关性
fprintf('[4/5] 计算IMU与GT的相关性...\n');

% 找到重叠的时间范围
t_start = max(gt_timestamp(1), imu_timestamp(1));
t_end = min(gt_timestamp(end), imu_timestamp(end));

% 插值到相同的时间点（使用GT的时间戳）
idx_gt = find(gt_timestamp >= t_start & gt_timestamp <= t_end);
t_common = gt_timestamp(idx_gt);

% 插值IMU数据到GT时间戳
gyro_z_interp = interp1(imu_timestamp, imu_gyro_z, t_common, 'linear');
gyro_x_interp = interp1(imu_timestamp, imu_gyro_x, t_common, 'linear');
gyro_y_interp = interp1(imu_timestamp, imu_gyro_y, t_common, 'linear');

% 提取对应的GT数据
gt_yaw_rate_aligned = gt_yaw_rate(idx_gt);
gt_speed_aligned = gt_speed(idx_gt);

% 只分析运动段（速度 > 0.5 m/s）
moving_idx = find(gt_speed_aligned > 0.5);
fprintf('  运动段帧数: %d / %d (%.1f%%)\n', length(moving_idx), length(t_common), ...
    length(moving_idx)/length(t_common)*100);

if length(moving_idx) < 100
    warning('运动段数据太少，结果可能不准确');
end

%% 4. 计算相关系数
fprintf('\n========== 相关性分析结果 ==========\n\n');

% 测试不同的符号组合
configs = struct('sign', {}, 'name', {});
configs(1).sign = 1;
configs(1).name = 'gyro_z (原始)';
configs(2).sign = -1;
configs(2).name = '-gyro_z (反转)';

best_corr = -inf;
best_config = '';

for i = 1:length(configs)
    sign_factor = configs(i).sign;
    config_name = configs(i).name;
    
    % 计算相关系数
    imu_signal = sign_factor * gyro_z_interp(moving_idx);
    gt_signal = gt_yaw_rate_aligned(moving_idx);
    
    % 去除NaN和Inf
    valid_idx = ~isnan(imu_signal) & ~isnan(gt_signal) & ...
                ~isinf(imu_signal) & ~isinf(gt_signal);
    
    if sum(valid_idx) > 10
        corr_coef = corrcoef(imu_signal(valid_idx), gt_signal(valid_idx));
        corr_value = corr_coef(1, 2);
        
        fprintf('配置: %s\n', config_name);
        fprintf('  相关系数: %.4f\n', corr_value);
        fprintf('  IMU均值: %.6f rad/s\n', mean(imu_signal(valid_idx)));
        fprintf('  GT均值:  %.6f rad/s\n', mean(gt_signal(valid_idx)));
        fprintf('  IMU标准差: %.6f rad/s\n', std(imu_signal(valid_idx)));
        fprintf('  GT标准差:  %.6f rad/s\n', std(gt_signal(valid_idx)));
        
        if corr_value > best_corr
            best_corr = corr_value;
            best_config = config_name;
        end
        fprintf('\n');
    end
end

fprintf('========================================\n');
fprintf('✓ 最佳配置: %s (相关系数=%.4f)\n', best_config, best_corr);
fprintf('========================================\n\n');

%% 5. 可视化对比
fprintf('[5/5] 生成可视化对比图...\n');

figure('Name', 'IMU与GT相关性分析', 'Position', [50, 50, 1600, 1000]);

% 子图1: GT轨迹（标注转弯点）
subplot(3, 3, 1);
plot(gt_x, gt_y, 'k-', 'LineWidth', 1.5); hold on;
% 标注大转弯点（yaw rate > 0.1 rad/s）
turn_idx = find(abs(gt_yaw_rate) > 0.1);
if ~isempty(turn_idx)
    plot(gt_x(turn_idx), gt_y(turn_idx), 'r.', 'MarkerSize', 8);
end
plot(gt_x(1), gt_y(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(gt_x(end), gt_y(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
title('GT轨迹 (红点=转弯)', 'FontSize', 11);
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;

% 子图2: GT yaw角速度
subplot(3, 3, 2);
plot(gt_timestamp, gt_yaw_rate * 180/pi, 'b-', 'LineWidth', 1);
title('GT Yaw角速度', 'FontSize', 11);
xlabel('时间 (s)'); ylabel('Yaw Rate (deg/s)');
grid on;

% 子图3: GT速度
subplot(3, 3, 3);
plot(gt_timestamp, gt_speed, 'g-', 'LineWidth', 1);
title('GT速度', 'FontSize', 11);
xlabel('时间 (s)'); ylabel('速度 (m/s)');
grid on;

% 子图4: IMU gyro_z原始信号
subplot(3, 3, 4);
plot(imu_timestamp, imu_gyro_z * 180/pi, 'r-', 'LineWidth', 1);
title('IMU Gyro Z (原始)', 'FontSize', 11);
xlabel('时间 (s)'); ylabel('Gyro Z (deg/s)');
grid on;

% 子图5: IMU gyro_z反转信号
subplot(3, 3, 5);
plot(imu_timestamp, -imu_gyro_z * 180/pi, 'm-', 'LineWidth', 1);
title('IMU Gyro Z (反转)', 'FontSize', 11);
xlabel('时间 (s)'); ylabel('-Gyro Z (deg/s)');
grid on;

% 子图6: 对齐后的对比（原始）
subplot(3, 3, 6);
plot(t_common(moving_idx), gt_yaw_rate_aligned(moving_idx) * 180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'GT'); hold on;
plot(t_common(moving_idx), gyro_z_interp(moving_idx) * 180/pi, 'r--', 'LineWidth', 1, 'DisplayName', 'IMU (原始)');
% 计算原始相关系数用于显示
try
    imu_orig = gyro_z_interp(moving_idx);
    gt_sig = gt_yaw_rate_aligned(moving_idx);
    valid = ~isnan(imu_orig) & ~isnan(gt_sig) & ~isinf(imu_orig) & ~isinf(gt_sig);
    if sum(valid) > 10
        corr_orig = corrcoef(imu_orig(valid), gt_sig(valid));
        corr_orig_val = corr_orig(1,2);
    else
        corr_orig_val = 0;
    end
catch
    corr_orig_val = 0;
end
title(sprintf('对比: 原始 (corr=%.3f)', corr_orig_val), 'FontSize', 11);
xlabel('时间 (s)'); ylabel('Yaw Rate (deg/s)');
legend('Location', 'best'); grid on;

% 子图7: 对齐后的对比（反转）
subplot(3, 3, 7);
plot(t_common(moving_idx), gt_yaw_rate_aligned(moving_idx) * 180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'GT'); hold on;
plot(t_common(moving_idx), -gyro_z_interp(moving_idx) * 180/pi, 'm--', 'LineWidth', 1, 'DisplayName', 'IMU (反转)');
title(sprintf('对比: 反转 (corr=%.3f)', best_corr), 'FontSize', 11);
xlabel('时间 (s)'); ylabel('Yaw Rate (deg/s)');
legend('Location', 'best'); grid on;

% 子图8: 散点图（原始）
subplot(3, 3, 8);
scatter(gt_yaw_rate_aligned(moving_idx) * 180/pi, gyro_z_interp(moving_idx) * 180/pi, 20, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
% 添加理想线（y=x）
lim = max(abs([xlim, ylim]));
plot([-lim, lim], [-lim, lim], 'k--', 'LineWidth', 1);
title('散点图: 原始', 'FontSize', 11);
xlabel('GT Yaw Rate (deg/s)'); ylabel('IMU Gyro Z (deg/s)');
axis equal; grid on;

% 子图9: 散点图（反转）
subplot(3, 3, 9);
scatter(gt_yaw_rate_aligned(moving_idx) * 180/pi, -gyro_z_interp(moving_idx) * 180/pi, 20, 'm', 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
% 添加理想线（y=x）
lim = max(abs([xlim, ylim]));
plot([-lim, lim], [-lim, lim], 'k--', 'LineWidth', 1);
title('散点图: 反转', 'FontSize', 11);
xlabel('GT Yaw Rate (deg/s)'); ylabel('-IMU Gyro Z (deg/s)');
axis equal; grid on;

sgtitle(sprintf('IMU-GT相关性分析 | 最佳配置: %s (相关系数=%.4f)', best_config, best_corr), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 6. 生成诊断报告
fprintf('\n========== 诊断报告 ==========\n\n');

if best_corr > 0.7
    fprintf('✓ 相关性良好 (%.4f > 0.7)\n', best_corr);
    if contains(best_config, '反转')
        fprintf('  → 建议: 设置 IMU_YAW_SIGN = -1\n');
    else
        fprintf('  → 建议: 保持 IMU_YAW_SIGN = 1\n');
    end
elseif best_corr > 0.3
    fprintf('⚠️  相关性中等 (%.4f)\n', best_corr);
    fprintf('  → 可能原因:\n');
    fprintf('     1. IMU数据噪声较大\n');
    fprintf('     2. 时间戳对齐不完美\n');
    fprintf('     3. IMU需要标定\n');
    if contains(best_config, '反转')
        fprintf('  → 建议: 设置 IMU_YAW_SIGN = -1，但降低权重到0.1\n');
    else
        fprintf('  → 建议: 保持 IMU_YAW_SIGN = 1，但降低权重到0.1\n');
    end
else
    fprintf('✗ 相关性很差 (%.4f < 0.3)\n', best_corr);
    fprintf('  → 严重问题:\n');
    fprintf('     1. IMU数据可能完全不可用\n');
    fprintf('     2. 坐标系完全不匹配（不只是符号问题）\n');
    fprintf('     3. 时间戳严重错位\n');
    fprintf('  → 建议: 禁用IMU融合 (设置 alpha_yaw = 0)\n');
end

% 检查IMU数据质量
imu_snr = mean(abs(gyro_z_interp(moving_idx))) / std(gyro_z_interp(moving_idx));
fprintf('\n信号质量分析:\n');
fprintf('  IMU信噪比: %.2f\n', imu_snr);
if imu_snr < 1
    fprintf('  ⚠️  信噪比过低，IMU数据噪声太大\n');
end

% 检查幅度匹配
gt_amplitude = std(gt_yaw_rate_aligned(moving_idx));
imu_amplitude = std(gyro_z_interp(moving_idx));
amplitude_ratio = imu_amplitude / gt_amplitude;
fprintf('  幅度比 (IMU/GT): %.2f\n', amplitude_ratio);
if amplitude_ratio < 0.5
    fprintf('  ⚠️  IMU幅度过小，可能需要增益校正\n');
elseif amplitude_ratio > 2
    fprintf('  ⚠️  IMU幅度过大，可能需要衰减\n');
end

fprintf('\n==========================================\n');

%% 7. 保存结果
result_file = fullfile(data_path, 'imu_gt_correlation_analysis.mat');
save(result_file, 'best_config', 'best_corr', 'gt_yaw_rate_aligned', ...
    'gyro_z_interp', 'moving_idx', 't_common');
fprintf('\n✓ 结果已保存到: %s\n', result_file);

saveas(gcf, fullfile(data_path, 'imu_gt_correlation_plot.png'));
fprintf('✓ 图表已保存\n');
