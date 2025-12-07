function plot_imu_visual_comparison(fusion_data, odo_trajectory, exp_trajectory, gt_data, save_dir)
%PLOT_IMU_VISUAL_COMPARISON 对比IMU-视觉融合与纯视觉SLAM结果
%   绘制多个子图对比不同方法的轨迹和精度
%   
%   输入:
%       fusion_data - IMU-视觉融合数据
%       odo_trajectory - 里程计轨迹
%       exp_trajectory - 经验地图轨迹
%       gt_data - 真值数据(可选)
%       save_dir - 保存目录(可选，默认当前目录)
%   
%   NeuroSLAM System Copyright (C) 2018-2019
%   Visualization Module (2024)

    if nargin < 5
        save_dir = '.';
    end

    figure('Name', 'IMU-Visual SLAM vs Pure Visual SLAM', 'Position', [100, 100, 1600, 1000]);
    
    % 子图1: 3D轨迹对比
    subplot(2, 3, 1);
    hold on; grid on;
    
    % 绘制融合轨迹
    plot3(fusion_data.pos(:,1), fusion_data.pos(:,2), fusion_data.pos(:,3), ...
        'r-', 'LineWidth', 2, 'DisplayName', 'IMU-Visual Fusion');
    
    % 绘制纯IMU积分轨迹
    plot3(fusion_data.imu_pos(:,1), fusion_data.imu_pos(:,2), fusion_data.imu_pos(:,3), ...
        'b--', 'LineWidth', 1.5, 'DisplayName', 'Pure IMU Integration');
    
    % 绘制视觉里程计轨迹
    if ~isempty(odo_trajectory)
        plot3(odo_trajectory(:,1), odo_trajectory(:,2), odo_trajectory(:,3), ...
            'g:', 'LineWidth', 1.5, 'DisplayName', 'Visual Odometry');
    end
    
    % 绘制经验地图轨迹
    if ~isempty(exp_trajectory)
        plot3(exp_trajectory(:,1), exp_trajectory(:,2), exp_trajectory(:,3), ...
            'm-.', 'LineWidth', 1.5, 'DisplayName', 'Experience Map');
    end
    
    % 如果有真值,绘制真值轨迹
    if nargin >= 4 && ~isempty(gt_data)
        plot3(gt_data(:,2), gt_data(:,3), gt_data(:,4), ...
            'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Trajectory Comparison');
    legend('Location', 'best');
    view(45, 30);
    axis equal;
    
    % 子图2: 2D俯视图轨迹
    subplot(2, 3, 2);
    hold on; grid on;
    plot(fusion_data.pos(:,1), fusion_data.pos(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'IMU-Visual Fusion');
    plot(fusion_data.imu_pos(:,1), fusion_data.imu_pos(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Pure IMU');
    if ~isempty(odo_trajectory)
        plot(odo_trajectory(:,1), odo_trajectory(:,2), 'g:', 'LineWidth', 1.5, 'DisplayName', 'Visual Odo');
    end
    if nargin >= 4 && ~isempty(gt_data)
        plot(gt_data(:,2), gt_data(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    end
    xlabel('X (m)'); ylabel('Y (m)');
    title('2D Trajectory (Top View)');
    legend('Location', 'best');
    axis equal;
    
    % 子图3: 位置不确定性
    subplot(2, 3, 3);
    hold on; grid on;
    time_vec = (1:fusion_data.count)';
    plot(time_vec, fusion_data.uncertainty(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X Uncertainty');
    plot(time_vec, fusion_data.uncertainty(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y Uncertainty');
    plot(time_vec, fusion_data.uncertainty(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z Uncertainty');
    xlabel('Frame'); ylabel('Uncertainty (m)');
    title('Position Uncertainty over Time');
    legend('Location', 'best');
    
    % 子图4: 姿态角度
    subplot(2, 3, 4);
    hold on; grid on;
    plot(time_vec, fusion_data.att(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Roll');
    plot(time_vec, fusion_data.att(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Pitch');
    plot(time_vec, fusion_data.att(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yaw');
    xlabel('Frame'); ylabel('Angle (degrees)');
    title('Attitude Angles');
    legend('Location', 'best');
    grid on;
    
    % 子图5: 速度分量
    subplot(2, 3, 5);
    hold on; grid on;
    plot(time_vec, fusion_data.vel(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
    plot(time_vec, fusion_data.vel(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
    plot(time_vec, fusion_data.vel(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vz');
    xlabel('Frame'); ylabel('Velocity (m/s)');
    title('Velocity Components');
    legend('Location', 'best');
    grid on;
    
    % 子图6: 精度统计
    subplot(2, 3, 6);
    axis off;
    
    % 计算统计信息
    trajectory_length = sum(sqrt(sum(diff(fusion_data.pos).^2, 2)));
    avg_uncertainty = mean(sqrt(sum(fusion_data.uncertainty.^2, 2)));
    max_uncertainty = max(sqrt(sum(fusion_data.uncertainty.^2, 2)));
    
    % 计算IMU漂移
    imu_drift = norm(fusion_data.imu_pos(end,:) - fusion_data.pos(end,:));
    
    % 如果有真值,计算误差
    if nargin >= 4 && ~isempty(gt_data) && size(gt_data, 1) == fusion_data.count
        pos_error = sqrt(sum((fusion_data.pos - gt_data(:, 2:4)).^2, 2));
        rmse = sqrt(mean(pos_error.^2));
        max_error = max(pos_error);
        final_error = pos_error(end);
        
        stats_text = sprintf(['统计信息:\n\n' ...
            '总轨迹长度: %.2f m\n' ...
            '平均不确定性: %.3f m\n' ...
            '最大不确定性: %.3f m\n' ...
            'IMU漂移距离: %.2f m\n\n' ...
            '相对真值误差:\n' ...
            'RMSE: %.3f m\n' ...
            '最大误差: %.3f m\n' ...
            '终点误差: %.3f m\n' ...
            '相对精度: %.2f%%'], ...
            trajectory_length, avg_uncertainty, max_uncertainty, imu_drift, ...
            rmse, max_error, final_error, (rmse/trajectory_length)*100);
    else
        stats_text = sprintf(['统计信息:\n\n' ...
            '总轨迹长度: %.2f m\n' ...
            '平均不确定性: %.3f m\n' ...
            '最大不确定性: %.3f m\n' ...
            'IMU漂移距离: %.2f m\n\n' ...
            '(无真值数据)'], ...
            trajectory_length, avg_uncertainty, max_uncertainty, imu_drift);
    end
    
    text(0.1, 0.5, stats_text, 'FontSize', 10, 'VerticalAlignment', 'middle', ...
        'FontName', 'Courier', 'FontWeight', 'bold');
    title('精度统计', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 保存图像
    save_path = fullfile(save_dir, 'imu_visual_slam_comparison.png');
    saveas(gcf, save_path);
    fprintf('对比图已保存: %s\n', save_path);
end
