function plot_imu_visual_comparison_with_gt(fusion_data, odo_traj, exp_traj, gt_data, save_path)
% PLOT_IMU_VISUAL_COMPARISON_WITH_GT 绘制IMU-视觉融合SLAM对比图（含Ground Truth）
%
% 参数:
%   fusion_data - 融合位姿数据结构体 (包含pos, att)
%   odo_traj - 里程计轨迹 [N×3]
%   exp_traj - 经验地图轨迹 [N×3]
%   gt_data - Ground Truth数据 [N×3] 或 [N×1]结构体
%   save_path - 保存路径

    % 检查gt_data格式并转换
    if isstruct(gt_data) && isfield(gt_data, 'pos')
        gt_positions = gt_data.pos;  % 如果是结构体
    elseif size(gt_data, 2) >= 3
        gt_positions = gt_data;  % 如果已经是矩阵
    else
        error('gt_data格式不正确');
    end
    
    % 创建图形 - 专业背景
    fig = figure('Position', [50, 50, 1400, 900]);
    set(fig, 'Color', [0.96 0.96 0.98]);  % 极淡灰蓝背景
    
    % 绘制XY平面轨迹 - 使用现代配色方案
    subplot(2, 2, 1);
    hold on;
    
    % 专业网格样式
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    % 高对比度专业配色
    plot(gt_positions(:,1), gt_positions(:,2), '-', 'Color', [0.2 0.2 0.2], ...
        'LineWidth', 3.0, 'DisplayName', 'Ground Truth');  % 深灰色
    
    plot(exp_traj(:,1), exp_traj(:,2), '-', 'Color', [0.84 0.15 0.16], ...
        'LineWidth', 2.5, 'DisplayName', 'Experience Map');  % 深红色
    
    plot(fusion_data.pos(:,1), fusion_data.pos(:,2), '--', 'Color', [0.12 0.47 0.71], ...
        'LineWidth', 2.5, 'DisplayName', 'IMU-Visual Fusion');  % 深蓝色
    
    plot(odo_traj(:,1), odo_traj(:,2), '-.', 'Color', [0.17 0.63 0.17], ...
        'LineWidth', 2.2, 'DisplayName', 'Visual Odometry');  % 深绿色
    
    xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('Top View (XY Plane)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    leg = legend('Location', 'northeast', 'FontSize', 11);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    axis equal;
    set(gca, 'FontSize', 11, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 绘制XZ平面轨迹 - 侧视图
    subplot(2, 2, 2);
    hold on;
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    plot(gt_positions(:,1), gt_positions(:,3), '-', 'Color', [0.2 0.2 0.2], ...
        'LineWidth', 3.0, 'DisplayName', 'Ground Truth');
    plot(exp_traj(:,1), exp_traj(:,3), '-', 'Color', [0.84 0.15 0.16], ...
        'LineWidth', 2.5, 'DisplayName', 'Experience Map');
    plot(fusion_data.pos(:,1), fusion_data.pos(:,3), '--', 'Color', [0.12 0.47 0.71], ...
        'LineWidth', 2.5, 'DisplayName', 'IMU-Visual Fusion');
    plot(odo_traj(:,1), odo_traj(:,3), '-.', 'Color', [0.17 0.63 0.17], ...
        'LineWidth', 2.2, 'DisplayName', 'Visual Odometry');
    
    xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Z Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('Side View (XZ Plane)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    leg = legend('Location', 'northwest', 'FontSize', 11);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    set(gca, 'FontSize', 11, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 绘制YZ平面轨迹 - 前视图
    subplot(2, 2, 3);
    hold on;
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    plot(gt_positions(:,2), gt_positions(:,3), '-', 'Color', [0.2 0.2 0.2], ...
        'LineWidth', 3.0, 'DisplayName', 'Ground Truth');
    plot(exp_traj(:,2), exp_traj(:,3), '-', 'Color', [0.84 0.15 0.16], ...
        'LineWidth', 2.5, 'DisplayName', 'Experience Map');
    plot(fusion_data.pos(:,2), fusion_data.pos(:,3), '--', 'Color', [0.12 0.47 0.71], ...
        'LineWidth', 2.5, 'DisplayName', 'IMU-Visual Fusion');
    plot(odo_traj(:,2), odo_traj(:,3), '-.', 'Color', [0.17 0.63 0.17], ...
        'LineWidth', 2.2, 'DisplayName', 'Visual Odometry');
    
    xlabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Z Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('Front View (YZ Plane)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    leg = legend('Location', 'southwest', 'FontSize', 11);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    set(gca, 'FontSize', 11, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 绘制3D轨迹 - 立体视图
    subplot(2, 2, 4);
    hold on;
    
    grid on;
    set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.20, 'GridColor', [0.75 0.75 0.75]);
    set(gca, 'Box', 'on', 'LineWidth', 1.2);
    set(gca, 'Color', 'w');
    
    plot3(gt_positions(:,1), gt_positions(:,2), gt_positions(:,3), '-', ...
        'Color', [0.2 0.2 0.2], 'LineWidth', 3.0, 'DisplayName', 'Ground Truth');
    plot3(exp_traj(:,1), exp_traj(:,2), exp_traj(:,3), '-', ...
        'Color', [0.84 0.15 0.16], 'LineWidth', 2.5, 'DisplayName', 'Experience Map');
    plot3(fusion_data.pos(:,1), fusion_data.pos(:,2), fusion_data.pos(:,3), '--', ...
        'Color', [0.12 0.47 0.71], 'LineWidth', 2.5, 'DisplayName', 'IMU-Visual Fusion');
    plot3(odo_traj(:,1), odo_traj(:,2), odo_traj(:,3), '-.', ...
        'Color', [0.17 0.63 0.17], 'LineWidth', 2.2, 'DisplayName', 'Visual Odometry');
    
    xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    zlabel('Z Position (m)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0 0]);
    title('3D Trajectory View', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0 0]);
    
    leg = legend('Location', 'northeast', 'FontSize', 11);
    set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    
    view(135, 25);
    set(gca, 'FontSize', 11, 'FontName', 'Arial');
    ax = gca;
    ax.XAxis.Color = [0.2 0.2 0.2];
    ax.YAxis.Color = [0.2 0.2 0.2];
    ax.ZAxis.Color = [0.2 0.2 0.2];
    ax.LineWidth = 1.2;
    
    % 保存图形
    saveas(gcf, fullfile(save_path, 'imu_visual_slam_comparison.png'));
    fprintf('对比图已保存: %s/imu_visual_slam_comparison.png\n', save_path);
end
