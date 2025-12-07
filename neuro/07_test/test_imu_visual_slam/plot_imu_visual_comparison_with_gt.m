function plot_imu_visual_comparison_with_gt(fusion_data, odo_trajectory, exp_trajectory, gt_data, save_dir)
    % PLOT_IMU_VISUAL_COMPARISON_WITH_GT 绘制包含Ground Truth的轨迹对比
    %
    % 输入:
    %   fusion_data - IMU-视觉融合数据
    %   odo_trajectory - 视觉里程计轨迹
    %   exp_trajectory - 经验地图轨迹
    %   gt_data - Ground Truth数据
    %   save_dir - 保存目录（可选）
    
    if nargin < 5
        save_dir = '.';
    end
    
    figure('Position', [100, 100, 1600, 1000]);
    
    %% 1. 3D轨迹对比
    subplot(2, 3, 1);
    hold on; grid on;
    plot3(gt_data.pos(:,1), gt_data.pos(:,2), gt_data.pos(:,3), ...
        'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    plot3(fusion_data.pos(:,1), fusion_data.pos(:,2), fusion_data.pos(:,3), ...
        'r-', 'LineWidth', 1.5, 'DisplayName', 'IMU-Visual Fusion');
    plot3(odo_trajectory(:,1), odo_trajectory(:,2), odo_trajectory(:,3), ...
        'b--', 'LineWidth', 1, 'DisplayName', 'Visual Odometry');
    if ~isempty(exp_trajectory) && any(exp_trajectory(:) ~= 0)
        plot3(exp_trajectory(:,1), exp_trajectory(:,2), exp_trajectory(:,3), ...
            'g:', 'LineWidth', 1.5, 'DisplayName', 'Experience Map');
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D轨迹对比（含Ground Truth）');
    legend('Location', 'best');
    view(45, 30);
    axis equal;
    
    %% 2. 2D俯视图对比
    subplot(2, 3, 2);
    hold on; grid on;
    plot(gt_data.pos(:,1), gt_data.pos(:,2), ...
        'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    plot(fusion_data.pos(:,1), fusion_data.pos(:,2), ...
        'r-', 'LineWidth', 1.5, 'DisplayName', 'IMU-Visual Fusion');
    plot(odo_trajectory(:,1), odo_trajectory(:,2), ...
        'b--', 'LineWidth', 1, 'DisplayName', 'Visual Odometry');
    if ~isempty(exp_trajectory) && any(exp_trajectory(:) ~= 0)
        plot(exp_trajectory(:,1), exp_trajectory(:,2), ...
            'g:', 'LineWidth', 1.5, 'DisplayName', 'Experience Map');
    end
    % 标记起点和终点
    plot(gt_data.pos(1,1), gt_data.pos(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '起点');
    plot(gt_data.pos(end,1), gt_data.pos(end,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '终点');
    xlabel('X (m)'); ylabel('Y (m)');
    title('2D轨迹对比（俯视图）');
    legend('Location', 'best');
    axis equal;
    
    %% 3. 位置误差随时间变化（相对于GT）
    subplot(2, 3, 3);
    hold on; grid on;
    
    % 计算位置误差
    fusion_error = sqrt(sum((fusion_data.pos - gt_data.pos).^2, 2));
    if size(odo_trajectory, 1) == size(gt_data.pos, 1)
        odo_error = sqrt(sum((odo_trajectory - gt_data.pos).^2, 2));
    else
        odo_error = zeros(size(fusion_error));
    end
    if size(exp_trajectory, 1) == size(gt_data.pos, 1) && any(exp_trajectory(:) ~= 0)
        exp_error = sqrt(sum((exp_trajectory - gt_data.pos).^2, 2));
    else
        exp_error = zeros(size(fusion_error));
    end
    
    frames = 1:length(fusion_error);
    plot(frames, fusion_error, 'r-', 'LineWidth', 1.5, 'DisplayName', 'IMU-Visual Fusion');
    if any(odo_error ~= 0)
        plot(frames, odo_error, 'b--', 'LineWidth', 1, 'DisplayName', 'Visual Odometry');
    end
    if any(exp_error ~= 0)
        plot(frames, exp_error, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Experience Map');
    end
    
    % 显示平均误差
    plot([1, length(frames)], [mean(fusion_error), mean(fusion_error)], ...
        'r--', 'LineWidth', 1, 'DisplayName', sprintf('Fusion平均: %.2fm', mean(fusion_error)));
    
    xlabel('帧数'); ylabel('位置误差 (m)');
    title('位置误差随时间变化（相对于GT）');
    legend('Location', 'best');
    
    %% 4. 误差分布对比
    subplot(2, 3, 4);
    hold on; grid on;
    
    errors = [fusion_error, odo_error, exp_error];
    labels = {'IMU-Visual Fusion', 'Visual Odometry', 'Experience Map'};
    boxplot(errors, 'Labels', labels);
    ylabel('位置误差 (m)');
    title('不同方法的误差分布');
    xtickangle(45);
    
    %% 5. XYZ各轴误差对比
    subplot(2, 3, 5);
    hold on; grid on;
    
    fusion_xyz_error = abs(fusion_data.pos - gt_data.pos);
    plot(frames, fusion_xyz_error(:,1), 'r-', 'LineWidth', 1, 'DisplayName', 'X误差');
    plot(frames, fusion_xyz_error(:,2), 'g-', 'LineWidth', 1, 'DisplayName', 'Y误差');
    plot(frames, fusion_xyz_error(:,3), 'b-', 'LineWidth', 1, 'DisplayName', 'Z误差');
    
    xlabel('帧数'); ylabel('位置误差 (m)');
    title('IMU-Visual Fusion各轴误差');
    legend('Location', 'best');
    
    %% 6. 轨迹长度对比
    subplot(2, 3, 6);
    
    % 计算各方法的轨迹长度
    gt_length = sum(sqrt(sum(diff(gt_data.pos).^2, 2)));
    fusion_length = sum(sqrt(sum(diff(fusion_data.pos).^2, 2)));
    odo_length = sum(sqrt(sum(diff(odo_trajectory).^2, 2)));
    if ~isempty(exp_trajectory) && any(exp_trajectory(:) ~= 0)
        exp_length = sum(sqrt(sum(diff(exp_trajectory).^2, 2)));
    else
        exp_length = 0;
    end
    
    lengths = [gt_length, fusion_length, odo_length, exp_length];
    labels = {'Ground Truth', 'IMU-Visual Fusion', 'Visual Odometry', 'Experience Map'};
    
    bar(lengths);
    set(gca, 'XTickLabel', labels);
    ylabel('轨迹长度 (m)');
    title('轨迹长度对比');
    xtickangle(45);
    grid on;
    
    % 添加数值标签
    for i = 1:length(lengths)
        text(i, lengths(i), sprintf('%.2fm', lengths(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    
    %% 保存图像
    save_path = fullfile(save_dir, 'imu_visual_slam_comparison.png');
    saveas(gcf, save_path);
    fprintf('对比图已保存: %s\n', save_path);
end
