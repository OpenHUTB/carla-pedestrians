%% 验证KITTI数据传递是否正确

fprintf('\n========== 验证KITTI数据传递 ==========\n\n');

% 1. 读取fusion数据
scriptPath = fileparts(mfilename('fullpath'));
testDir = fileparts(scriptPath);
test07Dir = fileparts(testDir);
neuroDir = fileparts(test07Dir);
data_path = fullfile(neuroDir, 'data', 'KITTI_07');
fusion_file = fullfile(data_path, 'fusion_pose.txt');

fprintf('[1/4] 读取fusion_pose.txt...\n');
fid = fopen(fusion_file, 'r');
fusion_raw = [];
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line) || isempty(line) || line(1) == '%'
        continue;
    end
    fusion_raw = [fusion_raw; str2num(line)];
end
fclose(fid);

fprintf('  ✓ 读取了 %d 行数据\n', size(fusion_raw, 1));

% 2. 解析数据
fusion_data = struct();
fusion_data.pos = fusion_raw(:, 2:4);
fusion_data.att = fusion_raw(:, 5:7);
fusion_data.vel = fusion_raw(:, 8:10);
fusion_data.imu_acc = fusion_raw(:, 11:13);
fusion_data.imu_gyro = fusion_raw(:, 14:16);
fusion_data.timestamp = fusion_raw(:, 1);

fprintf('\n[2/4] 检查GT轨迹...\n');
gt_traj_length = 0;
for i = 2:size(fusion_data.pos, 1)
    gt_traj_length = gt_traj_length + norm(fusion_data.pos(i,:) - fusion_data.pos(i-1,:));
end
fprintf('  GT轨迹长度: %.2f m\n', gt_traj_length);
fprintf('  起点: [%.2f, %.2f, %.2f]\n', fusion_data.pos(1,:));
fprintf('  终点: [%.2f, %.2f, %.2f]\n', fusion_data.pos(end,:));
fprintf('  直线距离: %.2f m\n', norm(fusion_data.pos(end,:) - fusion_data.pos(1,:)));

% 3. 检查IMU数据
fprintf('\n[3/4] 创建imu_data结构...\n');
imu_data = struct();
imu_data.gyro = fusion_data.imu_gyro;
imu_data.accel = fusion_data.imu_acc;
imu_data.timestamp = fusion_data.timestamp;
imu_data.count = size(fusion_data.imu_acc, 1);

fprintf('  字段检查:\n');
fprintf('    imu_data.gyro: %dx%d\n', size(imu_data.gyro));
fprintf('    imu_data.accel: %dx%d\n', size(imu_data.accel));
fprintf('    imu_data.timestamp: %dx%d\n', size(imu_data.timestamp));
fprintf('    imu_data.count: %d\n', imu_data.count);

fprintf('\n  IMU数据范围:\n');
fprintf('    陀螺仪 X: [%.3f, %.3f] rad/s\n', min(imu_data.gyro(:,1)), max(imu_data.gyro(:,1)));
fprintf('    陀螺仪 Y: [%.3f, %.3f] rad/s\n', min(imu_data.gyro(:,2)), max(imu_data.gyro(:,2)));
fprintf('    陀螺仪 Z: [%.3f, %.3f] rad/s\n', min(imu_data.gyro(:,3)), max(imu_data.gyro(:,3)));
fprintf('    加速度 X: [%.2f, %.2f] m/s²\n', min(imu_data.accel(:,1)), max(imu_data.accel(:,1)));
fprintf('    加速度 Y: [%.2f, %.2f] m/s²\n', min(imu_data.accel(:,2)), max(imu_data.accel(:,2)));
fprintf('    加速度 Z: [%.2f, %.2f] m/s²\n', min(imu_data.accel(:,3)), max(imu_data.accel(:,3)));

% 4. 测试imu_aided_visual_odometry函数
fprintf('\n[4/4] 测试视觉里程计函数调用...\n');
img_path = fullfile(data_path, 'image_0');
img_files = dir(fullfile(img_path, '*.png'));
[~, idx] = sort({img_files.name});
img_files = img_files(idx);

% 读取第一张图像
img1 = imread(fullfile(img_path, img_files(1).name));
if size(img1, 3) == 3
    img1 = rgb2gray(img1);
end

fprintf('  第1帧图像: %dx%d\n', size(img1, 2), size(img1, 1));

% 测试前10帧
fprintf('\n  测试前10帧的视觉里程计输出:\n');
fprintf('  帧号  | transV (m) | yawRotV (deg) | heightV (m)\n');
fprintf('  ------|------------|---------------|-------------\n');

test_trans_sum = 0;
test_yaw_sum = 0;

% 添加必要的路径
addpath(fullfile(neuroDir, '03_visual_odometry'));
addpath(fullfile(neuroDir, '09_vestibular'));

for i = 1:min(10, length(img_files))
    img = imread(fullfile(img_path, img_files(i).name));
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    try
        [transV, yawRotV, heightV] = imu_aided_visual_odometry(img, imu_data, i);
        fprintf('  %5d | %10.4f | %13.4f | %11.4f\n', i, transV, yawRotV, heightV);
        test_trans_sum = test_trans_sum + abs(transV);
        test_yaw_sum = test_yaw_sum + abs(yawRotV);
    catch ME
        fprintf('  %5d | ERROR: %s\n', i, ME.message);
    end
end

fprintf('\n  前10帧累积:\n');
fprintf('    平移累积: %.4f m (平均 %.4f m/帧)\n', test_trans_sum, test_trans_sum/10);
fprintf('    转向累积: %.4f 度 (平均 %.4f 度/帧)\n', test_yaw_sum, test_yaw_sum/10);

% 5. 对比分析
fprintf('\n========== 对比分析 ==========\n');
fprintf('GT前10帧:\n');
gt_trans_10 = 0;
for i = 2:10
    gt_trans_10 = gt_trans_10 + norm(fusion_data.pos(i,:) - fusion_data.pos(i-1,:));
end
fprintf('  GT平移: %.4f m (平均 %.4f m/帧)\n', gt_trans_10, gt_trans_10/9);

if test_trans_sum > 0
    ratio = test_trans_sum / gt_trans_10;
    fprintf('\n  视觉/GT比率: %.2f\n', ratio);
    if ratio < 0.5
        fprintf('  ⚠️  视觉估计距离太短！需要增大ODO_TRANS_V_SCALE\n');
        fprintf('  建议乘以: %.1f 倍\n', 1/ratio);
    elseif ratio > 2.0
        fprintf('  ⚠️  视觉估计距离太长！需要减小ODO_TRANS_V_SCALE\n');
        fprintf('  建议乘以: %.1f 倍\n', 1/ratio);
    else
        fprintf('  ✓ 视觉估计距离合理\n');
    end
else
    fprintf('  ⚠️  视觉里程计没有输出运动！\n');
end

fprintf('\n========== 验证完成 ==========\n\n');
