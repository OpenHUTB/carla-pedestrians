%% 追踪 coder.isColumnMajor 错误的真正来源
%  这个脚本会模拟网格搜索的调用流程，找出哪个函数触发了错误

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  追踪 coder.isColumnMajor 错误来源\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% 设置路径
cd('E:\Neuro_end\neuro');
addpath(genpath('07_test/07_test/test_imu_visual_slam'));
addpath(genpath('01_conjunctive_pose_cells_network'));
addpath(genpath('04_visual_template'));
addpath(genpath('03_visual_odometry'));
addpath(genpath('02_multilayered_experience_map'));
addpath(genpath('09_vestibular'));

%% 测试1: 基本函数
fprintf('测试1: 基本MATLAB函数\n');
test_img = rand(100, 100);

fprintf('  mean(): ');
try
    r = mean(test_img(:));
    fprintf('✓ 正常\n');
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

fprintf('  mean2(): ');
try
    r = mean2(test_img);
    fprintf('✓ 正常\n');
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

%% 测试2: 检查哪些文件被MATLAB加载
fprintf('\n测试2: 检查关键函数位置\n');

funcs_to_check = {
    'align_trajectories'
    'vt_compare_ncc'
    'visual_template'
    'visual_template_neuro_matlab_only'
    'visual_template_baseline'
    'hart_transformer_extractor_parameterized'
    'read_ground_truth'
    'read_fusion_pose'
    'run_ours_slam_lstm'
    'run_baseline_slam'
};

for i = 1:length(funcs_to_check)
    func_name = funcs_to_check{i};
    func_path = which(func_name);
    if isempty(func_path)
        fprintf('  %s: ✗ 未找到\n', func_name);
    else
        fprintf('  %s: %s\n', func_name, func_path);
    end
end

%% 测试3: 逐个测试关键函数
fprintf('\n测试3: 逐个测试关键函数\n');

% 3.1 测试 vt_compare_ncc
fprintf('\n  3.1 vt_compare_ncc: ');
try
    seg1 = rand(32, 64);
    seg2 = rand(32, 64);
    dist = vt_compare_ncc(seg1, seg2);
    fprintf('✓ 正常 (dist=%.4f)\n', dist);
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

% 3.2 测试 align_trajectories
fprintf('\n  3.2 align_trajectories: ');
try
    traj1 = rand(100, 3);
    traj2 = rand(100, 3);
    [a1, a2, R, t, s] = align_trajectories(traj1, traj2, 'umeyama_2d');
    fprintf('✓ 正常\n');
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

% 3.3 测试 hart_transformer_extractor_parameterized
fprintf('\n  3.3 hart_transformer_extractor_parameterized: ');
try
    test_gray = uint8(rand(240, 320) * 255);
    normImg = hart_transformer_extractor_parameterized(test_gray, 0.55, 0.6, 0.92);
    fprintf('✓ 正常 (size=%dx%d)\n', size(normImg, 1), size(normImg, 2));
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

% 3.4 测试 read_ground_truth
fprintf('\n  3.4 read_ground_truth: ');
try
    gt_file = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion\ground_truth.txt';
    gt_data = read_ground_truth(gt_file);
    fprintf('✓ 正常 (rows=%d)\n', size(gt_data.pos, 1));
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

%% 测试4: 模拟ATE计算流程
fprintf('\n测试4: 模拟ATE计算流程\n');

% 创建模拟轨迹
traj = rand(100, 2) * 100;
gt = rand(100, 2) * 100;

fprintf('  4.1 Sim3对齐 (本地函数): ');
try
    % 本地Sim3对齐
    traj_center = mean(traj, 1);
    ref_center = mean(gt, 1);
    traj_centered = traj - traj_center;
    ref_centered = gt - ref_center;
    scale = sqrt(sum(ref_centered(:).^2) / (sum(traj_centered(:).^2) + eps));
    traj_scaled = traj_centered * scale;
    H = traj_scaled' * ref_centered;
    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0
        V(:, end) = -V(:, end);
        R = V * U';
    end
    aligned = (R * traj_scaled')' + ref_center;
    errors = sqrt(sum((aligned - gt).^2, 2));
    ate = mean(errors);
    fprintf('✓ 正常 (ATE=%.2f)\n', ate);
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

fprintf('  4.2 使用 align_trajectories: ');
try
    traj3d = [traj, zeros(100, 1)];
    gt3d = [gt, zeros(100, 1)];
    [aligned, ~, ~, ~, ~] = align_trajectories(traj3d, gt3d, 'umeyama_2d');
    errors = sqrt(sum((aligned(:,1:2) - gt).^2, 2));
    ate = mean(errors);
    fprintf('✓ 正常 (ATE=%.2f)\n', ate);
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

%% 测试5: 完整的run_ours_slam_lstm流程（简化版）
fprintf('\n测试5: 模拟完整SLAM流程\n');

data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';

% 5.1 读取数据
fprintf('  5.1 读取IMU数据: ');
try
    imu_data = read_imu_data(data_path);
    fprintf('✓ 正常 (%d帧)\n', imu_data.count);
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

% 5.2 读取融合位姿
fprintf('  5.2 读取融合位姿: ');
try
    fusion_data = read_fusion_pose(data_path);
    fprintf('✓ 正常\n');
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

% 5.3 读取GT
fprintf('  5.3 读取Ground Truth: ');
try
    gt_file = fullfile(data_path, 'ground_truth.txt');
    gt_data = read_ground_truth(gt_file);
    fprintf('✓ 正常 (%d帧)\n', size(gt_data.pos, 1));
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

% 5.4 测试图像处理
fprintf('  5.4 图像处理: ');
try
    img_files = dir(fullfile(data_path, '*.png'));
    if ~isempty(img_files)
        img = imread(fullfile(data_path, img_files(1).name));
        if size(img, 3) == 3
            img_gray = rgb2gray(img);
        else
            img_gray = img;
        end
        normImg = hart_transformer_extractor_parameterized(img_gray, 0.55, 0.6, 0.92);
        fprintf('✓ 正常\n');
    else
        fprintf('✗ 未找到图像\n');
    end
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
    fprintf('      调用栈:\n');
    for j = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(j).name, ME.stack(j).line);
    end
end

% 5.5 测试VT匹配（这可能是问题所在）
fprintf('  5.5 VT匹配 (visual_template_match): ');
try
    % 创建模拟VT历史
    vt_history = struct('id', 1, 'template', rand(32, 64), 'first_frame', 1);
    current_template = rand(32, 64);
    
    % 简单的NCC匹配
    stored = vt_history.template;
    diff = 1 - abs(corr2(current_template, stored));
    fprintf('✓ 正常 (diff=%.4f)\n', diff);
catch ME
    fprintf('✗ 错误: %s\n', ME.message);
end

%% 测试6: 搜索所有可能调用mean2的地方
fprintf('\n测试6: 搜索代码中的mean2调用\n');

search_dirs = {
    '07_test/07_test/test_imu_visual_slam'
    '04_visual_template'
    '09_vestibular'
};

for d = 1:length(search_dirs)
    dir_path = fullfile('E:\Neuro_end\neuro', search_dirs{d});
    m_files = dir(fullfile(dir_path, '**', '*.m'));
    
    for f = 1:length(m_files)
        file_path = fullfile(m_files(f).folder, m_files(f).name);
        content = fileread(file_path);
        lines = strsplit(content, '\n');
        
        for ln = 1:length(lines)
            line = lines{ln};
            % 跳过注释行
            if ~isempty(strtrim(line)) && ~startsWith(strtrim(line), '%')
                % 检查是否有mean2函数调用（不是变量赋值）
                if contains(line, 'mean2(') && ~contains(line, 'mean2 =') && ~contains(line, 'local_mean2')
                    fprintf('  发现 mean2() 调用:\n');
                    fprintf('    文件: %s\n', file_path);
                    fprintf('    行号: %d\n', ln);
                    fprintf('    内容: %s\n', strtrim(line));
                end
            end
        end
    end
end

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  诊断完成\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
