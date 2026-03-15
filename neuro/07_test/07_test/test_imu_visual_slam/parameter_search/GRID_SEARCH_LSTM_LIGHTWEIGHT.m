%% LSTM门控参数网格搜索 - 轻量级版本
%  解决MATLAB内存不足闪退问题
%  
%  优化措施:
%    1. 减少帧数到300帧
%    2. 每次实验后清理内存
%    3. 减少参数组合数
%    4. 禁用不必要的计算

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   LSTM门控参数网格搜索 - 轻量级版本 (防闪退)                 ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置 - 精简版
% 只测试9个组合 (3x3)，固定output_gate=0.92
input_gate_values = [0.45, 0.55, 0.65];
forget_gate_values = [0.5, 0.6, 0.7];
output_gate_values = [0.92];  % 固定

FAST_FRAMES = 300;  % 只用300帧，大幅减少内存

fprintf('配置:\n');
fprintf('  帧数: %d (轻量级)\n', FAST_FRAMES);
fprintf('  参数组合: %d\n', length(input_gate_values) * length(forget_gate_values) * length(output_gate_values));
fprintf('  预计时间: ~10分钟\n\n');

%% 初始化路径
currentDir = fileparts(mfilename('fullpath'));
testDir = fileparts(currentDir);
rootDir = fileparts(fileparts(fileparts(fileparts(currentDir))));

addpath(fullfile(rootDir, '04_visual_template'));
addpath(fullfile(rootDir, '04_visual_template/04_visual_template'));
addpath(fullfile(testDir, 'utils'));

%% 读取数据
data_path = fullfile(rootDir, 'data', 'Town01Data_IMU_Fusion');
fprintf('数据路径: %s\n', data_path);

% 读取图像列表
img_files = dir(fullfile(data_path, '*.png'));
num_frames = min(length(img_files), FAST_FRAMES);
fprintf('处理帧数: %d\n', num_frames);

% 读取融合位姿
fusion_file = fullfile(data_path, 'fused_pose.txt');
fusion_data = readmatrix(fusion_file, 'NumHeaderLines', 1);
fprintf('融合位姿: %d 条\n', size(fusion_data, 1));

% 读取GT
gt_file = fullfile(data_path, 'ground_truth.txt');
gt_data = readmatrix(gt_file, 'NumHeaderLines', 1);
gt_pos = gt_data(1:num_frames, 2:4);  % x, y, z
fprintf('GT数据: %d 条\n\n', size(gt_data, 1));

%% 运行简化Baseline
fprintf('运行Baseline...\n');
baseline_traj = run_simple_baseline(data_path, img_files, fusion_data, num_frames);
baseline_ate = compute_simple_ate(baseline_traj, gt_pos);
fprintf('Baseline ATE: %.2f m\n\n', baseline_ate);

% 清理内存
clear baseline_traj;
java.lang.System.gc();

%% 网格搜索
fprintf('开始网格搜索...\n');
results = [];
idx = 0;
total = length(input_gate_values) * length(forget_gate_values) * length(output_gate_values);

for i_in = 1:length(input_gate_values)
    for i_fg = 1:length(forget_gate_values)
        for i_out = 1:length(output_gate_values)
            idx = idx + 1;
            
            input_gate = input_gate_values(i_in);
            forget_gate = forget_gate_values(i_fg);
            output_gate = output_gate_values(i_out);
            
            fprintf('[%d/%d] in=%.2f, fg=%.2f, out=%.2f ', idx, total, input_gate, forget_gate, output_gate);
            
            try
                % 重置持久化变量
                clear hart_transformer_extractor_parameterized;
                
                % 运行实验
                ours_traj = run_simple_ours(data_path, img_files, fusion_data, num_frames, ...
                    input_gate, forget_gate, output_gate);
                
                % 计算ATE
                ours_ate = compute_simple_ate(ours_traj, gt_pos);
                improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
                
                fprintf('-> ATE=%.2f, 改进=%+.1f%%\n', ours_ate, improvement);
                
                % 保存结果
                r.input_gate = input_gate;
                r.forget_gate = forget_gate;
                r.output_gate = output_gate;
                r.ate = ours_ate;
                r.improvement = improvement;
                results = [results; r];
                
                % 清理内存
                clear ours_traj;
                
            catch ME
                fprintf('-> 失败: %s\n', ME.message);
            end
            
            % 强制垃圾回收
            java.lang.System.gc();
            pause(0.1);
        end
    end
end

%% 显示结果
fprintf('\n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  结果汇总\n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('Baseline ATE: %.2f m\n\n', baseline_ate);

if ~isempty(results)
    % 按改进排序
    [~, sort_idx] = sort([results.improvement], 'descend');
    
    fprintf('排名  input  forget  output   ATE     改进\n');
    fprintf('────────────────────────────────────────────\n');
    for i = 1:length(sort_idx)
        r = results(sort_idx(i));
        if i == 1
            fprintf('★ %d   %.2f   %.2f    %.2f   %.2f   %+.1f%%\n', ...
                i, r.input_gate, r.forget_gate, r.output_gate, r.ate, r.improvement);
        else
            fprintf('  %d   %.2f   %.2f    %.2f   %.2f   %+.1f%%\n', ...
                i, r.input_gate, r.forget_gate, r.output_gate, r.ate, r.improvement);
        end
    end
    
    % 最优参数
    best = results(sort_idx(1));
    fprintf('\n最优参数: input=%.2f, forget=%.2f, output=%.2f\n', ...
        best.input_gate, best.forget_gate, best.output_gate);
    fprintf('最优ATE: %.2f m (改进 %+.1f%%)\n', best.ate, best.improvement);
end

%% 保存结果
save_path = fullfile(rootDir, 'data', 'lstm_lightweight_results.mat');
save(save_path, 'results', 'baseline_ate');
fprintf('\n结果已保存: %s\n', save_path);


%% ========== 辅助函数 ==========

function traj = run_simple_baseline(data_path, img_files, fusion_data, num_frames)
    % 简化的Baseline - 只用视觉里程计
    traj = zeros(num_frames, 3);
    x = 0; y = 0; z = 0;
    yaw = 0;
    
    for frame = 1:num_frames
        if frame <= size(fusion_data, 1)
            vtrans = fusion_data(frame, 1) * 1.2;
            vrot = fusion_data(frame, 2) * 1.0;
        else
            vtrans = 0;
            vrot = 0;
        end
        
        yaw = yaw + vrot;
        x = x + vtrans * cos(yaw);
        y = y + vtrans * sin(yaw);
        
        traj(frame, :) = [x, y, z];
    end
end

function traj = run_simple_ours(data_path, img_files, fusion_data, num_frames, input_gate, forget_gate, output_gate)
    % 简化的Ours - 使用LSTM参数
    traj = zeros(num_frames, 3);
    x = 0; y = 0; z = 0;
    yaw = 0;
    
    % VT相关
    vt_history = [];
    VT_THRESHOLD = 0.06;
    
    img_dir = img_files(1).folder;
    
    for frame = 1:num_frames
        % 读取图像
        img_path = fullfile(img_dir, img_files(frame).name);
        try
            img = imread(img_path);
            if size(img, 3) == 3
                img = rgb2gray(img);
            end
        catch
            img = uint8(zeros(240, 320));
        end
        
        % 里程计
        if frame <= size(fusion_data, 1)
            vtrans = fusion_data(frame, 1) * 1.2;
            vrot = fusion_data(frame, 2) * 1.0;
        else
            vtrans = 0;
            vrot = 0;
        end
        
        % 使用参数化特征提取
        try
            normImg = hart_transformer_extractor_parameterized(img, input_gate, forget_gate, output_gate);
        catch
            normImg = double(img) / 255;
        end
        
        % 简化的VT匹配
        [vt_id, ~] = simple_vt_match(normImg, vt_history, VT_THRESHOLD);
        
        if vt_id == 0
            % 新VT
            new_vt.template = imresize(normImg, [32, 64]);
            new_vt.x = x;
            new_vt.y = y;
            vt_history = [vt_history; new_vt];
        else
            % 闭环校正
            correction = 0.1;
            x = x + correction * (vt_history(vt_id).x - x);
            y = y + correction * (vt_history(vt_id).y - y);
        end
        
        % 更新位姿
        yaw = yaw + vrot;
        x = x + vtrans * cos(yaw);
        y = y + vtrans * sin(yaw);
        
        traj(frame, :) = [x, y, z];
    end
end

function [vt_id, score] = simple_vt_match(normImg, vt_history, threshold)
    vt_id = 0;
    score = inf;
    
    if isempty(vt_history)
        return;
    end
    
    current = imresize(normImg, [32, 64]);
    
    for i = 1:length(vt_history)
        stored = vt_history(i).template;
        
        % 简单的SAD匹配
        diff = abs(current(:) - stored(:));
        sad = mean(diff);
        
        if sad < score
            score = sad;
            if sad < threshold
                vt_id = i;
            end
        end
    end
end

function ate = compute_simple_ate(traj, gt)
    % 简单的ATE计算 - 带Sim3对齐
    min_len = min(size(traj, 1), size(gt, 1));
    traj = traj(1:min_len, 1:2);
    gt = gt(1:min_len, 1:2);
    
    % 尝试多个旋转角度
    best_ate = inf;
    for angle = 0:30:330
        theta = angle * pi / 180;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        rotated = (R * traj')';
        
        % Sim3对齐
        aligned = sim3_align(rotated, gt);
        
        errors = sqrt(sum((aligned - gt).^2, 2));
        ate_tmp = mean(errors);
        
        if ate_tmp < best_ate
            best_ate = ate_tmp;
        end
    end
    
    ate = best_ate;
end

function aligned = sim3_align(traj, ref)
    traj_center = mean(traj, 1);
    ref_center = mean(ref, 1);
    
    traj_c = traj - traj_center;
    ref_c = ref - ref_center;
    
    scale = sqrt(sum(ref_c(:).^2) / (sum(traj_c(:).^2) + eps));
    traj_s = traj_c * scale;
    
    H = traj_s' * ref_c;
    [U, ~, V] = svd(H);
    R = V * U';
    
    if det(R) < 0
        V(:, end) = -V(:, end);
        R = V * U';
    end
    
    aligned = (R * traj_s')' + ref_center;
end
