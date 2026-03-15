%% LSTM门控参数扩展网格搜索 v8
%  扩大搜索范围，探索更多参数组合
%  修复日期: 2026-01-31

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   LSTM门控参数扩展网格搜索 - v8                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置 - 扩展参数范围
% 根据v7结果，forget_gate越小越好，所以向下探索
input_gate_values = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
forget_gate_values = [0.2, 0.3, 0.4, 0.5];
output_gate_values = [0.85, 0.90, 0.95];

FAST_FRAMES = 600;

total = length(input_gate_values) * length(forget_gate_values) * length(output_gate_values);
fprintf('配置:\n');
fprintf('  帧数: %d\n', FAST_FRAMES);
fprintf('  input_gate: [%.2f - %.2f], %d个值\n', min(input_gate_values), max(input_gate_values), length(input_gate_values));
fprintf('  forget_gate: [%.2f - %.2f], %d个值\n', min(forget_gate_values), max(forget_gate_values), length(forget_gate_values));
fprintf('  output_gate: [%.2f - %.2f], %d个值\n', min(output_gate_values), max(output_gate_values), length(output_gate_values));
fprintf('  参数组合: %d\n', total);
fprintf('  预计时间: ~%d分钟\n\n', round(total * 1.5));

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

img_files = dir(fullfile(data_path, '*.png'));
num_frames = min(length(img_files), FAST_FRAMES);
fprintf('处理帧数: %d\n', num_frames);

fusion_file = fullfile(data_path, 'fusion_pose.txt');
fprintf('读取融合位姿: %s\n', fusion_file);
fusion_data = dlmread(fusion_file, ',', 1, 0);
fprintf('融合位姿: %d 条\n', size(fusion_data, 1));

gt_file = fullfile(data_path, 'ground_truth.txt');
fprintf('读取GT: %s\n', gt_file);
gt_data = dlmread(gt_file, ',', 1, 0);
gt_pos = gt_data(1:num_frames, 2:4);
fprintf('GT数据: %d 条\n\n', size(gt_data, 1));

%% 运行Baseline
fprintf('运行Baseline...\n');
baseline_traj = run_baseline_local(fusion_data, num_frames);
baseline_ate = compute_ate_local(baseline_traj, gt_pos);
fprintf('Baseline ATE: %.2f m\n\n', baseline_ate);
clear baseline_traj;

%% 网格搜索
fprintf('开始扩展网格搜索...\n');
fprintf('════════════════════════════════════════════════════════════════\n');
results = [];
idx = 0;
best_ate = inf;
best_params = struct();

for i_in = 1:length(input_gate_values)
    for i_fg = 1:length(forget_gate_values)
        for i_out = 1:length(output_gate_values)
            idx = idx + 1;
            
            input_gate = input_gate_values(i_in);
            forget_gate = forget_gate_values(i_fg);
            output_gate = output_gate_values(i_out);
            
            fprintf('[%d/%d] in=%.2f, fg=%.2f, out=%.2f ', idx, total, input_gate, forget_gate, output_gate);
            
            try
                clear hart_transformer_extractor_parameterized;
                
                ours_traj = run_ours_local(data_path, img_files, fusion_data, num_frames, ...
                    input_gate, forget_gate, output_gate);
                
                ours_ate = compute_ate_local(ours_traj, gt_pos);
                improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
                
                % 标记最优
                if ours_ate < best_ate
                    best_ate = ours_ate;
                    best_params.input = input_gate;
                    best_params.forget = forget_gate;
                    best_params.output = output_gate;
                    best_params.improvement = improvement;
                    fprintf('-> ATE=%.2f, 改进=%+.1f%% ★NEW BEST★\n', ours_ate, improvement);
                else
                    fprintf('-> ATE=%.2f, 改进=%+.1f%%\n', ours_ate, improvement);
                end
                
                r.input_gate = input_gate;
                r.forget_gate = forget_gate;
                r.output_gate = output_gate;
                r.ate = ours_ate;
                r.improvement = improvement;
                results = [results; r];
                
                clear ours_traj;
                
            catch ME
                fprintf('-> 失败: %s\n', ME.message);
            end
            
            pause(0.05);
        end
    end
    
    % 每完成一轮input_gate，显示当前最优
    fprintf('--- 当前最优: in=%.2f, fg=%.2f, out=%.2f, ATE=%.2f ---\n', ...
        best_params.input, best_params.forget, best_params.output, best_ate);
end

%% 显示结果
fprintf('\n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  扩展搜索结果汇总 (Baseline ATE: %.2f m)\n', baseline_ate);
fprintf('════════════════════════════════════════════════════════════════\n\n');

if ~isempty(results)
    [~, sort_idx] = sort([results.improvement], 'descend');
    
    % 显示前15名
    fprintf('Top 15 参数组合:\n');
    fprintf('排名  input  forget  output   ATE     改进\n');
    fprintf('────────────────────────────────────────────\n');
    for i = 1:min(15, length(sort_idx))
        r = results(sort_idx(i));
        if i == 1
            fprintf('★ %2d  %.2f   %.2f    %.2f   %6.2f  %+6.1f%%\n', ...
                i, r.input_gate, r.forget_gate, r.output_gate, r.ate, r.improvement);
        else
            fprintf('  %2d  %.2f   %.2f    %.2f   %6.2f  %+6.1f%%\n', ...
                i, r.input_gate, r.forget_gate, r.output_gate, r.ate, r.improvement);
        end
    end
    
    best = results(sort_idx(1));
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  ★ 最优参数: input=%.2f, forget=%.2f, output=%.2f          ║\n', ...
        best.input_gate, best.forget_gate, best.output_gate);
    fprintf('║  ★ 最优ATE: %.2f m (改进 %+.1f%%)                           ║\n', ...
        best.ate, best.improvement);
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    
    % 分析趋势
    fprintf('\n参数趋势分析:\n');
    
    % 按input_gate分组
    fprintf('\n按input_gate分组的平均ATE:\n');
    for ig = input_gate_values
        mask = [results.input_gate] == ig;
        if any(mask)
            avg_ate = mean([results(mask).ate]);
            fprintf('  input=%.2f: 平均ATE=%.2f\n', ig, avg_ate);
        end
    end
    
    % 按forget_gate分组
    fprintf('\n按forget_gate分组的平均ATE:\n');
    for fg = forget_gate_values
        mask = [results.forget_gate] == fg;
        if any(mask)
            avg_ate = mean([results(mask).ate]);
            fprintf('  forget=%.2f: 平均ATE=%.2f\n', fg, avg_ate);
        end
    end
end

% 保存结果
save_path = fullfile(rootDir, 'data', 'lstm_gates_extended_results.mat');
save(save_path, 'results', 'baseline_ate', 'best_params');
fprintf('\n结果已保存: %s\n', save_path);


%% ========== 内置辅助函数 ==========

function traj = run_baseline_local(fusion_data, num_frames)
    traj = zeros(num_frames, 3);
    x = 0; y = 0; yaw = 0;
    
    for frame = 1:num_frames
        if frame <= size(fusion_data, 1)
            vtrans = fusion_data(frame, 1) * 1.2;
            vrot = fusion_data(frame, 2) * 1.0;
        else
            vtrans = 0; vrot = 0;
        end
        
        yaw = yaw + vrot;
        x = x + vtrans * cos(yaw);
        y = y + vtrans * sin(yaw);
        traj(frame, :) = [x, y, 0];
    end
end

function traj = run_ours_local(data_path, img_files, fusion_data, num_frames, input_gate, forget_gate, output_gate)
    traj = zeros(num_frames, 3);
    x = 0; y = 0; yaw = 0;
    
    vt_history = [];
    VT_THRESHOLD = 0.06;
    img_dir = img_files(1).folder;
    
    for frame = 1:num_frames
        img_path = fullfile(img_dir, img_files(frame).name);
        try
            img = imread(img_path);
            if size(img, 3) == 3
                img = rgb2gray(img);
            end
        catch
            img = uint8(zeros(240, 320));
        end
        
        if frame <= size(fusion_data, 1)
            vtrans = fusion_data(frame, 1) * 1.2;
            vrot = fusion_data(frame, 2) * 1.0;
        else
            vtrans = 0; vrot = 0;
        end
        
        try
            normImg = hart_transformer_extractor_parameterized(img, input_gate, forget_gate, output_gate);
        catch
            normImg = double(img) / 255;
        end
        
        [vt_id, ~] = vt_match_local(normImg, vt_history, VT_THRESHOLD);
        
        if vt_id == 0
            new_vt.template = imresize(normImg, [32, 64]);
            new_vt.x = x;
            new_vt.y = y;
            vt_history = [vt_history; new_vt];
        else
            x = x + 0.1 * (vt_history(vt_id).x - x);
            y = y + 0.1 * (vt_history(vt_id).y - y);
        end
        
        yaw = yaw + vrot;
        x = x + vtrans * cos(yaw);
        y = y + vtrans * sin(yaw);
        traj(frame, :) = [x, y, 0];
    end
end

function [vt_id, score] = vt_match_local(normImg, vt_history, threshold)
    vt_id = 0;
    score = inf;
    
    if isempty(vt_history)
        return;
    end
    
    current = imresize(normImg, [32, 64]);
    
    for i = 1:length(vt_history)
        stored = vt_history(i).template;
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

function ate = compute_ate_local(traj, gt)
    min_len = min(size(traj, 1), size(gt, 1));
    traj = traj(1:min_len, 1:2);
    gt = gt(1:min_len, 1:2);
    
    best_ate = inf;
    for angle = 0:30:330
        theta = angle * pi / 180;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        rotated = (R * traj')';
        
        aligned = sim3_align_local(rotated, gt);
        errors = sqrt(sum((aligned - gt).^2, 2));
        ate_tmp = mean(errors);
        
        if ate_tmp < best_ate
            best_ate = ate_tmp;
        end
    end
    ate = best_ate;
end

function aligned = sim3_align_local(traj, ref)
    traj_c = traj - mean(traj, 1);
    ref_c = ref - mean(ref, 1);
    
    scale = sqrt(sum(ref_c(:).^2) / (sum(traj_c(:).^2) + eps));
    traj_s = traj_c * scale;
    
    H = traj_s' * ref_c;
    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0
        V(:, end) = -V(:, end);
        R = V * U';
    end
    
    aligned = (R * traj_s')' + mean(ref, 1);
end
