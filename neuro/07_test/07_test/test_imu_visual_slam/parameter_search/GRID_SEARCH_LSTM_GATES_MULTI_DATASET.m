%% LSTM门控参数网格搜索 - 轻量级防闪退版本 v7
%  兼容老版本MATLAB (使用dlmread替代readmatrix)
%  修复日期: 2026-01-31

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   LSTM门控参数网格搜索 - 600帧版本 v7                        ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 配置
input_gate_values = [0.45, 0.55, 0.65];
forget_gate_values = [0.5, 0.6, 0.7];
output_gate_values = [0.92];

FAST_FRAMES = 600;  % 600帧

total = length(input_gate_values) * length(forget_gate_values) * length(output_gate_values);
fprintf('配置:\n');
fprintf('  帧数: %d\n', FAST_FRAMES);
fprintf('  参数组合: %d\n', total);
fprintf('  预计时间: ~15-20分钟\n\n');

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

% ★★★ 使用dlmread替代readmatrix（兼容老版本MATLAB）★★★
% 注意: 文件名是 fusion_pose.txt 不是 fused_pose.txt
fusion_file = fullfile(data_path, 'fusion_pose.txt');
fprintf('读取融合位姿: %s\n', fusion_file);
fusion_data = dlmread(fusion_file, ',', 1, 0);  % 跳过1行表头
fprintf('融合位姿: %d 条\n', size(fusion_data, 1));

gt_file = fullfile(data_path, 'ground_truth.txt');
fprintf('读取GT: %s\n', gt_file);
gt_data = dlmread(gt_file, ',', 1, 0);  % 跳过1行表头
gt_pos = gt_data(1:num_frames, 2:4);
fprintf('GT数据: %d 条\n\n', size(gt_data, 1));

%% 运行Baseline
fprintf('运行Baseline...\n');
baseline_traj = run_baseline_local(fusion_data, num_frames);
baseline_ate = compute_ate_local(baseline_traj, gt_pos);
fprintf('Baseline ATE: %.2f m\n\n', baseline_ate);

clear baseline_traj;

%% 网格搜索
fprintf('开始网格搜索...\n');
results = [];
idx = 0;

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
                ours_traj = run_ours_local(data_path, img_files, fusion_data, num_frames, ...
                    input_gate, forget_gate, output_gate);
                
                ours_ate = compute_ate_local(ours_traj, gt_pos);
                improvement = (baseline_ate - ours_ate) / baseline_ate * 100;
                
                fprintf('-> ATE=%.2f, 改进=%+.1f%%\n', ours_ate, improvement);
                
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
            
            pause(0.1);
        end
    end
end

%% 显示结果
fprintf('\n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  结果汇总 (Baseline ATE: %.2f m)\n', baseline_ate);
fprintf('════════════════════════════════════════════════════════════════\n\n');

if ~isempty(results)
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
    
    best = results(sort_idx(1));
    fprintf('\n★ 最优参数: input=%.2f, forget=%.2f, output=%.2f\n', ...
        best.input_gate, best.forget_gate, best.output_gate);
    fprintf('★ 最优ATE: %.2f m (改进 %+.1f%%)\n', best.ate, best.improvement);
end

% 保存结果
save_path = fullfile(rootDir, 'data', 'lstm_gates_multi_dataset_results.mat');
save(save_path, 'results', 'baseline_ate');
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
