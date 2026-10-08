% tar_lap_eval.m — Turnaround 折返集: 全段 + 分段(lap) 口径 ATE (论文口径 compute_metrics_with_alignment)
% 配置: dc基线(无A-fix) / a_disk(现A-fix cosine β1) / step_best(sweep候选) / EKF / VO
% 另输出: 原始路径长比(path/GT, 1.0=真实度量尺度) + 重锚定跳变计数(单帧位移>5m)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';

gt  = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
cfgs = {...
    'dc基线(无A-fix)',  fullfile(tar,'dc_final','exp_trajectory.txt');
    'a_disk(现A-fix)',  fullfile(tar,'a_final','exp_trajectory.txt');
    'stepβ2win400(候选)', dlmread('/tmp/tar_step_best.txt',',');
    'EKF(参考)',        fullfile(tar,'a_final','imu_aided_trajectory.txt');
    'VO(参考)',         fullfile(tar,'a_final','pure_visual_trajectory.txt')};

L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('GT: %d 帧, 路径长 %.1f m\n\n', size(gt,1), L);

% 全段口径(论文 Table 口径: 前100帧锚定 Sim(3))
fprintf('=== 全段 (前100帧锚定 Sim(3), 论文口径) ===\n');
fullR = zeros(size(cfgs,1),1); fullF = zeros(size(cfgs,1),1); fullD = zeros(size(cfgs,1),1);
for k = 1:size(cfgs,1)
    tr = cfgs{k,2};
    if ischar(tr), tr = csvread(tr); end
    [rn,fe,dr] = compute_metrics_with_alignment(tr, gt, L);
    fullR(k)=rn; fullF(k)=fe; fullD(k)=dr;
    fprintf('  %-20s ATE=%8.2f  终点=%7.2f  漂移=%6.2f%%\n', cfgs{k,1}, rn, fe, dr);
end

% lap 边界: GT 距起点 <8m 且连续 >=20 帧 视为"在起点", 离开起点即 lap 边界
d0 = sqrt(sum((gt-gt(1,:)).^2,2));
at = d0 < 8;
atS = at;
for i = 1:numel(at)
    if at(i)
        run = 0; j = i;
        while j <= numel(at) && at(j), j = j+1; end
        run = j - i;
        atS(i:min(j-1,numel(at))) = (run >= 20);
        i = j-1;
    end
end
cand = find(atS(1:end-1) & ~atS(2:end));          % 离开起点的帧(1-based)
bounds = [0; cand(:); numel(gt)];
bounds = bounds(diff(bounds) >= 500);            % 过滤过短段
fprintf('\n=== lap 边界 (帧): %s ===\n', mat2str(bounds));

% 分段口径: 每 lap 独立 compute_metrics_with_alignment
fprintf('\n=== 分段 ATE (每 lap 独立前100帧锚定 Sim(3)) ===\n');
nlap = numel(bounds)-1;
lapR = zeros(nlap, size(cfgs,1));
for k = 1:size(cfgs,1)
    tr = cfgs{k,2};
    if ischar(tr), tr = csvread(tr); end
    for i = 1:nlap
        a = bounds(i)+1; b = bounds(i+1);
        seg = gt(a:b,:); Lseg = sum(sqrt(sum(diff(seg).^2,2)));
        lapR(i,k) = compute_metrics_with_alignment(tr(a:b,:), seg, Lseg);
    end
end
hdr = '  %-22s';
for k = 1:size(cfgs,1), hdr = [hdr, sprintf('  %-14s', cfgs{k,1})]; end
fprintf(hdr, ''); fprintf('\n');
for i = 1:nlap
    line = sprintf('  lap%d [%d:%d] (%d帧)', i-1, bounds(i), bounds(i+1), bounds(i+1)-bounds(i));
    for k = 1:size(cfgs,1), line = [line, sprintf('  %14.2f', lapR(i,k))]; end
    fprintf('%s\n', line);
end

% 原始尺度恢复 + 跳变计数
fprintf('\n=== 原始(无对齐)路径长比 & 重锚定跳变(单帧>5m) ===\n');
for k = 1:size(cfgs,1)
    tr = cfgs{k,2};
    if ischar(tr), tr = csvread(tr); end
    Ltr = sum(sqrt(sum(diff(tr).^2,2)));
    jumps = sum(sqrt(sum(diff(tr).^2,2)) > 5);
    maxdisp = max(sqrt(sum(diff(tr).^2,2)));
    fprintf('  %-20s path/GT=%+.1f%%  跳变数=%4d  最大单帧位移=%8.2f m\n', ...
        cfgs{k}{1}, (Ltr/L-1)*100, jumps, maxdisp);
end
fprintf('\nLAP_EVAL_DONE\n');
