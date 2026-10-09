% tar_perlap_paper.m — 用论文真实口径(主ablation compute_metrics_with_alignment, 版本B全轨迹Procrustes)
% 在 Turnaround 上做 分段(每lap独立对齐)ATE: 这是"闭环增益唯一可测"的口径
% (每lap独立吸收该lap的全局Sim3, 残差=非系统性局部漂移=闭环唯一能改善的量)
% 同时给 全局(版本B单段) 作对照, 得出 系统性 vs 非系统性 分解
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';

gt   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
cfgs = {'EKF',       csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
        'dc基线',    csvread(fullfile(tar,'dc_final','exp_trajectory.txt'));
        'a_disk(A-fix)', csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
        'VO',        csvread(fullfile(tar,'a_final','pure_visual_trajectory.txt'))};
L = sum(sqrt(sum(diff(gt).^2,2)));
fprintf('GT %d帧 路径%.1fm\n\n', size(gt,1), L);

% ---- lap 边界: GT距起点<8m 且连续>=20帧 视为在起点 ----
d0 = sqrt(sum((gt-gt(1,:)).^2,2));
at = d0 < 8; atS = at;
i = 1;
while i <= numel(at)
    if at(i)
        j = i;
        while j <= numel(at) && at(j), j = j+1; end
        atS(i:min(j-1,numel(at))) = (j-1-i+1) >= 20;
        i = j;
    else
        i = i+1;
    end
end
cand = find(atS(1:end-1) & ~atS(2:end));
bounds = [0; cand(:); numel(gt)];
bounds = bounds(diff(bounds) >= 500);
fprintf('=== lap 边界: %s ===\n\n', mat2str(bounds));

ncfg = size(cfgs,1);
nlap = numel(bounds)-1;
lapR = zeros(nlap, ncfg);
for k = 1:ncfg
    tr = cfgs{k,2};
    for i = 1:nlap
        a = bounds(i)+1; b = bounds(i+1);
        seg = gt(a:b,:); Lseg = sum(sqrt(sum(diff(seg).^2,2)));
        lapR(i,k) = compute_metrics_with_alignment(tr(a:b,:), seg, Lseg);
    end
end
% 加权分段ATE(非系统性局部漂移)
wsum = 0; ewsum = 0;
for k = 1:ncfg
    for i = 1:nlap
        wsum = wsum + (bounds(i+1)-bounds(i));
        ewsum = ewsum + lapR(i,k)^2 * (bounds(i+1)-bounds(i));
    end
    fprintf('  %-14s 分段(非系统性)加权ATE = %8.2f m\n', cfgs{k,1}, sqrt(ewsum/wsum));
    ewsum = 0;
end
fprintf('\n');
for i = 1:nlap
    line = sprintf('  lap%d [%5d:%5d] (%5d帧)', i-1, bounds(i), bounds(i+1), bounds(i+1)-bounds(i));
    for k = 1:ncfg, line = [line, sprintf('  %14.2f', lapR(i,k))]; end
    fprintf('%s\n', line);
end
fprintf('\n=== 全局(版本B单段Procrustes, 论文主表口径) 对照 ===\n');
for k = 1:ncfg
    fprintf('  %-14s 全局ATE = %8.2f m\n', cfgs{k,1}, compute_metrics_with_alignment(cfgs{k,2}, gt, L));
end
fprintf('\n判读: 分段(非系统性)加权ATE >> 全局 → EKF/NLM 含大量闭环可校正的局部漂移, 真实增益物理可达;\n');
fprintf('      分段 ≈ 全局 → 误差几乎全是全局Sim3, 闭环在该口径下无真实增益空间.\n');
fprintf('PERLAP_PAPER_DONE\n');
