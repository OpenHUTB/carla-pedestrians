% verify_euroc.m — EuRoC MH_01/03 仓库口径(compute_metrics_with_alignment)指标
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
for d = {'/home/yangrb/openhutb/neuro/data/02_EuRoc_Dataset/MH_01_easy', ...
         '/home/yangrb/openhutb/neuro/data/02_EuRoc_Dataset/MH_03_medium'}
    sr = fullfile(d{1}, 'slam_results');
    T = load(fullfile(sr, 'euroc_trajectories.mat'));
    n = min([size(T.gt_aligned,1), size(T.exp_aligned,1), size(T.fusion_aligned,1), size(T.pure_visual_aligned,1)]);
    gt = T.gt_aligned(1:n,:); exp = T.exp_aligned(1:n,:);
    fus = T.fusion_aligned(1:n,:); vo = T.pure_visual_aligned(1:n,:);
    L = sum(sqrt(sum(diff(gt).^2,2)));
    [rn,fe,dr] = compute_metrics_with_alignment(exp, gt, L);
    [re,ee,de] = compute_metrics_with_alignment(fus, gt, L);
    [rv,ve,dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('%s L=%.1fm: NLM=%.2f(t=%.2f,dr=%.2f%%) EKF=%.2f(%.2f%%) VO=%.2f(%.2f%%)\n', ...
        d{1}(end-21:end), L, rn,fe,dr, re,de, rv,dv);
end
fprintf('VERIFY_EUROC_DONE\n');
