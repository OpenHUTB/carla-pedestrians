% verify_all.m — 论文 Table 1 全量核验: 7数据集 x 3方法 (仓库口径)
% 输出: RMSE / 终点误差 / 漂移率, 并打印 vs EKF / vs VO 改进率
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/ablation');
base = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
sets = {'Town01Data_IMU_Fusion','Town02Data_IMU_Fusion','Town05Data_IMU_Fusion', ...
        'Town10HDData_IMU_Fusion','KITTI07Data_IMU_Fusion'};
for k = 1:numel(sets)
    ds = sets{k};
    sr = fullfile(base, ds, 'slam_results');
    gt  = csvread(fullfile(sr, 'ground_truth_backup.txt'));
    nlm = csvread(fullfile(sr, 'exp_trajectory.txt'));
    ekf = csvread(fullfile(sr, 'imu_aided_trajectory.txt'));
    vo  = csvread(fullfile(sr, 'pure_visual_trajectory.txt'));
    L = sum(sqrt(sum(diff(gt).^2, 2)));
    [rn,fe,dr] = compute_metrics_with_alignment(nlm, gt, L);
    [re,ee,de] = compute_metrics_with_alignment(ekf, gt, L);
    [rv,ve,dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('%s L=%.0fm: NLM=%.2f(t=%.2f,dr=%.2f%%) EKF=%.2f(t=%.2f,dr=%.2f%%) VO=%.2f(t=%.2f,dr=%.2f%%) vsEKF=%+.1f%% vsVO=%+.1f%%\n', ...
        ds, L, rn,fe,dr, re,ee,de, rv,ve,dv, ...
        (re-rn)/re*100, (rv-rn)/rv*100);
end
% EuRoC
for d = {'/home/yangrb/openhutb/neuro/data/02_EuRoc_Dataset/MH_01_easy', ...
         '/home/yangrb/openhutb/neuro/data/02_EuRoc_Dataset/MH_03_medium'}
    sr = fullfile(d{1}, 'slam_results');
    T = load(fullfile(sr, 'euroc_trajectories.mat'));
    n = min([size(T.gt_aligned,1), size(T.exp_aligned,1), size(T.fusion_aligned,1), size(T.pure_visual_aligned,1)]);
    gt = T.gt_aligned(1:n,:); expt = T.exp_aligned(1:n,:);
    fus = T.fusion_aligned(1:n,:); vo = T.pure_visual_aligned(1:n,:);
    L = sum(sqrt(sum(diff(gt).^2,2)));
    [rn,fe,dr] = compute_metrics_with_alignment(expt, gt, L);
    [re,ee,de] = compute_metrics_with_alignment(fus, gt, L);
    [rv,ve,dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('%s L=%.1fm: NLM=%.2f(t=%.2f,dr=%.2f%%) EKF=%.2f(t=%.2f,dr=%.2f%%) VO=%.2f(t=%.2f,dr=%.2f%%) vsEKF=%+.1f%% vsVO=%+.1f%%\n', ...
        d{1}(end-21:end), L, rn,fe,dr, re,ee,de, rv,ve,dv, ...
        (re-rn)/re*100, (rv-rn)/rv*100);
end
% Turnaround (若已生成) — 结果在数据集根目录 a_final/ dc_final/
tar = fullfile(base, 'Town01Turnaround_IMU_Fusion');
if exist(fullfile(tar, 'a_final', 'exp_trajectory.txt'), 'file')
    gt  = csvread(fullfile(tar, 'a_final', 'ground_truth_backup.txt'));
    ekf = csvread(fullfile(tar, 'a_final', 'imu_aided_trajectory.txt'));
    vo  = csvread(fullfile(tar, 'a_final', 'pure_visual_trajectory.txt'));
    L = sum(sqrt(sum(diff(gt).^2, 2)));
    nlmC = csvread(fullfile(tar, 'a_final', 'exp_trajectory.txt'));
    nlmD = csvread(fullfile(tar, 'dc_final', 'exp_trajectory.txt'));
    [rn,fe,dr] = compute_metrics_with_alignment(nlmC, gt, L);
    [rd,fd,dd] = compute_metrics_with_alignment(nlmD, gt, L);
    [re,ee,de] = compute_metrics_with_alignment(ekf, gt, L);
    [rv,ve,dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('TURNAROUND L=%.0fm: NLM(constraint)=%.2f(t=%.2f,dr=%.2f%%) NLM(dc only)=%.2f(t=%.2f,dr=%.2f%%) EKF=%.2f(t=%.2f,dr=%.2f%%) VO=%.2f(t=%.2f,dr=%.2f%%)\n', ...
        L, rn,fe,dr, rd,fd,dd, re,ee,de, rv,ve,dv);
    fprintf('  constraint vs dc: %+.1f%% | NLM(constraint) vs EKF: %+.1f%% | vs VO: %+.1f%%\n', ...
        (rd-rn)/rd*100, (re-rn)/re*100, (rv-rn)/rv*100);
else
    fprintf('TURNAROUND a_final 尚未生成\n');
end
fprintf('VERIFY_ALL_DONE\n');
