% verify_table1.m — 锁定论文 Table 1 的5个室外集 Sim(3) 数字(仓库口径)
% 打印: 各文件行数 / compute_metrics_with_alignment(NLM,EKF,VO) 的 RMSE+终点误差+漂移 /
%       函数内部 Procrustes 尺度 / MATLAB内手动Umeyama(用于对照Python)
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
    fprintf('%s  rows: gt=%d nlm=%d ekf=%d vo=%d  L=%.0fm\n', ds, size(gt,1), size(nlm,1), size(ekf,1), size(vo,1), L);
    [rn,fen,dn] = compute_metrics_with_alignment(nlm, gt, L);
    [re,fee,de] = compute_metrics_with_alignment(ekf, gt, L);
    [rv,fev,dv] = compute_metrics_with_alignment(vo,  gt, L);
    fprintf('  func(NLM,EKF,VO)=%.2f(t=%.2f,%.2f%%) | %.2f(%.2f%%) | %.2f(%.2f%%)\n', ...
        rn,fen,dn, re,de, rv,dv);
    % 手动 Umeyama 全轨迹 Sim(3) 对照
    [sN,rn2] = umeyama_rmse(nlm, gt);
    [sE,re2] = umeyama_rmse(ekf, gt);
    fprintf('  manualUmeyama: NLM=%.2f(s=%.3f) EKF=%.2f(s=%.3f)\n', rn2, sN, re2, sE);
end
fprintf('VERIFY_TABLE1_DONE\n');

function [s, rmse] = umeyama_rmse(src, dst)
    n = min(size(src,1), size(dst,1));
    src = src(1:n,1:3); dst = dst(1:n,1:3);
    ms = mean(src); md = mean(dst);
    sc = src - ms; dc = dst - md;
    H = dc.' * sc;
    [U,S,V] = svd(H);
    D = eye(3); D(3,3) = sign(det(V.'*U.'));
    R = V.' * D * U.';
    s = (S(1,1) + S(2,2) + D(3,3)*S(3,3)) / sum(sc(:).^2);
    t = md - s * (ms * R.');
    al = s * (src * R.') + t;
    e = sqrt(sum((al - dst).^2, 2));
    rmse = sqrt(mean(e.^2));
end
% (手动Umeyama对照部分省略——仓库口径以 compute_metrics_with_alignment 为准)
