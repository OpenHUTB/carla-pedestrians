% kb_baseline_at.m — 各数据集现有 a_final (S2关) 的 ATE 基线, 两种口径
root = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
datasets = {'Town01Turnaround_IMU_Fusion','Town05Data_IMU_Fusion','KITTI07Data_IMU_Fusion', ...
            'Town01Data_IMU_Fusion','Town02Data_IMU_Fusion','Town10HDData_IMU_Fusion'};
for i = 1:numel(datasets)
    ds = datasets{i};
    gt0 = csvread(fullfile(root, ds, 'a_final', 'ground_truth_backup.txt'));
    nl  = csvread(fullfile(root, ds, 'a_final', 'exp_trajectory.txt'));
    ekf = csvread(fullfile(root, ds, 'a_final', 'imu_aided_trajectory.txt'));
    n = min([size(gt0,1), size(nl,1), size(ekf,1)]);
    gt = gt0(1:n,:); nl = nl(1:n,:); ekf = ekf(1:n,:);
    L = sum(sqrt(sum(diff(gt).^2,2)));
    % 版本A: 首100帧锚定 Sim(3) 3D
    va_n = compute_metrics_with_alignment(nl, gt, L);
    va_e = compute_metrics_with_alignment(ekf, gt, L);
    % simple: 起点+长度匹配, 无旋转
    c = nl - nl(1,:); g = gt - gt(1,:);
    s = L/sum(sqrt(sum(diff(c).^2,2)));
    vs_n = sqrt(mean(sum((c*s-g).^2,2)));
    c = ekf - ekf(1,:);
    vs_e = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
    fprintf('%-30s  NLM verA=%8.2f simple=%7.2f | EKF verA=%8.2f simple=%7.2f\n', ...
        ds, va_n, vs_n, va_e, vs_e);
end
fprintf('KB_BASELINE_AT_DONE\n');
