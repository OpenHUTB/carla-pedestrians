% VERIFY_PAPER_METRICS.m — 用论文官方口径 (compute_metrics_with_alignment, 全轨迹Sim(3))
% 对恢复EKF后的 4 CARLA 集 × 3 方法做权威 ATE 计算 (Table 1 草稿核验用)
cd('/home/yangrb/openhutb');
addpath('neuro/07_test/test_imu_visual_slam/ablation');
sets = {'Town01Data_IMU_Fusion','Town02Data_IMU_Fusion','Town05Data_IMU_Fusion','Town10HDData_IMU_Fusion'};
for i = 1:numel(sets)
    d = sets{i};
    base = ['neuro/data/01_NeuroSLAM_Datasets/' d '/'];
    gt  = readmatrix([base 'ground_truth.txt']);  gt  = gt(:,2:4);
    fu  = readmatrix([base 'fusion_pose.txt']);   fu  = fu(:,2:4);
    vo  = readmatrix([base 'visual_odometry.txt']); vo = vo(:,2:4);
    m   = load([base 'slam_results/trajectories.mat']);
    nlm = m.exp_trajectory;
    gl  = sum(sqrt(sum(diff(gt).^2, 2)));
    [r1,f1,dr1,~] = compute_metrics_with_alignment(nlm, gt, gl);
    [r2,f2,dr2,~] = compute_metrics_with_alignment(fu,  gt, gl);
    [r3,f3,dr3,~] = compute_metrics_with_alignment(vo,  gt, gl);
    fprintf('%-24s GTlen=%.1f NLM=%7.2f (term %.2f, dr %.2f%%)  EKF=%7.2f (term %.2f, dr %.2f%%)  VO=%7.2f (term %.2f, dr %.2f%%)\n', ...
        d, gl, r1, f1, dr1, r2, f2, dr2, r3, f3, dr3);
end
disp('=== DONE ===');
