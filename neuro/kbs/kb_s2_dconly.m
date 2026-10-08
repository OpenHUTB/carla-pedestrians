% kb_s2_dconly.m — 复现 "仅DC(无A-fix) verA = 755m" 基线, 验证论文引用数字
% DC-only = preS2 - L_field (L_field 按 core 公式从事件重算, 确定性)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
ad = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion/a_final';
gt0 = csvread(fullfile(ad,'ground_truth_backup.txt'));
preS2 = csvread(fullfile(ad,'exp_trajectory_preS2.txt'));
ekf = csvread(fullfile(ad,'imu_aided_trajectory.txt'));
d = load(fullfile(ad,'loop_constraint_events.mat'));
evs = d.NLM_LOOP_EVENTS;
n = min([size(gt0,1), size(preS2,1), size(ekf,1)]);
gt = gt0(1:n,:); preS2 = preS2(1:n,:); ekf = ekf(1:n,:);
L = sum(sqrt(sum(diff(gt).^2,2)));

% 按 core A-fix 公式重算 L_field
f0v = (1:n)';
L_field = zeros(n,3);
for k = 1:size(evs,1)
    t_now = evs(k,1); t_old = evs(k,2); c = evs(k,3:5);
    if t_old >= t_now || t_old < 1
        if t_now > 1, L_field(t_now-1,:) = L_field(t_now-1,:) + c; end
        continue;
    end
    seg = f0v >= t_old & f0v < t_now;
    if any(seg)
        t = f0v(seg) - t_old;
        F = 0.5*(1 - cos(pi*t/(t_now - t_old)));
        L_field(seg,:) = L_field(seg,:) + F(:)*c;
    end
end
dconly = preS2 - L_field;

va_dc = compute_metrics_with_alignment(dconly, gt, L);
va_af = compute_metrics_with_alignment(preS2, gt, L);
fprintf('仅DC(无A-fix)    版本A = %8.2f m  (论文引用755)\n', va_dc);
fprintf('DC+A-fix (preS2) 版本A = %8.2f m  (论文861.48)\n', va_af);
fprintf('EKF              版本A = %8.2f m\n', compute_metrics_with_alignment(ekf, gt, L));
fprintf('KB_S2_DCONLY_DONE\n');
