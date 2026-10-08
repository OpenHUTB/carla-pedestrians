% kb_s2_winsweep.m — win 扫描 (当前代匹配pair, 与 core nlm_sim3_reanchor 逻辑逐条一致)
% 输入: a_final/exp_trajectory_preS2.txt (S2输入) + a_final/loop_constraint_events.mat (同代)
% 逻辑与 core 完全一致: 非链式; 跳过条件 numel(cWin)<30 || numel(seg)<2; 尺度门 [0.05,20]
% 无行数门 (core 无行数门; 之前 kb_s2_tune2 加行数门是缺陷, 7事件为假象)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
ad = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion/a_final';
gt0 = csvread(fullfile(ad,'ground_truth_backup.txt'));
base0 = csvread(fullfile(ad,'exp_trajectory_preS2.txt'));
ekf = csvread(fullfile(ad,'imu_aided_trajectory.txt'));
d = load(fullfile(ad,'loop_constraint_events.mat'));
evs0 = d.NLM_LOOP_EVENTS;
n = min([size(gt0,1), size(base0,1), size(ekf,1)]);
gt = gt0(1:n,:); base0 = base0(1:n,:); ekf = ekf(1:n,:);
L = sum(sqrt(sum(diff(gt).^2,2)));

% S2-off 基线 (= preS2 本身)
va0 = compute_metrics_with_alignment(base0, gt, L);
c = base0 - base0(1,:); g = gt - gt(1,:);
vs0 = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
fprintf('S2off (preS2)     版本A=%8.2f m  simple=%7.2f m\n', va0, vs0);
c = ekf - ekf(1,:);
fprintf('EKF               版本A=%8.2f m  simple=%7.2f m\n', ...
    compute_metrics_with_alignment(ekf, gt, L), sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2))));

for win = [200 300 400 500 600 800]
    [traj, napp] = s2_core(base0, evs0, win);
    va = compute_metrics_with_alignment(traj, gt, L);
    c = traj - traj(1,:);
    vs = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
    fprintf('win=%4d  版本A=%8.2f m  simple=%7.2f m  (生效%d事件)\n', win, va, vs, napp);
end
fprintf('KB_S2_WINSWEEP_DONE\n');

function [traj, napp] = s2_core(base0, events, win)
    % 与 core nlm_sim3_reanchor 逐条一致 (非链式: base 只读, 写入 traj)
    traj = base0; base = base0; n = size(base,1); napp = 0;
    d0 = sqrt(sum((base - base(1,:)).^2,2));
    sra = find(d0 > 1, 1, 'first'); if isempty(sra), sra = 1; end
    aWin = base(sra : min(sra+win-1, n), :); ca = mean(aWin,1);
    evs = sortrows(events, 1);
    for k = 1:size(evs,1)
        t_now = evs(k,1);
        if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
        cWin = base(max(t_now-win+1,1) : min(t_now, t_next-1), :);
        seg  = (t_now+1) : (t_next-1);
        if numel(cWin) < 30 || numel(seg) < 2 || size(aWin,1) ~= size(cWin,1), continue; end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
        if T.b < 0.05 || T.b > 20, continue; end
        cc = mean(cWin,1);
        tvec = ca - T.b*(cc*T.T);
        traj(seg,:) = T.b*(base(seg,:)*T.T) + tvec;
        napp = napp + 1;
    end
end
