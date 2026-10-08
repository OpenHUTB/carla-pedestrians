% tar_s2_tune.m — S2 参数调优 (win 扫描) + core全量输出口径核验 + 中性性结构确认
% 输入 = S2 前的轨迹 (a_final 备份, 含 raw+DC+A-fix)
% 1) win ∈ {200,300,400,500,600} 非链式S2: 版本A + simple + per-lap oracle
% 2) 当前盘上 core 输出 (win=400) 的版本A 口径 + per-lap oracle (论文口径核验)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt0   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
preS2 = csvread('/tmp/turnaround_a_final_preS2_20261006/exp_trajectory.txt');
core400 = csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
ekf   = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
n = min([size(gt0,1), size(preS2,1), size(core400,1), size(ekf,1)]);
gt = gt0(1:n,:); preS2 = preS2(1:n,:); core400 = core400(1:n,:); ekf = ekf(1:n,:);
L = sum(sqrt(sum(diff(gt).^2,2)));
load(fullfile(tar,'a_final','loop_constraint_events.mat'), 'NLM_LOOP_EVENTS');
bounds = [59;2386;4714;7037;min(9365,n)];

fprintf('=== 1) win 扫描 (非链式, 与core一致) ===\n');
for win = [200 300 400 500 600]
    traj = s2_apply(preS2, NLM_LOOP_EVENTS, win);
    va = compute_metrics_with_alignment(traj, gt, L);
    c = traj - traj(1,:); g = gt - gt(1,:);
    s = L/sum(sqrt(sum(diff(c).^2,2)));
    vs = sqrt(mean(sum((c*s-g).^2,2)));
    pl = zeros(numel(bounds)-1,1);
    for j = 1:numel(bounds)-1
        a = bounds(j)+1; b = bounds(j+1);
        pl(j) = compute_metrics_with_alignment(traj(a:b,:), gt(a:b,:), sum(sqrt(sum(diff(gt(a:b,:)).^2,2))));
    end
    fprintf('win=%4d  版本A=%8.2f m  simple=%7.2f m  plap=[%s]\n', win, va, vs, mat2str(pl,3));
end

fprintf('\n=== 2) core全量输出 (盘上 a_final, win=400) 核验 ===\n');
va = compute_metrics_with_alignment(core400, gt, L);
c = core400 - core400(1,:); g = gt - gt(1,:);
s = L/sum(sqrt(sum(diff(c).^2,2)));
vs = sqrt(mean(sum((c*s-g).^2,2)));
pl = zeros(numel(bounds)-1,1);
for j = 1:numel(bounds)-1
    a = bounds(j)+1; b = bounds(j+1);
    pl(j) = compute_metrics_with_alignment(core400(a:b,:), gt(a:b,:), sum(sqrt(sum(diff(gt(a:b,:)).^2,2))));
end
fprintf('core400  版本A=%8.2f m  simple=%7.2f m  plap=[%s]\n', va, vs, mat2str(pl,3));
c = preS2 - preS2(1,:);
fprintf('preS2    版本A=%8.2f m  simple=%7.2f m\n', ...
    compute_metrics_with_alignment(preS2, gt, L), ...
    sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2))));
c = ekf - ekf(1,:);
fprintf('EKF      版本A=%8.2f m  simple=%7.2f m\n', ...
    compute_metrics_with_alignment(ekf, gt, L), ...
    sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2))));
fprintf('S2_TUNE_DONE\n');

function traj = s2_apply(base0, events, win)
    base = base0; n = size(base,1);
    d0 = sqrt(sum((base - base(1,:)).^2,2));
    sra = find(d0 > 1, 1, 'first'); if isempty(sra), sra = 1; end
    aWin = base(sra : min(sra+win-1, n), :); ca = mean(aWin,1);
    evs = sortrows(events, 1);
    for k = 1:size(evs,1)
        t_now = evs(k,1);
        if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
        cWin = base(max(t_now-win+1,1) : min(t_now, t_next-1), :);
        seg  = (t_now+1) : (t_next-1);
        if numel(cWin) < 30 || numel(seg) < 2, continue; end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
        if T.b < 0.05 || T.b > 20, continue; end
        cc = mean(cWin,1);
        tvec = ca - T.b*(cc*T.T);
        base(seg,:) = T.b*(base(seg,:)*T.T) + tvec;
    end
    traj = base;
end
