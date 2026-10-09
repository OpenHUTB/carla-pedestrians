% kb_s2_tune2.m — (A) win扫描(Turnaround matched backup pair, 非链式+行数门)
%                (B) 开放集 S2 中性确认: on-disk a_final(S2off) → 离线S2on, before/after
% 关键修正: 之前 win 扫描误用"旧轨迹+新事件"错配pair → 现统一用同一次run的匹配pair。
clear all; close all; clc;
root = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
bak  = '/tmp/turnaround_a_final_preS2_20261006';
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');

fprintf('=== A) win 扫描 (Turnaround matched backup pair: 旧轨迹861.48 + 旧事件, 非链式) ===\n');
gt0   = csvread(fullfile(bak,'ground_truth_backup.txt'));
traj0 = csvread(fullfile(bak,'exp_trajectory.txt'));
d = load(fullfile(bak,'loop_constraint_events.mat'));
evs0 = d.NLM_LOOP_EVENTS;
n = min([size(gt0,1), size(traj0,1)]);
gt = gt0(1:n,:); traj0 = traj0(1:n,:);
L = sum(sqrt(sum(diff(gt).^2,2)));
for win = [200 300 400 500]
    [traj, napp] = s2_apply(traj0, evs0, win);
    va = compute_metrics_with_alignment(traj, gt, L);
    c = traj - traj(1,:); g = gt - gt(1,:);
    vs = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
    fprintf('win=%4d  版本A=%8.2f m  simple=%7.2f m  (生效%d事件)\n', win, va, vs, napp);
end

fprintf('\n=== B) 开放集 S2 中性确认 (on-disk a_final = S2off 基线 → 离线 S2on, win=400) ===\n');
datasets = {'Town05Data_IMU_Fusion','KITTI07Data_IMU_Fusion'};
for i = 1:numel(datasets)
    ds = datasets{i};
    gt0 = csvread(fullfile(root, ds, 'a_final', 'ground_truth_backup.txt'));
    nl0 = csvread(fullfile(root, ds, 'a_final', 'exp_trajectory.txt'));
    ef  = load(fullfile(root, ds, 'a_final', 'loop_constraint_events.mat'));
    evs = ef.NLM_LOOP_EVENTS;
    n = min([size(gt0,1), size(nl0,1)]);
    gt = gt0(1:n,:); nl0 = nl0(1:n,:);
    L = sum(sqrt(sum(diff(gt).^2,2)));
    va_off = compute_metrics_with_alignment(nl0, gt, L);
    c = nl0 - nl0(1,:); g = gt - gt(1,:);
    vs_off = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
    [nl_on, napp] = s2_apply(nl0, evs, 400);
    va_on = compute_metrics_with_alignment(nl_on, gt, L);
    c = nl_on - nl_on(1,:);
    vs_on = sqrt(mean(sum((c*(L/sum(sqrt(sum(diff(c).^2,2))))-g).^2,2)));
    fprintf('%-26s events=%2d 生效=%2d  S2off verA=%8.2f simple=%7.2f | S2on verA=%8.2f simple=%7.2f  Δsimple=%+7.2f\n', ...
        ds, size(evs,1), napp, va_off, vs_off, va_on, vs_on, vs_on-vs_off);
end
fprintf('\n判读: Δsimple≤0 → S2不反噬(中性或更优); Δsimple>0 → 该集反噬, 需限定claim或加门控.\n');
fprintf('KB_S2_TUNE2_DONE\n');

function [traj, napp] = s2_apply(base0, events, win)
    base = base0; n = size(base,1); napp = 0;
    if isempty(events), traj = base; return; end
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
        base(seg,:) = T.b*(base(seg,:)*T.T) + tvec;
        napp = napp + 1;
    end
    traj = base;
end
