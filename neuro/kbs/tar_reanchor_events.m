% tar_reanchor_events.m — 用【真实16个闭环事件】(loop_constraint_events.mat) 复验全Sim(3)重锚
% 与 tar_global_reanchor.m(4个lap边界)的区别: 事件集 = 系统真实记录的 NLM_LOOP_EVENTS
% 目的: 确认 core 里用全部事件实现时机制仍成立 (事件间距323-1154帧, win=400)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt    = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
adisk = csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
ekf   = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
L = sum(sqrt(sum(diff(gt).^2,2)));
load(fullfile(tar,'a_final','loop_constraint_events.mat'), 'NLM_LOOP_EVENTS');
win = 400;
n = size(adisk,1);

% 锚点窗口 = 首移动段 (自动跳过静止前导段)
d0 = sqrt(sum((adisk - adisk(1,:)).^2,2));
sra = find(d0 > 1, 1, 'first'); if isempty(sra), sra = 1; end
aWin = adisk(sra : min(sra+win-1, n), :);
fprintf('事件数=%d | 首移动帧=%d | 锚点窗口=[%d:%d]\n', size(NLM_LOOP_EVENTS,1), sra, sra, sra+win-1);

evs = sortrows(NLM_LOOP_EVENTS, 1);
corr = adisk;
n_app = 0;
for k = 1:size(evs,1)
    t_now = evs(k,1);
    if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
    cWin = adisk(max(t_now-win+1,1) : min(t_now, t_next-1), :);
    seg  = (t_now+1) : (t_next-1);
    if numel(cWin) < 50 || numel(seg) < 2
        fprintf('  ev%d t_now=%5d: 跳过(窗口退化)\n', k, t_now); continue;
    end
    [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
    if T.b < 0.05 || T.b > 20
        fprintf('  ev%d t_now=%5d: 跳过(scale=%.3f 退化)\n', k, t_now, T.b); continue;
    end
    corr(seg,:) = T.b * adisk(seg,:) * T.T + repmat(T.c(1,:), numel(seg),1);
    n_app = n_app + 1;
    fprintf('  ev%d t_now=%5d t_old=%5d: scale=%.3f |c|=%8.2f seg=[%d:%d]\n', ...
        k, t_now, evs(k,2), T.b, norm(T.c), seg(1), seg(end));
end

fprintf('\n应用 %d/%d 事件\n\n', n_app, size(evs,1));

% --- 两种论文口径 ---
fprintf('=== 版本A (首100帧Sim3, ablation Table口径) ===\n');
fprintf('  a_disk现状(A-fix平移)     = %8.2f m\n', compute_metrics_with_alignment(adisk, gt, L));
fprintf('  a_disk + 全Sim(3)重锚(16ev)= %8.2f m\n', compute_metrics_with_alignment(corr, gt, L));
fprintf('  EKF 基线                  = %8.2f m\n', compute_metrics_with_alignment(ekf, gt, L));

fprintf('\n=== simple (core: 起点+长度匹配, 无旋转) ===\n');
for nm = {'a_disk现状(A-fix平移)',adisk; 'a_disk+全Sim(3)重锚(16ev)',corr; 'EKF',ekf}
    tr = nm{2};
    c = tr - tr(1,:); g = gt - gt(1,:);
    l1 = sum(sqrt(sum(diff(c).^2,2))); l2 = sum(sqrt(sum(diff(g).^2,2)));
    s = l2/l1;
    fprintf('  %-22s ATE = %8.2f m (scale=%.4f)\n', nm{1}, sqrt(mean(sum((c*s-g).^2,2))), s);
end
fprintf('REANCHOR_EVENTS_DONE\n');
