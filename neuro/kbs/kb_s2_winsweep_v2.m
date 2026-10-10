% kb_s2_winsweep_v2.m — Turnaround (10-09 代, 10 事件) S2 拟合窗扫参
% 复用 MATLAB 内置 procrustes (与 core nlm_sim3_reanchor 同调用同约定),
% 逐行复刻 A-fix 爬坡注入 + 非链式 S2, 保证与落盘 a_final 同口径.
% 自校准: win=400 必须复现 a_final 的 anchored100=113.11 / scaleLocked2D=58.65,
% 否则整表作废 (FAIL 退出).
% 运行: matlab -batch "cd('/home/yangrb/openhutb/neuro/kbs'); kb_s2_winsweep_v2"
clear all;
BASE = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt     = csvread(fullfile(BASE, 'a_final', 'ground_truth_backup.txt'));
pres2  = csvread(fullfile(BASE, 'a_final', 'exp_trajectory_preS2.txt'));
a_disk = csvread(fullfile(BASE, 'a_final', 'exp_trajectory.txt'));
M  = load(fullfile(BASE, 'a_final', 'loop_constraint_events.mat'));
ev = M.NLM_LOOP_EVENTS;
if size(ev, 1) == 5 && size(ev, 2) > 5, ev = ev'; end
assert(size(ev, 2) == 5, 'events shape');
n = size(gt, 1);
disp(sprintf('n=%d  events=%d  gap[min,max]=[%d,%d]  residMax=%.2fm', ...
    n, size(ev,1), min(ev(:,1)-ev(:,2)), max(ev(:,1)-ev(:,2)), ...
    max(sqrt(sum(ev(:,3:5).^2,2)))));

% S2 输入 = preS2 落盘轨迹 (core L675 保存, 已含 A-fix 爬坡, 勿重复注入)
base = pres2;

% ---------- 非链式 S2 (逐行复刻 nlm_sim3_reanchor L985-1030) ----------
d0  = sqrt(sum((base - base(1,:)).^2, 2));
sra = find(d0 > 1, 1, 'first');
if isempty(sra), sra = 1; end
evs = sortrows(ev, 1);

wins = [100 150 200 250 300 350 400 450 500 600 700 800 1000 1200];
printf = @() 0; % no-op
% 表头
disp(sprintf('  %6s %10s %12s %10s  %s', 'win', 'anch100', 'scale2D', 's2_app', 'verdict'));
best_w = 400; best_anch = inf;
ok_calib = false;
for w = wins
    aWin = base(sra:min(sra+w-1, n), :);
    ca   = mean(aWin, 1);
    out  = base;
    n_app = 0;
    for k = 1:size(evs,1)
        t_now = evs(k,1);
        if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
        cWin = base(max(t_now-w+1,1) : min(t_now, t_next-1), :);
        seg  = (t_now+1) : (t_next-1);
        if numel(cWin) < 30 || numel(seg) < 2 || size(aWin,1) ~= size(cWin,1)
            continue
        end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
        if T.b < 0.05 || T.b > 20
            continue
        end
        Rmat = T.T;
        cc = mean(cWin, 1);
        tvec = ca - T.b * (cc * Rmat);
        out(seg,:) = T.b * (base(seg,:) * Rmat) + tvec;
        n_app = n_app + 1;
    end
    a100 = metric_anch100(out, gt);
    s2d  = metric_scale2d(out, gt);
    tag  = '';
    if w == 400
        dev = max(abs(out - a_disk));
        ok_calib = (dev < 0.05);
        tag = sprintf('  <== calib: vsDisk maxDelta=%.4f (%s)', ...
            dev, ternary(ok_calib, 'OK', 'FAIL'));
    end
    disp(sprintf('  %6d %10.2f %12.2f %9d/%d%s', w, a100, s2d, n_app, size(evs,1), tag));
    if a100 < best_anch, best_anch = a100; best_w = w; end
end

disp('─────────────────────────────────────────────────────────');
if ~ok_calib
    disp('!! 校准 FAIL: win=400 未复现 a_final, 本表作废');
    exit(1);
end
disp(sprintf('自校准 OK (win=400 与 a_final 逐帧一致)'));
disp(sprintf('best win = %d   anchored100 = %.2f m (基准400窗值见上表)', best_w, best_anch));

% ---------- 辅助口径 ----------
% 与 compute_metrics_with_alignment.m 逐式一致 (前100帧 Procrustes, 全轨迹检验)
function r = metric_anch100(E, G)
n = min(size(E,1), size(G,1)); E = E(1:n,:); G = G(1:n,:);
af = min(100, n);
[~, ~, transform] = procrustes(G(1:af,:), E(1:af,:), 'Scaling', true);
Ea = transform.b * E * transform.T + repmat(transform.c(1,:), n, 1);
err = sqrt(sum((Ea - G).^2, 2));
r = sqrt(mean(err.^2));
end

% 与 compute_final_metrics.py scale_locked_2d 逐式一致 (起点+总长匹配, 无旋转, 2D)
function r = metric_scale2d(E, G)
n = min(size(E,1), size(G,1)); E = E(1:n,:); G = G(1:n,:);
e = E - E(1,:); g = G - G(1,:);
Le = sum(sqrt(sum(diff(e).^2, 2))); Lg = sum(sqrt(sum(diff(g).^2, 2)));
s = Lg / Le;
d = e*s - g;
r = sqrt(mean(sum(d(:,1:2).^2, 2)));
end

function s = ternary(c, a, b)
s = a; if ~c, s = b; end
end
