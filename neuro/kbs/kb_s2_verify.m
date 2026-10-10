% kb_s2_verify.m — win=250 vs 400 防伪验证 (per-lap oracle + 事件生效集)
% 判据 (NLM_LOOP_DESIGN.md §9.3): S2 增益必须纯来自全局 Sim(3) 移除,
%   per-lap 独立对齐 ATE 在 S2off vs S2on 间应基本不变 (无局部畸变/无GT泄漏).
% 运行: matlab -batch "cd('/home/yangrb/openhutb/neuro/kbs'); kb_s2_verify"
clear all;
BASE = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt     = csvread(fullfile(BASE, 'a_final', 'ground_truth_backup.txt'));
pres2  = csvread(fullfile(BASE, 'a_final', 'exp_trajectory_preS2.txt'));
a_disk = csvread(fullfile(BASE, 'a_final', 'exp_trajectory.txt'));
M  = load(fullfile(BASE, 'a_final', 'loop_constraint_events.mat'));
ev = M.NLM_LOOP_EVENTS;
if size(ev, 1) == 5 && size(ev, 2) > 5, ev = ev'; end
n = size(gt, 1);
evs = sortrows(ev, 1);
base = pres2;   % S2 输入 (已含 A-fix 爬坡)
d0  = sqrt(sum((base - base(1,:)).^2, 2));
sra = find(d0 > 1, 1, 'first');
if isempty(sra), sra = 1; end

disp(sprintf('n=%d events=%d  锚点窗起点 sra=%d', n, size(evs,1), sra));

% --- 事件逐事件拟合尺度 (win=400 vs 250) ---
fprintf('  %3s %7s %6s %14s %14s\n', 'k','t_now','seg_len','s@400','s@250');
for k = 1:size(evs,1)
    t_now = evs(k,1);
    if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
    seglen = t_next - t_now - 1;
    line = sprintf('  %3d %7d %6d', k, t_now, seglen);
    for w = [400 250]
        cWin = base(max(t_now-w+1,1) : min(t_now, t_next-1), :);
        aWin = base(sra:min(sra+w-1, n), :);
        if numel(cWin) < 30 || seglen < 2 || size(aWin,1) ~= size(cWin,1)
            line = [line sprintf(' %14s', '-')];
        else
            [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
            if T.b < 0.05 || T.b > 20
                line = [line sprintf(' %10.3f(REJ)', T.b)];
            else
                line = [line sprintf(' %10.4f(ok)', T.b)];
            end
        end
    end
    disp(line);
end

% --- S2 输出 ---
out400 = s2run(base, evs, sra, 400, n);
out250 = s2run(base, evs, sra, 250, n);

% --- per-lap oracle ---
bounds = [1; lap_bounds_idx(gt); n];
variants = {'S2off(pres2)','S2 w=400','S2 w=250','EKF(a_disk)'};
trajs = {pres2, out400, out250, a_disk};
fprintf('\n--- per-lap oracle (每lap独立full-Sim3, 防伪: S2off vs S2on 应不变) ---\n');
hdr = '  %-16s';
for v = 1:numel(variants), hdr = [hdr sprintf('  %12s', variants{v})]; end
disp(hdr);
for i = 1:length(bounds)-1
    a_ = bounds(i); b_ = bounds(i+1);
    fprintf('  %-16s', sprintf('lap%d[%d:%d]', i, a_, b_-a_));
    for v = 1:numel(variants)
        E = trajs{v}(a_:b_, :); G = gt(a_:b_, :);
        fprintf('  %12.2f', lap_ate(E, G));
    end
    disp('');
end
disp('');
disp('若 S2off 与 S2 w=250 各 lap 值接近(±1m内) → 增益纯全局, 通过防伪.');

function out = s2run(base, evs, sra, win, n)
    aWin = base(sra:min(sra+win-1, n), :);
    ca   = mean(aWin, 1);
    out  = base;
    for k = 1:size(evs,1)
        t_now = evs(k,1);
        if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
        cWin = base(max(t_now-win+1,1) : min(t_now, t_next-1), :);
        seg  = (t_now+1) : (t_next-1);
        if numel(cWin) < 30 || numel(seg) < 2 || size(aWin,1) ~= size(cWin,1)
            continue
        end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
        if T.b < 0.05 || T.b > 20, continue, end
        cc = mean(cWin, 1);
        tvec = ca - T.b * (cc * T.T);
        out(seg,:) = T.b * (base(seg,:) * T.T) + tvec;
    end
end

function b = lap_bounds_idx(G)
    d = sqrt(sum((G - G(1,:)).^2, 2));
    in_s = d < 8;
    b = [];
    for i = 2:length(d)
        if in_s(i-1) && ~in_s(i), b = [b; i]; end
    end
end

function r = lap_ate(E, G)
    nn = min(size(E,1), size(G,1)); E = E(1:nn,:); G = G(1:nn,:);
    if nn < 10, r = NaN; return; end
    [~, ~, T] = procrustes(G, E, 'Scaling', true);
    Ea = T.b * E * T.T + repmat(T.c(1,:), nn, 1);
    err = sqrt(sum((Ea - G).^2, 2));
    r = sqrt(mean(err.^2));
end
