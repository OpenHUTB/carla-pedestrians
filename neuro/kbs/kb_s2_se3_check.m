function kb_s2_se3_check(inDir)
% §4.5 闭环基准复算: anchored100 口径 (前 100 帧拟合 Sim(3), 整条套用)
%
% 口径说明 (2026-11 修正):
%   论文 §4.5 报告的 842.54 m / 55.92 m 是 **终点误差 (terminal error)**, 即
%   锚定对齐后最后一帧的位置偏差; 早期版本的本函数只返回 RMSE
%   (sqrt(mean(err^2))), 会打印 477.76 / 87.25, 与论文值对不上。现在两种都打印。
%
% 期望值 (2026-11 重采批次: Town01 折返, 10000 帧 / 500 s / 881.5 m / 36 个接受事件):
%   exp_trajectory (A+S2, 部署) : terminal = 55.92 m    RMSE = 87.25 m    <- 论文报告值
%   fusion_pose    (EKF 核)     : terminal = 842.54 m   RMSE = 477.76 m   <- 论文报告值
%   preS2          (仅 A, 无 S2): terminal = 565.31 m   RMSE = 331.66 m   (论文未报告)
%   比值 842.54 / 55.92 = 15.07  ("15x")
%   scale2d (纯长度尺度 2D)     : 仅作诊断, 论文未报告
%
% 输入: export_closed_loop_inputs.py 生成的 5 个 txt (默认 /tmp/se3_*.txt)
%   se3_gt.txt se3_pres2.txt se3_final.txt se3_ekf.txt se3_events.txt
% 用法:
%   kb_s2_se3_check              % 读 /tmp
%   kb_s2_se3_check('/path/dir') % 读指定目录
%
% 注: reanchor_se3 变体 (SE(3) 无尺度) 论文未报告期望值, 只作诊断打印。

if nargin < 1 || isempty(inDir)
    inDir = '/tmp';
end
inDir = char(inDir);

gt     = csvread(fullfile(inDir, 'se3_gt.txt'));
preS2  = csvread(fullfile(inDir, 'se3_pres2.txt'));
finalT = csvread(fullfile(inDir, 'se3_final.txt'));
ekf    = csvread(fullfile(inDir, 'se3_ekf.txt'));
evs    = csvread(fullfile(inDir, 'se3_events.txt'));

L = sum(sqrt(sum(diff(gt, 1, 1).^2, 2)));
fprintf('input    : %s\n', inDir);
fprintf('sizes    : gt=%dx%d pre=%dx%d final=%dx%d ekf=%dx%d events=%dx%d\n', ...
        size(gt,1),size(gt,2), size(preS2,1),size(preS2,2), size(finalT,1),size(finalT,2), ...
        size(ekf,1),size(ekf,2), size(evs,1),size(evs,2));
fprintf('gt length: %.1f m   frames: %d   events: %d\n', L, size(gt,1), size(evs,1));

ok = true;
ok = report('preS2 (A only) ', preS2,  gt, NaN,    NaN)   && ok;
ok = report('S2    (A+loop)', finalT, gt, 55.92,  87.25)  && ok;
ok = report('EKF   (core)  ', ekf,    gt, 842.54, 477.76) && ok;

fprintf('ratio EKF/S2 terminal : %.2fx   (paper: 15x)\n', ...
        anchored100_terminal(ekf, gt) / anchored100_terminal(finalT, gt));
fprintf('scale2d rmse  S2/EKF : %.2f / %.2f  (diagnostic only)\n', ...
        scale2d_rmse(finalT, gt), scale2d_rmse(ekf, gt));

[se3, napp] = reanchor_se3(preS2, evs, 250);
fprintf('SE(3) variant        : terminal=%.2f rmse=%.2f n_app=%d/%d  (no paper reference)\n', ...
        anchored100_terminal(se3, gt), anchored100(se3, gt), napp, size(evs,1));

if ok
    fprintf('\nRESULT: PASS  (all paper-reported numbers reproduced within 0.5 m)\n');
else
    fprintf('\nRESULT: FAIL  (see mismatch deltas above)\n');
end
end

% ────────────────────────────────────────────────────────────────────────
function ok = report(tag, est, gt, expTerm, expRmse)
    term = anchored100_terminal(est, gt);
    rmse = anchored100(est, gt);
    if isnan(expTerm)
        fprintf('%s : terminal=%9.2f  rmse=%9.2f  (no reference)\n', tag, term, rmse);
        ok = true;
        return;
    end
    dt = term - expTerm;
    dr = rmse - expRmse;
    pass = abs(dt) <= 0.5 && abs(dr) <= 0.5;
    ok = pass;
    if pass, verdict = 'PASS'; else, verdict = 'FAIL'; end
    fprintf('%s : terminal=%9.2f (exp %7.2f, d=%+6.2f)  rmse=%9.2f (exp %7.2f, d=%+6.2f)  %s\n', ...
            tag, term, expTerm, dt, rmse, expRmse, dr, verdict);
end

function [ea, T, s] = anchored_fit(est, gt, k)
% 前 k 帧拟合 Sim(3) (Umeyama), 整套变换套用到整条轨迹
    n = min(size(est,1), size(gt,1)); est = est(1:n,:); gt = gt(1:n,:);
    e0 = est(1:k,:); g0 = gt(1:k,:);
    e0c = e0 - mean(e0,1); g0c = g0 - mean(g0,1);
    [U,S,Vt] = svd(g0c' * e0c, 'econ');
    d = det(Vt' * U');
    T = Vt' * diag([1 1 d]) * U';
    s = sum(S * [1; 1; d]) / sum(e0c(:).^2);
    tvec = mean(g0,1) - s * (mean(e0,1) * T);
    ea = s * (est * T) + tvec;
end

function rm = anchored100(est, gt)
% anchored100 后的整条 RMSE (m) — 论文不报此量, 保留供诊断
    ea = anchored_fit(est, gt, 100);
    g  = gt(1:size(ea,1),:);
    rm = sqrt(mean(sum((ea - g).^2, 2)));
end

function te = anchored100_terminal(est, gt)
% anchored100 后的 **终点误差** (m) — 论文 §4.5 的报告量
    ea = anchored_fit(est, gt, 100);
    g  = gt(1:size(ea,1),:);
    te = sqrt(sum((ea(end,:) - g(end,:)).^2, 2));
end

function rm = scale2d_rmse(est, gt)
    n = min(size(est,1), size(gt,1)); est = est(1:n,:); gt = gt(1:n,:);
    e = est - est(1,:); g = gt - gt(1,:);
    Le = sum(sqrt(sum(diff(e,1,1).^2, 2)));
    Lg = sum(sqrt(sum(diff(g,1,1).^2, 2)));
    s = Lg / Le;
    ea = e * s;
    rm = sqrt(mean(sum((ea(:,1:2) - g(:,1:2)).^2, 2)));
end

function [traj, napp] = reanchor_se3(traj, events, win)
    % nlm_sim3_reanchor 的 SE(3) 变体: 仅 'Scaling', false, 无尺度门
    n = size(traj,1); base = traj;
    d0 = sqrt(sum((base - base(1,:)).^2, 2));
    sra = find(d0 > 1, 1, 'first');
    if isempty(sra), sra = 1; end
    aWin = base(sra:min(sra+win-1, n), :);
    ca = mean(aWin, 1);
    evs = sortrows(events, 1);
    napp = 0;
    for k = 1:size(evs,1)
        t_now = evs(k,1);
        if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
        cWin = base(max(t_now-win+1, 1):min(t_now, t_next-1), :);
        seg  = (t_now+1):(t_next-1);
        if numel(cWin) < 30 || numel(seg) < 2 || size(aWin,1) ~= size(cWin,1)
            continue;
        end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', false);
        Rmat = T.T;
        cc = mean(cWin, 1);
        tvec = ca - (cc * Rmat);
        traj(seg,:) = (base(seg,:) * Rmat) + tvec;
        napp = napp + 1;
    end
end
