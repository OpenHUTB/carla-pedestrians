function kb_s2_se3_check()
% S2 的 SE(3) 变体 (procrustes 无尺度) 复算: anchored100 + scale2d
% 校验基准 (neuro/kbs/final_metrics.json turnaround 权威值):
%   preS2 (A-fix, 无S2)   : anchored100=323.48, scale2d=191.08
%   exp_trajectory (A+S2) : anchored100=44.24,  scale2d=46.63
%   fusion_pose (EKF)     : anchored100=398.05, scale2d=422.31
% 数据: 由 Python 预清洗到 /tmp/se3_*.txt (与 compute_final_metrics.py 同口径, 裁剪到 n10k)
gt = csvread('/tmp/se3_gt.txt');
preS2  = csvread('/tmp/se3_pres2.txt');
finalT = csvread('/tmp/se3_final.txt');
ekf = csvread('/tmp/se3_ekf.txt');
evs = csvread('/tmp/se3_events.txt');

fprintf('sizes: gt=%dx%d pre=%dx%d final=%dx%d ekf=%dx%d\n', ...
        size(gt,1),size(gt,2), size(preS2,1),size(preS2,2), size(finalT,1),size(finalT,2), size(ekf,1),size(ekf,2));

fprintf('validate preS2 : anch100=%.2f scale2d=%.2f  (expect 323.48/191.08)\n', ...
        anchored100(preS2, gt), scale2d_rmse(preS2, gt));
fprintf('validate S2    : anch100=%.2f scale2d=%.2f  (expect 44.24/46.63)\n', ...
        anchored100(finalT, gt), scale2d_rmse(finalT, gt));
fprintf('validate EKF   : anch100=%.2f scale2d=%.2f  (expect 398.05/422.31)\n', ...
        anchored100(ekf, gt), scale2d_rmse(ekf, gt));

[se3, napp] = reanchor_se3(preS2, evs, 250);
fprintf('SE3 variant    : anch100=%.2f scale2d=%.2f  n_app=%d/%d\n', ...
        anchored100(se3, gt), scale2d_rmse(se3, gt), napp, size(evs,1));
end

function rm = anchored100(est, gt)
    n = min(size(est,1), size(gt,1)); est = est(1:n,:); gt = gt(1:n,:);
    e0 = est(1:100,:); g0 = gt(1:100,:);
    e0c = e0 - mean(e0,1); g0c = g0 - mean(g0,1);
    [U,S,Vt] = svd(g0c' * e0c, 'econ');
    d = det(Vt' * U');
    T = Vt' * diag([1 1 d]) * U';
    s = sum(S * [1; 1; d]) / sum(e0c(:).^2);
    tvec = mean(g0,1) - s * (mean(e0,1) * T);
    ea = s * (est * T) + tvec;
    rm = sqrt(mean(sum((ea - gt).^2, 2)));
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
