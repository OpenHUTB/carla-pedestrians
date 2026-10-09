% tar_reanchor_variants.m — 16真实事件 全Sim(3)重锚: 3种DOF/稳健性变体 + per-lap oracle 防伪判据
% V1 裸Sim(3): 用拟合的 b,R,t
% V2 钳位尺度: b 钳到 [0.5,2]x 上一事件b (running先验, 防窗口退化尺度翻), t 用质心约束重算
% V3 SE(3): 只用 R,t, 强制 b=1 (检验尺度是否主导校正)
% 判据: 版本A + simple 是否稳定 < EKF(745); 各变体 per-lap oracle 是否保持 ≈ 基线(无局部畸变注入)
clear all; close all; clc;
addpath('/home/yangrb/openhutb/neuro/07_test/07_test/test_imu_visual_slam/ablation');
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
gt0   = csvread(fullfile(tar,'a_final','ground_truth_backup.txt'));
adisk = csvread(fullfile(tar,'a_final','exp_trajectory.txt'));
ekf   = csvread(fullfile(tar,'a_final','imu_aided_trajectory.txt'));
n = min([size(gt0,1), size(adisk,1), size(ekf,1)]);
gt = gt0(1:n,:); adisk = adisk(1:n,:); ekf = ekf(1:n,:);
L = sum(sqrt(sum(diff(gt).^2,2)));
load(fullfile(tar,'a_final','loop_constraint_events.mat'), 'NLM_LOOP_EVENTS');
win = 400;
d0 = sqrt(sum((adisk - adisk(1,:)).^2,2));
sra = find(d0 > 1, 1, 'first'); if isempty(sra), sra = 1; end
aWin = adisk(sra : min(sra+win-1, n), :);
evs = sortrows(NLM_LOOP_EVENTS, 1);
fprintf('n=%d 事件=%d 锚点窗=[%d:%d] win=%d\n', n, size(evs,1), sra, min(sra+win-1,n), win);

% 预计算段区间 (与 core 实现一致: 下一事件前的帧)
segA = zeros(size(evs,1),1); segB = zeros(size(evs,1),1);
for k = 1:size(evs,1)
    t_now = evs(k,1);
    if k < size(evs,1), t_next = evs(k+1,1); else, t_next = n+1; end
    segA(k) = t_now+1; segB(k) = t_next-1;
end

v1 = run_variant(evs, segA, segB, adisk, aWin, win, 'V1');
v2 = run_variant(evs, segA, segB, adisk, aWin, win, 'V2');
v3 = run_variant(evs, segA, segB, adisk, aWin, win, 'V3');

variants = {'a_disk现状(平移A-fix)', adisk;
            'V1 裸Sim(3)',      v1;
            'V2 钳位尺度',      v2;
            'V3 SE(3)无尺度',   v3;
            'EKF 基线',         ekf};
bounds = [59;2386;4714;7037;min(9365,n)];

fprintf('\n%-20s %-13s %-11s %-10s   per-lap oracle\n', '变体','版本A(论文)','simple','路径/GT');
for k = 1:size(variants,1)
    nm = variants{k,1}; tr = variants{k,2};
    va = compute_metrics_with_alignment(tr, gt, L);
    c = tr - tr(1,:); g = gt - gt(1,:);
    l1 = sum(sqrt(sum(diff(c).^2,2))); s = L/l1;
    vs = sqrt(mean(sum((c*s-g).^2,2)));
    pl = zeros(numel(bounds)-1,1);
    for j = 1:numel(bounds)-1
        a = bounds(j)+1; b = bounds(j+1);
        pl(j) = compute_metrics_with_alignment(tr(a:b,:), gt(a:b,:), sum(sqrt(sum(diff(gt(a:b,:)).^2,2))));
    end
    fprintf('%-20s %11.2f m %9.2f m %8.3f   [%s]\n', nm, va, vs, l1/L, mat2str(pl,3));
end
fprintf('\n判读: V3(SE(3)) 若接近 V1 → 尺度非主导, 可用更安全的 SE(3) 重锚; V3 若退回~800 → 尺度主导, 必须全Sim(3).\n');
fprintf('      各变体 per-lap oracle 都应 ≈ 基线(48/60/171/51) → 增益纯来自全局Sim(3)移除, 无局部畸变注入.\n');
fprintf('REANCHOR_VARIANTS_DONE\n');

function corr = run_variant(evs, segA, segB, adisk, aWin, win, mode)
    corr = adisk;
    prev_b = 1.0;
    ca = mean(aWin,1);
    for k = 1:size(evs,1)
        cWin = adisk(max(evs(k,1)-win+1,1) : min(evs(k,1), segB(k)), :);
        if numel(cWin) < 30 || segB(k) < segA(k), continue; end
        [~, ~, T] = procrustes(aWin, cWin, 'Scaling', true);
        Rmat = T.T;                 % 3x3, 行向量右乘: anchor ≈ b*(cur*Rmat)+t
        b = T.b;
        if strcmp(mode,'V2')
            lo = 0.5*prev_b; hi = 2.0*prev_b;
            if b < lo, b = lo; elseif b > hi, b = hi; end
            prev_b = b;
        elseif strcmp(mode,'V3')
            b = 1.0;
        end
        cc = mean(cWin,1);
        tvec = ca - b*(cc*Rmat);    % 质心约束: 保持窗口质心映射一致(对钳位稳健)
        seg = segA(k):segB(k);
        corr(seg,:) = b*(adisk(seg,:)*Rmat) + tvec;
    end
end
