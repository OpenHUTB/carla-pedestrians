%% REGEN_PAPER_FIGURES.m — 用本地数据(dc_final/EKF-odo+DC v3)重出论文4张图
%  输出: neuro/kbs/fig/{representative_performance, performance_summary,
%        ablation_unified, KITTI_07_input_output_visualization}.pdf
%  口径: 7-DoF Procrustes 全轨迹对齐 (与 compute_metrics_with_alignment.m 一致)
%  用法: matlab_batch.sh REGEN_PAPER_FIGURES.m   (无头, 输出到 neuro/kbs/fig)
%-----------------------------------------------------------------------------
ROOT  = '/home/yangrb/openhutb/neuro/data';
OUT   = '/home/yangrb/openhutb/neuro/kbs/fig';
C_NLM = hex2rgb('4490C2');
C_EKF = hex2rgb('F4A582');
C_VO  = hex2rgb('92C59B');

set(0,'DefaultAxesFontName','Liberation Serif',...
    'DefaultTextFontName','Liberation Serif',...
    'DefaultLineLineWidth',1.0);

run_fig('representative_performance', @fig_representative);
run_fig('performance_summary', @fig_performance_summary);
run_fig('ablation_unified', @fig_ablation);
run_fig('KITTI_07_input_output', @fig_kitti_io);
disp('ALL FIGURES DONE (skipped ones need re-collected data)');

%=============================================================================
function run_fig(name, f)
try
    f();
catch ME
    fprintf('[SKIP] %s: %s (line %d, %s)\n', name, ME.message, ME.stack(1).line, ME.stack(1).file);
    if numel(ME.stack) > 1
        fprintf('    <- %s line %d\n', ME.stack(2).file, ME.stack(2).line);
    end
end
end

%=============================================================================
function fig_representative()
ROOT = '/home/yangrb/openhutb/neuro/data';
OUT  = '/home/yangrb/openhutb/neuro/kbs/fig';
C_NLM = hex2rgb('4490C2'); C_EKF = hex2rgb('F4A582'); C_VO = hex2rgb('92C59B');
t01 = load_ds('Town01');
mh3 = load_euroc('MH_03_medium');
mp  = load_map('Town01');
fs = 8.5;

fig = figure('Visible','off','Color','w','Position',[0 0 483 340]);   % 6.71x4.72in @100dpi
ax = subplot(2,2,1);
cats = {'RMSE/10','Drift (%)','RPE x10','VT/10','Loops'};
v_nlm = [t01.rmse/10, t01.drift, t01.rpe*10, mp.n_templates/10, mp.n_loops];
v_ekf = [t01.ekf_rmse/10, t01.ekf_drift, t01.ekf_rpe*10, 0, 0];
v_vo  = [t01.vo_rmse/10, t01.vo_drift, t01.vo_rpe*10, 0, 0];
b = bar((1:5)', [v_nlm; v_ekf; v_vo]', 0.72);
b(1).FaceColor = C_NLM; b(2).FaceColor = C_EKF; b(3).FaceColor = C_VO;
for k=1:3, b(k).EdgeColor='k'; b(k).LineWidth=0.4; end
ax.XTick = 1:5; ax.XTickLabel = cats; ax.XTickLabel.FontSize = fs-1.5;
ylabel(ax,'Value (scaled)','FontSize',fs);
title(ax,'(a) Town01 Metrics (517 m, 5000 frames)','FontSize',fs+0.5);
style_ax(ax, fs);

ax = subplot(2,2,2);
n = min(numel(t01.err), numel(t01.ekf_err), numel(t01.vo_err));
plot(ax,(1:n)',t01.err(1:n),   'Color',C_NLM,'LineWidth',1.2); hold(ax,'on');
plot(ax,(1:n)',t01.ekf_err(1:n),'Color',C_EKF,'LineStyle','--','LineWidth',1.2);
plot(ax,(1:n)',t01.vo_err(1:n),'Color',C_VO,'LineStyle',':','LineWidth',0.9, ...
     'Marker','.','MarkerSize',4);
xlabel(ax,'Frame Number','FontSize',fs); ylabel(ax,'Position Error (m)','FontSize',fs);
title(ax,'(b) Town01 Error Evolution (aligned per frame)','FontSize',fs+0.5);
style_ax(ax, fs);
lg = legend(ax,'NeuroLocMap','EKF Fusion','Visual Odometry','FontSize',fs-1.5,...
            'Location','northwest'); lg.Position = [0.03 0.80 0.22 0.15];

ax = subplot(2,2,3);
cats = {'RMSE/10','Drift (%)','RPE x10'};
v_nlm = [mh3.nlm.rmse/10, mh3.nlm.drift, mh3.nlm.rpe*10];
v_ekf = [mh3.ekf.rmse/10, mh3.ekf.drift, mh3.ekf.rpe*10];
v_vo  = [mh3.vo.rmse/10,  mh3.vo.drift,  mh3.vo.rpe*10];
b = bar((1:3)', [v_nlm; v_ekf; v_vo]', 0.72);
b(1).FaceColor = C_NLM; b(2).FaceColor = C_EKF; b(3).FaceColor = C_VO;
for k=1:3, b(k).EdgeColor='k'; b(k).LineWidth=0.4; end
ax.XTick = 1:3; ax.XTickLabel = cats; ax.XTickLabel.FontSize = fs-1.5;
ylabel(ax,'Value (scaled)','FontSize',fs);
title(ax,'(c) MH_03 Metrics (127 m, 2699 frames)','FontSize',fs+0.5);
style_ax(ax, fs);

ax = subplot(2,2,4);
n = min(numel(mh3.nlm.err), numel(mh3.ekf.err), numel(mh3.vo.err));
plot(ax,(1:n)',mh3.nlm.err(1:n),'Color',C_NLM,'LineWidth',1.0); hold(ax,'on');
plot(ax,(1:n)',mh3.ekf.err(1:n),'Color',C_EKF,'LineStyle','--','LineWidth',1.0);
plot(ax,(1:n)',mh3.vo.err(1:n), 'Color',C_VO,'LineStyle',':','LineWidth',0.8, ...
     'Marker','.','MarkerSize',3);
xlabel(ax,'Frame Number','FontSize',fs); ylabel(ax,'Position Error (m)','FontSize',fs);
title(ax,'(d) MH_03 Error Evolution (aligned per frame)','FontSize',fs+0.5);
style_ax(ax, fs);
lg = legend(ax,'NeuroLocMap','EKF Fusion','Visual Odometry','FontSize',fs-1.5);
lg.Position = [0.03 0.80 0.22 0.15];

mytight(fig);
savefig(fig,[OUT '/representative_performance.pdf'],[482.05251/72 339.64125/72]);
fprintf('[1/4] representative  Town01 NLM=%.2f/%.2f EKF=%.2f/%.2f VO=%.2f/%.2f n=%d vt=%d loops=%d\n', ...
    t01.rmse,t01.drift, t01.ekf_rmse,t01.ekf_drift, t01.vo_rmse,t01.vo_drift, ...
    mp.n, mp.n_templates, mp.n_loops);
fprintf('      MH03 NLM=%.2f EKF=%.2f VO=%.2f (论文 3.32/3.32/3.43)\n', ...
    mh3.nlm.rmse, mh3.ekf.rmse, mh3.vo.rmse);
close(fig);
end

%=============================================================================
function fig_performance_summary()
ROOT = '/home/yangrb/openhutb/neuro/data';
OUT  = '/home/yangrb/openhutb/neuro/kbs/fig';
C_NLM = hex2rgb('4490C2'); C_EKF = hex2rgb('F4A582'); C_VO = hex2rgb('92C59B');
ds_list = {'Town01','Town02','Town05','Town10HD','KITTI07'};
d = cell(1,numel(ds_list));
for i=1:numel(ds_list), d{i} = load_ds(ds_list{i}); end
fs = 8;
labels = {'Town01','Town02','Town05','Town10HD','KITTI 07','MH_01','MH_03'};
nlm_v = [d{1}.rmse, d{2}.rmse, d{3}.rmse, d{4}.rmse, d{5}.rmse, 4.14, 3.32];
ekf_v = [d{1}.ekf_rmse, d{2}.ekf_rmse, d{3}.ekf_rmse, d{4}.ekf_rmse, d{5}.ekf_rmse, 4.14, 3.32];
vo_v  = [d{1}.vo_rmse, d{2}.vo_rmse, d{3}.vo_rmse, d{4}.vo_rmse, d{5}.vo_rmse, 3.74, 3.43];
lens  = [d{1}.len_m, d{2}.len_m, d{3}.len_m, d{4}.len_m, d{5}.len_m, 81, 127]/1000;

fig = figure('Visible','off','Color','w','Position',[0 0 466 155]);   % 6.46x2.15in @100dpi
ax = subplot(1,3,1);
b = bar((1:7)', [nlm_v; ekf_v; vo_v]', 0.78);
b(1).FaceColor = C_NLM; b(2).FaceColor = C_EKF; b(3).FaceColor = C_VO;
for k=1:3, b(k).EdgeColor='k'; b(k).LineWidth=0.3; end
ax.XTick = 1:7; ax.XTickLabel = labels;
ax.XTickLabel.FontSize = fs-1.5; ax.XTickLabel.Rotation = 40;
ax.XTickLabel.HorizontalAlignment = 'right';
ylabel(ax,'RMSE (m)','FontSize',fs);
title(ax,'(a) RMSE Comparison','FontSize',fs+0.5);
style_ax(ax, fs-0.5);
lg = legend(ax,'NeuroLocMap','EKF','VO','FontSize',fs-2,'Location','northwest');
lg.Position = [0.05 0.80 0.20 0.15];

ax = subplot(1,3,2);
imp_e = (ekf_v - nlm_v)./ekf_v*100;
s = 120 + lens*2600;
hold(ax,'on');
for i=1:7
    if imp_e(i) >= 0
        scatter(ax,i,imp_e(i),s(i),C_NLM,'filled','EdgeColor','k','LineWidth',0.6);
        if imp_e(i) > 3
            text(ax,i,imp_e(i)*0.55,sprintf('%+.1f%%',imp_e(i)),'HorizontalAlignment','center', ...
                 'FontSize',fs-1.5,'Color','w');
        else
            text(ax,i,-1.4,sprintf('%+.1f%%',imp_e(i)),'HorizontalAlignment','center', ...
                 'FontSize',fs-1.5);
        end
    else
        scatter(ax,i,imp_e(i),s(i),[0.85 0.48 0.48],'filled','EdgeColor','k','LineWidth',0.6);
        text(ax,i,imp_e(i)-1.6,sprintf('%+.1f%%',imp_e(i)),'HorizontalAlignment','center', ...
             'FontSize',fs-1.5);
    end
end
yline(ax,0,'k--','LineWidth',0.8);
ax.XTick = 1:7; ax.XTickLabel = labels;
ax.XTickLabel.FontSize = fs-1.5; ax.XTickLabel.Rotation = 40;
ax.XTickLabel.HorizontalAlignment = 'right';
ylabel(ax,'Improvement vs EKF (%)','FontSize',fs);
title(ax,'(b) Improvement vs EKF (bubble size $\propto$ length)','FontSize',fs+0.5);
style_ax(ax, fs-0.5);
ax.YLim = [-1.8 max(imp_e)*1.35];

ax = subplot(1,3,3);
imp_v = (vo_v - nlm_v)./vo_v*100;
hold(ax,'on');
for i=1:7
    if imp_v(i) >= 0
        bar(ax,i,imp_v(i),0.6,'FaceColor',C_NLM,'EdgeColor','k','LineWidth',0.3);
        text(ax,i,imp_v(i)+1.8,sprintf('%+.1f%%',imp_v(i)),'HorizontalAlignment','center', ...
             'FontSize',fs-1.5,'Color',[0.1 0.29 0.55]);
    else
        bar(ax,i,imp_v(i),0.6,'FaceColor',[0.85 0.48 0.48],'EdgeColor','k','LineWidth',0.3);
        text(ax,i,imp_v(i)-6.0,sprintf('%+.1f%%',imp_v(i)),'HorizontalAlignment','center', ...
             'FontSize',fs-1.5,'Color',[0.55 0.1 0.1]);
    end
end
yline(ax,0,'k','LineWidth',0.8);
ax.XTick = 1:7; ax.XTickLabel = labels;
ax.XTickLabel.FontSize = fs-1.5; ax.XTickLabel.Rotation = 40;
ax.XTickLabel.HorizontalAlignment = 'right';
ylabel(ax,'Improvement vs VO (%)','FontSize',fs);
title(ax,'(c) Improvement vs VO','FontSize',fs+0.5);
style_ax(ax, fs-0.5);
ax.YLim = [-28 max(imp_v)*1.3];

mytight(fig);
savefig(fig,[OUT '/performance_summary.pdf'],[465.01119/72 154.7695895566/72]);
fprintf('[2/4] performance_summary  impE=[');
for i=1:7, fprintf(' %+.1f',imp_e(i)); end; fprintf(']\n');
fprintf('      impV=[');
for i=1:7, fprintf(' %+.1f',imp_v(i)); end; fprintf(']\n');
close(fig);
end

%=============================================================================
function fig_ablation()
ROOT = '/home/yangrb/openhutb/neuro/data';
OUT  = '/home/yangrb/openhutb/neuro/kbs/fig';
C_NLM = hex2rgb('4490C2'); C_EKF = hex2rgb('F4A582'); C_VO = hex2rgb('92C59B');
ds_list = {'Town01','Town02','Town05','Town10HD','KITTI07'};
d = cell(1,numel(ds_list));
mp = cell(1,numel(ds_list));
for i=1:numel(ds_list)
    d{i} = load_ds(ds_list{i});
    mp{i} = load_map(ds_list{i});
end
t01 = d{1};
fs = 8.5;

fig = figure('Visible','off','Color','w','Position',[0 0 488 235]);   % 6.79x3.26in @100dpi
% (a) 消融柱状图
ax = subplot(1,2,1);
cats = {'Full','w/o IMU','w/o ExpMap'};
vals = [t01.rmse, t01.vo_rmse, t01.ekf_rmse];
b = bar(1:3, vals', 0.55);
b.FaceColor = 'flat';
b.CData = [C_NLM; C_VO; C_EKF];
b.EdgeColor = 'k'; b.LineWidth = 0.5;
ax.XTick = 1:3; ax.XTickLabel = cats; ax.XTickLabel.FontSize = fs;
for i=1:3
    text(ax,i,vals(i)+0.6,sprintf('%.1f m',vals(i)),'HorizontalAlignment','center','FontSize',fs);
end
ylabel(ax,'RMSE (m)','FontSize',fs);
title(ax,'(a) Ablation RMSE on Town01 (517 m)','FontSize',fs+0.5);
ax.YLim = [0 max(vals)*1.15];
style_ax(ax, fs);

% (b) 模板增长
ax = subplot(1,2,2);
hold(ax,'on');
styles = {'-','--','-.',':','-'};
colors = {C_NLM,[0.85 0.48 0.48],C_VO,[0.55 0.37 0.75],[0.91 0.64 0.24]};
for i=1:numel(ds_list)
    m = mp{i};
    xy = [m.x; m.y];
    seg = sqrt(sum(diff(xy,1,2).^2,1));   % 1 x (N-1) 节点间位移
    arc = [0, cumsum(seg)];               % 1 x N 累计弧长
    arc = arc / max(arc) * 100;
    plot(ax, arc, m.growth, styles{i}, 'Color', colors{i}, 'LineWidth', 1.3);
    text(ax, 102, m.growth(end), sprintf('%s (%d)', ds_list{i}, m.n_templates), ...
         'FontSize', fs-2, 'Color', colors{i});
end
yline(ax,5,'k-','LineWidth',1.2);
text(ax,102,5,'RatSLAM ($\sim$5 templates)','FontSize',fs-2);
xlabel(ax,'Trajectory Progress (% of length)','FontSize',fs);
ylabel(ax,'Accumulated Visual Templates','FontSize',fs);
title(ax,'(b) Visual Template Growth (5 outdoor sequences)','FontSize',fs+0.5);
ax.XLim = [0 210];
style_ax(ax, fs);
mytight(fig);
savefig(fig,[OUT '/ablation_unified.pdf'],[487.70965/72 234.64125/72]);
fprintf('[3/4] ablation  Full=%.2f VO=%.2f EKF=%.2f\n', t01.rmse, t01.vo_rmse, t01.ekf_rmse);
close(fig);
end

%=============================================================================
function fig_kitti_io()
ROOT = '/home/yangrb/openhutb/neuro/data';
OUT  = '/home/yangrb/openhutb/neuro/kbs/fig';
C_NLM = hex2rgb('4490C2'); C_EKF = hex2rgb('F4A582'); C_VO = hex2rgb('92C59B');
sub = [ROOT '/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion'];
gt  = load_pos(fullfile(sub,'ground_truth.txt'), [1 2 3]);
nlm = load_pos(fullfile(sub,'dc_final/exp_trajectory.txt'), [1 2 3]);
[A, s] = procrustes_7dof(gt, nlm);
n = min(size(gt,1), size(A,1));
gt = gt(1:n,:); A = A(1:n,:);
imu = load_pos(fullfile(sub,'aligned_imu.txt'), [1:7]);
mp  = load_map('KITTI07');
dk  = load_ds('KITTI07');
fs = 8.5;

fig = figure('Visible','off','Color','w','Position',[0 0 574 322]);   % 7.97x4.47in @100dpi

% (a) 输入: 4 张灰度图 + 航向箭头 (单 axes 网格)
ax = subplot(2,2,1);
ax.XLim = [0 54]; ax.YLim = [0 16];
axis(ax,'off');
title(ax,'(a) Input: RGB Images + IMU Data','FontSize',fs+0.5);
frames = [110, 330, 551, 771];
cw = 26; ch = 6.8; gap = 2.0;
for k=1:4
    fr = frames(k);
    p = fullfile(sub, sprintf('%06d.png', fr));
    if ~isfile(p), p = fullfile(sub, sprintf('%04d.png', fr)); end
    if ~isfile(p), continue; end
    im = im2single(imread(p));
    if ndims(im) == 3 && size(im,3) == 3, im = rgb2gray(im); end
    r = floor((k-1)/2); c = mod(k-1,2);
    x0 = c*(cw+gap); y0 = 8.4 - r*(ch+gap);
    xv = linspace(x0, x0+cw, size(im,2));
    yv = linspace(y0, y0+ch, size(im,1));
    image(ax, xv, yv, im);
    hold(ax,'on');
    bp = patch;                                  % 对象式: 本环境函数式 patch(x,y) 会误报参数不足
    bp.XData = [x0 x0+cw x0+cw x0]; bp.YData = [y0 y0 y0+ch y0+ch];
    bp.Parent = ax; bp.FaceColor = 'none'; bp.EdgeColor = 'k'; bp.LineWidth = 0.7;
    text(ax, x0+0.5, y0+ch-0.9, sprintf('Frame %d', fr), 'FontSize', fs-1, ...
         'Color','w','BackgroundColor',[0.2 0.2 0.2]);
    g0 = gt(fr,:); g1 = gt(min(fr+3,n),:);
    ang = atan2(g1(2)-g0(2), g1(1)-g0(1));
    cx = x0+cw/2; cy = y0+ch/2;
    quiver(ax, cx-1.7*cos(ang), cy-1.7*sin(ang), 3.4*cos(ang), 3.4*sin(ang), 0, ...
           'Color',C_NLM,'LineWidth',2.4,'MaxHeadSize',0.8);
end

% (b) 上: 轨迹对齐
ax = subplot(2,2,2);
hold(ax,'on');
plot(ax, gt(:,1), gt(:,2), 'k-', 'LineWidth', 1.8);
plot(ax, A(:,1), A(:,2), 'Color', C_NLM, 'LineWidth', 1.3);
plot(ax, gt(1,1), gt(1,2), 'o', 'MarkerFaceColor',[0.24 0.6 0.24],'MarkerSize',6);
plot(ax, gt(end,1), gt(end,2), 's', 'MarkerFaceColor',[0.75 0.22 0.17],'MarkerSize',6);
e = sqrt(sum((A-gt).^2,2));
str = sprintf('RMSE: NLM %.2f / EKF %.2f / VO %.2f m', ...
    sqrt(mean(e.^2)), dk.ekf_rmse, dk.vo_rmse);
text(ax, 0.98, 0.06, str, 'Units','normalized','HorizontalAlignment','right', ...
     'FontSize', fs-1, 'BackgroundColor','w','EdgeColor',[0.5 0.5 0.5]);
xlabel(ax,'X (m)','FontSize',fs); ylabel(ax,'Y (m)','FontSize',fs);
title(ax,'Trajectory (aligned, official 7-DoF metric)','FontSize',fs+0.5);
lg = legend(ax,'Ground Truth','NeuroLocMap','Start','End','FontSize',fs-1.5,...
            'Location','northeast'); lg.Position = [0.62 0.70 0.30 0.22];
axis(ax,'equal'); style_ax(ax, fs);

% (b) 下左: IMU
ax = subplot(2,2,3);
acc = sqrt(imu(:,2).^2 + imu(:,3).^2 + imu(:,4).^2);
gyro = imu(:,6);
t = imu(:,1);
yyaxis(ax,'left');
plot(ax, t, acc, 'Color',[0.85 0.48 0.48],'LineWidth',0.5);
yline(ax,9.81,'--','Color',[0.85 0.48 0.48],'LineWidth',0.8);
ylabel(ax,'Acceleration (m/s$^2$)','FontSize',fs); xlabel(ax,'Time (s)','FontSize',fs);
yyaxis(ax,'right');
plot(ax, t, gyro, 'Color',C_NLM,'LineWidth',0.5);
ylabel(ax,'Angular Vel (rad/s)','FontSize',fs,'Color',C_NLM);
title(ax,'IMU Sensor Data (10 Hz, 692 m highway)','FontSize',fs+0.5);
style_ax(ax, fs);
legend(ax,'Accel norm','Gyro Z','FontSize',fs-2,'Location','northeast');

% (b) 下右: 经验地图拓扑
ax = subplot(2,2,4);
hold(ax,'on');
plot(ax, mp.x, mp.y, '-', 'Color', C_NLM, 'LineWidth', 1.0);
for i=1:size(mp.loops,1)
    ii = mp.loops(i,1); jj = mp.loops(i,2);
    plot(ax, [mp.x(ii) mp.x(jj)], [mp.y(ii) mp.y(jj)], '--', ...
         'Color',[0.24 0.6 0.24],'LineWidth',1.4);
end
plot(ax, mp.x, mp.y, '.', 'MarkerSize', 3, 'Color', C_NLM);
if ~isempty(mp.loops)
    ii = mp.loops(1,1); jj = mp.loops(1,2);
    plot(ax, [mp.x(ii) mp.x(jj)], [mp.y(ii) mp.y(jj)], '--', ...
         'Color',[0.24 0.6 0.24],'LineWidth',1.8);
    plot(ax, [mp.x(ii) mp.x(jj)], [mp.y(ii) mp.y(jj)], '*', 'MarkerSize', 8, ...
         'MarkerFaceColor',[0.75 0.22 0.17],'MarkerEdgeColor','k');
    lg = legend(ax,'Loop closure edge','FontSize',fs-1.5,'Location','northeast');
end
xlabel(ax,'X (m)','FontSize',fs); ylabel(ax,'Y (m)','FontSize',fs);
title(ax, sprintf('Experience Map Topology (%d nodes, %d loop edges)', mp.n, size(mp.loops,1)), ...
      'FontSize', fs+0.5);
style_ax(ax, fs);

mytight(fig);
savefig(fig,[OUT '/KITTI_07_input_output_visualization.pdf'],[573.5874375/72 321.64125/72]);
fprintf('[4/4] KITTI_io  nodes=%d loops=%d RMSE=%.2f scale=%.2f\n', mp.n, size(mp.loops,1), ...
    sqrt(mean(e.^2)), s);
close(fig);
end

%=============================================================================
function style_ax(ax, fs)
set(ax, 'FontSize', fs, 'LineWidth', 0.8);   % FontSize 同时控制刻度字号
grid(ax,'on'); set(ax,'XGrid','on','YGrid','on','Box','on');
try
    g = findobj(ax,'-property','LineStyle');
    for i=1:numel(g)
        if isa(g(i), 'matlab.graphics.chart.primitive.GridLine')
            set(g(i),'LineWidth',0.4,'Color',[0.7 0.7 0.7],'LineStyle','-');
        end
    end
end
end

%=============================================================================
function c = hex2rgb(h)
c = [hex2dec(h(1:2)), hex2dec(h(3:4)), hex2dec(h(5:6))]/255;
end

%=============================================================================
function savefig(fig, path, sz)
% sz: 目标 PDF 尺寸 (英寸)。本干净 R2021b 安装对 PaperSize/PaperPosition
% 按厘米解释 (实测 7.967 -> 226pt, 28.3465pt/cm), 故先换算成厘米再写入。
sz_cm = sz * 2.54;
fig.PaperPositionMode = 'manual';
fig.PaperPosition = [0 0 sz_cm(1) sz_cm(2)];
fig.PaperSize = sz_cm;
print(fig, path, '-dpdf');
end

%=============================================================================
function mytight(fig)
% 本干净 R2021b 安装无 tightlayout/layout: 手动给各 axes 加小边距防标题/轴标重叠
axs = findobj(fig, 'type', 'axes');
for i = 1:numel(axs)
    p = axs(i).Position;
    axs(i).Position = [p(1)+0.015, p(2)+0.010, p(3)-0.030, p(4)-0.020];
end
end

%=============================================================================
function place_ax(ax, fig, r, c, nr, nc)
% 按 figure 归一化尺寸精确定位 axes (figure 窗口像素 = 目标 PDF 尺寸 @100dpi)
mL = 0.075; mR = 0.025; mT = 0.06; mB = 0.10; gap = 0.028;
W = (1 - mL - mR - gap*(nc-1)) / nc;
H = (1 - mT - mB - gap*(nr-1)) / nr;
x = mL + (c-1)*(W+gap);
y = 1 - mT - (r-1)*(H+gap) - H;
ax.Position = [x, y, W, H];
end

%=============================================================================
function [A, s] = procrustes_7dof(gt, est)
n = min(size(gt,1), size(est,1));
gt = gt(1:n,:); est = est(1:n,:);
gs = gt - mean(gt); es = est - mean(est);
H = gs' * es;
[U, S, V] = svd(H);
D = diag([1 1 sign(det(U*V))]);
R = U * D * V';
s = sum(diag(S)) / max(sum(es(:).^2), 1e-12);
t = mean(gt,1) - s*(R*mean(est,1)')';
A = (s*(R*est'))' + t;
end

%=============================================================================
function P = load_pos(path, cols)
% readmatrix 自动识别 CSV 表头并跳过 (gt/fusion 有表头, 轨迹无表头), 返回纯数值
P = readmatrix(path);
P = P(:, cols);
end

%=============================================================================
function ok = ds_ready(ds)
sub = ['/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/' ds 'Data_IMU_Fusion'];
ok = isfile(fullfile(sub, 'ground_truth.txt')) && ...
     isfile(fullfile(sub, 'fusion_pose.txt')) && ...
     isfile(fullfile(sub, 'dc_final/exp_trajectory.txt')) && ...
     isfile(fullfile(sub, 'dc_final/pure_visual_trajectory.txt')) && ...
     isfile(fullfile(sub, 'dc_final/experiences.mat'));
end

%=============================================================================
function d = load_ds(ds)
ROOT = '/home/yangrb/openhutb/neuro/data';
sub = [ROOT '/01_NeuroSLAM_Datasets/' sprintf('%sData_IMU_Fusion', ds)];
gt  = load_pos(fullfile(sub,'ground_truth.txt'), [1 2 3]);
nlm = load_pos(fullfile(sub,'dc_final/exp_trajectory.txt'), [1 2 3]);
ekf = load_pos(fullfile(sub,'fusion_pose.txt'), [1 2 3]);
vo  = load_pos(fullfile(sub,'dc_final/pure_visual_trajectory.txt'), [1 2 3]);
d.gt = gt;
d.len_m = sum(sqrt(sum(diff(gt,2,1).^2,2)));
[A,s] = procrustes_7dof(gt,nlm);
n = min(size(gt,1),size(A,1));
d.err = sqrt(sum((A(1:n,:)-gt(1:n,:)).^2,2))';
d.rmse = sqrt(mean(d.err.^2));
d.drift = d.err(end)/d.len_m*100;
d.rpe = mean(sqrt(sum((diff(A(1:n,:),1,2)-diff(gt(1:n,:),1,2)).^2,2)));
d.A = A;
[A2,~] = procrustes_7dof(gt,ekf);
n2 = min(size(gt,1),size(A2,1));
d.ekf_err = sqrt(sum((A2(1:n2,:)-gt(1:n2,:)).^2,2))';
d.ekf_rmse = sqrt(mean(d.ekf_err.^2));
d.ekf_drift = d.ekf_err(end)/d.len_m*100;
d.ekf_rpe = mean(sqrt(sum((diff(A2(1:n2,:),1,2)-diff(gt(1:n2,:),1,2)).^2,2)));
[A3,~] = procrustes_7dof(gt,vo);
n3 = min(size(gt,1),size(A3,1));
d.vo_err = sqrt(sum((A3(1:n3,:)-gt(1:n3,:)).^2,2))';
d.vo_rmse = sqrt(mean(d.vo_err.^2));
d.vo_drift = d.vo_err(end)/d.len_m*100;
d.vo_rpe = mean(sqrt(sum((diff(A3(1:n3,:),1,2)-diff(gt(1:n3,:),1,2)).^2,2)));
end

%=============================================================================
function d = load_euroc(mh_dir)
ROOT = '/home/yangrb/openhutb/neuro/data';
m = load([ROOT '/02_EuRoc_Dataset' sprintf('/%s/slam_results/euroc_trajectories.mat', mh_dir)]);
g = m.gt_data.pos;
while iscell(g), g = g{1}; end
g = reshape(g, 3, [])';
d.gt = g;
nm_list = {'nlm','ekf','vo'};
key_list = {'exp_trajectory','fusion_data','pure_visual_traj'};
for i=1:3
    nm = nm_list{i};
    key = key_list{i};
    v = m.(key);
    if isnumeric(v)
        p = v;                                % exp_trajectory/pure_visual_traj 是裸 N x 3
    else
        p = v.pos;                            % fusion_data 是 struct 数组
        while iscell(p), p = p{1}; end
    end
    p = reshape(p, 3, [])';
    [A,~] = procrustes_7dof(g, p);
    n = min(size(g,1), size(A,1));
    err = sqrt(sum((A(1:n,:)-g(1:n,:)).^2,2))';
    L = sum(sqrt(sum(diff(g,2,1).^2,2)));
    d.(nm).err = err;
    d.(nm).rmse = sqrt(mean(err.^2));
    d.(nm).drift = err(end)/L*100;
    d.(nm).rpe = mean(sqrt(sum((diff(A(1:n,:),1,2)-diff(g(1:n,:),1,2)).^2,2)));
    d.(nm).n = n;
end
end

%=============================================================================
function m = load_map(ds)
ROOT = '/home/yangrb/openhutb/neuro/data';
p = [ROOT '/01_NeuroSLAM_Datasets' sprintf('/%sData_IMU_Fusion/dc_final/experiences.mat', ds)];
E = load(p, 'EXPERIENCES');
E = E.EXPERIENCES;
if iscell(E), E = E{1}; end
n = numel(E);
% 注意: 该 .mat 由 scipy 存出, 整段访问 E.vt_id 只得到 [1 1],
% 必须逐元素提取 (与 Python 版 loadmat 口径一致, 已验证 964/929/19)
x = zeros(1,n); y = zeros(1,n); vt = zeros(1,n);
for i = 1:n
    x(i)  = E(i).x_exp;
    y(i)  = E(i).y_exp;
    vt(i) = E(i).vt_id;
end
vt_int = round(vt);
vt_pos = vt_int(vt_int > 0);
vmax = max([vt_pos(:); 0]);   % 列向量拼接, 避免行向量+标量维度不一致
seen = false(1, vmax+1);
growth = zeros(1, n);
for i = 1:n
    if vt_int(i) > 0 && ~seen(vt_int(i)), seen(vt_int(i)) = true; end
    growth(i) = nnz(seen);
end
m.n = n; m.x = x; m.y = y;
m.n_templates = numel(unique(vt_pos));
m.growth = growth;
m.loops = [];
for i = 1:n
    nl = size(E(i).links, 2);
    for j = 1:nl
        t = round(E(i).links(j).exp_id) - 1;   % 1-based exp_id -> 0-based target
        % 0-based 邻接判定(与 Python 版一致): i 是 1-based, 0-based 源 = i-1
        if t >= 0 && t < n && t ~= i-2 && t ~= i
            m.loops(end+1, 1:2) = [min(i, t+1), max(i, t+1)]; %#ok<AGROW>
        end
    end
end
m.loops = unique(m.loops, 'rows');
m.n_loops = size(m.loops, 1);
end

