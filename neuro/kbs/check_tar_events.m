% check_tar_events.m — 检查 Turnaround A/B 的闭环事件数与报告口径
clear all; close all; clc;
tar = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets/Town01Turnaround_IMU_Fusion';
for d = {'a_final', 'dc_final'}
    f = fullfile(tar, d{1}, 'loop_constraint_events.mat');
    fprintf('== %s ==\n', f);
    if exist(f, 'file')
        E = load(f);
        for k = whos('-file', f)
            fprintf('  %s: %s\n', k.name, mat2str(k.size, 1));
        end
        if isfield(E, 'loop_constraint_events')
            ev = E.loop_constraint_events;
            fprintf('  事件数=%d\n', numel(ev));
            if ~isempty(ev)
                for i = 1:min(numel(ev), 12)
                    fprintf('   ev%d: frame=%d corr=%s\n', i, ev(i).frame, mat2str(ev(i).corr, 2));
                end
            end
        else
            fprintf('  (字段名不同, 已加载字段见上)\n');
        end
    else
        fprintf('  (无)\n');
    end
end
fprintf('CHECK_EVENTS_DONE\n');
