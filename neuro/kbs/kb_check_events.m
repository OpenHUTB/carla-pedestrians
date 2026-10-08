% kb_check_events.m — 各数据集 a_final 闭环事件计数 (S2 中性判定基础)
root = '/home/yangrb/openhutb/neuro/data/01_NeuroSLAM_Datasets';
datasets = {'Town01Data_IMU_Fusion','Town02Data_IMU_Fusion','Town05Data_IMU_Fusion', ...
            'Town10HDData_IMU_Fusion','KITTI07Data_IMU_Fusion','Town01Turnaround_IMU_Fusion'};
for i = 1:numel(datasets)
    f = fullfile(root, datasets{i}, 'a_final', 'loop_constraint_events.mat');
    if exist(f, 'file')
        d = load(f);
        ev = d.NLM_LOOP_EVENTS;
        fprintf('%-30s : %d events\n', datasets{i}, size(ev, 1));
        if size(ev, 1) > 0 && size(ev, 1) <= 8
            disp(ev);
        end
    else
        fprintf('%-30s : (no events mat)\n', datasets{i});
    end
end
fprintf('KB_CHECK_EVENTS_DONE\n');
