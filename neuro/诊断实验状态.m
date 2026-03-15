%% 诊断实验状态
% 检查workspace中是否有实验结果

clearvars -except baseline_* ours_* NUM_VT_* NUM_EXPS_* gt_* EXPERIENCES*;
clc;

fprintf('========== 诊断实验状态 ==========\n\n');

%% 检查Baseline结果
fprintf('[1] 检查Baseline结果...\n');
if exist('baseline_odo_traj', 'var')
    fprintf('  ✓ baseline_odo_traj: %d 帧\n', size(baseline_odo_traj, 1));
else
    fprintf('  ✗ baseline_odo_traj: 不存在\n');
end

if exist('baseline_exp_traj', 'var')
    fprintf('  ✓ baseline_exp_traj: %d 帧\n', size(baseline_exp_traj, 1));
else
    fprintf('  ✗ baseline_exp_traj: 不存在\n');
end

if exist('NUM_VT_BL', 'var')
    fprintf('  ✓ NUM_VT_BL: %d\n', NUM_VT_BL);
else
    fprintf('  ✗ NUM_VT_BL: 不存在\n');
end

if exist('NUM_EXPS_BL', 'var')
    fprintf('  ✓ NUM_EXPS_BL: %d\n', NUM_EXPS_BL);
else
    fprintf('  ✗ NUM_EXPS_BL: 不存在\n');
end

%% 检查Ours结果
fprintf('\n[2] 检查Ours结果...\n');
if exist('ours_odo_traj', 'var')
    fprintf('  ✓ ours_odo_traj: %d 帧\n', size(ours_odo_traj, 1));
else
    fprintf('  ✗ ours_odo_traj: 不存在\n');
end

if exist('ours_exp_traj', 'var')
    fprintf('  ✓ ours_exp_traj: %d 帧\n', size(ours_exp_traj, 1));
else
    fprintf('  ✗ ours_exp_traj: 不存在\n');
end

if exist('NUM_VT_OURS', 'var')
    fprintf('  ✓ NUM_VT_OURS: %d\n', NUM_VT_OURS);
else
    fprintf('  ✗ NUM_VT_OURS: 不存在\n');
end

if exist('NUM_EXPS_OURS', 'var')
    fprintf('  ✓ NUM_EXPS_OURS: %d\n', NUM_EXPS_OURS);
else
    fprintf('  ✗ NUM_EXPS_OURS: 不存在\n');
end

%% 检查Ground Truth
fprintf('\n[3] 检查Ground Truth...\n');
if exist('gt_data', 'var')
    fprintf('  ✓ gt_data: %d 帧\n', size(gt_data.pos, 1));
elseif exist('gt_pos_aligned', 'var')
    fprintf('  ✓ gt_pos_aligned: %d 帧\n', size(gt_pos_aligned, 1));
else
    fprintf('  ✗ Ground Truth: 不存在\n');
end

%% 检查全局变量
fprintf('\n[4] 检查全局变量...\n');
global EXPERIENCES NUM_EXPS VT NUM_VT;
if ~isempty(EXPERIENCES)
    fprintf('  ✓ EXPERIENCES: %d 个\n', length(EXPERIENCES));
else
    fprintf('  ✗ EXPERIENCES: 空\n');
end

if ~isempty(NUM_EXPS)
    fprintf('  ✓ NUM_EXPS: %d\n', NUM_EXPS);
else
    fprintf('  ✗ NUM_EXPS: 空\n');
end

if ~isempty(VT)
    fprintf('  ✓ VT: %d 个\n', length(VT));
else
    fprintf('  ✗ VT: 空\n');
end

if ~isempty(NUM_VT)
    fprintf('  ✓ NUM_VT: %d\n', NUM_VT);
else
    fprintf('  ✗ NUM_VT: 空\n');
end

%% 列出所有变量
fprintf('\n[5] Workspace中的所有变量:\n');
whos

%% 诊断结论
fprintf('\n========== 诊断结论 ==========\n');

has_baseline = exist('baseline_odo_traj', 'var') && exist('NUM_VT_BL', 'var');
has_ours = exist('ours_odo_traj', 'var') && exist('NUM_VT_OURS', 'var');

if has_baseline && has_ours
    fprintf('✓ 实验数据完整,可以继续可视化\n');
    fprintf('\n建议操作:\n');
    fprintf('1. 运行可视化脚本:\n');
    fprintf('   >> QUICK_VIEW_RESULTS\n');
    fprintf('\n2. 或者手动保存数据:\n');
    fprintf('   >> save(''comparison_results.mat'', ''baseline_*'', ''ours_*'', ''NUM_*'')\n');
    
    % 计算基本统计
    fprintf('\n基本统计:\n');
    if exist('baseline_exp_traj', 'var')
        bl_len = sum(sqrt(sum(diff(baseline_exp_traj).^2, 2)));
        fprintf('  Baseline轨迹长度: %.1f m\n', bl_len);
    end
    if exist('ours_exp_traj', 'var')
        ours_len = sum(sqrt(sum(diff(ours_exp_traj).^2, 2)));
        fprintf('  Ours轨迹长度: %.1f m\n', ours_len);
    end
    
elseif has_baseline && ~has_ours
    fprintf('⚠️ 只有Baseline数据,Ours未完成\n');
    fprintf('\n建议: 重新运行实验\n');
    fprintf('   >> test_imu_visual_fusion_slam2\n');
    
elseif ~has_baseline && has_ours
    fprintf('⚠️ 只有Ours数据,Baseline未完成\n');
    fprintf('\n建议: 重新运行实验\n');
    fprintf('   >> test_imu_visual_fusion_slam2\n');
    
else
    fprintf('✗ 没有实验数据\n');
    fprintf('\n建议: 运行实验\n');
    fprintf('   >> cd E:\\Neuro_end\\neuro\\07_test\\07_test\\test_imu_visual_slam\\core\n');
    fprintf('   >> test_imu_visual_fusion_slam2\n');
end

fprintf('========================================\n');
