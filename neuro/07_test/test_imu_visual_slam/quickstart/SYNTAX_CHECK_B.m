% SYNTAX_CHECK_B.m — B方案改动三文件语法检查
p = '/home/yangrb/openhutb/neuro';
files = {[p '/02_multilayered_experience_map/exp_map_iteration.m'], ...
         [p '/02_multilayered_experience_map/create_new_exp.m'], ...
         [p '/07_test/test_imu_visual_slam/core/test_imu_visual_fusion_slam.m'], ...
         [p '/07_test/test_imu_visual_slam/quickstart/FTEST_ONE.m'], ...
         [p '/07_test/test_imu_visual_slam/quickstart/BTEST_ONE.m'], ...
         [p '/07_test/test_imu_visual_slam/quickstart/RECOGN_SWEEP_ONE.m']};
for k = 1:numel(files)
    try
        feval(@checkcode, files{k});
        fprintf('SYNTAX_OK_%d\n', k);
    catch e
        fprintf('SYNTAX_ERR_%d: %s\n', k, e.message);
    end
end
disp('SYNTAX_CHECK_DONE');
