% 临时脚本: Town01 闭环分析 (NLM_USE_EKF_ODO=true)
% set NLM_ROOT to your checkout; defaults to the author's path
nlmRoot = getenv('NLM_ROOT');
if isempty(nlmRoot)
    nlmRoot = fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
end
cd(fullfile(nlmRoot, 'neuro', '07_test', 'test_imu_visual_slam'));
addpath(fullfile(pwd, 'utils'));
addpath(fullfile(pwd, 'core'));
global NLM_USE_EKF_ODO;
NLM_USE_EKF_ODO = true;
analyze_loop_closures('Town01Data_IMU_Fusion');
disp('=== ANALYZE_LOOP_TOWN01 DONE ===');
