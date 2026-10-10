% Probe: does MATLAB 2021b actually run, read images, and reach the SLAM core?
disp(sprintf('MATLAB_OK v%s', version));
try
    I = imread('/home/yangrb/openhutb/neuro/data/Town01Turnaround_IMU_Fusion/0001.png');
    fprintf('imread_OK size=%dx%d\n', size(I,1), size(I,2));
catch e
    fprintf('imread_FAIL: %s\n', e.message);
end
% Is the SLAM core function resolvable?
try
    which('test_imu_visual_fusion_slam');
catch
    fprintf('core_not_on_path (run via quickstart mfilename path)\n');
end
