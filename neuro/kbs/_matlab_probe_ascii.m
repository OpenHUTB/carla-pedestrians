% ASCII-only probe: no non-ASCII characters at all.
disp(sprintf('MATLAB_OK v%s', version));
try
    I = imread('/home/yangrb/openhutb/neuro/data/Town01Turnaround_IMU_Fusion/0001.png');
    fprintf('imread_OK size=%dx%d\n', size(I,1), size(I,2));
catch e
    fprintf('imread_FAIL: %s\n', e.message);
end
