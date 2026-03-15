%% Add GT Data to MAT File
% Fix GT not showing issue

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  Add GT Data to MAT File                                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% 1. Read GT data
fprintf('[1/3] Read Ground Truth data...\n');

data_path = 'E:\Neuro_end\neuro\data\Town01Data_IMU_Fusion';
gt_file = fullfile(data_path, 'ground_truth.txt');

if ~exist(gt_file, 'file')
    fprintf('  ✗ GT file does not exist: %s\n', gt_file);
    return;
end

% Read TXT (CSV format) - Use read_ground_truth function
gt_data = read_ground_truth(gt_file);
fprintf('  ✓ Read successful, %d rows\n', gt_data.count);

fprintf('  ✓ GT data read successfully\n');
fprintf('    Data points: %d\n', size(gt_data.pos, 1));
fprintf('    Trajectory length: %.2f meters\n', sum(sqrt(sum(diff(gt_data.pos).^2, 2))));

%% 2. Load existing MAT file
fprintf('\n[2/3] Load existing MAT file...\n');

mat_file = fullfile(data_path, 'comparison_results', 'comparison_results.mat');

if ~exist(mat_file, 'file')
    fprintf('  ✗ MAT file does not exist: %s\n', mat_file);
    fprintf('  Please run experiment first to generate MAT file\n');
    return;
end

fprintf('  ✓ MAT file exists\n');

% Load all variables
load(mat_file);
fprintf('  ✓ MAT file loaded successfully\n');

%% 3. Add GT data and save
fprintf('\n[3/3] Add GT data and save...\n');

% Check if gt_data already exists
if exist('gt_data', 'var')
    fprintf('  ⚠️  gt_data already exists in MAT file, will be overwritten\n');
end

% Save (append mode)
try
    save(mat_file, 'gt_data', '-append');
    fprintf('  ✓ GT data added to MAT file\n');
catch ME
    fprintf('  ✗ Save failed: %s\n', ME.message);
    return;
end

%% 4. Verify
fprintf('\n[4/4] Verify...\n');

% Reload to verify
clear gt_data;
load(mat_file, 'gt_data');

if exist('gt_data', 'var') && isstruct(gt_data) && isfield(gt_data, 'pos')
    fprintf('  ✓ Verification successful!\n');
    fprintf('    gt_data.pos: %s\n', mat2str(size(gt_data.pos)));
    fprintf('    gt_data.timestamp: %s\n', mat2str(size(gt_data.timestamp)));
else
    fprintf('  ✗ Verification failed!\n');
    return;
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  Complete!                                                   ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

fprintf('\nNow you can run plot_traj_with_gt to view trajectory comparison\n');
