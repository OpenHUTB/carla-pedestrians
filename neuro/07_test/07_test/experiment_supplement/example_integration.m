%% 示例：如何集成性能监控到你的代码
%  这个脚本展示了如何修改 test_imu_visual_fusion_slam.m
%  不需要运行这个脚本，只需要参考它来修改你的主函数

%% ============================================================
%%  第1步：在文件开头初始化（clear all 之后）
%% ============================================================

% 添加utils路径（如果还没添加）
addpath('../utils');

% 初始化性能监控器
perf_monitor = matlab_performance_monitor();

% 初始化模板统计
vt_stats = matlab_template_statistics();

% 初始化消融实验配置（可选）
% global ABLATION_MODE;
% if isempty(ABLATION_MODE), ABLATION_MODE = 'full'; end
% ablation_config = matlab_ablation_config(ABLATION_MODE);

fprintf('✅ 性能监控已初始化\n');

%% ============================================================
%%  第2步：在主循环中添加计时
%% ============================================================

num_frames = 5000;  % 你的帧数

for frame_idx = 1:num_frames
    
    %% === Visual Processing ===
    perf_monitor.start('visual_processing');
    
    % 【你的原代码】
    % 例如：
    % img = imread(sprintf('image_%04d.png', frame_idx));
    % vt_result = process_visual_template(img);
    % visual_velocity = compute_visual_odometry(img, prev_img);
    
    perf_monitor.stop('visual_processing');
    
    %% === IMU Processing ===
    perf_monitor.start('imu_processing');
    
    % 【你的原代码】
    % 例如：
    % imu_data = load_imu_data(frame_idx);
    % imu_velocity = integrate_imu(imu_data);
    % imu_rotation = extract_rotation(imu_data);
    
    perf_monitor.stop('imu_processing');
    
    %% === Complementary Fusion ===
    perf_monitor.start('fusion');
    
    % 【你的原代码】
    % 例如：
    % fused_velocity = complementary_filter(visual_velocity, imu_velocity, alpha);
    % fused_rotation = complementary_filter(visual_rotation, imu_rotation, alpha);
    
    perf_monitor.stop('fusion');
    
    %% === Grid Cell Update ===
    perf_monitor.start('grid_cell_update');
    
    % 【你的原代码】
    % 例如：
    % POSECELL = update_3d_grid_cells(POSECELL, fused_velocity);
    
    perf_monitor.stop('grid_cell_update');
    
    %% === HDC Update ===
    perf_monitor.start('hdc_update');
    
    % 【你的原代码】
    % 例如：
    % HDC = update_hdc_network(HDC, fused_rotation);
    
    perf_monitor.stop('hdc_update');
    
    %% === Experience Map ===
    perf_monitor.start('experience_map');
    
    % 【你的原代码】
    % 例如：
    % [EXPS, NUM_EXPS] = update_experience_map(EXPS, NUM_EXPS, VT_ID, POSECELL);
    
    perf_monitor.stop('experience_map');
    
    %% === 更新统计 ===
    % 假设 VT_ID 和 VT_ID_COUNT 是全局变量
    % vt_stats.update(VT_ID, VT_ID_COUNT);
    
    %% === 进度显示（可选，但不要放在计时范围内）===
    if mod(frame_idx, 100) == 0
        fprintf('Progress: %d/%d frames\n', frame_idx, num_frames);
    end
end

%% ============================================================
%%  第3步：循环结束后生成报告
%% ============================================================

fprintf('\n========== 生成性能报告 ==========\n');

% 打印性能报告到控制台
perf_monitor.print_report();

% 生成LaTeX表格（可以直接复制到论文）
fprintf('\n========== LaTeX表格（复制到论文）==========\n');
perf_monitor.generate_latex_table();

% 打印模板统计
% vt_stats.print_report('Town01');

% 保存数据到MAT文件
dataset_name = 'Town01';  % 根据你的数据集修改
perf_monitor.save_report(sprintf('performance_%s.mat', dataset_name));
% vt_stats.save_statistics(sprintf('vt_stats_%s.mat', dataset_name));

fprintf('\n✅ 实验完成！数据已保存。\n');

%% ============================================================
%%  第4步：运行多个数据集（批处理脚本）
%% ============================================================

% 可以创建一个批处理脚本来运行所有数据集：

function run_all_experiments()
    datasets = {'Town01', 'Town02', 'Town10', 'MH_01', 'MH_03'};
    
    for i = 1:length(datasets)
        dataset = datasets{i};
        fprintf('\n\n============ Running %s ============\n', dataset);
        
        % 设置全局变量
        global DATASET_NAME;
        DATASET_NAME = dataset;
        
        % 运行主函数
        if contains(dataset, 'Town')
            test_imu_visual_fusion_slam;
        else
            test_euroc_fusion_slam;
        end
        
        fprintf('✅ %s 完成\n', dataset);
    end
    
    fprintf('\n\n🎉 所有实验完成！\n');
end

%% ============================================================
%%  第5步：消融实验（可选）
%% ============================================================

function run_ablation_experiments()
    modes = {'pure_visual', 'pure_imu', 'full'};
    
    for i = 1:length(modes)
        mode = modes{i};
        fprintf('\n\n============ Ablation Mode: %s ============\n', mode);
        
        % 设置消融模式
        global ABLATION_MODE;
        ABLATION_MODE = mode;
        
        % 运行实验
        test_imu_visual_fusion_slam;
        
        fprintf('✅ Mode %s 完成\n', mode);
    end
    
    fprintf('\n\n🎉 消融实验完成！\n');
end

%% ============================================================
%%  使用说明
%% ============================================================

% 1. 【一次性集成】将上面第1-3步的代码复制到你的主函数中
% 2. 【运行单个实验】直接运行你的主函数：
%    >> test_imu_visual_fusion_slam
% 3. 【批量运行】创建批处理脚本：
%    >> run_all_experiments()
% 4. 【消融实验】运行消融测试：
%    >> run_ablation_experiments()

%% 预期输出文件：
%  - performance_Town01.mat        % 性能数据
%  - vt_stats_Town01.mat           % 模板统计
%  - vt_growth_Town01.png          % 模板增长图
%  - （在控制台）LaTeX表格代码    % 直接复制到论文

fprintf('\n参考此示例修改你的主函数即可！\n');
