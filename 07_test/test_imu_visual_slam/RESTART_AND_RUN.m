%% 重启MATLAB并运行测试的说明
%
% 由于MATLAB函数缓存问题，修改后的函数不会自动重新加载。
% 
% === 请按以下步骤操作 ===
%
% 1. 完全退出MATLAB
%    在命令窗口输入: quit
%    或者: 文件 -> 退出
%
% 2. 重新启动MATLAB
%
% 3. 运行以下命令:
%    cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
%    test_imu_visual_slam_hart_cornet
%
% === 已修复的内容 ===
%
% visual_template_neuro_matlab_only.m（成功方法）已恢复到验证配置：
%   ✓ ClipLimit: 0.03 -> 0.02
%   ✓ 添加 imgaussfilt(0.5)
%   ✓ 完整流程：adapthisteq(0.02) -> imgaussfilt(0.5) -> Sobel -> 融合 -> 归一化
%
% test_imu_visual_slam_hart_cornet.m 配置：
%   ✓ USE_HART_CORNET = false（使用成功方法）
%   ✓ VT_MATCH_THRESHOLD = 0.07
%
% === 预期结果 ===
%
%   VT数量：     ~299个（不是5个！）
%   经验节点：   ~426个（不是185个！）
%   RMSE：       ~126米（不是297米！）
%   处理时间：   ~189秒
%
% === 如果VT还是5个 ===
%
% 说明MATLAB仍然在使用缓存的旧版本。请：
%   1. 确认已完全退出MATLAB（不是关闭窗口，是退出进程）
%   2. 重新启动MATLAB
%   3. 不要运行任何其他脚本，直接运行测试
%
% =====================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('⚠️  重要提示\n');
fprintf('========================================\n');
fprintf('由于MATLAB函数缓存，您需要：\n');
fprintf('\n');
fprintf('1. 退出MATLAB：\n');
fprintf('   >> quit\n');
fprintf('\n');
fprintf('2. 重新启动MATLAB\n');
fprintf('\n');
fprintf('3. 运行测试：\n');
fprintf('   >> cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam\n');
fprintf('   >> test_imu_visual_slam_hart_cornet\n');
fprintf('\n');
fprintf('========================================\n');
fprintf('预期：VT ~299个，RMSE ~126米\n');
fprintf('========================================\n');
fprintf('\n');

% 提示用户
answer = input('是否现在退出MATLAB？(y/n): ', 's');
if strcmpi(answer, 'y')
    fprintf('正在退出MATLAB...\n');
    fprintf('请重新启动后运行测试。\n');
    pause(1);
    quit;
else
    fprintf('请手动退出MATLAB后重新启动。\n');
end
