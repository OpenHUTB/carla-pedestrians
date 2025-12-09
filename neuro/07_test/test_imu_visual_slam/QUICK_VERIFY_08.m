%% 快速验证阈值0.08的效果
%  比0.09更快的验证脚本（只处理1000帧）

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║      快速验证 VT阈值 0.08                       ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

fprintf('【测试参数】\n');
fprintf('  VT阈值: 0.08\n');
fprintf('  处理帧数: 1000帧（超快验证）\n');
fprintf('  预期时间: ~35秒\n\n');

fprintf('【预期结果】（1000帧）\n');
fprintf('  VT数量: 60-70个\n');
fprintf('  经验节点: 85-100个\n');
fprintf('  如果在此范围→完整测试应该接近299个VT\n\n');

response = input('是否开始快速验证？(y/n): ', 's');

if ~strcmpi(response, 'y')
    fprintf('\n已取消。\n');
    return;
end

fprintf('\n正在运行...\n');
fprintf('════════════════════════════════════════════════\n\n');

% 设置快速模式
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 1000;

% 运行测试
tic;
test_imu_visual_fusion_slam;
elapsed = toc;
clear global FAST_TEST_MODE FAST_TEST_FRAMES;

fprintf('\n════════════════════════════════════════════════\n');
fprintf('  验证完成！\n');
fprintf('════════════════════════════════════════════════\n\n');

% 读取结果
global NUM_VT NUM_EXPS;

if ~isempty(NUM_VT) && ~isempty(NUM_EXPS)
    fprintf('【实际结果】（1000帧）\n');
    fprintf('  VT数量: %d\n', NUM_VT);
    fprintf('  经验节点: %d\n', NUM_EXPS);
    fprintf('  运行时间: %.1f秒\n\n', elapsed);
    
    % 估算5000帧的结果
    vt_5000 = NUM_VT * 4.8;  % 经验系数（非线性增长）
    exp_5000 = NUM_EXPS * 4.2;
    
    fprintf('【估算完整结果】（5000帧）\n');
    fprintf('  预计VT数量: ~%.0f个\n', vt_5000);
    fprintf('  预计经验节点: ~%.0f个\n', exp_5000);
    fprintf('  预计时间: ~%.0f秒\n\n', elapsed * 5.5);
    
    % 判断
    vt_ok = (NUM_VT >= 60 && NUM_VT <= 70);
    exp_ok = (NUM_EXPS >= 85 && NUM_EXPS <= 100);
    
    fprintf('【诊断结果】\n');
    if vt_ok
        fprintf('  ✅ VT数量在预期范围\n');
    elseif NUM_VT < 60
        fprintf('  ⚠️ VT数量偏少（%d < 60）\n', NUM_VT);
        fprintf('     估算5000帧: %.0f个（目标299）\n', vt_5000);
        fprintf('     建议：降低阈值到0.075\n');
    else
        fprintf('  ⚠️ VT数量偏多（%d > 70）\n', NUM_VT);
        fprintf('     估算5000帧: %.0f个（目标299）\n', vt_5000);
        fprintf('     建议：提高阈值到0.085\n');
    end
    
    if exp_ok
        fprintf('  ✅ 经验节点在预期范围\n');
    else
        fprintf('  ⚠️ 经验节点: %d（预期85-100）\n', NUM_EXPS);
    end
    
    fprintf('\n');
    
    % 给出下一步建议
    if vt_ok && exp_ok
        fprintf('╔════════════════════════════════════════════════╗\n');
        fprintf('║  ✨ 配置看起来不错！                            ║\n');
        fprintf('╚════════════════════════════════════════════════╝\n\n');
        
        fprintf('【下一步】\n');
        fprintf('  运行完整测试以确认：\n');
        fprintf('    >> RUN_ENHANCED_VT_SLAM\n\n');
        
        fprintf('  如果完整测试结果：\n');
        fprintf('    • VT = 280-320 → 完美 ✅\n');
        fprintf('    • RMSE < 135m → 成功 ✅\n');
        
    elseif NUM_VT < 60
        fprintf('╔════════════════════════════════════════════════╗\n');
        fprintf('║  ⚠️ VT数量偏少，需要调低阈值                    ║\n');
        fprintf('╚════════════════════════════════════════════════╝\n\n');
        
        fprintf('【建议操作】\n');
        fprintf('  1. 修改 test_imu_visual_fusion_slam.m\n');
        fprintf('     第72行：0.08 → 0.075\n\n');
        fprintf('  2. 重新运行本验证脚本\n');
        fprintf('     >> QUICK_VERIFY_08\n\n');
        
    elseif NUM_VT > 70
        fprintf('╔════════════════════════════════════════════════╗\n');
        fprintf('║  ⚠️ VT数量偏多，需要调高阈值                    ║\n');
        fprintf('╚════════════════════════════════════════════════╝\n\n');
        
        fprintf('【建议操作】\n');
        fprintf('  1. 修改 test_imu_visual_fusion_slam.m\n');
        fprintf('     第72行：0.08 → 0.085\n\n');
        fprintf('  2. 重新运行本验证脚本\n');
        fprintf('     >> QUICK_VERIFY_08\n\n');
        
    else
        fprintf('╔════════════════════════════════════════════════╗\n');
        fprintf('║  ⚠️ 结果接近但可能需要微调                      ║\n');
        fprintf('╚════════════════════════════════════════════════╝\n\n');
        
        fprintf('【建议】\n');
        fprintf('  运行完整测试观察实际效果：\n');
        fprintf('    >> RUN_ENHANCED_VT_SLAM\n\n');
    end
    
else
    fprintf('⚠️ 无法读取结果，请检查测试是否成功运行。\n\n');
end

fprintf('════════════════════════════════════════════════\n\n');
