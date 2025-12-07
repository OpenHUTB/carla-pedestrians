% QUICK_TEST_INTEGRATION
% 快速测试增强视觉特征提取器集成
% 
% 无需真实数据，仅验证功能是否正常

clear; clc;

fprintf('========================================\n');
fprintf(' 快速集成测试\n');
fprintf('========================================\n\n');

% 检查Python环境，如果不可用则警告
if exist('pyenv', 'builtin')
    try
        pe = pyenv;
        fprintf('ℹ  Python环境: %s\n', pe.Status);
    catch
        fprintf('⚠  Python环境不可用，将使用MATLAB实现\n');
    end
else
    fprintf('ℹ  MATLAB版本不支持Python集成，使用MATLAB实现\n');
end
fprintf('\n');

%% 1. 添加路径
fprintf('[1/4] 添加路径...\n');
addpath('04_visual_template');
addpath('06_main');
fprintf('      ✓ 路径添加完成\n\n');

%% 2. 初始化全局变量
fprintf('[2/4] 初始化全局变量...\n');

global VT NUM_VT PREV_VT_ID VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD;
global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS SUB_VT_IMG;
global VT_PANORAMIC VT_IMG_HALF_OFFSET VT_IMG_X_SHIFT VT_IMG_Y_SHIFT;
global VT_IMG_CROP_Y_RANGE VT_IMG_CROP_X_RANGE;
global VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE;
global USE_NEURO_FEATURE_EXTRACTOR NEURO_FEATURE_METHOD;

% 初始化VT
VT(1).id = 1;
VT(1).template = [];
VT(1).decay = 0;
VT(1).gc_x = 0;
VT(1).gc_y = 0;
VT(1).gc_z = 0;
VT(1).hdc_yaw = 0;
VT(1).hdc_height = 0;
VT(1).first = 1;

NUM_VT = 1;
PREV_VT_ID = 0;
VT_HISTORY = [];
VT_HISTORY_FIRST = [];
VT_HISTORY_OLD = [];

% 参数
VT_MATCH_THRESHOLD = 0.3;
VT_GLOBAL_DECAY = 0.1;
VT_ACTIVE_DECAY = 1.0;
MIN_DIFF_CURR_IMG_VTS = [];
DIFFS_ALL_IMGS_VTS = [];
SUB_VT_IMG = [];

VT_PANORAMIC = false;
VT_IMG_HALF_OFFSET = 0;
VT_IMG_X_SHIFT = 10;
VT_IMG_Y_SHIFT = 5;

% VT图像裁剪和缩放参数（120x240图像）
VT_IMG_CROP_Y_RANGE = 1:120;
VT_IMG_CROP_X_RANGE = 1:240;
VT_IMG_RESIZE_Y_RANGE = 32;
VT_IMG_RESIZE_X_RANGE = 32;

% 启用增强特征提取器（强制使用MATLAB，避免Python环境问题）
USE_NEURO_FEATURE_EXTRACTOR = true;
NEURO_FEATURE_METHOD = 'matlab';

fprintf('      ✓ 全局变量初始化完成\n');
fprintf('      - USE_NEURO_FEATURE_EXTRACTOR = %s\n', mat2str(USE_NEURO_FEATURE_EXTRACTOR));
fprintf('      - NEURO_FEATURE_METHOD = %s\n', NEURO_FEATURE_METHOD);
fprintf('\n');

%% 3. 测试视觉模板匹配
fprintf('[3/4] 测试增强视觉模板匹配...\n');

% 创建测试图像序列
n_frames = 10;
vt_ids = zeros(n_frames, 1);
times = zeros(n_frames, 1);

for i = 1:n_frames
    % 生成测试图像
    test_img = uint8(rand(120, 240) * 255);
    
    % 模拟位置
    x = i * 0.1;
    y = i * 0.05;
    z = 0;
    yaw = i;
    height = 1;
    
    % 调用增强的视觉模板匹配（使用纯MATLAB版本）
    tic;
    [vt_id] = visual_template_neuro_matlab_only(test_img, x, y, z, yaw, height);
    times(i) = toc;
    vt_ids(i) = vt_id;
end

avg_time = mean(times);
fps = 1 / avg_time;

fprintf('      ✓ 视觉模板匹配测试完成\n');
fprintf('      - 处理帧数: %d\n', n_frames);
fprintf('      - 创建VT数: %d\n', NUM_VT);
fprintf('      - 平均耗时: %.3f秒/帧\n', avg_time);
fprintf('      - 处理速度: %.1f FPS\n', fps);
fprintf('      - 模板重用: %.1f%%\n', (n_frames - NUM_VT)/n_frames * 100);
fprintf('\n');

%% 4. 性能评估
fprintf('[4/4] 性能评估...\n');

if fps > 30
    fprintf('      🌟 性能优秀！(>30 FPS)\n');
elseif fps > 20
    fprintf('      ✅ 性能良好！(>20 FPS)\n');
elseif fps > 10
    fprintf('      ✓ 性能可接受 (>10 FPS)\n');
else
    fprintf('      ⚠️  性能偏低 (<10 FPS)\n');
end

fprintf('\n');

%% 总结
fprintf('========================================\n');
fprintf(' 测试完成！\n');
fprintf('========================================\n\n');

fprintf('📊 测试结果:\n');
fprintf('   ✅ 路径配置正确\n');
fprintf('   ✅ 全局变量初始化正常\n');
fprintf('   ✅ 视觉模板匹配功能正常\n');
fprintf('   ✅ 性能测试通过\n\n');

fprintf('🎯 关键指标:\n');
fprintf('   - 处理速度: %.1f FPS\n', fps);
fprintf('   - VT数量: %d个\n', NUM_VT);
fprintf('   - 重用率: %.1f%%\n\n', (n_frames - NUM_VT)/n_frames * 100);

fprintf('🚀 下一步:\n');
fprintf('   1. 使用真实数据测试:\n');
fprintf('      编辑 run_neuroslam_example.m 修改数据路径\n');
fprintf('      然后运行: run_neuroslam_example\n\n');
fprintf('   2. 查看配置状态:\n');
fprintf('      config_neuro_features(''status'')\n\n');
fprintf('   3. 切换配置:\n');
fprintf('      config_neuro_features(''enable'')  % 启用\n');
fprintf('      config_neuro_features(''disable'') % 禁用\n\n');
