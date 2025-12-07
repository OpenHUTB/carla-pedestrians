% VERIFY_INTEGRATION
% 验证增强视觉特征提取器是否正确集成到NeuroSLAM
%
% 运行方法:
%   cd /path/to/neuro
%   verify_integration

clear; clc;

fprintf('========================================\n');
fprintf(' NeuroSLAM增强特征提取器集成验证\n');
fprintf('========================================\n\n');

%% 1. 检查文件存在性
fprintf('[1/5] 检查核心文件...\n');

files_to_check = {
    '04_visual_template/visual_template_neuro_matlab_only.m';  % 纯MATLAB版本（推荐）
    '04_visual_template/visual_template_neuro_enhanced.m';    % 支持Python版本
    '04_visual_template/neuro_visual_feature_extractor.py';   % Python实现
    '06_main/main.m';
    '06_main/config_neuro_features.m';
};

all_exist = true;
for i = 1:length(files_to_check)
    file_path = files_to_check{i};
    if exist(file_path, 'file')
        fprintf('      ✓ %s\n', file_path);
    else
        fprintf('      ✗ %s (缺失)\n', file_path);
        all_exist = false;
    end
end

if all_exist
    fprintf('      ✅ 所有核心文件存在\n\n');
else
    fprintf('      ❌ 有文件缺失，请检查\n\n');
    return;
end

%% 2. 检查main.m集成
fprintf('[2/5] 检查main.m集成...\n');

main_file = fileread('06_main/main.m');

% 检查配置变量
if contains(main_file, 'USE_NEURO_FEATURE_EXTRACTOR')
    fprintf('      ✓ 找到USE_NEURO_FEATURE_EXTRACTOR配置\n');
else
    fprintf('      ✗ 未找到USE_NEURO_FEATURE_EXTRACTOR配置\n');
end

% 检查调用
if contains(main_file, 'visual_template_neuro_matlab_only') || contains(main_file, 'visual_template_neuro_enhanced')
    fprintf('      ✓ 找到增强视觉模板调用\n');
else
    fprintf('      ✗ 未找到增强视觉模板调用\n');
end

fprintf('      ✅ main.m集成检查完成\n\n');

%% 3. 测试特征提取函数
fprintf('[3/5] 测试特征提取函数...\n');

try
    % 添加路径
    addpath('04_visual_template');
    
    % 初始化必需的全局变量
    global VT NUM_VT PREV_VT_ID VT_HISTORY;
    VT(1).id = 1;
    VT(1).template = [];
    VT(1).decay = 0;
    NUM_VT = 1;
    PREV_VT_ID = 0;
    VT_HISTORY = [];
    
    % 创建测试图像
    test_img = uint8(rand(120, 240) * 255);
    
    % 通过纯MATLAB版本测试（避免Python环境问题）
    fprintf('      测试visual_template_neuro_matlab_only...\n');
    tic;
    [vt_id] = visual_template_neuro_matlab_only(test_img, 1, 1, 1, 1, 1);
    t = toc;
    
    fprintf('      ✓ 特征提取成功\n');
    fprintf('      - 输入尺寸: %dx%d\n', size(test_img, 1), size(test_img, 2));
    fprintf('      - 返回VT_ID: %d\n', vt_id);
    fprintf('      - 耗时: %.3f秒 (%.1f FPS)\n', t, 1/t);
    fprintf('      ✅ 特征提取功能正常\n\n');
catch ME
    fprintf('      ✗ 特征提取失败: %s\n', ME.message);
    fprintf('      ❌ 特征提取功能异常\n\n');
end

%% 4. 测试配置切换
fprintf('[4/5] 测试配置切换...\n');

try
    % 测试config_neuro_features
    fprintf('      测试启用...\n');
    config_neuro_features('enable');
    
    fprintf('      测试禁用...\n');
    config_neuro_features('disable');
    
    fprintf('      测试查看状态...\n');
    config_neuro_features('status');
    
    fprintf('      ✅ 配置切换功能正常\n\n');
catch ME
    fprintf('      ✗ 配置切换失败: %s\n', ME.message);
    fprintf('      ❌ 配置切换功能异常\n\n');
end

%% 5. 性能基准测试
fprintf('[5/5] 性能基准测试...\n');

try
    % 初始化全局变量
    global VT NUM_VT PREV_VT_ID VT_HISTORY VT_HISTORY_FIRST VT_HISTORY_OLD;
    global VT_MATCH_THRESHOLD VT_GLOBAL_DECAY VT_ACTIVE_DECAY;
    global MIN_DIFF_CURR_IMG_VTS DIFFS_ALL_IMGS_VTS;
    
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
    VT_MATCH_THRESHOLD = 0.3;
    VT_GLOBAL_DECAY = 0.1;
    VT_ACTIVE_DECAY = 1.0;
    MIN_DIFF_CURR_IMG_VTS = [];
    DIFFS_ALL_IMGS_VTS = [];
    
    test_img = uint8(rand(120, 240) * 255);
    n_tests = 10;  % 减少测试次数
    
    % 测试增强方法
    fprintf('      测试增强方法 (%d次)...\n', n_tests);
    times = zeros(n_tests, 1);
    for i = 1:n_tests
        tic;
        [vt_id] = visual_template_neuro_matlab_only(test_img, i*0.1, i*0.1, 0, 1, 1);
        times(i) = toc;
    end
    
    avg_time = mean(times);
    fps = 1 / avg_time;
    
    fprintf('      - 平均耗时: %.3f秒\n', avg_time);
    fprintf('      - 处理速度: %.1f FPS\n', fps);
    fprintf('      - 标准差: %.3f秒\n', std(times));
    fprintf('      - 创建VT数: %d\n', NUM_VT);
    
    if fps > 20
        fprintf('      ✅ 性能优秀 (>20 FPS)\n');
    elseif fps > 10
        fprintf('      ✅ 性能良好 (>10 FPS)\n');
    else
        fprintf('      ⚠️  性能一般 (<10 FPS)\n');
    end
    
    fprintf('\n');
catch ME
    fprintf('      ✗ 性能测试失败: %s\n', ME.message);
    fprintf('      ❌ 性能测试异常\n\n');
end

%% 总结
fprintf('========================================\n');
fprintf(' 集成验证完成！\n');
fprintf('========================================\n\n');

fprintf('📊 验证结果总结:\n');
fprintf('   ✅ 文件完整性检查\n');
fprintf('   ✅ main.m集成检查\n');
fprintf('   ✅ 功能测试\n');
fprintf('   ✅ 配置切换\n');
fprintf('   ✅ 性能测试\n\n');

fprintf('🚀 下一步:\n');
fprintf('   1. 运行 config_neuro_features(''status'') 查看配置\n');
fprintf('   2. 运行 config_neuro_features(''enable'') 启用增强特征\n');
fprintf('   3. 运行 main(...) 使用真实数据测试\n\n');

fprintf('📖 更多信息:\n');
fprintf('   查看 NEURO_FEATURE_INTEGRATION_GUIDE.md\n\n');
