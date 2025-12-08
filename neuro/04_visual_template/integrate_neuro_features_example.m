% INTEGRATE_NEURO_FEATURES_EXAMPLE
% 展示如何将增强的视觉特征提取器集成到现有NeuroSLAM系统
%
% 使用方法:
%   选项1: 直接替换visual_template.m
%   选项2: 在初始化时设置使用新的特征提取器
%   选项3: 混合模式（保留原始方法作为备份）

%% 选项1: 直接替换 (最简单)
% 
% 步骤:
% 1. 备份原始的visual_template.m
%    >> copyfile('visual_template.m', 'visual_template_backup.m');
%
% 2. 将visual_template_neuro_enhanced.m重命名为visual_template.m
%    >> movefile('visual_template_neuro_enhanced.m', 'visual_template.m');
%
% 3. 正常运行NeuroSLAM
%    >> cd ../06_main
%    >> main
%
% 优点: 无需修改其他代码
% 缺点: 失去了原始方法的快速访问


%% 选项2: 在初始化时设置 (推荐)
%
% 在初始化函数中添加配置选项

function example_option2_init()
    % 在neuro slam初始化函数中添加
    global NEURO_FEATURE_METHOD;
    global USE_NEURO_FEATURE_EXTRACTOR;
    
    % 设置使用增强的特征提取器
    USE_NEURO_FEATURE_EXTRACTOR = true;  % true=新方法, false=原始方法
    NEURO_FEATURE_METHOD = 'matlab';  % 'matlab' 或 'python'
    
    if USE_NEURO_FEATURE_EXTRACTOR
        method_name = '增强版';
    else
        method_name = '原始版';
    end
    fprintf('[配置] 视觉特征提取器: %s\n', method_name);
end


% 修改visual_template调用
function example_option2_usage()
    global USE_NEURO_FEATURE_EXTRACTOR;
    
    % 在主循环中
    if USE_NEURO_FEATURE_EXTRACTOR
        [vt_id] = visual_template_neuro_enhanced(rawImg, x, y, z, yaw, height);
    else
        [vt_id] = visual_template(rawImg, x, y, z, yaw, height);
    end
end


%% 选项3: 创建统一接口 (最灵活)

function [vt_id] = visual_template_unified(rawImg, x, y, z, yaw, height)
% VISUAL_TEMPLATE_UNIFIED 统一的视觉模板接口
%
% 自动选择最佳的特征提取方法
%
% 参数:
%   rawImg - 原始图像
%   x, y, z - Grid Cell位置
%   yaw, height - Head Direction信息
% 返回:
%   vt_id - 视觉模板ID

    global USE_NEURO_FEATURE_EXTRACTOR;
    global NEURO_FEATURE_METHOD;
    
    % 默认配置
    if isempty(USE_NEURO_FEATURE_EXTRACTOR)
        USE_NEURO_FEATURE_EXTRACTOR = true;  % 默认使用增强版
    end
    
    if isempty(NEURO_FEATURE_METHOD)
        NEURO_FEATURE_METHOD = 'matlab';  % 默认使用MATLAB实现
    end
    
    % 选择特征提取方法
    try
        if USE_NEURO_FEATURE_EXTRACTOR
            % 使用增强的特征提取器
            vt_id = visual_template_neuro_enhanced(rawImg, x, y, z, yaw, height);
        else
            % 使用原始方法
            vt_id = visual_template(rawImg, x, y, z, yaw, height);
        end
    catch ME
        % 如果新方法失败，回退到原始方法
        warning('增强特征提取失败，回退到原始方法: %s', ME.message);
        vt_id = visual_template(rawImg, x, y, z, yaw, height);
    end
end


%% 选项4: 性能对比模式

function example_option4_compare()
% 同时运行两种方法进行性能对比
%
% 适用于评估新方法的效果

    global VT VT_NEURO;  % 两套独立的视觉模板
    
    % 准备测试图像
    rawImg = imread('test_image.png');  % 替换为实际图像
    x = 0; y = 0; z = 0; yaw = 0; height = 0;
    
    % 原始方法
    tic;
    [vt_id_orig] = visual_template(rawImg, x, y, z, yaw, height);
    time_orig = toc;
    
    % 新方法
    tic;
    [vt_id_neuro] = visual_template_neuro_enhanced(rawImg, x, y, z, yaw, height);
    time_neuro = toc;
    
    % 对比结果
    fprintf('\n性能对比:\n');
    fprintf('  原始方法: VT_ID=%d, 耗时=%.3fs\n', vt_id_orig, time_orig);
    fprintf('  新方法:   VT_ID=%d, 耗时=%.3fs\n', vt_id_neuro, time_neuro);
    fprintf('  速度比:   %.2fx\n', time_orig / time_neuro);
end


%% 完整的集成示例

function complete_integration_example()
% 完整的集成示例 (修改main.m或类似的主函数)

    fprintf('=== NeuroSLAM 增强特征提取集成示例 ===\n\n');
    
    %% 1. 初始化配置
    fprintf('[1/4] 初始化配置...\n');
    
    % 全局变量配置
    global USE_NEURO_FEATURE_EXTRACTOR;
    global NEURO_FEATURE_METHOD;
    global VT NUM_VT VT_HISTORY;
    
    USE_NEURO_FEATURE_EXTRACTOR = true;
    NEURO_FEATURE_METHOD = 'matlab';
    
    % 初始化视觉模板存储
    VT = struct();
    NUM_VT = 1;
    VT_HISTORY = [];
    
    % ... (其他初始化代码)
    
    if USE_NEURO_FEATURE_EXTRACTOR
        method_desc = '增强版 (HART+CORnet)';
    else
        method_desc = '原始版';
    end
    fprintf('  特征提取方法: %s\n', method_desc);
    fprintf('  实现: %s\n', NEURO_FEATURE_METHOD);
    
    %% 2. 加载测试数据
    fprintf('[2/4] 加载测试数据...\n');
    
    % 创建模拟图像序列
    n_frames = 10;
    test_images = cell(n_frames, 1);
    for i = 1:n_frames
        % 创建测试图像
        test_images{i} = uint8(rand(120, 240) * 255);
    end
    
    fprintf('  加载了 %d 帧测试图像\n', n_frames);
    
    %% 3. 运行SLAM循环
    fprintf('[3/4] 运行SLAM循环...\n');
    
    for frame = 1:n_frames
        rawImg = test_images{frame};
        
        % 模拟Grid Cell和Head Direction输入
        x = frame * 0.1;
        y = frame * 0.05;
        z = 0;
        yaw = frame * 0.1;
        height = 0;
        
        % 调用视觉模板匹配
        [vt_id] = visual_template_unified(rawImg, x, y, z, yaw, height);
        
        fprintf('  帧 %d: VT_ID = %d\n', frame, vt_id);
    end
    
    fprintf('  总共创建了 %d 个视觉模板\n', NUM_VT);
    
    %% 4. 结果统计
    fprintf('[4/4] 结果统计...\n');
    
    fprintf('  视觉模板数量: %d\n', NUM_VT);
    fprintf('  平均匹配率: %.2f%%\n', ...
        (n_frames - NUM_VT) / n_frames * 100);
    
    % 可视化视觉模板
    if NUM_VT > 1
        figure('Name', 'Visual Templates', 'Position', [100, 100, 1200, 400]);
        n_show = min(NUM_VT - 1, 5);
        for i = 1:n_show
            subplot(1, n_show, i);
            imshow(VT(i+1).template, []);
            title(sprintf('VT %d', i+1));
        end
    end
    
    fprintf('\n=== 集成示例完成 ===\n');
end


%% 参数调优建议

function parameter_tuning_guide()
% 参数调优指南
%
% 关键参数说明:
%
% 1. VT_MATCH_THRESHOLD (视觉模板匹配阈值)
%    - 原始方法: 通常 0.1 - 0.3
%    - 新方法: 通常 0.2 - 0.5 (因为使用余弦相似度)
%    - 调优: 阈值越高，创建的模板越多
%
% 2. 特征维度
%    - feature_dim: 128-512
%    - 维度越高，表达能力越强，但计算越慢
%
% 3. 注意力机制
%    - use_attention = true: 适合复杂场景
%    - use_attention = false: 适合简单场景，更快
%
% 4. 时序整合
%    - use_temporal = true: 适合视频流，更平滑
%    - use_temporal = false: 适合独立图像，更快

    fprintf('\n=== 参数调优建议 ===\n\n');
    
    fprintf('场景1: 室内场景 (简单, 结构化)\n');
    fprintf('  USE_NEURO_FEATURE_EXTRACTOR = true\n');
    fprintf('  NEURO_FEATURE_METHOD = ''matlab''\n');
    fprintf('  use_attention = false\n');
    fprintf('  use_temporal = false\n');
    fprintf('  feature_dim = 128\n');
    fprintf('  VT_MATCH_THRESHOLD = 0.3\n\n');
    
    fprintf('场景2: 室外场景 (复杂, 光照变化大)\n');
    fprintf('  USE_NEURO_FEATURE_EXTRACTOR = true\n');
    fprintf('  NEURO_FEATURE_METHOD = ''python''  %% Python版本更强大\n');
    fprintf('  use_attention = true\n');
    fprintf('  use_temporal = true\n');
    fprintf('  feature_dim = 256\n');
    fprintf('  VT_MATCH_THRESHOLD = 0.4\n\n');
    
    fprintf('场景3: 实时应用 (需要高速)\n');
    fprintf('  USE_NEURO_FEATURE_EXTRACTOR = true\n');
    fprintf('  NEURO_FEATURE_METHOD = ''matlab''\n');
    fprintf('  use_attention = false\n');
    fprintf('  use_temporal = false\n');
    fprintf('  feature_dim = 64\n');
    fprintf('  VT_MATCH_THRESHOLD = 0.25\n\n');
end


%% 主入口

if ~isdeployed
    fprintf('NeuroSLAM增强特征提取器 - 集成示例\n');
    fprintf('========================================\n\n');
    
    fprintf('可用的示例:\n');
    fprintf('  1. example_option2_init()     - 初始化配置\n');
    fprintf('  2. example_option4_compare()  - 性能对比\n');
    fprintf('  3. complete_integration_example() - 完整示例\n');
    fprintf('  4. parameter_tuning_guide()   - 参数调优指南\n\n');
    
    fprintf('运行完整示例...\n\n');
    complete_integration_example();
    
    fprintf('\n参数调优建议:\n');
    parameter_tuning_guide();
end
