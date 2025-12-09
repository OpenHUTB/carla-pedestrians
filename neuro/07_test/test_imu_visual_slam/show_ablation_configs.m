%% 显示所有消融实验配置
% 用于快速查看和理解各个消融实验的配置

clear; clc;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        消融实验配置总览                                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 定义所有配置
configs = {
    % 名称,           IMU, LSTM, Trans, Dual, Attn, Full,  说明
    '完整系统',       true, true, true,  true, true, true,  'Baseline - 所有组件都启用'
    '去掉IMU',       false, true, true,  true, true, true,  '纯视觉SLAM，不使用IMU传感器'
    '去掉LSTM',      true, false, true,  true, true, true,  '无时序记忆，只处理当前帧'
    '去掉Transformer', true, true, false, true, true, true,  '无全局上下文，只有局部特征'
    '去掉双流',      true, true, true,  false, true, true,  '单一特征流，无Dorsal/Ventral'
    '去掉注意力',    true, true, true,  true, false, true,  '处理全图，无空间聚焦'
    '简化特征',      true, false, false, false, false, false, '基础方法对比（adapthisteq）'
};

%% 显示表格
fprintf('┌──────────────────┬─────┬──────┬────────┬──────┬────────┬────────┬────────────────────────────┐\n');
fprintf('│ 配置名称         │ IMU │ LSTM │ Trans  │ 双流 │ 注意力 │ 完整特征│ 说明                       │\n');
fprintf('├──────────────────┼─────┼──────┼────────┼──────┼────────┼────────┼────────────────────────────┤\n');

for i = 1:size(configs, 1)
    name = configs{i, 1};
    imu = configs{i, 2};
    lstm = configs{i, 3};
    trans = configs{i, 4};
    dual = configs{i, 5};
    attn = configs{i, 6};
    full = configs{i, 7};
    desc = configs{i, 8};
    
    % 格式化名称（对齐）
    name_padded = pad(name, 18, 'right', '　');
    
    % 格式化描述（截断）
    if length(desc) > 28
        desc = [desc(1:25), '...'];
    else
        desc = pad(desc, 28, 'right');
    end
    
    fprintf('│ %s│  %s  │  %s   │   %s   │  %s  │   %s   │   %s   │ %s │\n', ...
        name_padded, ...
        bool2icon(imu), ...
        bool2icon(lstm), ...
        bool2icon(trans), ...
        bool2icon(dual), ...
        bool2icon(attn), ...
        bool2icon(full), ...
        desc);
    
    if i < size(configs, 1)
        fprintf('├──────────────────┼─────┼──────┼────────┼──────┼────────┼────────┼────────────────────────────┤\n');
    end
end

fprintf('└──────────────────┴─────┴──────┴────────┴──────┴────────┴────────┴────────────────────────────┘\n');

%% 显示组件说明
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        组件功能说明                                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('【IMU融合】\n');
fprintf('  · 功能: 融合加速度计和陀螺仪数据\n');
fprintf('  · 作用: 提供连续的运动估计，减少视觉漂移\n');
fprintf('  · 灵感: 人类前庭系统（感知加速和旋转）\n');
fprintf('\n');

fprintf('【LSTM时序建模】\n');
fprintf('  · 功能: 记忆历史帧的特征信息\n');
fprintf('  · 作用: 时序平滑，连贯性判断\n');
fprintf('  · 灵感: 大脑工作记忆（短期记忆保持）\n');
fprintf('\n');

fprintf('【Transformer全局上下文】\n');
fprintf('  · 功能: 全局-局部特征交互\n');
fprintf('  · 作用: 用全局统计调制局部特征\n');
fprintf('  · 灵感: 注意力机制中的全局信息整合\n');
fprintf('\n');

fprintf('【双流架构】\n');
fprintf('  · 功能: Dorsal（位置）+ Ventral（特征）\n');
fprintf('  · 作用: 分别处理"在哪里"和"是什么"\n');
fprintf('  · 灵感: 人类视觉皮层的双通路\n');
fprintf('\n');

fprintf('【空间注意力】\n');
fprintf('  · 功能: 高斯窗口聚焦重要区域\n');
fprintf('  · 作用: 突出显著特征，抑制背景\n');
fprintf('  · 灵感: 人类选择性注意机制\n');
fprintf('\n');

fprintf('【完整特征】\n');
fprintf('  · 功能: Gabor滤波 + 梯度 + 纹理\n');
fprintf('  · 作用: 丰富的多层次特征表达\n');
fprintf('  · 对比: 简化版只用adapthisteq\n');
fprintf('\n');

%% 显示预期结果
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        预期性能影响                                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('移除组件的预期影响（RMSE增加量）:\n');
fprintf('  1. IMU融合:       +10 ~ +20 m  (影响大，传感器融合关键)\n');
fprintf('  2. LSTM记忆:      +5 ~ +15 m   (影响中，时序平滑重要)\n');
fprintf('  3. Transformer:   +3 ~ +10 m   (影响中，全局信息有用)\n');
fprintf('  4. 双流架构:      +2 ~ +8 m    (影响小，但有区分价值)\n');
fprintf('  5. 空间注意力:    +1 ~ +5 m    (影响小，锦上添花)\n');
fprintf('  6. 完整特征:      +20 ~ +30 m  (vs简化baseline)\n');
fprintf('\n');

fprintf('预期VT数量变化:\n');
fprintf('  · 完整系统:       300 ~ 350\n');
fprintf('  · 去掉IMU:        280 ~ 330 (略少，视觉信息不够稳定)\n');
fprintf('  · 去掉LSTM:       250 ~ 300 (明显少，缺少时序判断)\n');
fprintf('  · 去掉Transformer: 270 ~ 320 (稍少)\n');
fprintf('  · 去掉双流:       280 ~ 330 (稍少)\n');
fprintf('  · 去掉注意力:     290 ~ 340 (影响小)\n');
fprintf('  · 简化特征:       280 ~ 330 (对比基准)\n');
fprintf('\n');

%% 显示运行建议
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        运行建议                                                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('【完整实验】（推荐用于论文）\n');
fprintf('  运行: RUN_ABLATION_STUDY\n');
fprintf('  时间: ~30-45分钟\n');
fprintf('  输出: 6张图表 + 3种表格 + LaTeX代码\n');
fprintf('\n');

fprintf('【快速测试】（用于调试和验证）\n');
fprintf('  运行: test_single_ablation\n');
fprintf('  时间: ~4-6分钟/配置\n');
fprintf('  用途: 测试单个配置是否正常工作\n');
fprintf('\n');

fprintf('【自定义实验】\n');
fprintf('  修改: ablation_study_main.m 中的 experiments 变量\n');
fprintf('  灵活: 可以添加任意组合的配置\n');
fprintf('\n');

%% 显示资源需求
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        系统需求                                                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('硬件需求:\n');
fprintf('  · 内存: >= 8 GB (推荐16 GB)\n');
fprintf('  · 磁盘: >= 5 GB 可用空间\n');
fprintf('  · CPU: 多核处理器（越多越快）\n');
fprintf('\n');

fprintf('软件需求:\n');
fprintf('  · MATLAB: R2018b 或更高版本\n');
fprintf('  · 工具箱: Image Processing Toolbox\n');
fprintf('  · 数据集: Town01 (5000帧)\n');
fprintf('\n');

fprintf('时间估算:\n');
fprintf('  · 单配置: 4-6 分钟\n');
fprintf('  · 全部7配置: 30-45 分钟\n');
fprintf('  · 生成图表: 2-3 分钟\n');
fprintf('  · 总计: ~40-50 分钟\n');
fprintf('\n');

%% 显示快速启动命令
fprintf('╔════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        快速启动                                                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('在MATLAB命令窗口中输入:\n\n');
fprintf('    >> cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam\n');
fprintf('    >> RUN_ABLATION_STUDY\n');
fprintf('\n');
fprintf('或者直接运行:\n\n');
fprintf('    >> ablation_study_main\n');
fprintf('\n');

fprintf('═══════════════════════════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% 辅助函数
function icon = bool2icon(val)
    if val
        icon = '✓';
    else
        icon = '✗';
    end
end
