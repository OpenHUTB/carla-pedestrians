# 实验数据补充方案

本目录包含论文实验数据补充的完整方案和代码模板。

## 📁 文件说明

### 1. **PAPER_ISSUES_AND_FIXES.md** 
论文问题详细分析
- 10个具体问题的识别
- 每个问题的修复方案
- 优先级分类（🔴必须/🟡建议/🟢可选）

### 2. **QUICK_FIX_GUIDE.md** ⭐ **从这里开始**
3小时快速修复指南
- 最小修改方案（立即可用）
- 逐步操作指南
- 修改前后对比

### 3. **EXPERIMENT_SUPPLEMENT_PLAN.md**
完整实验补充计划
- 7个实验的详细设计
- 预期数据表格式
- 时间表（2-5天）

### 4. **experiment_templates.py**
即插即用的代码模板
- `PerformanceMonitor` - 性能监控（解决FPS问题）
- `AblationConfig` - 消融实验模式切换
- `TemplateStatistics` - 模板统计收集
- `ExperimentRecorder` - 结果记录和LaTeX生成

---

## 🚀 快速开始

### **最快方案（3小时）**：

```bash
# 1. 查看快速修复指南
cat QUICK_FIX_GUIDE.md

# 2. 复制代码模板到你的项目
cp experiment_templates.py ../../your_project/

# 3. 按照指南修改你的代码
```

### **完整方案（2-3天）**：

```bash
# 查看完整实验计划
cat EXPERIMENT_SUPPLEMENT_PLAN.md
```

---

## 📊 主要问题清单

### 🔴 必须修改（影响投稿）：
1. ❌ 21 FPS缺少数据 → 添加性能测试
2. ❌ "15-20%"不准确 → 改为"~15%"
3. ❌ "多种天气"没测试 → 删除或补充实验
4. ❌ 消融实验不完整 → 添加Pure Vision/IMU对比

### 🟡 强烈建议：
5. 64×数据只来自Town01 → 补充其他场景
6. 基线不够 → 添加更多对比方法

### 🟢 可选：
7. 天气变化实验
8. Bias验证实验

---

## 💡 使用示例

### Python版本 - 添加性能监控到你的代码：

```python
from experiment_templates import PerformanceMonitor

# 初始化
perf = PerformanceMonitor()

# 在你的主循环中
for frame in dataset:
    perf.start('visual_processing')
    visual_result = process_visual(frame)
    perf.end('visual_processing')
    
    perf.start('imu_processing')
    imu_result = process_imu(frame)
    perf.end('imu_processing')
    
    # ... 其他模块

# 运行结束后
perf.print_report()  # 打印性能报告
perf.save_to_file()  # 保存为JSON
```

### ⭐ MATLAB版本 - 快速集成指南：

#### 已创建的MATLAB工具：

1. **`matlab_performance_monitor.m`** - 性能监控类
2. **`matlab_ablation_config.m`** - 消融实验配置
3. **`matlab_template_statistics.m`** - 模板统计
4. **`example_integration.m`** - 集成示例
5. **`HOW_TO_INTEGRATE_MATLAB.md`** ⭐ **详细集成指南**

#### 快速开始（3步）：

```bash
# 1. 复制工具到你的项目
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/experiment_supplement
cp matlab_*.m ../test_imu_visual_slam/utils/

# 2. 查看集成指南
cat HOW_TO_INTEGRATE_MATLAB.md

# 3. 参考example_integration.m修改你的主函数
```

#### MATLAB集成示例：

```matlab
% 在主函数开头
perf_monitor = matlab_performance_monitor();
vt_stats = matlab_template_statistics();

% 在主循环中
for frame_idx = 1:num_frames
    perf_monitor.start('visual_processing');
    % ... 你的visual代码 ...
    perf_monitor.stop('visual_processing');
    
    perf_monitor.start('imu_processing');
    % ... 你的IMU代码 ...
    perf_monitor.stop('imu_processing');
    
    % 更新统计
    vt_stats.update(VT_ID, VT_ID_COUNT);
end

% 循环结束后
perf_monitor.print_report();
perf_monitor.generate_latex_table();  % 生成LaTeX表格
vt_stats.print_report('Town01');
```

输出示例:
```
Performance Report
======================================================================
Component                 Mean(ms)     Std(ms)      Freq(Hz)     
----------------------------------------------------------------------
visual_processing            25.30        2.10         39.53
imu_processing                2.10        0.30        476.19
fusion                        3.80        0.50        263.16
...
----------------------------------------------------------------------
TOTAL PIPELINE               50.00        ---          20.00
======================================================================

========== Template Statistics: Town01 ==========
Total Frames:          5000
Final Template Count:  321
Unique VT Count:       321
Template Growth:       +316
=====================================================
```

---

## 📋 修改检查清单

在投稿前确认：

- [ ] Abstract中的数据claim已修改为保守说法
- [ ] 添加了性能测试表（FPS数据）
- [ ] 删除或验证了"多种天气"claim
- [ ] 补充了消融实验（至少Pure Visual对比）
- [ ] 所有数据都有来源和实验支持
- [ ] 统一了百分比数据（15% vs 15-20%）

---

## 📞 联系

如有问题，参考：
- `PAPER_ISSUES_AND_FIXES.md` - 详细问题分析
- `QUICK_FIX_GUIDE.md` - 快速操作指南
- `experiment_templates.py` 中的代码注释

---

**建议：先完成🔴红色必须项，再考虑其他改进！**

**预计3小时可完成最小可行修改，2-3天可完成完整补充。**
