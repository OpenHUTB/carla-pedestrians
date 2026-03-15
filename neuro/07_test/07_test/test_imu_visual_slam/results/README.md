# 📊 测试结果目录

这个目录用于存放**单次测试和实验**的输出结果。

> **注意**: `compare_datasets.m` 的4场景对比结果输出到 `~/datasets/comparison_results/`，因为它对比的是多个数据集，放在数据集目录更合理。

## 📁 目录结构

```
results/
├── figures/     # 图表输出（轨迹图、误差图等）
└── tables/      # 数据表格（指标CSV、LaTeX表格等）
```

---

## 🎨 figures/ - 图表输出

存放各种可视化图表：

### 来自核心测试脚本
- `*_trajectory_comparison.png`: 轨迹对比图（Town/EuRoC单次测试）
- `*_error_analysis.png`: 误差分析图
- `*_3d_trajectory.png`: 3D轨迹图
- `*_statistics.png`: 统计图表

### 来自消融实验
- `ablation_comparison_*.png`: 消融实验对比图
- `performance_bars_*.png`: 性能柱状图

### 来自VT对比
- `vt_methods_comparison.png`: VT方法对比
- `hart_features_visualization.png`: HART特征可视化

---

## �️ 输出位置设计说明

### 本目录 (results/)
用于**单次测试和实验**的结果：
- ✅ Town01/Town10/MH01/MH03 单次SLAM测试
- ✅ 消融实验（单数据集多配置对比）
- ✅ VT方法对比实验
- ✅ 特征分析图表

### ~/datasets/comparison_results/
用于**跨数据集对比**的结果（来自 `compare_datasets.m`）：
- ✅ `radar_performance.png`: 4数据集雷达图
- ✅ `heatmap_accuracy.png`: 精度热图
- ✅ `bubble_improvement.png`: 改进倍数气泡图
- ✅ `comparison_report.txt`: 对比报告

**设计原因**: 
- 4场景对比是数据集级别的分析，与数据集目录在一起更合理
- 避免混入单次测试结果
- 方便论文写作时快速找到对比图表

---

## � tables/ - 数据表格

存放数值结果表格：

### CSV格式
- `slam_metrics_*.csv`: SLAM精度指标
- `performance_comparison_*.csv`: 性能对比数据
- `ablation_results_*.csv`: 消融实验结果

### LaTeX格式
- `*.tex`: 用于论文的LaTeX表格

### MATLAB格式
- `*.mat`: MATLAB数据文件（轨迹、指标等）

---

## 💡 使用说明

### 单次测试（输出到 results/）
运行Town或EuRoC单次测试：

```matlab
% 运行单次测试
cd quickstart/
RUN_SLAM_TOWN01

% 结果自动保存到当前数据集的 slam_results/
% 例如: ~/neuro_111111/.../Town01Data_IMU_Fusion/slam_results/
% - trajectories.mat
% - town01_trajectory.png
% - town01_error.png
```

### 跨数据集对比（输出到 ~/datasets/comparison_results/）
运行4场景性能对比：

```matlab
% 运行4数据集对比
cd core/
compare_datasets

% 结果自动保存到 ~/datasets/comparison_results/
% - radar_performance.png
% - heatmap_accuracy.png
% - bubble_improvement.png
% - comparison_report.txt
```

### 消融实验（输出到 results/）
运行消融实验：

```matlab
cd ablation/
RUN_ABLATION_TOWN01

% 结果保存到 ablation/results/
```

---

## 🗂️ 建议的命名规范

### 图表文件
```
{数据集}_{类型}_{日期}.png

示例:
- town01_trajectory_20241213.png
- mh01_error_analysis_20241213.png
- comparison_radar_20241213.png
```

### 数据表格
```
{实验类型}_{数据集}_{日期}.csv

示例:
- metrics_town01_20241213.csv
- ablation_town10_20241213.csv
- comparison_4datasets_20241213.csv
```

---

## 📦 版本管理

### .gitignore 建议
```gitignore
# 忽略临时测试结果
results/figures/*.png
results/tables/*.csv

# 但保留重要的最终结果
!results/figures/final_*.png
!results/tables/paper_*.csv
```

### 清理旧结果
```bash
# 清理30天前的结果
find results/ -name "*.png" -mtime +30 -delete
find results/ -name "*.csv" -mtime +30 -delete
```

---

## 📊 当前状态

```bash
# 查看结果文件
ls -lh results/figures/
ls -lh results/tables/

# 统计图表数量
find results/figures/ -name "*.png" | wc -l

# 统计表格数量
find results/tables/ -name "*.csv" | wc -l
```

---

## ✅ 最佳实践

1. **定期备份重要结果** - 用于论文/报告的关键图表
2. **使用描述性文件名** - 包含数据集、日期、指标
3. **分类存储** - figures/和tables/分开
4. **版本标注** - 重要结果加版本号（v1, v2）
5. **清理临时文件** - 避免目录过大

---

**注意**: 这个目录下的文件通常**不需要提交到Git**，除非是用于文档展示的关键结果图。
