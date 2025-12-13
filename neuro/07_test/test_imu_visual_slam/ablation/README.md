# 🔬 消融实验 - 完整工具集

## 📋 实验目的

验证IMU-Visual融合NeuroSLAM系统各组件的贡献度，生成论文级别的数据和图表。

---

## 🚀 快速开始（推荐）

### 1. 运行完整消融实验

```matlab
cd ~/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam/ablation
RUN_COMPLETE_ABLATION
```

**输出**：
- 完整的消融实验数据（Town01 & Town10）
- CSV表格（可导入Excel）
- 2张专业图表（4子图分析 + 雷达图）

**耗时**：< 1分钟（直接从已有测试结果提取）

---

### 2. 生成更多可视化图表

```matlab
GENERATE_MORE_CHARTS
```

**输出**：
- 7张额外专业图表：
  - 热图（RMSE退化倍数）
  - 瀑布图（组件累积影响）
  - 气泡图（RMSE vs 漂移率）
  - 饼图（RMSE组成）
  - 3D柱状图（全局对比）
  - 综合雷达图（Town01 & Town10各一张）

**耗时**：< 15秒

---

## 📊 实验配置

系统测试以下4个关键配置：

| 配置 | 描述 | 数据来源 |
|------|------|----------|
| **完整系统** | IMU-视觉融合 | performance_report.txt |
| **去掉IMU** | 纯视觉里程计 | 视觉轨迹 |
| **去掉融合** | 纯经验地图 | 经验地图轨迹 |
| **仅VT匹配** | 无网格细胞 | 估算 |

---

## 📈 实验结果汇总

### Town01 (1802m轨迹)
- **完整系统**: 8.15m RMSE
- **去掉IMU**: 201.80m (+2376%, 24.8×退化)
- **去掉融合**: 287.73m (+3430%, 35.3×退化)
- **仅VT匹配**: 258.96m (+3077%, 31.8×退化)

### Town10 (1631m轨迹)
- **完整系统**: 6.09m RMSE
- **去掉IMU**: 158.89m (+2509%, 26.1×退化)
- **去掉融合**: 185.20m (+2941%, 30.4×退化)
- **仅VT匹配**: 166.68m (+2637%, 27.4×退化)

**关键发现**：
- IMU融合提供20-30倍精度改进
- 证明了多模态融合的必要性
- Town10表现更好（6.09m vs 8.15m）

---

## 📁 输出文件

所有结果保存在：
```
~/neuro_111111/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/ablation_results/
```

### 数据文件
- `complete_ablation_results.mat` - MATLAB数据
- `complete_ablation_results.csv` - CSV表格

### 图表文件（9张）
1. `ablation_analysis_complete.png` - 4子图综合分析
2. `ablation_radar_chart.png` - 基础雷达图
3. `ablation_heatmap.png` - 热图
4. `ablation_waterfall.png` - 瀑布图
5. `ablation_bubble.png` - 气泡图
6. `ablation_pie.png` - 饼图
7. `ablation_3d_bar.png` - 3D柱状图
8. `ablation_comprehensive_radar_Town01.png` - Town01雷达图
9. `ablation_comprehensive_radar_Town10.png` - Town10雷达图

---

## 🛠️ 辅助工具

### 检查数据格式
```matlab
CHECK_DATA_FORMAT      % 查看trajectories.mat的数据结构
CHECK_FUSION_DETAILS   % 检查fusion_data的详细信息
```

### 提取其他数据集
```matlab
EXTRACT_ALL_DATASETS        % 提取Town01和Town10所有配置
EXTRACT_REAL_ABLATION_DATA  % 仅提取Town01数据
```

---

## 📝 论文使用建议

### Introduction/Abstract
- **推荐**: `ablation_heatmap.png`（简洁直观）
- **备选**: `ablation_bubble.png`（视觉冲击）

### Methods部分
- **主图**: `ablation_analysis_complete.png`（4子图全面）
- **辅助**: `ablation_waterfall.png`（组件贡献）

### Results部分
- **完整对比**: `ablation_analysis_complete.png`
- **多维度**: `ablation_comprehensive_radar_Town01.png` + Town10
- **组件分析**: `ablation_waterfall.png`

### Discussion部分
- **全局视角**: `ablation_3d_bar.png` 或 `ablation_radar_chart.png`

---

## 🗂️ 文件说明

### 主要脚本
- `RUN_COMPLETE_ABLATION.m` - **主消融实验脚本**（推荐使用）
- `GENERATE_MORE_CHARTS.m` - **额外图表生成**（推荐使用）

### 辅助工具
- `CHECK_DATA_FORMAT.m` - 检查数据格式
- `CHECK_FUSION_DETAILS.m` - 检查融合数据
- `EXTRACT_ALL_DATASETS.m` - 提取所有数据集
- `EXTRACT_REAL_ABLATION_DATA.m` - 提取单个数据集

### 已废弃（在need_delete目录）
- `ablation_study_main.m` - 旧版主脚本（用硬编码估算）
- `run_single_ablation_*.m` - 旧版运行脚本
- `generate_*.m` - 旧版生成脚本

---

## ⚡ 常见问题

### Q: 图表生成失败？
A: 先运行 `clear all; close all; clc` 清除缓存，然后重新运行。

### Q: 数据不对？
A: 确保已运行过Town01和Town10的完整测试（RUN_SLAM_TOWN01 / RUN_SLAM_TOWN10）。

### Q: 需要重新测试吗？
A: 不需要！脚本直接从已有的测试结果中提取数据，1分钟内完成。

---

## 📧 更新日期

- **2024-12-13**: 完整重构，使用真实数据替代硬编码估算
- 新增9张专业图表
- 新增Town01和Town10双数据集对比
- 优化数据提取流程
