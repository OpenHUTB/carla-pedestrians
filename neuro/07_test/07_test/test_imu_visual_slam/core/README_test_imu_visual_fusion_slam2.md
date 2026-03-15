# test_imu_visual_fusion_slam2.m - 使用说明

## 📋 脚本概述

`test_imu_visual_fusion_slam2.m` 是一个**公平对比实验脚本**，用于对比两种NeuroSLAM方法的性能：

- **Baseline**: 原始NeuroSLAM（纯视觉）
- **Ours**: IMU-Visual Fusion NeuroSLAM（IMU辅助）

## 🎯 核心功能

### 方法对比

| 组件 | Baseline (NeuroSLAM) | Ours (IMU-Visual Fusion) |
|------|---------------------|-------------------------|
| **视觉里程计** | 纯视觉 (Scanline Intensity Profile) | IMU辅助 (互补滤波融合) |
| **视觉模板特征** | Patch Normalization | HART+Transformer双流 |
| **距离度量** | SAD (Sum of Absolute Differences) | 余弦距离 |
| **注意力机制** | 无 | Spatial + Self-Attention |
| **时序建模** | 无 | LSTM门控 |
| **IMU融合** | 无 | 互补滤波 (α_yaw=0.012) |

### 输出结果

1. **精度指标**：
   - ATE (Absolute Trajectory Error)
   - RMSE (Root Mean Square Error)
   - Max Error

2. **可视化图表**：
   - 轨迹对比图 (GT vs Baseline vs Ours)
   - 误差曲线图
   - 方法对比图
   - 详细分析图（9子图）

3. **统计数据**：
   - VT（视觉模板）数量
   - 经验节点数量
   - 改进百分比

---

## 🚀 快速开始

### 基本用法

```matlab
% 切换到脚本目录
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');

% 清除状态
clear functions;
clearvars -global;

% 设置数据集（可选）
global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';  % 默认

% 运行脚本
test_imu_visual_fusion_slam2
```

---

## 📊 支持的数据集

### CARLA模拟数据集

```matlab
% Town01 (默认) - 已测试 ✅
DATASET_NAME = 'Town01Data_IMU_Fusion';

% Town02 - 待测试 🔄
DATASET_NAME = 'Town02Data_IMU_Fusion';

% Town10 - 待测试 🔄
DATASET_NAME = 'Town10Data_IMU_Fusion';
```

### 真实数据集

```matlab
% KITTI Odometry
DATASET_NAME = 'KITTI_07';

% EuRoC MAV
DATASET_NAME = 'MH_01_easy';
DATASET_NAME = 'MH_03_medium';
```

---

## ⚙️ 全局变量配置

### 快速测试模式

```matlab
global FAST_TEST_MODE FAST_TEST_FRAMES;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 2500;  % 限制测试帧数
```

### 数据集选择

```matlab
global DATASET_NAME;
DATASET_NAME = 'Town01Data_IMU_Fusion';
```

---

## 📁 数据集目录结构

脚本会自动在以下位置查找数据：

```
neuro/data/
├── Town01Data_IMU_Fusion/
│   ├── fusion_pose.txt          # IMU融合位姿
│   ├── imu_data.txt              # IMU数据
│   ├── ground_truth.txt          # Ground Truth
│   └── images/                   # 图像序列
│
├── KITTI_07/
│   ├── KITTI_07/
│   │   ├── fusion_pose.txt
│   │   ├── aligned_imu.txt
│   │   ├── ground_truth.txt
│   │   └── image_0/
│
└── MH_01_easy/
    ├── MH_01_easy/
    │   ├── fusion_pose.txt
    │   ├── ground_truth.txt
    │   └── mav0/cam0/data/
```

---

## 📈 完整测试示例

### Town01 测试 ✅

```matlab
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');
clear functions; clearvars -global;

global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 5000;
DATASET_NAME = 'Town01Data_IMU_Fusion';

test_imu_visual_fusion_slam2
```

**预期结果**：
- 运行时间：~1-2小时
- ATE改进：+8.92%
- 结果保存：`data/Town01Data_IMU_Fusion/comparison_results/`

### Town02 测试 🔄

```matlab
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');
clear functions; clearvars -global;

global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 5000;
DATASET_NAME = 'Town02Data_IMU_Fusion';

test_imu_visual_fusion_slam2
```

**预期结果**：
- 运行时间：~1-2小时
- ATE改进：+8-10%（预期，与Town01类似）
- 结果保存：`data/Town02Data_IMU_Fusion/comparison_results/`

### Town10 测试 🔄

```matlab
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');
clear functions; clearvars -global;

global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 5000;
DATASET_NAME = 'Town10Data_IMU_Fusion';

test_imu_visual_fusion_slam2
```

**预期结果**：
- 运行时间：~1-2小时
- ATE改进：+8-10%（预期，与Town01类似）
- 结果保存：`data/Town10Data_IMU_Fusion/comparison_results/`

### KITTI 测试

```matlab
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');
clear functions; clearvars -global;

global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 1100;
DATASET_NAME = 'KITTI_07';

test_imu_visual_fusion_slam2
```

**预期结果**：
- 运行时间：~30-45分钟
- ATE改进：+0.73%
- 结果保存：`data/KITTI_07/KITTI_07/comparison_results/`

### EuRoC MH_01 测试

```matlab
cd('E:\Neuro_end\neuro\07_test\07_test\test_imu_visual_slam\core');
clear functions; clearvars -global;

global FAST_TEST_MODE FAST_TEST_FRAMES DATASET_NAME;
FAST_TEST_MODE = true;
FAST_TEST_FRAMES = 3000;
DATASET_NAME = 'MH_01_easy';

test_imu_visual_fusion_slam2
```

**预期结果**：
- 运行时间：~60分钟
- ATE改进：+12.79% ⭐（最佳结果）
- 结果保存：`data/MH_01_easy/comparison_results/`

---

## 📂 输出文件

测试完成后，结果保存在 `data/<DATASET_NAME>/comparison_results/` 目录：

```
comparison_results/
├── comparison_results.mat           # MATLAB数据文件
├── comparison_report.txt            # 文本报告
├── trajectory_comparison.png        # 轨迹对比图
├── error_comparison.png             # 误差曲线图
├── method_comparison.png            # 方法对比图
├── detailed_comparison.png          # 详细分析图（9子图）
├── baseline/
│   └── fusion_comparison_with_gt.png
└── ours/
    └── fusion_comparison_with_gt.png
```

---

## 🔧 脚本执行流程

### 10步流程

1. **[1/10] 添加依赖路径**
   - 自动添加所有必需的MATLAB路径

2. **[2/10] 读取数据**
   - 加载融合位姿、IMU数据、Ground Truth、图像

3. **[3/10] 初始化公共参数**
   - 设置SLAM参数（网格大小、阈值等）

4. **[4/10] 初始化Baseline模块**
   - 配置原始NeuroSLAM（Patch Normalization）

5. **[5/10] 运行Baseline SLAM**
   - 执行纯视觉SLAM，保存结果

6. **[6/10] 初始化Ours模块**
   - 配置IMU-Visual Fusion（HART+Transformer）

7. **[7/10] 运行Ours SLAM**
   - 执行IMU辅助SLAM，保存结果

8. **[8/10] 轨迹对齐**
   - 使用Sim(3)对齐轨迹到Ground Truth

9. **[9/10] 计算精度指标**
   - 计算ATE、RMSE、改进百分比

10. **[10/10] 生成对比可视化**
    - 生成所有对比图表

---

## ⚠️ 注意事项

### 1. 数据集准备

确保数据集已正确放置在 `neuro/data/` 目录下，并包含以下文件：
- `fusion_pose.txt`
- `ground_truth.txt`
- 图像序列

### 2. 内存要求

- 推荐：16GB RAM
- 最低：8GB RAM（使用FAST_TEST_MODE）

### 3. 运行时间

| 数据集 | 帧数 | 预计时间 | 状态 |
|--------|------|----------|------|
| Town01 | 5000 | 1-2小时 | ✅ 已完成 |
| Town02 | 5000 | 1-2小时 | 🔄 待测试 |
| Town10 | 5000 | 1-2小时 | 🔄 待测试 |
| KITTI_07 | 1100 | 30-45分钟 | ✅ 已完成 |
| MH_01_easy | 3000 | 60分钟 | ✅ 已完成 |
| MH_03_medium | 1876 | 50分钟 | ✅ 已完成 |

### 4. 常见问题

**Q: 脚本卡住不动？**
- A: 检查是否有MATLAB窗口需要关闭，或使用 `close all` 清理

**Q: 图片保存失败？**
- A: 脚本已使用 `print(gcf, ...)` 替代 `saveas`，兼容旧版MATLAB

**Q: 内存不足？**
- A: 减少 `FAST_TEST_FRAMES` 的值，或关闭其他程序

---

## 📊 已完成的测试结果

| 数据集 | 场景 | IMU质量 | Baseline ATE | Ours ATE | 改进 | 状态 |
|--------|------|---------|--------------|----------|------|------|
| **Town01** | 模拟城市 | 100Hz | 3.85m | 3.51m | **+8.92%** | ✅ |
| **Town02** | 模拟城市 | 100Hz | - | - | **待测试** | 🔄 |
| **Town10** | 模拟城市 | 100Hz | - | - | **待测试** | 🔄 |
| **KITTI_07** | 真实街道 | 10Hz | 71.15m | 70.63m | **+0.73%** | ✅ |
| **MH_01_easy** | 室内飞行 | 200Hz | 3.79m | 3.31m | **+12.79%** ⭐ | ✅ |
| **MH_03_medium** | 室内飞行 | 200Hz | 2.94m | 2.95m | **-0.50%** | ✅ |

**注**: Town02和Town10预期改进约+8-10%（与Town01类似，相同的CARLA模拟环境）

---

## 🔬 技术细节

### Baseline方法

- **视觉里程计**：Scanline Intensity Profile
- **视觉模板**：Patch Normalization (16x16 patches)
- **距离度量**：SAD (Sum of Absolute Differences)
- **阈值**：VT_MATCH_THRESHOLD = 0.10

### Ours方法

- **视觉里程计**：IMU辅助（互补滤波）
  - α_trans = 0.7 (平移)
  - α_yaw = 0.012 (偏航，网格搜索优化)
  - α_height = 0.5 (高度)

- **视觉模板**：HART+Transformer
  - Gabor滤波器组（8方向 × 5尺度）
  - 双流架构（Dorsal + Ventral）
  - Spatial Attention
  - Self-Attention (Transformer)
  - LSTM门控时序建模

- **距离度量**：余弦距离
- **阈值**：VT_MATCH_THRESHOLD = 0.10（与Baseline相同）

---

## 📚 相关文档

- `四数据集实验完成总结.md` - 完整实验结果总结
- `MH数据集完整测试指南.md` - EuRoC数据集测试指南
- `KITTI测试完成总结.md` - KITTI数据集测试总结
- `多数据集对比测试完成总结.md` - 多数据集对比分析

---

## 🤝 贡献者

- NeuroSLAM System Copyright (C) 2018-2019
- IMU-Visual Fusion Comparison Test (2024)

---

## 📧 联系方式

如有问题或建议，请查看相关文档或联系项目维护者。

---

**最后更新**: 2024年（根据实验完成时间）
