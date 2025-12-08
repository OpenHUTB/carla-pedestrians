# SLAM系统快捷启动脚本

## 📋 快捷脚本列表

### 🗺️ SLAM完整测试（仿真场景）

| 脚本名称 | 数据集 | 功能 |
|---------|--------|------|
| `RUN_SLAM_TOWN01.m` | Town01 | 运行Town01的完整SLAM测试（5000帧，IMU融合） |
| `RUN_SLAM_TOWN10.m` | Town10 | 运行Town10的完整SLAM测试（5000帧，IMU融合） |

### 🏢 SLAM真实场景测试

| 脚本名称 | 数据集 | 功能 |
|---------|--------|------|
| `RUN_REAL_CARPARK.m` | QUT Carpark | 真实停车场SLAM测试（10935帧，纯视觉） |

### 🔬 消融实验

| 脚本名称 | 数据集 | 功能 |
|---------|--------|------|
| `RUN_ABLATION_TOWN01.m` | Town01 | 运行Town01的消融实验 |
| `RUN_ABLATION_TOWN10.m` | Town10 | 运行Town10的消融实验 |

---

## 🚀 使用方法

在MATLAB命令窗口中，直接输入脚本名称即可：

### Town01完整测试流程：
```matlab
% 1. SLAM测试
RUN_SLAM_TOWN01

% 2. 消融实验
RUN_ABLATION_TOWN01
```

### Town10完整测试流程：
```matlab
% 1. SLAM测试
RUN_SLAM_TOWN10

% 2. 消融实验
RUN_ABLATION_TOWN10
```

### 真实场景测试：
```matlab
% QUT Carpark停车场（10935帧，纯视觉SLAM）
RUN_REAL_CARPARK
```

---

## 📊 输出文件位置

### Town01结果：
- **SLAM结果：** `data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results/`
- **消融结果：** `data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/ablation_results/`

### Town10结果：
- **SLAM结果：** `data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/slam_results/`
- **消融结果：** `data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/ablation_results/`

### QUT Carpark结果（真实场景）：
- **SLAM结果：** `DATASETS/01_NeuroSLAM_Datasets/03_QUTCarparkData/slam_results/`

---

## 📁 生成的文件

### SLAM测试输出（仿真场景：Town01/Town10）：
- 7张PNG图表（轨迹对比、精度分析、统计图）
- `performance_report.txt` - 性能报告
- `trajectories.mat` - MATLAB数据文件
- `odo_trajectory.txt`, `exp_trajectory.txt` - 轨迹数据

### SLAM测试输出（真实场景：QUT Carpark）：
- `carpark_slam_trajectories.png` - 轨迹对比图
- `carpark_slam_report.txt` - 详细性能报告
- `carpark_slam_results.mat` - MATLAB数据文件
- `odo_trajectory.txt`, `exp_trajectory.txt` - 轨迹数据

### 消融实验输出（每个数据集）：
- 6张PNG图表（综合对比、性能轮廓、热力图等）
- 3种格式表格（.md, .html, .csv）
- LaTeX论文素材（.tex文件）

---

## ⚙️ 配置说明

所有测试都使用 **Plan B最优配置**：
- VT阈值：0.06
- 全局调制权重：0.15
- 特征融合权重：[0.20, 0.30, 0.30, 0.20]
- LSTM遗忘门：0.5

---

## 💡 提示

- 所有脚本都会自动 `clear all; close all; clc;`
- 不需要手动设置全局变量
- 可以连续运行不同数据集的测试
- 每次运行会覆盖之前的结果（注意备份）
