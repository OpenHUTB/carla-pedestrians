# 🔬 NeuroSLAM 消融实验完整流程

## 📊 实验概述

### 目的
验证NeuroSLAM各组件贡献：Visual Templates、IMU Fusion、Experience Map

### 配置（3个）
1. **Complete System** = VT + IMU + ExpMap (最优)
2. **w/o Experience Map** = VT + IMU (中等)
3. **w/o IMU Fusion** = VT only (最差)

### 数据集（2个）
- **Town01**: 城市场景 1802m
- **MH_03**: 室内场景 127m

---

## 🔄 完整流程（4个步骤）

### Step 1: 运行SLAM生成轨迹数据

#### 位置
```bash
cd ~/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/core
matlab
```

#### 执行
```matlab
>> test_imu_visual_fusion_slam
```

#### 输入文件（每个数据集）
```
Town01Data_IMU_Fusion/
├── fusion_pose.txt       [N×4] timestamp,x,y,z
├── ground_truth.txt      [N×4] timestamp,x,y,z
├── visual_odometry.txt   [N×7] timestamp,dx,dy,dz,qw,qx,qy,qz
└── aligned_imu.txt       [N×7] timestamp,ax,ay,az,wx,wy,wz
```

#### 输出文件
```
slam_results/trajectories.mat (Town01) 或
slam_results/euroc_trajectories.mat (MH_03)

包含变量：
- exp_trajectory     [N×3]  配置1: Complete System
- imu_aided_traj     [N×3]  配置2: w/o Experience Map
- pure_visual_traj   [N×3]  配置3: w/o IMU Fusion
- gt_data.pos        [N×3]  Ground Truth
- fusion_data.pos    [N×3]  EKF结果（不用于消融）
```

#### 生成原理
```
每一帧：
1. 读取VO → 累积 → pure_visual_traj (配置3)
2. 读取IMU → IMU+VO融合 → imu_aided_traj (配置2)
3. 视觉模板匹配 → 经验地图 → 闭环修正 → exp_trajectory (配置1)
```

---

### Step 2: 运行消融实验计算指标

#### 位置
```bash
cd ~/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/ablation
matlab
```

#### 执行
```matlab
>> RUN_ABLATION
```

#### 调用的函数
```matlab
compute_metrics_with_alignment(traj, gt, gt_length)
```

#### 函数功能
```matlab
function [rmse, final_error, drift_rate, traj_aligned] = ...
    compute_metrics_with_alignment(traj, gt, gt_length)
    
% 1. 裁剪到相同长度
min_len = min(size(traj,1), size(gt,1));

% 2. Procrustes对齐（7-DoF：平移+旋转+缩放）
[~, ~, transform] = procrustes(gt(1:100,:), traj(1:100,:), 'Scaling', true);

% 3. 应用变换到全轨迹
traj_aligned = transform.b * traj * transform.T + transform.c;

% 4. 计算指标
rmse = sqrt(mean(sum((traj_aligned - gt).^2, 2)));
final_error = norm(traj_aligned(end,:) - gt(end,:));
drift_rate = (final_error / gt_length) * 100;
```

#### 输出文件
```
ablation_results/
├── ablation_results_aligned.mat  (MATLAB结构体)
└── ablation_results_aligned.csv  (表格数据)
```

#### MAT文件内容
```matlab
all_results.Town01.Complete.rmse = 202.96
all_results.Town01.Complete.drift_rate = 5.72
all_results.Town01.No_ExpMap.rmse = 262.06
all_results.Town01.No_IMU.rmse = 326.19
all_results.MH_03.Complete.rmse = 4.28
...
```

---

### Step 3: 生成可视化图表

#### 执行
```matlab
>> GENERATE_ABLATION
```

#### 生成3类图表

**图1: 柱状图（主图）**
```matlab
% ablation_main_figure.png
bar(ablation_rmse_matrix)
颜色: 绿(Complete) 橙(w/o ExpMap) 红(w/o IMU)
```

**图2: 退化百分比图**
```matlab
% ablation_degradation_figure.png
degradation = (rmse - baseline) / baseline * 100
bar(degradation)
```

**图3: 雷达图**
```matlab
% ablation_radar_*.png (每个数据集一个)
polarplot(norm_rmse, norm_drift, norm_final_error)
外圈=好，内圈=差
```

#### 输出文件
```
ablation_results/
├── ablation_main_figure.png
├── ablation_degradation_figure.png
├── ablation_radar_Town01.png
└── ablation_radar_MH_03.png
```

---

### Step 4: 结果分析

#### 典型结果（Town01）
```
Configuration        RMSE(m)  Drift(%)  退化
─────────────────────────────────────────
Complete System      202.96   5.72      基准
w/o Experience Map   262.06   19.21     +29%
w/o IMU Fusion       326.19   31.12     +61%
```

#### 结论
- Experience Map贡献: 29%
- IMU Fusion额外贡献: 32% (61%-29%)
- 两个组件都很重要

---

## 🛠️ 关键代码位置

### SLAM主程序
```
文件: core/test_imu_visual_fusion_slam.m
行号: 
- 347: pure_visual_traj累积
- 354: imu_aided_traj生成
- 360: exp_trajectory来自EXPERIENCES
- 420: 保存trajectories.mat
```

### 消融实验脚本
```
文件: ablation/RUN_ABLATION.m
核心逻辑:
- 62-66行: 配置1 Complete
- 69-81行: 配置2 w/o ExpMap
- 83-96行: 配置3 w/o IMU
- 88-101行: 计算组件贡献
```

### 图表生成脚本
```
文件: ablation/GENERATE_ABLATION.m
- 52-106行: 图1柱状图
- 108-157行: 图2退化图
- 159-232行: 图3雷达图
```

---

## ❓ 常见问题

**Q: 为什么要对齐轨迹？**
A: SLAM有尺度不确定性，对齐后才能公平比较轨迹形状精度

**Q: MH_03为什么差异小？**
A: 短距离(127m)，误差累积少，组件贡献不明显

**Q: 如何验证结果正确？**
A: Complete应该最优，w/o IMU应该最差

**Q: 如何添加更多配置？**
A: 需要修改SLAM代码生成新的轨迹变量

---

## 📝 快速命令总结

```bash
# 完整流程（在MATLAB中）
cd core
test_imu_visual_fusion_slam  % 10-30分钟

cd ../ablation
RUN_ABLATION                  % 1-2分钟
GENERATE_ABLATION        % 1分钟

# 查看结果
cd ../../data/01_NeuroSLAM_Datasets/ablation_results
ls -lh *.png *.csv
```
