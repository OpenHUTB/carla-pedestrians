# 🔄 消融实验流程图解

## 📊 数据流向图

```
┌─────────────────────────────────────────────────────────────────┐
│                   Step 1: SLAM数据生成阶段                       │
│              test_imu_visual_fusion_slam.m                      │
└─────────────────────────────────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   输入数据 (每个数据集目录)           │
         │                                      │
         │  • fusion_pose.txt                  │
         │  • ground_truth.txt                 │
         │  • visual_odometry.txt              │
         │  • aligned_imu.txt                  │
         └──────────────────────────────────────┘
                              ↓
         ┌─────────────────────────────────────────────┐
         │        SLAM处理（逐帧）                      │
         │                                             │
         │  Frame 1, 2, 3, ..., N                     │
         │    ├─ 读VO → 累积                          │
         │    │   → pure_visual_traj (配置3)          │
         │    │                                        │
         │    ├─ 读IMU → IMU+VO融合                   │
         │    │   → imu_aided_traj (配置2)            │
         │    │                                        │
         │    └─ 视觉模板匹配 → 经验地图 → 闭环       │
         │        → exp_trajectory (配置1)            │
         └─────────────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   输出文件                            │
         │                                      │
         │  slam_results/trajectories.mat       │
         │  ├─ exp_trajectory [N×3]            │
         │  ├─ imu_aided_traj [N×3]            │
         │  ├─ pure_visual_traj [N×3]          │
         │  ├─ gt_data.pos [N×3]               │
         │  └─ fusion_data.pos [N×3]           │
         └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                 Step 2: 消融实验评估阶段                         │
│                      RUN_ABLATION.m                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
         ┌─────────────────────────────────────────────┐
         │  加载轨迹数据 (Town01 & MH_03)              │
         │                                             │
         │  load('trajectories.mat')                  │
         │  • exp_trajectory                          │
         │  • imu_aided_traj                          │
         │  • pure_visual_traj                        │
         │  • gt_data                                 │
         └─────────────────────────────────────────────┘
                              ↓
         ┌─────────────────────────────────────────────┐
         │  调用评估函数 (每个配置)                    │
         │                                             │
         │  compute_metrics_with_alignment()          │
         │    ├─ 1. 裁剪轨迹到相同长度                │
         │    ├─ 2. Procrustes对齐 (前100帧)         │
         │    ├─ 3. 应用变换到全轨迹                  │
         │    └─ 4. 计算RMSE/Drift/EndError          │
         └─────────────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   计算组件贡献                        │
         │                                      │
         │  ExpMap贡献 = (w/o ExpMap RMSE      │
         │               - Complete RMSE)      │
         │  IMU贡献    = (w/o IMU RMSE         │
         │               - w/o ExpMap RMSE)    │
         └──────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   输出文件                            │
         │                                      │
         │  ablation_results/                   │
         │  ├─ ablation_results_aligned.mat    │
         │  └─ ablation_results_aligned.csv    │
         └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                 Step 3: 可视化图表生成                           │
│                 GENERATE_ABLATION.m                        │
└─────────────────────────────────────────────────────────────────┘
                              ↓
         ┌─────────────────────────────────────────────┐
         │  加载实验结果                                │
         │                                             │
         │  load('ablation_results_aligned.mat')      │
         │  提取：                                     │
         │  • ablation_rmse_matrix [2×3]             │
         │  • ablation_drift_matrix [2×3]            │
         │  • ablation_final_error_matrix [2×3]      │
         └─────────────────────────────────────────────┘
                              ↓
         ┌─────────────────────────────────────────────┐
         │  生成3类图表                                 │
         │                                             │
         │  图1: 柱状图 (RMSE对比)                     │
         │    bar(rmse_matrix)                        │
         │    颜色: 绿(Complete)橙(w/o ExpMap)红(w/o IMU)│
         │                                             │
         │  图2: 退化图 (性能下降%)                     │
         │    degradation = (rmse-baseline)/baseline  │
         │    bar(degradation)                        │
         │                                             │
         │  图3: 雷达图 (多指标)                        │
         │    polarplot([RMSE, Drift, EndErr])       │
         │    每个数据集单独归一化                      │
         └─────────────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   输出图表                            │
         │                                      │
         │  ablation_results/                   │
         │  ├─ ablation_main_figure.png        │
         │  ├─ ablation_degradation_figure.png │
         │  ├─ ablation_radar_Town01.png       │
         │  └─ ablation_radar_MH_03.png        │
         └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                     Step 4: 结果分析                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
         ┌──────────────────────────────────────┐
         │   论文写作                            │
         │                                      │
         │  • 插入图表到LaTeX                   │
         │  • 撰写Ablation Study章节           │
         │  • 分析组件贡献                      │
         │  • 对比不同场景                      │
         └──────────────────────────────────────┘
```

---

## 🔍 关键函数调用链

### SLAM阶段调用关系
```
test_imu_visual_fusion_slam.m
├─ 读取数据文件
│  ├─ load fusion_pose.txt
│  ├─ load ground_truth.txt
│  ├─ load visual_odometry.txt
│  └─ load aligned_imu.txt
│
├─ 初始化轨迹
│  ├─ pure_visual_traj = zeros(N,3)
│  ├─ imu_aided_traj = zeros(N,3)
│  └─ exp_trajectory = zeros(N,3)
│
└─ 逐帧处理
   ├─ Frame 1, 2, ..., N
   │  ├─ 累积VO
   │  │  └─ pure_visual_traj(i,:) = pure_visual_traj(i-1,:) + delta_vo
   │  │
   │  ├─ IMU融合
   │  │  ├─ read aligned_imu.txt
   │  │  ├─ compute imu_delta = integrate(imu)
   │  │  └─ imu_aided_traj(i,:) = imu_aided_traj(i-1,:) + imu_delta
   │  │
   │  └─ 经验地图
   │     ├─ visual_template_matching()
   │     ├─ create_experience_node()
   │     ├─ loop_closure_detection()
   │     ├─ pose_graph_optimization()
   │     └─ exp_trajectory(i,:) = EXPERIENCES(CUR_EXP_ID).pos
   │
   └─ 保存结果
      └─ save('slam_results/trajectories.mat', ...)
```

### 消融实验阶段调用关系
```
RUN_ABLATION.m
├─ load trajectories.mat
│  ├─ exp_trajectory
│  ├─ imu_aided_traj
│  ├─ pure_visual_traj
│  └─ gt_data
│
├─ 配置1: Complete System
│  └─ compute_metrics_with_alignment(exp_trajectory, gt)
│     ├─ procrustes() → 对齐
│     └─ 计算 RMSE, Drift, EndError
│
├─ 配置2: w/o Experience Map
│  └─ compute_metrics_with_alignment(imu_aided_traj, gt)
│
├─ 配置3: w/o IMU Fusion
│  └─ compute_metrics_with_alignment(pure_visual_traj, gt)
│
├─ 计算组件贡献
│  ├─ exp_contrib = (cfg2_rmse - cfg1_rmse) / cfg2_rmse * 100
│  └─ imu_contrib = (cfg3_rmse - cfg2_rmse) / cfg3_rmse * 100
│
└─ 保存结果
   ├─ save ablation_results_aligned.mat
   └─ save ablation_results_aligned.csv
```

### 可视化阶段调用关系
```
GENERATE_ABLATION.m
├─ load ablation_results_aligned.mat
│
├─ 提取数据矩阵
│  ├─ ablation_rmse_matrix [2×3]
│  ├─ ablation_drift_matrix [2×3]
│  └─ ablation_final_error_matrix [2×3]
│
├─ 图1: 柱状图
│  ├─ figure()
│  ├─ bar(rmse_matrix)
│  ├─ 设置颜色: [绿, 橙, 红]
│  ├─ 添加数值标注
│  └─ saveas('ablation_main_figure.png')
│
├─ 图2: 退化图
│  ├─ figure()
│  ├─ degradation = (rmse - baseline) / baseline * 100
│  ├─ bar(degradation)
│  └─ saveas('ablation_degradation_figure.png')
│
└─ 图3: 雷达图 (for each dataset)
   ├─ figure()
   ├─ 归一化指标 (per-dataset)
   │  ├─ norm_rmse = 1 - (rmse - min) / (max - min)
   │  ├─ norm_drift = 1 - (drift - min) / (max - min)
   │  └─ norm_final = 1 - (final - min) / (max - min)
   ├─ polarplot([norm_rmse, norm_drift, norm_final])
   └─ saveas('ablation_radar_*.png')
```

---

## ⏱️ 时间估计

```
Step 1: SLAM生成        → 10-30分钟 (取决于数据集大小)
  Town01: ~15分钟
  MH_03: ~5分钟

Step 2: 消融实验计算    → 1-2分钟
  Procrustes对齐: 快
  指标计算: 快

Step 3: 图表生成        → 1分钟
  3类图表: 快速

总计: 约15-35分钟 (主要是SLAM运行时间)
```

---

## 💾 文件大小估计

```
输入数据 (每个数据集):
  fusion_pose.txt      ~500KB
  ground_truth.txt     ~500KB
  visual_odometry.txt  ~1MB
  aligned_imu.txt      ~2MB

输出数据:
  trajectories.mat     ~2MB
  ablation_results.mat ~50KB
  ablation_results.csv ~5KB
  图表 *.png          ~200KB each

总计: 约10MB per dataset
```
