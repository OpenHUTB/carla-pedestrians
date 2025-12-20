# ⚡ 消融实验快速参考

## 🚀 快速开始（3步）

```bash
# 在MATLAB中执行:

# Step 1: 生成SLAM轨迹 (10-30分钟)
cd core
test_imu_visual_fusion_slam

# Step 2: 运行消融实验 (1分钟)
cd ../ablation
RUN_ABLATION

# Step 3: 生成图表 (1分钟)
GENERATE_ABLATION
```

---

## 📁 关键文件位置

```
core/
└─ test_imu_visual_fusion_slam.m     ← SLAM主程序

ablation/
├─ RUN_ABLATION.m                    ← 消融实验脚本
├─ GENERATE_ABLATION.m          ← 图表生成
└─ compute_metrics_with_alignment.m  ← 评估函数

data/01_NeuroSLAM_Datasets/
├─ Town01Data_IMU_Fusion/
│  ├─ [输入数据 .txt]
│  └─ slam_results/trajectories.mat  ← SLAM输出
│
└─ ablation_results/
   ├─ ablation_results_aligned.mat   ← 实验结果
   ├─ ablation_results_aligned.csv
   └─ *.png                          ← 图表
```

---

## 🔍 数据流向

```
输入 → SLAM → 轨迹 → 评估 → 结果 → 图表

.txt → test_imu_visual_fusion_slam.m
         ↓
     trajectories.mat
         ↓
     RUN_ABLATION.m
         ↓
     ablation_results_aligned.mat
         ↓
     GENERATE_ABLATION.m
         ↓
     *.png
```

---

## 📊 3个配置

| # | 配置名 | 组件 | 变量名 | 预期 |
|---|--------|------|--------|------|
| 1 | Complete | VT+IMU+ExpMap | exp_trajectory | 最优 |
| 2 | w/o ExpMap | VT+IMU | imu_aided_traj | 中等 |
| 3 | w/o IMU | VT | pure_visual_traj | 最差 |

---

## 📈 关键指标

```matlab
RMSE (m)        - 平均轨迹误差
Final Error (m) - 终点误差  
Drift Rate (%)  - 漂移率 = Final Error / 轨迹长度 * 100
```

---

## 🎨 3类图表

1. **柱状图** (ablation_main_figure.png)
   - 直观显示3个配置的RMSE
   - 绿(最优) 橙(中) 红(最差)

2. **退化图** (ablation_degradation_figure.png)
   - 显示去掉组件后性能下降%
   - 量化组件贡献

3. **雷达图** (ablation_radar_*.png)
   - 多指标综合对比
   - 外圈=好，内圈=差

---

## 🔧 核心函数

### compute_metrics_with_alignment()
```matlab
输入: traj [N×3], gt [N×3], gt_length (scalar)
处理:
  1. 裁剪到相同长度
  2. Procrustes对齐 (前100帧)
  3. 应用变换到全轨迹
  4. 计算RMSE/Drift/EndError
输出: rmse, final_error, drift_rate, traj_aligned
```

---

## ✅ 检查清单

### 数据准备
- [ ] fusion_pose.txt 存在
- [ ] ground_truth.txt 存在
- [ ] visual_odometry.txt 存在
- [ ] aligned_imu.txt 存在

### SLAM运行
- [ ] trajectories.mat 生成
- [ ] 包含 exp_trajectory
- [ ] 包含 imu_aided_traj
- [ ] 包含 pure_visual_traj

### 消融实验
- [ ] ablation_results_aligned.mat 生成
- [ ] ablation_results_aligned.csv 生成
- [ ] Complete < w/o ExpMap < w/o IMU (RMSE)

### 图表生成
- [ ] ablation_main_figure.png 清晰
- [ ] ablation_degradation_figure.png 清晰
- [ ] ablation_radar_*.png 清晰

---

## 🐛 常见问题速查

| 问题 | 原因 | 解决 |
|------|------|------|
| 找不到trajectories.mat | SLAM未运行 | 先运行test_imu_visual_fusion_slam |
| RMSE都是NaN | 数据未对齐 | 检查轨迹长度，确保>100帧 |
| Complete不是最优 | 数据或代码问题 | 检查轨迹变量是否正确 |
| 图表字体太小 | 默认设置 | 已在GENERATE_ABLATION中设置大号字体 |
| MH_03差异小 | 正常现象 | 短距离场景，组件贡献有限 |

---

## 📖 典型结果

```
Town01 (1802m):
Configuration        RMSE    Drift   退化
─────────────────────────────────────────
Complete System      202.96  5.72%   基准
w/o Experience Map   262.06  19.21%  +29%
w/o IMU Fusion       326.19  31.12%  +61%

结论: Experience Map贡献29%, IMU贡献32%

MH_03 (127m):
Configuration        RMSE   Drift   退化
─────────────────────────────────────────
Complete System      4.28   0.15%   基准
w/o Experience Map   4.31   0.20%   +0.7%
w/o IMU Fusion       4.53   0.97%   +5.8%

结论: 短距离场景，组件贡献小但仍然正向
```

---

## 📝 论文写作提示

```latex
% 推荐章节结构
\section{Experiments}
\subsection{Experimental Setup}
  - 数据集介绍
  - 评估协议 (Procrustes对齐)

\subsection{Ablation Study}
  - 3个配置说明
  - 结果表格
  - 图1: ablation_main_figure.png
  - 分析组件贡献

\subsection{Discussion}
  - 长距离 vs 短距离
  - 组件协同作用
```

---

## 💡 下一步

### 如果要扩展实验:
1. 添加更多配置 (如"IMU only")
2. 视觉特征消融 (去掉HART/CORnet等)
3. 参数敏感性分析
4. 更多数据集验证

### 如果要优化结果:
1. 调整经验地图参数
2. 优化IMU融合权重
3. 改进闭环检测阈值
4. 增加轨迹平滑

---

## 🔗 相关文档

- `ABLATION_WORKFLOW.md` - 详细流程说明
- `WORKFLOW_DIAGRAM.md` - 流程图解
- `FIGURE_EXPLANATIONS.md` - 图表详解
- `ABLATION_CORRECTNESS_VERIFICATION.md` - 正确性验证
