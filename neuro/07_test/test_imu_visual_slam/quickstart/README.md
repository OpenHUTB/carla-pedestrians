# 🚀 一键运行脚本

## Town数据集

```matlab
RUN_SLAM_TOWN01  % Town01城市街道 (1802m)
RUN_SLAM_TOWN10  % Town10城市街道 (1631m)
```

## EuRoC数据集

```matlab
RUN_SLAM_MH01   % MH_01_easy 室内大厅 (81m)
RUN_SLAM_MH03   % MH_03_medium 室内大厅 (127m)
```

## 使用说明

所有脚本会自动：
1. 加载数据
2. 运行SLAM测试
3. 生成轨迹对比图
4. 计算精度指标
5. 保存结果到 `slam_results/`

## 输出

- `trajectories.mat` / `euroc_trajectories.mat`: 轨迹数据
- `*_trajectory.png`: 轨迹对比图
- `*_error.png`: 误差分析图
