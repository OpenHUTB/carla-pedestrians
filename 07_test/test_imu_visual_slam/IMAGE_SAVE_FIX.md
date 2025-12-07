# 图像保存路径修复总结

## 🎯 问题描述

**用户报告**: 测试不同场景时，应该保存4张图片，但有2张图片没有更新，仍然是之前Town10的图片。

---

## 🔍 问题原因

### 原问题分析

```matlab
% evaluate_slam_accuracy 被调用3次：
metrics_fusion_gt = evaluate_slam_accuracy(..., result_path);     % 第1次
metrics_exp_gt = evaluate_slam_accuracy(..., result_path);        % 第2次
metrics_odo_gt = evaluate_slam_accuracy(..., result_path);        # 第3次

% 但每次都保存相同文件名：
save_path = fullfile(save_dir, 'slam_accuracy_evaluation.png');  # ❌ 被覆盖
```

**结果**: 后两次调用覆盖了第一次的图片，最终只保留最后一个（Visual Odometry）的评估图。

### 应该生成的4张图

| 序号 | 文件名 | 内容 | 状态 |
|-----|--------|------|------|
| 1 | `imu_visual_slam_comparison.png` | 3D轨迹对比（所有方法） | ✅ 正常 |
| 2 | `slam_accuracy_imu_fusion.png` | IMU-Fusion精度评估 | ❌ 被覆盖 |
| 3 | `slam_accuracy_experience_map.png` | 经验地图精度评估 | ❌ 被覆盖 |
| 4 | `slam_accuracy_visual_odometry.png` | 视觉里程计精度评估 | ✅ 保留（最后一次） |

---

## 🔧 修复方案

### 1. 修改 `evaluate_slam_accuracy.m` 函数签名

```matlab
% 修改前
function [metrics] = evaluate_slam_accuracy(estimated_trajectory, ground_truth, save_dir)

% 修改后
function [metrics] = evaluate_slam_accuracy(estimated_trajectory, ground_truth, save_dir, method_name)
```

**新增参数**: `method_name` - 方法名称，用于生成唯一文件名

### 2. 动态文件名生成

```matlab
% 修改前（所有方法使用相同文件名）
save_path = fullfile(save_dir, 'slam_accuracy_evaluation.png');

% 修改后（根据方法名生成不同文件名）
if ~isempty(method_name)
    filename = sprintf('slam_accuracy_%s.png', method_name);
else
    filename = 'slam_accuracy_evaluation.png';  % 向后兼容
end
save_path = fullfile(save_dir, filename);
```

### 3. 更新测试脚本调用

```matlab
% IMU-Visual Fusion
metrics_fusion_gt = evaluate_slam_accuracy(fusion_pos_aligned, gt_pos_aligned, result_path, 'imu_fusion');
% → 生成: slam_accuracy_imu_fusion.png

% Experience Map
metrics_exp_gt = evaluate_slam_accuracy(exp_traj_aligned, gt_pos_aligned, result_path, 'experience_map');
% → 生成: slam_accuracy_experience_map.png

% Visual Odometry
metrics_odo_gt = evaluate_slam_accuracy(odo_traj_aligned, gt_pos_aligned, result_path, 'visual_odometry');
% → 生成: slam_accuracy_visual_odometry.png
```

---

## ✅ 修复后的完整输出文件

### Town01场景输出目录

```
/data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results/
├── imu_visual_slam_comparison.png           # 3D轨迹对比（所有方法）
├── slam_accuracy_imu_fusion.png             # IMU-Fusion精度评估 ✨ 新增
├── slam_accuracy_experience_map.png         # 经验地图精度评估 ✨ 新增
├── slam_accuracy_visual_odometry.png        # 视觉里程计精度评估
├── trajectories.mat                         # 所有轨迹数据
├── odo_trajectory.txt                       # 视觉里程计轨迹
├── exp_trajectory.txt                       # 经验地图轨迹
├── ground_truth_backup.txt                  # Ground Truth备份
└── performance_report.txt                   # 性能对比报告
```

### Town10场景输出目录

```
/data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/slam_results/
├── imu_visual_slam_comparison.png           # 3D轨迹对比
├── slam_accuracy_imu_fusion.png             # IMU-Fusion精度评估 ✨ 新增
├── slam_accuracy_experience_map.png         # 经验地图精度评估 ✨ 新增
├── slam_accuracy_visual_odometry.png        # 视觉里程计精度评估
├── ... (其他文件)
```

---

## 📊 每张图的内容

### 1. `imu_visual_slam_comparison.png`
- **左上**: 3D轨迹对比（Ground Truth + 3种方法）
- **右上**: 2D俯视图
- **左中**: 位置误差曲线
- **右中**: 误差分布直方图
- **左下**: XYZ轴误差
- **右下**: 轨迹长度对比

### 2-4. `slam_accuracy_*.png` (每个方法独立)
- **左上**: 绝对轨迹误差随时间变化
- **右上**: 误差分布直方图
- **左下**: 每段轨迹误差柱状图
- **右下**: 误差vs行驶距离

---

## 🎯 文件命名规范

### 通用格式
```
slam_accuracy_{method_name}.png
```

### 支持的方法名

| 方法名 | method_name | 文件名 |
|--------|-------------|--------|
| IMU-Visual Fusion | `imu_fusion` | `slam_accuracy_imu_fusion.png` |
| Experience Map | `experience_map` | `slam_accuracy_experience_map.png` |
| Visual Odometry | `visual_odometry` | `slam_accuracy_visual_odometry.png` |
| （默认/兼容） | `''` (空) | `slam_accuracy_evaluation.png` |

---

## 🔄 向后兼容性

### 旧代码仍然可用
```matlab
% 不传递method_name参数（旧代码）
metrics = evaluate_slam_accuracy(est_traj, gt_traj, save_dir);
% → 仍然生成: slam_accuracy_evaluation.png
```

### 新代码推荐用法
```matlab
% 传递method_name参数（新代码）
metrics = evaluate_slam_accuracy(est_traj, gt_traj, save_dir, 'my_method');
% → 生成: slam_accuracy_my_method.png
```

---

## 🚀 验证步骤

### 1. 重新运行Town01测试
```matlab
cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

### 2. 检查输出文件
```bash
ls -lh /path/to/Town01Data_IMU_Fusion/slam_results/*.png
```

**预期输出**:
```
-rw-r--r-- 1 user user  XXX KB  imu_visual_slam_comparison.png
-rw-r--r-- 1 user user  XXX KB  slam_accuracy_imu_fusion.png          ✅ 新增
-rw-r--r-- 1 user user  XXX KB  slam_accuracy_experience_map.png      ✅ 新增
-rw-r--r-- 1 user user  XXX KB  slam_accuracy_visual_odometry.png
```

### 3. 切换到Town10测试
```matlab
% 修改 test_imu_visual_fusion_slam.m
data_path = 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion';
% 运行测试
test_imu_visual_fusion_slam
```

**预期**: Town10目录下也生成4张独立的图片，不会与Town01混淆。

---

## 📝 修改文件清单

### 已修改的文件

1. ✅ **`/neuro/09_vestibular/evaluate_slam_accuracy.m`**
   - 添加 `method_name` 参数
   - 实现动态文件名生成
   - 保持向后兼容

2. ✅ **`/neuro/07_test/test_imu_visual_slam/test_imu_visual_fusion_slam.m`**
   - 更新3次 `evaluate_slam_accuracy` 调用
   - 传递唯一的方法名: `'imu_fusion'`, `'experience_map'`, `'visual_odometry'`

3. ✅ **`/neuro/07_test/test_imu_visual_slam/IMAGE_SAVE_FIX.md`** (本文档)
   - 详细说明修复方案

---

## 💡 额外改进建议

### 1. 添加时间戳（可选）
```matlab
% 在文件名中添加时间戳，避免不同运行之间覆盖
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('slam_accuracy_%s_%s.png', method_name, timestamp);
```

### 2. 场景名称自动识别（可选）
```matlab
% 从数据路径自动提取场景名称
[~, scene_name] = fileparts(data_path);
filename = sprintf('%s_slam_accuracy_%s.png', scene_name, method_name);
% 例如: Town01Data_IMU_Fusion_slam_accuracy_imu_fusion.png
```

### 3. 生成汇总HTML报告（未来扩展）
```matlab
% 自动生成包含所有图片的HTML报告
generate_html_report(result_path, {'imu_fusion', 'experience_map', 'visual_odometry'});
```

---

## 🎓 经验总结

### 问题教训
1. **文件命名冲突**: 多次调用同一函数时，需要参数化输出文件名
2. **测试不充分**: 应该检查所有生成的文件，而不只是最后一个
3. **场景隔离**: 不同测试场景的输出应该完全隔离（已通过目录隔离实现）

### 设计原则
1. **唯一性**: 每个输出文件应该有唯一的文件名
2. **可追溯性**: 文件名应该清晰表明内容和来源
3. **向后兼容**: 修改应该不破坏现有代码

### 最佳实践
1. ✅ 使用方法名参数化文件名
2. ✅ 保持向后兼容（默认参数）
3. ✅ 将输出隔离到场景特定目录
4. ✅ 提供清晰的文档说明

---

## ✅ 验证清单

运行测试后检查：

- [ ] Town01生成4张PNG图片
- [ ] 文件名包含方法名标识
- [ ] 每张图片内容独立（不重复）
- [ ] Town10生成4张PNG图片（独立于Town01）
- [ ] 所有图片保存在正确的 `slam_results/` 目录

---

**修复完成时间**: 2024年11月29日  
**状态**: ✅ 已完成，等待验证  
**影响范围**: Town01和Town10测试，所有未来场景测试
