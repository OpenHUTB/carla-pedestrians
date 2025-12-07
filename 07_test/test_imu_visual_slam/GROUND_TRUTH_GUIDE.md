# Ground Truth对比功能使用指南

## 📋 概述

为了准确评估IMU-Visual SLAM的性能，需要有**Ground Truth（真值）**作为参考。Ground Truth是从CARLA仿真器直接获取的车辆真实位置，用于与各种SLAM方法的估计结果进行对比。

---

## 🔴 当前问题

您当前的数据**缺少Ground Truth文件**，因此无法进行完整的精度对比分析。

**原因**: 旧版本的Python采集脚本没有保存车辆的真实位置。

---

## ✅ 解决方案：重新采集数据

### 步骤1: 运行更新后的Python脚本

```bash
cd /home/dream/neuro_111111/carla-pedestrians/neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py
```

**新功能**: 
- ✅ 自动保存`ground_truth.txt`（车辆真实位置）
- ✅ 每10帧flush，确保数据安全
- ✅ 包含时间戳、位置、姿态和速度

### 步骤2: 等待数据采集完成

```
保存图像 1/5000
保存图像 2/5000
...
已保存 100 条融合位姿数据  ← 每100帧显示进度
...
保存图像 5000/5000
达到最大保存数量，退出
```

### 步骤3: 验证Ground Truth文件

```bash
# 检查文件是否生成
ls -lh /home/dream/neuro_111111/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/ground_truth.txt

# 查看文件内容
head /home/dream/neuro_111111/carla-pedestrians/neuro/data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/ground_truth.txt
```

**预期输出**:
```
timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,vel_x,vel_y,vel_z
10.000000,58.123456,18.234567,-8.345678,0.123456,0.234567,0.345678,1.234567,2.345678,0.123456
...
```

### 步骤4: 运行SLAM测试

```matlab
cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

**预期输出**（带Ground Truth）:
```
[4/9] 读取IMU-视觉融合数据...
成功读取 5000 条融合位姿数据
成功读取 5000 条Ground Truth数据  ← 新增！
✓ 已加载Ground Truth数据            ← 新增！
真实轨迹长度: XXX.XX 米             ← 新增！

[7/9] 评估轨迹精度...
========== 相对于Ground Truth的精度评估 ==========  ← 新增！

--- IMU-视觉融合轨迹 vs Ground Truth ---
绝对轨迹误差 (ATE):
  RMSE:     X.XX m
  平均值:   X.XX m
  ...

--- 经验地图轨迹 vs Ground Truth ---
  ...

--- 视觉里程计轨迹 vs Ground Truth ---
  ...
```

---

## 📊 新的可视化功能

### 1. 3D轨迹对比（含Ground Truth）

- **黑色粗线**: Ground Truth（真值）
- **红色线**: IMU-Visual Fusion
- **蓝色虚线**: Visual Odometry
- **绿色点线**: Experience Map

### 2. 2D俯视图对比

- 清晰显示各方法与真值的差异
- 标记起点（绿色）和终点（红色）

### 3. 位置误差随时间变化

- 实时显示各方法相对于Ground Truth的误差
- 显示平均误差线

### 4. 误差分布箱线图

- 对比各方法的误差统计特性
- 显示中位数、四分位数、异常值

### 5. XYZ各轴误差

- 分析X、Y、Z三个方向的误差分布
- 识别哪个方向漂移最严重

### 6. 轨迹长度对比

- 对比各方法估计的轨迹长度与真值
- 柱状图显示数值

---

## 📈 精度评估指标

### 1. 绝对轨迹误差 (ATE - Absolute Trajectory Error)

```
ATE = sqrt(mean((估计位置 - 真实位置)^2))
```

**意义**: 直接衡量估计轨迹与真值的整体偏差

### 2. 相对位姿误差 (RPE - Relative Pose Error)

```
RPE = sqrt(mean((相邻帧位移估计 - 真实位移)^2))
```

**意义**: 衡量局部一致性，不受全局漂移影响

### 3. 轨迹长度误差

```
长度误差 = |估计长度 - 真实长度| / 真实长度 × 100%
```

**意义**: 衡量尺度估计的准确性

### 4. 终点误差

```
终点误差 = ||估计终点 - 真实终点||
```

**意义**: 衡量累积漂移

---

## 🎯 Ground Truth文件格式

### ground_truth.txt

```csv
timestamp,pos_x,pos_y,pos_z,roll,pitch,yaw,vel_x,vel_y,vel_z
10.000000,58.123456,18.234567,-8.345678,0.123456,0.234567,0.345678,1.234567,2.345678,0.123456
10.050000,55.234567,17.345678,-8.456789,0.234567,0.345678,0.456789,1.345678,2.456789,0.234567
...
```

**字段说明**:
- `timestamp`: 时间戳（秒）
- `pos_x, pos_y, pos_z`: CARLA车辆真实3D位置（米）
- `roll, pitch, yaw`: CARLA车辆真实姿态角（度）
- `vel_x, vel_y, vel_z`: CARLA车辆真实速度（m/s）

---

## 🔍 与fusion_pose.txt的区别

| 文件 | 内容 | 用途 |
|------|------|------|
| **ground_truth.txt** | CARLA车辆**真实位置** | 评估参考标准 |
| **fusion_pose.txt** | IMU-Visual **融合后估计位置** | 待评估的SLAM结果 |

**关键**: 
- `ground_truth.txt` = 真值（从CARLA直接获取）
- `fusion_pose.txt` = 估计值（EKF融合后）

---

## ⚠️  注意事项

### 1. 时间戳对齐

Ground Truth和其他数据的时间戳必须严格对齐。脚本已自动处理。

### 2. 坐标系一致

所有数据使用CARLA世界坐标系（右手系）：
- X: 前方
- Y: 左方  
- Z: 上方

### 3. 数据完整性

确保采集过程不被中断（Ctrl+C），否则Ground Truth可能不完整。

---

## 🚀 快速开始

### 一键重新采集数据

```bash
# 1. 启动CARLA服务器
cd ~/carla/CARLA_0.9.XX
./CarlaUE4.sh

# 2. 运行采集脚本
cd /home/dream/neuro_111111/carla-pedestrians/neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py

# 3. 等待完成后，运行MATLAB测试
matlab -nodisplay -r "cd('/home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam'); test_imu_visual_fusion_slam; exit"
```

---

## 📚 相关文档

- **QUICK_FIX_SUMMARY.md** - 所有问题快速参考
- **DATA_READ_FIX.md** - 数据读取详解
- **FUNCTION_CALL_FIX.md** - 函数调用详解
- **check_fusion_data.m** - 数据诊断工具

---

## ✅ 检查清单

重新采集数据前：
- [ ] CARLA服务器已启动
- [ ] Python脚本已更新（包含Ground Truth保存）
- [ ] 磁盘空间充足（约2-3GB）
- [ ] 没有其他程序占用CARLA

重新采集数据后：
- [ ] `ground_truth.txt`文件存在
- [ ] 文件大小约800-900KB（5000行）
- [ ] 文件包含表头和数据
- [ ] 时间戳连续且合理

---

**最后更新**: 2024年  
**状态**: ✅ 已支持Ground Truth对比功能
