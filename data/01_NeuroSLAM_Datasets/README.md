# 数据集说明

## 📁 数据集结构

本项目需要以下数据集（**未包含在GitHub仓库中**，需自行采集或下载）：

```
data/01_NeuroSLAM_Datasets/
├── Town01Data_IMU_Fusion/
│   ├── 0001.png - 5000.png          # RGB图像序列 (640×480)
│   ├── aligned_imu.txt              # IMU数据 (加速度+陀螺仪)
│   ├── fusion_pose.txt              # EKF融合位姿
│   └── ground_truth.txt             # 真实轨迹
│
└── Town10Data_IMU_Fusion/
    └── (相同结构)
```

---

## 📥 如何获取数据集？

### 方式1: 自己采集数据（推荐）

使用CARLA仿真器采集数据：

```bash
# 1. 启动CARLA服务器
cd /path/to/carla-0.9.15
./CarlaUE4.sh

# 2. 运行数据采集脚本
cd neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py
```

**采集参数**（在脚本中配置）：
- `TOWN = 'Town01'` 或 `'Town10'`
- `DURATION = 250` (秒)
- `CAMERA_FPS = 20`
- `IMU_FREQUENCY = 100`

采集完成后，数据将自动保存到此目录。

---

### 方式2: 下载预采集数据（如果有）

如果作者提供了预采集的数据集，请从以下位置下载：

- **Town01数据集**: [链接待添加]
- **Town10数据集**: [链接待添加]

下载后解压到对应目录。

---

## 📊 数据集规格

### Town01 数据集
- **帧数**: 5000
- **时长**: ~250秒
- **轨迹长度**: ~1.8 km
- **图像尺寸**: 640×480
- **文件大小**: ~2.5 GB

### Town10 数据集
- **帧数**: 5000
- **时长**: ~250秒
- **轨迹长度**: ~1.6 km
- **图像尺寸**: 640×480
- **文件大小**: ~2.5 GB

---

## 🔍 数据格式说明

### 图像文件
- **格式**: PNG
- **命名**: `0001.png`, `0002.png`, ..., `5000.png`
- **尺寸**: 640×480×3 (RGB)

### aligned_imu.txt
```
# timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
0.050, -11.183, -15.941, 5.995, -0.001, 0.002, -0.001
0.100, -11.205, -15.923, 6.012, -0.002, 0.002, -0.001
...
```

### fusion_pose.txt
```
# timestamp, x, y, z, roll, pitch, yaw, uncertainty_x, uncertainty_y, uncertainty_z
0.050, 1.10, -1.99, 0.00, 0.01, 0.02, 1.57, 0.088, 0.088, 0.088
...
```

### ground_truth.txt
```
# timestamp, x, y, z, roll, pitch, yaw
0.050, 1.10, -1.99, 0.00, 0.00, 0.00, 1.57
...
```

---

## ⚠️ 注意事项

1. **数据集太大无法上传到GitHub**  
   完整数据集约5GB，不适合Git版本控制。

2. **采集新数据前请检查磁盘空间**  
   确保至少有10GB可用空间。

3. **CARLA版本要求**  
   推荐使用CARLA 0.9.13 - 0.9.15。

4. **采集时间**  
   Town01和Town10各需约5-10分钟采集。

---

## 🚀 快速验证

采集或下载数据后，验证数据完整性：

```matlab
cd neuro
quick_test_integration  % 30秒快速测试
```

如果测试通过，说明数据集正确！

---

**数据集版本**: 1.0  
**更新日期**: 2025-12-07  
**CARLA版本**: 0.9.15
