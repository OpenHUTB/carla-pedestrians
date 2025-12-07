# IMU-视觉融合SLAM 快速开始指南

## 5分钟上手

### 前置条件

1. **CARLA仿真器** 已安装并运行
   ```bash
   cd /path/to/carla-0.9.15
   ./CarlaUE4.sh
   ```

2. **Python环境** (Python 3.7+)
   ```bash
   pip install numpy opencv-python scipy carla
   ```

3. **MATLAB** (R2019b或更高版本)

### 第一步: 采集IMU-视觉数据 (5-10分钟)

```bash
cd /home/dream/neuro_111111/carla-pedestrians/neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py
```

**等待采集完成**:
- 屏幕会显示实时图像
- 终端打印进度: `保存图像 X/5000`
- 每100帧显示融合质量指标
- 按`q`键可提前退出

**采集完成后检查输出**:
```bash
ls ../data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/
# 应该看到:
# 0001.png, 0002.png, ... (图像文件)
# aligned_imu.txt (IMU数据)
# fusion_pose.txt (融合位姿)
# dataset_metadata.txt (元数据)
```

### 第二步: 运行SLAM测试 (2-3分钟)

在MATLAB中执行:

```matlab
cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

**运行过程**:
1. 加载数据 (10秒)
2. SLAM处理 (1-2分钟，取决于帧数)
3. 生成可视化 (20秒)
4. 保存结果 (5秒)

### 第三步: 查看结果

测试完成后会自动生成:

1. **对比可视化图**: `imu_visual_slam_comparison.png`
   - 包含6个子图对比不同方法

2. **精度评估图**: `slam_accuracy_evaluation.png`
   - 误差分析和统计

3. **性能报告**: `slam_results/performance_report.txt`
   ```
   打开查看:
   - 轨迹长度对比
   - 位置不确定性
   - 漂移率
   - 改进效果百分比
   ```

## 常见问题快速解决

### Q1: Python脚本报错 "连接CARLA失败"
**A**: 先启动CARLA服务器
```bash
cd /path/to/carla-0.9.15
./CarlaUE4.sh
```
等待完全启动(看到"Traffic Manager"字样)后再运行Python脚本。

### Q2: MATLAB报错 "数据路径不存在"
**A**: 修改 `test_imu_visual_fusion_slam.m` 中的路径:
```matlab
data_path = fullfile(rootDir, 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion');
```
确保路径指向你的数据文件夹。

### Q3: 图像数量太少
**A**: 修改 `IMU_Vision_Fusion_EKF.py` 中的参数:
```python
MAX_SAVE_IMG = 500  # 减少到500帧用于快速测试
```

### Q4: 内存不足
**A**: 分批处理数据，修改MATLAB脚本:
```matlab
num_frames = min(500, length(img_files));  % 只处理前500帧
```

## 性能调优快速提示

### 提高融合精度
在 `IMU_Vision_Fusion_EKF.py` 中降低观测噪声:
```python
self.R = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
```

### 提高IMU权重
在 `imu_aided_visual_odometry.m` 中:
```matlab
alpha_yaw = 0.8;  % 从0.7改为0.8
```

### 更平滑的轨迹
增加EKF过程噪声:
```python
self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001])
```

## 一键测试脚本

创建 `quick_test.sh`:
```bash
#!/bin/bash
echo "启动IMU-视觉融合SLAM快速测试..."

# 步骤1: 采集数据
echo "[1/2] 采集数据..."
cd /home/dream/neuro_111111/carla-pedestrians/neuro/00_collect_data
timeout 300 python IMU_Vision_Fusion_EKF.py  # 5分钟后自动退出

# 步骤2: 运行MATLAB测试
echo "[2/2] 运行SLAM..."
matlab -batch "cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam; test_imu_visual_fusion_slam"

echo "测试完成! 查看结果:"
echo "  - imu_visual_slam_comparison.png"
echo "  - slam_accuracy_evaluation.png"
```

使用:
```bash
chmod +x quick_test.sh
./quick_test.sh
```

## 进阶使用

### 自定义地图
修改 `IMU_Vision_Fusion_EKF.py`:
```python
TARGET_MAP = "Town01"  # 改为其他地图
OUTPUT_DIR = '../data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/'
```

### 调整采样率
```python
IMU_SAMPLE_RATE = 100  # 提高到100Hz
CAMERA_SAMPLE_RATE = 30  # 提高到30Hz
# 记得修改相机的sensor_tick
rgb_bp.set_attribute("sensor_tick", "0.033")  # 1/30
```

### 添加更多传感器
在 `IMU_Vision_Fusion_EKF.py` 中添加:
```python
# GPS传感器
gps_bp = bp_lib.find('sensor.other.gnss')
gps = world.spawn_actor(gps_bp, transform, attach_to=vehicle)
gps.listen(lambda data: data_queue.put(SensorData('gps', data.timestamp, data)))
```

## 下一步

1. 阅读 `README_IMU_Visual_Fusion.md` 了解详细架构
2. 调整参数优化性能
3. 尝试不同地图和场景
4. 集成到你的应用中

## 获取帮助

- 查看完整文档: `README_IMU_Visual_Fusion.md`
- 参数说明: 文档中的"参数调优指南"章节
- 故障排查: 文档中的"故障排查"章节
