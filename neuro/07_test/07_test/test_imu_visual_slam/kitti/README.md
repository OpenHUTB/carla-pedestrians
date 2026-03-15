# KITTI Dataset Experiments

KITTI Odometry Dataset（真实车辆场景）的完整实验代码和工具。

## 📁 文件夹结构

```
kitti/
├── data_processing/          # 数据预处理脚本
│   └── prepare_kitti_fusion_data.m     # KITTI数据转换为fusion格式
├── core/                     # 核心测试脚本
│   └── test_kitti_fusion_slam.m        # KITTI完整融合SLAM测试
├── utils/                    # 工具函数（待添加）
├── results/                  # 实验结果输出目录
└── README.md                 # 本文档
```

## 📊 数据集信息

**KITTI Sequence 07**
- 场景：城市街道
- 轨迹长度：694米
- 帧数：1101帧
- 传感器：单目相机 + IMU (OXTS)
- 图像分辨率：1241 x 376 (灰度)
- Ground Truth：高精度GPS/IMU融合

## 🚀 快速开始

### 0. 诊断KITTI数据（推荐先运行）

```matlab
% 在MATLAB中
cd /path/to/kitti

% 运行诊断脚本
diagnose_kitti
```

**诊断脚本会输出**：
- 轨迹长度、速度统计
- 转弯信息
- 图像尺寸和质量
- **建议的参数值**

### 1. 数据准备

确保KITTI数据已下载并整理：

```
/home/dream/neuro_1newest/carla-pedestrians/neuro/data/KITTI_07/
├── image_0/        # 1101张单目图像
├── oxts/           # 1106个IMU文件
├── poses.txt       # Ground Truth轨迹
├── calib.txt       # 相机标定
└── times.txt       # 时间戳
```

### 2. 运行数据预处理

```matlab
% 在MATLAB中运行
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/kitti/data_processing

% 设置数据路径
kitti_path = '/home/dream/neuro_1newest/carla-pedestrians/neuro/data/KITTI_07';

% 预处理数据（生成fusion_pose.txt）
fusion_data = prepare_kitti_fusion_data(kitti_path);
```

**输出**：
- `fusion_pose.txt` - 融合后的完整数据文件

### 3. 运行完整测试

```matlab
% 在MATLAB中运行
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/kitti/core

% 设置全局路径变量
global KITTI_DATA_PATH;
KITTI_DATA_PATH = '/home/dream/neuro_1newest/carla-pedestrians/neuro/data/KITTI_07';

% 运行测试
test_kitti_fusion_slam;
```

**输出结果**：
- 性能报告（FPS、各模块耗时）
- 精度评估（ATE、RTE、Drift）
- 轨迹对比图
- 视觉模板统计
- 保存在 `KITTI_07/` 目录下

## 📈 预期结果

### 性能指标
- **FPS**: 15-25 (预期)
- **Visual Templates**: 200-400 (预期)
- **Pipeline Time**: ~40-60ms/frame

### 精度指标
- **Trajectory Length**: 694米
- **Drift Rate**: 8-15% (预期，相比纯VO有改善)

## 🔧 脚本说明

### data_processing/prepare_kitti_fusion_data.m

**功能**：
1. 读取KITTI Ground Truth poses
2. 读取OXTS IMU数据
3. 同步时间戳
4. 生成fusion_pose.txt格式文件

**输入**：
- `poses.txt` - GT轨迹（3x4变换矩阵）
- `oxts/data/*.txt` - IMU数据（30个值/帧）
- `times.txt` - 时间戳

**输出**：
- `fusion_pose.txt` - 统一格式的融合数据
  - 列：timestamp, x, y, z, roll, pitch, yaw, vx, vy, vz, ax, ay, az, wx, wy, wz

### core/test_kitti_fusion_slam.m

**功能**：
1. 初始化NeuroSLAM系统
2. 加载KITTI数据
3. 运行IMU-Visual融合SLAM
4. 性能监控和统计
5. 精度评估和可视化

**主要模块**：
- Visual Odometry（IMU辅助）
- Visual Template Matching
- Yaw-Height HDC
- 3D Grid Cells
- Experience Map

**输出**：
- 性能报告文件
- 轨迹对比图
- VT统计报告
- 精度评估结果

## 📋 依赖关系

需要以下模块（自动添加到路径）：
- `01_conjunctive_pose_cells_network/` - 位姿细胞网络
- `04_visual_template/` - 视觉模板
- `03_visual_odometry/` - 视觉里程计
- `02_multilayered_experience_map/` - 经验地图
- `07_test/experiment_supplement/` - 性能监控工具

## 🎯 与其他数据集对比

| 数据集 | 场景 | 轨迹长度 | IMU | 地点 |
|--------|------|---------|-----|------|
| **KITTI 07** | 城市车辆 | 694m | ✅ 真实 | 真实 |
| CARLA | 城市车辆 | 1700-2200m | ✅ 融合 | 仿真 |
| EuRoC | 室内MAV | 80-130m | ✅ 真实 | 真实 |

**KITTI优势**：
- ✅ 真实车辆场景
- ✅ 真实IMU数据
- ✅ 标准基准数据集
- ✅ 可与SOTA方法对比

## 📝 注意事项

1. **图像格式**：KITTI使用PNG灰度图像，已自动处理
2. **IMU数据**：OXTS格式包含30个值，脚本自动提取需要的加速度和角速度
3. **坐标系**：
   - KITTI: X前，Y左，Z上（相机坐标系）
   - NeuroSLAM: 内部自动转换
4. **时间同步**：已根据timestamps对齐

## 🔧 参数调整指南

### 如果轨迹不准确

**症状**：
- 终点误差很大（>100m）
- 缩放因子远离1.0（<0.5 或 >2.0）
- 漂移率很高（>50%）

**诊断步骤**：

1. **运行诊断脚本**：
```matlab
cd kitti/
diagnose_kitti  % 会给出建议的ODO_TRANS_V_SCALE值
```

2. **调整视觉里程计尺度**：
编辑 `core/test_kitti_fusion_slam.m` 第106-108行：
```matlab
'ODO_TRANS_V_SCALE', XXX, ...  % 使用诊断建议的值
'ODO_HEIGHT_V_SCALE', XXX, ...  % 通常与TRANS_V_SCALE相同
```

3. **清理并重新运行**：
```matlab
cd kitti/
clean_and_rerun  % 自动清理并重新运行
```

### 关键参数说明

- **ODO_TRANS_V_SCALE**: 平移速度缩放因子
  - 太小：估计距离偏短
  - 太大：估计距离偏长
  - KITTI建议范围：80-120

- **VT_MATCH_THRESHOLD**: 视觉模板匹配阈值
  - 太低：模板太多，计算慢
  - 太高：模板太少，精度差
  - KITTI建议：0.08-0.10

## 🐛 故障排除

### 问题0：轨迹明显错误
**症状**：虽然有转弯，但轨迹是直线或漂移严重

**解决**：
```matlab
% 1. 先运行诊断
diagnose_kitti

% 2. 根据建议调整参数
% 编辑 core/test_kitti_fusion_slam.m

% 3. 清理并重新运行
clean_and_rerun
```

### 问题1：找不到图像文件
```matlab
% 检查路径
ls /home/dream/neuro_1newest/carla-pedestrians/neuro/data/KITTI_07/image_0/
```

### 问题2：IMU数据缺失
```matlab
% 检查IMU文件
ls /home/dream/neuro_1newest/carla-pedestrians/neuro/data/KITTI_07/oxts/data/ | wc -l
```

### 问题3：fusion_pose.txt生成失败
```matlab
% 重新生成
cd kitti/data_processing
fusion_data = prepare_kitti_fusion_data('/path/to/KITTI_07');
```

## 📚 参考文献

1. KITTI Vision Benchmark Suite: http://www.cvlibs.net/datasets/kitti/
2. Geiger et al. "Are we ready for Autonomous Driving?" CVPR 2012

---

**Created**: 2024-12-22  
**Last Updated**: 2024-12-22  
**Maintainer**: NeuroSLAM Project
