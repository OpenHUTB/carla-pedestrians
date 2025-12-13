# EuRoC MAV Dataset - 数据集说明

> **用于NeuroSLAM系统的EuRoC数据集测试指南**

---

## 📊 数据集概述

本项目使用EuRoC MAV Dataset中的两个序列进行真实环境SLAM测试：

| 数据集 | 难度 | 轨迹长度 | 图像数 | 测试结果(RMSE) |
|--------|------|---------|-------|---------------|
| **MH_01_easy** | Easy ⭐ | 81m | 3682 | 3.44m |
| **MH_03_medium** | Medium ⭐⭐ | 127m | 2722 | 1.04m |

---

## 🔗 下载链接

### MH_01_easy (Machine Hall 01 - Easy)

**官方下载**:
```
http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
文件大小: ~3.6 GB
```

**数据集特点**:
- 场景: 机械大厅室内环境
- 难度: 简单
- 运动: 平缓飞行
- 纹理: 丰富
- 光照: 良好

---

### MH_03_medium (Machine Hall 03 - Medium)

**官方下载**:
```
http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.bag
文件大小: ~2.6 GB
```

**数据集特点**:
- 场景: 机械大厅室内环境
- 难度: 中等
- 运动: 快速飞行，有旋转
- 纹理: 丰富
- 挑战: 快速运动，角速度大

---

## 🚀 快速开始

### 1. 下载数据集

```bash
# 创建目录
mkdir -p ~/datasets/euroc_raw
cd ~/datasets/euroc_raw

# 下载MH_01
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag

# 下载MH_03
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.bag
```

---

### 2. 数据预处理

```bash
cd ~/neuro_111111/scripts

# 处理MH_01
./process_euroc_complete.sh ~/datasets/euroc_raw/MH_01_easy.bag MH_01_easy

# 处理MH_03
./process_euroc_complete.sh ~/datasets/euroc_raw/MH_03_medium.bag MH_03_medium
```

**预处理步骤**:
1. 提取图像 → `images/`
2. 提取IMU → `aligned_imu.txt`
3. 提取Ground Truth → `ground_truth.txt`
4. 生成融合数据 → `fusion_pose.txt`

---

### 3. 运行SLAM测试

在MATLAB中:

```matlab
% 进入测试目录
cd ~/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam

% 测试MH_01
RUN_SLAM_MH01

% 测试MH_03
RUN_SLAM_MH03
```

---

## 📈 测试结果

### MH_01_easy 性能

```
IMU-Visual融合:
  RMSE: 3.44m
  漂移率: 0.14%
  终点误差: 0.11m

视觉里程计:
  RMSE: 40.69m
  漂移率: 81.08%
  终点误差: 65.67m

融合改进: 11.8倍 🚀
```

---

### MH_03_medium 性能

```
IMU-Visual融合:
  RMSE: 1.04m
  漂移率: 0.028%
  终点误差: 0.04m

视觉里程计:
  RMSE: 22.85m
  漂移率: 22.57%
  终点误差: 26.67m

融合改进: 22.0倍 🚀
```

---

## 📁 数据格式

### 目录结构

```
~/datasets/euroc_converted/MH_01_easy/
├── images/                   # 左相机图像 (PNG格式)
├── aligned_imu.txt           # IMU数据
├── ground_truth.txt          # Ground Truth
├── fusion_pose.txt           # 融合结果
└── slam_results/             # SLAM输出
```

### 文件格式

**aligned_imu.txt**:
```
timestamp(ns), ax(m/s²), ay, az, gx(rad/s), gy, gz
```

**ground_truth.txt**:
```
timestamp(ns), x(m), y, z, qw, qx, qy, qz
```

**fusion_pose.txt**:
```
timestamp(ns), x, y, z, qw, qx, qy, qz, cov_x, cov_y, cov_z
```

---

## 🎯 数据集对比

| 特性 | MH_01 (Easy) | MH_03 (Medium) |
|------|-------------|---------------|
| **轨迹长度** | 81m | 127m |
| **运动速度** | 慢 | 快 |
| **场景变化** | 小 | 大 |
| **VT节点数** | 14 | 201 |
| **经验节点** | 少 | 多 |
| **我们的精度** | 3.44m | **1.04m** ⭐ |
| **漂移率** | 0.14% | **0.028%** ⭐ |

**关键发现**: 
- MH_03虽然难度更高，但我们的精度更好
- 原因: 场景变化大，VT节点更丰富，融合效果更好

---

## 📚 参考资料

### 官方资源

- **官网**: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- **论文**: The EuRoC micro aerial vehicle datasets (IJRR 2016)
- **arXiv**: https://arxiv.org/abs/1703.01815

### 引用

```bibtex
@article{Burri2016IJRR,
  title={The EuRoC micro aerial vehicle datasets},
  author={Burri, Michael and Nikolic, Janosch and Gohl, Pascal and 
          Schneider, Thomas and Rehder, Joern and Omari, Sammy and 
          Achtelik, Markus W and Siegwart, Roland},
  journal={The International Journal of Robotics Research},
  volume={35},
  number={10},
  pages={1157--1163},
  year={2016}
}
```

---

## 🔍 常见问题

**Q: 为什么需要EuRoC数据集？**  
A: 验证算法在真实环境下的性能，对比模拟数据（CARLA）的结果。

**Q: 如何选择测试序列？**  
A: 
- 初步验证 → MH_01 (Easy)
- 鲁棒性测试 → MH_03 (Medium)
- 极限测试 → MH_04/05 (Difficult)

**Q: Ground Truth精度如何？**  
A: Vicon系统提供亚毫米级精度（<1mm）

**Q: 可以测试其他EuRoC序列吗？**  
A: 可以！下载对应的bag文件，使用相同的处理脚本即可。

---

## ✅ 检查清单

使用EuRoC数据集前，请确认：

- [ ] 已下载 MH_01_easy.bag (~3.6GB)
- [ ] 已下载 MH_03_medium.bag (~2.6GB)
- [ ] 已运行 `process_euroc_complete.sh` 预处理
- [ ] 已验证数据完整性（图像数量、txt文件）
- [ ] 已在MATLAB中测试 `RUN_SLAM_MH01` 和 `RUN_SLAM_MH03`

---

**更新日期**: 2024-12-13  
**相关文档**: 
- 完整指南: `~/datasets/EUROC_DATASET_GUIDE.md`
- 差异报告: `~/neuro_111111/NEURO_DIFF_REPORT.md`
