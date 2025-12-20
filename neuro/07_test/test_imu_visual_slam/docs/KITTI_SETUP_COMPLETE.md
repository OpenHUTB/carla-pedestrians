# ✅ KITTI Raw Dataset 工具集已就绪

车载城市道路场景 - 真实环境惯视融合测试

---

## 🎉 已创建的工具

### 📂 scripts/ 目录

| 文件 | 功能 | 使用方式 |
|------|------|----------|
| `download_kitti_raw.sh` | 下载KITTI数据 | `bash download_kitti_raw.sh` |
| `convert_kitti_to_neuro.py` | 格式转换 | `python3 convert_kitti_to_neuro.py --input ...` |
| `setup_kitti_complete.sh` | 一键完整流程 | `bash setup_kitti_complete.sh` |
| `README_KITTI.md` | 工具文档 | 详细说明 |

### 📂 quickstart/ 目录

| 文件 | 功能 |
|------|------|
| `RUN_SLAM_KITTI.m` | KITTI SLAM测试 |

### 📂 core/ 目录

| 文件 | 功能 |
|------|------|
| `test_kitti_fusion_slam.m` | KITTI处理核心脚本 |

### 📂 docs/ 目录

| 文件 | 功能 |
|------|------|
| `KITTI_QUICKSTART.md` | 快速开始指南 |

---

## 🚀 立即开始（三选一）

### 方案A: 一键完成（推荐）⭐

```bash
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/scripts
bash setup_kitti_complete.sh
```

自动完成：下载 → 转换 → 测试

---

### 方案B: 分步执行

```bash
cd scripts

# 1. 下载数据
bash download_kitti_raw.sh

# 2. 转换格式
python3 convert_kitti_to_neuro.py \
    --input ~/neuro_1newest/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0001_sync

# 3. 运行测试
cd ../quickstart
matlab -batch "RUN_SLAM_KITTI"
```

---

### 方案C: 只查看可用序列

```bash
cd scripts
bash download_kitti_raw.sh --list
```

---

## 📋 推荐测试序列

### 🏙️ 城市道路（最推荐）

| 序列 | 帧数 | 长度 | 场景特点 |
|------|------|------|----------|
| `2011_09_26_drive_0001` | ~108 | ~450m | 短序列，快速测试 |
| `2011_09_26_drive_0005` | ~154 | ~680m | 中等长度，标准测试 ⭐ |
| `2011_09_26_drive_0011` | ~233 | ~900m | 动态物体多，挑战性 |
| `2011_09_26_drive_0014` | ~314 | ~1.2km | 长序列，完整评估 |

### 🏘️ 住宅区

- `2011_09_30_drive_0016` - 住宅区道路
- `2011_09_30_drive_0027` - 城市+住宅混合

---

## 📊 数据说明

### KITTI Raw包含：

✅ **RGB单目相机** (Point Grey Flea 2)
- 分辨率: 1392×512 → 转换为 640×480
- 频率: 10 Hz
- 格式: PNG彩色 → 灰度

✅ **完整IMU数据** (OXTS RT 3003)
- 加速度: ax, ay, az (m/s²)
- 角速度: wx, wy, wz (rad/s)
- 姿态角: roll, pitch, yaw (rad)
- 频率: 10 Hz

✅ **GPS定位** (作为Ground Truth参考)

### 与CARLA对比：

| 特性 | KITTI | CARLA |
|------|-------|-------|
| 环境 | 真实德国城市 | 仿真城镇 |
| 相似度 | 城市道路 | 城市道路 |
| 视角 | 车载 | 车载 |
| IMU | 真实噪声 | 可配置噪声 |
| 用途 | 真实场景验证 | 算法开发 |

---

## 🎯 使用建议

### 论文实验设计：

**1. 仿真数据（CARLA）**
- Town01, Town02, Town10
- 算法有效性验证
- 消融实验

**2. 真实数据（KITTI）**
- 2-3个城市序列
- 真实场景适用性
- 鲁棒性验证

**3. 对比分析**
- 仿真 vs 真实性能差异
- 真实噪声影响分析
- 泛化能力评估

---

## ⚠️ 注意事项

### 下载相关：

1. **首次下载需注册**
   - 访问: http://www.cvlibs.net/datasets/kitti/raw_data.php
   - 注册免费账号

2. **数据量大**
   - 每个序列 1-3 GB
   - 建议先下载0001和0005测试

3. **下载可能较慢**
   - 服务器在德国
   - 可能需要VPN或镜像

### Python依赖：

```bash
pip3 install numpy pillow scipy
```

### MATLAB路径：

确保已添加NeuroSLAM核心路径。

---

## 📁 文件位置

```
/home/dream/neuro_1newest/
├── datasets/
│   ├── kitti_raw/              # 原始下载
│   │   └── 2011_09_26/
│   │       └── 2011_09_26_drive_0001_sync/
│   │           ├── image_02/
│   │           └── oxts/
│   │
│   └── kitti_converted/        # 转换后数据
│       └── 2011_09_26_drive_0001/
│           ├── images/
│           ├── imu_data.mat
│           └── slam_results/   # 测试结果
│
└── carla-pedestrians/neuro/07_test/test_imu_visual_slam/
    └── scripts/                # 工具脚本
```

---

## 🔧 下一步

选择你的路径：

**路径1: 快速验证**
```bash
bash scripts/setup_kitti_complete.sh
# 10-20分钟完成，获得初步结果
```

**路径2: 系统测试**
```bash
# 测试多个序列
bash scripts/setup_kitti_complete.sh 2011_09_26_drive_0001
bash scripts/setup_kitti_complete.sh 2011_09_26_drive_0005
bash scripts/setup_kitti_complete.sh 2011_09_26_drive_0011

# 批量对比分析
matlab -batch "compare_kitti_results"
```

**路径3: 消融实验**
```bash
# 先完成数据处理
bash scripts/setup_kitti_complete.sh 2011_09_26_drive_0005

# 运行消融实验
cd ablation
matlab -batch "RUN_COMPLETE_ABLATION"
```

---

## 📞 帮助

遇到问题？检查：

1. ✅ 阅读 `docs/KITTI_QUICKSTART.md`
2. ✅ 查看 `scripts/README_KITTI.md`
3. ✅ 验证Python依赖安装
4. ✅ 确认网络连接
5. ✅ 检查磁盘空间（需要5-10GB）

---

**准备就绪！开始你的KITTI测试吧！** 🚗💨
