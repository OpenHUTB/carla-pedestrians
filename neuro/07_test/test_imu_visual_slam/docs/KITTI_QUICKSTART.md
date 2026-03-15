# KITTI Raw Dataset 快速开始指南

真实车载城市道路场景 - 惯视融合系统测试

---

## 📋 数据集简介

**KITTI Raw Dataset**
- **场景：** 德国城市道路（类似CARLA Town）
- **传感器：** RGB单目相机 + GPS/INS (IMU)
- **采集平台：** 改装的VW Passat B6
- **频率：** 10Hz (相机和IMU同步)
- **IMU数据：** 完整的加速度和角速度测量

**推荐序列（城市场景）：**
- `2011_09_26_drive_0001` - 短序列，快速测试
- `2011_09_26_drive_0005` - 中等长度，完整测试
- `2011_09_26_drive_0011` - 动态物体多，挑战性高

---

## 🚀 快速开始（三步走）

### 步骤1️⃣: 下载KITTI Raw数据

```bash
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/scripts

# 方法A: 下载推荐序列（默认）
bash download_kitti_raw.sh

# 方法B: 下载指定序列
bash download_kitti_raw.sh 2011_09_26_drive_0005

# 方法C: 查看可用序列
bash download_kitti_raw.sh --list
```

**下载位置：** `/home/dream/neuro_1newest/datasets/kitti_raw/`

**⚠️ 注意：** 
- 首次下载需要在KITTI官网注册账号
- 如果wget下载失败，可能需要手动登录下载
- 每个序列约1-3GB

---

### 步骤2️⃣: 转换数据格式

```bash
# 转换KITTI格式 -> NeuroSLAM格式
python3 scripts/convert_kitti_to_neuro.py \
    --input /home/dream/neuro_1newest/datasets/kitti_raw/2011_09_26/2011_09_26_drive_0001_sync
```

**转换内容：**
- ✅ 图像：PNG → 灰度图 (640x480)
- ✅ IMU：OXTS格式 → 标准格式 (accel + gyro)
- ✅ 时间戳：同步对齐

**输出位置：** `/home/dream/neuro_1newest/datasets/kitti_converted/2011_09_26_drive_0001/`

**目录结构：**
```
2011_09_26_drive_0001/
├── images/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── imu_data.mat          # IMU数据（MATLAB格式）
└── metadata.json         # 元数据
```

---

### 步骤3️⃣: 运行SLAM测试

```bash
cd /home/dream/neuro_1newest/carla-pedestrians/neuro/07_test/test_imu_visual_slam/quickstart

# 运行KITTI SLAM测试
matlab -batch "RUN_SLAM_KITTI"
```

**输出结果：**
```
kitti_converted/2011_09_26_drive_0001/slam_results/
├── trajectories.mat          # 轨迹数据
└── kitti_slam_results.png    # 可视化结果
```

---

## 📊 结果说明

### 保存的轨迹数据

| 变量名 | 含义 | 用途 |
|--------|------|------|
| `pure_visual_trajectory` | 纯视觉里程计 | 消融实验 - w/o IMU |
| `odo_trajectory` | IMU-VO融合里程计 | 中间结果 |
| `exp_trajectory` | 经验地图轨迹 | 完整系统输出 |
| `imu_data` | IMU原始数据 | 分析参考 |

### 可视化图表

1. **轨迹对比** - 三种方法的2D轨迹
2. **轨迹长度** - 累积距离曲线
3. **IMU数据** - 角速度和加速度
4. **融合改进** - 位置差异分析

---

## 🔧 批量处理

### 处理多个序列

创建批量脚本 `process_all_kitti.sh`:

```bash
#!/bin/bash
# 批量处理KITTI序列

SEQUENCES=(
    "2011_09_26_drive_0001"
    "2011_09_26_drive_0005"
    "2011_09_26_drive_0011"
)

for seq in "${SEQUENCES[@]}"; do
    echo "处理序列: $seq"
    
    # 转换格式
    python3 scripts/convert_kitti_to_neuro.py \
        --input "/home/dream/neuro_1newest/datasets/kitti_raw/2011_09_26/${seq}_sync"
    
    # 运行SLAM
    matlab -batch "KITTI_DATA_PATH='/home/dream/neuro_1newest/datasets/kitti_converted/$seq'; test_kitti_fusion_slam"
done
```

---

## 📈 消融实验

使用KITTI数据进行消融实验：

```matlab
% 编辑 ablation/RUN_COMPLETE_ABLATION.m
datasets = {
    'kitti_converted/2011_09_26_drive_0001', 'KITTI_0001', 450.0;
    'kitti_converted/2011_09_26_drive_0005', 'KITTI_0005', 680.0;
};

% 运行消融实验
cd ../ablation
matlab -batch "RUN_COMPLETE_ABLATION"
```

---

## ❓ 常见问题

### Q1: 下载失败，提示需要登录？

**解决：**
1. 访问 http://www.cvlibs.net/datasets/kitti/raw_data.php
2. 注册账号并登录
3. 手动下载需要的序列（.zip文件）
4. 解压到 `/home/dream/neuro_1newest/datasets/kitti_raw/`

### Q2: IMU数据格式是什么？

**KITTI OXTS数据包含：**
- 位置：GPS (lat/lon/alt)
- 姿态：Roll/Pitch/Yaw (rad)
- 速度：North/East/Forward (m/s)
- **加速度：** ax, ay, az (m/s²) ⭐
- **角速度：** wx, wy, wz (rad/s) ⭐

我们提取加速度和角速度用于惯性融合。

### Q3: 为什么使用image_02（左侧相机）？

KITTI有4个相机：
- `image_00/01`: 左右灰度立体
- `image_02/03`: 左右彩色立体

我们使用 `image_02` (左侧彩色) 并转换为灰度，与CARLA数据保持一致。

### Q4: KITTI vs CARLA 有什么区别？

| 特性 | KITTI | CARLA |
|------|-------|-------|
| 场景 | 真实城市 | 仿真城市 |
| 噪声 | 真实传感器噪声 | 可配置噪声 |
| Ground Truth | GPS/INS | 完美真值 |
| 挑战性 | 高（天气、光照） | 可控 |

KITTI用于**真实场景验证**，CARLA用于**算法开发**。

---

## 🎯 推荐工作流

**开发阶段：**
1. 使用CARLA快速迭代算法
2. 批量测试Town01/02/10
3. 进行消融实验

**验证阶段：**
1. 使用KITTI真实数据测试
2. 对比仿真vs真实性能
3. 分析真实场景下的问题

**论文写作：**
- CARLA结果展示算法有效性
- KITTI结果验证真实场景适用性
- 对比分析提升说服力

---

## 📚 参考资料

**KITTI官方：**
- 数据集主页: http://www.cvlibs.net/datasets/kitti/
- Raw Data: http://www.cvlibs.net/datasets/kitti/raw_data.php
- 论文: "Vision meets Robotics: The KITTI Dataset" (IJRR 2013)

**工具库：**
- pykitti: https://github.com/utiasSTARS/pykitti
- kitti2bag: https://github.com/tomas789/kitti2bag

---

## 📞 技术支持

遇到问题？检查：
1. 数据路径是否正确
2. Python依赖是否安装 (`pip install numpy pillow scipy`)
3. MATLAB路径是否设置 (`addpath`)
4. IMU数据文件是否生成

Good luck! 🚗💨
