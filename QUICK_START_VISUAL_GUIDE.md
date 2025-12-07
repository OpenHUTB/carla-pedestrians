# 🚀 NeuroSLAM快速启动可视化指南

## 一分钟了解整个系统

```
┌─────────────────────────────────────────────────────────────┐
│                  NeuroSLAM系统全景图                         │
│                                                              │
│  输入 → 处理 → 输出                                          │
│                                                              │
│  [图像+IMU] → [类脑SLAM] → [3D轨迹+地图]                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 三个核心问题

### Q1: 这个系统做什么？
**A**: 模拟大鼠海马体，实现**3D视觉SLAM** (同时定位与建图)

### Q2: 系统有什么特点？
**A**: **类脑设计** + **多传感器融合** + **实时性能**

### Q3: 如何快速上手？
**A**: **3步启动** (数据→配置→运行) ⏱️ 5分钟

---

## 📊 系统处理流程 (可视化)

```
第1步: 数据输入
┌───────────────────────────────────────────────┐
│  📷 RGB图像 (640×480, 20Hz)                   │
│  📡 IMU数据 (加速度+陀螺仪, 100Hz)            │
│  📍 Ground Truth (可选)                       │
└───────────────┬───────────────────────────────┘
                ↓
                
第2步: 里程计融合
┌───────────────────────────────────────────────┐
│  🔄 视觉里程计 (扫描线匹配)                   │
│  🔄 IMU里程计 (陀螺仪+加速度计)               │
│  ➡️  互补融合: 70% IMU偏航 + 70% 视觉平移    │
│  📤 输出: transV, yawRotV, heightV            │
└───────────────┬───────────────────────────────┘
                ↓
                
第3步: 场景识别
┌───────────────────────────────────────────────┐
│  👁️ 视觉模板匹配                              │
│  🧠 HART+CORnet特征提取                       │
│  📤 输出: VT ID (场景标识)                    │
└───────────────┬───────────────────────────────┘
                ↓
                
第4步: 姿态估计
┌────────────────┬──────────────────────────────┐
│  🧭 头部朝向细胞│  📍 3D网格细胞               │
│  (yaw+height)  │  (x, y, z)                   │
│  36×36 环状网络│  61×61×51 立方网络           │
└────────────────┴───────────┬──────────────────┘
                              ↓
                              
第5步: 地图构建
┌───────────────────────────────────────────────┐
│  🗺️ 经验地图 (拓扑+度量)                      │
│  • 节点: VT + Grid Cell + HDC                 │
│  • 边: 相邻节点的位移                         │
│  • 优化: 迭代松弛算法                         │
└───────────────┬───────────────────────────────┘
                ↓
                
第6步: 输出结果
┌───────────────────────────────────────────────┐
│  📊 3D轨迹 (x, y, z, yaw)                     │
│  📈 精度评估 (ATE/RPE/轨迹误差)               │
│  📉 可视化图表 (7张)                          │
│  💾 数据保存 (.mat + .txt)                    │
└───────────────────────────────────────────────┘
```

---

## 🔧 三种启动方式 (选一个)

### 方式1: 快速测试 ⭐ (推荐新手)

**时间**: 30秒  
**数据**: 内置测试图像  
**目的**: 验证系统功能

```matlab
% 假设你已经在项目根目录 carla-pedestrians/neuro/
quick_test_integration
```

**预期输出**:
```
✓ 特征提取速度: 71 FPS
✓ VT数量: 5-6个
✓ 模板重用率: 75%
✓ 所有测试通过
```

---

### 方式2: Town01完整SLAM ⭐⭐ (推荐进阶)

**时间**: 3-5分钟  
**数据**: Town01数据集 (5000帧, ~1.8km)  
**目的**: 完整SLAM演示

```matlab
% 从neuro目录进入测试目录
cd 07_test/test_imu_visual_slam
test_imu_visual_slam_hart_cornet
```

**预期输出**:
```
✓ VT数量: 5
✓ 经验节点: 185
✓ RMSE: 152.87m
✓ 轨迹完整性: 95.3%
✓ 生成3张可视化图表
```

**查看结果**:
```bash
# 图表保存在:
data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/slam_results_hart_cornet/
├── imu_visual_slam_comparison.png      # 轨迹对比
├── slam_accuracy_slam_exp_trajectory.png  # 精度评估
└── slam_statistics_slam_exp_trajectory.png # 统计图
```

---

### 方式3: 从零采集数据 ⭐⭐⭐ (推荐研究)

**时间**: 10-20分钟 (含数据采集)  
**数据**: 自己采集  
**目的**: 定制化研究

#### 步骤1: 启动CARLA
```bash
cd /path/to/carla-0.9.15
./CarlaUE4.sh
```

#### 步骤2: 采集数据
```bash
cd neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py
```

**采集参数** (在脚本顶部修改):
```python
TOWN = 'Town01'        # 地图选择
DURATION = 250         # 采集时长(秒)
CAMERA_FPS = 20        # 相机帧率
IMU_FREQUENCY = 100    # IMU采样率
```

**采集过程**:
```
[1/5] 连接CARLA服务器
[2/5] 生成随机路径
[3/5] 初始化传感器 (RGB相机 + IMU)
[4/5] 开始采集 (自动避障)
[5/5] 保存数据到 data/01_NeuroSLAM_Datasets/
```

#### 步骤3: 运行SLAM
```matlab
cd neuro/07_test/test_imu_visual_slam
test_imu_visual_slam_hart_cornet
```

---

## 🎨 特征提取方法对比

### 方法1: 原始Patch Normalization

```
图像 → 裁剪 → 缩放 → Patch归一化 → 像素差匹配
```

**特点**:
- ✅ 速度快 (~50 FPS)
- ❌ 对光照敏感
- ❌ 对噪声敏感

---

### 方法2: 简化增强特征 ⭐ (推荐)

```
图像 → 灰度化 → adapthisteq → 高斯平滑 → Min-Max归一化 → 余弦距离
```

**特点**:
- ✅ **超快速度** (71 FPS, 5.92倍提升)
- ✅ **强鲁棒性** (抗噪声/光照/模糊)
- ✅ **高重用率** (75%模板重用)
- ✅ **纯MATLAB** (无依赖)

**性能**:
```
Town01 (5000帧):
- VT数量: 1365
- RMSE: 142.57m (最佳)
- 轨迹完整性: 38% (压缩)
```

**适用场景**:
- 局部精确定位 (<500m)
- 场景细节识别
- 需要最快速度

---

### 方法3: HART+CORnet ⭐⭐ (推荐长轨迹)

```
图像 
  ↓
V1层: Gabor滤波器 (4方向×2尺度)
  ↓
V2层: 局部对比度归一化 + 特征融合
  ↓
V4层: 多尺度池化 (3个尺度)
  ↓
IT层: 高层语义特征
  ↓
HART注意力: 多层融合 (底层50% + 中层30% + 高层20%)
  ↓
归一化 → 余弦距离匹配
```

**特点**:
- ✅ **轨迹完整性最高** (95.3%)
- ✅ **场景智能聚类** (5个VT覆盖1.8km)
- ✅ **全局一致性好** (轨迹误差4.74%)
- ⚠️ RMSE略高 (+7%)

**性能**:
```
Town01 (5000帧):
- VT数量: 5
- RMSE: 152.87m
- 轨迹完整性: 95.3% (最佳)
```

**适用场景**:
- 长距离导航 (>1km)
- 复杂城市环境
- 需要回环检测
- 全局地图构建

---

## 🔀 如何切换特征提取方法？

### 在 test_imu_visual_slam_hart_cornet.m 中:

```matlab
% 第15行：选择是否使用HART+CORnet
USE_HART_CORNET = true;   % true=HART+CORnet, false=简化增强
```

### 在 main.m 中:

```matlab
% 第68-69行：选择增强特征方法
USE_NEURO_FEATURE_EXTRACTOR = true;   % true=增强, false=原始
NEURO_FEATURE_METHOD = 'matlab';      % 'matlab'或'python'
```

---

## 📊 性能对比速查表

| 指标 | 原始方法 | 简化增强 | HART+CORnet |
|------|---------|---------|-------------|
| **速度** | ~50 FPS | **71 FPS** ⭐ | ~30 FPS |
| **VT数量** | 5 | 1365 | 5 |
| **RMSE** | 129.39m | **142.57m** | 152.87m |
| **轨迹完整性** | 95.3% | 38% ❌ | **95.3%** ⭐ |
| **鲁棒性** | 一般 | **优秀** ⭐ | 优秀 |
| **适用场景** | 基准 | 快速原型 | 长轨迹 |

**推荐选择**:
- **研究实验** → 简化增强 (最快, 强鲁棒)
- **长轨迹SLAM** → HART+CORnet (最完整)
- **局部定位** → 简化增强 (最低RMSE)

---

## 🎯 参数调优速查表

### VT匹配阈值 (核心参数)

```matlab
VT_MATCH_THRESHOLD = 0.07;  % 默认值
```

| 阈值 | VT数量 | 效果 | 适用场景 |
|------|--------|------|---------|
| 0.15 | 少 (~5) | 场景过度聚类 | 原始基准 |
| 0.10 | 中 (~600) | 平衡 | 一般场景 |
| **0.07** | 多 (~1300) | **最佳局部精度** | **简化增强推荐** |
| 0.05 | 很多 (~2000+) | 可能过度分割 | 复杂城市 |

**调优原则**:
```
阈值 ↑ → VT数量 ↓ → 轨迹完整性 ↑ → RMSE ↑
阈值 ↓ → VT数量 ↑ → 轨迹完整性 ↓ → RMSE ↓
```

### 经验地图阈值

```matlab
DELTA_EXP_GC_HDC_THRESHOLD = 15;  % 默认值
```

| 阈值 | 节点数量 | 效果 |
|------|---------|------|
| 10 | 多 | 细粒度地图，计算量大 |
| **15** | 中 | **平衡** ⭐ |
| 20 | 少 | 粗粒度地图，快速 |

### IMU融合权重

```matlab
% 在 imu_aided_visual_odometry.m 中
ALPHA_YAW = 0.7;     % IMU权重(偏航): 70% IMU + 30% 视觉
ALPHA_TRANS = 0.3;   % IMU权重(平移): 30% IMU + 70% 视觉
```

**权重设计原理**:
- **偏航依赖IMU** (70%): 陀螺仪短期精度高
- **平移依赖视觉** (70%): 加速度计积分漂移大

---

## 🐛 常见问题快速诊断

### 问题1: "找不到函数"

```matlab
% 错误信息
Undefined function or variable 'visual_template_hart_cornet'

% 解决方案 (在neuro目录下执行)
addpath(genpath('.'));
savepath;

% 或重新运行
quick_test_integration
```

---

### 问题2: "VT数量异常"

**VT过多 (>2000)**:
```matlab
% 增加阈值
VT_MATCH_THRESHOLD = 0.10;  % 从0.07增加
```

**VT过少 (<5)**:
```matlab
% 减小阈值
VT_MATCH_THRESHOLD = 0.05;  % 从0.07减小

% 或切换到简化增强
USE_HART_CORNET = false;
```

---

### 问题3: "轨迹压缩严重"

**症状**: 轨迹完整性 <50%

**原因**: VT切换过频 → 位移累积不足

**解决方案**:
```matlab
% 方案1: 使用HART+CORnet
USE_HART_CORNET = true;

% 方案2: 增加VT阈值
VT_MATCH_THRESHOLD = 0.10;

% 方案3: 增加经验地图阈值
DELTA_EXP_GC_HDC_THRESHOLD = 20;
```

---

### 问题4: "RMSE过高"

**症状**: RMSE > 250m

**诊断步骤**:
```matlab
% 1. 检查数据路径
fprintf('数据路径: %s\n', data_path);
fprintf('图像数量: %d\n', length(dir(fullfile(data_path, '*.png'))));

% 2. 检查VT状态
fprintf('VT数量: %d\n', NUM_VT);
fprintf('VT阈值: %.3f\n', VT_MATCH_THRESHOLD);

% 3. 检查经验节点
fprintf('经验节点: %d\n', NUM_EXPS);
fprintf('节点阈值: %d\n', DELTA_EXP_GC_HDC_THRESHOLD);

% 4. 运行诊断脚本
cd 07_test/test_imu_visual_slam
analyze_hart_features  % 分析特征距离分布
```

---

## 📚 学习路径推荐

### 初学者 (0-1周)

1. **阅读文档**:
   - `README.md` - 项目概述
   - `START_HERE.md` - 快速开始
   - `COMPLETE_SYSTEM_GUIDE.md` - 完整指南 ⭐

2. **运行测试**:
   ```matlab
   quick_test_integration          % 30秒测试
   test_imu_visual_slam_hart_cornet  % 完整SLAM
   ```

3. **理解核心概念**:
   - Grid Cell (3D位置编码)
   - Visual Template (场景识别)
   - Experience Map (拓扑地图)

---

### 进阶用户 (1-2周)

1. **深入源码**:
   - `main.m` - 主循环
   - `exp_map_iteration.m` - 经验地图
   - `gc_iteration.m` - 网格细胞

2. **参数调优**:
   ```matlab
   test_town10_tuning  % 批量测试
   ```

3. **采集自己的数据**:
   ```bash
   python IMU_Vision_Fusion_EKF.py
   ```

---

### 研究人员 (2周+)

1. **修改特征提取**:
   - `extract_features_hart_cornet.m`
   - 实现新的CORnet层
   - 调整注意力机制

2. **扩展功能**:
   - 添加深度传感器
   - 实现语义SLAM
   - 多机器人协同

3. **发表论文**:
   - 使用`08_draw_fig_for_paper/`生成图表
   - 参考`latex/`中的模板

---

## 🎬 视频教程 (推荐)

虽然没有视频，但可以按以下顺序学习:

1. **5分钟**: 运行`quick_test_integration`
2. **10分钟**: 运行完整Town01 SLAM
3. **20分钟**: 阅读`COMPLETE_SYSTEM_GUIDE.md`
4. **1小时**: 调试`main.m`理解主循环
5. **1天**: 采集并运行自己的数据

---

## 📞 获取帮助

### 文档导航

- **快速开始**: `START_HERE.md`
- **完整指南**: `COMPLETE_SYSTEM_GUIDE.md` ⭐⭐⭐
- **HART特征**: `HART_CORNET_SUMMARY.md`
- **本文档**: `QUICK_START_VISUAL_GUIDE.md`

### 代码导航

```
关键文件清单:
├── main.m                          # ⭐⭐⭐ 主程序入口
├── test_imu_visual_slam_hart_cornet.m  # ⭐⭐ SLAM测试
├── extract_features_hart_cornet.m  # ⭐ 特征提取
├── exp_map_iteration.m             # ⭐ 经验地图
├── gc_iteration.m                  # Grid Cell
└── yaw_height_hdc_iteration.m      # HDC
```

---

## 🎉 总结

### 系统核心价值

1. **类脑设计** - 模拟海马体空间认知
2. **多传感器** - RGB相机 + IMU融合
3. **实时性能** - 30-70 FPS处理速度
4. **长轨迹** - 支持>1.5km导航
5. **开源完整** - 纯MATLAB，易扩展

### 三步快速上手

```
步骤1: quick_test_integration        (30秒)
步骤2: test_imu_visual_slam_hart_cornet  (5分钟)
步骤3: 阅读 COMPLETE_SYSTEM_GUIDE.md  (30分钟)
```

### 选择合适的方法

```
快速原型 → 简化增强 (71 FPS)
长轨迹SLAM → HART+CORnet (95%轨迹完整性)
局部定位 → 简化增强 (142.57m RMSE)
```

---

**现在开始你的NeuroSLAM之旅吧！** 🚀

```matlab
% 确保你在neuro目录下
quick_test_integration
```
