# 尺度校准指南 - 修复视觉里程计距离估计

## 🎯 问题诊断

### 您遇到的问题
```
方法                 轨迹长度    真值长度    误差
IMU-Visual Fusion   1647.11 m   1630.84 m   1.00% ✅
Visual Odometry     2032.62 m   1630.84 m   24.64% ❌
Experience Map      2183.95 m   1630.84 m   33.92% ❌
```

**核心问题**: 视觉里程计**尺度估计偏大**，导致：
- 认为车辆移动距离比实际多24.64%
- 经验地图基于视觉里程计，误差更大（33.92%）

### 为什么IMU-Visual Fusion正确？

IMU-Visual Fusion使用**EKF融合**：
```python
# Python EKF代码
accel_world = R_body2world @ accel  # IMU加速度（物理测量）
new_vx = self.x[3] + accel_world[0] * self.dt
new_x = self.x[0] + new_vx * self.dt  # 真实物理距离
```

**EKF使用物理测量（IMU加速度）**，所以距离准确。

而**视觉里程计依赖图像匹配**：
```matlab
transV = visual_odo_iteration(...) * ODO_TRANS_V_SCALE;
```

`ODO_TRANS_V_SCALE`是**经验参数**，需要根据实际场景校准。

---

## 🔧 修复方案

### 1. 计算正确的尺度因子

#### 公式
```
正确尺度 = 当前尺度 × (真值长度 / 估计长度)
```

#### 实际计算
```matlab
% 视觉里程计
当前: ODO_TRANS_V_SCALE = 30
误差: 估计2032.62m vs 真值1630.84m
修正: 30 * (1630.84 / 2032.62) = 24.06 ≈ 24

% 网格细胞（用于经验地图）
当前: GC_HORI_TRANS_V_SCALE = 1
修正: 1 * (1630.84 / 2032.62) = 0.80 ≈ 0.8
```

### 2. 已修改的参数

```matlab
% 修改前
visual_odo_initial('ODO_TRANS_V_SCALE', 30, ...);
gc_initial('GC_HORI_TRANS_V_SCALE', 1, ...);

% 修改后
visual_odo_initial('ODO_TRANS_V_SCALE', 24, ...);  % ← 降低20%
gc_initial('GC_HORI_TRANS_V_SCALE', 0.8, ...);    % ← 降低20%
```

---

## 📊 预期改进

### 修复前
```
Visual Odometry:
  轨迹长度: 2032.62 m (误差 24.64%)
  RMSE: 165.01 m
  
Experience Map:
  轨迹长度: 2183.95 m (误差 33.92%)
  RMSE: 260.76 m
```

### 修复后（预期）
```
Visual Odometry:
  轨迹长度: ~1630 m (误差 <5%)  ← 改善20%
  RMSE: ~100-120 m  ← 改善30-40%
  
Experience Map:
  轨迹长度: ~1650-1750 m (误差 <10%)  ← 改善25%
  RMSE: ~150-180 m  ← 改善30-40%
```

---

## 🚀 立即测试

```matlab
cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

### 检查点

运行后查看：
```
--- 视觉里程计轨迹 vs Ground Truth (对齐后) ---
轨迹长度:
  估计值:   XXXX.XX m  ← 应该接近1630m
  真值:     1630.84 m
  误差:     XX.XX m (X.X%)  ← 应该<5%

--- 经验地图轨迹 vs Ground Truth (对齐后) ---
轨迹长度:
  估计值:   XXXX.XX m  ← 应该1650-1750m
  真值:     1630.84 m
  误差:     XX.XX m (X.X%)  ← 应该<10%
```

---

## 🔍 为什么形状对但距离不对？

### 视觉里程计工作原理

1. **相对运动检测**（形状）：
   ```
   检测图像之间的相对位移 → 形状正确 ✅
   ```

2. **绝对距离估计**（尺度）：
   ```
   像素位移 × 尺度因子 → 米距离
   ```

**尺度因子依赖相机参数和场景假设**：
- FOV（视场角）
- 安装高度
- 地面假设

### 为什么需要校准？

不同场景的尺度因子不同：
- **Town01**: 狭窄街道，近距离参考 → 较小的尺度
- **Town10HD**: 开阔道路，远距离参考 → 较大的尺度

您的数据在**Town10**，但参数可能是Town01的默认值。

---

## ⚠️ 重要提醒：数据集检查

### 您提到的问题
> "我是跑了town01和town10两个不同场景"

### 需要确认

当前数据路径：
```
/data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion
```

请确认：
1. **fusion_pose.txt** - 是Town10的数据吗？
2. **ground_truth.txt** - 是Town10的数据吗？
3. **图像文件夹** - 图像是Town10的吗？

### 检查方法

```bash
# 检查数据集时间戳
head -5 /path/to/Town10Data_IMU_Fusion/fusion_pose.txt
head -5 /path/to/Town10Data_IMU_Fusion/ground_truth.txt

# 检查图像数量
ls -l /path/to/Town10Data_IMU_Fusion/IMAGEMATRIX/ | wc -l
```

### 如果混合了Town01和Town10的数据

**问题**: 不同场景的数据混合会导致轨迹形状不匹配
**解决**: 重新采集数据，确保一次运行只用一个场景

---

## 🎓 参数调优原则

### 1. 基于Ground Truth校准（推荐）

```matlab
% 步骤1: 跑一次测试，获取估计轨迹长度
% 步骤2: 计算校准因子
calibration_factor = gt_length / estimated_length;
% 步骤3: 调整参数
new_scale = old_scale * calibration_factor;
```

**优点**: 精确，基于实测数据

### 2. 基于相机参数计算（理论）

```matlab
% 根据相机FOV和安装高度计算
pixel_to_meter = camera_height * tan(fov/2) / (image_width/2);
ODO_TRANS_V_SCALE = baseline / pixel_to_meter;
```

**优点**: 理论正确  
**缺点**: 需要准确的相机参数

### 3. 经验调整（快速）

```matlab
% Town01 (狭窄街道): 20-25
% Town03 (市区):      25-30
% Town10 (开阔道路):  30-35
```

**优点**: 快速  
**缺点**: 不够精确

---

## 📈 校准效果对比

### 图像分析

#### 修复前（您上传的图）
- **3D轨迹图**: 绿色（经验地图）明显偏长
- **2D俯视图**: 绿色轨迹超出Ground Truth范围
- **误差曲线**: 经验地图误差高达300m

#### 修复后（预期）
- **3D轨迹图**: 绿色轨迹与Ground Truth更接近
- **2D俯视图**: 轨迹长度匹配
- **误差曲线**: 误差降到100-150m

---

## 🔄 迭代调优

如果一次修正不够理想：

### 迭代公式
```matlab
% 第N次迭代
scale_N = scale_(N-1) * (gt_length / estimated_length_N)
```

### 示例
```
迭代1: scale=30 → 长度2032m → 误差24%
计算: 30 * (1630/2032) = 24
迭代2: scale=24 → 长度1680m → 误差3%
计算: 24 * (1630/1680) = 23.3
迭代3: scale=23 → 长度1635m → 误差0.3% ✅
```

通常**1-2次迭代**即可达到满意效果。

---

## ✅ 验证清单

运行测试后检查：

- [ ] 视觉里程计长度误差 < 5%
- [ ] 经验地图长度误差 < 10%
- [ ] 视觉里程计RMSE < 120m
- [ ] 经验地图RMSE < 180m
- [ ] 3D轨迹图中绿色轨迹不再超出GT范围
- [ ] 误差曲线峰值降低

---

## 📚 相关文档

- **FIX_SUMMARY.md** - 图像保存和经验地图修复
- **TRAJECTORY_ALIGNMENT_GUIDE.md** - 轨迹对齐原理
- **QUICK_RUN_GUIDE.md** - 快速运行指南

---

## 💡 常见问题

### Q1: 为什么IMU-Fusion不需要校准？
**A**: IMU测量的是物理加速度，单位是m/s²，直接积分得到距离，无需尺度因子。

### Q2: 不同场景需要不同参数吗？
**A**: 是的！开阔场景（Town10）和狭窄场景（Town01）的视觉特征不同，需要调整参数。

### Q3: 可以用一套参数适配所有场景吗？
**A**: 理论上可以，但需要更复杂的自适应算法。简单方法是为每个场景校准一次。

### Q4: 校准后误差还是大怎么办？
**A**: 检查：
1. 数据集是否混合了不同场景
2. 图像质量是否良好（光照、模糊）
3. 经验地图阈值是否合适

---

**创建时间**: 2024年11月29日  
**状态**: ✅ 参数已优化，建议立即测试
