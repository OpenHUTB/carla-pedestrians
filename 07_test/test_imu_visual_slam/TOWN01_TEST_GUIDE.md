# Town01 SLAM测试指南

## ✅ 数据采集完成

您已成功在Town01采集数据：
```bash
数据目录: /data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion/
- fusion_pose.txt     : 849KB ✅
- ground_truth.txt    : 487KB ✅ (自动生成)
- aligned_imu.txt     : 331KB ✅
- IMAGEMATRIX/        : 5000张图像 ✅
```

---

## 🎯 Town01 vs Town10 场景特征

### Town01 - 狭窄街道
- **环境**: 城市街道，建筑密集
- **视觉特征**: 近距离建筑物，视野受限
- **轨迹特点**: 转弯较多，速度较慢
- **推荐参数**: 较小的尺度因子

### Town10HD - 开阔道路  
- **环境**: 高速公路，视野开阔
- **视觉特征**: 远距离地标，视野广
- **轨迹特点**: 直线较多，速度较快
- **推荐参数**: 较大的尺度因子

---

## 🔧 已调整的参数

### 视觉里程计尺度
```matlab
% Town10HD (之前)
ODO_TRANS_V_SCALE = 24

% Town01 (当前)
ODO_TRANS_V_SCALE = 22  ← 降低8%
```

**原因**: Town01街道狭窄，视觉特征更近，需要更小的尺度

### 网格细胞尺度
```matlab
% Town10HD (之前)
GC_HORI_TRANS_V_SCALE = 0.8

% Town01 (当前)
GC_HORI_TRANS_V_SCALE = 0.7  ← 降低12.5%
```

### 数据路径
```matlab
% Town10HD (之前)
data_path = 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion';

% Town01 (当前)
data_path = 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion';  ✅
```

---

## 🚀 运行Town01测试

```matlab
cd /home/dream/neuro_111111/carla-pedestrians/neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

### 预期输出
```
[1/9] 初始化路径...
[2/9] 初始化全局变量...
[3/9] 初始化模块参数...
[4/9] 读取IMU-视觉融合数据...
  数据路径: .../Town01Data_IMU_Fusion  ← Town01数据
  成功读取Ground Truth数据 ✅

[5/9] SLAM处理完成！
  经验地图节点数: 150-300  ← Town01转弯多，节点较多
  
[7/9] 评估轨迹精度...
--- IMU-视觉融合轨迹 vs Ground Truth (对齐后) ---
  轨迹长度误差: <5%  ← 应该很准确
  RMSE: XX.XX m

--- 视觉里程计轨迹 vs Ground Truth (对齐后) ---
  轨迹长度误差: <10%  ← 调整后应该改善
  
--- 经验地图轨迹 vs Ground Truth (对齐后) ---
  轨迹长度误差: <15%  ← 可能比Town10稍高（转弯多）
```

---

## 📊 Town01预期性能

### IMU-Visual Fusion
- **轨迹长度误差**: <3% (最佳)
- **RMSE**: 30-60m (取决于轨迹长度)
- **优势**: EKF融合，物理测量准确

### Visual Odometry  
- **轨迹长度误差**: 5-10% (调整后)
- **RMSE**: 60-120m
- **特点**: 狭窄街道可能受视觉遮挡影响

### Experience Map
- **轨迹长度误差**: 10-20%
- **RMSE**: 100-180m
- **特点**: Town01转弯多，累积误差可能较大

---

## 🔍 参数微调方法

如果第一次运行发现视觉里程计距离仍不准确：

### 步骤1: 查看轨迹长度误差
```
--- 视觉里程计轨迹 vs Ground Truth (对齐后) ---
轨迹长度:
  估计值:   XXXX.XX m
  真值:     YYYY.YY m
  误差:     ZZ.ZZ m (W.W%)
```

### 步骤2: 计算校准因子
```matlab
% 如果估计值偏大（如误差+15%）
new_scale = 22 * (真值 / 估计值)

% 例如：估计1200m, 真值1000m
new_scale = 22 * (1000 / 1200) = 18.3 ≈ 18
```

### 步骤3: 修改参数
```matlab
visual_odo_initial('ODO_TRANS_V_SCALE', 18, ...);
gc_initial('GC_HORI_TRANS_V_SCALE', 0.6, ...);
```

### 步骤4: 重新运行测试
验证误差是否降到<5%

---

## 📈 Town01特有问题

### 1. 视觉遮挡
Town01建筑物密集，可能出现：
- 视觉模板匹配失败
- 视觉里程计抖动

**解决**: IMU-Visual Fusion会自动处理（IMU填补视觉失效期）

### 2. 频繁转弯
Town01转弯多，可能导致：
- 经验地图节点数增加
- 累积误差增大

**解决**: 适当降低`DELTA_EXP_GC_HDC_THRESHOLD`（已设为15）

### 3. 速度变化大
Town01有加速/减速/转弯：
- 视觉里程计尺度估计可能不稳定

**解决**: IMU-Visual Fusion自适应调整

---

## ✅ 验证清单

运行测试后检查：

### 数据读取
- [ ] 成功读取5000帧图像
- [ ] 成功读取Ground Truth数据
- [ ] 数据路径显示Town01

### 精度评估
- [ ] IMU-Visual Fusion长度误差 < 3%
- [ ] Visual Odometry长度误差 < 10%
- [ ] Experience Map长度误差 < 20%

### 可视化
- [ ] 图像保存在`Town01Data_IMU_Fusion/slam_results/`
- [ ] 3D轨迹图显示合理的形状
- [ ] 误差曲线在可接受范围

---

## 🔄 切换回Town10测试

如果想切换回Town10：

```matlab
% 修改 test_imu_visual_fusion_slam.m
data_path = 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion';

% 调整参数
visual_odo_initial('ODO_TRANS_V_SCALE', 24, ...);  % Town10用24
gc_initial('GC_HORI_TRANS_V_SCALE', 0.8, ...);     % Town10用0.8
```

---

## 📊 对比Town01 vs Town10

测试完成后，可以对比两个场景：

### 轨迹长度
```
Town01: ~1000-1500m (取决于路线)
Town10: ~1500-2000m (通常更长)
```

### 精度表现
```
Town01 IMU-Fusion: RMSE 30-60m
Town10 IMU-Fusion: RMSE 60-80m

Town01更准确 (场景更小，误差累积少)
```

### 经验地图节点数
```
Town01: 200-400个 (转弯多)
Town10: 100-200个 (直线多)
```

---

## 💡 Town01优化建议

### 如果误差较大

**问题1**: 视觉里程计长度误差>15%
```matlab
# 解决：进一步降低尺度
ODO_TRANS_V_SCALE = 18-20
```

**问题2**: 经验地图节点太少<50
```matlab
# 解决：降低阈值
DELTA_EXP_GC_HDC_THRESHOLD = 10
```

**问题3**: 轨迹抖动严重
```matlab
# 解决：增加平滑
EXP_CORRECTION = 0.7  (从0.5提高)
```

---

## 📚 相关文档

- **SCALE_CALIBRATION_GUIDE.md** - 尺度校准详解
- **FIX_SUMMARY.md** - 问题修复总结
- **TRAJECTORY_ALIGNMENT_GUIDE.md** - 轨迹对齐原理

---

## 🎯 总结

| 项目 | 状态 | 说明 |
|------|------|------|
| 数据采集 | ✅ | Town01数据已准备好 |
| Ground Truth | ✅ | 自动生成，487KB |
| 测试脚本 | ✅ | 路径已改为Town01 |
| 参数调整 | ✅ | 适配Town01场景 |

**立即运行**: `test_imu_visual_fusion_slam`  
**预期时间**: 5-10分钟处理5000帧

---

**创建时间**: 2024年11月29日  
**测试场景**: Town01 (狭窄街道)  
**状态**: ✅ 所有准备工作完成，可立即测试
