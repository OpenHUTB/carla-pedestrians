# 生物启发导航架构图 - 最终版更新总结

## ✅ 完成时间
2024年12月8日

---

## 📊 架构图改进内容

### 1. **左侧：Environment Input（环境输入）**

#### 改进前：
- 简单的场景照片，没有标注

#### 改进后：✅
- **CARLA Town01场景**（绿色标签）
  - 标注：Urban Road（城市道路）
  - 使用真实数据集照片：`0500.png`
  - 突出城市环境特征
  
- **CARLA Town10场景**（蓝色标签）
  - 标注：Different Scene（不同场景）
  - 使用真实数据集照片：`2500.png`
  - 展示场景差异性

- **图标增强**
  - 添加车辆图标（灰色矩形）
  - 添加行人图标（灰色圆形）
  - IMU和RGB输入标注

---

### 2. **中间：Perception Network（感知网络）**

#### 改进前：
- V1、V2、V4、MT/MST等节点只是简单圆圈
- 没有具体算法说明
- 缺少大脑图标

#### 改进后：✅

##### 2.1 **大脑轮廓图标**
- 添加椭圆形大脑轮廓（淡橙色，半透明）
- 参考用户提供的图片风格
- 中间分界线区分左右脑区

##### 2.2 **上半部分：Vestibular System（前庭系统）**
- 淡橙色背景区域
- **半规管图标**：三个不同角度的圆弧（0°、60°、120°）
- **IMU输入卡片**：
  - Acceleration（加速度）
  - Angular Velocity（角速度）

##### 2.3 **下半部分：Visual Cortex（视觉皮层）**

**Dorsal Pathway（背侧通路 - 紫色）：运动和空间处理**

| 节点 | 功能 | 实际算法 |
|------|------|----------|
| Retina/LGN | 视网膜/外侧膝状体 | Grayscale + CLAHE |
| V1 | 初级视觉皮层 | Gaussian Smooth |
| MT/MST | 中颞叶/上颞叶 | Motion Features |
| Path Integration | 路径积分 | 3D Grid Cells |

**Ventral Pathway（腹侧通路 - 橙色）：场景识别**

| 节点 | 功能 | 实际算法 |
|------|------|----------|
| Retina/LGN | 视网膜/外侧膝状体 | Grayscale + CLAHE |
| V1 | 初级视觉皮层 | Gaussian Smooth |
| V2/V4 | 次级/第四视觉皮层 | Downsample 64×64 |
| IT | 下颞叶 | Row-mean Norm |
| VT | 视觉模板 | Visual Template |

---

### 3. **右侧：Decision Network（决策网络）**

#### 改进前：
- DPC、MPC、VPC等简单框
- 没有标注实际的建图功能

#### 改进后：✅

##### 3.1 **Pose Representation（姿态表征）**

**3D Grid Cells（绿色框）**
- FCC Lattice（面心立方晶格）
- Position: (x, y, z)
- 3D空间位置表征

**Head Direction Cells（橙色框）**
- Multilayered（多层结构）
- Orientation: yaw
- 方向表征

##### 3.2 **Spatial Mapping（空间建图）**

**Experience Map（蓝色框）**
- 功能列表：
  - ✅ Loop Closure Detection（回环检测）
  - ✅ VT Association（视觉模板关联）
  - ✅ Map Relaxation（地图优化）

##### 3.3 **Navigation Control（导航控制）**

**Action Selection（紫色框）**
- CPC（Conjunctive Pose Cells，联合姿态细胞）
- 结合Grid Cells和HDC的决策

**Trajectory Planning（紫色框）**
- MLP（Multi-layer Perceptron，多层感知机）
- 轨迹规划和控制

---

### 4. **最右侧：Control Output（控制输出）**

#### 输出图标：
- **方向盘图标**（灰色圆形 + 横杆）
  - Steering Angle（转向角）
  
- **油门图标**（绿色矩形）
  - Acceleration（加速度）

---

### 5. **底部：Neural Anatomical Alignment**

- 淡粉色区域
- 连接Perception Network和Decision Network
- 强调生物学对应关系

---

## 🎨 视觉设计特点

### 配色方案（参考用户提供的图片）：
- **Perception Network**：淡米色 `#F8E8D8`
- **Decision Network**：淡橙色 `#FFE8D6`
- **Vestibular System**：柔和橙色 `#FFB88C`
- **Dorsal Pathway**：紫色系 `#B39DDB`
- **Ventral Pathway**：橙色系 `#FFB74D`
- **3D Grid Cells**：绿色 `#81C784`
- **HDC**：橙色 `#FFB74D`
- **Experience Map**：蓝色 `#64B5F6`
- **Action/Planning**：紫色 `#BA68C8`, `#9575CD`

### 形状和布局：
- **圆角矩形**：所有主要区域
- **圆形节点**：视觉通路各阶段
- **大脑轮廓**：椭圆形，半透明背景
- **箭头**：棕色系，粗线条
- **标签卡片**：白色背景，清晰标注

---

## 📝 LaTeX图注更新

### 更新后的Caption：

```latex
\caption{Bio-inspired navigation system architecture. 
\textbf{Left:} CARLA simulation environments (Town01 urban road, 
Town10 different scene) providing RGB images and IMU data. 
\textbf{Middle (Perception Network):} Brain-inspired sensory processing 
with (1) Vestibular system mimicking half-circular canals for IMU 
acceleration and angular velocity processing; (2) Visual cortex 
dual-stream processing—dorsal pathway (Retina/LGN→V1→MT/MST) for 
motion and spatial features leading to 3D grid cells, ventral pathway 
(Retina/LGN→V1→V2/V4→IT) for scene recognition using biologically-
plausible algorithms (CLAHE, Gaussian smoothing, row-mean normalization) 
producing visual templates (VT). 
\textbf{Right (Decision Network):} Spatial mapping and navigation with 
(1) 3D grid cells (FCC lattice) for position representation (x,y,z); 
(2) Multilayered head direction cells for orientation (yaw); 
(3) Experience map for spatial graph construction with loop closure 
detection, VT association, and map relaxation; (4) Action selection 
via conjunctive pose cells (CPC) and trajectory planning via MLP 
generating steering angle and acceleration control outputs. 
Brown arrows indicate information flow through the neural anatomical 
alignment.}
```

---

## 🔍 与参考图片的对比

### 参考图1特点：
- ✅ 简约的大脑轮廓图标
- ✅ 清晰的Perception/Decision分区
- ✅ 真实街景照片
- ✅ 方向盘和油门图标

### 参考图2特点：
- ✅ 中文标注（半规管、海马体等）
- ✅ 大脑解剖图标
- ✅ 空间地图可视化
- ✅ 多层次的神经网络连接

### 我们的改进：
- ✅ **结合两者优点**：英文专业术语 + 生物学图标
- ✅ **突出实际算法**：CLAHE、Gaussian、Row-mean等
- ✅ **强调建图功能**：Experience Map、Loop Closure
- ✅ **CARLA场景差异**：Town01和Town10标注清晰

---

## 📂 生成文件

```bash
neuro/kbs/
├── draw_bio_architecture_final.py      # 最终版脚本
├── fig/bio_nav_architecture.pdf        # 更新后的架构图PDF
├── fig/bio_nav_architecture.png        # 更新后的架构图PNG
└── NeuroSLAM_KBS.pdf                   # 更新后的论文（24页）
```

---

## ✨ 主要改进总结

| 改进点 | 改进前 | 改进后 |
|--------|--------|--------|
| **场景标注** | 无标注 | Town01/Town10清晰标注，绿色/蓝色标签 |
| **大脑图标** | 无 | 添加椭圆形大脑轮廓，半透明背景 |
| **视觉通路** | 简单圆圈 | 详细标注实际算法（CLAHE、Gaussian等） |
| **建图功能** | 简单框 | 明确标注Loop Closure、VT Association、Map Relaxation |
| **决策网络** | 抽象 | 具体标注3D Grid Cells、HDC、CPC、MLP |
| **输出控制** | 简单 | 方向盘和油门图标，直观清晰 |
| **配色** | 基础 | 参考用户图片，淡雅专业 |
| **布局** | 死板 | 圆角矩形，层次分明，简约现代 |

---

## 🎯 符合用户要求

✅ **图片突出两个场景差异**：Town01和Town10清晰标注，不同颜色标签  
✅ **标上CARLA Town01 Town10**：绿色和蓝色标签框  
✅ **中间的腹侧层和背侧层添加人脑图标**：大脑轮廓椭圆形图标  
✅ **V1 V4节点不简单**：详细标注实际算法（CLAHE、Gaussian、Row-mean等）  
✅ **列出系统的模型**：虽然不是HART和CornerR（这些是深度学习模型），但列出了实际使用的生物启发算法  
✅ **右边标注实际功能**：Experience Map、Loop Closure、3D Grid Cells、HDC、CPC、MLP  
✅ **参考提供的照片风格**：简约、淡雅配色、圆角设计、图标化表示  

---

## 📌 注意事项

1. **用户提到的HART和CornerR**：这些是深度学习目标检测模型，但实际论文系统使用的是生物启发的图像处理算法（CLAHE、Gaussian等），所以标注的是实际使用的方法。

2. **建图功能**：Experience Map（经验地图）就是系统的建图功能，包含Loop Closure Detection（回环检测）和Map Relaxation（地图优化）。

3. **3D Grid Cells和HDC**：这些是系统核心的空间表征机制，对应哺乳动物大脑的网格细胞和方向细胞。

---

## 🚀 下一步建议

如果需要进一步调整，可以：
1. 调整Town01和Town10的照片选择（使用更有代表性的场景）
2. 修改配色方案（如果需要更鲜艳或更淡雅的颜色）
3. 调整布局比例（左中右三部分的宽度）
4. 添加更多图标（如半规管的3D示意图）
5. 细化箭头连接（显示更详细的信息流）

---

**✅ 架构图更新完成！参考了用户提供的图片风格，突出了场景差异、添加了大脑图标、标注了实际算法和建图功能！**
