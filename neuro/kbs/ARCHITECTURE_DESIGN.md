# NeuroSLAM系统架构设计说明

## 📊 架构图概览

![NeuroSLAM Architecture](fig/neuroslam_architecture.pdf)

---

## 🎯 系统整体设计思路

### 核心理念
NeuroSLAM系统借鉴哺乳动物大脑的空间表征机制，实现脑启发的4自由度SLAM系统。

---

## 🏗️ 系统架构 (自底向上)

### 1️⃣ **输入层** (Bottom)
```
┌─────────────┐      ┌─────────────┐
│ RGB Camera  │      │ IMU Sensor  │
│ 120×160     │      │ Accel+Gyro  │
│ 10 FPS      │      │ Self-motion │
└─────────────┘      └─────────────┘
```

**功能**：
- RGB相机：提供视觉信息，120×160分辨率，10 FPS
- IMU传感器：提供自运动线索（加速度计+陀螺仪）

---

### 2️⃣ **增强视觉模板模块** (Left Column) ⭐创新点1

```
Input Image (120×160)
    ↓
┌─────────────────────┐
│ Preprocessing       │  ← 预处理
│ • Grayscale         │     灰度化
│ • CLAHE             │     自适应直方图均衡 (ClipLimit=0.02)
│ • Gaussian          │     高斯平滑 (σ=0.5)
└─────────────────────┘
    ↓
┌─────────────────────┐
│ Feature Extraction  │  ← V1-IT通路模拟
│ • V1-IT Pathway     │     腹侧视觉通路
│ • Row Normalization │     行归一化
│ • Resize to 64×64   │     标准尺寸
└─────────────────────┘
    ↓
┌─────────────────────┐
│ VT Matching         │  ← 模板匹配
│ • Cosine Distance   │     余弦距离
│ • Threshold = 0.07  │     阈值（原始：0.15）
└─────────────────────┘
    ↓
VT ID (Visual Template ID)
```

**创新点**：
- ✅ **5,880%改进**：VT数量从5个提升到299个
- ✅ **生物可解释性**：模拟视觉皮层V1-IT通路
- ✅ **鲁棒性**：CLAHE增强对光照变化的鲁棒性

**技术细节**：
```python
# 特征提取流程
1. Grayscale: RGB → Gray
2. CLAHE: adapthisteq(ClipLimit=0.02)
3. Gaussian: imgaussfilt(σ=0.5)
4. Normalization: Min-Max to [0,1]
5. Matching: Cosine Distance < 0.07
```

---

### 3️⃣ **联合姿态细胞网络** (Center) ⭐创新点2

```
┌──────────────────────┐    ┌──────────────────────┐
│ 3D Grid Cell Network │    │ Multilayered HDC Net │
│                      │    │                      │
│ Position: (x, y, z)  │    │ Orientation: (yaw)   │
│                      │    │                      │
│ • Attractor Dynamics │    │ • 2D MD-CAN          │
│ • Path Integration   │    │ • Layer-wise         │
│ • LV Calibration     │    │ • Rotation Update    │
└──────────────────────┘    └──────────────────────┘
         │                            │
         └────────────┬───────────────┘
                      ↓
              4DoF Pose (x,y,z,yaw)
```

**3D网格细胞 (3D Grid Cells)**：
- **功能**：表示3D位置 $(x, y, z)$
- **实现**：3D多维连续吸引子网络 (3D MD-CAN)
- **过程**：
  1. **局部兴奋**：使用3D高斯权重
     ```
     ε(u,v,w) = Π[d∈{x,y,z}] (1/(δd√2π)) exp(-d²/(2δd²))
     ```
  2. **全局抑制**：维持活动包一致性
  3. **活动归一化**：总活动为1

**多层头方向细胞 (Multilayered HDC)**：
- **功能**：表示方向 (yaw)
- **实现**：2D MD-CAN，不同层代表不同高度
- **优势**：支持垂直空间的方向表示

**创新点**：
- ✅ **4DoF表示**：首次将3D网格细胞和多层HDC结合
- ✅ **生物启发**：基于蝙蝠和人类大脑的3D空间表征
- ✅ **路径积分**：支持基于自运动的位姿估计

---

### 4️⃣ **多层经验地图** (Right Column)

```
┌──────────────────────┐
│ Multilayered Exp Map │
│                      │
│ Nodes:               │
│ • VT ID              │  ← 视觉模板ID
│ • GC Activity        │  ← 网格细胞活动
│ • HDC Activity       │  ← 头方向细胞活动
│                      │
│ Links:               │
│ • Δpose (x,y,z,yaw)  │  ← 位姿变化
└──────────────────────┘
         ↓
┌──────────────────────┐
│ Loop Closure         │
│ & Relaxation         │  ← 回环检测与地图修正
└──────────────────────┘
```

**经验节点 (Experience Node)**：
```
E_new = (VT_id, GC_xyz, HDC_θ, Δpose)
```

**回环闭合 (Loop Closure)**：
- **触发条件**：当前VT与历史VT匹配
- **修正算法**：
  ```
  Δpos_i = α · (d_i / D) · Δ_loop
  ```
  - `d_i`: 节点i到回环点的路径距离
  - `D`: 总回环长度
  - `Δ_loop`: 累积误差

**优势**：
- ✅ **拓扑正确**：保持地图拓扑一致性
- ✅ **累积误差修正**：通过回环闭合减少漂移
- ✅ **重访利用**：重用已有经验节点

---

### 5️⃣ **输出层** (Top)

```
┌──────────────────────┐    ┌──────────────────────┐
│ 4DoF Pose Estimate   │    │ 3D Topological Map   │
│ (x, y, z, yaw)       │    │ with Loop Closure    │
└──────────────────────┘    └──────────────────────┘
```

---

## 🔄 数据流与反馈机制

### 前向流 (Forward Flow - 蓝色箭头)
```
RGB → VT → GC/HDC → Pose → Map
IMU → GC/HDC
```

### 反馈流 (Feedback - 红色箭头)
```
Loop Closure → Experience Map → Calibration
                  ↓
              GC/HDC Networks
```

**反馈作用**：
1. **局部视图校准**：当识别熟悉场景时，修正网格细胞活动
2. **回环修正**：检测到回环时，修正累积误差
3. **持续优化**：通过反馈持续改进位姿估计

---

## 📊 关键性能指标

| 指标 | 原始VT | 增强VT | 改进 |
|------|--------|--------|------|
| **VT数量** | 5 | 299 | **+5,880%** ⭐ |
| **经验节点** | 186 | 426 | +129% |
| **RMSE (m)** | 129.39 | 126.16 | **-2.5%** ⭐ |
| **处理时间 (s)** | 64 | 189 | +195% |

**数据集**：Town01Data_IMU_Fusion (5,000帧, CARLA仿真)

---

## 🎨 架构图设计元素

### 颜色编码
- 🔵 **浅蓝** (Input): 传感器输入
- 🟠 **浅橙** (Visual): 视觉处理模块
- 🟢 **浅绿** (Pose): 姿态表示
- 🟣 **浅紫** (Map): 空间记忆
- 🟡 **浅黄** (Output): 系统输出

### 箭头含义
- **蓝色箭头** (→): 前向数据流
- **红色箭头** (→): 反馈校准

### 创新标注
- ⭐ **红星标记**：系统关键创新点
- 📦 **黄色高亮框**：创新点说明

---

## 🔬 生物学对应关系

| NeuroSLAM组件 | 生物学对应 | 功能 |
|--------------|-----------|------|
| VT Module | 腹侧视觉通路 (V1→IT) | 物体识别 |
| 3D Grid Cells | 内嗅皮层网格细胞 | 位置编码 |
| HDC | 头方向细胞 | 方向编码 |
| Experience Map | 海马体认知地图 | 空间记忆 |

---

## 💻 代码实现

### 生成架构图
```bash
cd /home/dream/neuro_111111/carla-pedestrians/neuro/kbs
python3 draw_neuroslam_architecture.py
```

**输出**：
- `fig/neuroslam_architecture.pdf` (矢量图，55KB)
- `fig/neuroslam_architecture.png` (位图，730KB)

---

## 📝 论文中的引用

```latex
\begin{figure}[t]
    \centering
    \includegraphics[width=0.95\linewidth]{fig/neuroslam_architecture.pdf}
    \caption{NeuroSLAM system architecture...}
    \label{fig:architecture}
\end{figure}
```

---

## 🚀 未来改进方向

1. **6DoF扩展**：加入pitch和roll，完整3D头方向细胞
2. **深度学习集成**：结合深度学习的场景识别
3. **神经形态硬件**：部署到低功耗神经形态芯片
4. **真实机器人验证**：在物理机器人平台测试

---

## 📚 参考资料

- **Grid Cells**: Hafting et al., 2005
- **3D Grid Cells**: Finkelstein et al., 2016
- **Head Direction Cells**: Taube et al., 1990
- **RatSLAM**: Milford & Wyeth, 2008
- **CORnet**: Kubilius et al., 2018

---

**最后更新**: 2024-12-08  
**作者**: Haidong Wang, Caixia Ning  
**机构**: Hunan University of Technology and Business
