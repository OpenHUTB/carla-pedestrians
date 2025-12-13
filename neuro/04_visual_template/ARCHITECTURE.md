# HART+CORnet 特征提取器架构详解

## 系统架构图

```
输入图像 (RGB/Gray)
    ↓
┌─────────────────────────────────────┐
│  CORnet 层次化特征提取流程           │
├─────────────────────────────────────┤
│                                     │
│  V1层: 简单细胞                      │
│  ├─ Gabor滤波器 (4方向 × 2尺度)     │
│  ├─ Sobel边缘检测                   │
│  └─ 融合 (50%+50%)                  │
│         ↓                           │
│  V2层: 复杂细胞                      │
│  ├─ 局部最大池化 (2×2)              │
│  ├─ 非线性激活 (ReLU-like)          │
│  └─ 高斯平滑 (σ=1.0)                │
│         ↓                           │
│  V4层: 中层特征整合                  │
│  ├─ 多尺度提取 (σ=1.5, 3.0, 5.0)   │
│  ├─ 加权融合 (50%+30%+20%)          │
│  └─ 自适应对比度增强                │
│         ↓                           │
│  IT层: 高层语义特征                  │
│  ├─ 全局统计归一化                  │
│  ├─ 非线性变换 (tanh)               │
│  └─ 平滑整合 (σ=2.0)                │
│                                     │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│  HART 注意力机制                     │
├─────────────────────────────────────┤
│                                     │
│  底层注意力 (30%) ← V1特征          │
│  中层注意力 (40%) ← V4特征          │
│  高层注意力 (30%) ← 局部对比        │
│         ↓                           │
│  层次化融合                          │
│  空间注意力图                        │
│         ↓                           │
│  特征调制                            │
│  attended_features = IT × attention │
│                                     │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│  HART 时序建模 (LSTM)                │
├─────────────────────────────────────┤
│                                     │
│  持久化状态:                         │
│  ├─ lstm_hidden_state (h_t)        │
│  └─ lstm_cell_state (c_t)          │
│         ↓                           │
│  LSTM单元:                          │
│  ├─ 遗忘门: f_t = 0.7              │
│  ├─ 输入门: i_t = 0.3              │
│  ├─ 输出门: o_t = 0.8              │
│  │                                  │
│  │  c_t = f_t * c_{t-1} + i_t * x_t │
│  │  h_t = o_t * tanh(c_t)           │
│  │                                  │
│  └─ temporal_features = h_t         │
│                                     │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│  特征融合                            │
├─────────────────────────────────────┤
│                                     │
│  fused_features =                   │
│    0.6 * attended_features +        │
│    0.4 * temporal_features          │
│                                     │
└─────────────────────────────────────┘
         ↓
┌─────────────────────────────────────┐
│  归一化输出                          │
├─────────────────────────────────────┤
│                                     │
│  Min-Max归一化到 [0, 1]             │
│                                     │
└─────────────────────────────────────┘
         ↓
   输出特征图 (normalized)
```

---

## 详细模块说明

### 1. V1层：简单细胞

**生物学背景**:
- V1初级视觉皮层的简单细胞对特定方向的边缘敏感
- 不同神经元对不同方向和空间频率响应

**实现细节**:

#### Gabor滤波器
```matlab
orientations = [0, 45, 90, 135];  % 度
wavelengths = [4, 8];             % 像素
sigma = 3;                        % 高斯标准差

Gabor(x,y) = exp(-(x'^2 + y'^2)/(2σ^2)) * cos(2π * x'/λ)
```

其中:
- `x' = x*cos(θ) + y*sin(θ)` (旋转坐标)
- `λ` 是波长
- `σ` 是高斯包络标准差
- `θ` 是方向

#### Sobel算子
```matlab
Gx = [-1  0  1]       Gy = [-1 -2 -1]
     [-2  0  2]            [ 0  0  0]
     [-1  0  1]            [ 1  2  1]

edge_magnitude = sqrt(Gx^2 + Gy^2)
```

#### 融合策略
```matlab
v1_out = 0.5 * mean(gabor_responses) + 0.5 * edge_magnitude
```

**输出**: 边缘和方向敏感的特征图

---

### 2. V2层：复杂细胞

**生物学背景**:
- V2复杂细胞对位置有一定不变性
- 整合多个简单细胞的输出

**实现细节**:

#### 局部最大池化
```matlab
pooled = imerode(v1_features, strel('square', 2))
```
- 近似2×2最大池化
- 保留局部最强响应

#### 非线性激活
```matlab
v2_features = max(pooled, 0)  % ReLU-like
```

#### 平滑处理
```matlab
v2_out = imgaussfilt(v2_features, 1.0)
```

**输出**: 对位置轻微偏移不敏感的特征

---

### 3. V4层：中层特征整合

**生物学背景**:
- V4对形状、纹理、曲率等中层特征敏感
- 整合多个尺度的信息

**实现细节**:

#### 多尺度特征
```matlab
scale1 = imgaussfilt(v2_features, 1.5)  % 细节 (高频)
scale2 = imgaussfilt(v2_features, 3.0)  % 中等 (中频)
scale3 = imgaussfilt(v2_features, 5.0)  % 粗略 (低频)
```

#### 金字塔融合
```matlab
v4_features = 0.5 * scale1 +   % 50% 细节
              0.3 * scale2 +   % 30% 中等
              0.2 * scale3     % 20% 粗略
```

#### 对比度增强
```matlab
v4_out = adapthisteq(v4_features, 'ClipLimit', 0.02)
```

**输出**: 多尺度整合的中层特征

---

### 4. IT层：高层语义特征

**生物学背景**:
- IT（下颞叶）对物体类别和身份敏感
- 高度抽象的表示

**实现细节**:

#### 全局归一化
```matlab
global_mean = mean(v4_features(:))
global_std = std(v4_features(:))
normalized = (v4_features - global_mean) / (global_std + eps)
```

#### 非线性变换
```matlab
it_features = tanh(normalized)
```
- tanh将特征压缩到[-1, 1]
- 保留相对关系

#### 平滑整合
```matlab
it_out = imgaussfilt(it_features, 2.0)
```

**输出**: 高层抽象的语义特征

---

### 5. HART 注意力机制

**核心思想**:
- 模拟人类视觉注意力
- 自动关注图像中重要区域
- 抑制冗余背景

**三层注意力**:

```matlab
% 底层: 基于边缘
low_level = normalize(v1_features)

% 中层: 基于纹理和结构
mid_level = normalize(v4_features)

% 高层: 基于局部对比
high_level = normalize(abs(img - imgaussfilt(img, 5)))

% 层次化融合
attention = 0.3 * low_level +
            0.4 * mid_level +
            0.3 * high_level
```

**注意力调制**:
```matlab
attended_features = it_features .* attention
```

**特点**:
- 多层融合提高鲁棒性
- 空间自适应
- 保留重要信息

---

### 6. HART 时序建模 (LSTM)

**核心思想**:
- 利用视频的时序连续性
- 平滑帧间变化
- 提高轨迹稳定性

**LSTM单元**:

```
输入: x_t (当前特征)
状态: h_{t-1} (隐藏状态), c_{t-1} (细胞状态)

遗忘门: f_t = 0.7
输入门: i_t = 0.3
输出门: o_t = 0.8

更新细胞状态:
c_t = f_t * c_{t-1} + i_t * tanh(x_t)

更新隐藏状态:
h_t = o_t * tanh(c_t)

输出: h_t
```

**简化说明**:
- 保留70%历史信息 (遗忘门)
- 接受30%新信息 (输入门)
- 平滑输出 (输出门)

**持久化状态**:
```matlab
persistent lstm_hidden_state;
persistent lstm_cell_state;
```

---

## 数据流示例

### 输入图像
```
原始RGB图像: [120 × 160 × 3]
    ↓ rgb2gray
灰度图像: [120 × 160]
    ↓ normalize
归一化图像: [120 × 160], 范围[0, 1]
```

### V1输出
```
Gabor响应: [120 × 160 × 8]  (8个滤波器)
Sobel边缘: [120 × 160]
    ↓ 融合
V1特征: [120 × 160]
```

### V2输出
```
V1特征: [120 × 160]
    ↓ 池化
池化特征: [60 × 80]  (近似)
    ↓ 激活+平滑
V2特征: [60 × 80]
```

### V4输出
```
多尺度特征:
  scale1: [60 × 80]
  scale2: [60 × 80]
  scale3: [60 × 80]
    ↓ 融合
V4特征: [60 × 80]
```

### IT输出
```
V4特征: [60 × 80]
    ↓ 归一化+tanh
IT特征: [60 × 80], 范围[-1, 1]
```

### 注意力
```
底层/中层/高层注意力: [60 × 80]
    ↓ 加权融合
注意力图: [60 × 80], 范围[0, 1]
```

### 时序建模
```
当前特征: [60 × 80]
历史状态: [60 × 80]
    ↓ LSTM
时序特征: [60 × 80]
```

### 最终输出
```
attended_features + temporal_features
    ↓ 融合+归一化
最终特征: [60 × 80], 范围[0, 1]
```

注意: 实际尺寸取决于VT配置，这里假设为标准的12×16→处理→输出尺寸

---

## 计算复杂度分析

### 时间复杂度 (单帧)

| 模块 | 复杂度 | 时间占比 |
|------|--------|---------|
| V1 (Gabor) | O(N*M*K*F) | ~40% |
| V2 (Pooling) | O(N*M) | ~5% |
| V4 (Multi-scale) | O(N*M*S) | ~20% |
| IT (Normalize) | O(N*M) | ~5% |
| Attention | O(N*M) | ~10% |
| LSTM | O(N*M) | ~10% |
| Others | - | ~10% |

其中:
- N, M: 图像高度和宽度
- K: Gabor核大小
- F: 滤波器数量
- S: 尺度数量

### 空间复杂度

| 数据 | 大小 | 说明 |
|------|------|------|
| 输入图像 | N×M | 单通道 |
| V1响应 | N×M×F | F个滤波器 |
| LSTM状态 | 2×N'×M' | hidden + cell |
| 其他中间变量 | ~5×N×M | 各层特征 |

**总计**: 约 (F+8)×N×M

---

## 参数配置建议

### 性能优先配置
```matlab
% 减少Gabor滤波器
orientations = [0, 90];  % 2个方向
wavelengths = [4];       % 1个尺度

% 更快的平滑
sigma_v2 = 0.5;
sigma_v4 = [1.0, 2.0];   % 2个尺度

% 更简单的LSTM
forget_rate = 0.5;
```

### 精度优先配置
```matlab
% 更多Gabor滤波器
orientations = [0, 30, 60, 90, 120, 150];  % 6个方向
wavelengths = [3, 6, 12];                  % 3个尺度

% 更细致的多尺度
sigma_v4 = [1.0, 2.0, 4.0, 8.0];  % 4个尺度

% 更长的时序记忆
forget_rate = 0.9;
```

### 平衡配置（默认）
```matlab
orientations = [0, 45, 90, 135];  % 4个方向
wavelengths = [4, 8];             % 2个尺度
sigma_v4 = [1.5, 3.0, 5.0];       % 3个尺度
forget_rate = 0.7;
```

---

## 与原方法对比

### 原始简单方法

```
输入 → 灰度化 → CLAHE → Sobel边缘 → 融合 → 归一化 → 输出
```

**特点**:
- 单层处理
- 无层次化
- 无注意力
- 无时序建模
- 速度快但特征表达有限

### HART+CORnet方法

```
输入 → V1 → V2 → V4 → IT → 注意力 → 时序 → 融合 → 输出
```

**特点**:
- 4层层次化处理
- 多尺度特征
- 空间注意力机制
- LSTM时序建模
- 更强的特征表达能力

---

## 生物学启发

### V1-V2-V4-IT通路

这个架构模拟了灵长类视觉系统的腹侧通路（ventral stream）:

```
视网膜 → LGN → V1 → V2 → V4 → IT → PFC
(输入)         (边缘)(纹理)(形状)(物体)(决策)
```

### 注意力机制

模拟了视觉注意力的选择性处理:
- 显著性检测
- 资源分配
- 背景抑制

### 时序整合

模拟了视觉记忆和时序整合:
- 短期记忆
- 运动连续性
- 预测编码

---

## 参考资料

1. **CORnet论文**: Kubilius et al., "Brain-like object recognition with high-performing shallow recurrent ANNs", NeurIPS 2018

2. **HART论文**: Kosiorek et al., "Hierarchical attentive recurrent tracking", NeurIPS 2017

3. **视觉皮层**: Felleman & Van Essen, "Distributed hierarchical processing in the primate cerebral cortex", 1991

4. **注意力机制**: Itti et al., "A model of saliency-based visual attention for rapid scene analysis", 1998

5. **LSTM**: Hochreiter & Schmidhuber, "Long short-term memory", 1997
