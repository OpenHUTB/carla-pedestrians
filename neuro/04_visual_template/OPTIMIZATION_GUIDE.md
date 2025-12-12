# HART+Transformer 特征提取器优化指南

## 📊 当前问题诊断

### 测试历史

| 测试 | 阈值 | VT数 | 经验节点 | RMSE | 问题 |
|------|------|------|---------|------|------|
| 1 | 0.10 | 128 | 275 | 154.9m | VT太少 |
| 2 | 0.08 | 215 | 349 | 153.7m | 还是偏少 |
| **3** | **0.06** | **?** | **?** | **?** | 🔄 测试中 |

### 核心问题

**特征过度平滑** → VT区分度不足 → VT数量偏少

```
简化的全局-局部交互
     ↓
所有特征都被"平均化"
     ↓
不同场景看起来很相似
     ↓
阈值需要更低才能区分
```

---

## 🔧 优化方案

### 方案A：降低阈值（推荐，已应用）

**文件**：`test_imu_visual_fusion_slam.m`

```matlab
VT_MATCH_THRESHOLD = 0.06  % 从0.08降低
```

**优点**：
- ✅ 简单直接
- ✅ 不改变特征提取逻辑
- ✅ 快速测试

**缺点**：
- ⚠️ 治标不治本
- ⚠️ 可能需要继续调整

**预期结果**：
- VT: 310-340
- 经验节点: 420-460
- RMSE: 135-145m

---

### 方案B：减少全局调制权重

**文件**：`hart_transformer_extractor.m` (第101、105行)

**当前**：
```matlab
dorsal_enhanced = dorsal_features + 0.3 * global_weight * dorsal_normalized;
ventral_enhanced = ventral_features + 0.3 * global_weight * ventral_normalized;
```

**优化**：
```matlab
dorsal_enhanced = dorsal_features + 0.1 * global_weight * dorsal_normalized;
ventral_enhanced = ventral_features + 0.1 * global_weight * ventral_normalized;
```

**改变**：`0.3 → 0.1` (减少全局影响)

**优点**：
- ✅ 增加特征区分度
- ✅ 减少过度平滑
- ✅ 可能阈值用回0.08

**缺点**：
- ⚠️ 失去部分"全局上下文"
- ⚠️ 可能增加噪声敏感度

---

### 方案C：增强局部差异

**文件**：`hart_transformer_extractor.m` (第145行)

**当前**：
```matlab
fused_features = 0.20 * dorsal_features + 
                 0.30 * lstm_h + 
                 0.30 * transformer_features + 
                 0.20 * v1_output;
```

**优化1：增加Dorsal权重**（强调位置差异）
```matlab
fused_features = 0.30 * dorsal_features +    % 20%→30%
                 0.25 * lstm_h + 
                 0.25 * transformer_features + 
                 0.20 * v1_output;
```

**优化2：增加V1权重**（强调基础特征）
```matlab
fused_features = 0.15 * dorsal_features + 
                 0.25 * lstm_h + 
                 0.25 * transformer_features + 
                 0.35 * v1_output;            % 20%→35%
```

**优点**：
- ✅ 强调不同类型的特征
- ✅ 增加整体区分度

**缺点**：
- ⚠️ 需要多次实验找最佳比例

---

### 方案D：回退到简化特征提取

**文件**：`visual_template_neuro_matlab_only.m` (第45行)

**当前**：
```matlab
normVtImg = hart_transformer_extractor(vtResizedImg);
```

**回退**：
```matlab
% 使用论文验证的简化方法
img_gray = vtResizedImg;
img_enhanced = adapthisteq(img_gray, 'ClipLimit', 0.02);
img_smooth = imgaussfilt(img_enhanced, 0.5);
normVtImg = (img_smooth - min(img_smooth(:))) / ...
            (max(img_smooth(:)) - min(img_smooth(:)) + eps);
```

**优点**：
- ✅ 已验证有效（VT=321, RMSE=126m）
- ✅ 计算更快
- ✅ 阈值0.08已优化

**缺点**：
- ⚠️ 失去HART的时序建模
- ⚠️ 失去双流架构
- ⚠️ 论文创新点减少

---

## 📈 推荐路线图

### 第1步：测试方案A（当前）

```matlab
阈值 = 0.06
特征提取 = HART+Transformer
```

**如果结果满意** → 完成 ✅

**如果VT还是<280** → 进入第2步

---

### 第2步：尝试方案B（减少全局调制）

修改 `hart_transformer_extractor.m`:
```matlab
line 101: 0.3 → 0.1
line 105: 0.3 → 0.1
阈值改回: 0.08
```

测试看VT数量

---

### 第3步：尝试方案C（调整融合权重）

测试不同权重组合：
```
测试1: [0.30, 0.25, 0.25, 0.20]  # 强调Dorsal
测试2: [0.15, 0.25, 0.25, 0.35]  # 强调V1
测试3: [0.25, 0.35, 0.20, 0.20]  # 强调LSTM
```

找到VT数量最接近目标的

---

### 第4步：如果都不行，考虑方案D

回退到简化方法：
- 已知有效（VT=321）
- 虽然失去部分创新，但保证性能

---

## 🎯 目标指标

| 指标 | 目标范围 | 可接受范围 |
|------|---------|-----------|
| VT数量 | 280-350 | 250-400 |
| 经验节点 | 400-500 | 350-550 |
| RMSE | 120-140m | 110-160m |

---

## 💡 调优技巧

### 1. 阈值调整规律

```
特征区分度高 → 阈值可以高 (0.10-0.15)
特征区分度中 → 阈值中等 (0.07-0.10)
特征区分度低 → 阈值要低 (0.05-0.07)
```

### 2. VT数量诊断

```
VT < 200  → 阈值太高 或 特征太平滑
VT 250-350 → ✅ 理想范围
VT > 400  → 阈值太低 或 特征太碎片化
```

### 3. RMSE诊断

```
RMSE < 120m  → 极好（可能过拟合）
RMSE 120-140m → ✅ 理想范围
RMSE > 160m  → VT区分度不足或数量不够
```

---

## 🔬 实验记录模板

每次测试后记录：

```markdown
### 测试X：[日期]

配置：
- 阈值: 0.XX
- 全局调制权重: 0.XX
- 融合权重: [XX, XX, XX, XX]
- 其他修改: XXX

结果：
- VT数量: XXX
- 经验节点: XXX
- RMSE: XXX m

评价：
- VT数量: ✅/⚠️/❌
- RMSE: ✅/⚠️/❌
- 下一步: XXX
```

---

## ✅ 快速检查清单

测试前：
- [ ] 清除MATLAB工作区 (`clear all`)
- [ ] 关闭所有图形窗口 (`close all`)
- [ ] 确认配置文件已保存

测试后：
- [ ] 记录VT数量
- [ ] 记录经验节点数
- [ ] 记录RMSE
- [ ] 保存结果图
- [ ] 更新实验记录

---

**当前状态**：✅ 方案A已应用，等待测试结果

**下一步**：
1. 运行测试
2. 检查VT数量是否在280-350
3. 如不满意，参考上述方案继续优化
