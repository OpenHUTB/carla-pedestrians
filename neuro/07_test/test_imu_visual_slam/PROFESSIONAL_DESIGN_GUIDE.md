# 🎨 专业论文图表设计指南

## ✨ 设计理念

**高对比度 + 统一风格 + 专业规范** - 适合学术论文发表的专业可视化！

---

## 🎯 核心改进

### 之前的问题 ❌
- 颜色太淡，对比度不够
- 字体颜色和背景融合
- 配色不统一（粉红/浅蓝混杂）
- 风格不专业，不适合论文

### 现在的专业风格 ✅
- ✅ **高对比度配色** - 深蓝/深红/深绿/深橙
- ✅ **统一边框** - Box on, LineWidth 1.2
- ✅ **加粗字体** - FontWeight bold
- ✅ **专业网格** - GridAlpha 0.25, 中等灰色
- ✅ **纯白背景** - 清晰干净
- ✅ **深色坐标轴** - [0.2 0.2 0.2]

---

## 🎨 统一配色方案

### **专业配色标准**

| 用途 | 颜色 | RGB值 | 可视化 |
|------|------|-------|--------|
| **Ground Truth** | 深灰色 | `[0.20 0.20 0.20]` | ⬛ |
| **Experience Map** | 深红色 | `[0.84 0.15 0.16]` | 🟥 |
| **IMU-Visual Fusion** | 深蓝色 | `[0.12 0.47 0.71]` | 🟦 |
| **Visual Odometry** | 深绿色 | `[0.17 0.63 0.17]` | 🟩 |
| **Statistics/Mean** | 深橙色 | `[0.90 0.40 0.00]` | 🟧 |
| **RMSE/Error** | 深蓝色 | `[0.12 0.47 0.71]` | 🟦 |
| **起点标记** | 深绿色 | `[0.17 0.63 0.17]` | 🟢 ▲ |
| **终点标记** | 深红色 | `[0.84 0.15 0.16]` | 🔴 ▼ |

### **配色使用原则**
- 主要曲线：深蓝色 (#1f77b4)
- 对比曲线：深红色 (#d62728)
- 辅助信息：深绿色 (#2ca02c)  
- 统计柱状：深橙色 (#e67e00)

---

## 📐 统一视觉元素

### **1. 背景设置**
```matlab
% 图形背景 - 极淡灰蓝
set(fig, 'Color', [0.96 0.96 0.98]);

% 子图背景 - 纯白
set(gca, 'Color', 'w');
```

### **2. 网格样式**
```matlab
grid on;
set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
```

### **3. 边框设置**
```matlab
set(gca, 'Box', 'on', 'LineWidth', 1.2);
```

### **4. 坐标轴样式**
```matlab
% 标签
xlabel('...', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
ylabel('...', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
title('...', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

% 坐标轴颜色
ax = gca;
ax.XAxis.Color = [0.2 0.2 0.2];
ax.YAxis.Color = [0.2 0.2 0.2];
ax.LineWidth = 1.2;
```

### **5. 字体规范**
```matlab
set(gca, 'FontSize', 11, 'FontName', 'Arial');
```

### **6. 图例样式**
```matlab
leg = legend('Location', 'best', 'FontSize', 11);
set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
```

### **7. 线宽层次**
```matlab
Ground Truth:      LineWidth = 3.0  % 最粗 - 基准线
Experience Map:    LineWidth = 2.8  % 次粗 - 主要结果
Fusion/Visual Odo: LineWidth = 2.5  % 中等 - 对比方法
辅助线:            LineWidth = 2.2  % 较细 - 参考信息
```

### **8. 标记设计**
```matlab
% 起点 - 向上三角形 + 深绿色 + 黑色边框
scatter(x, y, 140, [0.17 0.63 0.17], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, ...
    'Marker', '^');

% 终点 - 向下三角形 + 深红色 + 黑色边框
scatter(x, y, 140, [0.84 0.15 0.16], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2.5, ...
    'Marker', 'v');
```

---

## 📊 已更新的图表

### ✅ Figure 1: SLAM Accuracy Evaluation
**文件**: `09_vestibular/evaluate_slam_accuracy.m`

**6个子图**:
1. **Trajectory Error Evolution** - 深蓝曲线 + 深橙Mean + 深绿RMSE
2. **Error Distribution (CDF)** - 深蓝渐变填充 + 粗实线
3. **3D Error Decomposition** - 深红X + 深蓝Y + 深绿Z
4. **Segmented Error Analysis** - 深蓝箱线图
5. **Distance-based Error** - 深蓝柱状图 + 深色边框
6. **Spatial Error Heatmap** - 蓝色渐变 + 三角标记

---

### ✅ Figure 2: SLAM Statistics Summary
**文件**: `09_vestibular/evaluate_slam_accuracy.m`

**3个子图**:
1. **ATE Statistical Metrics** - 深蓝柱状图 + 数值标注
2. **Path Length Comparison** - 深绿柱状图 + 误差百分比
3. **Performance Scores** - 深橙柱状图 + 百分比显示

---

### ✅ Figure 3: Trajectory Comparison
**文件**: `plot_imu_visual_comparison_with_gt.m`

**4个视图**:
1. **Top View (XY Plane)** - 四条轨迹清晰对比
2. **Side View (XZ Plane)** - 高度变化可视化
3. **Front View (YZ Plane)** - 横向运动展示
4. **3D Trajectory View** - 立体全景

**配色**:
- Ground Truth: 深灰色 (3.0 pt)
- Experience Map: 深红色 (2.5 pt)
- IMU-Visual Fusion: 深蓝虚线 (2.5 pt)
- Visual Odometry: 深绿点划线 (2.2 pt)

---

### ✅ Figure 4: Real-World SLAM
**文件**: `test_real_carpark_slam.m`

**3个轨迹图 + 1个统计面板**:
1. **Visual Odometry Path** - 深蓝轨迹 + 绿色起点 + 红色终点
2. **Experience Map Path** - 深红轨迹 + 绿色起点 + 红色终点
3. **Real-World Path Comparison** - 蓝/红轨迹叠加对比

---

## 🆚 设计对比

### **旧设计（淡色风格）**
```matlab
% 背景
set(fig, 'Color', [0.98 0.98 1.00]);  % 淡蓝紫

% 配色
天空蓝 = [0.40 0.76 0.95];  % 太淡
粉红色 = [0.95 0.55 0.65];  % 太淡
柠檬绿 = [0.70 0.95 0.50];  % 太淡

% 网格
GridAlpha = 0.15;  % 太淡，几乎看不见

% 边框
Box = 'off';  % 无边框，不够规范

% 字体
FontWeight = 'normal';  % 不够醒目
Color = [0.3 0.3 0.3];  % 与背景对比度低
```

### **新设计（专业风格）**
```matlab
% 背景
set(fig, 'Color', [0.96 0.96 0.98]);  % 极淡灰蓝

% 配色
深蓝色 = [0.12 0.47 0.71];  % 对比度强
深红色 = [0.84 0.15 0.16];  % 对比度强
深绿色 = [0.17 0.63 0.17];  % 对比度强

% 网格
GridAlpha = 0.25;  % 清晰可见

% 边框
Box = 'on';  % 专业边框
LineWidth = 1.2;  % 加粗

% 字体
FontWeight = 'bold';  % 加粗醒目
Color = [0.15 0.15 0.15];  % 高对比度
```

---

## 📏 设计规范速查

### **快速配置模板**
```matlab
% === 创建图形 ===
fig = figure('Position', [50, 50, 1400, 900]);
set(fig, 'Color', [0.96 0.96 0.98]);

% === 子图设置 ===
subplot(2, 3, 1);
hold on;

% 网格和背景
grid on;
set(gca, 'GridLineStyle', '-', 'GridAlpha', 0.25, 'GridColor', [0.75 0.75 0.75]);
set(gca, 'Box', 'on', 'LineWidth', 1.2);
set(gca, 'Color', 'w');

% 绘制曲线
plot(x, y, '-', 'Color', [0.12 0.47 0.71], 'LineWidth', 2.5);

% 坐标轴
xlabel('X Label', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
ylabel('Y Label', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.15]);
title('Title', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

% 图例
leg = legend('Location', 'best', 'FontSize', 11);
set(leg, 'Box', 'on', 'Color', [1 1 1 0.95], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);

% 字体和坐标轴
set(gca, 'FontSize', 11, 'FontName', 'Arial');
ax = gca;
ax.XAxis.Color = [0.2 0.2 0.2];
ax.YAxis.Color = [0.2 0.2 0.2];
ax.LineWidth = 1.2;
```

---

## ✨ 特色设计元素

### **1. 渐变填充 (CDF图)**
```matlab
x_fill = [x_data; x_max; 0];
y_fill = [y_data; 0; 0];
fill(x_fill, y_fill, [0.12 0.47 0.71], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
plot(x_data, y_data, '-', 'Color', [0.12 0.47 0.71], 'LineWidth', 2.8);
```

### **2. 热力图配色**
```matlab
% 高对比度蓝色渐变
colormap([linspace(0.85, 0.12, 256)', ...
          linspace(0.92, 0.47, 256)', ...
          linspace(0.98, 0.71, 256)']);
```

### **3. 柱状图边框**
```matlab
bar(values, 'FaceColor', [0.12 0.47 0.71], ...
    'EdgeColor', [0.08 0.35 0.55], 'LineWidth', 1.2);
```

### **4. 箱线图样式**
```matlab
h = boxplot(data, 'Colors', [0.12 0.47 0.71], 'Symbol', 'o');
set(h, 'LineWidth', 1.8);
```

---

## 🎯 使用效果

### **对比度**
- ✅ 曲线与背景：强对比，清晰可读
- ✅ 文字与背景：高对比，易于识别
- ✅ 不同曲线间：颜色差异明显

### **统一性**
- ✅ 所有图表使用相同配色方案
- ✅ 边框、网格、字体完全统一
- ✅ 标记风格保持一致

### **专业性**
- ✅ 符合学术期刊标准
- ✅ 黑白打印效果良好
- ✅ 投影展示清晰醒目

---

## 🚀 重新生成图表

```matlab
% 关闭旧图窗
close all;

% 重新运行测试
RUN_SLAM_TOWN01  % 或其他测试脚本
```

### ✅ **检查清单**
重新生成后，您应该看到：

- [x] 整体背景：极淡灰蓝色 `[0.96 0.96 0.98]`
- [x] 子图背景：纯白色
- [x] 边框：清晰可见的黑色边框
- [x] 网格：中等对比度，GridAlpha = 0.25
- [x] 曲线颜色：深蓝/深红/深绿/深橙
- [x] 字体：全部加粗，深灰色
- [x] 坐标轴：深灰色 `[0.2 0.2 0.2]`
- [x] 图例：带边框，半透明白色背景
- [x] 标记：绿色▲起点，红色▼终点

---

## 💡 设计原则

### **1. 可读性第一**
- 高对比度保证在各种环境下都清晰可读
- 加粗字体确保关键信息醒目

### **2. 专业规范**
- 统一的配色方案
- 规范的边框和网格
- Arial字体，学术标准

### **3. 信息层次**
- 线宽区分重要性
- 颜色深浅表示主次
- 标记突出关键点

### **4. 印刷友好**
- 高对比度适合黑白打印
- 不依赖颜色传递唯一信息
- 线型辅助区分（实线/虚线/点划线）

---

## 📋 配色速查表

### **完整配色代码**
```matlab
% === 主要配色 ===
深蓝色 = [0.12 0.47 0.71];  % 主曲线、柱状图
深红色 = [0.84 0.15 0.16];  % Experience Map、终点
深绿色 = [0.17 0.63 0.17];  % Visual Odo、起点、RMSE
深橙色 = [0.90 0.40 0.00];  % 统计、Mean线
深灰色 = [0.20 0.20 0.20];  % Ground Truth

% === 辅助配色 ===
柱状图边框 = [0.08 0.35 0.55];  % 深蓝加深
背景浅灰 = [0.96 0.96 0.98];    % 图形背景
网格灰色 = [0.75 0.75 0.75];    % 网格线
文字深灰 = [0.15 0.15 0.15];    % 标签文字
标题黑色 = [0.10 0.10 0.10];    % 标题文字
坐标轴灰 = [0.20 0.20 0.20];    % 坐标轴
```

---

**现在您拥有一套完全统一、高对比度、符合学术标准的专业图表设计！** 🎯📊✨

**重新运行测试脚本即可看到专业效果！**
