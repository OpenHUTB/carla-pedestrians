# 📊 消融实验图表详细解释

## 图表概览

生成3个图表，每个图表都从不同角度展示组件贡献：

1. **图1**: 主图 - RMSE柱状对比图 (ablation_main_figure.png)
2. **图2**: 退化分析图 (ablation_degradation_figure.png)
3. **图3**: 雷达图 (ablation_radar_*.png)

---

## 图1: RMSE柱状对比图（主图）

### 📖 这是什么？

**最重要的消融实验图，适合放在论文正文中。**

直观展示3个配置的绝对RMSE值，用颜色表示性能好坏。

### 🎨 如何阅读？

**X轴**: 3个配置
- Complete System（完整系统）
- w/o Experience Map（无经验地图）
- w/o IMU Fusion（无IMU融合）

**Y轴**: RMSE（米），越低越好

**颜色编码**:
- 🟢 **绿色** = 最优（Complete System）
- 🟠 **橙色** = 中等（去掉经验地图）
- 🔴 **红色** = 最差（只有视觉）

**数值标注**: 柱子上方显示精确RMSE值

### 💡 告诉我们什么？

**Town01 例子：**
```
Complete System:     202.96m  ← 基准（最优）
w/o Experience Map:  262.06m  ← 高29%，说明经验地图很重要
w/o IMU Fusion:      326.19m  ← 高61%，说明IMU更重要
```

**结论：**
1. 柱子越矮越好
2. 绿色柱子应该是最矮的（验证系统正确）
3. 红色柱子最高，说明去掉所有组件后性能最差

**MH_03 例子：**
```
Complete System:  4.280m
w/o ExpMap:       4.305m  ← 只高1%
w/o IMU:          4.528m  ← 高6%
```

**说明：**
- 短距离场景（127m），组件贡献较小
- 但层次关系仍然正确
- 这是正常的！

### 📝 论文中怎么写？

```latex
\begin{figure}
\centering
\includegraphics[width=0.9\linewidth]{ablation_main_figure.png}
\caption{Ablation study results on Town01 and MH\_03 datasets. 
The complete system (green) achieves the lowest RMSE. Removing 
the Experience Map (orange) or IMU fusion (red) progressively 
degrades performance, demonstrating the contribution of each component.}
\label{fig:ablation_main}
\end{figure}

As shown in Figure~\ref{fig:ablation_main}, the complete system 
achieves RMSE of 202.96m on Town01. Removing the Experience Map 
increases error by 29\%, while further removing IMU fusion 
increases it by 61\%, demonstrating the importance of both components.
```

---

## 图2: 性能退化百分比图

### 📖 这是什么？

**量化组件贡献的图表。**

显示去掉每个组件后，性能降低了多少百分比（相对于Complete System）。

### 🎨 如何阅读？

**X轴**: 2个退化配置
- w/o Experience Map（去掉经验地图）
- w/o IMU Fusion（去掉IMU+经验地图）

**Y轴**: Performance Degradation (%)，越高说明该组件越重要

**注意**: 这里**没有**Complete System，因为它是基准（0%退化）

**颜色**:
- 🟠 **橙色**: 去掉经验地图的影响
- 🔴 **红色**: 去掉IMU的额外影响

### 💡 告诉我们什么？

**Town01 例子：**
```
基准: Complete System = 202.96m (0%)

w/o ExpMap:     +29.1%  ← 经验地图贡献29%
w/o IMU:        +60.7%  ← IMU+经验地图总共贡献61%

组件贡献计算:
- Experience Map: 29.1%
- IMU Fusion:     60.7% - 29.1% = 31.6%
```

**说明：**
- IMU融合的贡献（31.6%）比经验地图（29.1%）稍大
- 两个组件都很重要

**MH_03 例子：**
```
w/o ExpMap:  +0.69%   ← 经验地图贡献很小
w/o IMU:     +5.82%   ← IMU贡献5%
```

**说明：**
- 短距离场景，闭环机会少，经验地图作用有限
- IMU仍然有帮助，但不如长距离场景明显

### 📝 论文中怎么写？

```latex
\begin{figure}
\centering
\includegraphics[width=0.9\linewidth]{ablation_degradation_figure.png}
\caption{Performance degradation when removing components. 
On Town01, removing the Experience Map increases RMSE by 29.1\%, 
while further removing IMU increases it by an additional 31.6\%. 
The impact is less pronounced on the short MH\_03 sequence.}
\label{fig:ablation_degradation}
\end{figure}

Figure~\ref{fig:ablation_degradation} quantifies the contribution 
of each component. On the long-range Town01 sequence, both the 
Experience Map (29.1\%) and IMU fusion (31.6\%) make substantial 
contributions to accuracy. On the short MH\_03 sequence, the 
impact is smaller due to limited error accumulation.
```

---

## 图3: 综合性能雷达图

### 📖 这是什么？

**多指标综合对比图。**

同时展示3个性能指标（RMSE、漂移率、终点误差），用雷达图直观对比。

### 🎨 如何阅读？

**3个轴**:
- **RMSE**: 整体轨迹精度
- **Drift Rate**: 漂移率（单位距离的误差累积）
- **End Error**: 终点误差（最后一帧的位置误差）

**3条线**:
- 🟢 **绿线**: Complete System
- 🟠 **橙线**: w/o Experience Map
- 🔴 **红线**: w/o IMU Fusion

**关键规则**: 
- ✅ **外圈 = 好**（距离中心越远越好）
- ❌ **内圈 = 差**（靠近中心表示性能差）

**归一化**: 每个数据集单独归一化到[0,1]，最好的=1.0，最差的=0.0

### 💡 告诉我们什么？

**完美情况（Town01）：**
```
Complete System (绿线):  
  - 应该在最外圈（三个指标都最好）
  - 形成一个大三角形

w/o ExpMap (橙线):
  - 稍微靠内（性能稍差）
  - 三角形略小

w/o IMU (红线):
  - 最靠近中心（性能最差）
  - 三角形最小
```

**三角形大小对比：**
- 大三角形 = 好性能
- 小三角形 = 差性能
- Complete的三角形应该最大

**MH_03 特殊情况：**
```
由于数值非常接近（4.28m vs 4.53m），
归一化后的三角形会比较接近。
这是正常的！因为短距离场景误差累积小。
```

### 📝 论文中怎么写？

```latex
\begin{figure}
\centering
\includegraphics[width=0.7\linewidth]{ablation_radar_Town01.png}
\caption{Multi-metric comparison on Town01 using radar chart. 
The complete system (green) achieves the best performance across 
all metrics (larger triangle = better). Removing components 
progressively shrinks the performance envelope, demonstrating 
their contribution to overall accuracy.}
\label{fig:ablation_radar}
\end{figure}

The radar chart (Figure~\ref{fig:ablation_radar}) shows that 
the complete system consistently outperforms ablated configurations 
across all metrics: RMSE, drift rate, and end error. This 
multi-metric view confirms the complementary benefits of the 
Experience Map and IMU fusion.
```

---

## 🎯 三个图表的关系

### 图1（柱状图）- **主图，必须有**
- ✅ 最直观
- ✅ 显示绝对值
- ✅ 适合快速理解
- 📍 **放在论文正文**

### 图2（退化图）- **补充，强烈推荐**
- ✅ 量化贡献
- ✅ 便于比较组件重要性
- 📍 **放在论文正文或补充材料**

### 图3（雷达图）- **美化，可选**
- ✅ 多指标综合
- ✅ 视觉效果好
- ❌ 不太直观（需要解释）
- 📍 **放在补充材料，或演讲PPT**

---

## 📊 论文使用建议

### 主要论文（Main Paper）

**必须包含：**
- ✅ 图1（柱状图）- Section: Ablation Study

**可选包含：**
- ✅ 图2（退化图）- 如果空间允许

### 补充材料（Supplementary Material）

**建议包含：**
- ✅ 图3（雷达图）
- ✅ 详细的数值表格
- ✅ 更多数据集的结果

### 演讲PPT

**建议使用：**
- ✅ 图1（柱状图）- 最直观
- ✅ 图3（雷达图）- 视觉效果好

**避免使用：**
- ❌ 图2（退化图）- 需要太多解释

---

## ❓ 常见问题

### Q1: 为什么MH_03的柱子看起来"一样高"？

**A**: 因为数值非常接近（4.28m vs 4.31m vs 4.53m），在图表上难以区分。

**解决方法：**
- ✅ 看数值标注（已经显示3位小数）
- ✅ 在论文中强调这是短距离场景特性
- ✅ 重点展示Town01的结果

### Q2: 雷达图为什么Complete在外面？

**A**: 雷达图规则：**外圈=好，内圈=差**

**原因：**
- 归一化分数：1.0=最好，0.0=最差
- Complete System三个指标都最好，所以在最外圈
- 这是**正确的**！

### Q3: 如果读者不懂雷达图怎么办？

**A**: 
- ✅ 在图注中说明 "(Outer = Better)"
- ✅ 在正文中解释一次
- ✅ 或者不用雷达图，只用柱状图

### Q4: 两个数据集应该放在一起还是分开？

**A**: **放在一起对比**（当前做法是正确的）

**原因：**
- ✅ 显示方法的泛化能力
- ✅ 对比长距离vs短距离场景
- ✅ 论文通常要求多数据集验证

### Q5: 为什么不包含EKF？

**A**: **EKF不是你系统的组件！**

**正确做法：**
- ✅ 消融实验：只测试你系统的组件（3个配置）
- ✅ 方法对比：单独比较 Ours vs EKF
- ❌ 错误：把EKF放在消融实验里

---

## 📝 论文写作模板

### Section: Ablation Study

```latex
\subsection{Ablation Study}

We conducted systematic ablation experiments to evaluate the 
contribution of each component in our bio-inspired SLAM system. 
We compared three configurations:

\begin{itemize}
\item \textbf{Complete System}: Visual Templates + IMU + Experience Map
\item \textbf{w/o Experience Map}: Visual Templates + IMU (no loop closure)
\item \textbf{w/o IMU}: Visual Templates only (pure visual odometry)
\end{itemize}

Figure~\ref{fig:ablation_main} shows the RMSE comparison on two 
datasets with different characteristics: Town01 (1802m urban scenario) 
and MH\_03 (127m indoor scenario).

\textbf{Long-range performance (Town01):} The complete system achieves 
RMSE of 202.96m. Removing the Experience Map increases error to 262.06m 
(+29.1\%), demonstrating the effectiveness of loop closure correction. 
Further removing IMU fusion increases error to 326.19m (+60.7\% total), 
showing that IMU significantly improves odometry accuracy.

\textbf{Short-range performance (MH\_03):} On this shorter sequence, 
all configurations perform well (4.28-4.53m RMSE). The component 
contributions are smaller (+0.7\% and +5.8\%) due to limited error 
accumulation over short distances. Nevertheless, the performance 
hierarchy remains consistent, validating our approach.

Figure~\ref{fig:ablation_degradation} quantifies the relative 
contribution of each component. On Town01, IMU fusion (31.6\%) 
and the Experience Map (29.1\%) make comparable contributions, 
both being essential for accurate long-range navigation.

\textbf{Key findings:}
\begin{itemize}
\item Both components are important for long-range accuracy
\item IMU fusion provides immediate odometry improvement
\item Experience Map provides gradual correction via loop closure
\item Component benefits scale with trajectory length
\end{itemize}
```

---

## 🎯 总结

**图1 (柱状图)**:
- 👁️ 看哪个柱子最矮（最优）
- 🎨 看颜色（绿=好，红=差）
- 📊 看数值标注（精确值）

**图2 (退化图)**:
- 📈 看百分比高低（越高说明组件越重要）
- 🔢 计算单个组件贡献
- 🌍 对比不同数据集（长距离vs短距离）

**图3 (雷达图)**:
- 📐 看三角形大小（大=好）
- 🎯 看是否在外圈（外=好）
- 🔄 看三个指标是否一致

**记住：所有图表都应该显示 Complete System 最优！** ✅
