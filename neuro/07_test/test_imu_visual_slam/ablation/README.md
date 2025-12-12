# 🔬 消融实验

## 实验目的

验证IMU-Visual融合系统各组件的贡献度

## 运行实验

```matlab
RUN_ABLATION_TOWN01  % Town01完整消融实验
RUN_ABLATION_TOWN10  % Town10完整消融实验
```

## 实验配置

1. **仅视觉里程计**
2. **视觉 + IMU融合**
3. **视觉 + IMU + VT闭环**
4. **完整系统 (视觉 + IMU + VT + 经验地图)**

## 输出

- 性能对比表格
- 精度柱状图
- LaTeX格式表格
