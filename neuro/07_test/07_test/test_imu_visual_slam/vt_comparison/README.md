# 👁️ 视觉模板对比实验

## 对比的VT方法

1. **传统SIFT/ORB**
2. **HART特征**
3. **HART + Transformer (Plan B - 最优)**

## 运行对比

```matlab
compare_vt_methods     % 对比不同VT方法
analyze_hart_features  % 分析HART特征
```

## 评估指标

- VT匹配准确率
- 特征提取时间
- 闭环检测成功率
