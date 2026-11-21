# A1 视觉里程计：代码阅读笔记（写作原料，非论文文本）

用途：重写方法章 3.3 背侧流（运动估计支路）前，先读懂真实实现。本文只讲代码事实，不提供论文句子。每条都带 file:line，写时可回查。

## 一句话

真实 VO 是 RatSLAM/Milford 式**扫描线强度剖面匹配**：把图像的列（或行）亮度求和压成一维剖面，与上一帧的剖面在整数移位窗口内找最匹配的偏移——偏移换算成转速，匹配处的平均亮度差换算成平速。没有特征提取、没有 K⁻¹、没有光流方程、没有优化器。论文现稿那套"光学流几何推导"（WLS、投影矩阵、逆深度）在代码里完全不存在，这就是 A1 必须整段重写的原因。

## 数据流（visual_odometry_down.m，282 行）

```
rawImg
 ├─ 转速 ROI 子图 → 列求和 → 一维列剖面 ─┐
 ├─ 平速 ROI 子图 → (同上，共用剖面)      ├→ compare_segments(当前剖面, 上一帧剖面, ±26px)
 └─ 高度 ROI 子图 → 行求和 → 一维行剖面 ─┘        ↓
                                        最优整数偏移 + 该偏移处平均亮度差
                                              ↓
                              yawRotV / transV / heightV（限幅后输出）
```

主循环调用点：`07_test/test_imu_visual_slam/core/test_imu_visual_fusion_slam.m:452`（纯视觉 `visual_odometry`）→ `:465`（融合层 `imu_aided_visual_odometry`，属 A2 节）。

## 三路速度的具体机制

**偏航角速度（yaw）**
- 取转速 ROI 子图，`horiDegPerPixel = FOV_HORI_DEGREE / 子图宽度`（L106）——每像素对应多少水平视场角。
- 列亮度求和成 1D 剖面，按均值归一化抗整体亮度变化（L150-152）。
- `compare_segments` 在 ±`ODO_SHIFT_MATCH_HORI`=26 像素窗口内逐个整数移位，与上一帧剖面比对，返回差异最小的偏移（L156；`compare_segments.m`，入参注释即"shift_length 为考虑的像素偏移范围"）。
- 换算：`yawRotV = ODO_YAW_ROT_V_SCALE(=1) × 偏移像素 × 每像素角度`（L159）。
- 限幅：|yawRotV| > 2.5°/帧 时**沿用上一帧值**，否则才更新（L161-165）——这是抗光照突变的关键门控，代码注释明说"防止单位时间过大差异被采用"。

**平移速度（trans）**
- 不用几何三角，直接用**最优偏移处的平均绝对亮度差** × 标定常数：`transV = minDiffIntensityRot × ODO_TRANS_V_SCALE(=24)`（L180）。
- 代码注释原文原理："speeds are estimates based on the rate of image change... obtained from the filtered average absolute intensity difference between consecutive scanline intensity profiles at the best match for rotation"（L174-178）——感知觉速度 × 经验标定常数，这是 RatSLAM 一脉的古典做法。
- 限幅 >0.5 沿用上一帧（L183-187）。

**高度变化速度（height/pitch）**
- 独立 ROI，**行**亮度求和成剖面（L206-208）；先用 yaw 偏移量水平裁剪对齐（L198-202），说明高度估计依赖已估出的偏航。
- 竖直窗口 ±`ODO_SHIFT_MATCH_VERT`=20 像素匹配（L210）。
- 符号规则：偏移 <0 才产出正值高度速度，否则 0 或沿用上一帧（L238-244）；限幅同前（L246-250）。

## compare_segments 本体（compare_segments.m）

对 1D 剖面做滑动匹配：seg1 固定，seg2 在 ±shift_length 内逐像素移位，逐列累计亮度差，返回差最小的偏移与该处的差值。就是**一维互相关式模板匹配**，纯整数运算，无可学习参数。

## 参数真值（两条主线一致）

`test_imu_visual_fusion_slam.m:149-156` 经 `visual_odo_initial` 传入：
- `ODO_TRANS_V_SCALE=24`、`ODO_YAW_ROT_V_SCALE=1`、`ODO_HEIGHT_V_SCALE=20`
- 限幅 `MAX_TRANS_V_THRESHOLD=0.5`、`MAX_YAW_ROT_V_THRESHOLD=2.5`
- 匹配窗 `ODO_SHIFT_MATCH_HORI=26`（像素）、`ODO_SHIFT_MATCH_VERT=20`
- KITTI 线（`P2FT94_KITTI07.m` 头注释）："口径： CARLA core默认参数"——同一套 VO 参数，只覆盖闭环配置。

## 重写时值得如实交代的点（你自己决定怎么措辞）

1. 方法归属：这是 Milford 的 RatSLAM（2008）一脉的 appearance-based scanline VO，本系统将其从 2D 扩到含高度通道的 3D；引用 Ball et al. 2013（文件头即此引用）是现成的。
2. "标定常数 24" 的性质：把感知觉亮度差映射为米制平速的经验标定，随相机/场景调定——诚实写"empirically calibrated"比编一个几何推导立得住，导师复算也对得上代码。
3. 三个限幅门控（0.5/2.5 + 沿用上一帧）是鲁棒性来源，光照突变帧不会被强采——这是真实存在的机制，值得一句。
4. 高度通道依赖 yaw 对齐（先裁剪再匹配）——顺序关系是代码里实打实的。
5. 分辨率与 FOV：换算只用到 FOV 角度/像素宽度，全部输入是灰度图——没有深度、没有相机内参矩阵 K。论文现稿若保留 K 就是假。
