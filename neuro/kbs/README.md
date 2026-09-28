# NeuroSLAM_KBS_v2 — 修订版论文（对照用）

本文件夹是 `openhutb/neuro/kbs/` 原论文的**独立修订副本**，原论文文件**一字未动**。
用途：EKF 基线程序（`neuro/00_collect_data/IMU_Vision_Fusion_EKF.py`）已升级为最终实现后，
论文中基线描述与结果数字需要随之更新。本版把**可确定的修改直接改好**（红色标注），
把**依赖新数据才能定的数字**标红为"待重算"，方便你与旧版逐处对照。

## 一、文件清单（与旧版一致，格式不变）

| 文件 | 说明 |
|---|---|
| `NeuroSLAM_KBS.tex` | 主文件（摘要/贡献/结论处有红色待重算标记） |
| `sec_related_work.tex` | 相关工作，**未改动** |
| `sec_method_expanded.tex` | 方法（NeuroLocMap 主体，与 EKF 程序无关，**未改动内容**；仅 1 处图文件名大小写修正） |
| `experiment_section.tex` | 实验，**主要修改处**（EKF 基线描述重写 + 数字待重算标记） |
| `NeuroSLAM_KBS_all.bib` / `IEEEtran.bst` / `IEEEtran.cls` | 参考文献与格式文件，未改动 |
| `fig/` | 全部 16 张图（与旧版相同） |
| `NeuroSLAM_KBS.pdf` | 本地 tectonic(XeLaTeX) 编译产物，**红色文字即修订点**，可逐处对照 |

## 二、本版已做的修改（共 4 类，PDF 中全部红色显示）

1. **EKF 基线描述重写**（`experiment_section.tex` Baseline Methods 段，R2 版对齐 10 状态最终代码）
   原文 "tightly coupled EKF … representative of MSCKF/VINS-Mono" 与最终代码不符。
   改为与代码一致的描述：**10 状态**松耦合 EKF（第 10 维 log(VO尺度) 在线收敛），
   IMU 预测（陀螺姿态积分 + 静止启动零偏估计 + 零偏补偿航位推算）+ 四个门控观测分支：
   (i) 1D 水平速度幅值观测（VO 帧间位移×在线尺度，真实间隔 IMU 步数×dt，失配时速度重初始化）、
   (ii) roll/pitch 姿态观测、(iii) 短窗口相对位置观测（每 30 帧用当前滤波位置重锚，
   陀螺航向方向×VO 幅值×在线尺度，隔离 VO 全局航向漂移）、
   (iv) 独立尺度观测（转弯运动学 |a_lat|/|ω| ÷ VO 记录速度，窗中值）。
   逐分支卡方门控（0.99–0.999）、残差自适应 R（滑动窗口平滑 + 内点数质量因子，
   clip [0.008, 10]）、Joseph form 协方差更新 + 正则化（对称化 + 最小特征值钳制）。
2. **Fairness 段补充**（红色）：声明 CARLA 数字对应最终版 EKF 实现，
   采集参数 12 km/h、安全距离 6 m；需以该版本重采方可复现。
3. **3 处图引用兼容修正**（texpage/XeLaTeX 可直接编译，图内容不变）：
   - `fig/ablation_unified.eps` → `.pdf`
   - `fig/performance_summary.eps` → `.pdf`
   - `fig/3D_grid_cell_fcc_lattice.pdf` → `3d_...`（与磁盘文件名大小写一致）
4. **待重算标记**（红色方括号 [V2 ...]）：
   - 摘要 / 贡献点 4 / 结论的 **38.1%**
   - Table 1（CARLA 行长度/帧数）、Table 2（CARLA 三行 RMSE/Drift/Improvement、Success Rate）
   - 正文引用 EKF 数字的所有段落（Performance by Scenario / Representative / Discussion / Failure Case / KITTI 可视化）
   - 4 张含 EKF 曲线的图注（需重采后重新生成）

## 三、texpage 编译步骤

1. 把**整个 `NeuroSLAM_KBS_v2` 文件夹**上传到 texpage（含 `fig/`、`.cls`、`.bst`、`.bib`）。
2. 编译入口：`NeuroSLAM_KBS.tex`，引擎选 **XeLaTeX**（或 pdflatex 亦可，.cls 已随附）。
3. 参考文献：texpage 会自动跑 bibtex（已配 `\bibliographystyle{IEEEtran}`）。
4. 已用 tectonic(XeLaTeX) 本地验证：**0 error、0 未定义引用**，PDF 双栏 IEEE 格式与旧版一致。

## 四、待办数据清单（红色"待重算"项的解锁条件）

按依赖顺序：

| # | 待办 | 解锁哪些红色项 |
|---|---|---|
| 1 | 用最终版 EKF（12 km/h 采集参数）重采 **Town01 / Town02 / Town10** 的 ground_truth / fusion_pose / visual_odometry / aligned_imu | Table 1 CARLA 行、Table 2 CARLA 三行、正文所有 EKF 数字 |
| 2 | MATLAB 侧（`neuro/07_test/test_imu_visual_slam`）在**同批新数据**上重跑 NeuroLocMap + 消融 | NeuroLocMap 列、Improvement 列、Success Rate |
| 3 | KITTI 07 / EuRoC MH_01 / MH_03 在新代码版本上重跑确认（固定数据集，预期基本不变） | KITTI/EuRoC 行的确认 |
| 4 | 重新生成 4 张含 EKF 曲线的图（representative_performance / performance_summary / ablation_unified(b) / KITTI 可视化） | 4 张图 + 图注 |
| 5 | 汇总新均值替换 38.1%（摘要/贡献/结论 3 处） | 摘要、贡献点 4、结论 |

> 风险提醒：新 EKF 显著强于旧版（本仓库实测 Town01 融合 ATE≈67 m，旧版基线 253.6 m），
> 若 NeuroLocMap 不在同批新数据上重跑，可能出现"NeuroLocMap 差于新 EKF"的结论反转，
> 因此**必须先完成 #1、#2 再定稿数字**，不能只替换 EKF 列。

## 五、与旧版对照方式

- 旧版：`openhutb/neuro/kbs/`
- 新版：`~/NeuroSLAM_KBS_v2/`
- 快速 diff：`diff -ru /home/yangrb/openhutb/neuro/kbs /home/yangrb/NeuroSLAM_KBS_v2 --exclude=fig --exclude=*.pdf`
- PDF 中所有红色文字 = 本版相对旧版的修订/待办点；红色方括号 `[V2 ...]` = 待重算说明（定稿时删除）。
