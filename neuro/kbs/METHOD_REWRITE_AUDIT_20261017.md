# 方法章公式审计与重写工作清单

日期：2026-10-17。背景：导师意见（issue #75 及当面反馈）——方法部分此前由大模型生成，需要作者自己理解后重写；大模型生成的任何内容都不能使用。

本文档是重写前的核查底稿：方法章 53 个公式环境已逐一与仓库代码对照（两位审计员分组核对，关键结论经二次抽验，全部给出代码出处）。按导师要求，正文重写由作者本人完成；本文档只提供"公式 ↔ 代码"的对照事实与修改方向。

审计代码基准：`07_test/test_imu_visual_slam/core/`（部署驱动）、`00_collect_data/IMU_Vision_Fusion_EKF.py`（采集与 EKF）、`04_visual_template/`、`03_visual_odometry/`、`01_conjunctive_pose_cells_network/`、`02_multilayered_experience_map/`、`09_vestibular/`。

## A. 编造内容（代码中不存在，必须删除或整体替换）

1. **光学流几何推导整段**（式：K⁻¹ 反投影 q̃、f_i=q^t−q^{t-1}、WLS 加权 r_vis、A_i 投影矩阵、λ_i 逆深度、v_vis=κ·v̄）。实际视觉里程计是 RatSLAM/Milford 式**扫描线强度剖面匹配**：`03_visual_odometry/visual_odometry_down.m:106,156-180`（列/行强度求和 → compare_segments 找最优偏移 → yawRotV=ODO_YAW_ROT_V_SCALE·minOffsetYawRot·horiDegPerPixel，transV=minDiffIntensityRot·ODO_TRANS_V_SCALE，超阈沿用上一帧）。无特征跟踪、无 K⁻¹、无 WLS、无 A_i、无 λ_i。
2. **IIR 低通/高通互补滤波全段**（L_t=αL+(1−α)x、H=x−L、H_LP(s)=1/(1+τs) Laplace 分析、"低通给视觉/高通给IMU"）。实际融合是**固定权重凸组合 + 活动门控**：`09_vestibular/imu_aided_visual_odometry.m:80-101`（yawRotV=0.7·imu+0.3·vis 当视觉旋转>0.1°，否则纯 IMU；transV=0.3·imu+0.7·vis 当视觉>0.01；高度 0.5/0.5；随后限幅）。代码中无任何 IIR 状态递推。
3. **自适应权重与尺度更新式**（η=σ_vis/(σ_vis+σ_imu)、κ_{t+1}=κ_t(1+β·…)）。代码中权重是常数；真实尺度校正在 Python EKF 的第 10 维 log 尺度状态与 VelocityScaleModel s(v)=exp(a+b·v)（`IMU_Vision_Fusion_EKF.py:1430,1490`），非该形式。
4. **Hebbian 关联矩阵 Ξ 及矫正注入**（Ξ^{t+1}=max(η·VT·A, Ξ)、ΔA+=ρ/|S|·ΣVT·Ξ）。实际视觉锚定是"VT→网格位置记忆 + 随 decay 衰减的能量注入"：`01_conjunctive_pose_cells_network/3d_grid_cells_network/gc_iteration.m:87-99`（单细胞注入 energy=GC_VT_INJECT_ENERGY·(30−exp(1.2·decay))/30）。
5. **FCC 晶格 / 12 最近邻**。实际网络是 36³ 稠密高斯权重张量（RatSLAM 式 wrap 索引）：驱动 L199-201（GC_*_DIM=36）、`gc_iteration.m:109-115`。附图 `fig/3d_grid_cell_fcc_lattice.pdf` 同样与代码不符，需重画或改叙述。
6. **全网质心位姿读出**（population vector Σ 全网络）。实际读出是**峰值活动包 ±4 窗口 + 圆均值 atan2**（网格索引，非直接米制坐标）：`01_conjunctive_pose_cells_network/3d_grid_cells_network/get_gc_xyz.m:63-95`。
7. **τ_loop=0.05、T_min=50 帧、d_thresh=0.5 m**。无出处。经典闭环实际门：delta_em<20 + 多候选 ratio≥0.7 + CE≤20 + 间隔 300 帧（驱动 L96-99、L410-412）。
8. **五步地图松弛的环路残差/均摊**（e_loop=Σ T−T_loop、按 1/|P| 均摊、Step5 重锚 bump）。实际是**逐边迭代松弛**：EXP_LOOPS=8 轮、每边对 e0/e1 反向各修 EXP_CORRECTION=0.4× 残差（`02_multilayered_experience_map/exp_map_iteration.m:444-495`）；EKF-odo 模式下该松弛整体禁用（驱动 L122-123）。
9. **全局抑制 λ·Ā（λ=1.0）式**。实际是绝对阈值：`gc_iteration.m:144`（≥GC_GLOBAL_INHIB=0.0002 才保留并减阈值），驱动 L214。
10. **路径积分 floor(γvΔt) 整数移位**。实际是**分数权重混合 circshift + 90° 扇区旋转 + 残差角双线性**：`gc_iteration.m:155-192`（z 轴 floor/ceil 混合 L197-204）。

## B. 需改写（概念存在，公式或常数与代码不符）

1. 视觉模板描述子维度：论文 4096（64×64）→ **实际 12×16=192 维**（驱动 L164-167 VT_IMG_RESIZE 16×12）。
2. 网格细胞网络 61³ → **36³**；HDC 360×5 → **36×36**（驱动 L181-182, L199-201）。
3. 高斯核宽 σ=5 → **1.5**（驱动 L208-210 GC_EXCIT_*_VAR；另有抑制核 var=2 论文未写）。
4. LSTM 固定门 0.55/0.60/0.92 → **部署版 0.5/0.5/0.9**（`04_visual_template/hart_transformer_extractor.m:134-136`）；实验章"Implementation Details"里的同组数值需同步。机制（固定门 LSTM）真实存在，保留但数值以部署版为准。
5. 腹/背侧特征式的第二项 "Z" → 实际是 **V1 Gabor 响应 v1_output**（`hart_transformer_extractor.m:74-81`；0.6/0.4 与 0.4/0.3/0.3 权重本身正确）。
6. 经验创建判据（d_cos>0.08 或 距离>0.5m）→ 实际是 **GC/HDC 索引空间欧氏距离 delta_em>20**（`exp_map_iteration.m:159,173`；0.08 只属于 VT 图像模板层，论文混了两级）。
7. 经验边表示 (Δx,Δy,Δz,Δψ) → 实际是**极坐标相对量**（d_xy、d_z、相对当前节点朝向的有符号极角 heading/facing，`create_new_exp.m:44-51`）；EKF-odo 模式下新节点坐标为 EKF 锚点钉扎（L77-83）。
8. τ_VT=0.08 是默认值，实验按集覆盖 0.02–0.12（驱动 L96；P2S/P2FT 驱动覆盖）——需注明。
9. "不做在线零偏估计" 的限定只对 MATLAB 前端成立；Python EKF 有在线陀螺零偏估计（`IMU_Vision_Fusion_EKF.py:1645-1654`）——需按侧区分。
10. **叙事级矛盾（最重要）**：框架图 caption、引言、方法章多处宣称"frequency-domain complementary filtering"，而实际实现是固定权重+活动门控（见 A2）。三处必须统一改为与实现一致的表述。

## C. 与代码一致、重写时可保留的数学事实

灰度系数 0.299/0.587/0.114（rgb2gray，ITU-R 601）；min-max 归一化（`hart_transformer_extractor.m:167-168`）；余弦距离模板匹配；gyro z 轴角速度；v_trans=max(0,(‖a‖−g)Δt·0.5)、v_z=(a_z−g)Δt·0.3（`imu_aided_visual_odometry.m:68,71`，含 k 值）；可分 3D 高斯激励核（σ=1.5）；兴奋卷积 + 模 N 环绕；L1 归一化（`gc_iteration.m:148-149`）；HDC 吸引子动力学形态；部署闭环四门全部常数（τ=0.94、streak≥15±10、朝向 60°、单步 25 m、累计 35 m、arc 50 m、win=250、s∈[0.05,20]、非链式，驱动 L379-416, L1181-1206）；漂移场三件套（raised-cosine ramp L781-788、自适应 W L774-775、末端截断 L777-778、幅度门 L770-772）。尺度支路一节（3.6）本轮分组审计未覆盖，系 2026-10-16 按 `pr_scale_head.py` 与 EKF 实现逐条写入，重写时按同法复核即可。

## D. 形式问题（公式之外，全文）

1. LLM 式断词连字符 66 处（`Ves\-tib\-u\-lar`、`Com\-ple\-men\-tary`、`al\-go\-rith\-mic`、`per\-for\-mance` 等）——全部去除。
2. 方法章 em-dash 19 处——重写时按个人文风处理。
3. 套话词 1 处——重写时自然消除。
4. 框架图 caption 的"频域互补滤波"随 B10 一并更正。

## E. 重写纪律与验证

- 正文由作者本人重写；重写前先读对应代码（上文 file:line），用自己的语言描述，不用任何生成文本。
- 建议顺序：A1/A2（视觉里程计与融合两节伤最重）→ A4-A10（空间细胞各节）→ B（常数与维度）→ D（形式清理）。
- 每轮改完：`pdflatex`/`xelatex` 双语编译 + `python3 check_baseline.py`（性能不弱化门禁，113 项指标冻结比对）。
- 工作量估计：视觉里程计与融合两节整段重写 + 空间细胞各节改写，约 3–5 天；形式清理半天。
