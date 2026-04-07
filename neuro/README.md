图 2 互补 IMU - 视觉融合架构
一、图的核心内容
左子图：频率响应曲线
蓝色曲线：视觉系统的低通滤波器幅频响应（低频保留、高频衰减）
橙色曲线：IMU（前庭系统）的高通滤波器幅频响应（高频保留、低频衰减）
标注了截止频率 
ω 
c
​
 =10 rad/s
，以及该点的幅值 
R=0.707
（-3dB 点），是互补滤波器的核心参数。
中子图：时域信号融合效果
黑色虚线：地面真值（GT）速度
蓝色曲线：视觉原始速度（低频稳定、但响应慢）
黄色曲线：IMU 原始速度（高频灵敏、但有漂移）
绿色曲线：融合后的速度（结合两者优势，无漂移、响应快）
标注了融合前后的误差指标（
X 3.52→Y 1.74493
），验证融合效果。
右子图：生物系统类比框图
视觉系统（眼睛）对应低通滤波，前庭系统（耳朵）对应高通滤波
中枢神经系统（CNS，前庭核 + 小脑）完成融合，输出鲁棒的自运动估计
标注了融合公式 
v 
f
 =H_LP⋅v 
vis
 +H_HP⋅v 
imu
 
，以及核心优势（无漂移、快速响应）。
二、对应仓库文件
绘图脚本：neuro/kbs/fig/imu_visual_complementary_fusion.py
生成 PDF：neuro/kbs/fig/imu_visual_complementary_f...pdf
数据来源：CARLA 仿真环境采集的视觉 / IMU 同步传感器数据
三、绘制方法（可直接复现）
工具依赖：scipy.signal（滤波器设计、频率响应计算）、matplotlib + numpy（绘图）、matplotlib.patches（框图绘制）
运行步骤：
执行 python imu_visual_complementary_fusion.py
脚本自动生成频率响应曲线、时域融合曲线、生物类比框图
输出 PDF 文件 imu_visual_complementary_fusion.pdf
图 3 3D 网格细胞网络与 4-DoF 编码
一、图的核心内容
左子图：3D 网格细胞活动分布
展示 
61×61×61
 网格细胞的 3D 分层活动强度，用色阶区分不同深度的活动分布，对应空间位置的分布式编码。
标注了网格细胞的规模（
61×61×61 neurons
）和头方向细胞的关联。
中子图：简单立方 vs FCC 晶格对比
对比简单立方与 FCC（面心立方）两种晶格结构，FCC 为网格细胞的最优空间拓扑。
标注了晶格的坐标变换关系 
m(x=0)−z/2→2/a
。
右子图：4-DoF 位姿编码流程
3D 网格细胞编码 3D 位置，头方向细胞（Hdc）编码 1D 航向，两者拼接为 
[g_xyz;h_ψ]
标注了核心解码公式，最终输出 4-DoF（3D 位置 + 1D 航向）位姿表示。
二、对应仓库文件
绘图脚本：neuro/kbs/fig/3d_grid_cell_fcc_lattice.py
生成 PDF：neuro/kbs/fig/3d_grid_cell_fcc_lattice.pdf
数据来源：模型输出的网格细胞活动场矩阵、FCC 晶格坐标数据
三、绘制方法（可直接复现）
工具依赖：matplotlib.mplot3d（3D 可视化）、numpy（FCC 晶格坐标计算）、matplotlib.patches（框图绘制）
运行步骤：
执行 python 3d_grid_cell_fcc_lattice.py
脚本生成 3D 活动分布、晶格对比、4-DoF 编码流程框图
输出 PDF 文件 3d_grid_cell_fcc_lattice.pdf
图 4 Town01/MH03 代表性性能
一、图的核心内容
Town01 性能指标（左上）
柱状图展示核心指标：RMSE (
145.5 m
)、Drift% (
11.9%
)、RPE (
0.82
)、VT (
125
)、Loops (
47
)
Town01 误差演化（右上）
逐帧 ATE 误差曲线，对比 NeuroLocMap、EKF Fusion、Visual Odometry 三种方法，体现 NeuroLocMap 的低误差优势
MH03 性能指标（左下）
柱状图展示核心指标：RMSE (
3.3 m
)、Drift% (
2.1%
)、RPE (
0.18
)、VT (
171
)、Loops (
8
)
MH03 误差演化（右下）
逐帧 ATE 误差曲线，对比三种方法，验证 NeuroLocMap 在小场景下的鲁棒性
二、对应仓库文件
绘图脚本：neuro/kbs/fig/representative_performance.py
生成 PDF：neuro/kbs/fig/representative_performance....pdf
数据来源：Town01、MH03 数据集的 SLAM 定位结果、逐帧误差数据
三、绘制方法（可直接复现）
工具依赖：matplotlib（绘图）、pandas（整理实验指标、逐帧误差数据）
运行步骤：
执行 python representative_performance.py
脚本读取实验数据，生成指标柱状图、误差演化曲线
输出 PDF 文件 representative_performance.pdf
图 5 消融实验 + 视觉模板增长
一、图的核心内容
左子图：消融实验 RMSE 对比
柱状图对比 5 种模型配置：Full、w/o IMU、w/o Exp Map、w/o Transformer、w/o Dual-stream
标注了各配置的 RMSE 数值（Full:
145.5 m
，w/o IMU:
315.3 m
等），添加误差棒体现结果稳定性
右子图：视觉模板增长曲线
6 个数据集（Town01/Town02/Town10/MH01/MH03/KITTI07）的视觉模板数随帧索引的增长曲线
对比 RatSLAM 基线（≈5 个模板），标注 Town10 的 195 个模板，体现模型的模板积累能力
二、对应仓库文件
绘图脚本：neuro/kbs/fig/ablation_unified.py、neuro/kbs/fig/vt_growth_all_datasets.py
生成 PDF：neuro/kbs/fig/ablation_unified.pdf、neuro/kbs/fig/vt_growth_all_datasets.pdf
数据来源：消融实验结果、各数据集的视觉模板数时序数据
三、绘制方法（可直接复现）
工具依赖：matplotlib + seaborn（绘图）、numpy（整理模板数时序数据、消融实验数据）
运行步骤：
执行 python ablation_unified.py 生成消融实验图
执行 python vt_growth_all_datasets.py 生成模板增长图
输出对应 PDF 文件
图 6 6 数据集性能汇总
一、图的核心内容
(a) RMSE 对比柱状图
6 个数据集（Town01/Town02/Town10/KITTI07/MH01/MH03）的 NeuroLocMap/EKF/VO 三种方法 RMSE 对比
(b) 提升率气泡图
相对 EKF 的提升率，气泡大小与轨迹长度成正比，标注各数据集的提升百分比（如 Town10:
58.5%
）
(c) 成功率饼图
定位成功率
83%
（5/6 数据集成功），
17%
失败（对应 MH01）
二、对应仓库文件
绘图脚本：neuro/kbs/fig/performance_summary.py
生成 PDF：neuro/kbs/fig/performance_summary.pdf
数据来源：6 个数据集的 SLAM 定位结果、RMSE 统计数据
三、绘制方法（可直接复现）
工具依赖：matplotlib（绘图）、pandas（计算提升率、整理汇总数据）
运行步骤：
执行 python performance_summary.py
脚本读取汇总数据，生成 RMSE 对比、提升率气泡图、成功率饼图
输出 PDF 文件 performance_summary.pdf
图 7 KITTI07 输入 - 输出可视化
一、图的核心内容
左上：RGB Images
KITTI07 数据集的 4 个关键帧（Frame77/110/330/551），展示视觉输入
左下：IMU Sensor Data
IMU 加速度（Accel）、角速度（GyroZ）的时序曲线，标注关键帧位置
右上：3D Trajectory Comparison
地面真值（Ground Truth）与 NeuroLocMap 预测轨迹的 3D 对比，标注起点（绿）、终点（红）
右下：Experience Map Topology
经验图拓扑结构：51 个节点、8 个闭环，蓝色实线为顺序链接，绿色虚线为闭环边，标注起点 / 终点
二、对应仓库文件
绘图脚本：neuro/kbs/fig/KITTI_07_input_output_visualization.py
生成 PDF：neuro/kbs/fig/KITTI_07_input_output_visuali...pdf
数据来源：KITTI07 数据集的 RGB 图像、IMU 数据、定位轨迹、经验图拓扑数据
三、绘制方法（可直接复现）
工具依赖：cv2（OpenCV，读取关键帧图像）、matplotlib.mplot3d（3D 轨迹）、networkx（绘制经验图节点、边、闭环）、matplotlib（IMU 时序曲线）
运行步骤：
执行 python KITTI_07_input_output_visualization.py
脚本读取 KITTI07 数据，生成全链路可视化图
输出 PDF 文件 KITTI_07_input_output_visualization.pdf
