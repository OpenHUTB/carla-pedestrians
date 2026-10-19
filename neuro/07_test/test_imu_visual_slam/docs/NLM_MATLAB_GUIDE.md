# MATLAB 跑 NeuroLocMap（NLM）主系统教程（Windows / Ubuntu 双系统）

从零在本机跑通 NLM 生物启发 SLAM 主系统的完整流程，覆盖 Windows 与 Ubuntu 两种系统。KITTI 序列的细节另见 `KITTI_QUICKSTART.md`，本篇负责：环境准备、目录要求、CARLA 五集驱动、结果核对、双系统差异。

## 一、NLM 主系统的代码结构

```
neuro/
├── 01_conjunctive_pose_cells_network/    # 3D 网格细胞网络 + HDC 朝向细胞
├── 02_multilayered_experience_map/       # 经验地图（节点/边/松弛）
├── 03_visual_odometry/                   # 扫描线剖面匹配 VO
├── 04_visual_template/                   # 视觉模板 + LSTM 时序模块
├── 09_vestibular/                        # IMU-视觉融合层
├── data/01_NeuroSLAM_Datasets/           # 数据集（图像+IMU+GT+结果）
└── 07_test/test_imu_visual_slam/
    ├── core/     # 主驱动 test_imu_visual_fusion_slam.m（被 quickstart 调用）
    └── quickstart/  # 一键入口 P2FT94_*.m（每数据集一个驱动）
```

驱动用 `mfilename('fullpath')` 相对回退定位仓库根目录（core L51-52），**无绝对路径**——仓库放在哪个盘/哪个目录都能跑，但**路径里不要有中文或空格**（Windows 下尤其容易触发 MATLAB 与 Python 脚本的各种坑）。

## 二、环境准备（按系统二选一）

**Windows**

1. 安装 MATLAB R2021b 或更新（本教程验证版本 2021b；需勾选 **Image Processing Toolbox**——VO 的 `imresize` 依赖它）。
2. 确认工具箱：打开 MATLAB 运行 `which imresize`，应返回工具箱路径而非空。
3. 命令行批处理用系统自带 cmd 或 PowerShell（R2019a+ 支持 `matlab -batch`），无需额外配置。

**Ubuntu**

1. 安装 MATLAB（官方安装包或学校镜像；同样勾选 Image Processing Toolbox）。命令行确认 `which matlab` 可用。
2. 若 `matlab` 不在 PATH：`sudo ln -s /opt/MATLAB/R2021b/bin/matlab /usr/local/bin/matlab`（按实际安装路径）。
3. 系统依赖无——纯 MATLAB 计算，不需要 CARLA/Python 环境也能跑 SLAM 本体。

**两系统通用检查**

```matlab
which imresize          % Image Processing Toolbox 在位
which test_imu_visual_fusion_slam   % 路径未添加前应为空（正常）
```

## 三、运行主系统

**1. 数据集在位确认**（复跑不需要 CARLA，数据已在仓库 `neuro/data/01_NeuroSLAM_Datasets/`）：每数据集目录含顶层 PNG 帧 + `aligned_imu.txt` + `ground_truth.txt`，以及输入依赖 `slam_results/pr_embeddings_ft.mat`。

**2. 一键运行**（论文口径已内置：τ=0.94、EKF-odo、S2 关、结果子目录 `slam_results_p2ft_t94`）：

```bash
# Windows (cmd/PowerShell) 与 Ubuntu 相同写法
cd <仓库>/neuro/07_test/test_imu_visual_slam/quickstart
matlab -batch P2FT94_TOWN05      # CARLA Town05（1101 帧级）
matlab -batch P2FT94_TOWN03      # Town03
matlab -batch P2FT94_TOWN07      # Town07
matlab -batch P2FT94_TOWN10HD    # Town10HD
matlab -batch P2FT94_KITTI07     # KITTI07 真实序列
matlab -batch P2FT94_KITTI09     # KITTI09 真实序列
```

GUI 方式：打开 MATLAB → cd 到 `quickstart/` → 在命令行敲驱动名（如 `P2FT94_TOWN05`）回车，可看实时图窗。

**3. 运行时长**：单集约几分钟（帧数 1100–1600），无头模式跑完自动退出。

**4. 输出物**：写回各数据集目录的 `slam_results_p2ft_t94/`（trajectories.mat、performance_report.txt、experiences.mat 等）。**重跑原地覆盖**；要保留先 `cp -r`（Windows 用 `xcopy /E /I`）。

## 四、结果核对（Python 侧，双系统）

指标脚本在 `neuro/kbs/`，Windows 用 `python`、Ubuntu 用 `python3`：

```bash
# 依赖（一次性）: numpy scipy pandas matplotlib opencv-python
cd <仓库>/neuro/kbs
python compute_final_metrics.py    # Windows；Ubuntu 用 python3
python se3_metrics.py              # 双口径 SE(3)/Sim(3)
python check_baseline.py           # 113 项守护，任一劣化 exit 1
```

| 核对目标 | 脚本 | 论文数字 |
|---|---|---|
| Table 1 五主集 | `compute_final_metrics.py` | Town05 41.14/EKF 44.44 … KITTI07 18.45/22.09 |
| S.2 双口径（含 KITTI09） | `se3_metrics.py` | EKF 17.75/44.01，NLM 23.12/40.99 |
| 基线守护 | `check_baseline.py` | 全 PASS 才能合入 |

注意：指标脚本读各数据集的 `slam_results/`（规范目录）；复跑结果在 `slam_results_p2ft_t94/`，要让新数字进口径需按 `KITTI_QUICKSTART.md` 第四节第 0 步替换。

## 五、双系统差异速查

| 事项 | Windows | Ubuntu |
|---|---|---|
| 无头运行 | `matlab -batch 驱动名`（cmd/PowerShell） | `matlab -batch 驱动名` |
| 复制结果目录 | `xcopy /E /I src dst` | `cp -r src dst` |
| Python 命令 | `python` | `python3` |
| pip 依赖 | `pip install -r requirements.txt` | `pip3 install -r requirements.txt` |
| 路径分隔符 | MATLAB 内全用 `fullfile`，无需手写 `\` | 同左（`/`） |
| 控制台中文乱码 | 可能出现（见下 FAQ） | 可能出现（见下 FAQ） |
| 首次 CARLA 采集 | CARLA_0.9.16 exe + venv | CARLA_0.9.16 + venv（另见根 README） |

## 六、常见问题

| 现象 | 原因与处理 |
|---|---|
| `imresize` 未定义 | 没装 Image Processing Toolbox；MATLAB Add-On 安装后重试 |
| MATLAB 输出/注释乱码 | 仓库 .m 文件编码混合（UTF-8 与 GBK）。乱码只影响注释与打印，**不影响数值与文件输出**；可分别用 `slCharacterEncoding('UTF-8')`（Ubuntu）或默认 GBK（中文 Windows）减少乱码 |
| 找不到数据集 | 驱动相对 `neuro/data/01_NeuroSLAM_Datasets/<名>` 定位；必须从 `quickstart/` 启动且数据目录未改名/移动 |
| 报错缺 `pr_embeddings_ft.mat` | 基础目录 `slam_results/` 被动过；从备份或复现资产包恢复 |
| Windows 下 `matlab -batch` 卡住 | 确认没有License弹窗/其他 MATLAB 实例；任务管理器结束残留 MATLAB.exe 后重试 |
| 结果与论文数字不符 | 核对四件：用 `P2FT94_*` 驱动（非 `P2FT_*` 旧版）、`FAST_TEST_MODE=false`、结果目录 `slam_results_p2ft_t94/`、指标脚本读的规范目录是否已同步 |
| 路径含中文/空格报错 | 把仓库移到纯英文无空格路径（如 `D:\repos\carla-pedestrians` 或 `~/openhutb`） |

## 七、与采集的关系（想自己造数据时）

跑论文数据不需要 CARLA。要采集新场景时：启动 CARLA 0.9.16 服务器 → `python main.py`（根目录一键，参数见根 README 的 Available Options 表）→ 产出新数据集目录后，仿照 `P2FT94_TOWN05.m` 复制驱动改 `DATASET_NAME` 与 `FAST_TEST_FRAMES` 即可接入同一评测链。
