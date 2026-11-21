# NeuroLocMap 论文复现资产包（2026-10 初版）

> **状态：已被取代（superseded）。** 本文件描述 2026-10-17 的初版资产包
> （六集完整 `datasets/`，约 1.61 GB，代码快照 `c4d6c77`）。2026-11 起对外发布的是
> 最小资产包 `NLM_Repro_Assets_*`（约 45 MB，含 GT / EKF / VO / 闭环线轨迹 + Town01 折返集），
> 现行口径见 [`../../README.md`](../../README.md) 第 3 节与 [`REPRODUCE.md`](REPRODUCE.md)。
> 下表仅作历史归档，**不要**据此判断当前发布内容。

打包日期：2026-10-17。对应论文：NeuroLocMap（投稿 Knowledge-Based Systems）。
代码快照 commit：`c4d6c77c054882030589589d71edc86a593dde01`（分支 hutb，仓库 OpenHUTB/carla-pedestrians）。

本包收录"跑出论文全部数字"所需的最小资产：六个数据集、两条 KITTI 基线、尺度头权重、指标单源文件、论文图源与对应代码快照。

## 一、内容清单

### datasets/ —— 六个评测数据集（约 1.61 GB）

| 目录 | 数据 | 论文角色 |
|---|---|---|
| `Town05Data_IMU_Fusion/` | CARLA 0.9.16 仿真采集 | 主表五场景之一 |
| `Town03Data_IMU_Fusion/` | 同上 | 主表五场景之一 |
| `Town07Data_IMU_Fusion/` | 同上 | 主表五场景之一（失败案例分析对象） |
| `Town10HDData_IMU_Fusion/` | 同上 | 主表五场景之一（尺度头留出集） |
| `KITTI07Data_IMU_Fusion/` | KITTI odometry 07 真实序列重组 | 主表五场景之一（零样本真实世界） |
| `KITTI09Data_IMU_Fusion/` | KITTI odometry 09（1592 帧） | 补充材料 S.2 / 闭环分析 |

每个数据集含：图像帧（PNG）、IMU 文本、ground truth、以及论文口径的系统输出轨迹——只保留四类结果目录：`slam_results/`（主表闭环线与经典结果）、`slam_results_scale_nlm/`（全系统，消融表）、`slam_results_orbonly/`（ORB-only 消融）、`slam_results_p2ft_t94/`（τ=0.94 规范驱动结果）。其余过程变体（bak/backup/g20/pin 等中间产物）已剔除。

### baselines/ —— 论文基线轨迹（55 MB）

`_p11_backup_20261014/`：论文正文引用的 KITTI07 两条并存基线：
- `KITTI07/fp_old_synthetic.txt` + `imu_old_synthetic.txt` + `gt_old_synthetic.txt`：正文主表用的旧合成基线
- `KITTI07/fp_base1775.txt`：真 VIO EKF 基线（Sim(3) 17.75 m，补充材料 S.2）
- `KITTI07/fp_head030.txt`、`fp_live.txt`：尺度头消融与在线轨迹
- `Town07/fp_head_0.05.txt`、`fp_live_nlm.txt`：Town07 尺度头消融

### models/ —— 训练产物

- `pr_scale_head.pt`：单目尺度头权重（MLP 8→64→64→1，~4.8k 参数；三城镇训练、Town10HD 留出）
- `scale_p1/`：各序列逐帧尺度序列（`scale_series_Town05/03/07/10HD/KITTI07.npz`，用于复算尺度曲线）

### metrics/ —— 指标单一来源

- `final_metrics.json`：五场景全系统/EKF/VO 各口径最终指标（论文表 1 与摘要区间的唯一来源）
- `se3_metrics.json`：SE(3) 全矩阵口径
- `baseline_frozen.json`：113 项冻结基线（`check_baseline.py` 守护用，任一劣化 exit 1）

### figures/ —— 论文图源

`fig/` 全部论文图的 PDF 源与生成产物（框架图、结果曲线、消融图、可视化图等）。

### code_snapshot/ —— 代码快照（39 MB）

`git archive` 导出的完整 `neuro/` 目录与文档，对应 commit `c4d6c77`。关键入口：
- `neuro/07_test/test_imu_visual_slam/core/test_imu_visual_fusion_slam.m`：MATLAB 主驱动（~1216 行）
- `neuro/07_test/test_imu_visual_slam/quickstart/P2FT94_KITTI07.m`：KITTI 闭环线驱动（τ=0.94）
- `neuro/00_collect_data/IMU_Vision_Fusion_EKF.py`：CARLA 采集器 + Python EKF
- `neuro/kbs/compute_final_metrics.py`、`se3_metrics.py`：指标计算（`__file__` 锚定，任意 cwd 可调）
- `neuro/kbs/check_baseline.py`：基线守护

上游公共仓库内容（pedestrians-*、bip 等）不在包内，可从 GitHub 克隆。

## 二、环境

- CARLA 0.9.16（本机原路径 `/home/yangrb/下载/carla/CARLA_0.9.16/`，需另装，不在包内）
- Python 3.12 venv（原机 `/home/yangrb/openhutb/carla/`）
- Depth-Anything-V2 推理用 conda 环境 `tdmpc`，`HF_ENDPOINT=https://hf-mirror.com`
- MATLAB（主驱动与 RatSLAM 式网络层）

## 三、复现路径

1. CARLA 五场景：启动 CARLA 0.9.16 → 运行 `neuro/00_collect_data/IMU_Vision_Fusion_EKF.py` 采集（12 km/h、20 辆自动驾驶车、20 名运动学直驱行人）→ `neuro/07_test/test_imu_visual_slam/quickstart/` 下各场景驱动跑融合定位。
2. KITTI：`neuro/kbs/kitti09_assemble_raw.py` 组装原始序列 → `neuro/kbs/kitti_prepare_fusion.py` 转标准四文件格式 → `P2FT94_KITTI07.m` 跑闭环线。
3. 指标：`python3 neuro/kbs/compute_final_metrics.py` 与 `se3_metrics.py` 从轨迹文件重算两口径指标，应与 `metrics/final_metrics.json`、`se3_metrics.json` 逐位一致。
4. 守护：`python3 neuro/kbs/check_baseline.py` 对 113 项冻结指标全量比对，PASS 方可声明复现成功。

## 四、本包未收录（与论文数字无关）

- EuRoC（MH_01/MH_03）、Town01/02、Town01Turnaround：早期探索数据集，已退出论文口径
- `_kitti09_raw/`、`_kitti09_dl/`：KITTI09 原始下载与中间产物（可由公开源再生）
- 各 `*_backup_2026101*` 目录：历史过程快照
- CARLA 模拟器本体、conda/venv 环境：按第二节路径重装

## 五、完整性校验

解包后核对：datasets 六目录文件数分别为 Town05 517、Town03 517、Town07 517、Town10HD 517、KITTI07 1262、KITTI09 1617（大致量级，图像帧数以各目录实际为准）。
归档整体 SHA256 见随包同名 `.sha256` 文件。
