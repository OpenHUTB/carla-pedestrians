# KITTI 跑图教程（KITTI07 / KITTI09，τ=0.94 论文口径）

复跑论文两个 KITTI 真实车载序列的完整流程。KITTI 数据不走 CARLA 采集链，全部计算在 MATLAB 内完成（Ubuntu 与 Windows 双系统验证，MATLAB 2021b）。命令默认 Ubuntu 写法，Windows 差异（`python`/`python3`、`xcopy`/`cp`）集中见 `NLM_MATLAB_GUIDE.md` 第五节速查表与本篇标注。

| 序列 | 帧数 | 论文位置 | 驱动 | 论文数字 |
|---|---|---|---|---|
| KITTI07 | 1101 | 主表 Table 1（现实验证行） | `P2FT94_KITTI07.m` | NLM 18.45 m / EKF 22.09 m |
| KITTI09 | 1591 | 补充材料 S.2（零样本迁移） | `P2FT94_KITTI09.m` | EKF 17.75/44.01 m，NLM 闭环 23.12/40.99 m（Sim(3)/SE(3)） |

**数据集背景**：KITTI Raw Dataset，真实德国城市道路（改装 VW Passat B6 采集），RGB 单目相机 + GPS/INS，10 Hz 同步。本系统使用 `image_02`（左侧彩色相机转灰度，与 CARLA 数据一致）和 OXTS 惯导的加速度/角速度。

---

## 一、前置条件检查

**1. MATLAB 可用**

```bash
which matlab
# 预期: .../matlab_2021b/bin/matlab
```

Windows 下无需 `which`：在 cmd/PowerShell 确认 `matlab` 命令可用（R2019a+ 支持 `-batch`），MATLAB 内 `which imresize` 有返回（Image Processing Toolbox 在位）即可。

**2. 数据集在位**（图像帧在数据目录顶层，无 images/ 子目录）

```bash
cd neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion
ls *.png | wc -l                                   # 预期 1101
ls aligned_imu.txt ground_truth.txt

cd ../KITTI09Data_IMU_Fusion
ls *.png | wc -l                                   # 预期 1591
ls aligned_imu.txt ground_truth.txt
```

**3. PR 嵌入文件在位（易踩坑）**

```bash
ls KITTI07Data_IMU_Fusion/slam_results/pr_embeddings_ft.mat
ls KITTI09Data_IMU_Fusion/slam_results/pr_embeddings_ft.mat
```

（以下相对路径均以 `neuro/data/01_NeuroSLAM_Datasets/` 为基。）驱动从各数据集的**基础结果目录 `slam_results/`** 读取该嵌入文件作为闭环输入（core 驱动 L426），而本次输出写到 `slam_results_p2ft_t94/`。**`slam_results/` 是输入依赖，任何时候不要删除或改名。**

---

## 二、运行

无头模式（推荐，与论文批次一致，跑完自动退出）：

```bash
cd neuro/07_test/test_imu_visual_slam/quickstart
matlab -batch P2FT94_KITTI07     # 跑 07，约数分钟
matlab -batch P2FT94_KITTI09     # 跑 09，1591 帧稍慢
```

GUI 模式（想看图时）：打开 MATLAB，cd 到 `quickstart/`，运行对应驱动。

关于驱动脚本的四点说明：

1. **必须用 `P2FT94_*` 系列驱动**。同目录的 `P2FT_KITTI07.m` 是 τ 收紧前的旧口径（0.907–0.935 区间会放进假闭环），结果不可混用。
2. **不要手动把 07 驱动里的 `DATASET_NAME` 改成 09 来跑**——两序列帧数不同（1101 vs 1591），手改容易漏 `FAST_TEST_FRAMES` 等全局量；09 有现成驱动。
3. 驱动头注释里写的 `matlab -batch P2FT_KITTI07` 是旧名，以实际文件名为准。
4. 口径已内置且两序列完全一致（09 为零样本迁移，不动任何参数）：τ=0.94、`NLM_USE_EKF_ODO=true`、Sim(3) 重锚关闭、嵌入文件 `pr_embeddings_ft.mat`、结果子目录 `slam_results_p2ft_t94`。

---

## 三、输出物

结果写入各自数据集目录下的 `slam_results_p2ft_t94/`：

| 文件 | 内容 |
|---|---|
| `trajectories.mat` | 各轨迹（GT / EKF / 纯视觉 / NLM），指标脚本的输入 |
| `performance_report.txt` | 轨迹长度与漂移百分比摘要（人读） |
| `experiences.mat` | 经验地图节点与边 |
| `loop_constraint_events.mat` | 闭环事件记录 |
| `match_log.mat` | 逐帧匹配日志 |

重跑会**原地覆盖**该目录（07 与 09 在各自数据集目录下，互不覆盖）。论文定稿数字如需保留，先备份：

```bash
cd neuro/data/01_NeuroSLAM_Datasets
cp -r KITTI07Data_IMU_Fusion/slam_results_p2ft_t94 KITTI07Data_IMU_Fusion/slam_results_p2ft_t94_paper
cp -r KITTI09Data_IMU_Fusion/slam_results_p2ft_t94 KITTI09Data_IMU_Fusion/slam_results_p2ft_t94_paper
```

---

## 四、结果核对

**0. 关键前提：规范目录 vs 复跑目录。** 指标脚本（`compute_final_metrics.py`、`se3_metrics.py`）读取的 NLM 闭环线是各数据集下的 **`slam_results/trajectories.mat`（规范目录，论文数字出自这里）**，不是复跑输出目录 `slam_results_p2ft_t94/`。复跑后若要让新结果进入指标口径：

```bash
cd neuro/data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion
cp slam_results/trajectories.mat slam_results/trajectories.mat.bak      # 先备份规范文件
cp slam_results_p2ft_t94/trajectories.mat slam_results/trajectories.mat # 再替换
```

只验证复现、不动规范目录时，直接对比两目录的 `trajectories.mat` 即可。替换后必跑 `check_baseline.py`（见第 4 步）。

**1. 快看摘要**——`performance_report.txt` 的"轨迹长度"一节。07 参考量级（论文批次实测）：GT 694.70 m，EKF 前端 434.53 m（漂移 37.45%），NLM 经验地图 486.59 m（漂移 29.96%）。

**2. KITTI07 对主表**：

```bash
cd neuro/kbs
python3 compute_final_metrics.py
```

打印 Table 1（含 KITTI07 行 NLM/EKF/VO 三列）并写入 `final_metrics.json`。07 行论文数字：**NLM 18.45 m / EKF 22.09 m**。

**3. KITTI09 对补充材料 S.2**：

```bash
cd neuro/kbs
python3 se3_metrics.py
```

打印六集 SE(3)/Sim(3) 双口径摘要（含 KITTI09 行）并写入 `se3_metrics.json`。09 的 S.2 论文数字：EKF（真 VIO）**17.75 m（Sim(3)）/ 44.01 m（SE(3)）**，NLM 闭环线 **23.12 m / 40.99 m**。

S.2 读法：Sim(3) 口径下 NLM 闭环线（23.12 m）不敌真 VIO EKF（17.75 m）是**原理性的**——Sim(3) 对齐会把该序列的主导误差（全局尺度约 1.8）重新拟合吸收掉；SE(3) 刚性口径下 NLM 40.99 m 优于 EKF 44.01 m（**−6.8%**），这才是论文的度量主张口径。09 不进 Table 1（五主集为 Town05/03/07/10HD + KITTI07），`compute_final_metrics.py` 输出没有 09 行是正常的。

**4. 过基线守护**（替换规范目录或改过任何代码后必跑）：

```bash
cd neuro/kbs
python3 check_baseline.py    # 113 项冻结指标任一劣化则 exit 1
```

---

## 五、首次组装（历史参考，复跑跳过）

数据已按 NeuroSLAM 格式组装完毕，仅在新机器/新序列时需要重走本节。

1. **下载 raw 序列**：在 KITTI 官网注册后从 http://www.cvlibs.net/datasets/kitti/raw_data.php 下载（每个序列 1–3 GB；07 对应 `2011_09_26_drive_0007`，09 对应 `2011_09_26_drive_0009`）。
2. **组装脚本位置**：
   - KITTI07：`neuro/data/kitti07_assemble.py`
   - KITTI09：`neuro/kbs/kitti09_assemble_raw.py` + `neuro/kbs/kitti_prepare_fusion.py`

组装内容：图像 PNG 化转灰度、OXTS 惯导提取为 `aligned_imu.txt`（加速度 + 角速度，10 Hz 同步）、GPS/INS 作为 `ground_truth.txt`。

---

## 六、常见问题

| 现象 | 原因与处理 |
|---|---|
| 找不到数据集目录 | 驱动按 `neuro/data/01_NeuroSLAM_Datasets/<DATASET_NAME>` 定位（L231），从 `quickstart/` 目录启动即可；不要移动数据目录 |
| 报错缺 `pr_embeddings_ft.mat` | 基础目录 `slam_results/` 被动过；从备份或复现资产包恢复该文件 |
| 复跑后指标数字没变 | 指标脚本读的是 `slam_results/` 规范目录；按第四节第 0 步替换 `trajectories.mat` 后重算 |
| 09 的 Sim(3) 口径 NLM 输给 EKF | 原理性现象，不是 bug；看 SE(3) 刚性口径（−6.8%），读法见第四节第 3 步 |
| 无头跑出现 figure 告警 | 正常现象，仅绘图被跳过，数值与文件输出不受影响 |
| 指标与论文对不上 | 确认用的是 `P2FT94_*` 驱动、结果目录是 `slam_results_p2ft_t94/`；再不对则查 `final_metrics.json` / `se3_metrics.json` 与 `check_baseline.py` 定位漂移项 |
| MATLAB 卡住不动 | `-batch` 模式下不要关终端；确认没有另一个 MATLAB 进程占用同一结果目录 |
| Windows 下 `python3` 报"不是内部或外部命令" | Windows 用 `python`；`cp -r` 换 `xcopy /E /I`，其余命令写法一致 |
| 首次下载 raw 失败 | KITTI 官网需注册登录；wget 失败时手动浏览器下载后解压 |

**IMU 数据格式（OXTS）**：位置 GPS (lat/lon/alt)、姿态 Roll/Pitch/Yaw、速度 North/East/Forward、加速度 ax/ay/az、角速度 wx/wy/wz。系统提取加速度与角速度用于惯性融合。

**为什么用 image_02**：`image_00/01` 为左右灰度立体，`image_02/03` 为左右彩色立体；用 `image_02`（左侧彩色）转灰度，与 CARLA 数据管道保持一致。

**KITTI vs CARLA**：KITTI 提供真实传感器噪声与真实场景挑战性（光照、天气），用于真实场景验证；CARLA 提供完美真值与可控条件，用于算法开发与消融。

---

## 七、一页速查

```bash
# 跑（quickstart 目录下二选一或都跑）
cd neuro/07_test/test_imu_visual_slam/quickstart
matlab -batch P2FT94_KITTI07
matlab -batch P2FT94_KITTI09

# 核（kbs 目录下）
cd neuro/kbs
python3 compute_final_metrics.py   # 07 → Table 1 (18.45 / 22.09)
python3 se3_metrics.py             # 09 → S.2 双口径 (17.75/44.01 vs 23.12/40.99)
python3 check_baseline.py          # 替换规范目录后必跑
```

## 参考资料

- KITTI 数据集主页: http://www.cvlibs.net/datasets/kitti/
- Raw Data: http://www.cvlibs.net/datasets/kitti/raw_data.php
- 论文: "Vision meets Robotics: The KITTI Dataset" (IJRR 2013)
