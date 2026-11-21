# MATLAB 重跑手册（C 类定稿 · 本机无头执行 · 内部）

> **状态：内部机器手册，非发布文档。** 命令中的路径与 `~/matlab_batch.sh` 均为作者本机环境，
> 外部读者请用 [`REPRODUCE.md`](REPRODUCE.md)；本手册描述的是定稿前的重跑流程，
> 其中的数字清单不能作为论文数值来源。

> 适用：MATLAB 重装完成后，在本机一次性跑完 论文定稿所需的全部重跑，
> 并拿到 Table 1/2 的新数字。全部命令走 `~/matlab_batch.sh` 无头包装
> （默认工作目录 = `neuro/07_test/test_imu_visual_slam/quickstart`）。
> 状态核对（2026-10-01）：新生成器的 MH_01 EKF 文件已就位
> （`neuro/data/02_EuRoc_Dataset/MH_01_easy/`）；CARLA 四场景 NLM 数据目录已存在；
> **MH_03 数据缺失（machine_hall.zip 不完整），MH_03 需先解决数据**。

## 0. 冒烟测试（装好后第一件事，2 分钟）

```bash
~/matlab_batch.sh -e "disp(['OK ' version])"
```

出现 `OK 9.x.x` 即通过。若报 license/Settings 类错误，先别往下跑。

## 1. EuRoC 确认（论文口径 'simple' 对齐，test_euroc_fusion_slam.m）

```bash
# MH_01（新生成器 fusion_pose_vo_ekf.txt 已在位，3682 帧）
nohup ~/matlab_batch.sh RUN_SLAM_MH01 > /tmp/euroc_mh01.log 2>&1 &

# MH_03 —— 仅当数据就位后（neuro/data/02_EuRoc_Dataset/MH_03_medium/）
nohup ~/matlab_batch.sh RUN_SLAM_MH03 > /tmp/euroc_mh03.log 2>&1 &
```

- 结果：控制台 + `02_EuRoc_Dataset/<seq>/slam_results/euroc_trajectories.mat`
- **要拿的数字**：日志中 4 条轨迹（Pure VO / IMU-aided / EKF Fusion / Ours）
  的 RMSE（simple 对齐口径）——对应 Table 2 的 EuRoC 两行。
- ⚠️ 注意 MH_01 数据版本口径（本地 3682 帧/184s vs 官方 759 帧/38s），
  跑完先确认论文 Table 2 原数字是哪种数据，不一致则向老师确认口径。

## 2. KITTI07 确认

```bash
nohup ~/matlab_batch.sh RERUN_KITTI07 > /tmp/kitti07.log 2>&1 &
```

- 1101 帧，`NLM_USE_EKF_ODO=true`（与 C 类同批口径一致）。
- **要拿的数字**：日志中 EKF Fusion 与 Ours 的 RMSE（'simple' 对齐），
  对照论文 Table 2 KITTI07 行（现值：NLM 74.3 / EKF 85.0 / VO 85.6，
  Drift NLM 10.2 / EKF 13.8）。
- 本机自检值（2026-10-01，'simple' 口径，`KITTI07Data_IMU_Fusion/fusion_pose.txt`，1101 帧）：
  EKF ATE=141.25m / 终点漂移 11.13% —— 与论文 85.0m 差异大，
  需弄清论文 85.0m 用哪个文件/哪版数据（疑似旧 EKF 生成器或不同帧集）；
  以 MATLAB 重跑日志为准。

## 3. NLM 四场景重跑（C 类同批，核心工作，每场景独立进程、可并行但注意内存）

```bash
nohup ~/matlab_batch.sh RERUN_TOWN01   > /tmp/nlm_town01.log   2>&1 &
nohup ~/matlab_batch.sh RERUN_TOWN02   > /tmp/nlm_town02.log   2>&1 &
nohup ~/matlab_batch.sh RERUN_TOWN05   > /tmp/nlm_town05.log   2>&1 &
nohup ~/matlab_batch.sh RERUN_TOWN10HD > /tmp/nlm_town10hd.log 2>&1 &
```

- 口径：`FAST_TEST_MODE=false` 全量 5000 帧 + `NLM_USE_EKF_ODO=true`
  （NLM 运动源改用 EKF 轨迹逐帧增量；默认关闭的开关，重跑时开启）。
- 每个 RERUN 会覆写 `neuro/data/01_NeuroSLAM_Datasets/<Town>/slam_results/trajectories.mat`，
  旧版已有备份（`*_backup_202609*`），重跑前确认备份存在即可。
- **要拿的数字**：日志中 Ours / IMU-aided / Pure VO / EKF Fusion 的
  RMSE 与 Drift（'simple' 对齐）——Table 1 的 NLM 列 + EKF 列 + VO 列。
- 本机 'simple' 口径 EKF 自检值（2026-10-01，**当前** `01_NeuroSLAM_Datasets/<Town>/fusion_pose.txt`，
  5000 帧，即 `NLM_USE_EKF_ODO=true` 实际读取的数据源）：

  | 场景 | GT 长 | EKF ATE | EKF 漂移 |
  |---|---|---|---|
  | Town01 | 517.1m | 74.16m | 12.55% |
  | Town02 | 661.9m | 63.66m | 10.20% |
  | Town05 | 907.4m | 47.58m | 9.17% |
  | Town10HD | 448.8m | 88.90m | 31.41% |

  （MATLAB 重跑日志里的 EKF 列应与本表一致；不一致说明数据文件被换过。
  注：`/tmp/ekf_compare/` 是 9/27 另一批 replay，GT 长度不同，**不是**当前基准，勿引用。）

## 4. 消融（依赖第 3 步产出的 trajectories.mat）

```bash
nohup ~/matlab_batch.sh --dir ../07_test/test_imu_visual_slam/ablation RUN_ABLATION > /tmp/ablation.log 2>&1 &
```

- 读 `slam_results/trajectories.mat`，Procrustes(anchor-100 Sim(3)) 对齐，
  输出 `neuro/data/01_NeuroSLAM_Datasets/ablation_results/ablation_results_aligned.{mat,csv}`
  —— Table 3（消融）数字从 CSV 取。
- 覆盖 Town01/02/05/10HD（脚本内 datasets 表已含四场景）。

## 5. 取数汇总（跑完后一次性收集）

```bash
grep -hE 'RMSE|ATE|Drift|漂移' /tmp/nlm_town*.log /tmp/euroc_*.log /tmp/kitti07.log
cat ../data/01_NeuroSLAM_Datasets/ablation_results/ablation_results_aligned.csv
```

汇总表（Table 1/2/3 定稿值）填进 `neuro/kbs/README.md` 的 5 步清单第 1 步，
随后机械替换论文数字 + 删 30 处红色 [V2] 标记 + tectonic 编译。

## 常见坑

1. **RERUN 脚本会覆写 slam_results/**：跑之前确认 `*_backup_*` 存在，
   或自己再备份一份 `cp -r <Town>Dir/slam_results <Town>Dir/slam_results_bak_$(date +%m%d)`。
2. **NLM_USE_EKF_ODO=true 依赖 EKF odo 文件在数据集目录内**
   （Town: `fusion_pose_vo_ekf.txt` 或 `fusion_pose.txt`；KITTI: `fusion_pose.txt` 8 列）。
   缺文件会报 error，先补文件再跑。
3. **MATLAB 内存**：单场景 5000 帧峰值 ~4-6GB，四场景并行建议 ≤2 个一批。
4. **文件名中文/下划线开头**：`matlab_batch.sh` 文件投喂模式已处理，
   直接用脚本名模式（如 `~/matlab_batch.sh RERUN_TOWN01`）最稳。
5. **EuRoC MH_03**：数据目录 `02_EuRoc_Dataset/MH_03_medium/` 目前为空/缺失，
   重下数据后需跑 `euroc_csv_to_layout.py` 转成 cam0/data PNG + mav0 布局，
   再用新生成器 `euroc_ekf_fusion.py` 产出 `fusion_pose_vo_ekf.txt`，然后才能 RUN_SLAM_MH03。
