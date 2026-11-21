# 投稿前数值核对清单

一条命令版本：

```bash
python3 neuro/kbs/run_all_checks.py                      # 默认 ../data/01_NeuroSLAM_Datasets
NLM_DATA_ROOT=/path/datasets NLM_BACKUP_ROOT=/path/_p11_backup_20261014 \
  python3 neuro/kbs/run_all_checks.py --json report.json
```

它只读不写（不覆盖两个 JSON），逐格打印 `OK / FAIL(+偏差) / SKIP(缺产物)`，末尾给
`PASS / FAIL / SKIP` 计数，退出码 0 = 无 FAIL。下表就是它对着比的全部内容。

---

## A 输入清单（每集 6 项 + 3 个计数）

| 检查 | 期望 |
|---|---|
| `ground_truth.txt` | 存在，含 `pos_x/pos_y/pos_z` |
| `fusion_pose.txt` | 存在，含 `pos_x/pos_y/pos_z` |
| `visual_odometry.txt` | 存在，含 `vo_x/vo_y/vo_z` |
| `slam_results/trajectories.mat` | 存在，变量 `exp_trajectory` |
| `slam_results/experiences.mat` | 存在，变量 `NUM_EXPS` / `EXPERIENCES` |
| GT 帧数 | Town05/03/07/10HD 各 5000；KITTI 07 1101 |
| GT 长度 | 485.6 / 473.5 / 617.9 / 452.3 / 692.1 m（±0.5） |
| 节点数 `NUM_EXPS` | 175 / 91 / 270 / 481 / 964 |
| 模板数 | 175 / 91 / 270 / 481 / 964 |
| 闭环边（每 experience 首条 link） | 7 / 7 / 7 / 1 / 11 |

## B Table 1（full Sim(3)，RMSE / drift %）

| 集 | NLM | EKF | VO |
|---|---|---|---|
| Town05 | 41.14 / 14.30 | 44.44 / 13.26 | 48.26 / 24.16 |
| Town03 | 22.91 / 5.03 | 23.36 / 5.14 | 49.10 / 17.59 |
| Town07 | 39.49 / 13.17 | 40.64 / 12.82 | 50.07 / 14.35 |
| Town10HD | 18.61 / 1.57 | 18.61 / 1.57 | 35.86 / 15.58 |
| KITTI 07 | 18.45 / 9.71 | 22.09 / 8.33 | 27.84 / 9.86 |

## C Table 3（8 格/集；`Ours` 在 KITTI 07 就是闭环线）

| 集 | Ours S3 / SE3 | EKF+scale | EKF | VO |
|---|---|---|---|---|
| Town05 | 18.07 / 32.67 | 20.10 / 33.26 | 44.44 / 49.40 | 48.26 / 53.86 |
| Town03 | 15.31 / 17.31 | 16.07 / 17.47 | 23.36 / 23.53 | 49.10 / 49.15 |
| Town07 | 28.75 / 31.51 | 27.90 / 29.21 | 40.64 / 42.85 | 50.07 / 50.13 |
| Town10HD | 14.44 / 49.88 | 14.44 / 49.89 | 18.61 / 57.38 | 35.86 / 58.04 |
| KITTI 07 | 18.45 / 29.99 | n/a | 22.09 / 30.45 | 27.84 / 38.35 |

另加 §4.4 消融一格：Town05 `ORB-only` Sim(3) = **44.72**（对照学习嵌入 41.14、裸 EKF 44.44）。

`SKIP` 的三类产物：`slam_results_scale_nlm/`、`slam_results_orbonly/`、
`_p11_backup_20261014/<Town05|Town03|Town07|Town10HD>/fp_head030.txt`。
用 `export_repro_bundle.py` 从运行树补齐后，这三类 SKIP 会变成 OK/FAIL。

## D 闭环基准（Town01 折返）

| 项 | 期望 |
|---|---|
| GT 长度 | 881.5 m |
| 帧数 | 10000 |
| 接受事件（`NLM_LOOP_EVENTS` 唯一帧号） | 36 |
| terminal 部署轨迹 | 55.92 m |
| terminal EKF 核 | 842.54 m |
| terminal preS2 | 565.31 m |
| RMSE 部署 / EKF | 87.25 / 477.76 m |
| 比值 EKF/部署 | 15.07 |

口径提醒：论文报的是 **terminal error**（anchored100 后最后一帧偏差），RMSE 只用于诊断。
`kb_s2_se3_check.m` 两种都打印。

---

## 判定与常见原因

- `FAIL` 且偏差 < 0.05 m：多半是浮点舍入或行长截断差 1–2 帧，可接受但要在报告里写清。
- `FAIL` 且偏差大（几十米）：先看是不是把 `slam_results/`（闭环线）当成了全系统，
  或者混用了旧批次/新批次的轨迹。
- `A` 段就 FAIL（帧数/长度不对）：数据没对齐——GT 与轨迹行数必须一致，KITTI 07 是 1101 帧。
- `D` 段 events ≠ 36：`loop_event_frames.txt` 没有末尾换行，`wc -l` 会显示 35，
  以 `NLM_LOOP_EVENTS` 行数为准。
- 想连 KITTI 09（补充材料 S.2）一起核：该序列不在资产包内，`run_all_checks.py` 会整段 SKIP。
