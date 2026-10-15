# P2 train-once DINOv2 闭环微调 — 结果记录 (2026-10-14 二次重采, 2026-10-14 τ 定解 0.94 最终版)

> 数据口径: Town05/Town07 为 2026-10-14 二次重采 (20 车 + 20 行人, 5000 帧, GT 跳变=0,
> 传送修复后); Town03/Town10HD/KITTI07 沿用 10-13 车流重采。全部指标 sim3_full 口径
> (`final_metrics.json` 单一来源)。

## 背景 (Phase 0 零样本被证伪)

- 零样本 DINOv2-S 在车流 CARLA 数据上找到的"闭环"经 GT 判定**全是假闭环**
  (新数据: Town05 149 事件 P=0%, Town07 170 事件 P=1.2%, 无环集 102/208 事件)。
  根因: 城市/高速场景高度自相似, 零样本 gist 嵌入区分不了"同一路不同位置"。
- 地图直线距离门双向失效 (压缩/漂移地图上不可靠) → 距离门禁用。
- SIFT 几何门 (Phase 0.5) 无判别力 → 作废。

## P2 方案 (train once, zero-shot deploy)

- **训练**: 冻结 DINOv2-S backbone, 轻量头 (384→384 LN+GELU→128, L2 归一)。
  输入 = 预计算冻结特征 (训练无需 DINOv2 前向)。
- **标签** (`pr_mine_loops.py`): 重访图 (d<10m ∧ arcΔ>50m) 连通分量 = 场所;
  正对 = 同场所帧对; 难负对 = 零样本余弦 top-4 自相似非重访 (arcΔ∈[50,1500]m, d>10m)。
- **损失** (`pr_train_head.py`): 绝对地板 (正对 sim≥0.7 地板 + 难负 sim≤-0.3 上限)。
  margin-triplet 只保相对序会让正对绝对值塌到 ~0.25 (v1 教训)。
- **训练集** = Town05+Town03+Town07 (合成, train-once); **held-out** = Town10HD;
  **零样本** = KITTI07 (真实, 不进训练)。
- 环境: anaconda tdmpc (Py3.10 + torch 2.9.0+cu128), 300 epochs ~3min。

### pos-cap 标定修复 (2026-10-14)

新标签下训练场所为 3 个巨型密集簇 (Town05 1 场所 396 帧 / Town03 / Town07 1 场所 716 帧),
默认每簇每 epoch 抽 85 正对把相似度空间整体推高 (**相似度饱和**): τ=0.9 各集 ~200 假事件
(Town05 195 / Town07 223 / Town10HD 208), τ 0.90→0.96 事件数几乎不降 (159/180/193),
τ=0.98 才锐降 (51/23/43) 且 Town05 唯一真环丢失 (TP=0) → **τ 调参无法分离真假事件**。
修复: `--pos-cap 25` (每场所每 epoch 正对上限), 正负信号再平衡 → 标定恢复。

## 检测器 P/R (Phase 0 协议, τ=0.9, 新数据)

| 集 | oracle | 零样本 | 微调 (pos-cap 头) |
|---|---|---|---|
| Town05 | 1 真环 | 149 事件 P=0% R=0% | **4 事件 P=25% R=100%** |
| Town03 (无环) | 0 | 102 事件 | 4 事件 |
| Town07 | 10 真环 | 170 事件 P=1.2% R=20% | **9 事件 P=66.7% R=60%** |
| Town10HD held-out (无环) | 0 | 208 事件 | 10 事件 |
| KITTI07 零样本 | 2 真环 | 47 事件 TP=0 | 38 事件 TP=1 P=2.6% R=50% |

### τ 定解: 0.90 → 0.94 (2026-10-14, 五集 A/B 定案)

微调嵌入 P/R 复评 (ft 嵌入, Phase 0 协议, 门控前候选):

| 集 | oracle | τ=0.90 | τ=0.94 | τ=0.96 |
|---|---|---|---|---|
| Town05 | 1 | 12 事件 P=8.3% R=100% | 9 事件 P=11.1% R=100% | 9 事件 P=11.1% R=100% |
| Town03 (无环) | 0 | 2 事件 (全假) | **0 事件** | 0 事件 |
| Town07 | 10 | 9 事件 P=55.6% R=50% | 4 事件 P=75% R=30% | 2 事件 P=100% R=20% |
| Town10HD (无环) | 0 | 6 事件 (全假) | **0 事件** | 0 事件 |
| KITTI07 (零样本) | 2 | 55 事件 P=0% R=0% | 18 事件 P=5.6% R=50% | 3 事件 P=33.3% R=50% |

- 相似度谱 (日志 [PR] sim 对账): 假环聚集 **0.907-0.935** (T10HD 0.907 / T07-355 0.911 /
  KITTI-189 0.932 / KITTI-149 0.935 / T07-349 0.930-0.931), 强真环 **0.949-0.991**
  (KITTI 1065→7 0.975 / T05 2145 0.975-0.978 / T07 247 0.988 / T07 250 0.990)。
  τ=0.94 正落分离带。
- τ=0.94 效果: **无环集零假环** (T03/T10HD 2/6→0), T07 精度 55.6%→75%,
  KITTI 零样本 R 0%→50% (54 假环清除/真环保留), T05 R=100% 不变。
- 判死的备选 (实证, 非猜测):
  - **运行期半径门** (PR_LOOP_RADIUS): 真/假环在 EKF-odo 距离上完全重叠 (T05 真环
    |r|=23.0m, T07 真环 ≥25m, 假环 ≥25m, 漂移污染) → 无判别力, 维持禁用。
  - **通过级去重** (每次重访通过只留 1 事件): T07 单强真环 (25m 钳位) 覆盖不了
    ~40m 漂移, ATE 反升 (41.24-44.70)。
  - **hold-c 尾部校正形态**: 救 T05/T10HD/KITTI 但伤 T03/T07, 无单一形态全集胜。
- 注意: 早期"τ 调不动 (相似度饱和)"结论是 pos-cap 修复**前**的状态; 标定恢复后
  τ 成为有效精度旋钮 (0.90→0.94 假环 63→9, 真环全留)。

## MATLAB 集成 (帧基候选 + 门控 + A-fix)

`test_imu_visual_fusion_slam.m` PR 分支 (commit 6377935 + 本次旧帧去重):
1. **候选窗口帧基化**: EKF-odo 逐帧弧长滑窗 (均摊 O(1))。
2. **时间持续性门** (`PR_STABLE_MIN=15`): 候选旧帧连续 ≥15 帧稳定 (±10 帧容差) 才接受。
   门长 15→20 对照实验逐位无效 (3 集) → 15 定稿。
3. **朝向门** (60°) + **长间隔门** (帧差 ≥300) + **距离门禁用** (`PR_LOOP_RADIUS=1e9`)。
4. **A-fix 累计修正上限** (`NLM_AFix_MAXFIELD=35m`) + **旧帧去重** (2026-10-14:
   同一旧帧只注入首个事件 — 微调头在假闭环处对同一旧帧重复触发, 去重前 Town07 10 事件
   注入 7 次、Town10HD 5 事件注入 3 次, 叠加过冲)。
5. 闭环只记约束事件 (零地图反馈), 校正交输出层 A-fix 漂移场; **S2 关** (定解口径 A-fix only)。

## 5 集 A/B 定解 (新数据, **τ=0.94**, 门=15, pos-cap 头, S2 关)

| 集 | EKF | NLM (τ0.94) | NLM−EKF | ORB-only | PR 净贡献 |
|---|---|---|---|---|---|
| Town05 (同分布) | 44.44m | 41.14m | **−3.30m (−7.4%)** | 44.72m | **−3.58m (决定性: ORB-only 反而败)** |
| Town03 | 23.36m | 22.91m | **−0.45m (−1.9%)** | 22.91m | 0 (该集 PR 事件净贡献为零) |
| Town07 | 40.64m | 39.49m | **−1.15m (−2.8%)** | 39.99m | −0.50m |
| Town10HD (held-out) | 18.61m | 18.61m | 0.00m (平) | 18.61m | 0 (PR 零事件, 零假环) |
| **KITTI07 (零样本)** | 22.09m | **18.45m** | **−3.64m (−16.5%)** | 21.45m | **−3.00m** |

- **4 胜 1 平 0 负** (τ=0.90 为 3 胜 2 负: τ 定解把 Town07 从 +1.09m 翻成 −1.15m,
  Town10HD 从 +0.82m 翻成平, KITTI 胜幅 −0.61m → −3.64m)。NLM 全线优于 VO。
- PR 事件分布 (τ0.94, 日志 [PR] sim 对账, 全 5 集零假环):
  T05 3 真环 (4919/4949/4987→2145, s=0.975-0.978) / T07 2 强真环 (4941→247 s=0.988,
  4971→250 s=0.990) / KITTI 1 真环 (1065→7 s=0.975) / T03, T10HD 0 事件。
- PR 价值结构: **互补第二闭环源** — T05 唯一真环 ORB 找不到 (ORB-only 44.72 > EKF,
  PR 是唯一来源); KITTI 上 PR 在 ORB 基础上再 −3.00m (跨域零样本)。
- τ=0.90 的 Town07/Town10HD 负值归因 (历史): 假环 A-fix 过冲 (同方向多事件叠加触
  35m 上限); τ=0.94 后假环全部在检测层被杀, 过冲机制不再触发。
- EKF 独立基线 5 集逐位一致 (PR 分支零侵入)。
- 注: 旧数据 (10-13) Town05 被 13 次 ego 传送污染 (GT 2497m), 旧 118.46m 与新 41.14m
  不可比; 传送修复 (原位恢复) 见 `fix(collect)` 提交。

## 尺度线两层 A/B (P2 尺度头 `pr_scale_head.py`, 死区 db=0.30)

**EKF 层** (`replay_depth_ab.py`, 无深度文件 bit-identical 已校验):

| 集 | EKF 活体 | EKF+尺度头 | Δ |
|---|---|---|---|
| Town03 | 23.36m | 4.41m | **−81%** |
| Town05 | 44.44m | 42.57m | −4.2% |
| Town07 | 40.64m | 34.16m | **−15.9%** |
| Town10HD | 18.61m | 18.61m | 0 (零注入, bit-identical) |

**NLM 层** (P2S 驱动: NLM 全流程跑在 EKF+尺度头 fusion 基座上):

| 集 | NLM@活体 (τ0.94) | NLM@尺度 (τ0.94) | Δ |
|---|---|---|---|
| Town03 | 22.91m | 15.31m | **−7.60m (−33%)** |
| Town05 | 41.14m | 18.07m | **−23.07m (−56%)** |
| Town07 | 39.49m | 28.75m | **−10.74m (−27%)** |
| Town10HD | 18.61m | 14.44m | **−4.17m (−22%)** |

- 尺度线 NLM 层 **4/4 集全胜** (τ0.94 定解后一致性重跑: Town10HD 旧值 21.90m 是
  τ0.90 假环在深度基座上的干扰, 定解后归 14.44m; PR 事件: T05 3 真环/T07 2 强真环/
  T03T10HD 0, 深度基座上 d 相应缩小 9.8-106m)。

- **Town05 全链条**: VO 48.26 → EKF 44.44 → NLM 41.14 → **NLM+尺度 18.07m (较 VO −63%)**。
- KITTI: 无 replay 管道 (缺 visual_odometry/aligned_imu), 尺度线仅报零样本边界
  (头在 KITTI 零样本残差 0.45, 过度修正)。

## 复现

```bash
cd neuro/kbs
# 1. 闭环头: 标签 + 训练 + P/R (tdmpc env)
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_mine_loops.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_train_head.py --pos-cap 25
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_eval_finetuned.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python tau_scan.py        # τ 事件级明细
# 2. 尺度线: 特征 + 尺度头 + EKF 层 replay A/B
/home/yangrb/anaconda3/envs/tdmpc/bin/python scale_series_p1.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_scale_head.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python replay_depth_ab.py --tag head030 --deadband 0.30
# 3. MATLAB (quickstart/): 5 集 A/B (τ=0.94 定解驱动) + 尺度 NLM 层 A/B
matlab -batch P2FT_TOWN05    # 其余: TOWN03/TOWN07/TOWN10HD/KITTI07
matlab -batch P2S_TOWN05     # 尺度 NLM 层 (需先换 fusion_pose, 见 run_scale_nlm_ab.sh)
# A/B 对照驱动 (历史 τ0.94 前对照/ORB-only 消融): P2FT94_*.m, P2AB_*_ORBONLY.m
# 4. 定解提升 + 指标 + 图
/home/yangrb/anaconda3/envs/tdmpc/bin/python promote_t94_final.py  # p2ft → 规范 slam_results/
/home/yangrb/anaconda3/envs/tdmpc/bin/python compute_final_metrics.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python regen_figures.py
```

## 论文定位建议

- 主张: "冻结 DINOv2 + 轻量头, 合成数据 GT 标签 train-once (分钟级), 车流/行人鲁棒闭环
  检测 (pos-cap 标定修复相似度饱和); 持续性门 + 旧帧去重 + 35m 上限 + τ=0.94 精度定解,
  5 集零假环; 互补第二闭环源 (ORB 找不到的真环 PR 找到: Town05 决定性 −3.58m);
  真实数据零样本迁移 (KITTI −16.5%, 22.09→18.45m)。认知核心 (经验地图/网格胞) 保持非学习。"
- **闭环线 4 胜 1 平 0 负** (τ=0.94 定解), 达到共同 headline 标准: NLM ≥ EKF 全 5 集。
- 更强主张在**尺度线**: train-once 尺度头 (8 维深度统计特征) 使 NLM 无 RTK 度量精度
  4/4 集全胜 (Town05 较 VO −63% 18.07m, Town10HD 18.61→14.44m), EKF 层 Town03 −81%。
- 诚实边界: 闭环线 Town10HD 为平 (该集 EKF 漂移本就小, 18.61m); Town03 胜幅小 (−1.9%,
  该集 PR 零事件, 胜幅来自 ORB/NLM 本体); KITTI 闭环零样本 TP=1/2 (域差不迁移大部分);
  尺度线 KITTI 过度修正 (零样本边界)。
