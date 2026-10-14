# P2 train-once DINOv2 闭环微调 — 结果记录 (2026-10-14, 二次重采后最终版)

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

## 5 集 A/B 定解 (新数据, τ=0.9, 门=15, pos-cap 头, S2 关)

| 集 | NLM (P2) | EKF | VO | NLM−EKF |
|---|---|---|---|---|
| Town05 (同分布) | 41.14m | 44.44m | 48.26m | **−3.30m (−7.4%)** |
| Town03 | 22.91m | 23.36m | 49.10m | **−0.45m (−1.9%)** |
| Town07 | 41.73m | 40.64m | 50.07m | +1.09m (+2.7%) |
| Town10HD (held-out) | 19.43m | 18.61m | 35.86m | +0.82m (+4.4%) |
| **KITTI07 (零样本)** | **21.48m** | **22.09m** | **27.84m** | **−0.61m (−2.8%)** |

- **3/5 胜** (含零样本 KITTI); NLM 全线优于 VO (−13.7% ~ −46.5%)。
- Town07/Town10HD 小负值归因: ORB-only 消融证明 ORB 侧无问题 (Town07 39.99 < EKF,
  Town10HD 18.61 = EKF), 负值全来自 PR 侧假闭环 A-fix 过冲 — 无环/稀疏环集上
  EKF 实际漂移小 (Town10HD 18.6m), 假环修正 (各 ~25m) 叠加逼近 35m 上限 → 过校正。
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

| 集 | NLM@活体 | NLM@尺度 | Δ |
|---|---|---|---|
| Town03 | 22.91m | 15.31m | **−7.60m** |
| Town05 | 41.14m | 18.07m | **−23.07m (−56%)** |
| Town07 | 41.73m | 28.67m | **−13.06m (−31%)** |
| Town10HD | 19.43m | 21.90m | +2.47m (该集尺度线零注入, NLM 层为运行方差) |

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
# 3. MATLAB (quickstart/): 5 集 A/B + 尺度 NLM 层 A/B
matlab -batch P2FT_TOWN05    # 其余: TOWN03/TOWN07/TOWN10HD/KITTI07
matlab -batch P2S_TOWN05     # 尺度 NLM 层 (需先换 fusion_pose, 见 run_scale_nlm_ab.sh)
# 4. 回装 + 指标
bash refit_and_metrics.sh
```

## 论文定位建议

- 主张: "冻结 DINOv2 + 轻量头, 合成数据 GT 标签 train-once (分钟级), 车流/行人鲁棒闭环
  检测 (pos-cap 标定修复相似度饱和); 持续性门 + 旧帧去重 + 35m 上限把假闭环伤害压到
  ≤4.4%; 真实数据零样本迁移 (KITTI −2.8%)。认知核心 (经验地图/网格胞) 保持非学习。"
- 更强主张在**尺度线**: train-once 尺度头 (8 维深度统计特征) 使 NLM 无 RTK 度量精度
  Town05 较 VO −63% (18.07m), EKF 层 Town03 −81%。
- 诚实边界: 闭环线 CARLA 3/5 胜 (无环/稀疏环集假闭环过冲 +1~3%); KITTI 闭环零样本
  TP=1/2 (域差不迁移大部分); 尺度线 Town10HD 零注入 (死区), KITTI 过度修正。
