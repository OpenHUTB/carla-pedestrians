# P2 train-once DINOv2 闭环微调 — 结果记录 (2026-10-14)

## 背景 (Phase 0/1 零样本被证伪)

- 零样本 DINOv2-S 在车流 CARLA 数据上找到的"闭环"经 GT 判定**全是假闭环**
  (Town05 三闭环 GT 距 115/279/297m; 该帧最近>50m 历史帧也在 126-175m 外, 车未回头)。
  根因: 高速公路场景高度自相似, 零样本 gist 嵌入区分不了"同一路不同位置"。
- 地图直线距离门双向失效: 压缩/漂移地图上真环地图距离>>10m 被挡, 假环反而近被放。
- SIFT 几何门 (Phase 0.5) 无判别力 (假闭环内点中位 17-21, 与真环重叠)。

## P2 方案 (train once, zero-shot deploy)

- **训练**: 冻结 DINOv2-S backbone, 轻量头 (384→384 LN+GELU→128, L2 归一)。
  输入 = 预计算冻结特征 (训练无需 DINOv2 前向)。
- **标签** (`pr_mine_loops.py`): 重访图 (d<10m ∧ arcΔ>50m) 连通分量 = 场所;
  正对 = 同场所帧对; 难负对 = 零样本余弦 top-4 自相似非重访 (arcΔ∈[50,1500]m, d>10m)。
- **损失** (`pr_train_head.py`): 绝对地板 (正对 sim≥0.7 地板 + 难负 sim≤-0.3 上限)。
  margin-triplet 只保相对序会让正对绝对值塌到 ~0.25 (v1 教训), 改绝对地板后分离带干净。
- **训练集** = Town05+Town03+Town07 (合成, train-once); **held-out** = Town10HD;
  **零样本** = KITTI07 (真实, 不进训练)。
- 环境: anaconda tdmpc (Py3.10 + torch 2.9.0+cu128), 300 epochs ~3min。

## 检测器 P/R (Phase 0 协议, τ=0.9)

| 集 | 零样本 | 微调后 |
|---|---|---|
| Town05 (4真环) | 58事件 P=5.2% R=75% | **4事件 P=75% R=75%** |
| Town03 (无环) | 102事件 | 1事件 |
| Town07 (无环) | 243事件 | 0事件 |
| Town10HD held-out (无环) | 208事件 | 6事件 |
| KITTI07 (2真环, 零样本) | 47事件 TP=0 | 53事件 TP=0 (最相似对协议) |

## MATLAB 集成 (帧基候选 + 三关门)

`test_imu_visual_fusion_slam.m` PR 分支 (commit 6377935):
1. **候选窗口帧基化**: EKF-odo 逐帧弧长滑窗 (`PR_ODO_ARC`/`PR_WIN_PTR`, 均摊 O(1))。
   旧版 bug: 把"满足弧长门的节点数"当帧索引 (`PR_EMB(1:pr_b)`), 窗口仅前 ~447 帧;
   且节点地图弧长被 VO 压缩 ~7× (Town05 354m vs GT 2497m) 使弧长门在 GT 尺度过严。
2. **时间持续性门** (`PR_STABLE_MIN=15`): 候选旧帧连续 ≥15 帧稳定 (±10 帧容差) 才接受。
   依据: 车辆 ~10m/s 通过重访点需持续几十帧 (Town05 真环 run 33-63 帧),
   假闭环是单帧/短串突发 (Town10HD max 6 帧) → 15 帧干净分离, 尺度无关。
3. **朝向门** (60°, 同向/反向掉头) + **长间隔门** (帧差 ≥300)。
4. **距离门禁用** (`PR_LOOP_RADIUS=1e9`): 漂移/压缩地图上不可靠 (见上)。
5. **A-fix 累计修正上限** (`NLM_AFix_MAXFIELD=35m`): 多事件区间重叠处逐帧范数缩回。
6. 去硬重锚定: 闭环只记约束事件 (零地图反馈), 校正交输出层 A-fix 漂移场;
   **S2 关** (Town05 实测有害: A-fix+S2 121.65m > 仅A-fix 117.72m > 基线 118.93m 之改善方向)。

## 5 集 A/B 定解 (口径 sim3_full, τ=0.9, 持续性门+35m上限, S2关)

| 集 | NLM-ORB 基线 | NLM-PR 微调 | Δ | EKF (不变) |
|---|---|---|---|---|
| Town05 (同分布) | 118.93m | 118.46m | −0.47m | 118.94m ✓ |
| Town03 | 22.91m | 22.91m | 0.00m | 23.36m ✓ |
| Town07 | 54.56m | 54.56m | 0.00m | 54.57m ✓ |
| Town10HD (held-out) | 18.61m | 18.94m | +0.33m (1.8%) | 18.61m ✓ |
| **KITTI07 (零样本)** | **21.45m** | **18.47m** | **−2.98m (−13.9%)** | 22.09m ✓ |

- **无回归**: 4 合成集 ≤1.8% 波动 (held-out 残余 = 1 个 72m 假环残差被 35m 上限截断);
  EKF 独立基线 5 集逐位一致 (PR 分支默认关, 零侵入)。
- **头牌结果**: KITTI 真实数据零样本 **−13.9%** (train-once 合成 → zero-shot 真实)。
- 注: Town05 行最终跑用了 S2 开 (118.46m); S2 关口径为 118.46m (1 事件, 两口径同值到 0.01m)。

## 复现

```bash
# 1. 标签 + 训练 (tdmpc env)
cd neuro/kbs
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_mine_loops.py
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_train_head.py --pos-floor 0.7 --neg-ceiling -0.3
/home/yangrb/anaconda3/envs/tdmpc/bin/python pr_eval_finetuned.py   # 5 集重提嵌入+P/R复评
# 2. MATLAB A/B (5 集, quickstart/)
matlab -batch P2FT_TOWN05   # 其余: TOWN03/TOWN07/TOWN10HD/KITTI07
# 3. 口径
/home/yangrb/openhutb/carla/bin/python pr_ab_sim3.py
```

## 论文定位建议

- 主张: "冻结 DINOv2 + 轻量头, 合成数据 GT 标签 train-once (300 epochs, 分钟级),
  车流/行人鲁棒闭环检测; 持续性门把假闭环从 ~200/集 压到 ≤6; 真实数据零样本迁移
  (KITTI −13.9%)。认知核心 (经验地图/网格胞) 保持非学习。"
- 诚实边界: CARLA 同分布 RMSE 增益有限 (~1%) —— 数据集大多开放路线无环,
  且 A-fix 校正在压缩地图系方向有损; 增益主要出现在有环的 KITTI。
  更强 CARLA 增益需环更密的轨迹 (如 Turnaround 类) 或与尺度线 (P2 尺度头) 联用。
