# 方法章骨架 (2026-10-16, #29) — "NLM 全系统" 主形态叙事

> 目标形态：EKF 跟踪核 + **尺度支路（主贡献）** + 闭环线，"train once, zero-shot deploy"。
> 以现有 `sec_method_expanded.tex`（416 行，4 小节）为基座改写，非推倒重来。
> 数字一律引用 `final_metrics.json` / `se3_metrics.json`（单一来源）。

---

## 0. 覆盖差距诊断（改写动因）

现有方法章缺失（grep 证据）：

| 缺失内容 | 证据 | 论文主形态地位 |
|---|---|---|
| 尺度支路（P2 尺度头） | DINOv2/DAv2/微调头 **0 处** | **主贡献**：把 up-to-scale VIO 升到度量级 |
| train-once / zero-shot 叙事 | zero-shot 仅 1 处 | 全系统泛化主张的骨架 |
| PR 嵌入闭环线（帧基候选 + 门链） | 帧基候选重写未入章 | 全系统第三模块 |
| 双口径评测依据 | Sim(3) 仅 1 处 | 度量 SLAM 主张的口径诚实性 |

---

## 1. 骨架总览（4 小节 → 6 小节）

```
1 Notation and Conventions                    [现有, 微调]
2 Framework Architecture                      [现有, 增第三模块定位段]
3 Sensory Processing and Fusion               [现有, 尾部+尺度支路输入接口]
4 Neural Spatial Representation               [现有, 保留]
5 Monocular Scale Recovery Branch  ★新        [主贡献, ~60-80 行 tex]
6 Experience Map and Loop Closure             [现有, 重写 PR 闭环线段 ~30 行]
```

---

## 2. 各小节改写要点

### 小节 1（Notation）— 微调
- 增：全局尺度因子 $s \in \mathbb{R}^+$（VO 相对位姿与度量位姿之比）；DAv2 metric depth $\hat z_t$；
  尺度回归头 $g_\theta$（参数 θ 只在 CARLA 训练期优化一次）。

### 小节 2（Framework Architecture）— 加一段
四模块枚举后加三模块定位语：
"Beyond the four classical modules, a third processing pathway — a deep monocular scale
recovery branch — couples the visual front-end to the fused estimate: a lightweight regression
head $g_\theta$ over frozen DAv2 features maps each frame's metric depth to a scale
correction for the 10th EKF state $\log s$. Its parameters are optimized once on simulated
data and frozen at deployment (train once, zero-shot deploy), in contrast to online
scale-from-motion observability which plateaus at the VO-relative scale."

### 小节 3（Sensory Processing and Fusion）— 尾部接口段
dorsal 光流小节尾加接口句：
"The dorsal branch additionally provides the pixel-level inverse-depth field $\hat z_t$
(DAv2 metric depth) and frozen-backbone features consumed by the scale recovery branch
(Section 5); recognition and tracking remain decoupled from scale estimation."

### 小节 5 ★ Monocular Scale Recovery Branch（主贡献，全新）
定式：问题 → 结构 → 训练协议 → 门控集成 → 零样本性 → 复杂度。

1. **问题**（3-4 句）：单目 VO 位姿 up-to-scale；EKF 第 10 态在线尺度 $\log s$ 的运动学观测
   （$|a_{lat}|/|\omega|$ 归一化 VO 速度）仅转弯帧可观测（KITTI09: 294/1101 帧）且收敛到
   VO 相对尺度，推不到真实全局尺度。需要一路不依赖运动、不依赖回环的独立尺度证据。
2. **结构**（公式 (21)–(24)，接现有 eq 计数）：
   - 冻结 DAv2 backbone $\phi(\cdot)$（metric outdoor head）+ 轻量头 $g_\theta$：
     2×(Linear 256→256) + GELU + LayerNorm；
   - 输入：逐帧深度统计向量（分位数 + 梯度能量 + 相对运动先验），
     输出 $\hat{\ell}_t = g_\theta(\cdot) = \widehat{\log s_t}$；
   - 监督：训练期 GT 逐帧真值尺度 $\ell^*_t = \log(\|p^*_t-p^*_{t-1}\| / \|p_t-p_{t-1}\|)$。
3. **训练协议（train once）**：3 个 CARLA 场景（Town05/03/07）约 15k 帧样本，
   AdamW lr 1e-3，早停于留出集 Town10HD；**训练后冻结，部署期零更新**。
   强调：不是 per-sequence calibration，不是 online adaptation。
4. **门控集成**：$\hat{s}_t=\exp(g_\theta(\cdot))$ 经 chi-square 门 + log 域死区 (db=0.30)
   写入 $x_{10}$；深度支路启用时屏蔽运动学尺度观测（避免两路证据冲突拉锯）。
5. **零样本性论证**（1 段）：DAv2 零样本深度 bias 场景相关（P1.1 结论），但头学的是
   "DAv2 深度→真值尺度"的**映射**而非深度本身；CARLA 3 场景训练、Town10HD 留出 +
   KITTI07/09 零样本评估；失败模式诚实披露（KITTI 零样本尺度支路失效，见实验章口径节）。
6. **复杂度**：头部 ~0.2M 参数，逐帧前向 <1ms CPU，不进 31 FPS 关键路径。

### 小节 6（Experience Map and Loop Closure）— PR 闭环线段重写
现有经验地图迭代 + heuristic 闭环保留；PR 段（~30 行）：

1. **候选生成**：帧基候选 = 全历史 argmax（窗口 = EKF-odo 弧长 ≥ PR_ARC_GAP 的全部旧帧）；
   嵌入 = 冻结 DINOv2-S 全局场景描述 + P2 微调头（与尺度头同一 train-once 协议）。
2. **四重门链**（公式 (25)–(28)）：
   ① 相似度 τ=0.94（五集 A/B 定解，杀假环 0.907–0.935 / 留强真环 0.949+）
   ② 持续性门 ≥15 帧（±10 容差）——真环=车辆持续通过重访点，假环=单帧/短串突发，尺度无关
   ③ 朝向门 60°（同向/反向掉头容差）
   ④ 累计修正上限 35m + 单事件 25m 钳位。
3. **A-fix 注入动作**：真环放行 → 地图残差 $r$ 以 $\beta=1.0$ 步长修正（同 exp_map_iteration），
   写 link 边（去重）；与经验地图 relaxation 循环正交（后者禁用，EXP_LOOPS=0）。
4. **诚实披露段**（一段）：持续性门的结构性边界——重访发生在序列末尾时 streak 攒不到 15
   （KITTI09: 真环在末 2 帧，GT 位置差 3.8m）→ 门正确拒绝，what-if 放行仅 −0.12m；
   09 两个假环（GT 差 90/134m）被朝向门正确拒绝 → 门链跨域保持精度不牺牲。

---

## 3. 双口径评测的衔接（实验章接口，不在方法章展开）

小节 5 末尾桥接句："The metric-scale claim is evaluated under two alignments
(Section Experiments): rigid SE(3) alignment (the honest metric-SLAM metric) and full
Sim(3) refit (the shape-only upper bound that absorbs any global scale, by construction)."

主张限定语（已定解）：全 Sim(3) 重拟合 = **上界**；尺度支路 = **在线、仅尺度的校正**（9× 表述切开）。

---

## 4. 与实验章/数据的对应关系

| 方法章新内容 | 实验章对应 | 数据来源 |
|---|---|---|
| 尺度支路 §5 | 尺度线 A/B 表 + 9× 闭环 benchmark | `final_metrics.json` (EKF+尺度 / NLM-全系统) |
| PR 闭环线 §6 | τ=0.94 五集 A/B + KITTI09 诚实案例 | `se3_metrics.json` |
| train-once 协议 | 消融 (ORB-only / 零样本 vs 微调) | `ISSUES_20261015.md` 附表 |
| 09 负结果 (微调头域差距) | 零样本泛化讨论 | `ISSUES` 七.K |

---

## 5. 续写顺序与公式编号

- 续写顺序：小节 5 → 小节 6 重写段 → 小节 2 架构段 → 小节 3 接口段 → 小节 1 符号。
- 公式：小节 5 用 (21)–(24)（$g_\theta$ 输出与 $x_{10}$ 写入门控），
  PR 门链用 (25)–(28)；全部符号在 §1 补齐。
- EN 定稿后同步 zh（`sec_method_expanded_zh.tex`，EN/zh 同步是硬约束）。
- 编译检查：EN pdflatex / zh xelatex，MATLAB R2021b 约束不涉及本章。
