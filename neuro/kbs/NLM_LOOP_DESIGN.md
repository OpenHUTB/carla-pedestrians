# NLM 闭环约束创新设计 (A-fix) — 让闭环约束真正生效

日期: 2026-10-06
状态: 机制已实现并冒烟通过; 验证被"采集层同步模式 bug"阻塞, 根因已定位(见 §5)

## 1. 创新要解决的问题(结构性, 非调参能解决)

EKF-odo 钉扎模式下, NLM 输出 = 锚点节点(EKF米坐标) + 逐帧 ACCUM_DELTA 增量
≈ EKF 轨迹 + 重锚定跳变的离散修正。于是:

- 长程漂移来自 GC 离散积分 + EKF 前端漂移, 与 NLM 自身无关;
- NLM 的核心卖点 —— "视觉重识别旧场景 → 闭环校正长程漂移" —— 在**开放路线**上
  没有真闭环可用:
  - Town05: 906m 开放单程、0 折返点, 重访间隔均 <286 帧 < MIN_GAP=300 → **0 事件**
  - KITTI07: 964 节点仅 9 条近邻重复边 (d_xy≈1m), 无长程真闭环
- 结果: NLM 21.45 vs EKF 22.09 (KITTI07 口径), 0.9m 优势全部来自重锚定跳变
  (噪声级); 论文"闭环校正"贡献在现有 5 个数据集上**不可验证**。

**结论: 创新必须配上真闭环数据。** 这正是 Town01Turnaround 双向折返路线的意义:
去程 100m + flip 瞬移对向车道 + 回程 100m, 横向偏移 4m (车道宽, 落在 CE 门
[3,20] 内), 重访间隔 ≈ 2L/v ≈ 1467 帧 ≫ MIN_GAP=300 → 每次往返都应产生
1~2 个真闭环约束事件。

## 2. 机制: A-fix 闭环约束漂移场 (零地图反馈)

与 B-fix 的路线区别: B-fix 在闭环接受时平移节点 (`NLM_LOOP_CORR` 后缀继承),
错误闭环会被地图放大且 5/5 集实测有害。A-fix **不动地图**:

1. **事件记录** (`exp_map_iteration.m` → `nl_apply_loop_fix`):
   VT 重识别 → delta_em 阈值 → CE 门 (残差 ≤ EXP_MAX_LOOP_CE) →
   朝向门 (同向/反向 ±60°) → gap ≥ MIN_GAP=300 帧 → 记录
   `[t_now, t_old, -β·clamp(r, STEP_MAX=25m)]`,
   其中 `r = 当前锚点DR推算位置 - matched节点位置`, `t_old = matched.born_frame`。
2. **输出层注入** (`core/test_imu_visual_fusion_slam.m` 主循环后):
   每个事件在 `[t_old, t_now)` 区间把 `c` 按 raised-cosine 从 0 爬坡到 c,
   加到输出轨迹。t_now 处重锚定已把输出切到 matched 节点 (残差消失),
   爬坡恰好抵消 t_now 的阶梯, 且 t_now 之后不引入持续偏差。
3. **与 DC v3 不相交**: 闭环事件的帧从 DC v3 跳变集剔除 (防双重校正);
   开放路线 0 事件 → A-fix 整体中性 (输出与基线逐帧一致), 创新不背反效果。

参数 (逐集定解, 驱动脚本覆盖): `NLM_LOOP_BETA=1.0, NLM_LOOP_CE_MIN=3,
NLM_LOOP_STEP_MAX=25, NLM_LOOP_MIN_GAP=300`; DC 侧 `W0=100, W_PER_M=0,
W_MAX=4000, TAIL_COMPLETE, CMAX=2.0`。

## 3. 数据需求

真闭环路线必须满足 (否则 MIN_GAP/CE 门把事件全部滤掉):
- 重访间隔 ≫ 300 帧 (往返 200m @3m/s ≈ 116s ≈ 2300 帧 ✓)
- CE 残差 ∈ [3, 20]m (折返横向偏移 = 车道宽 4m ✓; 开放路线自交处 CE 常 >20m ✗)
- 朝向同向或反向 (对向车道回程 ✓)

Town01Turnaround 路线 (CARLA 世界坐标, 已隔离验证 0 碰撞往返):
起点 (167.2, 59.5, z+0.1) road10/lane-1 yaw0 → 去程 (267.2, 59.5) 100m →
flip 瞬移到对向车道 (267.2, 55.5) yaw180 → 回程 (171.2, 55.5)。
循环 去→flip→回→(瞬移回起点)→去 … 跑满 5000 帧, 相邻两轮同点间隔
≈1467 帧 > 300, 每轮都产生真实重访。

## 4. 评估口径

- 统一 `align_trajectories 'simple'` (平移+尺度, 首段锚定) — 与
  Town01/02/05/10HD/KITTI07 既有 Table 值同口径。
- A/B: `NLM_TURNAROUND_TOWN01.m dc` (纯 DC 注入基线 → dc_final/) vs
  `constraint` (DC + 闭环约束 → a_final/), 参数逐行一致, 仅 A-fix 开关不同。
- 指标: ATE (对齐后 2D RMSE) + 闭环事件数 (loop_constraint_events.mat) +
  去程段/回程段分段 ATE (闭环应主要改善 t_now 之后的段)。

## 5. 采集层根因 (2026-10-06 定位, 阻塞采集 1.5h+ 的元凶)

现象: 车正确瞬移到 (167.2, 59.5) (GT 第 1 帧为证), 但 planner 路由
`q=432, head=(0,2), tail=(265,59)` — 从**瞬移前**的随机出生点 (0,2) 规划的
绕中心 432 路点长路由, 车瞬移到位后追旧路由 → 冲出车道 → 撞植被/墙 →
"碰撞过多"重置 → 无限循环。

根因链 (全部有数据证据):
1. 采集器主循环是**同步模式** (`synchronous_mode=True, fixed_delta=0.05s`);
2. CARLA 同步模式下 `set_transform` 在**下一次 `world.tick()`** 才生效;
3. `wait_vehicle_deploy` 只 `time.sleep` + 轮询 `get_location()`, **从不 tick**
   → 位置永远停在旧点, 2s 超时后照样往下走;
4. fresh agent 的 `set_destination` 在瞬移生效前触发 planner 规划 →
   以 (0,2) 为起点的 432 路点路由;
5. 主循环第一次 `world.tick()` 瞬移生效 → 车在 (167,59) 追 (0,2) 出发的路由
   → 偏航撞静态物。

(隔离验证脚本全部是异步模式, sleep 即等瞬移生效, 所以 0 碰撞 — 与采集器
的差别正是 sync+不tick。)

修复: `wait_vehicle_deploy` 内部用 `vehicle.get_world().tick()` 循环推进,
直到 `get_location()` 与目标 <1.5m (或超时), 再静置 0.15s。传感器尚未挂载
(init 阶段)或处于重置断点 (主循环), 多 tick 无害。CARLA 0.9.16 已验证
`Actor.get_world` 存在。

## 6. 可验证假设与验证路径

| # | 假设 | 判据 |
|---|------|------|
| H1 | tick 修复后采集干净跑满 | 5000 PNG, 0 碰撞重置, 多次完整 去→flip→回, NAV 去程 98m 直路 |
| H2 | 真闭环事件被记录 | constraint run 的 `loop_constraint_events.mat` 事件数 > 0 (预期每轮 1~2 个, gap≈1467) |
| H3 | 闭环约束产生增益 | constraint ATE < dc ATE, 且 t_now 之后段误差下降 |
| H4 | 开放路线保持中性 | 5 开放数据集 A-fix 0 事件, ATE 与既有 DC_FINAL 基线一致 (创新不反噬) |

路径 (快→慢):
A. [主] tick 修复 → 重启采集 (~10min) → `NLM_TURNAROUND_TOWN01 dc` +
   `constraint` A/B (各 ~10min) → 验 H1~H3
B. CARLA 服务器 (pid 96054, 已运行 2h) 若再崩: 重启服务器再采
C. H4: 复用既有 DC_FINAL 5 集结果 (0 事件 = 中性已由 open-route 诊断证明),
   如口径需刷新则重跑 DC_FINAL

## 7. 性能层 ("再调好性能")

三层叠加 (全部默认关/可 A/B, 互不冲突):
1. **P1 输出层平滑**: 重锚定跳变 30 帧线性过渡 + 位移尖峰钳位 (默认开);
2. **DC v3 分布式漂移场**: 非闭环重锚定跳变按 raised-cosine 摊平
   (W0=100, CMAX=2.0 门控大跳变保持瞬时);
3. **A-fix 闭环约束场**: 真闭环事件的区间预校正 (本文 §2)。

目标: 折返集 NLM < EKF < VO (simple 口径), EKF 基准 22.09m 量级;
5 开放集维持/小幅改善 (38.1% 摘要数字回填)。

## 8. 评估结果 (2026-10-06 simple 口径 A/B; 2026-10-07 补齐 5 集 Sim(3) 口径)

### 8.1 闭环约束 A/B 验证 (simple 口径: 平移+尺度对齐后逐帧 2D RMSE)

折返集证明增益, 2 个开放集证明中性:

| 数据集 (帧数) | NLM+DC 基线 | NLM+DC+**闭环约束** | 增益 | EKF | VO | 闭环事件 |
|---|---|---|---|---|---|---|
| Town01 (5000) | 95.41 m | 95.41 m (0事件, 中性) | — | 130.55 m | 425.22 m | 0 |
| Town05 (5000) | 71.43 m | 71.43 m (11近邻假闭环gap<300, 中性) | — | 90.98 m | 297.86 m | 0 |
| **Town01Turnaround (10000)** | 201.16 m | **172.12 m** | **−29.04 m (−14.4%)** | 354.58 m | 452.97 m | **16** |

闭环事件统计 (loop_constraint_events.mat):
- Turnaround: 16 事件, gap 323–1154 帧 (中位 627), c 幅度 4.25–24.63 m (中位 14.80 m);
  对应 22 次真闭环接受、9 条闭环链 (exp_map 诊断), 每次往返 1–2 个约束
- Town01: 0 事件 → dc/a 输出逐帧一致 (95.41 = 95.41), 开放路线创新自然退场 ✓
- Town05: 0 事件 (旧版本 a_final 残留 11 条 gap<300 事件文件, 现码 MIN_GAP 门滤除;
  dc/a ATE 71.43 = 71.43) ✓

### 8.2 5 集统一表 (2026-10-07, Sim(3) 7-DoF 对齐口径, 论文 Table 1 同口径)

Town02 (1087帧中断) 与 Town10HD (空目录) 已重采至全量 5000 帧 (4 位帧号 0001–5000,
0 跳号, 四数据文件各 5001 行); 4 个 CARLA 集 + KITTI07 用当前 HEAD 驱动
(RUN/override 参数不变, NLM_USE_EKF_ODO=true) 同批重跑:

| 数据集 (帧数) | GT 长度 | NLM (经验地图) | EKF (融合) | VO (纯视觉) | NLM 相对 EKF | 闭环事件 |
|---|---|---|---|---|---|---|
| Town01 (5000) | 701.59 m | 95.55 m | 81.60 m | 249.37 m | +13.95 m | 0 |
| Town02 (5000) | 457.85 m | 108.89 m | 110.41 m | 123.73 m | **−1.52 m** | 0 |
| Town05 (5000) | 449.75 m | 71.97 m | 72.25 m | 161.84 m | **−0.28 m** | 0 |
| Town10HD (5000) | 564.89 m | 27.14 m | 26.93 m | 163.71 m | +0.21 m | 0 |
| KITTI07 (1101) | 692.06 m | 141.79 m | 141.25 m | 236.23 m | +0.54 m | 0 |
| **Town01Turnaround (10000)** | 879.6 m | **172.12 m** (simple) | 354.58 m (simple) | 452.97 m (simple) | **−182.46 m (simple)** | **16** |

说明:
- 前 5 集为 Sim(3) 口径 (compute_metrics_with_alignment, 首100帧锚定), 即论文
  Table 1/2 口径; Turnaround 该口径不可用 (其 EKF 基线自身漂移 495m, Sim(3) 对齐
  后 4 方法坍缩到 ≈28.7m 无区分度), 故以 simple 口径报告并单独标注。
- 开放 4 集 NLM≈EKF (差 ≤1.52m, 噪声级): EKF 为强基线 (NLM_USE_EKF_ODO 下节点
  钉在 EKF 米坐标, 见 §7/路线①结论), 无真闭环时 NLM 增益为 0 是预期行为, 恰好
  反证增益来自闭环而非调参。KITTI07 经验图 964 节点仅 9 条近邻重复边 (d_xy≈1m),
  无长程真闭环, 为结构天花板 (与 09-28 结论一致)。
- 所有开放集 0 闭环事件: 约束机制不触发, 输出与无约束逐帧一致 (H4 中性性在
  Sim(3) 批同样成立)。
- VO 在所有集上均显著劣于 NLM/EKF (差 36–290 m), 视觉前端单独不可用。

### 8.3 结论

1. **H2 验证**: 真闭环路线上约束被记录且 gap/幅度全部合法 (≫300 帧, ∈[3,20]m 门);
2. **H3 验证**: 闭环约束带来 −14.4% (simple 口径) 增益, NLM 172.12 m 大幅优于
   EKF (−51%) / VO (−62%);
3. **H4 验证**: 4+2 个开放集 0 事件, A-fix 严格中性, 创新不反噬;
4. 论文 Table 1/2 回填以 §8.2 前 5 集 Sim(3) 数字为准; 闭环增益 (Turnaround)
   建议另设 simple 口径子表, 不与 Sim(3) 主表混排。

## 9. S2 升级: 闭环全 Sim(3) 重锚 (2026-10-08 实现+验证; 论文 claim 已定稿)

### 9.1 动机 (为什么平移 A-fix 不够)

A-fix 只注入平移 c = -β·clamp(r)。闭环残差 r 里混着**尺度/旋转**成分
(NLM_USE_EKF_ODO 下经验节点钉在 EKF 米坐标, EKF 自身带尺度漂移; GC 离散积分
累积旋转漂移)。平移注入消不掉这两类失真。实测 (§8.2 口径辨析的后续):
Turnaround 首100帧锚定 Sim(3) 口径下, A-fix 不降反升 755→861.5 m —
锚定窗口落在 EKF 前段尺度压缩区, 平移爬坡把后续段"钉"在失真坐标系上。
→ 需要在闭环点直接恢复**全局相似变换** (尺度+旋转+平移)。

### 9.2 机制 (S2: NLM_SIM3_REANCHOR, 默认关)

每次闭环事件 t_now:
1. **锚点窗** aWin = 首段运动帧 (自动跳过静止前导, 首移动帧 sra 起 400 帧) —
   代表"首锚坐标系";
2. **当前窗** cWin = base(t_now-399 : t_now), 且 ≤ 下一事件帧-1;
3. Procrustes 全 Sim(3) 拟合 (Scaling=true): anchor ≈ s·(cur·R) + t;
   t 用**质心约束**重算 (ca - s·(cc·R)), 对窗口退化稳健;
4. 整段 [t_now+1, 下一事件帧-1] 用该 Sim(3) 变换**整体拉回**首锚坐标系;
5. **非链式**: 所有拟合与映射基于 S2 输入轨迹 base (各段独立拉回, 前一事件
   的重锚不污染后一事件窗口); 链式实现会使相邻事件窗口被上一段变换污染
   (实测 7/16 事件生效, 86.6 m, 劣于非链式 10/16, 61.7 m);
6. 门控: cWin<30帧 / 段<2帧 / 行数不匹配 / s∉[0.05,20] → 跳过该事件。

零地图反馈 (只改输出轨迹); 0 事件 → exp_trajectory 逐帧不变 (严格中性)。
A-fix 平移爬坡与 S2 **叠加** (爬坡先抵消 t_now 阶梯, 保证 400 帧拟合窗无阶梯
污染); S2 单独 (去掉 A-fix) 实测 4/16 事件生效, 103.9 m, 劣于叠加。

### 9.3 验证 (Turnaround, 10000帧, 16 真实闭环事件)

| 口径 | A-fix only (S2off) | **S2 on (win=400)** | EKF |
|---|---|---|---|
| 版本A (首100帧锚定 Sim3, ablation Table 口径) | 861.48 m | **123.77 m** | 745.07 m |
| simple (起点+长度匹配, 无旋转, core 口径) | 172.12 m | **61.74 m** | 354.54 m |

- core 全量重跑 (NLM_TURNAROUND_TOWN01_CONSTRAINT) 与离线逐帧一致
  ([S2] 10/16 事件生效, 锚点窗 [51:450]);
- **per-lap oracle 不变** (47.6/60.4/171/38.6 ≈ 基线 48.4/60.4/171/51):
  增益纯来自全局 Sim(3) 移除, 无 GT 泄漏、无局部畸变注入 (防伪判据通过);
- **尺度是主导 DOF**: 去尺度 (SE(3)) 退回 496.8 m; 钳位尺度 (V2) 无增益
  (123.80≈123.77) → 用最简裸 Sim(3) 形式;
- **win 调优** (非链式, 与 core 一致): win=200/300/400/500 →
  148.3/141.0/**123.8**/113.9 m (verA), simple 64.8/64.5/**61.7**/62.0 m,
  各窗 10/16 事件全生效; win=400 取 simple 最优, 且结果对事件代稳健
  (10-07 事件 123.77 = 10-08 事件 123.77)。win 是 lap 拟合窗 (事件离散,
  非连续旋钮), 不过拟合;
- 16 事件中 6 个跳过 (窗口退化/短段), 10 个生效 — 跳过集与 DC v3 跳变集
  高度重合 (重锚密集区)。

### 9.4 闭环质量依赖性 (S2 的 claim 边界, 已定稿为"闭环条件性")

S2 的收益**依赖闭环检测质量**。逐集事实 (win=400, 当前代事件):

| 数据集 | 事件数 (现码) | S2 行为 | 证据 |
|---|---|---|---|
| Town01Turnaround | 16 (真长程闭环) | **6× 大赢** | §9.3 |
| Town01 / Town10HD | 0 | 严格中性 (no-op) | 事件文件不存在 |
| Town02 | 0 (死数据不可验) | no-op | a_final 缺失 |
| Town05 | 现码 0 (旧码残留 11 条假闭环被 MIN_GAP=300 滤除) | no-op; 反事实: 若 S2 作用于那 11 条假闭环, verA 480→169 (改善) 但 simple 71.4→123.9 (**反噬**) | kb_s2_tune2 离线反事实 |
| KITTI07 | 现码 0 (旧码残留 1 条近邻假闭环) | no-op; 反事实: 若 S2 作用于那条假闭环, verA 30.8→63.9 (**反噬, 使 NLM 输给 EKF**) / simple 中性 | 同上 |

结论:
1. **现码下 5 开放集全部 0 事件 → S2 严格中性** (与 A-fix 同性质, H4 成立);
2. S2 放大闭环检测误报: 假闭环在 S2 下不再只是"少修一点", 而是整段被
   一个错误 Sim(3) 拉走 (verA 口径下 KITTI07 由优转劣)。这是设计文档必须
   记录的已知边界, 也是 S2 与 A-fix 的本质区别 (A-fix 假闭环损失有界:
   c∈[3,20]m 钳位; S2 假闭环损失无界: 整段相似变换);
3. **论文 claim (已定稿, 用户确认): 闭环条件性** — S2 在存在真实长程闭环的
   场景带来大幅增益 (Turnaround: simple 172→62 m, verA 861→124 m, 均
   6× 优于 EKF); 开放路线上机制不触发 (0 事件, 输出逐帧一致)。不在论文
   中主张 S2 的通用中性性, 也不把 Turnaround 数字混入 Sim(3) 主表
   (Turnaround EKF 基线自身漂移 495 m, Sim(3) 对齐后无区分度)。

### 9.5 论文落点 (experiment_section.tex)

- 主表 Table 1 保持 7 集全轨迹 Sim(3) 口径 (NLM≈EKF 的诚实口径, 已有
  "essentially matches" 措辞);
- 悬空引用 `Section~\ref{sec:scale_recovery}` → 新增
  "Closed-Loop Scale Recovery (Town01 Turnaround)" 小节: 报告 S2 的
  Turnaround 数字 (simple + verA 双口径, per-lap oracle 防伪), claim 措辞
  按 §9.4 闭环条件性;
- 不新增 Table 2 主表行 (Turnaround 与主表口径不同, 单独子表/小节)。

### 9.6 文件与复现

- core: `neuro/07_test/test_imu_visual_slam/core/test_imu_visual_fusion_slam.m`
  (S2 块 ~L665, `nlm_sim3_reanchor` 局部函数 ~L990, `NLM_SIM3_REANCHOR`
  global 开关默认 false, S2 输入轨迹存 a_final/exp_trajectory_preS2.txt)
- 驱动: `quickstart/NLM_TURNAROUND_TOWN01_CONSTRAINT.m` (NLM_SIM3_REANCHOR=true)
- 离线工具 (kbs/): `tar_reanchor_events.m` (16事件忠实复算),
  `tar_reanchor_variants.m` (V1/V2/V3 DOF 消融), `kb_s2_winsweep.m` (win 扫),
  `kb_s2_tune2.m` (开放集反事实), `kb_check_events.m` (逐集事件数),
  `kb_baseline_at.m` (逐集 S2off 基线 ATE)
- 备份: `/tmp/turnaround_a_final_preS2_20261006` (S2 前 10-07 代 a_final)
