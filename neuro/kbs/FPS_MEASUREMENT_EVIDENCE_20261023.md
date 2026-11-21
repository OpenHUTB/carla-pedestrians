# 实测计时证据（2026-10-23，本机 i9-14900HX / 32GB，全 CPU）

论文 runtime 表与 FPS 数字的原实测依据。全部为 `matlab -batch` 全量跑（含收尾评估与绘图），无 profiler 干净计时（tic/toc 包裹），配置复刻各自 P2FT94_* 驱动（τ=0.94/EKF-odo/S2 关），结果写独立审计目录不碰论文结果。

## 端到端墙钟（全流水线，含评估绘图）

| 场景 | 帧数 | 墙钟(s) | FPS | 每帧(ms) |
|---|---|---|---|---|
| Town05 | 5001 | 66.16 | 75.6 | 13.2 |
| Town03 | 5002 | 68.08 | 73.5 | 13.6 |
| Town07 | 5001 | 73.91 | 67.7 | 14.8 |
| Town10HD | 5001 | 74.59 | 67.1 | 14.9 |
| KITTI07 | 1101 | 28.20 | 39.0 | 25.6 |

四镇平均 14.1 ms/帧（70.8 FPS）。KITTI07 慢因：964 模板最稠密地图，模板匹配随地图增长。

## 逐模块剖析（MATLAB profiler，KITTI07 250 帧子集）

profiler 开销使绝对值偏高（剖析口径 34.4 ms/帧 vs 墙钟 25.6），仅用占比：

| 模块 | ms/帧(剖析) | 占比 |
|---|---|---|
| 网格细胞更新 gc_iteration | 16.78 | 49% |
| 腹侧模板提取+匹配 visual_template_neuro_matlab_only | 6.88 | 20% |
| 图像读入 imread+readpng | 4.60 | (含下行) |
| CLAHE 增强 adapthisteq | 2.92 | 图像输入合计 22% |
| 背侧扫描线 VO visual_odometry | 1.48 | 4% |
| HDC 更新 yaw_height_hdc_iteration | 0.89 | 3% |
| 经验地图更新 exp_map_iteration+get_gc_xyz | 0.82 | 2% |
| 前庭-视觉融合 imu_aided_visual_odometry | 0.06 | <0.2% |

脚本：/tmp/FPS_CLEAN_RUN.m（墙钟）、/tmp/FPS_PROBE_RUN.m（剖析）；结果 /tmp/fps_towns.txt、/tmp/fps_probe_modules.txt、/tmp/prof_data.mat。

## 一致性取证

- 审计跑（10-23）与论文冻结跑（10-14）的逐帧轨迹逐点偏差 = 0（imu_aided 1320.29m / pure VO 362.38m / exp 1334.65m 全等），流水线结果与输入 mtime 无关，数据完整性成立。
- 性能报告里"EKF前端 1320.29m 误差 90.78%"是报告模板按轨迹长度算的口径，与论文 Sim(3) RMSE 22.09m 口径不同，不是数据问题。
- 代码全 CPU：全仓库无 gpuArray/gpuDevice（errorIfgpuArray 只是输入校验）。

## 论文替换口径

- 摘要/贡献/结论/相关工作/方法/实验章 + highlights/cover letter：31 FPS → 39--76 FPS（measured, no GPU）
- 实验章实现细节：i5-12450H/16GB/RTX4050+"GPU加速" → i9-14900HX/32GB/RTX5070+"全 CPU 无 GPU"
- runtime 表：旧无出处 32.1ms/31.2Hz 表 → profiler 实测占比表（KITTI07 250 帧）
- 删除无出处断言：~2GB 内存、CPU 利用率 45-60%、功耗 15-20W vs 50-80W
- 遗留：理论复杂度段仍有 30×30×10=9000 与描述子 4096 维两处旧常数（审计 B 类，归学生重写主线处理）
