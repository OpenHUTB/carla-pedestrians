# `fig/` 资产说明

本目录既有论文实际编译的 7 张图，也有作图源文件（`.pptx`）和历史草稿图。
`NeuroSLAM_KBS.tex` 只引用下面第一张表的 7 个 PDF；其余文件不参与编译。

## 参与编译（7 张）

| 图标签 | 文件 | 定义于 |
|---|---|---|
| `fig:3D_grid_cell_network` | `fig/3d_grid_cell_network.pdf` | `sec_method_expanded.tex` |
| `fig:ablation_and_vt` | `fig/ablation_unified.pdf` | `experiment_section.tex` |
| `fig:imu_visual_fusion` | `fig/imu_visual_complementary_fusion.pdf` | `sec_method_expanded.tex` |
| `fig:kitti_input_output` | `fig/KITTI_07_input_output_visualization.pdf` | `experiment_section.tex` |
| `fig:performance_summary` | `fig/performance_summary.pdf` | `experiment_section.tex` |
| `fig:representative_performance` | `fig/representative_performance.pdf` | `experiment_section.tex` |
| `fig:system_overview` | `fig/2_framework.pdf` | `sec_method_expanded.tex` |

## 不参与编译

- `1_abstract.pdf`（0.8 MB）——图形摘要候选，投稿系统单独上传，正文未排版引用；
- `ekf_curve_KITTI07 / MH_03 / Town01 / Town02 / Town03 / Town05 / Town07 / Town10HD.pdf`
  与 `vt_growth_all_datasets.pdf`——早期草稿的逐集曲线，当前稿已不再使用（Town01/02 也不在主表）；
- 7 个 `.pptx`（合计约 21.8 MB，含 `other/` 下 5 个）——作图源文件，重建 PDF 时才需要。

## 非 PDF 资产（18 个，均不被 `\includegraphics` 引用）

| 类别 | 文件 | 说明 |
|---|---|---|
| 作图源 | 7 个 `.pptx`（含 `other/` 下 5 个） | 结果/方法图的原稿，重建时才需要，合计约 21.8 MB |
| EPS 备份 | `ablation_unified.eps`、`performance_summary.eps` | 出版社排版阶段可能要求 EPS，保留 |
| PNG 备份 | `ekf_curve_{Town01,Town02,Town05,Town10HD}.png`、`other/pipeline.png` | 草稿期位图 |
| 照片 | `HaidongWang.jpg`、`PengfeiXiao.jpg` | 正文未使用；`other/` 下那份重复副本已删除 |

## 重建
结果类 4 张由 `../regen_figures.py` 生成（需 `neuro/data/` 下的轨迹）；
方法类 3 张与 `1_abstract.pdf` 由本目录的 `.pptx` 源文件导出；`../src/` 下的
九个绘图脚本是早期草稿，输出为 `bio_nav_architecture` 等，未进入本稿。
