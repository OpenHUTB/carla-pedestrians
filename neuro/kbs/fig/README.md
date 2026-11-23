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
- 作图源文件 `.pptx`（5 个，合计约 21.8 MB）已移至 `docs/src/`，并在 `.gitignore` 中加了 `neuro/kbs/fig/**/*.pptx` 防回流规则；重建 PDF 时从那里取。

## 重建
结果类 4 张由 `../regen_figures.py` 生成（需 `neuro/data/` 下的轨迹）；
方法类 3 张与 `1_abstract.pdf` 为 MATLAB/PPT 导出，仓库内不含重建脚本。
