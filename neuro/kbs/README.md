## 📊 Paper Figures 2-7: Drawing Method & Reproduction Guide
This document details the drawing logic, corresponding scripts, data sources, and reproduction steps for all core figures in the paper, fully aligned with the repository structure.

---

Figure 5: Ablation Study & Visual Template Growth
1. Figure Core Content
Left Subplot (Experience Map Growth Curve): EM node count growth across 6 datasets (Town01/Town02/Town10/MH01/MH03/KITTI07), key value: Town10 (51 nodes)
Right Subplot (Visual Template Growth Curve): VT count growth across 6 datasets (baseline: RatSLAM ~5 templates), key value: Town10 (195 templates)
2. Corresponding Repository Files
Drawing Script (Left): neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m
Drawing Script (Right): neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m
Generated PDF (Left): neuro/kbs/fig/em_growth_all_datasets.pdf
Generated PDF (Right): neuro/kbs/fig/vt_growth_all_datasets.pdf
Data Source: EM/VT count time-series data from 6 datasets
3. Reproduction Method
Dependencies: MATLAB R2020b+ (no additional toolboxes)
Run Command:
bash
运行
run('neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m')
run('neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m')
Output: PDF files saved to neuro/kbs/fig/
