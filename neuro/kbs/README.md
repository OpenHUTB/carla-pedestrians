## 📊 Paper Figures 2-7: Drawing Method & Reproduction Guide
This document details the drawing logic, corresponding scripts, data sources, and reproduction steps for all core figures in the paper, fully aligned with the repository structure.

---

📌 Figure 5: Ablation Study & Visual Template Growth
1. Figure Core Content
Experience Map Growth Curve (Left)
Curve of experience map (EM) node count growth with frame index across 6 datasets (Town01/Town02/Town10/MH01/MH03/KITTI07)
Key node count value marked: Town10 (
51
 nodes), reflecting map construction efficiency
Visual Template Growth Curve (Right)
Visual template (VT) count growth curves for 6 datasets, comparing with RatSLAM baseline (
∼5
 templates)
Key template count value marked: Town10 (
195
 templates), highlighting template accumulation capability
2. Corresponding Repository Files
Drawing Scripts:
Left Subplot: neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m
Right Subplot: neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m
Generated PDFs:
Left Subplot: neuro/kbs/fig/em_growth_all_datasets.pdf
Right Subplot: neuro/kbs/fig/vt_growth_all_datasets.pdf
Data Source: EM node count & VT count time-series data from 6 datasets
3. Reproduction Method
Dependencies: MATLAB R2020b+ (no additional toolboxes required)
Run Command:
bash
运行
# Run in MATLAB Command Window
run('neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m')
run('neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m')
