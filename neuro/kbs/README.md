## 📊 Paper Figures 2-7: Drawing Method & Reproduction Guide
This document details the drawing logic, corresponding scripts, data sources, and reproduction steps for all core figures in the paper, fully aligned with the repository structure.

---

🎯 Figure 5: Ablation Study & Visual Template Growth
1. Figure Core Content
Left Subplot：Experience Map Growth Curve（EM 节点增长曲线）
Right Subplot：Visual Template Growth Curve（VT 模板增长曲线）
2. Corresponding Repository Files
bash
运行
# Left Subplot (EM Growth)
Script: neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m
Output: neuro/kbs/fig/em_growth_all_datasets.pdf

# Right Subplot (VT Growth)
Script: neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m
Output: neuro/kbs/fig/vt_growth_all_datasets.pdf
3. Reproduction Method
bash
运行
# Dependencies
MATLAB R2020b+

# Run in MATLAB Command Window
run('neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m')
run('neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m')

# Output: PDFs auto-saved to neuro/kbs/fig/
