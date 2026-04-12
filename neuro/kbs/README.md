## 📊 Paper Figures 2-7: Drawing Method & Reproduction Guide
This document details the drawing logic, corresponding scripts, data sources, and reproduction steps for all core figures in the paper, fully aligned with the repository structure.

---

### 📌 Figure 5: Ablation Study & Visual Template Growth
#### 1. Figure Core Content
- **Left Subplot (Ablation Study RMSE Comparison)**
  - Bar chart comparing 5 model configurations: Full, w/o IMU, w/o Exp Map, w/o Transformer, w/o Dual-stream
  - RMSE values for each configuration marked (Full: $145.5\ \text{m}$, w/o IMU: $315.3\ \text{m}$), with error bars to reflect result stability
- **Right Subplot (Visual Template Growth Curve)**
  - Visual template count growth curves for 6 datasets (Town01/Town02/Town10/MH01/MH03/KITTI07) with frame index
  - Comparison with RatSLAM baseline (~5 templates), Town10's 195 templates marked to reflect model's template accumulation capability

#### 2. Corresponding Repository Files
- Drawing Scripts: `neuro/kbs/fig/generate_ablation_unified.py`, `neuro/kbs/fig/generate_vt_growth.py`
- Generated PDFs: `neuro/kbs/fig/ablation_unified.pdf`, `neuro/kbs/fig/vt_growth_all_datasets.pdf`
- Data Source: Ablation experiment results, visual template count time-series data from each dataset

#### 3. Reproduction Method
- Dependencies: `matplotlib` + `seaborn` (plotting), `numpy` (template count and ablation data processing)
- Run Commands:
  ```bash
  python generate_ablation_unified.py
  python generate_vt_growth.py
  run('neuro/08_draw_fig_for_paper/02_EM_History/draw_em_history.m')
  run('neuro/08_draw_fig_for_paper/03_VT_History/draw_vt_history.m')

### 📌 Figure 6: 6-Dataset Performance Summary
#### 1. Figure Core Content
- **(a) RMSE Comparison Bar Chart**
  - RMSE comparison of NeuroLocMap/EKF/VO for 6 datasets (Town01/Town02/Town10/KITTI07/MH01/MH03)
- **(b) Improvement Bubble Chart**
  - Improvement rate relative to EKF, bubble size proportional to trajectory length, improvement percentage for each dataset marked (e.g., Town10: $58.5\%$)
- **(c) Success Rate Pie Chart**
  - Localization success rate $83\%$ (5/6 datasets successful), $17\%$ failure (corresponding to MH01)

#### 2. Corresponding Repository Files
- Drawing Script: `neuro/kbs/fig/generate_performance_summary.py`
- Generated PDF: `neuro/kbs/fig/performance_summary.pdf`
- Data Source: SLAM positioning results and RMSE statistics from 6 datasets

#### 3. Reproduction Method
- Dependencies: `matplotlib` (plotting), `pandas` (improvement rate calculation and summary data processing)
- Run Command:
  ```bash
  python generate_performance_summary.py
  run('neuro/08_draw_fig_for_paper/draw_performance_summary.m')

### 📌 Figure 7: KITTI-07 Input-Output Visualization
#### 1. Figure Core Content
- **Top-Left: RGB Images**
  - 4 key frames from KITTI-07 dataset (Frame 77/110/330/551), showing visual input
- **Bottom-Left: IMU Sensor Data**
  - Time-series curves of IMU acceleration (Accel) and angular velocity (GyroZ), with key frame positions marked
- **Top-Right: 3D Trajectory Comparison**
  - 3D comparison of Ground Truth and NeuroLocMap predicted trajectory, start (green) and end (red) marked
- **Bottom-Right: Experience Map Topology**
  - Experience map topology: 51 nodes, 8 loop closures, blue solid lines for sequential links, green dashed lines for loop closure edges, start/end marked

#### 2. Corresponding Repository Files
- Drawing Script: `neuro/kbs/fig/KITTI_07_manual_save_picture.m`
- Generated PDF: `neuro/kbs/fig/KITTI_07_input_output_visualization.pdf`
- Data Source: RGB images, IMU data, positioning trajectory, experience map topology data from KITTI-07 dataset

#### 3. Reproduction Method
- Dependencies: `cv2` (OpenCV, image reading), `matplotlib.mplot3d` (3D trajectory), `networkx` (topology drawing), `matplotlib` (IMU curve plotting)
- Run Command (MATLAB):
  ```matlab
  run KITTI_07_manual_save_picture.m
  run('neuro/08_draw_fig_for_paper/01_EM_OM/SynPerData/draw_3d_odomap_synperdata.m')
