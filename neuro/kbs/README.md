## 📊 Paper Figures 2-7: Drawing Method & Reproduction Guide
This document details the drawing logic, corresponding scripts, data sources, and reproduction steps for all core figures in the paper, fully aligned with the repository structure.

---

### 📌 Figure 2: Complementary IMU-Visual Fusion Architecture
#### 1. Figure Core Content
- **Left Subplot (Frequency Response)**
  - Blue curve: Low-pass filter amplitude-frequency response of the visual system (low-frequency retention, high-frequency attenuation)
  - Orange curve: High-pass filter amplitude-frequency response of the IMU (vestibular system) (high-frequency retention, low-frequency attenuation)
  - Cutoff frequency $\omega_c=10\ \text{rad/s}$, amplitude $R=0.707$ (-3dB point) marked as the core parameter of the complementary filter
- **Middle Subplot (Time-Domain Fusion)**
  - Black dashed line: Ground Truth (GT) velocity
  - Blue curve: Raw visual velocity (stable at low frequencies, slow response)
  - Yellow curve: Raw IMU velocity (sensitive at high frequencies, with drift)
  - Green curve: Fused velocity (combines advantages of both, drift-free, fast response)
  - Error metrics before/after fusion ($X\ 3.52 \to Y\ 1.74493$) marked to verify fusion effect
- **Right Subplot (Biological Analogy)**
  - Visual system (eyes) corresponds to low-pass filtering, vestibular system (ears) corresponds to high-pass filtering
  - CNS (vestibular nuclei + cerebellum) completes fusion to output robust self-motion estimation
  - Fusion formula $v^f = H_{LP} \cdot v^{vis} + H_{HP} \cdot v^{imu}$ and core advantages (drift-free, fast response) marked

#### 2. Corresponding Repository Files
- Drawing Script: `neuro/kbs/fig/generate_imu_visual_fusion.py`
- Generated PDF: `neuro/kbs/fig/imu_visual_complementary_fusion.pdf`
- Data Source: Synchronized visual/IMU sensor data collected from CARLA simulation environment

#### 3. Reproduction Method
- Dependencies: `scipy.signal` (filter design), `matplotlib` + `numpy` (plotting), `matplotlib.patches` (block diagram drawing)
- Run Command:
  ```bash
  python generate_imu_visual_fusion.py
Output: PDF file imu_visual_complementary_fusion.pdf

### 📌 Figure 3: 3D Grid Cell Network & 4-DoF Encoding
#### 1. Figure Core Content
- **Left Subplot (3D Grid Cell Activity)**
  - 3D layered activity distribution of $61\times61\times61$ grid cells, with color scales distinguishing activity intensity at different depths, corresponding to distributed encoding of spatial positions
  - Grid cell scale ($61\times61\times61\ \text{neurons}$) and head direction cell association marked
- **Middle Subplot (Simple Cubic vs FCC Lattice)**
  - Comparison of simple cubic and FCC (face-centered cubic) lattice structures, with FCC as the optimal spatial topology for grid cells
  - Lattice coordinate transformation relation $m(x=0) - z/2 \to 2/a$ marked
- **Right Subplot (4-DoF Pose Encoding)**
  - 3D grid cells encode 3D position, head direction cells (Hdc) encode 1D heading, concatenated as $[g_{xyz}; h_{\psi}]$
  - Core decoding formula marked, final output of 4-DoF (3D position + 1D heading) pose representation

#### 2. Corresponding Repository Files
- Drawing Script: `neuro/kbs/fig/generate_fig_3d_grid_cell.py`
- Generated PDF: `neuro/kbs/fig/3d_grid_cell_fcc_lattice.pdf`
- Data Source: Grid cell activity field matrix output by the model, FCC lattice coordinate data

#### 3. Reproduction Method
- Dependencies: `matplotlib.mplot3d` (3D visualization), `numpy` (FCC lattice coordinate calculation), `matplotlib.patches` (block diagram drawing)
- Run Command:
  ```bash
  python generate_fig_3d_grid_cell.py
Output: PDF file 3d_grid_cell_fcc_lattice.pdf
