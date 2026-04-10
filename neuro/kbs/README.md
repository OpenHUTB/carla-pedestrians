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
