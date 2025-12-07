# 🧠 NeuroSLAM: Brain-Inspired 3D SLAM System

**Version 2.0** | IMU-Visual Fusion + HART+CORnet Feature Extraction

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.7+-green.svg)](https://www.python.org/)
[![CARLA](https://img.shields.io/badge/CARLA-0.9.15-orange.svg)](https://carla.org/)
[![License](https://img.shields.io/badge/License-GPL--3.0-red.svg)](LICENSE)

A biologically-inspired Simultaneous Localization and Mapping (SLAM) system that mimics the spatial cognition mechanisms of the rat hippocampus, enhanced with state-of-the-art computer vision techniques.

---

## 🌟 Key Features

- 🧠 **Bio-Inspired Architecture**
  - Grid Cell network for 3D position encoding (61×61×51)
  - Head Direction Cell network for orientation estimation
  - Experience Map for topological-metric mapping

- 🎯 **Multi-Sensor Fusion**
  - RGB camera (640×480, 20Hz)
  - IMU (accelerometer + gyroscope, 100Hz)
  - Complementary filter fusion for robust odometry

- 🚀 **Advanced Feature Extraction**
  - **HART+CORnet**: Hierarchical brain-like visual features (V1→V2→V4→IT)
  - **Simplified Enhanced**: 71 FPS (5.92× speedup) with strong robustness
  - Spatial attention mechanism and LSTM temporal modeling

- 📊 **Comprehensive Evaluation**
  - ATE/RPE metrics
  - Ground truth comparison
  - 7 publication-quality figures

- 🔄 **Real-time Performance**
  - 30-70 FPS processing speed
  - Supports trajectories >1.5 km
  - Town01: 95.3% trajectory completeness, 152.87m RMSE

---

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Dataset](#dataset)
- [Usage](#usage)
- [Documentation](#documentation)
- [Performance](#performance)
- [Citation](#citation)
- [License](#license)

---

## 🔧 Installation

### Prerequisites

- **MATLAB** R2020b or later
- **Python** 3.7+ (for data collection)
- **CARLA Simulator** 0.9.13-0.9.15 (optional, for data collection)

### Python Dependencies

```bash
cd neuro/00_collect_data
pip install -r requirements.txt
```

### MATLAB Setup

```matlab
% Open MATLAB and navigate to the neuro directory
cd /path/to/neuro

% Add all paths
addpath(genpath('.'));
savepath;
```

---

## 🚀 Quick Start

### Option 1: Fast Test (30 seconds)

Test the feature extractor with built-in images:

```matlab
cd neuro
quick_test_integration
```

**Expected output:**
- ✓ Feature extraction: 71 FPS
- ✓ VT count: 5-6
- ✓ Template reuse rate: 75%

### Option 2: Full SLAM on Town01 (5 minutes)

Run complete SLAM with HART+CORnet features:

```matlab
cd neuro/07_test/test_imu_visual_slam
test_imu_visual_slam_hart_cornet
```

**Expected results (5000 frames, ~1.8km):**
- VT count: 5
- Experience nodes: 185
- RMSE: 152.87m
- Trajectory completeness: 95.3%

---

## 📁 Dataset

### Data Structure

```
neuro/data/01_NeuroSLAM_Datasets/
├── Town01Data_IMU_Fusion/
│   ├── 0001.png - 5000.png      # RGB image sequence
│   ├── aligned_imu.txt           # IMU data
│   ├── fusion_pose.txt           # EKF fusion poses
│   └── ground_truth.txt          # Ground truth trajectory
└── Town10Data_IMU_Fusion/
    └── (same structure)
```

### Acquire Datasets

**Note:** Datasets are **NOT included** in this repository due to size (~5GB).

#### Method 1: Collect Your Own (Recommended)

```bash
# 1. Start CARLA server
cd /path/to/carla-0.9.15
./CarlaUE4.sh

# 2. Run data collection script
cd neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py
```

Configure in the script:
- `TOWN = 'Town01'` or `'Town10'`
- `DURATION = 250` seconds
- Data auto-saved to `neuro/data/01_NeuroSLAM_Datasets/`

#### Method 2: Download Pre-collected Data

If available, download from:
- Town01: [Link TBD]
- Town10: [Link TBD]

See `data/01_NeuroSLAM_Datasets/README.md` for details.

---

## 📖 Usage

### Switch Between Feature Extractors

```matlab
% In test_imu_visual_slam_hart_cornet.m (line 15)

USE_HART_CORNET = true;   % HART+CORnet (best trajectory completeness)
USE_HART_CORNET = false;  % Simplified Enhanced (best speed & RMSE)
```

### Tune Parameters

```matlab
% VT matching threshold (line 106)
VT_MATCH_THRESHOLD = 0.07;  % Lower = more VTs, higher precision

% Experience map threshold
DELTA_EXP_GC_HDC_THRESHOLD = 15;  % Node creation threshold

% IMU-Visual fusion weights (in imu_aided_visual_odometry.m)
ALPHA_YAW = 0.7;     % 70% IMU for yaw
ALPHA_TRANS = 0.3;   % 30% IMU for translation
```

### Run on Custom Dataset

```matlab
% Modify data path in your script
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
datasetPath = fullfile(neuroRoot, 'data/MyDataset');

% Run SLAM
cd neuro/07_test/test_imu_visual_slam
test_imu_visual_slam_hart_cornet
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| **`COMPLETE_SYSTEM_GUIDE.md`** | 📘 Complete technical guide (60+ KB) |
| **`QUICK_START_VISUAL_GUIDE.md`** | 🚀 Visual quick start guide |
| **`HART_CORNET_SUMMARY.md`** | 🧠 HART+CORnet feature extractor |
| **`PATH_USAGE_GUIDE.md`** | 📂 Relative path usage guide |
| **`START_HERE.md`** | ⭐ Original quick start |

**Start with:** `QUICK_START_VISUAL_GUIDE.md` → `COMPLETE_SYSTEM_GUIDE.md`

---

## 📊 Performance Benchmarks

### Town01 (5000 frames, ~1.8 km)

| Method | VTs | Nodes | RMSE | Trajectory % | Speed |
|--------|-----|-------|------|--------------|-------|
| **Original** | 5 | 186 | 129.39m | 95.3% | ~25 FPS |
| **Simplified Enhanced** | 1365 | 2022 | **142.57m** ✅ | 38% | **71 FPS** ✅ |
| **HART+CORnet** | 5 | 185 | 152.87m | **95.3%** ✅ | ~30 FPS |

**Recommendation:**
- **Speed priority** → Simplified Enhanced (71 FPS)
- **Long trajectory** → HART+CORnet (95% completeness)
- **Local precision** → Simplified Enhanced (143m RMSE)

### Town10 (5000 frames, ~1.6 km)

| Method | VTs | Nodes | RMSE | Trajectory % | Drift Rate |
|--------|-----|-------|------|--------------|------------|
| **HART+CORnet** | 5 | 151 | 229.95m | 87.9% | 24.4% |

*Note: Town10 can be improved by lowering VT threshold to 0.05*

---

## 🏗️ System Architecture

```
Input (RGB + IMU)
    ↓
IMU-Visual Fusion Odometry
    ↓
    ├─→ Visual Template Matching (HART+CORnet)
    │   └─→ VT ID
    ↓
    ├─→ Head Direction Cell Network
    │   └─→ (yaw, height)
    ↓
    ├─→ 3D Grid Cell Network
    │   └─→ (x, y, z)
    ↓
Experience Map (Topological + Metric)
    ├─→ Node creation
    ├─→ Graph optimization
    └─→ Trajectory output
        ↓
Evaluation & Visualization
    └─→ ATE/RPE, figures
```

---

## 🔬 Scientific Background

### Neuroscience Inspiration

- **Grid Cells**: Nobel Prize 2014, spatial encoding in entorhinal cortex
- **Place Cells**: Hippocampal location recognition
- **Head Direction Cells**: Orientation sensing
- **Visual Cortex**: V1→V2→V4→IT hierarchical processing

### Key References

1. **NeuroSLAM**:
   ```
   Yu, F., Shang, J., Hu, Y., & Milford, M. (2019).
   NeuroSLAM: a brain-inspired SLAM system for 3D environments.
   Biological Cybernetics, 113(5-6), 515-545.
   ```

2. **HART (Hierarchical Attentive Recurrent Tracking)**:
   ```
   Kosiorek, A. R., Bewley, A., & Posner, I. (2017).
   Hierarchical Attentive Recurrent Tracking. NIPS 2017.
   ```

3. **CORnet (Brain-Like Object Recognition)**:
   ```
   Kubilius, J., et al. (2018).
   Brain-like Object Recognition with High-Performing Shallow Recurrent ANNs.
   NeurIPS 2018.
   ```

---

## 📁 Project Structure

```
neuro/
├── 00_collect_data/              # CARLA data collection
├── 01_conjunctive_pose_cells_network/  # Grid Cell + HDC
├── 02_multilayered_experience_map/      # Experience map
├── 03_visual_odometry/           # Visual odometry
├── 04_visual_template/           # Feature extraction (HART+CORnet)
├── 05_tookit/                    # Utilities
├── 06_main/                      # Main entry points
├── 07_test/                      # Test scripts
├── 09_vestibular/                # IMU fusion
├── data/                         # Datasets (not in repo)
├── latex/                        # LaTeX documents
└── referance/                    # References
```

---

## 🐛 Troubleshooting

### "Undefined function or variable"

```matlab
cd /path/to/neuro
addpath(genpath('.'));
savepath;
```

### "Data files not found"

See `data/01_NeuroSLAM_Datasets/README.md` for data acquisition.

### VT count abnormal

```matlab
% Too many VTs (>2000): increase threshold
VT_MATCH_THRESHOLD = 0.10;

% Too few VTs (<5): decrease threshold
VT_MATCH_THRESHOLD = 0.05;
```

More troubleshooting: See `COMPLETE_SYSTEM_GUIDE.md` Section 9.

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Original NeuroSLAM**: Fangwen Yu, Jianga Shang, Youjian Hu, Michael Milford
- **OpenRatSLAM**: David Ball, Gordon Wyeth, Michael Milford
- **CARLA Simulator**: CARLA Team
- **HART**: Adam Kosiorek, Alex Bewley, Ingmar Posner
- **CORnet**: Jonas Kubilius, Martin Schrimpf, Daniel Yamins

---

## 📧 Contact

- **Issues**: [GitHub Issues](https://github.com/your-username/neuro/issues)
- **Email**: [your-email@example.com]
- **Website**: [Project Homepage]

---

## 🌟 Star History

If you find this project helpful, please consider giving it a ⭐!

---

**Last Updated**: 2025-12-07  
**Version**: 2.0 (IMU-Visual Fusion + HART+CORnet)  
**Status**: ✅ Stable and Ready for Use
