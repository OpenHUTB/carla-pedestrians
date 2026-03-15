# NeuroSLAM: IMU-Visual Fusion SLAM System

A biologically-inspired SLAM system with IMU-visual fusion, featuring HART+Transformer visual template matching and 3D grid cell network for spatial representation.

## 🎯 Key Features

- **IMU-Visual Fusion**: Complementary filter-based sensor fusion for robust odometry
- **HART+Transformer**: Hierarchical Attentive Recurrent Tracking with Transformer for visual template matching
- **3D Grid Cell Network**: Biologically-inspired spatial representation
- **Multi-layer Experience Map**: Topological mapping with loop closure detection
- **Multi-dataset Validation**: Tested on CARLA, KITTI, and EuRoC datasets

## 📁 Project Structure

```
neuro/
├── 00_collect_data/          # Data collection and preprocessing
├── 01_conjunctive_pose_cells_network/  # 3D grid cell network
├── 02_multilayered_experience_map/     # Experience map implementation
├── 03_visual_odometry/       # Visual odometry module
├── 04_visual_template/       # Visual template matching (HART+Transformer)
├── 05_tookit/                # Utility functions and tools
├── 06_main/                  # Main SLAM system
├── 07_test/                  # Testing and experiments
├── 08_draw_fig_for_paper/    # Paper figure generation
├── 09_vestibular/            # IMU processing and fusion
└── kbs/kbs_1/                # Paper draft (KBS submission)
```

## 📊 Datasets

### Public Datasets Used

1. **CARLA Simulator Datasets**
   - Town01, Town02, Town10 scenarios
   - RGB images + IMU data + Ground truth
   - Download: [CARLA Official Website](https://carla.org/)

2. **KITTI Odometry Dataset**
   - Sequence 07 (outdoor driving)
   - Stereo images + IMU + GPS ground truth
   - Download: [KITTI Vision Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

3. **EuRoC MAV Dataset**
   - MH_01_easy, MH_03_medium (indoor MAV)
   - Stereo images + IMU + Vicon ground truth
   - Download: [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

### Our Processed Datasets

We provide pre-processed datasets with aligned IMU-visual data:
- **Format**: MATLAB `.mat` files with synchronized timestamps
- **Structure**: RGB images, IMU measurements, ground truth trajectories
- **Download**: [Coming soon - will be hosted on cloud storage]

## 🚀 Quick Start

See [START_HERE.md](neuro/START_HERE.md) for detailed instructions.

### Prerequisites

```matlab
MATLAB R2020b or later
Computer Vision Toolbox
Image Processing Toolbox
```

```python
Python 3.8+
numpy
opencv-python
scipy
```

### Core Components

1. **Visual Template Matching**: `neuro/04_visual_template/visual_template.m`
2. **Main SLAM System**: `neuro/06_main/main.m`
3. **IMU-Visual Fusion**: `neuro/09_vestibular/imu_aided_visual_odometry.m`
4. **Baseline Comparison**: `neuro/07_test/07_test/test_imu_visual_slam/core/test_imu_visual_fusion_slam2.m`

### Installation

1. Clone the repository:
```bash
git clone https://github.com/dream1112221/carla-pedestrians.git
cd carla-pedestrians
```

2. Install Python dependencies:
```bash
cd neuro/00_collect_data
pip install -r requirements.txt
```

3. Download datasets (see Datasets section above)

4. Configure data paths in MATLAB:
```matlab
cd neuro/06_main
% Edit config_neuro_features.m to set your data paths
```

## 📊 Experiments

### Run Multi-Dataset Comparison

```matlab
cd neuro/07_test/07_test/test_imu_visual_slam/quickstart
RUN_ALL_DATASETS_COMPARISON
```

## 📝 Paper

**Title**: NeuroSLAM: A Biologically-Inspired IMU-Visual Fusion SLAM System

**Status**: Under review (Knowledge-Based Systems)

**LaTeX Source**: `neuro/kbs/kbs_1/NeuroSLAM_KBS.tex`

**PDF**: `neuro/kbs/kbs_1/NeuroSLAM_KBS.pdf`


## 🔧 Requirements

### MATLAB
- MATLAB R2020b or later
- Computer Vision Toolbox
- Image Processing Toolbox
- Statistics and Machine Learning Toolbox

### Python
- Python 3.8+
- numpy >= 1.19.0
- opencv-python >= 4.5.0
- scipy >= 1.5.0
- matplotlib >= 3.3.0

See `neuro/00_collect_data/requirements.txt` for complete Python dependencies.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

