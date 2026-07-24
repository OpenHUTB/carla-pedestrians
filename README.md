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

### Prerequisites

- **Python 3.8+**
- **CARLA Simulator 0.9.8+** (for data collection)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/dream1112221/carla-pedestrians.git
cd carla-pedestrians
```

2. Install Python dependencies:
```bash
pip install -r neuro/requirements.txt
```

3. Install CARLA Python API:
```bash
pip install <CARLA_DIR>\PythonAPI\carla\dist\carla-*.whl
```

### One-Click Run

```bash
# 1. Start CARLA server (in a separate terminal)
cd <CARLA_DIR>
CarlaUE4.exe -RenderOffScreen -quality-level=Low

# 2. Run the full pipeline
cd carla-pedestrians
python main.py
```

This will automatically:
- Collect 5000 frames of IMU + Visual data from CARLA Town01
- Run ablation study (Pure IMU / Pure VO / EKF Fusion)
- Generate comparison charts

### Available Options

| Command | Description |
|---------|-------------|
| `python main.py` | Full pipeline: collect + evaluate |
| `python main.py --skip-collect` | Skip collection, only evaluate existing data |
| `python main.py --collect-only` | Only collect data, skip evaluation |
| `python main.py --host 192.168.1.1` | Connect to remote CARLA server |

### Core Components (Python)

1. **Data Collection**: `neuro/00_collect_data/IMU_Vision_Fusion_EKF.py`
2. **One-Click Entry**: `neuro/main.py`
3. **Ablation Study**: `neuro/07_test/run_ablation.py`
4. **Visual Odometry**: `neuro/00_collect_data/visual_odometry_opencv.py`

## 📊 Experiments

### Run Ablation Study

```bash
cd carla-pedestrians
python main.py --skip-collect    # Use existing data
```

Results are saved to `neuro/data/`:
- `ablation_comparison.png` — comparison chart
- `ablation_results.json` — detailed metrics

## 📝 Paper

**Title**: NeuroSLAM: A Biologically-Inspired IMU-Visual Fusion SLAM System

**Status**: Under review (Knowledge-Based Systems)

**LaTeX Source**: `neuro/kbs/kbs_1/NeuroSLAM_KBS.tex`

**PDF**: `neuro/kbs/kbs_1/NeuroSLAM_KBS.pdf`


## 🔧 Requirements

### Python
- Python 3.8+
- numpy >= 1.19.0
- opencv-python >= 4.5.0
- scipy >= 1.5.0
- matplotlib >= 3.3.0
- pandas >= 1.1.0

### CARLA
- CARLA Simulator 0.9.8+
- CARLA Python API (`carla` wheel)

### MATLAB (Legacy)
- MATLAB R2020b or later
- Computer Vision Toolbox
- Image Processing Toolbox
- Statistics and Machine Learning Toolbox

See `neuro/requirements.txt` for complete Python dependencies.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

