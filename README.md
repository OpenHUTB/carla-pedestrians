# NeuroLocMap: Brain-Inspired 4-DoF Localization-Mapping

Code and evaluation scripts for the manuscript

> **Brain-Inspired 4-DoF Localization-Mapping: Complementary Vestibular-Visual Fusion with Spatial Cells**
> Haidong Wang, Rongbo Yang — Hunan University of Technology and Business
> *Submitted to Knowledge-Based Systems* (2026).

The system couples dual-stream visual processing with IMU fusion on a **fixed 10-state EKF core** (frequency-domain complementarity instantiated as gated observation branches), represents space through **3D grid cells** and **multilayered head-direction cells**, and stores experience in a **multilayer experience map** that provides loop-closure re-anchoring. Two lightweight learned components — a scale-only head over a frozen monocular depth model, and a learned-embedding loop-closure line — are trained once on simulation and deployed without per-domain retraining.

The localization pipeline runs **on CPU only**; no GPU is used in the reported results.

> **Naming.** The manuscript calls the framework *NeuroLocMap*; the code, file names and the LaTeX sources keep the historical *NeuroSLAM* naming (`NeuroSLAM_KBS.tex`, `neuro/00_...`). The two names refer to the same system.

---

## 1. Quick start

```bash
git clone https://github.com/OpenHUTB/carla-pedestrians.git
cd carla-pedestrians
pip install -r neuro/requirements.txt

# 1) check that the processed sequences are in place
python3 neuro/kbs/check_datasets.py

# 2) recompute the paper's main table (5 scenarios, both alignment calibers)
python3 neuro/kbs/compute_final_metrics.py      # -> neuro/kbs/final_metrics.json

# 3) regenerate the figures used in the paper
python3 neuro/kbs/regen_figures.py
```

Prerequisites: Python 3.9+, NumPy/SciPy/OpenCV/Pandas/Matplotlib (see `neuro/requirements.txt`), PyTorch and `transformers` only for the learned components (see `neuro/requirements-learned.txt`), MATLAB only for the legacy `.m` re-anchoring scripts. A mirror of this repository is available at `https://github.com/82540295/carla-pedestrians`.

**Reproducing every number in the paper** — table by table, script by script — is documented in [`neuro/kbs/REPRODUCE.md`](neuro/kbs/REPRODUCE.md).

---

## 2. Repository layout

| Path | What it is |
|---|---|
| `neuro/00_collect_data/` | CARLA data collection, `IMU_Vision_Fusion_EKF.py` (the 10-state fusion core), KITTI/EuRoC adapters |
| `neuro/01_conjunctive_pose_cells_network/` | 3D grid-cell continuous attractor network |
| `neuro/02_multilayered_experience_map/` | Experience map, loop-closure detection and re-anchoring |
| `neuro/03_visual_odometry/` | Monocular VO front-end |
| `neuro/04_visual_template/` | Visual templates, HART+Transformer matching |
| `neuro/09_vestibular/` | IMU processing and bias estimation |
| `neuro/kbs/` | **Everything that produces the paper**: metrics, gate toolchain, scale/loop heads, figures, manuscript sources (`*.tex`), trained weights (`*.pt`), per-sequence records (`scale_p1/*.npz`, `*.json`) |
| `bip/`, `pedestrians-scenarios/`, `pedestrians-video-2-carla/`, `carla-common/`, `walker/`, `alphapose/`, `openpose/` | Mirrored third-party projects used for scenario generation — **not** part of the localization method, and licensed separately (see `NOTICE`) |

---

## 3. Data

* **CARLA (Town05 / Town03 / Town07 / Town10HD)** — collected with CARLA 0.9.16 using the code in `neuro/00_collect_data/` (20 Hz, commanded speed 12 km/h, 20 autopilot vehicles + 20 pedestrians). The processed sequences are released as a **minimal asset bundle** (~45 MB, `NLM_Repro_Assets_*`): per sequence the ground truth, the EKF and VO outputs, the loop-line trajectories (`.mat`), the depth-scale and aligned-IMU side files, plus the Town01 turnaround route. That bundle is enough to recompute the `EKF` / `VO` / loop-line cells of Table 4 and the whole closed-loop benchmark. Raw frames and the remaining trajectory directories (`slam_results_scale_nlm/`, `slam_results_orbonly/`, `_p11_backup_20261014/`) are not hosted (tens of GB); they are available from the corresponding author on request and can be packaged with `neuro/kbs/export_repro_bundle.py` into `neuro/kbs/repro_assets/` (`.gitignore`-exempt, about 10--15 MB, so they can be committed if you decide to publish them).
* **CARLA Town01 turnaround** (closed-loop benchmark of §4.5) — out-and-back route on the same streets, 10000 frames / 500 s / 881 m (881.5 m ground-truth length), 36 accepted loop events, acquired in an empty town (no background traffic) with the same sensor suite and 12 km/h speed command. Ships in the same asset bundle; the reported quantity is the **terminal drift after the anchored-100 Sim(3) fit** (842.54 m → 55.92 m, 15.07×). Reproduction is two commands: `export_closed_loop_inputs.py` builds the five input files, `kb_s2_se3_check.m` prints terminal and RMSE with a pass/fail verdict.
* **KITTI odometry 07 / 09** — public, see <https://www.cvlibs.net/datasets/kitti/eval_odometry.php>. Conversion to the pipeline format: `neuro/kbs/kitti_prepare_fusion.py`.
* **EuRoC MAV** (sanity check only) — public, see <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>.

Trained weights (`neuro/kbs/pr_head.pt`, `neuro/kbs/pr_scale_head.pt`) and all evaluation scripts **are** included. With the asset bundle in place, `se3_metrics.py` recomputes the `EKF`, `VO` and loop-line cells of Table 4 and the whole closed-loop benchmark; the remaining Table 4 cells (`Ours` on the four CARLA towns, `EKF+scale`, `ORB-only`) additionally need the pipeline's own trajectory directories, which `export_repro_bundle.py` packages from the run tree. `REPRODUCE.md` states cell by cell what the bundle covers.

---

## 4. Scope and known limitations

* The deployed filter runs four gated observation branches; a fifth (heading-from-turn-kinematics) branch is implemented but disabled in the configuration that produced the reported results, and is documented as such in the manuscript (Table 2).
* The scale head does **not** transfer to real KITTI imagery; all KITTI results run without it. This is reported, not hidden (manuscript §4.6 and Supplementary S.1).
* `neuro/kbs/*.m` scripts are legacy re-anchoring references kept for provenance; the published numbers come from the Python pipeline.
* Several mirrored third-party directories are unmodified copies; `bip/` is CC BY-NC 4.0 and therefore must not be used commercially.

---

## 5. License

* Authors' code (`neuro/`, including `neuro/kbs/`) and the two weight files: **MIT**, see `LICENSE`.
* Third-party directories: their own terms, see **`NOTICE`** (one of them, `bip/`, is non-commercial).

---

## 6. Citation

```bibtex
@article{wang2026neurolocmap,
  title   = {Brain-Inspired 4-DoF Localization-Mapping:
             Complementary Vestibular-Visual Fusion with Spatial Cells},
  author  = {Wang, Haidong and Yang, Rongbo},
  journal = {Knowledge-Based Systems},
  year    = {2026},
  note    = {Under review}
}
```

Contact: `open@hutb.edu.cn`
