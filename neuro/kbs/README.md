# repro_assets — optional reproduction artifacts

Drop here the trajectory directories that the minimal asset bundle does not carry,
so that Table 3 becomes fully recomputable instead of partly `carried`:

```
neuro/kbs/repro_assets/
├── 01_NeuroSLAM_Datasets/<seq>/slam_results_scale_nlm/trajectories.mat   # Ours column
├── 01_NeuroSLAM_Datasets/<seq>/slam_results_orbonly/trajectories.mat      # ORB-only ablation
└── _p11_backup_20261014/<Town05|Town03|Town07|Town10HD>/fp_head030.txt    # EKF+scale column
```

Produce them on the machine that holds the original run tree:

```bash
python3 neuro/kbs/export_repro_bundle.py --src /path/to/runs --out neuro/kbs/repro_assets
```

Then evaluate in place (no copying into `neuro/data/`):

```bash
NLM_DATA_ROOT=neuro/kbs/repro_assets/01_NeuroSLAM_Datasets \
NLM_BACKUP_ROOT=neuro/kbs/repro_assets/_p11_backup_20261014 \
python3 neuro/kbs/se3_metrics.py
```

`se3_metrics.json` then reports those cells as computed instead of `carried`.
These files are `.gitignore`-exempt on purpose (about 13 MB); commit them only if
you intend to publish the trajectories.
