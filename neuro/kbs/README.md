# neuro/kbs — evaluation, manuscript sources and reproduction

This directory holds everything that produces the numbers in the manuscript:
metric scripts, the gate/scale/loop toolchain, trained weights (`pr_head.pt`,
`pr_scale_head.pt`), figure sources, the LaTeX sources and the per-sequence
records.

> **Naming.** Manuscript = *NeuroLocMap*; code and LaTeX sources keep the historical
> *NeuroSLAM* file names. Same system, two names.

Start from the repository root — the main README is there:

| Looking for | File |
|---|---|
| project overview, repository layout, data availability, license | [`../../README.md`](../../README.md) |
| reproduction guide, table by table, script by script | [`REPRODUCE.md`](REPRODUCE.md) |
| one-command check of every number against the data (`PASS/FAIL/SKIP`) | [`run_all_checks.py`](run_all_checks.py) (section-by-section semantics are documented at the top of the script) |
| manuscript sources | `NeuroSLAM_KBS.tex` (main), `supplementary_KBS.tex`, `cover_letter_KBS.tex`, `highlights.tex` |
| checked-in manuscript values | [`expected_values.json`](expected_values.json) (as printed) / [`se3_metrics.json`](se3_metrics.json) + [`final_metrics.json`](final_metrics.json) (as computed) |

| Directory | Role |
|---|---|
| `fig/` | figure PDFs actually compiled, plus `.pptx` drawing sources — see `fig/README.md` |
| `src/` | nine early draft drawing scripts (`bio_nav_architecture`, `experience_map`, `grid_cell_activity`, `hdc_network`); none of their outputs is in the current manuscript |
| `module/elsarticle/` | vendored Elsevier class (the build uses the TeX Live installation; kept for offline/Overleaf builds) |
| `scale_p1/` | scale-series intermediates written by `scale_series_p1.py` (Supplementary S.1 figure) |
| `other/` | journal declaration form (`declarationStatement_kbs.docx`) |
| `legacy/` | older LaTeX variants, not compiled |
| `repro_assets/` | placeholder for the minimal asset bundle — see its `README.md` |

Only the files listed above are release documentation. The Chinese-language
research notes that used to sit here (design log, intermediate result dumps,
code-reading notes, rerun manual, asset manifest, FPS evidence) have been moved
out of the repository to keep it English-only; `REPRODUCE.md` is the single
authority for what is reproducible. Unused older LaTeX variants are in `legacy/`.
