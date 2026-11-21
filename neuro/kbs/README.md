# neuro/kbs — evaluation, manuscript sources and reproduction

This directory holds everything that produces the numbers in the manuscript:
metric scripts, the gate/scale/loop toolchain, trained weights (`pr_head.pt`,
`pr_scale_head.pt`), figure sources, the LaTeX sources and the per-sequence
records.

Start from the repository root — the main README is there:

| Looking for | File |
|---|---|
| project overview, repository layout, data availability, license | [`../../README.md`](../../README.md) |
| reproduction guide, table by table, script by script | [`REPRODUCE.md`](REPRODUCE.md) |
| one-command check of every number against the data (`OK/FAIL/SKIP`) | [`run_all_checks.py`](run_all_checks.py) + [`CHECKLIST_VERIFY.md`](CHECKLIST_VERIFY.md) |
| manuscript sources | `NeuroSLAM_KBS.tex` (main), `supplementary_KBS.tex`, `cover_letter_KBS.tex`, `highlights.tex` |
| checked-in manuscript values | [`expected_values.json`](expected_values.json) (as printed) / [`se3_metrics.json`](se3_metrics.json) + [`final_metrics.json`](final_metrics.json) (as computed) |

Notes in this directory are research records, not release documentation:
`REPRO_ASSETS_MANIFEST_20261017.md`, `RERUN_MANUAL.md`, `NLM_LOOP_DESIGN.md`,
`PR_P2_RESULTS.md` and the `CODE_READING_*` files describe intermediate states and
are labelled as such at the top; where they disagree with `REPRODUCE.md`, the
latter wins. Unused older LaTeX variants are in `legacy/`.
