# Computational results

Costs are in thousands of dollars; percentages are in percent units.
`*` retains the manuscript notation.

| Paper item | Results |
|---|---|
| Table 3: computational comparison | [Summary](computational_comparison/summary.md) |
| Table 4: cruise speed control | [Summary](cruise_speed_control/summary.md) |
| Restricted-master integrality | [Summary](computational_comparison/integrality.md) |
| Figure 3: copy generation | [Figure](copy_generation/manuscript_figure.pdf), [data](copy_generation/summary.md) |
| Figure 4(a): maximum copies | [Figure](sensitivity/max_copies/manuscript_figure.pdf), [data](sensitivity/max_copies/summary.md) |
| Figure 4(b): simultaneous generation | [Figure](sensitivity/simultaneous/manuscript_figure.pdf), [data](sensitivity/simultaneous/summary.md) |
| Tables S1-S4 and Figures S2-S3 | [Online supplement](supplement/README.md) |

`raw/` contains run outputs. `manuscript_figure.pdf` contains the paper figure.
Tables 3 and 4 use [manuscript values](../scripts/data/reported_values.json).

From the repository root:

```sh
python -m pip install -r requirements.txt
python scripts/summarize_results.py
python scripts/plot_results.py
```

The scripts generate eight tables and three `generated_figure.png` files.
Both accept `--output-dir PATH`. The pinned plotting environment requires Python 3.12+.
