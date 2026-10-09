[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)](https://pubsonline.informs.org/journal/ijoc)

# Solving aircraft recovery with copy and column generation

This archive is distributed in association with the [INFORMS Journal on
Computing](https://pubsonline.informs.org/journal/ijoc) under the [MIT License](LICENSE).

The software and data in this repository are a snapshot of the software and data
that were used in the research reported on the paper
[Solving aircraft recovery with copy and column generation](https://doi.org/10.1287/ijoc.2025.1669) by Zhouchun Huang, Xinjia Jiang, Xiaodong Luo, Qipeng P. Zheng, Panos M. Pardalos.

## Cite

To cite the contents of this repository, please cite both the paper and this repo, using their respective DOIs.

https://doi.org/10.1287/ijoc.2025.1669

https://doi.org/10.1287/ijoc.2025.1669.cd

Below is the BibTex for citing this snapshot of the repository.

```bibtex
@misc{Huang2025CCGCode,
  author = {Huang, Zhouchun and Jiang, Xinjia and Luo, Xiaodong and Zheng, Qipeng P. and Pardalos, Panos M.},
  publisher = {INFORMS Journal on Computing},
  title = {{Solving aircraft recovery with copy and column generation}},
  year = {2026},
  doi = {10.1287/ijoc.2025.1669.cd},
  url = {https://github.com/INFORMSJoC/2025.1669},
  note = {Available for download at https://github.com/INFORMSJoC/2025.1669},
}
```

DOIs follow the journal's assigned naming convention; release year and DOI
activation must be checked before publication.

## Description

The C++ implementation solves aircraft recovery using copy and column generation
(CCG), with optional cruise speed control (CSC). It includes time-space networks,
a path-based master problem, individual and simultaneous copy generation, and a
final integer program. Only the CCG solver is supported by the supplied build.

- `src/`: CCG source code.
- `data/1` through `data/30`: input instances.
- `results/`: manuscript tables, figures and raw outputs organized by experiment; see the [results index](results/README.md).
- `scripts/`: table-generation and plotting scripts; manuscript table inputs are in `scripts/data/reported_values.json`.
- `data/config.json`: author-supplied algorithm and cost settings.
- `docs/`: Linux and Windows Makefiles; run them from the repository root.

## Building

Dependencies: a C++20 compiler, OpenMP, Boost headers, nlohmann/json headers,
and Gurobi's C++ headers/library with a valid solver license. Gurobi is proprietary
and is not included or covered by this repository's MIT license. The manuscript
reports Gurobi 13.0, Windows 10, Intel Core i7-12700, 32 GB RAM, and 8 OpenMP threads.

Linux (GNU Make; use a Gurobi C++ library compatible with your compiler):

```sh
make -f docs/Makefile -j8 GUROBI_HOME=/opt/gurobi1301/linux64 GUROBI_LIB=gurobi130
```

Boost and nlohmann/json must be on the compiler include path. For custom locations,
pass `BOOST_ROOT=/path/to/boost` and `JSON_ROOT=/path/to/json/single_include`.
The Linux build has not been tested on this Windows host.

Windows, from an **x64 Native Tools Command Prompt for Visual Studio**:

```bat
set "GUROBI_HOME=D:\Tools\gurobi1301\win64"
nmake /f docs\Makefile.windows BOOST_ROOT=D:\Tools\boost_1_88_0 JSON_ROOT=D:\Tools\json\single_include
set "PATH=%GUROBI_HOME%\bin;%PATH%"
bin\arp.exe data 1 CCG
```

`GUROBI_LIB` can override the versioned library name. Existing C++ tests run with
`nmake /f docs\Makefile.windows test` or `make -f docs/Makefile test`.

## Replicating

Run commands from the repository root. On Linux:

```sh
./bin/arp data 1 CCG
for i in $(seq 1 30); do ./bin/arp data $i CCG || break; done
```

On Windows PowerShell (with Gurobi's bin directory on PATH):

```powershell
1..30 | ForEach-Object { & .\bin\arp.exe data "$_" CCG; if ($LASTEXITCODE -ne 0) { throw "Instance $_ failed" } }
```

For CSC, set `rule_config.use_cruise_control` and
`algorithm_config.solve_cruise_time_decisions` to `true`, with
`algorithm_config.seq_copy_generation` set to `true`. Outputs go to
`results/reproduced/<instance>/CCG` or `CCG-CSC`, preserving archived results.
Repeated runs with the same settings overwrite the corresponding reproduced output.

Sensitivity settings in `algorithm_config`:

| Experiment | Settings |
|---|---|
| Figure 4(a), instance 1 with CSC | `max_num_copies`: 1, 2, 5, 10, 20, 100, 10000 (infinity) |
| Figure 4(b), original | `seq_copy_generation=true`, `solve_cruise_time_decisions=true` |
| Figure 4(b), no conic | `seq_copy_generation=true`, `solve_cruise_time_decisions=false` |
| Figure 4(b), no multiple | `seq_copy_generation=false`, `solve_cruise_time_decisions=false` |

Figure 4(b) uses all 30 instances with CSC and `max_num_copies=2`.
Figure 3 is redrawn from the supplied 35-iteration log.

## Results

Each algorithm folder may contain `kpi.out` (costs and performance), `result.out`
(aircraft schedules), `solution.out` (flight assignments), `schedule.dat` (input
schedule review), and `copy_gen.out` (iteration history). Baseline outputs remain
available, but their deleted solvers cannot be rerun using this source release.
Portable scripts rebuild the tables and redraw the three numerical figures
from the retained manuscript values and archived files:

```sh
python -m pip install -r requirements.txt
python scripts/summarize_results.py
python scripts/plot_results.py
```

Table generation needs Python 3.10 or newer. The pinned plotting environment in
`requirements.txt` uses Python 3.12 or newer, matplotlib and numpy.
The plotting script reads raw files directly and saves `generated_figure.png`
in `results/copy_generation/`, `results/sensitivity/max_copies/` and
`results/sensitivity/simultaneous/`. Both scripts accept `--output-dir PATH`
to write generated artifacts elsewhere.
The [results index](results/README.md) collects manuscript result tables,
figures and output files by experiment. 
