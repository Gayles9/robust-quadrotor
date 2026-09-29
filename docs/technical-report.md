# Technical report and operating boundary

Revision 3.0, dated 2026-09-29, consolidates the implemented mathematical stack
and retained evidence through merged software baseline
`67499e9eb2c9a0acd72eb45d7be690f1bb2e3a2b` (PR #39).

The existing PDF and editable LaTeX package keep the filenames
`Robust_Quadrotor_Technical_Report_v1.pdf` and `Robust_Quadrotor_Report_v1.zip`
and their artifact identities/history. The revision inside the report is
authoritative. Generated reports and flight histories remain outside Git.

## What it covers

- Frames, rigid-body plant, actuators, numerical integration, sensors and clocks.
- ESKF prediction, correction/reset, shared endpoint noise memory and causal replay.
- Supported stationary alignment, nonlinear attitude/bias uncertainty, one-time
  exact velocity conditioning and the first prediction after support removal.
- Minimum-snap planning, reference bounds, cascade/geometric control, observation
  availability, timed numerical abort and bounded vertical compensation.
- Original and supported-start flight results, correlated whole-flight error
  budgets, independent-inclination feasibility and the stopping decision.
- The [source-indexed operating envelope](operating-envelope.md), reproducible
  audits, exact provenance and the separately scoped next integration contract.

The combined supported-prior candidate passes all nine tested clean absolute
flight limits, including hover peaks of 6.7539, 5.6943 and 5.5644 cm. Five clean
comparisons nevertheless fail strict no-regression against one or both controls.
These previously studied seeds are not fresh candidate qualification. The
original cascade remains default; the mass and geometric requirements stay open.
No new scientific flight, tuning, sensor or integration is part of this report.

## Build and audit

From the package's `report/` directory:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

The package includes a self-contained LaTeX build, bibliography, vector hover
comparison, exact historical and current summary JSON, full unrounded data,
independent audit scripts and source/verification indices. Raw flight payloads
remain in their existing evidence archives. From the package root:

```bash
python tools/audit_report_evidence.py
OPENBLAS_NUM_THREADS=1 python tools/audit_current_evidence.py
```

Optional `--original`, `--boundary` and `--combined` paths reauthenticate and
rescore all 84 saved executions without running the flight implementation.
`--render` regenerates the tables and vector figure from authenticated values.
The numerical audit needs NumPy; figure regeneration additionally uses Matplotlib.
No shell escape or Python is required to compile the included LaTeX.

Fresh audit coverage is nine exact summary reports (two historical and seven
current), 436 payload references across the three supported campaigns, 84
full-grid metric reconstructions and 144 Gram windows. Full ESKF/command replay
and the nonlinear calibration populations remain separately dated prior results;
the report does not relabel them as newly executed verification.

| Evidence | SHA-256 |
| --- | --- |
| Original supported campaign report | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Boundary-only report | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |
| Combined-prior report | `6d44ab913156fafe3846a630065cdaad9eee2dc9c64c1ea50234b82fbfa314fa` |
| Whole-flight budget report | `b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb` |
| Inclination feasibility report | `f0b03f6d39ec746bcad7d71aba337191d6bac9c43c63af4ead777a2be8280164` |

The complete index in the package also binds nonlinear release and saved-history
diagnosis reports, source files and previous artifact hashes. Historical
convergence measurements stay dated 2026-09-23. Earlier executions retain their
own source/provenance; they are not relabelled as clean runs of PR #39.

The [revision 3.0 closeout](progress/2026-09-29-operating-envelope-report.md)
records exact checks and deliverable hashes. The
[revision 2.0 closeout](progress/2026-09-28-technical-report.md) remains historical.
The [next task](next-steps.md) is a simulation-only ROS 2/PX4 interface and
acceptance design, with estimated-flight G2 explicitly open.
