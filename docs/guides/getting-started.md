# Getting started

The Python core runs without ROS 2, PX4 or Gazebo. Use Python 3.12, uv 0.12.3
and GNU Make. The project fixes its environment in `pyproject.toml`,
`.python-version` and `uv.lock`; install from that lockfile.

Use Linux or WSL for the complete supported workflow. Artifact publication uses
Linux's `renameat2` system call to install completed runs without overwriting
existing evidence. macOS does not support that part of the implementation.

## Install and run the first example

```bash
git clone https://github.com/Gayles9/robust-quadrotor.git
cd robust-quadrotor
uv sync --locked
uv run python -m experiments.minimum_snap_example --output results/minimum-snap-demo
```

The example fits a fixed-duration minimum-snap path through four waypoints. It
independently checks waypoint residuals, continuity and integrated cost, prints
a JSON summary and creates three files:

| File | What to inspect |
| --- | --- |
| `trajectory.png` | The path in North/East/altitude coordinates and snap over time |
| `trajectory.npz` | Waypoints, durations, polynomial coefficients and sampled derivatives |
| `report.json` | Snap cost, comparison cost, numerical residuals and source/data hashes |

Plotting uses a headless backend, so a desktop display is unnecessary. Snap is
the fourth time derivative of position. Reducing its squared integral encourages
a smooth reference; this example does not check obstacles, actuators or flight
tracking. Those are separate concerns in the
[trajectory guide](../design/trajectory-missions.md).

Choose a new output directory for every run. The tools deliberately refuse to
overwrite evidence. `results/` at the repository root is ignored by Git.

## Run a virtual flight

This fixed validation set exercises the baseline controller with the simulated
true state: hover, square tracking, a vertical step, mild wind, and a refined
square run for numerical comparison.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.position_control_validation \
  --partition fixed --workers 1 --output results/position-control
uv run python -m experiments.plot_position_control \
  --input results/position-control --output results/position-control-plots
```

The run writes `report.json` and `trial-000.npz` through `trial-004.npz`.
The plotter reads and validates those histories, then writes
`mission_hover.png`, `mission_square.png`, `mission_vertical_step.png`,
`mission_mild_wind.png`, `mission_path_and_actuation.png` and `plot_manifest.json`.
Start with the report's acceptance results and the position-error plots.

True-state feedback isolates the controller. The
[estimated-feedback guide](../design/estimated-feedback.md) explains the separate
sensor/estimator loop; its performance cannot be inferred from this example.

A performance campaign may exit unsuccessfully because a declared flight
requirement failed. Keep its report and failure ledger. Do not change thresholds
or omit trials to turn a retained failure into a pass.

## Check the installation

```bash
PYTEST_ADDOPTS='-W error' make check
```

This checks documentation links and example syntax, Ruff lint/format, strict
mypy and pytest. See [development practices](../development.md) for individual
checks, packaging and numerical reproducibility.

For longer experiments, an explicit worker count and `OPENBLAS_NUM_THREADS=1`
avoid giving each worker its own large thread pool. Follow each study's frozen
protocol and required parent artifacts rather than treating every module as an
independent demo. Continue with [system design](system-design.md) and
[results and validation](../results/README.md).
