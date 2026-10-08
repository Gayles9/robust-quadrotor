# Setup and reproducibility

The Python core runs without ROS 2, PX4 or Gazebo. Use Python 3.12 and uv 0.12.3;
`pyproject.toml`, `.python-version` and `uv.lock` define the project environment.
GNU Make is used for the convenience targets.

## Install and check

From a checkout of the repository:

```bash
uv sync --locked
PYTEST_ADDOPTS='-W error' make check
```

`make check` checks documentation links and example syntax, Ruff lint/format,
strict mypy and pytest. Individual checks are available when editing one area:

```bash
make docs-check
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments scripts
uv run pytest -q -W error tests/unit/test_geometric_control.py
```

`make docs-check` uses only the Python standard library. Markdown renders on
GitHub; there is no separate documentation server or LaTeX build in this
repository. Python code blocks are syntax-checked. Interface descriptions and
pseudocode are labelled as text so they are not presented as complete programs.
Syntax checking does not imply that a snippet with caller-supplied inputs is
standalone. The README's example and the complete planning example are exercised
in the documentation audit.

The September 26 controller audit used Linux, Python 3.12.14, uv 0.12.3,
NumPy 2.5.2, pytest 9.1.1, Ruff 0.16.2 and mypy 1.20.2. Matplotlib 3.11.2 supplies
headless experiment plots. These are recorded execution versions, not additional
manual installation steps; install from the lockfile.

## Run a small example

```bash
uv run python -m experiments.minimum_snap_example --output results/minimum-snap-demo
```

This generates a trajectory plot, data and metrics. It is a planning example,
not a simulated flight. Outputs must use a new path; existing results are not
overwritten. The `results/` directory is ignored by Git. Other experiments and
their intended use are listed in the [documentation map](documentation-map.md).

For CPU-heavy campaigns I normally restrict BLAS threads and select an explicit
worker count. This avoids each worker starting its own large thread pool:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.trajectory_mission_validation \
  --workers 3 --output results/trajectory-missions
```

A performance campaign may return a nonzero exit status because a declared
flight requirement failed. Read its report and failure ledger. Do not change a
threshold, omit a trial or reuse an observed seed as fresh validation to make
that exit status disappear.

## What reproducibility means here

The experiment fixes its configuration, seed assignments, scoring windows and
source fingerprint. Its saved reports bind payload files with SHA-256. Loading
and independent rescoring check both data integrity and the conclusions drawn
from the data. Source changes during a campaign invalidate its recorded run.

Exact historical bytes also depend on the numerical environment. Different
BLAS kernels can change a derived floating-point value by a few units in the
last place, even with the same NumPy version and seed. The
[cascade-design guide](../design/feedback-design.md#reproduction) explains the existing
protocol-fixture boundary. Production hashes and archive checks remain strict;
a different numerical realization must not be called an exact replay.

An earlier independent SciPy calculation checked chi-square quantiles during
estimator validation. SciPy is not a project dependency. Temporary tools used to
render documentation likewise do not change the runtime or locked environment.

## Earlier ROS 2 and PX4 compatibility environment

The August 16 compatibility spike used the following environment. This is a
record of that test, not a requirement for running the Python core and not a
claim about current upstream releases.

| Component | Tested version or configuration |
| --- | --- |
| WSL2 guest | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy desktop, with `ros-dev-tools` and `ros-jazzy-ros-gz` |
| ROS vendor Gazebo | 8.11.0 |
| PX4/system Gazebo | 8.15.0 at `/usr/bin/gz` |
| PX4 | v1.17.0, commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| QGroundControl | v5.1 stable AppImage |
| Python / GCC / CMake / Git | 3.12.3 / 13.3.0 / 3.28.3 / 2.43.0 |

Keep ROS and PX4 terminal environments separate: sourcing ROS can select its
vendor Gazebo instead of the tested PX4/system version. In a clean PX4 terminal,
`gz` resolved to `/usr/bin/gz` and `GZ_CONFIG_PATH` was empty. Source
`/opt/ros/jazzy/setup.bash` only in a dedicated ROS terminal. Prefer headless
Gazebo when graphics are unnecessary. The
[compatibility record](records/px4-gazebo-compatibility.md) describes
the flight and observed limitations.
