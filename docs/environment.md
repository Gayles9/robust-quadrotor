# Development Environment

Inventory recorded on 2026-08-12.

## Host Hardware

- Lenovo Yoga 9, model 82BG
- Intel Core i7-1195G7
- 8 GB physical RAM
- Intel Iris Xe integrated graphics
- Approximately 231 GB free on the Windows drive at inventory time

## Host and WSL

- Windows host reported Windows 10 Home with build 26200. This product-name/build pairing is inconsistent and requires later confirmation.
- WSL2 with Ubuntu 24.04.4 LTS
- WSL kernel 6.6.114.1-microsoft-standard-WSL2
- WSL reported 3.6 GiB memory and 1.0 GiB swap

## Development Tools

- Python 3.12.3
- Git 2.43.0
- GCC 13.3.0
- CMake 3.28.3
- uv 0.12.3

Limited WSL resources remain a consideration for simultaneous PX4/Gazebo use, while Gazebo
graphical lag was directly observed. Prefer headless simulation when graphics are unnecessary.

## Week 2 Compatibility Matrix

Compatibility spike completed on 2026-08-16 with the following pinned environment:

| Component | Pinned version or configuration |
| --- | --- |
| WSL2 guest | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy desktop |
| ROS development tools | `ros-dev-tools` |
| ROS--Gazebo integration | `ros-jazzy-ros-gz` |
| ROS vendor Gazebo | 8.11.0 |
| PX4/system Gazebo | 8.15.0 at `/usr/bin/gz` |
| PX4 release | v1.17.0 |
| PX4 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| QGroundControl | v5.1 stable AppImage |
| Python | 3.12.3 |
| GCC | 13.3.0 |
| CMake | 3.28.3 |
| Git | 2.43.0 |
| uv | 0.12.3 |

## ROS and PX4 Environment Separation

Keep ROS and PX4 terminal environments separate to avoid selecting the ROS vendor Gazebo
instead of the PX4/system Gazebo.

- Default PX4 terminals must not source ROS. In a clean PX4 environment, `gz` must resolve
  to `/usr/bin/gz` version 8.15.0 and `GZ_CONFIG_PATH` must remain empty.
- Activate ROS 2 Jazzy manually only in dedicated ROS terminals:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

- ROS-activated terminals must not build or launch PX4.

## Nominal ESKF Evaluation Environment

The 2026-09-23 consistency-evaluation milestone was checked on Linux with Python 3.12.14,
uv 0.12.3, NumPy 2.5.2, pytest 9.1.1, Ruff 0.16.2 and mypy 1.20.2. This is the audit
environment, not a revision of the historical WSL inventory above. Project dependencies,
lockfile, tool pins and CI configuration are unchanged.

Use `uv sync --locked`, then `uv run make check`. The nominal study is executable with
`uv run python experiments/eskf_consistency.py --partition smoke --output /tmp/eskf-smoke.json`.
Choose a new output path for each run; results are never overwritten. `development` and
`validation` select the separately frozen longer partitions. The CLI records actual
installed versions, Git HEAD/dirty status and a digest of source/configuration files.
Run it from a real Git checkout with the project environment installed. The evaluation
does not require ROS, PX4, SciPy, plotting packages or a display.

An independently installed SciPy 1.17.0 was used only as an audit oracle for chi-square
quantiles. It is not a dependency of the package, tests or experiment. Exact verification
commands and numerical findings are in the
[consistency record](progress/2026-09-23-eskf-consistency-evaluation.md).
