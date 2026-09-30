# Robust Autonomous Quadrotor

A Python project for simulating a quadrotor, estimating its motion from noisy
sensors, and controlling it along a planned trajectory.

I am building the mathematical stack from the rigid-body model upward so I can
inspect how modeling assumptions, estimation errors, and controller choices
affect flight. The core runs independently of ROS 2 and PX4. Experiments use
explicit parameters and random seeds, and retain failures alongside successes.

## What works

The repository contains a nonlinear flight model with motors, wind and drag;
simulated IMU, position and altitude sensors; a 15-state error-state Kalman filter;
cascaded and geometric controllers; and minimum-snap trajectory planning. A
mission runner connects these pieces for virtual takeoff, tracking and landing.
Saved measurements can be replayed to check the estimator independently.

The current limitation is control using noisy state estimates. The geometric
controller tracks well when supplied with the simulated true state. With estimated
feedback, the latest supported-start option passes both full hovers and improves
nominal spline RMSE, but misses the matched wind-tracking comparison. The cascade
remains the default. The
[controller discussion](docs/controller-tradeoffs.md) explains the observed
problems, tested alternatives, and why an improvement in one metric can make
another worse. [Project status](docs/status.md) lists the evidence and remaining
work.

The earlier supported-cascade research brings all three short-hold hover peaks
below 8 cm, but fails five of nine strict no-regression comparisons. The new
supported-geometric full-hold peaks are 5.21 and 4.62 cm, versus 8.73 and 18.10 cm
with matched unaligned starts. Its wind RMSE is 3.73% worse than matched cascade,
so its frozen comparison also fails. Both options remain experimental. The
original mass-mismatch failure and fresh qualification are unresolved.

The [operating envelope](docs/operating-envelope.md) is the source-indexed guide
to what is implemented, what passes, and what remains conditional or failed.
[Report revision 3.0](docs/technical-report.md) develops the mathematics, including
supported alignment, nonlinear release uncertainty and the whole-flight error
budget. The [bounded supported geometric comparison](docs/supported-geometric.md)
combines the existing startup fixes with one fixed geometric profile. The
[next-step plan](docs/next-steps.md) pauses controller improvement after that
experiment, with professor-facing report preparation reserved for a later task.
Report revision 3.0 predates this final comparison.

## Run it

Use Python 3.12, uv 0.12.3 and GNU Make. From the repository root:

```bash
uv sync --locked
PYTEST_ADDOPTS='-W error' make check
uv run python -m experiments.minimum_snap_example --output results/minimum-snap-demo
```

The checks cover documentation, lint, formatting, types and tests. The small
trajectory example writes a plot, numerical data and metrics; it does not run a
flight. Choose a new output directory each time. See the
[setup guide](docs/environment.md) for individual checks, headless execution and
reproducibility details.

## Find an explanation or implementation

| If you want to understand… | Start here |
| --- | --- |
| How the complete system fits together | [System design](docs/system-design.md) |
| Why the controller still misses its targets | [Controller problems and tradeoffs](docs/controller-tradeoffs.md) |
| The equations, interfaces and source for a subsystem | [Technical documentation](docs/README.md) |
| What has been demonstrated and what remains | [Operating envelope](docs/operating-envelope.md) and [current status](docs/status.md) |
| The next bounded piece of work | [Next-step plan](docs/next-steps.md) |
| Why a design was chosen, or how it was tested | [Design decisions](docs/decisions/README.md) and [verification records](docs/progress/README.md) |

Core algorithms are in [`src/quadrotor_math`](src/quadrotor_math), experiments in
[`experiments`](experiments), and tests in [`tests/unit`](tests/unit). Generated
histories and plots are excluded from Git. The
[frame contract](docs/architecture/frame-contract.md) defines NED world, FRD body,
and body-to-world quaternion conventions used throughout.

This is a simulation research project. Earlier PX4/Gazebo compatibility checks
are recorded, but the custom estimator/controller stack has not been integrated
into a deployed flight system or validated on hardware.
