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
controller tracks well when supplied with the simulated true state, but it does
not yet meet the combined hover and control-effort requirements with estimated
feedback. The cascade remains the default. The
[controller discussion](docs/controller-tradeoffs.md) explains the observed
problems, tested alternatives, and why an improvement in one metric can make
another worse. [Project status](docs/status.md) lists the evidence and remaining
work.

The bounded startup/hover investigation is now closed: its single candidate
reduced spline effort but still failed hover, so the accepted controllers remain
unchanged. The [closeout](docs/progress/2026-09-26-geometric-transient-closeout.md)
records the result. [Observation health monitoring](docs/observation-health.md)
now reports accepted-data loss and recovery independently for position and
altitude. An optional [observation supervisor](docs/observation-supervision.md)
now aborts numerical missions when explicit per-stream budgets expire. The next
task is broader integrated evaluation under faults and model mismatch.

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
| What has been demonstrated and what remains | [Current status](docs/status.md) |
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
