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

The current research focus is control using noisy state estimates. The geometric
controller tracks well when supplied with the simulated true state. The latest
experimental change filters vertical and horizontal feedback forces separately,
reducing the startup transient while retaining analytic trajectory feedforward.
The [final comparison](../results/final-geometric.md) independently selects a cascade
comparator and checks fresh hover, trajectory and fault cases. The cascade
remains the default. The
[controller discussion](../results/controller-tradeoffs.md) explains the observed
problems, tested alternatives, and why an improvement in one metric can make
another worse. [Project status](status-before-overhaul.md) lists the evidence and remaining
work.

On twelve reserved cases, the final geometric profile reduces balanced RMSE by
4.8% versus the independently selected cascade and 20.3% versus its original
settings. Its whole-flight peaks and moment effort remain higher. The measured
gain is retained even though it misses the predeclared 10% improvement target.

The earlier supported-start studies retain their passing absolute limits and
failed comparisons. The final geometric study implements a further improvement
and reports average tracking error, peak error and moment effort separately.
The original mass-mismatch failure and broader flight qualification remain open.

The [operating envelope](../results/operating-envelope.md) is the source-indexed guide
to what is implemented, what passes, and what remains conditional or failed.
[Report revision 3.0](report-history.md) develops the mathematics, including
supported alignment, nonlinear release uncertainty and the whole-flight error
budget. The [preceding supported geometric comparison](../results/supported-geometric.md)
combines the existing startup fixes with one fixed geometric profile. The
[next-step plan](previous-plan.md) pauses controller improvement after the
final bounded study, with professor-facing report preparation reserved for a
later task. Report revision 3.0 predates both geometric follow-ups.

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
[setup guide](setup-environment.md) for individual checks, headless execution and
reproducibility details.

## Find an explanation or implementation

| If you want to understand… | Start here |
| --- | --- |
| How the complete system fits together | [System design](../guides/system-design.md) |
| Controller gains, remaining failures and measured tradeoffs | [Controller performance and tradeoffs](../results/controller-tradeoffs.md) |
| The equations, interfaces and source for a subsystem | [Technical documentation](documentation-map.md) |
| What has been demonstrated and what remains | [Operating envelope](../results/operating-envelope.md) and [current status](status-before-overhaul.md) |
| The next bounded piece of work | [Next-step plan](previous-plan.md) |
| Why a design was chosen, or how it was tested | [Design decisions](../decisions/README.md) and [verification records](records/README.md) |

Core algorithms are in [`src/quadrotor_math`](../../src/quadrotor_math), experiments in
[`experiments`](../../experiments), and tests in [`tests/unit`](../../tests/unit). Generated
histories and plots are excluded from Git. The
[frame contract](../architecture/frame-contract.md) defines NED world, FRD body,
and body-to-world quaternion conventions used throughout.

This is a simulation research project. Earlier PX4/Gazebo compatibility checks
are recorded, but the custom estimator/controller stack has not been integrated
into a deployed flight system or validated on hardware.
