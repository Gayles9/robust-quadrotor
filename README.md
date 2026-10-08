# Robust Autonomous Quadrotor

A Python project for simulating a quadrotor, estimating its motion from noisy
sensors, and controlling it along a planned trajectory.

I am building the mathematical stack from the rigid-body model upward so I can
inspect how modeling assumptions, estimation errors and controller choices
affect flight. The core runs independently of ROS 2 and PX4. Experiments use
explicit parameters and random seeds, and retain failures alongside successes.

## What works

- A nonlinear flight model with rotor allocation, motor response, wind and drag.
- Simulated inertial, position and altitude sensors, with recorded measurements
  and a 15-state error-state Kalman filter (ESKF) for estimating motion and bias.
- Cascaded and geometric controllers, plus virtual takeoff, tracking and landing.
- Minimum-snap trajectory planning: smooth polynomial paths through timed waypoints.
- Deterministic tests, saved-history replay and bounded comparison studies.

The [final geometric comparison](docs/results/final-geometric.md) reports **4.8%
lower balanced tracking RMSE** than an independently selected cascade on twelve
reserved cases. RMSE measures typical position error over a flight. Whole-flight
peaks and moment effort remain higher, and the predeclared 10% improvement target
is missed. The original cascade remains the default; this is a measured tradeoff
within the tested simulation conditions.

## Run the first example

Use Linux or WSL, Python 3.12 and uv 0.12.3. From the repository root:

```bash
uv sync --locked
uv run python -m experiments.minimum_snap_example --output results/minimum-snap-demo
```

The example plans a path through four waypoints and writes:

- `trajectory.png`: the path and its snap, the fourth time derivative of position.
- `trajectory.npz`: waypoints, polynomial coefficients and sampled derivatives.
- `report.json`: cost, numerical residuals and reproducibility hashes.

This is a small planning example. To run a simulated flight, follow the
[getting-started guide](docs/guides/getting-started.md). Use a new output directory
for each run; generated results are excluded from Git.

## Explore the project

| Your next question | Start here |
| --- | --- |
| How do I install, run a flight and inspect its outputs? | [Getting started](docs/guides/getting-started.md) |
| How does the flight loop fit together? | [System design](docs/guides/system-design.md) |
| Where are the equations and implementation interfaces? | [Technical documentation](docs/README.md) |
| What has been demonstrated, and under which assumptions? | [Results and validation](docs/results/README.md) |
| Why do the controllers behave differently? | [Controller tradeoffs](docs/results/controller-tradeoffs.md) |
| What does the official technical write-up cover? | [Report guide](docs/report.md); PDF and source will be added separately |

Core algorithms are in [`src/quadrotor_math`](src/quadrotor_math), reproducible
study runners and plotters in [`experiments`](experiments), and checks and small
fixtures in [`tests`](tests). The [development guide](docs/development.md) covers
verification and maintenance.

## Scope and limitations

This is a simulation research project. The custom estimator/controller stack
has not been deployed through PX4 or validated on hardware. The plant uses
illustrative parameters; landing ends at a virtual airborne plane without contact
physics. Supported-start studies assume an explicit stationary fixture. An
observation-loss abort stops the numerical mission; it is not a physical
fallback maneuver.

Good true-state tracking, estimator consistency and successful noisy closed-loop
flight are separate findings. Broader noisy-feedback qualification and the
original mass-mismatch requirement remain open. The
[capability and requirement ledger](docs/results/operating-envelope.md) gives the
precise boundaries; [next steps](docs/next-steps.md) keeps planned work separate.
