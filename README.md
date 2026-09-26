# Robust Autonomous Quadrotor

A Python numerical system for quadrotor dynamics, sensing, state estimation,
feedback control and trajectory generation. The mathematical core is independent
of ROS 2 and PX4 and uses explicit coordinate frames, deterministic randomness,
reproducible experiments and independent numerical checks.

## Current scope

| Subsystem | Implemented | Boundary |
| --- | --- | --- |
| Plant | Six-degree-of-freedom dynamics, quaternion rotations, rotor allocation, motor lag, wind/drag, Euler and projected RK4 | Illustrative parameters; no ground-contact or identified hardware model |
| Sensors and runs | Noisy IMU/position/altitude, bias walks, scheduled delivery, truth/nominal mismatch, named RNG streams, authenticated artifacts and replay | Explicit supported schedules and numerical backend |
| Estimator | 15-state ESKF prediction, correction/reset, gating, endpoint propagation, replay and causal online execution | Explicit prior; stale data rejected; no automatic startup alignment |
| Feedback | Attitude/rate and position/velocity cascade; true-state and estimated-state missions | Estimated-state **8 cm full-hold qualification remains open** |
| Planning | Minimum-snap position solver, whole-curve nominal bounds, bounded uniform retiming and true-state mission integration | No time-optimal allocation, obstacles, torque/motor feasibility proof or estimated-state trajectory qualification |
| Integration | Prior PX4/Gazebo compatibility spike | ROS 2/C++ wrappers and integrated missions remain future work |

The two frozen estimated-feedback profiles retain their historical **29/30** and
**28/30** fresh-case results. Neither satisfies every original hover condition.
Software checks and accepting a numerical reference do not convert those failures
into passes. The [closeout audit](docs/progress/2026-09-25-repository-audit.md)
records the common-case comparison and chosen reference. Failed tuning studies
remain reproducible; no further automatic gain search is planned.

See [technical status](docs/status.md) for remaining scope and limitations.

## Setup and checks

Use Python 3.12 and uv 0.12.3, as pinned in `pyproject.toml`.

```bash
uv sync --locked
PYTEST_ADDOPTS='-W error' make check
```

The gate runs Ruff lint/format, strict mypy and pytest. Tests cover analytical
physics, frame/sign conventions, Jacobians, covariance algebra, causal data flow,
numerical convergence, controller behavior, persistence and trajectory optimality.
Test counts are recorded with their commits; they are not a flight certificate.

## Start reading

- [Documentation map](docs/README.md): subsystem guides and evidence navigation.
- [Frame contract](docs/architecture/frame-contract.md): NED world, FRD body,
  body-to-world `R_WB`, scalar-first `q_WB`.
- [Plant, sensors and reproducible runs](docs/foundations.md).
- [Estimation](docs/estimation.md), [attitude control](docs/control.md),
  [position control and missions](docs/position-control.md).
- [Estimated feedback](docs/estimated-feedback.md) and
  [cascade design](docs/feedback-design.md).
- [Minimum-snap trajectories](docs/trajectories.md): equations, usage and checks.
- [Trajectory bounds and missions](docs/trajectory-missions.md): timing policy,
  reference feasibility and measured flight results.
- [Architecture decisions](docs/decisions/README.md),
  [dated verification records](docs/progress/README.md), and [changelog](CHANGELOG.md).

## Repository layout

```text
src/quadrotor_math/   Middleware-independent numerical algorithms
tests/unit/          Mathematical, boundary and integration checks
experiments/         Reproducible campaigns, diagnostics and plotting commands
docs/                Technical guides, decisions and dated evidence
```

Generated histories, figures and large logs stay outside Git. Experiment output
directories must be new. See [environment notes](docs/environment.md) for pinned
tooling and the numerical-backend boundary on byte-identical historical replay.

The immediate task is recovering the unpublished geometric-controller source
before tuning can continue. The [recovery audit](docs/progress/2026-09-26-source-recovery-audit.md)
distinguishes retained flight evidence from code available in this repository.
System fault accommodation and ROS/PX4 integration remain separate milestones.
