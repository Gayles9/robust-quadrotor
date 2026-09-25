# Documentation map

Use [status](status.md) for current scope, then the relevant subsystem guide.
The chronological progress files preserve what each earlier commit actually
implemented and measured; their historical next actions are not today's plan.

| Topic | Guide | Decisions / evidence |
| --- | --- | --- |
| Frames and physics | [Frame contract](architecture/frame-contract.md), [foundations](foundations.md) | [Decisions 0002–0003](decisions/README.md) |
| Environment and reproduction | [Environment](environment.md), [foundations](foundations.md) | [Progress index](progress/README.md) |
| ESKF | [Estimation](estimation.md) | [Decisions 0004–0010](decisions/README.md) |
| Attitude/rate control | [Control](control.md) | [Decision 0011](decisions/0011-baseline-attitude-control.md) |
| Position control and missions | [Position control](position-control.md) | [Decision 0012](decisions/0012-position-control-and-missions.md) |
| Estimated feedback | [Integration](estimated-feedback.md), [design](feedback-design.md) | [Closeout audit](progress/2026-09-25-repository-audit.md) |
| Minimum-snap planning | [Trajectories](trajectories.md) | [Decision 0015](decisions/0015-minimum-snap-trajectory.md) |

## Experiments

Run modules from the repository root with `uv run python -m experiments.NAME`.
The module help and linked guides define input/output paths and frozen protocols.

- `euler_rk4_convergence`: numerical convergence.
- `eskf_consistency`, `eskf_validation`: estimator and fault/consistency evidence.
- `attitude_control_validation`, `position_control_validation`: baseline flight.
- `estimated_feedback_validation`, `feedback_bandwidth_validation`: original and
  two designed estimated-feedback profiles; failures remain explicit.
- `feedback_baseline_comparison`: the fixed six observed full-hover comparisons.
- `minimum_snap_example`: waypoint interpolation, continuity and cost example.
- `kalman_sandbox`: small educational Kalman calculations.
- `plot_*`: plots of the corresponding saved campaign evidence.

`feedback_startup_diagnostic`, `feedback_motor_damping_probe` and
`feedback_codesign` retain the bounded diagnostic/rejected design experiments.
They are not supported production controllers. Their code and dated records
remain to reproduce the negative results; do not rerun them as open-ended tuning.
