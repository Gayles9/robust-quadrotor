# Verification records

These dated records preserve the implementation decisions, calculations,
commands, failures and measured results for each milestone. Start with
[current status](../status.md) for what is supported today and the
[project review](../project-review.md) for the latest repository audit.

A record's test count, publication state and next action describe that session.
They do not override the [current plan](../next-steps.md). Historical workstation
and scratch paths have been replaced with portable example paths; numerical
results, seeds, thresholds and commit identities are preserved.

## Controller and planning studies

- [2026-09-28: Causal early-flight diagnosis](2026-09-28-early-flight-diagnosis.md)
- [2026-09-28: Technical report and operating boundary](2026-09-28-technical-report.md)
- [2026-09-28: Bounded vertical compensation](2026-09-28-vertical-compensation.md)
- [2026-09-28: Integrated robustness evaluation](2026-09-28-integrated-robustness.md)
- [2026-09-28: Observation loss supervision](2026-09-28-observation-loss-supervision.md)
- [2026-09-26: Geometric startup/hover closeout](2026-09-26-geometric-transient-closeout.md)
- [2026-09-26: Source recovery audit](2026-09-26-source-recovery-audit.md)
- [2026-09-26: Geometric reimplementation](2026-09-26-geometric-reimplementation.md)
- [2026-09-26: Geometric project audit](2026-09-26-geometric-project-audit.md)
- [2026-09-25: Trajectory missions](2026-09-25-trajectory-missions.md)
- [2026-09-25: Repository audit](2026-09-25-repository-audit.md)
- [2026-09-25: Minimum snap](2026-09-25-minimum-snap.md)
- [2026-09-24: Translational feedback design](2026-09-24-translational-feedback-design.md)
- [2026-09-24: Position control and missions](2026-09-24-position-control-and-missions.md)
- [2026-09-24: Hover abort scoring](2026-09-24-hover-abort-scoring.md)
- [2026-09-24: Feedback startup diagnostic](2026-09-24-feedback-startup-diagnostic.md)
- [2026-09-24: Feedback design](2026-09-24-feedback-design.md)
- [2026-09-24: Feedback codesign](2026-09-24-feedback-codesign.md)
- [2026-09-24: Estimated state feedback](2026-09-24-estimated-state-feedback.md)
- [2026-09-24: Baseline attitude control](2026-09-24-baseline-attitude-control.md)

## State estimation

- [2026-09-28: Stationary pre-arm alignment design](2026-09-28-prearm-alignment-design.md)
- [2026-09-28: Attitude-estimation startup audit](2026-09-28-attitude-startup-audit.md)
- [2026-09-28: Observation health monitoring](2026-09-28-observation-health-monitoring.md)
- [2026-09-23: ESKF sensor replay](2026-09-23-eskf-sensor-replay.md)
- [2026-09-23: ESKF post merge audit](2026-09-23-eskf-post-merge-audit.md)
- [2026-09-23: ESKF measurement updates and contract audit](2026-09-23-eskf-measurement-updates-and-contract-audit.md)
- [2026-09-23: ESKF innovation gating](2026-09-23-eskf-innovation-gating.md)
- [2026-09-23: ESKF endpoint calibration](2026-09-23-eskf-endpoint-calibration.md)
- [2026-09-23: ESKF consistency evaluation](2026-09-23-eskf-consistency-evaluation.md)
- [2026-09-23: ESKF completion](2026-09-23-eskf-completion.md)
- [2026-09-22: ESKF prediction core](2026-09-22-eskf-prediction-core.md)

## Sensors, recorded runs and wind

- [2026-09-22: Environmental wind and drag](2026-09-22-environmental-wind-and-drag.md)
- [2026-09-22: Complete run generation](2026-09-22-complete-run-generation.md)
- [2026-09-14: Run configuration and random streams](2026-09-14-run-configuration-and-random-streams.md)
- [2026-09-11: Fixed rate sensor scheduling](2026-09-11-fixed-rate-sensor-scheduling.md)
- [2026-09-10: Local position and barometric altitude sensors](2026-09-10-local-position-and-barometric-altitude-sensors.md)
- [2026-09-09: Reproducible IMU bias random walk](2026-09-09-reproducible-imu-bias-random-walk.md)
- [2026-09-08: Reproducible accelerometer white noise](2026-09-08-reproducible-accelerometer-white-noise.md)
- [2026-09-03: Constant accelerometer bias](2026-09-03-constant-accelerometer-bias.md)
- [2026-09-02: Reproducible gyroscope white noise](2026-09-02-reproducible-gyroscope-white-noise.md)
- [2026-09-02: Constant gyroscope bias](2026-09-02-constant-gyroscope-bias.md)
- [2026-09-01: Ideal gyroscope angular velocity](2026-09-01-ideal-gyroscope-angular-velocity.md)
- [2026-09-01: Ideal accelerometer specific force](2026-09-01-ideal-accelerometer-specific-force.md)

## Dynamics, integration and initial setup

- [2026-08-31: Torque free rotation invariants](2026-08-31-torque-free-rotation-invariants.md)
- [2026-08-30: Rigid body state history validation](2026-08-30-rigid-body-state-history-validation.md)
- [2026-08-30: Gravity only mechanical energy](2026-08-30-gravity-only-mechanical-energy.md)
- [2026-08-30: Gravity only ballistic trajectory](2026-08-30-gravity-only-ballistic-trajectory.md)
- [2026-08-30: Balanced hover equilibrium](2026-08-30-balanced-hover-equilibrium.md)
- [2026-08-23: RK4 state integration](2026-08-23-rk4-state-integration.md)
- [2026-08-23: Euler RK4 simulation parity](2026-08-23-euler-rk4-simulation-parity.md)
- [2026-08-23: Euler RK4 convergence results](2026-08-23-euler-rk4-convergence-results.md)
- [2026-08-23: Deterministic RK4 simulation](2026-08-23-deterministic-rk4-simulation.md)
- [2026-08-19: Foundational dynamics and integration](2026-08-19-foundational-dynamics-and-integration.md)
- [2026-08-16: Week 02 compatibility spike](2026-08-16-week-02-compatibility-spike.md)
- [2026-08-12: Session 01](2026-08-12-session-01.md)

## Writing a verification record

Record the starting commit, the problem being addressed, the changed behavior,
the reasoning and alternatives, and the exact verification commands and results.
Keep a failing case visible and explain the conclusion it supports. End with the
remaining limitation and the next bounded action. Large generated payloads stay
outside Git; record their provenance and checksums when they support a result.
