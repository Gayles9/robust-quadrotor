# Verification records

These dated records preserve the implementation decisions, calculations,
commands, failures and measured results for each milestone. Start with
[current status](../status-before-overhaul.md) for what is supported today and the
[project review](../repository-audit.md) for the latest repository audit.

A record's test count, publication state and next action describe that session.
They do not override the [current plan](../previous-plan.md). Historical workstation
and scratch paths have been replaced with portable example paths; numerical
results, seeds, thresholds and commit identities are preserved.

## Report and operating boundary

- [2026-09-29: Original-sensor operating envelope and report revision 3.0](operating-envelope-report.md)
- [2026-09-30: Final geometric improvement and comparison](final-geometric-comparison.md)
- [2026-09-29: Supported-start geometric comparison](supported-geometric-comparison.md)

## Controller and planning studies

- [2026-09-29: Saved combined-prior tradeoff diagnosis](combined-prior-diagnosis.md)

- [2026-09-29: Combined supported velocity and nonlinear release](combined-supported-prior.md)

- [2026-09-29: Nonlinear release uncertainty and isolated flight comparison](nonlinear-release.md)
- [2026-09-29: Release-aware prediction and uncertainty](release-aware-prediction.md)
- [2026-09-28: Supported velocity prior and release uncertainty](supported-velocity-prior.md)

- [2026-09-28: Causal early-flight diagnosis](early-flight-diagnosis.md)
- [2026-09-28: Technical report and operating boundary](technical-report.md)
- [2026-09-28: Bounded vertical compensation](vertical-compensation.md)
- [2026-09-28: Integrated robustness evaluation](integrated-robustness.md)
- [2026-09-28: Observation loss supervision](observation-loss-supervision.md)
- [2026-09-26: Geometric startup/hover closeout](geometric-transient-closeout.md)
- [2026-09-26: Source recovery audit](source-recovery-audit.md)
- [2026-09-26: Geometric reimplementation](geometric-reimplementation.md)
- [2026-09-26: Geometric project audit](geometric-project-audit.md)
- [2026-09-25: Trajectory missions](trajectory-missions.md)
- [2026-09-25: Repository audit](repository-audit.md)
- [2026-09-25: Minimum snap](minimum-snap.md)
- [2026-09-24: Translational feedback design](translational-feedback-design.md)
- [2026-09-24: Position control and missions](position-control-and-missions.md)
- [2026-09-24: Hover abort scoring](hover-abort-scoring.md)
- [2026-09-24: Feedback startup diagnostic](feedback-startup-diagnostic.md)
- [2026-09-24: Feedback design](feedback-design.md)
- [2026-09-24: Feedback codesign](../../progress/2026-09-24-feedback-codesign.md)
- [2026-09-24: Estimated state feedback](estimated-state-feedback.md)
- [2026-09-24: Baseline attitude control](baseline-attitude-control.md)

## State estimation

- [2026-09-28: Independent supported-start validation](independent-supported-start.md)
- [2026-09-28: Residual supported-hover diagnosis](residual-supported-hover.md)
- [2026-09-28: Navigation-feedback isolation](navigation-feedback-isolation.md)
- [2026-09-28: Supported-start flight comparison](supported-start-flight.md)
- [2026-09-28: Standalone supported pre-arm component](prearm-component.md)
- [2026-09-28: Nonlinear pre-arm uncertainty](prearm-nonlinear-uncertainty.md)
- [2026-09-28: Stationary pre-arm alignment design](prearm-alignment-design.md)
- [2026-09-28: Attitude-estimation startup audit](attitude-startup-audit.md)
- [2026-09-28: Observation health monitoring](observation-health-monitoring.md)
- [2026-09-23: ESKF sensor replay](eskf-sensor-replay.md)
- [2026-09-23: ESKF post merge audit](eskf-post-merge-audit.md)
- [2026-09-23: ESKF measurement updates and contract audit](eskf-measurement-updates-and-contract-audit.md)
- [2026-09-23: ESKF innovation gating](eskf-innovation-gating.md)
- [2026-09-23: ESKF endpoint calibration](eskf-endpoint-calibration.md)
- [2026-09-23: ESKF consistency evaluation](eskf-consistency-evaluation.md)
- [2026-09-23: ESKF completion](eskf-completion.md)
- [2026-09-22: ESKF prediction core](eskf-prediction-core.md)

## Sensors, recorded runs and wind

- [2026-09-22: Environmental wind and drag](environmental-wind-and-drag.md)
- [2026-09-22: Complete run generation](complete-run-generation.md)
- [2026-09-14: Run configuration and random streams](run-configuration-and-random-streams.md)
- [2026-09-11: Fixed rate sensor scheduling](fixed-rate-sensor-scheduling.md)
- [2026-09-10: Local position and barometric altitude sensors](local-position-and-barometric-altitude-sensors.md)
- [2026-09-09: Reproducible IMU bias random walk](reproducible-imu-bias-random-walk.md)
- [2026-09-08: Reproducible accelerometer white noise](reproducible-accelerometer-white-noise.md)
- [2026-09-03: Constant accelerometer bias](constant-accelerometer-bias.md)
- [2026-09-02: Reproducible gyroscope white noise](reproducible-gyroscope-white-noise.md)
- [2026-09-02: Constant gyroscope bias](constant-gyroscope-bias.md)
- [2026-09-01: Ideal gyroscope angular velocity](ideal-gyroscope-angular-velocity.md)
- [2026-09-01: Ideal accelerometer specific force](ideal-accelerometer-specific-force.md)

## Dynamics, integration and initial setup

- [2026-08-31: Torque free rotation invariants](torque-free-rotation-invariants.md)
- [2026-08-30: Rigid body state history validation](rigid-body-state-history-validation.md)
- [2026-08-30: Gravity only mechanical energy](gravity-only-mechanical-energy.md)
- [2026-08-30: Gravity only ballistic trajectory](gravity-only-ballistic-trajectory.md)
- [2026-08-30: Balanced hover equilibrium](balanced-hover-equilibrium.md)
- [2026-08-23: RK4 state integration](rk4-state-integration.md)
- [2026-08-23: Euler RK4 simulation parity](euler-rk4-simulation-parity.md)
- [2026-08-23: Euler RK4 convergence results](euler-rk4-convergence-results.md)
- [2026-08-23: Deterministic RK4 simulation](deterministic-rk4-simulation.md)
- [2026-08-19: Foundational dynamics and integration](foundational-dynamics-and-integration.md)
- [2026-08-16: Week 02 compatibility spike](px4-gazebo-compatibility.md)
- [2026-08-12: Session 01](project-start.md)

## Writing a verification record

Record the starting commit, the problem being addressed, the changed behavior,
the reasoning and alternatives, and the exact verification commands and results.
Keep a failing case visible and explain the conclusion it supports. End with the
remaining limitation and the next bounded action. Large generated payloads stay
outside Git; record their provenance and checksums when they support a result.
