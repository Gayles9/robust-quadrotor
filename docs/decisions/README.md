# Design decisions

These pages explain the model assumptions, interface contracts and evaluation
criteria behind the implementation. Start with the [system design](../guides/system-design.md)
for an overview, then use this index to examine a specific choice.
[Results](../results/README.md) distinguish demonstrated behavior from the limits
of each experiment.

## Foundations

- [Python workflow](0001-python-workflow.md)
- [Frames](0002-frame-conventions.md)
- [Wind and drag](0003-environmental-wind-and-drag.md)

## State estimation

- [Error-state conventions](0004-eskf-error-state-conventions.md)
- [Same-epoch measurement updates](0005-eskf-measurement-updates.md)
- [Measurement-only sensor replay](0006-eskf-sensor-replay.md)
- [Pre-update innovation diagnostics and outlier gating](0007-eskf-innovation-gating.md)
- [Aligned consistency evaluation and frozen nominal ensemble](0008-eskf-consistency-evaluation.md)
- [Estimator completion scope and frozen validation protocol](0009-eskf-completion-validation.md)
- [Endpoint integration and matched sample-noise covariance](0010-eskf-endpoint-propagation.md)
- [Causal observation health monitoring](0021-observation-health-monitoring.md)
- [Bounded attitude-estimation startup audit](attitude-startup-audit.md)
- [Stationary pre-arm alignment design and feasibility](stationary-prearm-alignment-design.md)
- [Nonlinear joint pre-arm uncertainty](nonlinear-prearm-uncertainty.md)
- [Standalone supported pre-arm alignment](standalone-prearm-alignment.md)
- [Physically supported-start flight comparison](supported-start-flight-evaluation.md)
- [Independent supported-start repeatability and fault validation](independent-supported-start-validation.md)
- [Supported velocity prior and release uncertainty gate](supported-velocity-prior.md)
- [Release-aware prediction and uncertainty gate](release-aware-prediction.md)
- [Nonlinear release uncertainty and isolated flight comparison](nonlinear-release-flight-comparison.md)
- [Combined supported velocity and nonlinear release comparison](combined-supported-velocity-comparison.md)
- [Saved combined-prior tradeoff diagnosis](combined-prior-tradeoff-diagnosis.md)
- [Whole-flight estimation/control error-budget review](whole-flight-error-budget.md)
- [Independent-inclination contract and feasibility](independent-inclination-feasibility.md)

## Feedback control

- [Bounded cascaded attitude and body-rate baseline](0011-baseline-attitude-control.md)
- [True-state position control and baseline missions](0012-position-control-and-missions.md)
- [Causal online ESKF and estimated-state mission feedback](0013-estimated-state-mission-feedback.md)
- [Joint cascade bandwidth and moment-matched initialization](0014-estimated-feedback-bandwidth.md)
- [Bounded supervision of observation loss](0022-observation-loss-supervision.md)
- [Integrated robustness evaluation](0023-integrated-robustness-evaluation.md)
- [Bounded vertical compensation](0024-bounded-vertical-compensation.md)
- [Causal early-flight diagnosis](0025-causal-early-flight-diagnosis.md)
- [Residual supported-hover diagnosis](residual-supported-hover-diagnosis.md)
- [Supported-hover navigation-feedback isolation](navigation-feedback-isolation.md)

## Planning

- [Fixed-duration minimum-snap position trajectories](0015-minimum-snap-trajectory.md)
- [Trajectory bounds, bounded timing and true-state missions](0016-trajectory-feasibility-and-missions.md)

## Geometric control

- [Geometric tracking with filtered force derivatives](0017-geometric-reimplementation.md)
- [Geometric estimator-correction study](0018-geometric-estimator-corrections.md)
- [Measured physical derivatives and their limits](0019-measured-geometric-derivatives.md)
- [One coherent feedback-force shaping experiment](0020-coherent-geometric-force.md)
- [Supported-start geometric comparison](supported-geometric-comparison.md)
- [Final bounded geometric improvement and fresh comparison](final-geometric-comparison.md)
- [True-state geometric regression adapter](geometric-regression-adapter.md)
