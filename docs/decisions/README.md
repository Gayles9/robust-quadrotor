# Design decisions

These records explain why I chose an interface, model or validation protocol,
what alternatives I considered, and what would justify changing the decision.
They preserve the reasoning at the time. Use [current status](../status.md) and
the [next-step plan](../next-steps.md) for the present scope.

## Foundations

- [0001: Python workflow](0001-python-workflow.md)
- [0002: Frames](0002-frame-conventions.md)
- [0003: Wind and drag](0003-environmental-wind-and-drag.md)

## State estimation

- [0004: Error-state conventions](0004-eskf-error-state-conventions.md)
- [0005: Same-epoch measurement updates](0005-eskf-measurement-updates.md)
- [0006: Measurement-only sensor replay](0006-eskf-sensor-replay.md)
- [0007: Pre-update innovation diagnostics and outlier gating](0007-eskf-innovation-gating.md)
- [0008: Aligned consistency evaluation and frozen nominal ensemble](0008-eskf-consistency-evaluation.md)
- [0009: Estimator completion scope and frozen validation protocol](0009-eskf-completion-validation.md)
- [0010: Endpoint integration and matched sample-noise covariance](0010-eskf-endpoint-propagation.md)
- [0021: Causal observation health monitoring](0021-observation-health-monitoring.md)
- [0026: Bounded attitude-estimation startup audit](0026-attitude-startup-audit.md)
- [0027: Stationary pre-arm alignment design and feasibility](0027-stationary-prearm-alignment-design.md)
- [0028: Nonlinear joint pre-arm uncertainty](0028-nonlinear-prearm-uncertainty.md)
- [0029: Standalone supported pre-arm alignment](0029-standalone-prearm-alignment.md)
- [0030: Physically supported-start flight comparison](0030-supported-start-flight-evaluation.md)
- [0031: Independent supported-start repeatability and fault validation](0031-independent-supported-start-validation.md)

## Feedback control

- [0011: Bounded cascaded attitude and body-rate baseline](0011-baseline-attitude-control.md)
- [0012: True-state position control and baseline missions](0012-position-control-and-missions.md)
- [0013: Causal online ESKF and estimated-state mission feedback](0013-estimated-state-mission-feedback.md)
- [0014: Joint cascade bandwidth and moment-matched initialization](0014-estimated-feedback-bandwidth.md)
- [0022: Bounded supervision of observation loss](0022-observation-loss-supervision.md)
- [0023: Integrated robustness evaluation](0023-integrated-robustness-evaluation.md)
- [0024: Bounded vertical compensation](0024-bounded-vertical-compensation.md)
- [0025: Causal early-flight diagnosis](0025-causal-early-flight-diagnosis.md)
- [0032: Residual supported-hover diagnosis](0032-residual-supported-hover-diagnosis.md)
- [0033: Supported-hover navigation-feedback isolation](0033-navigation-feedback-isolation.md)

## Planning

- [0015: Fixed-duration minimum-snap position trajectories](0015-minimum-snap-trajectory.md)
- [0016: Trajectory bounds, bounded timing and true-state missions](0016-trajectory-feasibility-and-missions.md)

## Geometric control

- [0017: Geometric reimplementation](0017-geometric-reimplementation.md)
- [0018: Geometric estimator-correction study](0018-geometric-estimator-corrections.md)
- [0019: Measured physical derivatives and development rejection](0019-measured-geometric-derivatives.md)
- [0020: One coherent feedback-force shaping experiment](0020-coherent-geometric-force.md)

## Writing a new decision

Use the next number and a descriptive filename, such as `0034-short-title.md`.
Explain the problem before the equations. State the evidence needed to accept
the change and the limits that remain afterward.

```markdown
# NNNN: Title

- Date:
- Context:
- Options:
- Decision:
- Reason:
- Consequences/Risks:
- Revisit Trigger:
```
