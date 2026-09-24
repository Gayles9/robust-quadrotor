# Decision Records

Record architectural or version changes here as numbered decision records, such as `0001-short-title.md`.

## ESKF contracts

- [0004: Error-state conventions](0004-eskf-error-state-conventions.md)
- [0005: Same-epoch measurement updates](0005-eskf-measurement-updates.md)
- [0006: Measurement-only sensor replay](0006-eskf-sensor-replay.md)
- [0007: Pre-update innovation diagnostics and outlier gating](0007-eskf-innovation-gating.md)
- [0008: Aligned consistency evaluation and frozen nominal ensemble](0008-eskf-consistency-evaluation.md)
- [0009: Estimator completion scope and frozen validation protocol](0009-eskf-completion-validation.md)
- [0010: Endpoint integration and matched sample-noise covariance](0010-eskf-endpoint-propagation.md)

## Control contracts

- [0011: Bounded cascaded attitude and body-rate baseline](0011-baseline-attitude-control.md)
- [0012: True-state position control and baseline missions](0012-position-control-and-missions.md)
- [0013: Causal online ESKF and estimated-state mission feedback](0013-estimated-state-mission-feedback.md)

## Template

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
