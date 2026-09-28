# Next-step plan

The bounded startup/hover study is closed. Its single coherent-force candidate
reduced spline effort and passed the paired spline comparison, but failed the
unchanged full-hover requirement. I retained the cascade default and original
geometric implementation. The [verification record](progress/2026-09-26-geometric-transient-closeout.md)
explains the diagnosis and rejection. Estimated-feedback qualification remains
open; closing a tuning study does not close that flight gate.

## Completed: observation health monitoring

The [monitor](observation-health.md) reports accepted-observation age, recent
dispositions and explicit loss/recovery transitions separately for position and
altitude. Its optional mission integration is passive. Complete mission and
saved-payload equality, causal reconstruction, timing boundaries, recovery and
invalid-input behavior are covered by the
[verification record](progress/2026-09-28-observation-health-monitoring.md).

## Next bounded task: supervisor responses to persistent information loss

Audit the monitor and existing mission transition/abort logic, then freeze a
small response policy before implementing it. Define which observations each
mission phase requires, how long a degraded condition may persist, and when
tracking should be stopped. Treat position and altitude independently, and
distinguish explicitly disabled fusion from an unexpected observation loss.

The design must account for what the current estimator can observe. A fresh
altitude stream does not establish horizontal or heading accuracy, and the
simulation's truth safety monitor is not available as an onboard input. Choose
actions supported by the existing mission/controller interfaces; explain any
abort or virtual landing semantics without claiming a hardware safety policy.

Acceptance should cover nominal equivalence, one rejected observation,
persistent rejection, missing/delayed data, independent stream loss, recovery,
phase boundaries and terminal command behavior. Inject faults under an explicit
deterministic protocol. Verify that decisions depend only on causally available
estimator and health information, retain all failures, and keep saved-evidence
authentication and the complete software gate.

Automatic alignment, new sensors, estimator/controller tuning and ROS/PX4
adapters remain separate scopes. Availability thresholds from the monitor's
example are not automatically suitable action deadlines.

## Later milestones

- Run broader integrated evaluation with faults and model mismatch under a
  frozen protocol, preserving comparator failures and the open flight limits.
- Update the technical report from verified source and results, including the
  controller operating boundary and failed studies.
- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
