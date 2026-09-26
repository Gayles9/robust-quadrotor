# Next-step plan

The bounded startup/hover study is closed. Its single coherent-force candidate
reduced spline effort and passed the paired spline comparison, but failed the
unchanged full-hover requirement. I retained the cascade default and original
geometric implementation. The [verification record](progress/2026-09-26-geometric-transient-closeout.md)
explains the diagnosis and rejection. Estimated-feedback qualification remains
open; closing a tuning study does not close that flight gate.

## Next bounded task: observation health monitoring

Define and implement a small, deterministic health-monitoring interface around
the existing estimator event diagnostics. The immediate goal is to distinguish
an isolated rejected observation from persistent loss of usable position or
altitude information. It should report evidence for a later supervisor policy.

Start by auditing the event statuses, acquisition/delivery timestamps and mission
guards against current source and tests. Then freeze the interface and transition
rules before implementing them. Keep local-position and barometric streams
separate: losing one does not mean the other has stopped working.

The interface should expose the age of the last accepted observation, recent
acceptance/rejection evidence and explicit health transitions. It must use only
causally available events and the current clock. Thresholds must be explicit,
related to the configured sensor schedules and checked at their boundaries.
Sensor freshness alone must not be described as proof of estimator accuracy or
observability.

Acceptance requires deterministic tests for nominal delivery, one outlier,
repeated rejection, missing observations, recovery, exact time boundaries,
invalid ordering and reset. Integration must leave existing estimator estimates,
controller commands and mission outcomes unchanged when monitoring is enabled.
Existing software/documentation checks and saved-evidence authentication remain
required.

This task does not add automatic startup alignment, new sensors, estimator
retuning, a degraded-flight policy or ROS/PX4 adapters. A later explicit design
will decide what action a supervisor takes when information remains unavailable.

## Later milestones

- Define and validate supervisor responses to persistent health degradation.
- Run broader integrated evaluation with faults and model mismatch under a
  frozen protocol, preserving comparator failures and the open flight limits.
- Update the technical report from verified source and results, including the
  controller operating boundary and failed studies.
- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
