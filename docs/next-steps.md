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

## Completed: bounded observation loss supervision

The [supervisor](observation-supervision.md) applies explicit, independent
unhealthy-time budgets and a latched numerical abort. Local position is required
in all active phases; altitude can be explicitly optional. Recovery must finish
before expiry, phase changes preserve the timers, existing safety guards retain
priority, and terminal epochs issue no command. The
[verification record](progress/2026-09-28-observation-loss-supervision.md)
documents component fault injection, closed-loop regression and saved-evidence
reconstruction. This closes the bounded response-interface step, not general
robust-flight qualification or a hardware fallback policy.

## Next bounded task: integrated robustness evaluation

Audit the monitor/supervisor milestone, then freeze a small campaign before
executing it. Cover actual hover, tracking and virtual landing with nominal
sensors, independent position/altitude dropout, rejection and delay, followed
by explicitly declared wind and mass mismatch. Define fault timing, durations,
seeds, health thresholds, response budgets and success criteria in advance.

Start with the unchanged cascade default and a matched supervision-off
comparator for each case. Use a causal live fault-delivery boundary; retain an
exhaustive fault ledger outside the estimator and supervisor. Do not infer
closed-loop dropout performance from the current component-only dropout tests.
Archive complete sensor/estimate/command histories, authenticate them, and
reconstruct every health transition and abort decision.

Measure nominal false aborts, fault detection and response times, accepted-data
age, command cutoff, tracking/estimation error over the common horizon, and
completion or abort reason. Keep all comparator failures. Distinguish a correct
timed abort from successful mission completion, and separate simulation truth
guards from observation-driven decisions. A passing short stationary fixture
does not demonstrate acceptable maneuver performance.

Conclude whether the declared policy meets that frozen campaign. If it fails,
retain the failed cases and propose a separate evidence-based change; do not
retune controllers, budgets or acceptance limits within the evaluation. Full
software checks and authenticated evidence remain required. Automatic
alignment, new sensors, estimator/controller tuning and ROS/PX4 adapters are
separate scopes.

## Later milestones

- Update the technical report from verified source and results, including the
  controller operating boundary and failed studies.
- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
