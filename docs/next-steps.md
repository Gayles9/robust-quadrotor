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

## Completed: integrated robustness evaluation

The [frozen campaign](integrated-robustness.md) executes twelve paired cases
with original cascade parameters and known seeds. It retains actual hover,
translation and virtual landing, causal position/altitude faults, recovery,
wind and mass mismatch. Each mode's full saved evidence is authenticated and
reconstructed before response and flight criteria are scored separately.
The [verification record](progress/2026-09-28-integrated-robustness.md) preserves
all outcomes, including the failed hover peak and mass-mismatch landing.

## Next bounded task: vertical disturbance rejection

Audit the integrated evaluation and diagnose the 10% mass-mismatch case from
its saved histories. Compare the measured vertical offset with the existing
PD controller's equilibrium prediction; check estimator error, motor/thrust
limits and the complete takeoff/landing response. Review the earlier bounded
integral probes before proposing a new mechanism. They did not close the
startup/hover gap and must not be presented as an already-qualified remedy.

Freeze at most one justified, explicitly experimental vertical disturbance-
compensation candidate, with defined state, bounds, anti-windup, initialization,
reset and observation-loss behavior. It may use causal measured/estimated
feedback and commanded quantities, never the simulator's true mass or fault
labels. Preserve the original default and compare the candidate against the
saved original cascade under unchanged seeds, flight limits and fault policy.

Require a complete mass-mismatch mission within the original campaign's error
limits, and no regression in nominal, recovery, wind or timed-fault behavior.
Retain the known hover-peak failure unless a separately justified change actually
passes its unchanged requirement. A mass-offset improvement cannot qualify the
controller as a whole. Do not run a gain sweep, relax the landing tolerance,
reopen reserved qualification seeds or silently promote an experimental option.

Conclude with complete authenticated evidence, full software checks, a clear
accept/reject decision and the next bounded action. Geometric-controller
qualification and a physical fallback maneuver remain separate scopes.

## Later milestones

- Update the technical report from verified source and results, including the
  controller operating boundary and failed studies.
- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
