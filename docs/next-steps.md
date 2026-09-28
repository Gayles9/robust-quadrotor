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

## Completed: bounded vertical disturbance compensation

The [single-candidate study](vertical-compensation.md) diagnoses the mass offset
against the PD equilibrium and evaluates one bounded vertical integral with
health gating, conditional anti-windup and exact trace reconstruction. The
[verification record](progress/2026-09-28-vertical-compensation.md) preserves
all twelve paired comparisons against the original authenticated baseline.

The candidate fixes the tested mass-case landing timeout and greatly reduces
its final error, but whole-flight RMSE is 17.57 cm against the original 15 cm
limit. It also fails the frozen hover no-regression condition, even though the
increase is small. The original 8 cm hover requirement remains open. Keep the
original cascade default and retain the candidate only as an explicit research
option. Do not retune it, loosen the comparison or promote a mass-offset benefit
into whole-flight qualification. Geometric qualification remains separate.

## Completed: technical report and operating boundary

The [report consolidation](technical-report.md) updates the original LaTeX
report to the current mathematical stack and evidence. It separates implemented
software, development results, unresolved flight qualification and unimplemented
integration. Source/report identities, reproduction instructions, failed studies,
controller defaults and numerical-abort limitations are explicit. The
[verification record](progress/2026-09-28-technical-report.md) records the audit.
No controller, threshold, seed or mission behavior changed.

## Completed: causal early-flight diagnosis

The [investigation](early-flight-diagnosis.md) authenticates the saved campaigns
and separates mass-load learning from the horizontal hover transient. An unfitted
scalar model reproduces both mass-case vertical trajectories to about 8 mm RMS.
One attitude-only oracle reduces the original hover peak from 10.7563 to
7.2291 cm with unchanged sensor draws and full scoring. It shows useful headroom,
but simulated truth is not an implementable controller input. The
[verification record](progress/2026-09-28-early-flight-diagnosis.md) preserves the
causal finding, limitations and explicit go/no-go.

## Completed: attitude-estimation startup audit

The [audit](attitude-startup-audit.md) reconstructs both original and oracle
full estimator histories exactly. The first position correction accounts for
0.8443 degrees of the early tilt increase; its ordinary-sized innovation passes
the existing gate. Independent joint conditioning/reset agrees to roundoff.
The [verification record](progress/2026-09-28-attitude-startup-audit.md) documents
the conservative prior, coupled corrections, covariance limits, exact recovery
of damaged evidence and no demonstrated implementation defect.

The decision is no-go for a speculative correction law under the unchanged
freely flying initialization. A stationary-gravity pseudo-measurement is not
valid during arbitrary accelerated flight. This closes the audit, not the
hover requirement, and does not prove that all measurement-only designs fail.

## Next bounded task: pre-arm alignment contract and feasibility design

Audit the startup closeout, then establish whether a mechanically supported,
nonaccelerating pre-arm interval is an allowed operating requirement. This
must be an explicit scope decision, not an assumption hidden in an estimator.

1. Specify a measurement-only alignment interface using existing IMU samples,
   explicit timing and an externally guaranteed supported-stationary condition.
   IMU-only quietness cannot prove absence of constant acceleration.
2. Derive roll/pitch and gyro-bias information, retained yaw ambiguity, and
   accelerometer-bias uncertainty. Do not invent yaw observability or supply
   simulated attitude as an input. Do not choose a duration from the failed seed.
3. Freeze independent analytic/noisy tests, sample reuse/cross-covariance rules,
   motion rejection, failure behavior and a quantitative initialization-error
   budget before implementation. Check whether the required guarantee is usable.
4. Define later full-flight regression with the added pre-arm stage disclosed,
   all original flight time and limits retained, and original free-flight results
   still labelled separately. No qualification claim follows from this design.

Acceptance is a feasible, explicit contract with independent verification
criteria, or a justified stop if the required operating information is absent.
Do not integrate it into flight yet. If supported stationarity is not allowed,
request a different independent attitude-information source or an explicit
startup-requirement redesign. No gain/cutoff sweep, reserved seeds, scoring
shift, threshold relaxation or geometric reopening is authorized. The original
cascade and endpoint ESKF remain default; the separate mass transient stays open.

Any future project-completion percentage must declare an agreed weighted
milestone denominator; campaign pass fractions are not project completion.

## Later milestones

- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
