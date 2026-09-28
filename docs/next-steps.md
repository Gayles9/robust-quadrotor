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

## Completed: stationary pre-arm contract and feasibility design

The [contract](prearm-alignment.md) requires external evidence of mechanical
support, motors off and stationarity throughout acquisition and release. It
specifies inclination/gyro-bias estimation, retained heading and accelerometer
bias uncertainty, rejection, sample ownership and later flight regression.
The [record](progress/2026-09-28-prearm-alignment-design.md) retains all outcomes.
The allowed design scope does not establish that hardware or the existing
free-flight model supplies the required support condition.

A 0.5 s window meets the analytic noise budgets. Nominal 99th-percentile axis
error is 0.1869 degrees, with one rejection in 5,000. However, the full nonlinear
Gaussian covariance check gives maximum normalized variance 1.1122 versus the
frozen 1.10 cap; the linearized check gives 1.0622. The first-order design is
no-go for production implementation. No budgets, noise scales or duration were
tuned after the result, and no flight case was run.

## Completed: nonlinear pre-arm uncertainty derivation

The [derivation](prearm-nonlinear-uncertainty.md) propagates a fixed local Gaussian
model through nonlinear rotations and retains terminal-bias correlations using
positive quadrature. Its [verification record](progress/2026-09-28-prearm-nonlinear-uncertainty.md)
reconstructs the original failure exactly and preserves all 15,000 outcomes.
The old Gaussian set passes at 1.0614; a fresh set passes at 1.0822 against the
unchanged 1.10 cap. The paired first-order result on that fresh set is 1.1424.
No empirical inflation, new noise assumptions or duration tuning was used.

This is go for a standalone alignment component, not a demonstrated flight
improvement. The local approximation and physical support condition remain
explicit. The original first-order study stays preserved as a failed design.

## Next bounded task: standalone supported pre-arm alignment component

1. Audit this closeout, then freeze the public API and independent component
   acceptance before implementing the approved nonlinear model in the core.
2. Implement explicit acquisition/support provenance, motors-off status,
   revocation, paired sample IDs and clocks, fixed prior/profile restrictions,
   window ownership, latched rejection, uncertainty diagnostics and a one-time
   fresh-sample release. Incomplete or invalid acquisition cannot become ready.
3. Preserve the full covariance at the computed rotation mean, unchanged heading
   and accelerometer-bias information, supported bias-walk hold and fresh endpoint
   sample independence. Reject unsupported prior correlations or stale/overlapping
   handoffs. Do not substitute assumed independence for missing information.
4. Test independently against the analytic contract and the preserved numerical
   reference. Cover missing/revoked support, motors enabled, clock/sample faults,
   motion rejection, readiness boundaries, repeated release and the required
   constant-acceleration ambiguity. Retain all original uncertainty limits.

Acceptance is a tested standalone component with no production arming command
or flight integration. Then scope a separate supported-start flight evaluation
with physically modeled support, common flight noise, full original scoring
time and unchanged performance/no-regression criteria. Reserved qualification
seeds remain unopened. The cascade and endpoint ESKF stay default; mass-transient
and geometric qualification remain separate. Hardware still requires an
established support procedure and validated sensor assumptions.

Any future project-completion percentage must declare an agreed weighted
milestone denominator; campaign pass fractions are not project completion.

## Later milestones

- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
