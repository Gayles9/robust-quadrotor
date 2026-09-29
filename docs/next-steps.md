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

## Completed: standalone supported pre-arm alignment

The [component](prearm-component.md) implements the fixed nonlinear model,
externally supplied support evidence, paired sample/clock ownership, restricted
priors, latched rejection, diagnostics and one-time fresh endpoint handoff.
The [verification record](progress/2026-09-28-prearm-component.md) preserves
all 15,000 reference comparisons and 200 complete acquisition/release sessions.
Covariance and complete-session compatibility statistics reproduce exactly.
Constant acceleration remains unobservable without genuine external support.
No arming command or flight integration was added.

## Completed: physically supported-start flight evaluation

The [frozen comparison](supported-start-flight.md) executes eight flights across
hover, nominal translation, wind and mass mismatch. An explicit fixture holds
zero motion with motors off, supplies the correct gravity/drag reaction, and
releases only after the fresh pre-arm sample. The supported unaligned and
aligned arms share physical startup and all flight random draws. The original
moving/spinning baseline remains a separately authenticated comparator.

The [verification record](progress/2026-09-28-supported-start-flight.md) retains
all outcomes and complete saved replay. Aligned hover peak is 2.506 cm against
8.588 cm in the identical supported release without alignment; the original
baseline is 10.756 cm. The unchanged 8 cm hover limit passes. Nominal and wind
tracking RMSE improve, and their original conditions pass. Mass mismatch still
fails: aligned whole-flight RMSE is 42.526 cm with landing timeout at 25 s.
The frozen startup comparison passes, while all-four-case flight qualification
does not. The initializer remains experiment-only.

## Completed: independent supported-start validation

The [independent campaign](independent-supported-start.md) freezes three fresh
seeds across hover, nominal tracking and wind, plus the eight original fault
patterns under a fourth seed. Clean pairs isolate alignment; fault pairs isolate
supervision with the same aligned prior. Full histories, original pre-fault
sensor records and all individual outcomes are preserved in the
[verification record](progress/2026-09-28-independent-supported-start.md).

All three fresh hover peaks improve, but one remains at 10.18 cm against the
unchanged 8 cm limit. The other two reach 5.46 and 5.24 cm. All six nominal/wind
tracking cases pass their flight conditions. The integration decision is no-go:
repeatable improvement is not the same as satisfying every required condition.
All eight observation-fault response comparisons pass with the original timing
rules; recovery completes and persistent faults produce the required numerical
abort. All 34 planned histories are retained.
Keep the initializer experiment-only and retain the original controller default.

## Completed: residual supported-hover diagnosis

The [offline diagnosis](residual-supported-hover.md) authenticates all previous
scores and reconstructs all six full hover histories. Saved navigation-error
forcing accounts for -7.729 cm of the failed peak's -9.693 cm East displacement.
The direct initial-state/first-0.5-s response is only 0.796 mm in norm there.
An unfitted sampled cascade model reproduces the failed flight to 0.761 mm
horizontal RMS; across all six histories the range is 0.169..1.389 mm.
The [record](progress/2026-09-28-residual-supported-hover.md) retains the signed
accounting, bias errors, local-model assumptions and exact reconstruction checks.
The step succeeds as a diagnosis, while the 10.18 cm hover failure remains.

## Completed: navigation-feedback isolation

The [single oracle](navigation-feedback-isolation.md) changes only the failed
aligned seed47001 flight's outer position/velocity inputs to simultaneous truth.
Hover peak falls from 10.1808 to 1.9896 cm (80.46%); whole-flight RMSE falls from
6.4891 to 3.9256 cm (39.50%). Both flights complete at 15.5 s. The
[record](progress/2026-09-28-navigation-feedback-isolation.md) preserves complete
saved reconstruction, named-noise pairing and every metric. The frozen headroom
comparison passes. The result is diagnostic and leaves ordinary flight
qualification open; it does not identify separate p-only and v-only effects.

## Completed: supported velocity prior and release screen

The [prior-only candidate](supported-velocity-prior.md) performs exact one-time
Gaussian conditioning on genuinely stationary support. Its algebra, rejection
paths, PSD/rank and full covariance checks pass. All 17 existing release
configurations fail the mandatory uncertainty screen: the analytic ballistic
case has a 1.22625 cm/s down-velocity error, about 119 candidate standard
deviations. The supported left-limit sample is being averaged across a force
discontinuity. The [record](progress/2026-09-28-supported-velocity-prior.md) retains
all outcomes. Under the predeclared stopping rule, zero scientific flights run
and all 25 conditional regression flights are skipped. The investigation succeeds
and rejects this combination of prior and release map.

## Next bounded task: release-aware first prediction interval

1. Audit this closeout and freeze one causal boundary rule, its uncertainty
   contract and acceptance. Preserve the genuinely stationary support and the
   fresh supported sample; do not assume stationarity after release.
2. Derive a one-sided first-interval prediction from the first post-release IMU
   measurement, with the existing gyro endpoints and full sample-noise memory.
   The left supported force cannot represent an open free-flight interval. Do
   not invent a second simultaneous sample or read simulated true velocity/force.
3. Check the exact ballistic limit, smooth-force consistency and bounded numerical
   error at the chosen time step. Derive the complete discrete Jacobians and covariance;
   verify them independently, including singular initial velocity uncertainty,
   fresh-noise ownership and any changed cross terms. Reject overconfidence rather
   than tune Q/R or add a covariance floor. Keep later prediction intervals intact.
4. Implement only the derived experiment boundary after those checks. Freeze a
   comparison that isolates the boundary change before combining it with velocity
   conditioning. Preserve all known hover seeds, clean/fault conditions, full
   startup and original thresholds; require fresh validation before integration.
   Stop at the frozen decision and publish it, including an unsuccessful result.

Correcting this small interval is a numerical/uncertainty prerequisite. It does
not guarantee recovery of the oracle's whole-flight improvement or establish that
velocity alone explains its combined position/velocity-feedback effect.

The mass-induced offset remains a separate open requirement. Do not silently
combine the rejected integral candidate, reopen geometric tuning, claim hardware
stationarity, run a new gain/seed sweep or integrate alignment into normal
missions while the fresh-seed hover requirement remains unresolved.

Any future project-completion percentage must declare an agreed weighted
milestone denominator; campaign pass fractions are not project completion.

## Later milestones

- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
