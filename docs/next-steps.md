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

## Next bounded task: attitude-estimation startup audit

Audit the diagnostic closeout and reuse the authenticated original and oracle
hover histories. Do not generate another controller trial first.

1. Reconstruct roll/pitch error and covariance through the first second and the
   subsequent approach to the hover peak. Separate IMU propagation from each
   position/altitude correction; inspect coupled velocity and bias corrections.
2. Explain why thrust-axis estimation error grows from 2.2941 degrees initially
   to 3.1614 degrees at 0.2 s. Check innovation timing, Jacobian/reset signs and
   covariance consistency against the declared initial prior and sensor model.
   Fix a demonstrated in-scope defect if found; do not infer one from a failed
   flight score alone.
3. Assess which attitude information is available causally from existing
   measurements during this freely flying initialization. Do not assume a
   stationary gravity measurement, supply true attitude, silently tighten the
   prior, change sensors or tune process/measurement noise to the observed run.
4. Return one justified measurement-only correction design and freeze its
   interface, independent tests and original full-flight regression acceptance
   before implementation, or record a justified no-go and the missing information.

Acceptance is an auditable estimator explanation and an implementable bounded
design or justified stop. There is no gain/cutoff search, reserved seed access,
startup removal, scoring shift, threshold relaxation or geometric reopening.
The present step does not authorize implementing a flight-controller candidate.
The separate mass transient remains open; attitude improvement must not be
represented as a solution to the measured vertical force deficit.

Any future project-completion percentage must declare an agreed weighted
milestone denominator; campaign pass fractions are not project completion.

## Later milestones

- Implement custom ROS 2/PX4 integration under separate acceptance criteria.

The reserved geometric qualification seeds remain unopened. Reopening controller
design needs a new, evidence-based scope; another gain or cutoff search is not
the automatic next action.
