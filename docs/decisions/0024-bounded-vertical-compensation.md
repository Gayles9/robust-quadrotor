# ADR 0024: one bounded vertical disturbance-compensation candidate

Status: frozen before implementation and candidate flights, 2026-09-28.
Outcome: implemented and evaluated; candidate rejected. The
[complete record](../progress/2026-09-28-vertical-compensation.md) retains all
24 executions, 12 passing response cases, and the mass-RMSE and hover comparison
failures. The frozen policy and acceptance conditions below are unchanged.

## Audit and diagnosis

Audited main is `0f1702f080690b572bafee819deb6522acaadf25`, tree
`d1814de6b03cfa3bd8ff930ea00609737bbe7372`; its post-merge CI passed.
Fresh live-fault, evidence and position-control checks pass 163 tests in 62.80 s.
The saved mass pair authenticates and reconstructs exactly. No preceding defect
requires repair. Its report SHA-256 is
`ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09`.

The 1.1 kg truth / 1.0 kg nominal case has mean vertical error 0.436975 m over
17..25 s; the last two seconds' mean estimated-minus-true vertical position is
only 0.001055 m. There are zero outer or inner limit flags. Commanded thrust
ranges 9.81..11.317405 N. The nominal PD equilibrium predicts
`(1.1/1.0 - 1)*9.81/2.25 = 0.436 m`, matching the persistent offset.

The earlier bounded-integral prototype targeted horizontal startup response
and reached its 0.5 m/s² bound while failing the hover requirement. This task
does not revive that architecture or repeat its gain search. It addresses the
newly measured vertical steady-force deficit with one independently specified
scalar state. Estimator, references, PD gains and geometric control stay fixed.

## Candidate and state contract

Add an explicit opt-in vertical integral acceleration `I` in NED m/s²:
`a_command = a_PD + [0, 0, I]`, with `I_dot = 0.5*(z_reference-z_estimate)`.
Keep the existing position-control feasibility projection and all bounds.
The integral gain is **0.5 s^-3**, update period **0.02 s**, and state bound
**±1.5 m/s²**. Initialize/reset to zero. No mass or fault label is an input.

The gain follows `(s+0.5)^2*(s+2) = s^3+3s^2+2.25s+0.5` for the nominal
unsaturated vertical model with the existing PD gains. The 1.1 kg case with
0.025 s linear motor lag has continuous poles approximately -37.1202, -1.8277,
-0.6195 and -0.4326 s^-1. This is a design rationale, not a nonlinear proof.
The state bound exceeds the required -0.981 m/s² steady correction without
changing the existing ±2 m/s² acceleration limit. No gain sweep is permitted.

At each outer epoch, apply the current state to the command, then prepare the
next state using forward Euler and the current estimated error. Learning is
allowed only when both observation streams are HEALTHY. WAITING, DEGRADED,
LOST and RECOVERING freeze the state; confirmed recovery resumes learning.
Retain compensation across phase changes and loss, avoiding an abrupt reset.
Terminal mission guards execute before controller calls. An attitude-domain
abort may retain a computed-but-unused outer attempt, explicitly diagnostic.

Freeze integration when the previous inner command had rate, moment or
allocation limiting. At an outer acceleration/thrust limit, reject only an
increment that drives further into the vertical limit; allow unwinding.
Use 1e-12 m/s² only to distinguish a projected vertical limit from roundoff.
Finally clip the proposed integral state to its declared bound. This is bounded
conditional integration, not a complete actuator-dynamics anti-windup proof.
Validate calls atomically; reject wrong clocks, nonfinite inputs, reused state,
controller mismatch, disabled/missing health streams and geometric combination.

Keep mission and saved-history dataclasses unchanged. Record applied/next
integral state, error, health/limit flags and update reason in a separate
authenticated trace. Reconstruct it and every affected command on replay.

## Frozen evaluation and acceptance

Reuse all twelve ADR 0023 cases, both supervision modes, original seeds, priors,
plant/sensor clocks, fault plans, references, controller gains, response budgets
and flight limits. Run **24 new candidate executions**, with at most two process
workers and single-threaded BLAS. Compare with the immutable, already audited
24-execution baseline above; authenticate all baseline bytes again and retain
its full report and evidence identity. Do not silently generate a new baseline.

Candidate acceptance requires zero numerical/evidence failures, all twelve
inherited response checks, mass-case completion within every inherited flight
limit, and preservation of the existing passing nominal-tracking, recovery and
wind flight gates. For the already failing hover, retain every previously
passing flight condition and require its peak not to exceed the baseline peak
(1e-12 m arithmetic tolerance). Its unchanged 8 cm gate remains a failure until
actually passed. Report every metric delta; retaining an inherited pass is not
a claim that every scalar metric improved. Require bounded compensation,
correct freeze/unwind/reset behavior and no terminal command.

The canonical campaign protocol SHA-256 is
`6bab1e089730485310a9d69a3d49361618c73982663b75f2a9efe8fc9fa9635e`,
anchored in tests before the candidate maneuver campaign.

The candidate remains an explicit experimental option even if this vertical
scope passes. Whole-flight qualification, fresh seeds, a default-controller
change and physical fallback remain separate decisions. A failure ends this
single-candidate study with its evidence retained; do not retune or relax limits.

Software acceptance includes independent sign/step/anti-windup tests, strict
causal and invalid-call checks, unchanged default payloads, exact saved ESKF,
health, mission, controller and compensation-trace reconstruction, rejection of
changed protocol/claims/trace, and retained numerical failures. Use short smoke
fixtures for CI, not as flight qualification. Run the complete warning-strict
software gate and hosted CI before merging. Save full evidence outside Git.
