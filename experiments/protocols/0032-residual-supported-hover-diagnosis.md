# ADR 0032: residual supported-hover diagnosis

Status: protocol and acceptance frozen before analysis/implementation, 2026-09-28.

## Preceding audit

Audit main `5e532ab1f8b991643aff7628ea59d69d2915cacc` (PR 30), equal to the
live GitHub main reference. Fresh warning-strict supported-start, independent
validation, robustness and documentation checks pass 59 tests in 45.59 s.
The independent archive verifier authenticates all 234 payloads and reconstructs
34 flight scores and 17 decisions. Review of the saved-history contracts,
controller equations and published closeout finds no demonstrated defect.
The fresh-seed hover failure remains a valid failure; software success does not
override it.

## Fixed scope

Perform an offline diagnosis of the six saved nominal-hover histories at seeds
47001, 47002 and 47003, both supported unaligned and aligned. Seed 47001 aligned
is the primary failed case; the others are descriptive comparisons. Authenticate
the complete campaign before reading diagnostics, with input report SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Retain complete histories, the inclusive 5..11 s hover window, 0.08 m hover and
0.15 m whole-flight/final-position limits, and original speed/completion rules.
No new scientific flight, oracle, gain/seed search or controller change is in scope.

Reconstruct saved commands from the original gains and estimated state, and
each physical plant/motor interval. Separate true tracking, estimated tracking,
navigation error, thrust-axis estimation/tracking, bias error, motor and drag
effects. Derive a sampled linear PD response decomposition with the original
gains and clocks, retaining reference forcing, initial state, navigation forcing,
command limits, attitude estimation/tracking, mass, motor, drag and within-step
integration residuals. The sum must reproduce the full saved physical trajectory.
Report signed vectors/contributions: norms are not additive percentages.

The acceleration terms are an explicitly ordered identity. Their linear response
is pathwise accounting with all forcing histories frozen, not a nonlinear
intervention that removes an estimator or controller error. Correlated feedback
terms may cancel. Do not present component subtraction as an achievable flight.
Use 0..0.5 s as a fixed descriptive release interval, reporting its propagated
contribution alongside later forcing; this does not isolate physical support's
counterfactual effect. Support is absent after release in all saved flights.

If a small-angle sampled cascade model is used to interpret oscillation, derive
its coefficients from existing gains, inertia, motor lag and clocks only. Do not
fit coefficients to the failure. Check agreement and disclose neglected terms;
eigenvalues of an approximate model are not nonlinear stability qualification.

## Acceptance and decision

1. Authenticate all referenced campaign payloads and independently recover all
   34 original scores/decisions; preserve the six full diagnostic histories.
2. Reconstruct controller commands to 1e-12 and plant/motor steps to 1e-12;
   acceleration closure to 1e-11 m/s² and full response closure to 1e-10 m and
   m/s. A residual outside these bounds requires investigation, not tuning.
3. Verify the diagnostic algebra with independent analytic/synthetic cases,
   including NED signs, sample holds, impulse response, correlated cancellation,
   support-release timing and failure-preserving scoring. Pass relevant tests,
   repository quality gates and hosted CI before publication.
4. State which mechanisms quantitatively explain the observed failure and which
   remain coupled/unidentified. Support one concrete next experiment only if
   justified; rejecting speculative implementation is a successful diagnosis.
5. Preserve evidence outside Git and publish the scope, equations, actual
   outcomes, limits and exact next action. Keep normal mission integration,
   geometric qualification, mass compensation and hardware readiness open.

This step cannot close the hover gate because it changes no flight behavior.
