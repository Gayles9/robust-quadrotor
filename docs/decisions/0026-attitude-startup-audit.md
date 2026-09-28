# ADR 0026: bounded attitude-estimation startup audit

Status: scope frozen before new startup analysis, 2026-09-28.
This is an estimator audit, not a controller candidate or flight qualification.

## Preceding audit and evidence recovery

Audit main `7ee788091bb24fefb0e31a9e17754429365e1343`, tree
`e08f895bdf5a0ef312b2a1d703eaeb3bc290b838`. Post-merge CI 36463274850
passed. Fresh warning-strict early-flight and documentation checks pass 35 tests.
The restored original campaign matches its published ZIP digest. The stored
early-flight ZIP is incomplete: 15,035,145 bytes, SHA-256
`1746a3ae6a6cc6eae3334a973d320d272da2099439996f688e724797ca462774`.
Its original analysis and oracle reports, protocol, covariance chunks and other
complete members can be authenticated against the surviving manifest and the
published report digests. The main oracle data member is incomplete.

Recover that missing member by repeating only the identical ADR 0025 execution
at its unchanged source fingerprint, original configuration and seed. Require
the recovered member to match the previously recorded SHA-256
`4765988eee5822d2b58aaba1fc2a49e97e49afe02c4883720e8277cbc9ce535c`.
Retain the old reports/provenance and distinguish recovery execution provenance.
If the digest differs, stop recovery and report the blocker; do not relabel new
data as the old evidence. Do not add a seed, intervention or controller candidate.
Verify the completed original oracle evidence before implementing new analysis.

Local checks use the existing Python 3.12.14 / NumPy 2.5.2 environment directly;
global uv differs from the repository pin. Change no dependency or tool pin.

## Fixed questions and method

Use authenticated original and attitude-oracle hover data, seed 30, full 15.5 s
flights. Separate IMU prediction and each position/altitude correction through
the first second, then follow the existing 1..5 s and inclusive 5..11 s windows
to the original hover peak. Reconstruct right-local true-minus-estimate attitude
errors, thrust-axis error, physical covariance, coupled velocity/bias changes,
innovation timing and rejection. Explain the original 2.2941 to 3.1614 degree
startup change; an individual worsening update is not by itself a filter defect.

Check the actual endpoint sample-noise model, full joint conditioning/reset,
measurement Jacobians and right-local injection signs with independent algebra
and finite differences. Compare declared prior moments with the truth generator.
Report covariance/innovation diagnostics descriptively; correlated epochs from
one seed are not a population calibration test. Quantify measurement information
and tilt/bias ambiguity with an explicitly stated local model and rank/nullspace
checks. A local hover model must not be claimed to prove full nonlinear or
time-varying unobservability.

No truth enters estimator reconstruction: truth is used only after each update
to score diagnostics. Do not assume a freely flying accelerometer measures only
gravity, tighten the prior, alter sensors, tune Q/R or reinterpret prior results.
Production gains, controller family, limits, health, clocks, trajectories, scoring
and thresholds stay fixed. No reserved seeds or gain/cutoff sweep are permitted.

## Acceptance and decision

1. Authenticate both inputs and reconstruct saved state, covariance and updates.
2. Account for startup attitude growth by propagation and individual updates;
   check innovation/gain/Joseph/reset signs and coupled corrections independently.
3. Explain available causal attitude information and its limits during flight.
4. Fix a demonstrated in-scope defect if found. Otherwise return one justified
   measurement-only design with frozen interface, independent tests and original
   full-flight regression acceptance, or a justified no-go with specific missing
   information. Do not implement a speculative correction in this audit.
5. Pass relevant and complete software checks, preserve reproducible evidence
   outside Git, and state whether this audit succeeded separately from flight
   performance. Cascade remains default; mass-transient and geometric decisions
   are not reopened by this step.
