# ADR 0037: combined supported velocity and nonlinear release comparison

Status: frozen before implementation and candidate flights, 2026-09-29.

## Preceding audit

Audit PR 35 merge `9dad5d79229f1b8b194bc21c0e1cd9e988646fc8`, tree
`e21e946a284e21073275d5f08036b776788a675f`, against clean local main and live
GitHub. Warning-strict prior/release/nonlinear/durable checks pass 80 tests.
The complete 17-case mathematical replay reproduces both priors and ballistic
checks exactly. The independent NumPy-only checker authenticates the saved
boundary-only flights, rescores them, and reproduces all nonlinear errors within
1.388e-16. No preceding algorithm defect is demonstrated. Its failed flight
acceptance remains a failure.

## Candidate and ownership

Use the existing SupportedVelocityConditioner once on the externally supported
release before initial observations. Its Gaussian constraint is H=[0 I 0] on
the 21-dimensional joint state. For this independent, zero-mean velocity profile,
conditioning removes only the three velocity covariance rows/columns. It changes
neither the nominal state nor the IMU sample, bias/attitude correlation, position
uncertainty, noise model or acquisition evidence. Reject a missing, stale, revoked,
moving or mismatched support assertion. Do not infer support from quiet IMU data.

Then use exactly the ADR 0036 nonlinear first prediction, including its fresh
sample correlations, and ordinary endpoint prediction on every later interval.
The support constraint is never reapplied in flight. Time-zero position/altitude
observations remain causal. Online and replay must receive the same conditioned
initial endpoint and reconstruct the complete first-prediction trace exactly.
Keep normal mission APIs, all production source, cascade default, controller
gains, Q/R, sensors, seed streams and mission/safety thresholds unchanged.

Before scientific flights, test the isolated initial covariance change, original
input ownership, first-only routing, released velocity uncertainty, fresh-noise
correlations, exception restoration and complete smoke-flight replay. Retain
the unchanged ADR 0036 mathematical gate for the exact prior; do not resample it.

## Matched controls and fixed protocol

Authenticate ADR 0036's boundary-only report SHA-256
`54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2`
and mathematical report SHA-256
`8daddf91b2bbfc446a98a431e974303f58837057684ac3a7e11118d378384c81`.
Retain its execution source identity
`cd190a70ccae59e9d437e9c44555902f0ce28912eae8dff8fa5c7d4a1a09b215`.
Authenticate ADR 0031's original report SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Existing execution sources must retain that identity when the new combined
experiment module is excluded. Do not rewrite old reports to bind new source.

Run exactly 25 candidate flights: hover, nominal tracking and wind tracking for
each of seeds 47001/47002/47003; off/on supervision pairs for the original eight
fault cases at seed 47004. Use at most two worker processes. Reuse saved controls;
do not rerun them. Save each candidate before auditing/scoring, with durable
no-clobber output. Retain all histories, configurations, diagnostics, traces,
checks and signed metric changes against BOTH controls. Reconstruct estimator,
commands, health, supervision, plant/motors and paired sensor randomness from
saved data. Authenticate the full case/mode ledger and source/protocol identities.

## Acceptance and stopping

All three hover flights must pass the original 8 cm peak limit. All nine clean
flights must pass original completion, 15 cm whole-flight RMSE/final-position and
15 cm/s final-speed limits, including every startup sample. Each clean RMSE,
and each hover peak, must not exceed EITHER matched control by more than 1e-12.
Require all original eight fault-response conditions; both recovery arms must
also meet flight conditions. Numerical abort is not safe physical landing.

If a prerequisite fails, run no scientific flights. Once flights start, retain
all 25 planned valid outcomes even if a performance condition fails. Never retry
a valid flight, change seeds, fit parameters or loosen a limit after inspection.
Record infrastructure failures separately. On any acceptance failure, retain
the explicit experiment and stop; do not promote it. Even if this known-seed
comparison passes, fresh independently frozen validation is required before
normal mission integration. No mass compensation, geometric retuning, hardware
stationarity claim or future integration work is included.

## Closeout

Pass relevant/full software gates and hosted CI. Preserve complete experiment
evidence outside Git with independent integrity and flight-score checks. Report
implementation, measured benefit, acceptance and adoption separately. State the
next bounded step from the actual outcome.
