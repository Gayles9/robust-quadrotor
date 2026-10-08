# Combining supported velocity and nonlinear release prediction

This comparison asks whether exact supported velocity and nonlinear release
uncertainty work better together than either the original aligned startup or
the boundary-only correction. Matching against both controls distinguishes a
useful combination from an improvement that merely recovers a previous loss.

The [recorded combination](../results/combined-supported-prior.md) passed the
absolute flight limits but failed the requirement to avoid regression against
both matched controls. It is not adopted as the default mission behavior.

## Candidate and ownership

`SupportedVelocityConditioner` acts once at the externally supported release,
before initial observations. Its Gaussian constraint is $`H=[0\ I\ 0]`$ on the
21-dimensional joint state. For the independent zero-mean velocity profile,
conditioning removes only three velocity covariance rows and columns. Nominal
state, IMU sample, attitude/bias correlation, position uncertainty, noise and
acquisition evidence are unchanged.

The support assertion must be current, stationary and bound to the release;
missing, stale, revoked, moving or mismatched assertions reject. Quiet IMU
measurements cannot establish support.

The candidate then uses the [nonlinear first prediction](nonlinear-release-flight-comparison.md),
including fresh-sample correlations, followed by ordinary endpoint prediction.
The velocity constraint is never reapplied in flight. Time-zero position and
altitude observations remain causal. Online and replay use the same conditioned
endpoint and reproduce the full first-prediction trace.

Normal mission APIs, production source, cascade default, gains, Q/R, sensors,
random streams and thresholds stay fixed. Verification checks the isolated
covariance change, original input ownership, first-only routing, released
velocity uncertainty, noise correlations, exception restoration and complete
smoke-flight replay. The existing mathematical exact-prior gate is reused
without resampling.

## Matched evidence

| Input | SHA-256 |
| --- | --- |
| Original aligned report | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Boundary-only report | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |
| Nonlinear mathematical report | `8daddf91b2bbfc446a98a431e974303f58837057684ac3a7e11118d378384c81` |
| Boundary-only execution source | `cd190a70ccae59e9d437e9c44555902f0ce28912eae8dff8fa5c7d4a1a09b215` |

The earlier execution source retains its identity when the new combined module
is excluded. Reports remain bound to their actual execution source; they are
not rewritten to claim a newer source.

There are 25 candidate flights: hover, nominal tracking and wind tracking at
seeds 47001/47002/47003, and supervision off/on pairs for eight faults at seed
47004. Saved controls are reused. At most two worker processes execute the
campaign, and each candidate is saved to durable no-clobber output before
scoring.

Full histories, configurations, diagnostics, traces and signed changes are
retained against both controls. Saved data reconstruct estimator, commands,
health, supervision, plant/motors and paired sensor randomness. The complete
case/mode ledger and source/protocol identities are authenticated.

## Acceptance and limits

All three hovers must meet the original 8 cm peak requirement. All nine clean
flights must complete with at most 15 cm whole-flight RMSE/final-position error
and 15 cm/s final speed, including every startup sample. Each clean RMSE and
each hover peak must be no more than either matched control plus 1e-12.

All eight original fault-response conditions remain required. Both recovery
arms must also pass flight limits. Numerical abort is not a physical landing
demonstration.

Prerequisite failure prevents scientific execution; once eligible, all 25
planned valid outcomes remain in the evidence even if performance fails.
Infrastructure failures are recorded separately. Seeds, gains and thresholds
are fixed, with no replacement of valid failures.

Known-seed improvement would still require fresh validation before normal
mission integration. The study includes no mass compensation, geometric tuning
or hardware stationarity claim. The
[tradeoff diagnosis](combined-prior-tradeoff-diagnosis.md) explains why better
initial information need not improve every finite noisy trajectory.
