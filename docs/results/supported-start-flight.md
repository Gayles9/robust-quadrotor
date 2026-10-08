# Supported-start flight comparison

This experiment tests whether estimating inclination and sensor biases while
the vehicle is held still improves flight after release. It uses the
[pre-arm component](../design/prearm-component.md) and an explicit stationary
support fixture. The [study protocol](../decisions/supported-start-flight-evaluation.md)
specifies four known maneuver cases and eight flights in four pairs. Normal flight
algorithms and controller defaults are unchanged.

## Separate alignment from the changed operating condition

The original campaign starts with slightly perturbed position, velocity,
attitude and angular rate, and motors already spinning near hover speed. A
motors-off stationary interval cannot be inserted into that initialization
without changing its physical condition.

| Comparison | Physical start | Estimator alignment |
| --- | --- | --- |
| Archived original | Original moving state and spinning motors | Original independent prior |
| Supported unaligned | Constant pose, zero motion and motors, then fixture release | Original means/covariance with elapsed bias walk |
| Supported aligned | Identical fixture release and sensor draws | Verified pre-arm component output |

Both supported arms retain the original true position and attitude. Their p/v
estimate and covariance remain the original independent prior. Thus the direct
aligned-versus-unaligned comparison changes only initial attitude/bias estimation
and covariance. Comparing either to the original also includes the disclosed
physical startup change. Its result therefore cannot replace the original
initialization's failed result.

## Mechanical support and the boundary sample

An ideal external fixture provides the reaction force that balances gravity
and wind drag for 0.5025 s. Position and attitude are constant; velocity,
angular rate and rotor speeds are zero. The saved force balance substantiates
the simulated support evidence. Specific force is minus gravity expressed in
body coordinates, even with motors off; free-fall zero specific force would
be the wrong sensor model for this supported interval.

The 201 alignment samples span 0.5 s. A separate fresh sample at 0.5025 s is
acquired under support and becomes flight sample zero. The fixture is removed
immediately afterward. Actual rotor speeds start at zero and evolve with the
existing motor lag. Position, velocity, attitude, rate and motor speed remain
continuous. The external reaction disappears as a finite force step.

The first measurement is the supported left limit at the release boundary.
The existing endpoint ESKF integrates across that first discontinuous interval
without a special correction. Its startup discretization error is retained in
all scores. This is an ideal fixture release, not a ground-contact takeoff,
physical landing or validated hardware support procedure.

The [supported velocity prior screen](supported-velocity-prior.md) quantifies
why this retained first-interval error prevents simply reducing the initial
velocity covariance. The [release-aware prediction](../design/release-prediction.md)
explains a separately evaluated treatment of this boundary.

## Information, sample ownership and replay

Support acquisition has independent deterministic white-noise and bias-walk
streams. It does not advance any existing flight random stream. The fresh
release measurement uses the original flight stream's first draw; all later
IMU noise, bias increments and slow-sensor draws retain the original schedule
and seeds. The verifier reconstructs every actual draw after accounting for
each run's physical signal and bias, including wind and mass effects.

The component sees measurements, its declared prior and external support
evidence. Truth belongs to the fixture, sensor generator and evaluator. The
controller continues to receive the normal ESKF estimate. The handoff checks
the entire 21x21 endpoint covariance, including attitude/bias cross terms and
fresh-sample noise, plus the actual fresh IMU values. No alignment sample is
replayed as a new flight sample.

An experiment-only adapter supplies the supported physical acceleration at
the first sensor call. Subsequent calls use the normal freely flying sensor
model. The adapter restores on exceptions and is never shared between threads.
Existing reconstruction code verifies every plant interval, estimator state,
covariance, event, control command, mission guard and supervisor decision.

## Results and decision

The predeclared startup comparison passes. The aligned hover, nominal tracking and
wind cases pass all their original flight conditions. The mass case still fails
RMSE, final position and completion. All eight executions and four archived
comparators are preserved in the [verification record (ZIP)](../../evidence/development-records.zip).

| Metric, cm | Original | Supported unaligned | Supported aligned |
| --- | ---: | ---: | ---: |
| Hover peak, inclusive 5..11 s | 10.756 | 8.588 | **2.506** |
| Hover whole-flight RMSE | 7.660 | 7.421 | **3.624** |
| Nominal tracking whole-flight RMSE | 6.244 | 6.780 | **4.258** |
| Wind tracking whole-flight RMSE | 6.717 | 7.198 | **5.040** |
| Mass tracking whole-flight RMSE | 42.393 | 42.780 | **42.526, fails** |

Alignment reduces the scored hover peak by 70.8% against the physically identical
supported control, and 76.7% against the differently initialized original run.
Early hover thrust-axis estimation error peaks at 0.106 degrees instead of
3.000 degrees in the supported unaligned run. The whole flight, including the
takeoff transient outside the scored hover window, is retained in RMSE.

This does not improve every metric. Final position errors increase slightly in
the aligned nominal and wind runs while remaining within their original limits.
The aligned mass case still has 43.352 cm final error and `landing_timeout` at
25 s. Better inclination estimation does not supply missing mass compensation.

Known seeds 30/31 establish a benefit in these cases, not general qualification.
The [independent validation](independent-supported-start.md) evaluates
repeatability and observation-fault responses. It confirms improvement in all
three independent hover pairs but retains one 10.18 cm failure. The initializer
therefore remains experimental, the cascade remains the default, and geometric
control and mass compensation retain their separate limitations.

## Acceptance and reproduction

The full flight is scored, including startup. Hover keeps the inclusive 5..11 s
window and 8 cm peak limit. Whole-flight RMSE and final position remain limited
to 15 cm, and final speed to 15 cm/s. Alignment must pass the complete hover
case, avoid increasing its peak relative to either comparator, and preserve
every condition passed by either comparator in the four cases. Mass-case
failures and all-four-case qualification are reported separately.

Only known seeds 30/31 are used. This is a bounded development comparison;
reserved qualification seeds, fault campaigns and geometric qualification
remain outside this comparison. The reported result uses the original settings
and thresholds without tuning to its outcomes.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_start_validation --baseline results/original-campaign --output results/supported-start --workers 2
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_start_validation --baseline results/original-campaign --verify results/supported-start
```

The baseline path must contain the authenticated original
[integrated robustness campaign](integrated-robustness.md).
Use a new output directory. A valid comparison that misses acceptance exits 1
and retains its evidence; implementation or evidence errors raise separately.
Full replay of this saved study requires its recorded source fingerprint.
Use the matching source snapshot identified in the verification record: a global
fingerprint can change when experiment files change, even if the original
controller behavior is unchanged.
