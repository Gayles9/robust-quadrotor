# 2026-09-08: Reproducible Accelerometer White Noise

## Milestone objective

This milestone adds a reproducible noisy accelerometer boundary after the ideal
specific-force and constant additive bias layers. The public function
`accelerometer_specific_force_measurement_body(...)` combines those quantities with
caller-configured, per-axis, per-sample white noise.

The increment establishes explicit generator ownership, validation order, random-consumption
semantics, deterministic replay, and bounded population-level evidence. It does not add a
stateful sensor model, continuous-time noise conversion, estimation, robustness, or hardware
fidelity. Sprint 3, Week 6 remains in progress, and Gate G1 remains open.

## Repository state before documentation

The source-and-test increment was completed and verified from synchronized `main` at:

```text
93d12b7 feat: add constant accelerometer bias
```

Before this documentation increment, the working tree contained exactly:

```text
 M src/quadrotor_math/imu.py
 M tests/unit/test_imu.py
```

Nothing was staged. The protected source and test checksums were:

```text
25522fc1f8478b0a249f30fb2ed38c8b3de774bdc047a4781f87e4c61496e211  src/quadrotor_math/imu.py
278ae42c87dc36b044b116c2f7522f68983cfac1ea57e1c05bdcac9bed11ea56  tests/unit/test_imu.py
```

## Public API and module ownership

The public API is:

```text
accelerometer_specific_force_measurement_body(
    ideal_specific_force_B: NDArray[np.float64],
    accelerometer_bias_B: NDArray[np.float64],
    noise_standard_deviation_B: NDArray[np.float64],
    rng: Generator,
) -> NDArray[np.float64]
```

The function belongs in `src/quadrotor_math/imu.py` beside the ideal accelerometer,
constant-accelerometer-bias, ideal gyroscope, and noisy-gyroscope boundaries. It does not own
dynamics, integration, simulation scheduling, or generator construction.

No helper, dataclass, sensor hierarchy, configuration object, or generic stochastic
abstraction was introduced. The existing IMU module and direct NumPy operations are
sufficient for this boundary.

## Physical and stochastic measurement equation

The implemented equation is:

```text
specific_force_measurement_B =
    ideal_specific_force_B
    + accelerometer_bias_B
    + noise_standard_deviation_B * standard_normal_sample_B
```

The multiplication is component-wise, allowing each body axis to use its own nonnegative
noise standard deviation. The standard deviation is configured per sample in m/s². The
function performs no sample-rate conversion or continuous-time noise-density scaling.

The dimensionless standard-normal vector comes from exactly:

```text
standard_normal_sample_B = rng.standard_normal(3)
```

## Frame, shape, and units contract

The project world frame remains north-east-down (NED), and the body frame remains
forward-right-down (FRD). Every quantity at this boundary is already resolved along the
body-aligned FRD axes, so no attitude transformation occurs here.

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `ideal_specific_force_B` | `(3,)` | body-aligned FRD axes | m/s² |
| `accelerometer_bias_B` | `(3,)` | body-aligned FRD axes | m/s² |
| `noise_standard_deviation_B` | `(3,)` | body-aligned FRD axes | m/s² |
| `standard_normal_sample_B` | `(3,)` | component-aligned draw | dimensionless |
| returned measurement | `(3,)` | body-aligned FRD axes | m/s² |

Finite positive, negative, and zero ideal-specific-force and bias components are valid.
Noise standard deviations may be zero or positive but cannot be negative.

## Caller-owned RNG and fixed consumption

The caller constructs and supplies the NumPy `Generator`. The measurement function does not
create, seed, reset, replace, or hide a generator.

Every valid call consumes exactly one vectorized `standard_normal(3)` draw. This remains true
when all configured standard deviations are zero. Keeping the draw count independent of the
noise configuration prevents later seeded samples from shifting when a caller changes
between zero and nonzero noise.

Equal seeds reproduce equal multi-call measurement sequences when inputs and call order are
equal. Consecutive calls on one generator consume successive reference draws. Rejected calls
consume no random values because every validation guard executes before sampling.

This contract does not promise identical bitstreams across different NumPy versions or bit
generators.

## Validation contract and order

Validation and execution occur in this order:

1. `ideal_specific_force_B` shape;
2. `accelerometer_bias_B` shape;
3. `noise_standard_deviation_B` shape;
4. `ideal_specific_force_B` finiteness;
5. `accelerometer_bias_B` finiteness;
6. `noise_standard_deviation_B` finiteness;
7. `noise_standard_deviation_B` nonnegativity;
8. one `rng.standard_normal(3)` draw; and
9. measurement calculation.

The exact `ValueError` messages are:

```text
ideal_specific_force_B must have shape (3,)
accelerometer_bias_B must have shape (3,)
noise_standard_deviation_B must have shape (3,)
ideal_specific_force_B must contain only finite values
accelerometer_bias_B must contain only finite values
noise_standard_deviation_B must contain only finite values
noise_standard_deviation_B must be nonnegative
```

Exact shape guards prevent malformed arrays from being silently broadcast. Finiteness is
checked before nonnegativity, and the last configuration guard still precedes the random
draw.

## Output ownership

For valid float64 vectors, NumPy arithmetic produces an independently owned float64 result
with shape `(3,)`. Tests confirm that the result shares memory with none of the ideal
specific-force, bias, or standard-deviation inputs.

No separate copy, coercion, or dtype-validation behavior was introduced. The typed public
contract supplies float64 arrays, and the measurement arithmetic already creates independent
storage.

## Verified behavioral coverage

The accelerometer measurement tests establish:

- the seeded measurement equation using the exact reference `standard_normal(3)` sample;
- independent output ownership;
- exact-shape validation for all three array inputs;
- rejection of NaN and both infinities in every array input;
- rejection of negative noise standard deviation;
- zero-noise output while retaining the fixed one-draw policy;
- same-seed replay across a two-call sequence;
- successive reference-draw consumption across consecutive calls;
- preservation of caller-owned RNG state after rejection; and
- configured population mean and per-axis sample standard deviations.

The rejected-call characterization uses a negative standard-deviation component, which is
the final validation boundary before sampling. Matching the generator's next sample against
an untouched reference generator shows that validation completed without consuming random
values.

## Deterministic population evidence

The deterministic characterization uses seed `12345` and 20,000 measurements. The expected
mean is the ideal specific force plus constant accelerometer bias.

The observed sample mean was:

```text
[1.2500229816960966, -0.8197265295976988, -9.71012729229157]
```

The observed sample standard deviation with `ddof=1` was:

```text
[0.010066131990903391, 0.01997493761618382, 0.029648855328060025]
```

Absolute mean errors in units of `sigma / sqrt(n)` were:

```text
[0.32501026306151765, 1.9337277592106337, 0.6000616170776093]
```

Absolute sample-standard-deviation errors in units of
`sigma / sqrt(2 * (n - 1))` were:

```text
[1.3226067516590352, 0.25061757248751937, 2.3409059547562645]
```

Every normalized error is strictly below the acceptance limit of `5.0`. This deterministic
finite sample supports consistency with the configured first two moments on all three axes.
It does not prove Gaussianity, independence, stationarity, or portability across NumPy
versions.

## Complete quality-gate evidence

The complete repository gate was run once with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

It executed:

```text
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments
uv run pytest
```

Results:

- Ruff lint passed.
- Ruff non-modifying format verification reported 48 files already formatted.
- Strict mypy passed over `src` and `experiments` with no issues in 12 source files.
- Pytest collected and passed 309 tests in 0.85 seconds.
- The complete IMU module passed 79 tests.
- No failures, errors, skips, or warnings occurred.
- `git diff --check` passed.
- `git diff --cached --check` passed.
- Verification changed no files.
- Nothing was staged.

## Limitations and non-claims

This milestone does not add or establish:

- accelerometer bias drift or random walk;
- continuous-time noise-density conversion;
- automatic sample-rate scaling;
- Gaussianity or independence proofs;
- stationarity or sensor-bandwidth behavior;
- scale-factor or cross-axis error;
- axis misalignment;
- saturation, clipping, or quantization;
- timestamps, latency, or sample scheduling;
- temperature, vibration, mounting, calibration, or lever-arm effects;
- serialized RNG state or cross-version random-bitstream guarantees;
- saved replay artifacts or Monte Carlo campaigns;
- state estimation, controller behavior, or robustness evidence;
- hardware fidelity; or
- ROS 2, PX4, or simulator integration.

## Gate G1 and sprint status

Sprint 3, Week 6 remains in progress.

Gate G1 remains open and only partially satisfied. This milestone adds reproducible
accelerometer white noise, explicit RNG ownership, fixed draw consumption, validation before
sampling, deterministic replay, rejected-call RNG preservation, and bounded first-two-moment
evidence. It does not establish all remaining sensor, uncertainty, estimation, robustness,
saved-replay, or integration evidence required by the gate.

## Files changed by the completed milestone

The completed milestone consists of:

```text
src/quadrotor_math/imu.py
tests/unit/test_imu.py
README.md
docs/progress/2026-09-08-reproducible-accelerometer-white-noise.md
```

The documentation step changes only `README.md` and this new progress record. The source and
test changes predate documentation and must retain their verified checksums.

Nothing is staged, committed, or pushed by the documentation increment.

## Next exact action

Continue Gate G1 work without claiming completion until the remaining deterministic and
stochastic evidence required by the canonical master plan is implemented. The repository does
not contain that master-plan document, so a more specific next engineering increment must be
taken from the external authoritative plan rather than inferred here.
