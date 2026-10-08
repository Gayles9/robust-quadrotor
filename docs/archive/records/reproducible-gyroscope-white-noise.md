# 2026-09-02: Reproducible Gyroscope White Noise

## Milestone objective

This milestone adds a reproducible noisy gyroscope boundary after the ideal measurement and
constant additive bias layers. The public function
`gyroscope_angular_velocity_measurement_body(...)` combines those quantities with
caller-configured, per-axis, per-sample white noise.

The increment establishes an explicit stochastic contract: who owns the random-number
generator, exactly how many values each call consumes, which configurations are valid, and
which deterministic tests support replay and population-level claims. It does not add a
stateful sensor model, continuous-time noise conversion, estimation, robustness, or hardware
fidelity. Gate G1 remains open.

## Repository state before the milestone

The milestone followed the completed constant-gyroscope-bias increment. Before this
documentation increment, the implemented and verified source-and-test work existed as two
unstaged modified files:

```text
 M src/quadrotor_math/imu.py
 M tests/unit/test_imu.py
```

The documentation increment changes only `README.md` and this progress record. Source and
test files were protected with before-and-after SHA-256 checksums.

## Final API and module ownership

The public API is:

```text
gyroscope_angular_velocity_measurement_body(
    ideal_angular_velocity_B,
    gyroscope_bias_B,
    noise_standard_deviation_B,
    rng,
) -> angular_velocity_measurement_B
```

The function belongs to `src/quadrotor_math/imu.py` beside the ideal accelerometer, ideal
gyroscope, and deterministic gyroscope-bias boundaries. It does not own dynamics,
integration, simulation scheduling, or random-generator construction.

No helper, dataclass, sensor hierarchy, measurement container, or generic stochastic
abstraction was introduced. The existing IMU module and direct NumPy operations are
sufficient for this boundary.

## Physical and stochastic measurement model

The final implementation equation is:

```text
angular_velocity_measurement_B =
    ideal_angular_velocity_B
    + gyroscope_bias_B
    + noise_standard_deviation_B * standard_normal_sample_B
```

The noise multiplication is component-wise. Each axis can therefore use its own nonnegative
standard deviation. The standard deviation is a per-sample configuration in rad/s; the
function performs no sample-rate conversion or continuous-time noise-density scaling.

The standard-normal vector comes from exactly:

```text
rng.standard_normal(3)
```

The returned value is the ideal body angular velocity plus constant bias plus the configured
white-noise contribution.

## Frame, shape, and units contract

The project world frame remains north-east-down (NED), and the body frame is
forward-right-down (FRD). Every quantity at this boundary is already resolved along the
body-aligned FRD axes, so no attitude transformation occurs.

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `ideal_angular_velocity_B` | `(3,)` | body-aligned FRD axes | rad/s |
| `gyroscope_bias_B` | `(3,)` | body-aligned FRD axes | rad/s |
| `noise_standard_deviation_B` | `(3,)` | body-aligned FRD axes | rad/s |
| `standard_normal_sample_B` | `(3,)` | component-aligned draw | dimensionless |
| returned measurement | `(3,)` | body-aligned FRD axes | rad/s |

Finite positive, negative, and zero angular velocities and biases are valid. Noise standard
deviations may be zero or positive but cannot be negative.

## Caller-owned RNG decision

The caller constructs and supplies a NumPy `Generator`. The measurement function does not
create, seed, reset, replace, or hide a generator. This keeps experiment-level seed policy
outside the sensor equation and makes sequence ownership explicit.

Two independently created generators with the same seed reproduce the same measurements when
they receive the same inputs in the same call order. A caller can also retain and advance one
generator across consecutive measurement calls without hidden resets.

## Exact random-consumption policy

Every valid call consumes one vectorized `standard_normal(3)` draw. The policy is independent
of the configured standard-deviation values. In particular, an all-zero standard-deviation
vector still consumes the normal draw even though multiplication makes its noise contribution
exactly zero.

Fixed consumption prevents later seeded samples from shifting merely because configuration
changes between zero and nonzero noise. The function does not draw one scalar at a time,
reuse an earlier sample, or optimize away the draw for zero noise.

Rejected inputs consume no random values. All validation finishes before the caller-owned
generator is used.

## Validation contract and exact order

The validation and execution order is exactly:

1. `ideal_angular_velocity_B` shape;
2. `gyroscope_bias_B` shape;
3. `noise_standard_deviation_B` shape;
4. `ideal_angular_velocity_B` finiteness;
5. `gyroscope_bias_B` finiteness;
6. `noise_standard_deviation_B` finiteness;
7. `noise_standard_deviation_B` nonnegativity;
8. RNG draw; and
9. measurement calculation.

The exact validation messages are:

```text
ideal_angular_velocity_B must have shape (3,)
gyroscope_bias_B must have shape (3,)
noise_standard_deviation_B must have shape (3,)
ideal_angular_velocity_B must contain only finite values
gyroscope_bias_B must contain only finite values
noise_standard_deviation_B must contain only finite values
noise_standard_deviation_B must be nonnegative
```

The shape guards prevent broadcasting malformed inputs. Finiteness is checked before
nonnegativity so NaN and infinities receive the established finiteness error. The final
configuration guard still precedes sampling.

## Output ownership

The measurement is formed through NumPy arithmetic rather than returned as an input alias.
For the valid float64 vector contract, the result owns storage independently of the ideal
measurement, bias, and standard-deviation arrays.

No additional copy or dtype-validation behavior was added. The typed interface states the
float64 contract, and the direct arithmetic already provides independent output storage.

## TDD development sequence

Development proceeded through focused RED/GREEN and characterization increments:

1. The seeded measurement-equation test initially failed during collection because the
   measurement function did not exist.
2. The minimum production function added one vectorized draw and the measurement equation.
3. Independent output ownership was characterized without requiring additional copying.
4. Three separate shape tests produced missing-validation failures before their guards were
   added in the required order.
5. Three parameterized finiteness definitions produced missing-validation failures before
   their guards were added. Together they cover nine invalid values.
6. Negative-standard-deviation rejection first produced `DID NOT RAISE ValueError`, then
   passed after the final pre-sampling configuration guard was added.
7. Zero-noise consumption, same-seed replay, successive draws, rejected-call preservation,
   and population statistics were characterized without further production changes.

Each production change addressed one demonstrated missing behavior. Characterization tests
then fixed the stochastic policy without expanding the implementation.

## Exact behaviors covered by tests

The gyroscope measurement tests establish:

- the seeded equation using the exact reference `standard_normal(3)` sample;
- independently owned output memory;
- separate shape validation for all three array inputs;
- finiteness rejection for NaN and both infinities in every array input;
- rejection of a negative standard-deviation component;
- zero-noise output with fixed random consumption;
- same-seed replay across a two-measurement sequence;
- successive draw consumption across consecutive calls;
- preservation of RNG state after a rejected call; and
- configured population mean and sample standard deviations.

There is one seeded equation test and one independent-memory test. Three shape definitions
cover three cases. Three parameterized finiteness definitions cover nine cases. The remaining
characterizations each have one focused failure meaning.

## Deterministic replay evidence

The replay characterization creates two independent generators from seed `12345`. Each
generator receives the same valid inputs for two calls. The complete two-sample measurement
arrays compare exactly equal.

This proves deterministic multi-call replay for equal seeds and equal call sequences. It does
not claim portability across different NumPy versions or bit generators beyond the tested
environment.

## Zero-noise behavior

With `noise_standard_deviation_B = [0, 0, 0]`, the returned value equals exactly the ideal
angular velocity plus bias within the established numerical comparison tolerance. The test
then compares the next measurement-generator sample with the next sample from a reference
generator that was manually advanced once.

Their equality proves that the valid zero-noise call still consumed one three-value draw.
Production must not skip the draw when every standard deviation is zero.

## Successive-draw advancement

Two consecutive calls use one shared measurement generator. An independently seeded
reference generator supplies the expected first and second `standard_normal(3)` vectors. Each
measurement matches the corresponding reference equation.

This establishes draw order and shows that production neither recreates nor resets the
generator and does not reuse a prior sample.

## Rejected-call RNG preservation

The rejected-call characterization uses a negative standard-deviation component, which is
the final validation guard before sampling. After the expected `ValueError`, the caller-owned
generator's next sample exactly matches the untouched reference generator's first sample.

If any random values had been consumed before validation completed, those samples would not
match. This test therefore covers RNG-state preservation at the final rejection boundary
without repeating every invalid-input case.

## Statistical population evidence

The deterministic population characterization uses seed `12345` and 20,000 measurements.
The sample mean is:

```text
[0.7200229816960914, -0.4297265295977012, 1.1398727077084245]
```

The sample standard deviation with `ddof=1` is:

```text
[0.0100661319909034, 0.0199749376161838, 0.0296488553280600]
```

Absolute mean errors, expressed in units of `sigma / sqrt(n)`, are:

```text
[0.3250102629877233, 1.9337277591933626, 0.6000616171058709]
```

Absolute sample-standard-deviation errors, expressed in units of
`sigma / sqrt(2 * (n - 1))`, are:

```text
[1.3226067516582374, 0.2506175724872418, 2.3409059547567272]
```

Every error is below the acceptance bound of `5.0` standard errors. The fixed seed makes the
population deterministic. This finite sample supports consistency with the configured first
two moments on all three axes; it does not prove perfect Gaussianity or independence.

## Important failures and corrections

- The initial RED failed during collection with `ImportError` because the measurement
  function did not yet exist.
- Shape, finiteness, and negative-standard-deviation tests produced the intended
  missing-validation failures before their GREEN guards.
- The first population assertion passed an array-valued absolute tolerance to
  `assert_allclose`. When the comparison failed, NumPy's failure formatting raised
  `TypeError` while attempting to format that array as a scalar.
- The population test was corrected by expressing each axis error in standard-error units and
  comparing the resulting arrays with a three-element array containing `5.0`.
- Two test signature or assignment layouts required manual Ruff-compatible corrections.
- No production behavior changed to correct a test assertion or formatting mistake.

These corrections preserved the intended contracts and kept production changes tied to
behavioral RED failures.

## Quality-gate evidence

The complete IMU module passes 48 tests with no warnings. The complete repository gate was
run with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

It reported:

- Ruff lint passed;
- Ruff non-modifying format verification passed for 46 files;
- strict mypy passed over `src` and `experiments` with no issues in 12 source files;
- pytest collected and passed 278 tests in 1.18 seconds;
- no tests failed, skipped, or errored;
- no warnings were emitted; and
- `git diff --check` passed.

## Deferred behavior and non-claims

This milestone does not add or claim:

- gyroscope bias random walk or drift;
- continuous-time noise-density conversion;
- automatic sample-rate scaling;
- accelerometer bias or noise;
- scale-factor error or axis misalignment;
- saturation or quantization;
- latency, timestamps, or sample scheduling;
- temperature, vibration, calibration, or hardware fidelity;
- estimator performance or state estimation;
- saved replay artifacts or Monte Carlo campaigns;
- control or robustness evidence; or
- ROS 2, PX4, or simulator integration.

The generator is caller-owned, but this increment does not define an application-wide seed
registry, serialized RNG state, or cross-version bitstream guarantee.

## Gate G1 implications

Gate G1 remains open. This milestone adds reproducible gyroscope white noise, explicit RNG
ownership, fixed consumption, deterministic replay, and population-level evidence. It does
not complete the remaining sensor modeling, uncertainty, estimation, robustness, or
integration work required by the gate.

## Files changed

The completed milestone consists of:

```text
src/quadrotor_math/imu.py
tests/unit/test_imu.py
README.md
docs/progress/2026-09-02-reproducible-gyroscope-white-noise.md
```

The documentation increment itself changes only the final two files. Production and test
changes predate this record and remain unstaged.

## Current repository state

The verified working tree contains the completed source, test, and documentation changes.
Nothing is staged, committed, or pushed by this increment. The mathematical core remains
independent of ROS 2, PX4, and simulator middleware.

## Next exact development action

Add the first RED test for constant additive accelerometer bias while preserving the existing
NED/FRD frame, shape, units, validation, and output-ownership contracts.
