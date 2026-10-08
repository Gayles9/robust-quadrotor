# 2026-09-10: Local Position and Barometric Altitude Sensors

## Milestone objective

This milestone adds four pure, ROS/PX4-independent measurement boundaries for generic local
position and positive-up barometric altitude. It establishes the frames, units, shapes,
datum convention, equations, validation order, ownership rules, fixed random-consumption
policy, and deterministic first-two-moment evidence for both sensor channels.

The increment does not add GPS geodesy, atmospheric-pressure physics, hidden sensor state,
sample scheduling, timestamps, held measurements, truth interpolation, or delivery delay.
Sprint 3, Week 6 remains in progress, and Gate G1 remains open and partially satisfied.

## Starting commit and repository baseline

The architecture review began from clean, synchronized `main` at:

```text
b97cb29 feat: add reproducible IMU bias random walks
```

Both `HEAD` and `origin/main` resolved to:

```text
b97cb29f100522a4c1278bbab7041a2cb44a8305
```

Neither position-sensor file existed at that clean starting point. Before this documentation
step, the completed source-and-test increment consisted of exactly two untracked files:

```text
?? src/quadrotor_math/position_sensors.py
?? tests/unit/test_position_sensors.py
```

Nothing was staged. Their verified implementation checksums were:

```text
1827fd9d653630c74330013c3c1e40fe4ccdc298011221257bd13c70d16b66fe  src/quadrotor_math/position_sensors.py
476839d7a26e12bb6e759e6d0a6792bb15f53d05e6ac90a027f6f8f9c4429310  tests/unit/test_position_sensors.py
```

## Architecture decisions

The position channel is a generic local Cartesian sensor, not a GPS
latitude/longitude boundary. Its truth and measurement vectors are expressed directly in the
repository's north-east-down world frame. No geodetic conversion belongs in these functions.

Barometric altitude is exposed as a positive-up scalar. NED world position remains positive
down along `position_W[2]`, so the sign conversion is explicit and publicly named rather than
overloading a down-position as altitude. The caller selects the altitude assigned to the NED
origin through `reference_altitude`.

The barometric boundary is a geometric or indicated-altitude proxy. It contains no
atmospheric pressure, temperature, mean-sea-level, ellipsoid, or other hidden datum model.

Both noisy boundaries are value functions. They calculate one measurement for one valid
call, while the caller owns bias values and random generators. They do not decide when a
sample is due and do not retain a previous value. Sample rates, timestamps, held values,
acquisition scheduling, truth interpolation, and delivery delay belong in a future
ROS/PX4/Gazebo-independent scheduling layer.

## Public API and module ownership

The public functions are:

```text
ideal_position_measurement_world(
    position_W: NDArray[np.float64],
) -> NDArray[np.float64]

position_measurement_world(
    ideal_position_W: NDArray[np.float64],
    position_bias_W: NDArray[np.float64],
    noise_standard_deviation_W: NDArray[np.float64],
    rng: Generator,
) -> NDArray[np.float64]

ideal_barometric_altitude_from_position_world(
    position_W: NDArray[np.float64],
    reference_altitude: float,
) -> float

barometric_altitude_measurement(
    ideal_altitude: float,
    barometric_altitude_bias: float,
    noise_standard_deviation: float,
    rng: Generator,
) -> float
```

All four belong to `src/quadrotor_math/position_sensors.py`. The module remains independent
of ROS 2, PX4, Gazebo, message types, and simulator scheduling.

## Frames, shapes, units, and datum

| Quantity | Shape/type | Frame or convention | Units |
| --- | --- | --- | --- |
| `position_W` | float64 `(3,)` | NED world | m |
| `ideal_position_W` | float64 `(3,)` | NED world | m |
| `position_bias_W` | float64 `(3,)` | NED world axes | m |
| `noise_standard_deviation_W` | float64 `(3,)` | NED world axes, per sample | m |
| position output | float64 `(3,)` | NED world | m |
| `reference_altitude` | scalar | positive-up altitude at NED origin | m |
| `ideal_altitude` | scalar | positive up | m |
| `barometric_altitude_bias` | scalar | positive-up signed bias | m |
| scalar `noise_standard_deviation` | scalar | positive-up channel, per sample | m |
| barometric output | Python `float` | positive up | m |
| standard-normal samples | vector or scalar | component-aligned/dimensionless | 1 |

Positive NED world `z` is down. Consequently, increasing `position_W[2]` decreases the
positive-up altitude, while negative down-position represents upward displacement and
increases altitude. `reference_altitude` is caller-selected; the functions attach no hidden
mean-sea-level, ellipsoid, pressure, or calibration meaning to it.

## Measurement equations

The ideal local-position equation is:

```text
ideal_position_measurement_W = copy(position_W)
```

The noisy local-position equation is:

```text
standard_normal_sample_W = rng.standard_normal(3)

position_measurement_W =
    ideal_position_W
    + position_bias_W
    + noise_standard_deviation_W * standard_normal_sample_W
```

The ideal positive-up altitude conversion is:

```text
altitude = reference_altitude - position_W[2]
```

The noisy positive-up barometric-altitude equation is:

```text
standard_normal_sample = rng.standard_normal()

measured_altitude =
    ideal_altitude
    + barometric_altitude_bias
    + noise_standard_deviation * standard_normal_sample
```

Both noise-standard-deviation inputs are per-sample standard deviations. They are not
continuous-time densities and are not automatically scaled by a sample period.

## Validation order and error messages

`ideal_position_measurement_world(...)` validates in this order:

1. `position_W` shape; and
2. `position_W` finiteness.

Its exact messages are:

```text
position_W must have shape (3,)
position_W must contain only finite values
```

`position_measurement_world(...)` validates in this order:

1. `ideal_position_W` shape;
2. `position_bias_W` shape;
3. `noise_standard_deviation_W` shape;
4. `ideal_position_W` finiteness;
5. `position_bias_W` finiteness;
6. `noise_standard_deviation_W` finiteness;
7. `noise_standard_deviation_W` nonnegativity;
8. one vectorized random draw; and
9. the measurement calculation.

Its exact messages are:

```text
ideal_position_W must have shape (3,)
position_bias_W must have shape (3,)
noise_standard_deviation_W must have shape (3,)
ideal_position_W must contain only finite values
position_bias_W must contain only finite values
noise_standard_deviation_W must contain only finite values
noise_standard_deviation_W must be nonnegative
```

`ideal_barometric_altitude_from_position_world(...)` validates in this order:

1. `position_W` shape;
2. `position_W` finiteness;
3. `reference_altitude` finiteness; and
4. the positive-up conversion.

Its exact messages are:

```text
position_W must have shape (3,)
position_W must contain only finite values
reference_altitude must be finite
```

`barometric_altitude_measurement(...)` validates in this order:

1. `ideal_altitude` finiteness;
2. `barometric_altitude_bias` finiteness;
3. `noise_standard_deviation` finiteness;
4. `noise_standard_deviation` nonnegativity;
5. one scalar random draw; and
6. the measurement calculation.

Its exact messages are:

```text
ideal_altitude must be finite
barometric_altitude_bias must be finite
noise_standard_deviation must be finite
noise_standard_deviation must be nonnegative
```

No additional scalar type or magnitude restrictions are imposed beyond these finite-value and
nonnegative-standard-deviation contracts.

## Ownership and random-consumption semantics

All inputs remain caller-owned and unmodified. The ideal local-position function explicitly
copies `position_W`. The noisy vector calculation returns an independently allocated float64
array sharing memory with none of its three vector inputs. Both altitude functions return
Python floats.

The caller owns every NumPy `Generator`. Neither stochastic function creates, seeds, resets,
or hides a generator. Every valid position measurement consumes exactly one
`rng.standard_normal(3)` vector draw. Every valid barometric measurement consumes exactly one
scalar `rng.standard_normal()` draw. This policy includes valid zero-noise calls so changing
noise configuration between zero and nonzero values does not shift later seeded draws.

All validation guards run before sampling. A rejected position call and a rejected
barometric call were each checked at the final negative-noise boundary against an untouched
reference generator; both consume no random values. Equal seeds and equal two-call sequences
replay exactly, and consecutive calls use successive generator draws in order.

No hidden sensor state exists. Position bias and barometric bias are supplied per call rather
than retained or evolved internally.

## RED/GREEN development sequence

Development proceeded in bounded increments from the clean architecture review:

1. The read-only review fixed the local NED position boundary, positive-up altitude
   convention, explicit datum conversion, pure value-function scope, caller-owned RNG, and
   separation from scheduling and delay.
2. The first ideal-position test produced a missing-module RED. The minimum GREEN created
   `position_sensors.py` and returned an independent copy. Follow-up characterization fixed
   shape `(3,)`, float64 dtype, input nonmutation, then shape and finiteness validation.
3. The noisy-position seeded equation first produced a missing-import RED. Its minimum GREEN
   added the exact bias-plus-white-noise equation and one vector draw. Tests then established
   output ownership, zero-noise draw consumption, same-seed replay, and successive draws.
4. Seven position-validation test functions produced 13 failing cases: three shape cases,
   nine finiteness parameters, and one negative-noise case. Every failure was a missing
   `ValueError`. The GREEN added all seven guards in the specified order before sampling.
5. Final position characterizations established rejected-call RNG preservation and fixed-seed
   20,000-sample first-two-moment evidence, bringing the position portion to 26 cases.
6. The ideal-altitude sign test produced a missing-import RED. Its GREEN added the explicit
   `reference_altitude - position_W[2]` conversion and Python-float output. One passing type
   characterization and seven missing-guard RED cases then established shape and finiteness
   validation, reaching 37 total module cases.
7. The noisy barometric seeded equation produced a missing-import RED. Its minimum GREEN
   added one scalar draw and the exact additive equation. Four stochastic-policy tests added
   scalar type, zero-noise consumption, two-call replay, and successive-draw behavior.
8. Four barometric-validation test functions produced ten missing-guard failures: nine
   non-finite parameters and one negative-noise case. The GREEN added all four guards in order
   before sampling, taking the module to 52 passing cases.
9. The final two tests established rejected-call RNG preservation and the fixed-seed scalar
   population evidence. The complete position-sensor module ended at 54 passing tests.

## Ruff interruptions and bounded canonicalization

Three early increments stopped on Ruff layout failures without continuing behavioral work.
The initial missing-module RED had an unnecessary blank line inside what was then one import
block. After the production module existed, Ruff correctly reclassified its import as
first-party and required the opposite grouping: a blank line after NumPy. The seeded position
equation RED also used a multiline expression that Ruff required in its canonical one-line
layout. Each defect was repaired in an isolated formatting-only step with no behavior change.

After those interruptions, the workflow switched to bounded Ruff canonicalization: the safe
fixer and formatter were allowed to modify only the file authorized for that increment, the
result was inspected, and non-modifying lint and format checks covered source and tests. This
prevented layout-only follow-up work from being mixed with behavioral changes. The final
barometric population test, for example, retained its formula while Ruff canonicalized the
mean-error expression onto one line.

## Stale-prompt checksum stop

After ideal barometric work had advanced the source and test files, a duplicate prompt for the
already completed position stochastic characterization carried older source and test
checksums. The read-only preflight detected that mismatch and stopped without editing. Work
resumed only under a corrected prompt whose checksums matched the active working tree. This
was the intended safety behavior: stale task context never overwrote newer verified work.

## Verified behavioral coverage

The completed module contains 54 tests covering:

- exact mixed-sign ideal NED position values;
- vector dtype, shape, independent storage, and input nonmutation;
- ideal-position shape and finiteness validation;
- seeded noisy-position equation and exact vector-draw policy;
- noisy-position ownership, zero-noise draw consumption, replay, and advancement;
- all noisy-position shape, finiteness, and nonnegative-noise guards;
- rejected position-call RNG preservation and population moments;
- positive-down/positive-up altitude sign anchors, including the origin datum;
- Python-float ideal-altitude output and ideal-altitude validation;
- seeded noisy-barometric equation and scalar output;
- barometric zero-noise consumption, replay, and successive draws;
- barometric finiteness and nonnegative-noise validation;
- rejected barometric-call RNG preservation; and
- barometric population moments.

## Deterministic position population evidence

The position population characterization uses seed `12345`, 20,000 measurements,
`ideal_position_W = [12.5, -7.25, 3.0] m`, `position_bias_W = [0.5, -0.25, 1.0] m`,
and `noise_standard_deviation_W = [0.1, 0.2, 0.3] m`.

The empirical mean is:

```text
[13.000229816960843, -7.497265295977045, 3.998727077084256]
```

The empirical sample standard deviation using `ddof=1` is:

```text
[0.1006613199090335, 0.1997493761618388, 0.2964885532806003]
```

Mean errors in normalized standard-error units are:

```text
[0.32501026288755136, 1.933727759169497, 0.6000616171002187]
```

Sample-standard-deviation errors in normalized standard-error units are:

```text
[1.3226067516582025, 0.25061757248693656, 2.340905954756227]
```

## Deterministic barometric population evidence

The barometric population characterization uses seed `12345`, 20,000 measurements,
`ideal_altitude = 125.0 m`, `barometric_altitude_bias = 1.5 m`, and
`noise_standard_deviation = 0.75 m`.

```text
empirical_mean = 126.50627051923546
empirical_standard_deviation = 0.7507717228494047
mean_error_in_standard_errors = 1.182380446120635
standard_deviation_error_in_standard_errors = 0.20578761495795705
```

Every normalized position and barometric error is strictly below `5.0`. These fixed-seed,
finite-sample results support consistency with the configured means and sample standard
deviations. They do not prove Gaussianity, independence, correlation structure,
stationarity, or hardware fidelity.

## Verification evidence before the final repository gate

Before documentation and the final repository gate:

- bounded Ruff lint passed for `src/quadrotor_math/position_sensors.py` and
  `tests/unit/test_position_sensors.py`;
- bounded Ruff format verification reported both files already formatted;
- bounded mypy reported no issues in the one position-sensor source file;
- the complete position-sensor module passed 54 tests;
- no failures, errors, skips, or warnings occurred;
- `git diff --check` and `git diff --cached --check` passed; and
- nothing was staged.

This focused evidence preceded the completed repository gate and is retained separately from
the later repository-wide verification.

## Limitations and deferred behavior

This milestone does not establish or implement:

- GPS geodesy or latitude/longitude;
- atmospheric pressure or temperature physics;
- mean-sea-level, ellipsoid, or other implicit altitude datums;
- Gaussianity, independence, correlation, or stationarity from finite population evidence;
- drift or internally evolved position/barometric bias;
- dropouts, quantization, saturation, or hardware calibration;
- sampling-rate management, timestamps, held values, or delivery delay;
- truth interpolation or truth/nominal-model separation;
- run-level replay artifacts or Monte Carlo campaigns;
- wind, drag, or deliberate model mismatch;
- hardware fidelity, state estimation, or controller behavior; or
- ROS 2, PX4, Gazebo, or message integration.

The measurement functions contain no scheduling mechanism and do not represent a complete
sensor layer.

## Gate G1 and sprint status

Sprint 3, Week 6 remains in progress.

Gate G1 remains open and partially satisfied. This milestone establishes local-position and
positive-up barometric-altitude values, validation, ownership, fixed random-consumption
semantics, replay, rejected-call preservation, and bounded first-two-moment evidence. It does
not complete the remaining sensor, uncertainty, simulation, estimation, robustness,
saved-replay, or integration evidence required by the gate.

Remaining work includes sample rates, timestamps, delivery delay, truth interpolation, held
measurements, truth/nominal separation, run-level replay, wind, drag, and deliberate model
mismatch.

The exact next action is a read-only architecture review for the ROS-independent sensor
scheduling and timestamp boundary. No scheduling implementation commitment is inferred by
this record.

## Completed repository gate

After the focused and pre-gate checks above, the repository gate for the documented working
tree was run exactly once with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

It completed successfully with these exact results:

- Ruff lint: passed;
- Ruff format: 53 files already formatted;
- mypy: no issues in 13 source files;
- pytest: 402 passed in 1.44s;
- position-sensor tests: 54; and
- failures, errors, skips, and warnings: none.

The previous 348-test repository baseline plus the 54 new position-sensor tests accounts for
the verified 402-test total. This subsequent status correction changes documentation only;
it does not rerun the repository gate and leaves the already verified Python artifacts
unchanged.

## Files changed by the completed milestone

The completed milestone consists of:

```text
src/quadrotor_math/position_sensors.py
tests/unit/test_position_sensors.py
README.md
docs/progress/2026-09-10-local-position-and-barometric-altitude-sensors.md
```

The documentation step changes only `README.md` and this progress record. The source and test
changes predate documentation and retain their verified checksums.

Nothing is staged, committed, or pushed by this documentation increment.
