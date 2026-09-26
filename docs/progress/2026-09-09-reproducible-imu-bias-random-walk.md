# 2026-09-09: Reproducible IMU Bias Random Walk

## Milestone objective

This milestone adds explicit, reproducible bias-state evolution for the accelerometer and
gyroscope. Two public functions advance caller-owned three-axis bias vectors by one
continuous-time random-walk step using caller-owned NumPy generators.

The increment establishes the discrete mathematics, NED/FRD interpretation, units,
validation order, fixed random-consumption policy, state ownership, and bounded deterministic
evidence. It does not add hidden sensor state, sample scheduling, estimation, robustness, or
hardware fidelity. Sprint 3, Week 6 remains in progress, and Gate G1 remains open.

## Repository state before documentation

The source-and-test increment was completed on synchronized `main` at:

```text
962cc1a feat: add reproducible accelerometer white noise
```

Before documentation, the working tree contained exactly:

```text
 M src/quadrotor_math/imu.py
 M tests/unit/test_imu.py
```

Nothing was staged. The protected implementation checksums were:

```text
cf20fd990f169a289849ff9fa39b68ade5ec32f8781c4960ea72f3f5a9dabf6d  src/quadrotor_math/imu.py
e4622c8b6fcc391f14320c92c9dc17a652bfd481dd973712938f8d1f6c794cac  tests/unit/test_imu.py
```

## Public API and module ownership

The public functions are:

```text
accelerometer_bias_random_walk_step_body(
    current_accelerometer_bias_B: NDArray[np.float64],
    accelerometer_bias_random_walk_density_B: NDArray[np.float64],
    time_step: float,
    rng: Generator,
) -> NDArray[np.float64]

gyroscope_bias_random_walk_step_body(
    current_gyroscope_bias_B: NDArray[np.float64],
    gyroscope_bias_random_walk_density_B: NDArray[np.float64],
    time_step: float,
    rng: Generator,
) -> NDArray[np.float64]
```

Both belong in `src/quadrotor_math/imu.py`. They own one mathematical state transition only;
they do not own generator construction, sampling schedules, complete sensor objects,
dynamics, integration, estimation, or simulation.

No shared helper, dataclass, sensor hierarchy, configuration object, or hidden state was
introduced. The caller retains the current bias and passes the returned bias into a later
step when continued evolution is wanted.

## Discrete random-walk mathematics

Both functions implement the component-wise update

```text
b_(k+1) = b_k + sigma_b * sqrt(dt) * z_k
z_k ~ N(0, I_3)
```

where `b_k` is the current bias, `sigma_b` is the nonnegative per-axis continuous-time
random-walk density, `dt` is the positive `time_step` in seconds, and `z_k` is the
dimensionless three-axis standard-normal sample returned by `rng.standard_normal(3)`.

The standard deviation of a continuous-time random-walk increment grows with the square root
of elapsed time. Multiplication by `sqrt(time_step)` therefore makes increment variance scale
linearly with elapsed time. Scaling density directly by `time_step` would instead make that
variance scale quadratically and would not implement the stated density contract.

## Frame, shape, and units contract

The project world frame remains north-east-down (NED), and the body frame remains
forward-right-down (FRD). These functions operate entirely on body-aligned quantities, so no
world/body attitude transformation occurs within either update.

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `current_accelerometer_bias_B` | `(3,)` | body-aligned FRD axes | m/s² |
| `accelerometer_bias_random_walk_density_B` | `(3,)` | body-aligned FRD axes | (m/s²)/sqrt(s) |
| returned accelerometer bias | `(3,)` | body-aligned FRD axes | m/s² |
| `current_gyroscope_bias_B` | `(3,)` | body-aligned FRD axes | rad/s |
| `gyroscope_bias_random_walk_density_B` | `(3,)` | body-aligned FRD axes | (rad/s)/sqrt(s) |
| returned gyroscope bias | `(3,)` | body-aligned FRD axes | rad/s |
| `time_step` | scalar | not frame-resolved | s |
| `z_k` | `(3,)` | component-aligned draw | dimensionless |

The typed public contract uses float64 arrays. Each density multiplied by the square root of
seconds yields its corresponding bias-increment units.

## Caller-owned state and RNG

The caller owns both the bias state and the NumPy `Generator`. Neither function creates,
seeds, resets, replaces, or hides a generator. Neither function mutates either caller-owned
input vector.

Every valid call consumes exactly one vectorized draw:

```text
standard_normal_sample_B = rng.standard_normal(3)
```

This fixed policy includes valid zero-density calls. Their returned bias equals the current
bias, but the draw is still consumed so subsequent seeded samples do not shift when density
configuration changes between zero and nonzero values.

Rejected calls consume no random values because every validation guard executes before
sampling. A zero-time-step rejection was independently checked against an untouched reference
generator to establish this boundary for the gyroscope function.

This ownership and consumption contract supports deterministic replay for equal seeds, equal
inputs, and equal call sequences using the configured generator. It does not promise
identical random bitstreams across NumPy versions or different bit generators.

## Validation contract and error messages

Each function validates and executes in this order:

1. current-bias shape;
2. density shape;
3. current-bias finiteness;
4. density finiteness;
5. density nonnegativity;
6. `time_step` finiteness;
7. strict `time_step` positivity;
8. one `rng.standard_normal(3)` draw; and
9. bias-state calculation.

The exact accelerometer error messages are:

```text
current_accelerometer_bias_B must have shape (3,)
accelerometer_bias_random_walk_density_B must have shape (3,)
current_accelerometer_bias_B must contain only finite values
accelerometer_bias_random_walk_density_B must contain only finite values
accelerometer_bias_random_walk_density_B must be nonnegative
time_step must be finite
time_step must be positive
```

The exact gyroscope error messages are:

```text
current_gyroscope_bias_B must have shape (3,)
gyroscope_bias_random_walk_density_B must have shape (3,)
current_gyroscope_bias_B must contain only finite values
gyroscope_bias_random_walk_density_B must contain only finite values
gyroscope_bias_random_walk_density_B must be nonnegative
time_step must be finite
time_step must be positive
```

Exact shape guards prevent malformed vectors from reaching broadcasting. Finiteness is
checked before density sign, and non-finite time steps are distinguished from finite zero or
negative time steps. All failures occur before random sampling.

## Output ownership and nonmutation

For valid float64 vectors, NumPy arithmetic produces an independently allocated float64
result with exact shape `(3,)`. The accelerometer ownership characterization confirms that
the returned state shares memory with neither input and that both caller-owned inputs retain
their original values. The gyroscope implementation uses the same direct expression and
ownership structure.

No explicit in-place update, input coercion, or hidden state mutation is performed.

## TDD progression

Development proceeded in bounded RED/GREEN increments:

1. Initial accelerometer and gyroscope tests imported their respective public random-walk
   functions before those names existed, producing missing-import REDs. Each minimum GREEN
   added the sensor-specific public function and direct seeded update equation.
2. Accelerometer behavioral increments fixed the one-vector draw contract, zero-density draw
   consumption, same-seed replay, successive-draw advancement, square-root time scaling,
   independently allocated output, and bounded population moments.
3. Bounded accelerometer validation RED/GREEN steps established current-bias and density
   shapes, their finiteness, density nonnegativity, finite time, strict positive time, and
   rejected-call RNG preservation before sampling.
4. Gyroscope-specific bounded validation RED/GREEN steps independently established its
   current-bias and density shapes, current-bias and density finiteness, and density
   nonnegativity with exact messages.
5. The final combined gyroscope time-step RED contained five parameter cases: NaN and both
   infinities required the finite-time message, while zero and `-0.25` required the
   positive-time message. A sixth focused case required a rejected zero-time call to preserve
   generator state. All six first failed because time validation was absent.
6. The minimum combined GREEN added the finite and strict-positive time guards after density
   validation and before sampling. All six focused cases then passed, followed by all 118 IMU
   cases without failures, errors, skips, or warnings.

## Verified behavioral coverage

The milestone adds 39 collected IMU cases. Accelerometer tests concentrate the broader
stochastic characterization in one implementation, covering:

- the exact seeded update equation;
- exactly one vectorized draw per valid call;
- draw consumption at zero density;
- same-seed trajectory replay;
- successive generator draws across calls;
- rejected-call RNG preservation;
- square-root time scaling;
- independent output ownership and input nonmutation; and
- deterministic population mean and sample-standard-deviation evidence.

Gyroscope tests independently establish its public seeded equation and sensor-specific
validation boundaries: current-bias shape and finiteness, density shape, finiteness, and
nonnegativity, time-step finiteness and strict positivity, and rejected-call RNG preservation.

## Deterministic accelerometer population evidence

The deterministic population characterization uses seed `12345` and 20,000 bias increments.
The empirical mean increments are:

```text
[2.2981696089236184e-05,
 0.0002734704022983798,
 -0.0001272922915769823]
```

The empirical sample standard deviations are:

```text
[0.01006613199090333,
 0.019974937616183835,
 0.02964885532805995]
```

The expected standard deviations are:

```text
[0.01, 0.02, 0.03]
```

Mean errors in standard-error units are:

```text
[0.3250102629573453,
 1.9337277591899757,
 0.6000616171123963]
```

Sample-standard-deviation errors in standard-error units are:

```text
[1.322606751657821,
 0.2506175724873806,
 2.3409059547567734]
```

Every normalized error is strictly below `5.0`. This finite deterministic sample supports
the configured first two moments on all three axes. It does not prove Gaussianity,
independence, stationarity, hardware fidelity, or cross-version reproducibility.

## Verification evidence before the final repository gate

Before documentation and the final repository gate:

- Ruff lint passed for `src/quadrotor_math/imu.py` and `tests/unit/test_imu.py`.
- Ruff non-modifying format checks reported both files already formatted.
- The complete IMU module passed 118 tests.
- No failures, errors, skips, or warnings occurred.
- `git diff --check` passed.
- `git diff --cached --check` passed.
- Nothing was staged.

## Final repository gate

The final repository gate for this documented working tree is run exactly once with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

Successful completion requires:

- Ruff lint to pass;
- Ruff non-modifying format verification to pass;
- strict mypy over `src` and `experiments` to report no issues;
- pytest to collect and pass exactly 348 tests; and
- no failures, errors, skips, or warnings.

This progress record represents the milestone as fully verified only after that one command
succeeds with these results.

## Limitations and non-claims

This milestone does not establish:

- Gaussianity, independence, or stationarity from the finite population sample;
- hardware fidelity or calibrated sensor behavior;
- identical random streams across NumPy versions or different bit generators;
- sample scheduling, timestamps, latency, or automatic step-size selection;
- temperature, vibration, mounting, or lever-arm effects;
- scale-factor, cross-axis, or misalignment errors;
- saturation, clipping, or quantization;
- saved replay artifacts or Monte Carlo campaigns;
- state estimation, controller behavior, or robustness evidence; or
- ROS 2, PX4, or simulator integration.

## Gate G1 and sprint status

Sprint 3, Week 6 remains in progress.

Gate G1 remains open and only partially satisfied. This milestone adds explicit
accelerometer and gyroscope bias-random-walk state transitions, continuous-time density
scaling, caller-owned state and RNG behavior, pre-sampling validation, rejected-call RNG
preservation, and bounded first-two-moment evidence. It does not establish all remaining
sensor, uncertainty, estimation, robustness, saved-replay, or integration evidence required
by the gate.

The near-term roadmap remains to continue Gate G1 work without claiming completion until its
remaining deterministic and stochastic evidence is implemented. No successor increment is
inferred by this record.

## Files changed by the completed milestone

The completed milestone consists of:

```text
src/quadrotor_math/imu.py
tests/unit/test_imu.py
README.md
docs/progress/2026-09-09-reproducible-imu-bias-random-walk.md
```

The documentation step changes only `README.md` and this new progress record. The source and
test changes predate documentation and retain their verified checksums.

Nothing is staged, committed, or pushed by this documentation increment.
