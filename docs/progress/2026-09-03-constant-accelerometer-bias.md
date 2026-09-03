# 2026-09-03: Constant Accelerometer Bias

## Milestone purpose and scope

This milestone adds one deterministic accelerometer-error layer after the ideal specific-force
boundary. The public function `accelerometer_specific_force_with_bias_body(...)` in
`src/quadrotor_math/imu.py` adds a supplied constant three-axis bias to an ideal accelerometer
specific-force vector while preserving explicit frame, unit, validation, and ownership
contracts.

The increment adds no stochastic behavior, hidden state, estimator, controller, robustness
claim, or simulator integration. It establishes deterministic constant accelerometer bias
only. Gate G1 remains open.

## Why the bias is an additive body-frame vector

A body-aligned accelerometer reports three signed components along the vehicle's sensor axes.
A constant sensor bias is therefore represented as a three-axis offset in the same
forward-right-down (FRD) body frame and the same m/s² units as the ideal specific force. The
measurement equation is component-wise:

```text
f_measured,B = f_ideal,B + b_accelerometer,B
```

This direct representation keeps each axis offset explicit. The function treats the supplied
bias as constant for the call; it does not estimate, initialize, or evolve it.

## Quantity contract

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `ideal_specific_force_B` | `(3,)` | body-aligned FRD axes | m/s² |
| `accelerometer_bias_B` | `(3,)` | body-aligned FRD axes | m/s² |
| returned biased specific force | `(3,)` | body-aligned FRD axes | m/s² |

Positive and negative bias components act along the signed FRD body axes. A negative finite
component is valid and represents bias in the negative direction of the corresponding axis.
The bias is specific-force offset in m/s², not angular rate in rad/s.

## Complete production API and behavior

```python
def accelerometer_specific_force_with_bias_body(
    ideal_specific_force_B: NDArray[np.float64],
    accelerometer_bias_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return biased accelerometer specific force in the FRD body frame.

    Args:
        ideal_specific_force_B: Ideal body-aligned accelerometer output expected
            to have shape ``(3,)``, expressed in the forward-right-down (FRD)
            body frame in m/s².
        accelerometer_bias_B: Constant additive three-axis bias expected to have
            shape ``(3,)``, expressed in the FRD body frame in m/s².

    Returns:
        Biased body-frame specific force in m/s².

    Raises:
        ValueError: If either input does not have shape ``(3,)`` or contains a
            non-finite value.
    """
    if ideal_specific_force_B.shape != (3,):
        raise ValueError("ideal_specific_force_B must have shape (3,)")

    if accelerometer_bias_B.shape != (3,):
        raise ValueError("accelerometer_bias_B must have shape (3,)")

    if not np.all(np.isfinite(ideal_specific_force_B)):
        raise ValueError("ideal_specific_force_B must contain only finite values")

    if not np.all(np.isfinite(accelerometer_bias_B)):
        raise ValueError("accelerometer_bias_B must contain only finite values")

    return ideal_specific_force_B + accelerometer_bias_B
```

The implemented calculation is exactly:

```text
specific_force_with_bias_B
    = ideal_specific_force_B + accelerometer_bias_B
```

## Validation order and exact messages

Validation and execution occur in this exact order:

1. Ideal-specific-force shape.
2. Accelerometer-bias shape.
3. Ideal-specific-force finiteness.
4. Accelerometer-bias finiteness.
5. Addition only after validation succeeds.

The exact `ValueError` messages are:

```text
ideal_specific_force_B must have shape (3,)
accelerometer_bias_B must have shape (3,)
ideal_specific_force_B must contain only finite values
accelerometer_bias_B must contain only finite values
```

No validation imposes a sign or magnitude restriction on finite components.

## Broadcasting hazard

Without exact shape validation, NumPy can combine a malformed `(3, 1)` array with a valid
`(3,)` vector and broadcast them to a `(3, 3)` matrix:

```text
(3, 1) + (3,) -> (3, 3)
```

That arithmetic is valid to NumPy but violates this API's three-axis vector contract. The two
shape guards prevent either malformed input from silently becoming a matrix.

## Ownership semantics

For the validated arrays used by this API, NumPy addition produces an independently owned
result. Tests confirm that the returned array shares memory with neither
`ideal_specific_force_B` nor `accelerometer_bias_B`.

Production performs no separate explicit copy operation and makes no dtype conversion. The
typed public contract supplies float64 arrays, and the direct addition already creates the
required result storage.

## TDD sequence

The behavior was developed through these bounded increments:

1. **Missing-function RED.** Importing the new public function failed during collection with
   the intended missing-name `ImportError`.
2. **Minimal additive GREEN.** The function was added with the direct component-wise NumPy
   addition.
3. **Passing ownership characterization.** The addition result already shared memory with
   neither input, so no production copy was added.
4. **Invalid ideal-shape RED and GREEN.** A malformed `(3, 1)` ideal input initially
   broadcast instead of raising; the exact ideal-shape guard corrected it.
5. **Invalid bias-shape RED and GREEN.** A malformed `(3, 1)` bias initially broadcast
   instead of raising; the exact bias-shape guard corrected it.
6. **Non-finite ideal-value RED and GREEN.** Parameterized NaN, positive-infinity, and
   negative-infinity cases initially propagated into the result; the ideal finiteness guard
   corrected all three.
7. **Non-finite bias-value RED and GREEN.** Parameterized NaN, positive-infinity, and
   negative-infinity cases initially propagated into the result; the bias finiteness guard
   corrected all three.
8. **Passing zero-bias boundary characterization.** A zero bias returned values exactly equal
   to the ideal specific force without a production change.

## Verified test coverage

The focused tests establish:

- addition of a known constant three-axis bias;
- independently owned output with no shared memory with either input;
- rejection of malformed ideal and bias shapes;
- rejection of NaN, positive infinity, and negative infinity in either input;
- acceptance of valid finite negative components; and
- exact equality with the ideal specific force for zero bias.

The complete IMU test module passes 59 tests. The complete repository passes 289 tests.

## Complete quality-gate evidence

Before this new progress file existed, the verification-only repository gate ran with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

It reported:

- Ruff lint passed;
- Ruff non-modifying format verification passed with 47 files already formatted;
- strict mypy over `src experiments` passed with no issues in 12 source files;
- pytest collected and passed 289 tests in 1.08 seconds;
- no tests failed, skipped, or errored;
- no warnings were reported;
- `git diff --check` passed; and
- verification did not change repository contents.

## Explicit exclusions

This milestone does not add or claim:

- accelerometer white noise;
- an RNG input or hidden RNG state in the bias-only function;
- bias drift or random walk;
- scale-factor or cross-axis error;
- saturation, clipping, quantization, latency, or sample scheduling;
- automatic unit or frame conversion; or
- integration into the simulation pipeline.

It also does not establish estimator performance, closed-loop control, robustness campaigns,
hardware fidelity, ROS 2, PX4, or simulator behavior.

## Gate G1 implications

Gate G1 remains open. This milestone adds deterministic constant accelerometer bias only; it
does not imply that the full accelerometer measurement model is complete. Accelerometer noise
and the remaining gate evidence are still outstanding.

## Documentation increment and worktree scope

This documentation increment changes only `README.md` and this new progress record. The
source and test changes predate it and are protected with before-and-after SHA-256 checksums.
No existing progress record is modified. Nothing is staged, committed, or pushed by this
increment.

## Next exact action

Add the first RED test for reproducible accelerometer white noise. Use a caller-owned seeded
RNG, follow the established gyroscope white-noise design, and preserve deterministic replay
and explicit random-consumption semantics.
