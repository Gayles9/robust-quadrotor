# 2026-09-02: Constant Gyroscope Bias

## Milestone purpose

This milestone adds one deterministic sensor-error layer after the ideal body-aligned
gyroscope boundary. The public function
`gyroscope_angular_velocity_with_bias_body(...)` in `src/quadrotor_math/imu.py` adds a supplied
constant three-axis bias to an ideal gyroscope measurement while preserving explicit frames,
units, validation, and output ownership.

The increment separates rigid-body truth, ideal sensing, and one deterministic measurement
error without introducing randomness or state. It makes no stochastic, estimation,
robustness, control, or hardware-fidelity claim. Gate G1 remains open.

## Repository state before the milestone

The milestone began from a clean synchronized `main` worktree at:

```text
14f92b9 feat: add ideal gyroscope angular velocity
```

That commit contained the ideal accelerometer and ideal gyroscope boundaries. It had no
constant-bias production function or bias tests. After the source-and-test increment was
completed and verified, but before this documentation increment, the only worktree changes
were these unstaged files:

```text
 M src/quadrotor_math/imu.py
 M tests/unit/test_imu.py
```

Nothing was staged.

## Physical meaning and measurement equation

A constant additive gyroscope bias is a fixed offset resolved along each sensor axis. For a
body-aligned sensor, both the ideal angular velocity and the bias use the forward-right-down
(FRD) body axes. The biased measurement is:

```text
angular_velocity_with_bias_B
    = ideal_angular_velocity_B + gyroscope_bias_B
```

The addition is component-wise. No magnitude or sign restriction is imposed: positive,
negative, and zero finite bias components are physically valid under this mathematical
contract. The function treats `gyroscope_bias_B` as a supplied constant vector for the call;
it neither estimates nor evolves that vector.

## Frame, shape, and unit contract

The project world frame remains north-east-down (NED), and the body frame is
forward-right-down (FRD). No world-frame quantity enters this body-aligned bias boundary.

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `ideal_angular_velocity_B` | `(3,)` | body-aligned FRD axes | rad/s |
| `gyroscope_bias_B` | `(3,)` | body-aligned FRD axes | rad/s |
| returned biased measurement | `(3,)` | body-aligned FRD axes | rad/s |

The typed public contract uses float64 NumPy arrays. The three components correspond to
angular velocity about body forward, right, and down, respectively.

## Truth, ideal output, and biased measurement

The implemented layering is:

```text
truth omega_B
    -> ideal gyroscope output
    -> deterministic additive bias
    -> biased measurement
```

`ideal_gyroscope_angular_velocity_body(...)` owns the truth-to-ideal boundary and protects
caller-owned truth state with an independently owned float64 ideal measurement.
`gyroscope_angular_velocity_with_bias_body(...)` then owns only the next deterministic
addition. Keeping these steps separate prevents an ideal sensor definition from silently
acquiring an error model and leaves future stochastic behavior as another explicit layer.

No simulator integration was necessary for this small sensor-error boundary. Direct tests of
the equation, validation, and memory behavior provide the relevant evidence without coupling
the function to propagation or state-history code.

## Why bias preceded noise

Constant bias is deterministic, has no sampling semantics, and requires no RNG ownership. It
therefore establishes the first non-ideal measurement layer while keeping its physical and
software contract directly inspectable. Adding it before noise isolates component-wise offset
behavior, validation, and ownership from later questions about draws, seeds, sequence
reproducibility, and per-sample statistics.

No stochastic claims have been made. In particular, this milestone provides no random
sequence, distribution, statistical characterization, seed record, or saved replay.

## Public API and module ownership

```python
def gyroscope_angular_velocity_with_bias_body(
    ideal_angular_velocity_B: NDArray[np.float64],
    gyroscope_bias_B: NDArray[np.float64],
) -> NDArray[np.float64]: ...
```

The function belongs to `src/quadrotor_math/imu.py` beside the ideal accelerometer and
gyroscope boundaries. It does not calculate dynamics, transform frames, advance a simulation,
or own sensor configuration, so it does not belong in the dynamics, rotations, or simulation
modules.

No generic helper, dataclass, sensor hierarchy, measurement container, or package export
change was introduced. The existing IMU module and one direct NumPy expression are sufficient
for this increment.

## Why a supplied three-axis vector

A three-axis input states the full physical quantity directly: each FRD sensor axis can have
its own constant offset, with independent sign and magnitude. It also keeps ownership with the
caller and makes repeat calls deterministic and transparent.

A scalar bias was not selected because silently applying one number to all three axes would
discard the axis-specific contract or require an implicit broadcasting policy. Hidden mutable
sensor state was deferred because this function is a stateless mathematical boundary.
Initialization draws were deferred because they require a distribution and RNG-ownership
contract. Bias random walk was deferred because it additionally requires state evolution,
sample time, process-noise units, and reproducibility decisions.

## Output ownership and mutation protection

For valid vectors, NumPy addition produces the independently owned result:

```python
return ideal_angular_velocity_B + gyroscope_bias_B
```

The returned array shares no memory with either the ideal measurement or the supplied bias.
Neither upstream input is mutated, so later downstream mutation of the result cannot corrupt
either input.

No explicit copy or dtype-conversion logic was added to the bias function. The typed public
contract uses float64 arrays, and ordinary addition of the two valid float64 vectors produces
the float64 result with its own storage.

## Validation order and exact errors

Validation occurs in this exact order:

1. `ideal_angular_velocity_B` has shape `(3,)`;
2. `gyroscope_bias_B` has shape `(3,)`;
3. every `ideal_angular_velocity_B` component is finite;
4. every `gyroscope_bias_B` component is finite; and
5. the two vectors are added.

The exact `ValueError` messages are:

| Invalid condition | Exact message |
| --- | --- |
| Invalid ideal-measurement shape | `ideal_angular_velocity_B must have shape (3,)` |
| Invalid bias shape | `gyroscope_bias_B must have shape (3,)` |
| Non-finite ideal-measurement entry | `ideal_angular_velocity_B must contain only finite values` |
| Non-finite bias entry | `gyroscope_bias_B must contain only finite values` |

The function rejects malformed or non-finite inputs rather than reshaping, repairing,
clipping, or propagating them.

## TDD history and observed failures

The behavior was developed in these small increments:

1. **Missing-name RED.** The first bias-addition test imported
   `gyroscope_angular_velocity_with_bias_body` from the existing IMU module. Test collection
   failed because that public name did not exist.
2. **Minimum addition GREEN.** The function was added with the direct NumPy addition needed
   to produce the expected component-wise mixed-sign result.
3. **Passing ownership characterization.** The direct addition already returned independent
   storage. `np.shares_memory` confirmed that the output aliased neither input, so no explicit
   copy was necessary and this characterization passed without a production change.
4. **Ideal-shape RED/GREEN.** A malformed ideal measurement initially reached NumPy addition
   instead of raising the required error. An exact `(3,)` ideal-input guard and message made
   the test pass.
5. **Bias-shape RED/GREEN.** A malformed bias initially reached NumPy addition instead of
   raising the required error. An exact `(3,)` bias guard and message made the test pass.
6. **Ideal-finiteness RED/GREEN.** NaN and both infinities in the ideal measurement initially
   propagated through addition. The ideal-input finiteness guard and exact message made all
   three parameterized cases pass.
7. **Bias-finiteness RED/GREEN.** NaN and both infinities in the bias initially propagated
   through addition. The bias finiteness guard and exact message made all three parameterized
   cases pass.

Before exact shape validation, NumPy broadcasting was unsafe for this API: adding a `(3, 1)`
array and a `(3,)` vector can produce a `(3, 3)` result instead of rejecting the malformed
input. Shape guards were added at the boundary to prevent that numerically valid but
contractually incorrect behavior.

## Test inventory and claims

The focused bias tests in `tests/unit/test_imu.py` are:

```text
test_gyroscope_angular_velocity_with_bias_body_adds_constant_bias
test_gyroscope_angular_velocity_with_bias_body_returns_independent_measurement
test_gyroscope_angular_velocity_with_bias_body_rejects_invalid_ideal_shape
test_gyroscope_angular_velocity_with_bias_body_rejects_invalid_bias_shape
test_gyroscope_angular_velocity_with_bias_body_rejects_nonfinite_ideal_values
test_gyroscope_angular_velocity_with_bias_body_rejects_nonfinite_bias_values
```

They establish:

- component-wise addition for mixed-sign ideal and bias components;
- independently owned output with no shared memory with either upstream input;
- separate exact-shape rejection and messages for the ideal measurement and bias; and
- independent rejection of NaN, positive infinity, and negative infinity in each input.

The six test definitions collect ten executed cases: two direct behavior cases, two shape
cases, three parameterized ideal-finiteness cases, and three parameterized bias-finiteness
cases. The complete IMU module contains 28 passing cases. No dedicated zero-bias
characterization test was added.

## Complete repository-gate evidence

For the verified working tree on 2026-09-02, the complete gate was run with a writable
task-specific uv cache:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

It reported:

- Ruff lint passed;
- Ruff non-modifying format verification passed for 45 files;
- strict mypy passed over `src` and `experiments` with no issues in 12 source files;
- pytest collected and passed 258 tests;
- no tests failed, skipped, or errored;
- no warnings were emitted; and
- `git diff --check` passed.

## Limitations and non-claims

This milestone adds only a caller-supplied constant additive bias vector. It does not add:

- white noise;
- bias random walk or drift;
- RNG use, ownership, seeds, or initialization draws;
- sample time or time evolution;
- scale factors;
- axis misalignment or cross-axis sensitivity;
- saturation or clipping;
- quantization;
- latency or sampling policy;
- temperature, calibration, or hardware effects; or
- generic sensor/configuration abstractions.

It does not establish sensor statistics, saved stochastic replay, state estimation,
closed-loop control, Monte Carlo robustness, hardware fidelity, ROS 2, PX4, or simulator
integration. It also does not prove that upstream dynamics or the ideal gyroscope output are
correct beyond their separately owned tests.

## Gate G1 status

Gate G1 remains open. This milestone establishes deterministic
truth/ideal/biased-measurement separation. It does not establish stochastic reproducibility,
sensor statistics, saved replay, estimation, robustness, deliberate uncertainty, or hardware
realism.

## Documentation increment and worktree scope

This documentation increment changes only `README.md` and this progress record. The source
and test modifications predate it and were protected with before-and-after SHA-256 checksums.
Nothing is staged, committed, or pushed by this increment.

## Exact next action

Add the first RED test for a gyroscope measurement with caller-owned RNG and three-axis
per-sample white-noise standard deviation, following the completed stochastic IMU design
review.
