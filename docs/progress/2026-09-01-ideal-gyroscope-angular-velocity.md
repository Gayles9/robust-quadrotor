# 2026-09-01: Ideal Gyroscope Angular Velocity

## Milestone purpose

This milestone adds one ROS/PX4-independent mathematical boundary for an ideal body-aligned
gyroscope. The public function `ideal_gyroscope_angular_velocity_body(...)` in
`src/quadrotor_math/imu.py` accepts rigid-body angular velocity already expressed in the
forward-right-down body frame and returns the corresponding ideal measurement.

The boundary is deliberately deterministic and minimal. It establishes the ideal gyroscope
quantity, frame, units, validation, and returned-value ownership. It does not establish
realistic sensor behavior, stochastic reproducibility, state estimation, uncertainty, or
robustness. Gate G1 remains open.

## Repository state before the milestone

The milestone began on branch `main` at:

```text
095d790 feat: add ideal accelerometer specific force
```

That synchronized commit already contained the ideal accelerometer specific-force boundary,
the NED/FRD frame contract, rigid-body dynamics, explicit-Euler and projected-RK4 propagation,
deterministic simulation, structural history validation, and the existing physical and
numerical characterizations. No gyroscope production function or gyroscope test existed.

## Physical interpretation

An ideal gyroscope measures rigid-body angular velocity relative to an inertial frame,
resolved along the sensor axes. For a sensor aligned with the body axes, its measurement is
the supplied body-frame angular velocity directly:

```text
gyroscope_output_B = omega_B
```

There is no sign change, axis reordering, gravity compensation, or attitude transformation.
The ideal output preserves each supplied body-rate component exactly.

Angular velocity is common to every point of an ideal rigid body. Position therefore does not
enter this measurement equation. Translational velocity and translational acceleration
describe different physical quantities and are also unnecessary. Gravity affects
translational dynamics but is not a gyroscope input. Attitude would be required only if the
angular velocity were supplied in a different frame; `omega_B` is already resolved along the
body-aligned sensor axes.

## Frame and unit contract

The project world frame is north-east-down (NED), and the body frame is forward-right-down
(FRD). NED remains the canonical world convention, but no world-frame quantity enters this
identity sensor boundary.

| Quantity | Shape | Frame | Units |
| --- | --- | --- | --- |
| `omega_B` | `(3,)` | FRD body | rad/s |
| returned gyroscope measurement | `(3,)` | body-aligned FRD sensor axes | rad/s |

The three components are angular velocities about body forward, right, and down respectively.
Positive components follow the right-hand rule. Zero and finite negative angular velocities
are physically valid. No arbitrary magnitude limit is imposed on this ideal mathematical
primitive.

## Public API and module ownership

```python
def ideal_gyroscope_angular_velocity_body(
    omega_B: NDArray[np.float64],
) -> NDArray[np.float64]: ...
```

The function belongs to `src/quadrotor_math/imu.py`, which already owns the ideal
accelerometer boundary. It does not belong to `dynamics.py` because it does not calculate or
propagate angular velocity, and it does not belong to `rotations.py` because no frame
transformation is performed.

No `gyroscope.py` module, generic sensor abstraction, configuration dataclass, measurement
container, helper function, or package export change was introduced. The existing IMU module
is sufficient ownership for this increment.

## Returned-value ownership

Although the ideal measurement has the same numerical values as `omega_B`, returning the
input object directly would alias caller-owned truth state. Downstream mutation of that
measurement could then corrupt the rigid-body state, and later mutation of truth state could
retroactively change an existing measurement.

The function therefore returns:

```python
np.array(
    omega_B,
    dtype=np.float64,
    copy=True,
)
```

The result is an independently owned float64 array. It shares no memory with the supplied
truth-state array.

## Validation contract and exact errors

The function implements two checks:

| Invalid condition | Exact `ValueError` message |
| --- | --- |
| `omega_B.shape != (3,)` | `omega_B must have shape (3,)` |
| Any non-finite entry | `omega_B must contain only finite values` |

Validation occurs in this order:

1. exact shape `(3,)`;
2. finiteness of every component; and
3. construction of the independently owned float64 result.

The shape check rejects a `(3, 1)` column matrix even though it contains three scalar values.
Accepting that matrix would violate the vector contract and allow unsafe broadcasting or
inconsistent state-history shapes. NaN, positive infinity, and negative infinity are rejected
because none is a finite physical angular velocity. Finite negative rates and zero are not
rejected, and no sensor-range or magnitude policy is invented.

## TDD sequence and observed RED failures

The implementation was developed through five small behavior increments:

1. **Missing-name RED.** The physical-value test imported
   `ideal_gyroscope_angular_velocity_body` from the existing `quadrotor_math.imu` module.
   Collection failed with `ImportError: cannot import name
   'ideal_gyroscope_angular_velocity_body'`; it was not a missing-module failure.
2. **Physical-value GREEN.** The function was introduced with the minimum direct return
   `return omega_B`, establishing only exact body-rate preservation.
3. **Ownership RED/GREEN.** `np.shares_memory` initially returned `True` and the ownership
   assertion failed. The return was replaced with an explicit independently owned float64
   copy, after which both value and ownership tests passed.
4. **Shape RED/GREEN.** A `(3, 1)` input initially produced `Failed: DID NOT RAISE
   ValueError`. The exact `(3,)` shape guard and error message were then added.
5. **Finiteness RED/GREEN.** The NaN, positive-infinity, and negative-infinity cases initially
   all produced `Failed: DID NOT RAISE ValueError`. A finiteness guard with the exact shared
   error message made all three cases pass.

Each RED failure was attributable to the one missing behavior under development. No
production behavior was hidden behind a broad abstraction or introduced ahead of its test.

## Test evidence

The focused tests in `tests/unit/test_imu.py` are:

```text
test_ideal_gyroscope_angular_velocity_body_returns_body_angular_velocity
test_ideal_gyroscope_angular_velocity_body_returns_independent_measurement
test_ideal_gyroscope_angular_velocity_body_rejects_invalid_shape
test_ideal_gyroscope_angular_velocity_body_rejects_nonfinite_values
```

Together they establish:

- exact preservation of mixed-sign `[0.7, -0.4, 1.1] rad/s` components;
- unchanged component ordering and signs in the FRD body frame;
- independent measurement memory rather than truth-state aliasing;
- rejection of a `(3, 1)` column matrix with the exact shape error; and
- rejection of NaN, positive infinity, and negative infinity with the exact finiteness error.

The four test definitions collect six cases because the finiteness test is parameterized over
three invalid values. The complete IMU test module contains 18 passing cases after this
milestone.

No simulator composition test was required. This function is an identity measurement
boundary over a supplied state quantity; passing simulator output through it would not add
evidence beyond the direct value, ownership, and boundary-validation tests. The function does
not validate the dynamics, integration, or simulation that generated `omega_B`.

## Implementation behavior

For a valid finite float64 vector, the function:

1. verifies exact shape `(3,)`;
2. verifies that all three entries are finite; and
3. returns an independent float64 array with identical values.

The implementation performs no matrix multiplication, gravity subtraction, integration,
clipping, normalization, calibration, time update, or random sampling.

## Scientific and architectural decisions

- Model the ideal body-aligned measurement as `gyroscope_output_B = omega_B`.
- Preserve the repository's established FRD `omega_B` name and rad/s units.
- Require exact vector shape rather than reshaping or repairing malformed inputs.
- Reject non-finite values rather than propagating invalid sensor outputs.
- Return an independent float64 array to protect caller-owned truth state.
- Accept zero, negative finite rates, and all finite magnitudes.
- Keep the function beside the ideal accelerometer in the existing IMU module.
- Keep the boundary deterministic and free of state, configuration, and RNG ownership.
- Avoid a generic sensor abstraction or configuration dataclass without demonstrated need.

## Deliberately deferred sensor effects

This milestone does not introduce:

- constant or time-varying gyroscope bias;
- white measurement noise;
- bias drift or bias random walk;
- random-number-generator inputs or ownership rules;
- seed or stochastic configuration records;
- scale-factor error;
- axis misalignment or cross-axis sensitivity;
- saturation or clipping;
- quantization;
- temperature dependence;
- sampling policy, sample timestamps, or latency;
- measurement containers or covariance fields;
- sensor configuration dataclasses;
- stateful sensor models; or
- generic sensor interfaces.

Those effects require explicit physical equations, units, time semantics, configuration, and
reproducibility decisions. Adding them to this identity boundary would obscure rather than
clarify its contract.

## Complete quality-gate evidence

For the verified working tree on 2026-09-01, the canonical `make check` gate reported:

- Ruff lint passed;
- Ruff non-modifying formatting verification passed for 44 files;
- strict mypy passed over `src` and `experiments` with no issues in 12 source files;
- the complete pytest suite passed 248 tests;
- no tests failed, skipped, or errored;
- no warnings were emitted; and
- `git diff --check` passed.

## Limitations and non-claims

Correct ideal output means only that a valid supplied `omega_B` is preserved in the declared
frame and units with independent memory. It is not evidence of realistic sensor behavior or
hardware fidelity. It does not prove that the dynamics produced the correct angular velocity,
and it does not establish simulation accuracy.

The milestone makes no stochastic claim and provides no statistical characterization,
reproducible noise sequence, replay artifact, truth-versus-nominal parameter separation, or
uncertainty model. It does not implement or validate estimation, control, robustness,
calibration, ROS 2, PX4, or simulator integration.

## Gate G1 status

Gate G1 remains open. This milestone establishes an ideal gyroscope boundary only. It does
not establish sensor realism, stochastic reproducibility, replay, estimation, uncertainty, or
robustness.

## Current Git/worktree state

Before this documentation increment, the completed milestone consisted of two unstaged
modified files:

```text
 M src/quadrotor_math/imu.py
 M tests/unit/test_imu.py
```

This progress record documents that verified working tree. The documentation increment adds
only `README.md` and this progress record; it does not change production code or tests.

## Exact next action

Conduct a read-only stochastic IMU sensor-model and RNG-ownership design review before
introducing bias or noise tests.
