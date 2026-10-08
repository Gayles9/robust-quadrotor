# 2026-09-01: Ideal Accelerometer Specific Force

## Milestone purpose

This milestone adds one ROS/PX4-independent mathematical boundary for ideal accelerometer
specific force. The public function
`ideal_accelerometer_specific_force_body(...)` in `src/quadrotor_math/imu.py` maps supplied
world-frame inertial translational acceleration and body attitude into the forward-right-down
body frame.

The function models the ideal specific force associated with the supplied translational
acceleration and attitude. It does not establish the broader claim that an accelerometer
always measures thrust. Gate G1 remains open.

## Public API

```python
ideal_accelerometer_specific_force_body(
    translational_acceleration_W,
    R_WB,
    gravity_acceleration,
)
```

The return value is a float64 vector with shape `(3,)` containing ideal accelerometer
specific force in the FRD body frame in m/s².

## Frame and unit contract

| Quantity | Shape | Frame or direction | Units |
| --- | --- | --- | --- |
| `translational_acceleration_W` | `(3,)` | NED world | m/s² |
| `R_WB` | `(3, 3)` | FRD body to NED world | dimensionless |
| `gravity_acceleration` | scalar | nonnegative magnitude | m/s² |
| returned `specific_force_B` | `(3,)` | FRD body | m/s² |

The world frame is north-east-down (NED), so positive world `z` points down. The body frame
is forward-right-down (FRD). `R_WB` maps body-coordinate vector components into world
coordinates, while `R_WB.T` maps world-coordinate vector components into body coordinates.

## Specific-force derivation

Let `a_W` be the supplied inertial translational acceleration in world coordinates. In the
uniform-gravity model,

```text
g_W = [0, 0, g]
```

where `g` is the finite nonnegative gravity magnitude. Specific force excludes gravitational
acceleration, so in world coordinates it is

```text
f_W = a_W - g_W.
```

Because `f_W` is expressed in world coordinates and the sensor result is required in body
coordinates, the body-to-world rotation must be inverted. A proper rotation matrix is
orthogonal, so its inverse is its transpose:

```text
f_B = R_WB.T @ f_W
    = R_WB.T @ (a_W - g_W).
```

Gravity is subtracted because an ideal accelerometer reports specific force, not the
gravity-inclusive inertial acceleration supplied as `a_W`. `R_WB.T` is used because `R_WB`
maps in the opposite direction, from body coordinates into world coordinates.

## Worked tilted-thrust calculation

The tilted characterization uses

```text
a_W = [-4.0, 0.0, 9.81] m/s²
g_W = [ 0.0, 0.0, 9.81] m/s²

R_WB = [
    [ 0, 0, 1],
    [ 0, 1, 0],
    [-1, 0, 0],
]
```

Subtracting gravity gives

```text
a_W - g_W = [-4.0, 0.0, 0.0] m/s².
```

Rotating that world vector into the body frame gives

```text
f_B = R_WB.T @ [-4.0, 0.0, 0.0]
    = [0.0, 0.0, -4.0] m/s².
```

For this declared attitude and acceleration, the specific force maps to negative body `z`,
consistent with upward thrust in an FRD body frame. This is a scenario-specific physical
anchor, not a claim that every accelerometer output is thrust.

## Level rest, free fall, and zero gravity

At level rest or ideal hover,

```text
a_W = [0, 0, 0]
R_WB = identity
g_W = [0, 0, g]
f_B = [0, 0, -g].
```

The vehicle has zero inertial acceleration, but support or thrust opposes gravity, so an
ideal accelerometer reports upward specific force along negative FRD body `z`.

In gravity-only free fall,

```text
a_W = g_W
f_B = R_WB.T @ [0, 0, 0]
    = [0, 0, 0]
```

for any valid attitude. Gravity is present in the inertial acceleration, but free fall has
zero ideal specific force.

Zero gravity is the valid lower boundary of the scalar contract. With `g = 0` and identity
`R_WB`, `g_W` is zero and the returned specific force equals the supplied inertial
acceleration exactly.

## Validation contract and exact errors

The function implements these checks and exact `ValueError` messages:

| Invalid condition | Exact message |
| --- | --- |
| `translational_acceleration_W.shape != (3,)` | `translational_acceleration_W must have shape (3,)` |
| `R_WB.shape != (3, 3)` | `R_WB must have shape (3, 3)` |
| Non-finite acceleration entry | `translational_acceleration_W must contain only finite values` |
| Non-finite rotation entry | `R_WB must contain only finite values` |
| Non-finite or negative gravity magnitude | `gravity_acceleration must be finite and nonnegative` |

Validation occurs in this order:

1. `translational_acceleration_W` shape;
2. `R_WB` shape;
3. `translational_acceleration_W` finiteness;
4. `R_WB` finiteness;
5. `gravity_acceleration` finiteness and nonnegativity;
6. construction of `gravity_W`; and
7. calculation and return of specific force.

This function owns validation of the immediate array and scalar boundary. Proper-rotation
construction and correctness remain owned by the existing rotations layer. The accelerometer
function does not normalize, repair, or independently test `R_WB` orthogonality or determinant
`+1`.

## Test evidence

The focused IMU test module contains nine test definitions and twelve collected cases. They
establish:

- tilted specific force maps to negative body `z`;
- level rest or hover returns `[0, 0, -g]`;
- gravity-only free fall returns zero regardless of the tested tilted attitude;
- zero gravity is accepted and returns the supplied acceleration at identity attitude;
- malformed acceleration and rotation shapes raise their exact errors;
- non-finite acceleration and rotation entries raise their exact errors; and
- NaN, positive infinity, negative infinity, and a negative finite gravity value all raise
  the shared gravity-contract error.

All numerical characterizations use `rtol=0.0` and `atol=1e-12`.

## Complete quality-gate evidence

For the verified working tree on 2026-09-01:

- Ruff lint passed;
- Ruff formatting verification passed;
- strict mypy passed over `src` and `experiments` with no issues in 12 source files;
- the complete pytest suite passed 242 tests;
- no warnings were emitted; and
- `git diff --check` passed.

## Design decisions

- One small public function owns ideal accelerometer specific-force calculation.
- The equation remains explicit NumPy code without helpers or sensor classes.
- Validation rejects invalid inputs rather than normalizing, repairing, or clipping them.
- Zero gravity is accepted because gravity is a nonnegative magnitude, not a strictly positive
  parameter.
- Rotation orthogonality and determinant checks are not duplicated across mathematical
  layers.
- The milestone remains deterministic and does not introduce a random-number generator,
  configuration object, or stateful sensor model.

## Scientific limitations

The implemented boundary has:

- ideal, deterministic output only;
- no bias;
- no white noise or random walk;
- no scale-factor error;
- no axis misalignment;
- no saturation;
- no quantization;
- no temperature dependence;
- no sample timing or latency;
- no sensor offset from the centre of mass;
- no centripetal, tangential, or other lever-arm acceleration;
- no vibration;
- no mounting dynamics;
- no calibration model; and
- no stochastic reproducibility or seed/config record yet.

The result is only the ideal specific force associated with the supplied translational
acceleration and attitude under the declared uniform-gravity and frame conventions. It is not
a complete accelerometer measurement or hardware model.

## Deferred scope and Gate G1

This milestone defers gyroscope behavior, stochastic IMU behavior, sensor configuration,
state estimation, calibration, mounting and lever-arm physics, and simulator or flight-stack
integration. It also defers rotation-matrix orthogonality and determinant validation to the
existing rotations layer.

Gate G1 remains open because the project still lacks, among other planned items:

- stochastic sensor models and statistical tests;
- deterministic run seed/config records;
- saved replay artifacts;
- truth-versus-nominal parameter separation; and
- wind/drag and deliberate model mismatch.

## Current Git/worktree state

Before this documentation increment, the completed source and test files were untracked:

```text
?? src/quadrotor_math/imu.py
?? tests/unit/test_imu.py
```

This progress record documents that verified working tree rather than attributing the
milestone to a commit. No source or test file was changed during documentation.

After this documentation increment, the worktree scope is:

```text
 M README.md
?? docs/progress/2026-09-01-ideal-accelerometer-specific-force.md
?? src/quadrotor_math/imu.py
?? tests/unit/test_imu.py
```

The source and test entries predate the documentation increment; only `README.md` and this
progress record belong to the documentation change.

## Exact next action

Conduct a read-only ideal-gyroscope API and ownership design review before adding tests or
production code.
