# 0004: ESKF Error-State Conventions

Date: 2026-09-22

Status: Accepted

## Context

The first state-estimation increment needs a single, testable convention for nominal
IMU propagation and covariance prediction. The repository uses a north-east-down
(NED) world frame `W`, a forward-right-down (FRD) body and sensor frame `B`, and
Hamilton scalar-first quaternions. The quaternion `q_WB` maps body-coordinate vectors
into world coordinates and obeys

```text
q_dot_WB = 0.5 q_WB ⊗ [0, omega_B].
```

Mixing left- and right-local attitude errors would change Jacobian frames and signs.
This decision fixes those choices before measurement updates are introduced.

## Decision

### Nominal state

The nominal state is

```text
x_nominal = (
    position_W,             # (3,), NED, m
    velocity_W,             # (3,), NED, m/s
    q_WB,                   # (4,), unit Hamilton scalar-first, body to world
    accelerometer_bias_B,   # (3,), FRD, m/s²
    gyroscope_bias_B,       # (3,), FRD, rad/s
).
```

The quaternion sign is not canonicalized. State arrays are finite, independently
owned, C-contiguous `float64` values and are exposed read-only. Quaternion norm evaluation
converts finite arithmetic overflow or invalid arithmetic into the existing unit-norm
validation error without leaking a runtime warning; the strict unit-norm tolerance is
unchanged.

### Error state and injection

The 15-state error ordering is

```text
delta_x = [
    delta_position_W,           # 0:3
    delta_velocity_W,           # 3:6
    delta_theta_B,              # 6:9
    delta_accelerometer_bias_B, # 9:12
    delta_gyroscope_bias_B,     # 12:15
].
```

Position, velocity, and bias errors are additive, with bias errors defined as true
minus nominal. Attitude uses a right-multiplicative body-local error:

```text
R_true_WB = R_nominal_WB Exp(skew(delta_theta_B))
q_true_WB = q_nominal_WB ⊗ delta_q(delta_theta_B)

delta_q(phi) = [
    cos(norm(phi) / 2),
    sin(norm(phi) / 2) / norm(phi) phi,
].
```

The implementation uses a Taylor expansion near zero and normalizes at the existing
quaternion normalization boundary. Right multiplication matches the repository's
body-rate kinematics because both the angular rate and the local perturbation are
expressed in `B` for `q_dot_WB = 0.5 q_WB ⊗ [0, omega_B]`.

### IMU and gravity conventions

The measurement and bias signs are

```text
specific_force_measurement_B
    = specific_force_true_B + accelerometer_bias_true_B + n_a_B

angular_velocity_measurement_B
    = angular_velocity_true_B + gyroscope_bias_true_B + n_g_B.
```

Nominal propagation therefore subtracts both nominal biases. In NED coordinates,
gravity is `gravity_W = [0, 0, gravity_acceleration]`; positive world `z` is down. IMU
measurements and corrected acceleration and angular rate are constant over one
prediction interval. Nominal biases remain constant during that interval.

### Continuous error dynamics

Let `R = R_WB`,
`f = specific_force_measurement_B - accelerometer_bias_B`, and
`omega = angular_velocity_measurement_B - gyroscope_bias_B`. With 3-by-3 blocks and
`[.]x` denoting `skew(.)`, the continuous state matrix is

```text
        delta_p  delta_v  delta_theta  delta_b_a  delta_b_g
F = [      0        I          0           0          0     ]
    [      0        0       -R [f]x       -R          0     ]
    [      0        0       -[omega]x      0         -I     ]
    [      0        0          0           0          0     ]
    [      0        0          0           0          0     ].
```

The continuous process-noise ordering is

```text
w = [n_a_B, n_g_B, n_wa_B, n_wg_B],
```

where the final two terms drive accelerometer- and gyroscope-bias random walks. The
noise-input matrix is

```text
          n_a_B  n_g_B  n_wa_B  n_wg_B
G = [       0      0       0       0   ]
    [      -R      0       0       0   ]
    [       0     -I       0       0   ]
    [       0      0       I       0   ]
    [       0      0       0       I   ].
```

All unlisted blocks in `F` and `G` are exactly zero.

### First-order covariance prediction

For the continuous process-noise spectral-density matrix `Q_c` in the fixed noise
ordering, the high-rate baseline discretization is

```text
Phi = I + F dt
Q_d_raw = G Q_c G.T dt
Q_d = 0.5 (Q_d_raw + Q_d_raw.T).
```

The explicit final symmetrization changes only floating-point roundoff because the
mathematical `G Q_c G.T` is symmetric. The implementation preserves every diagonal and every
exactly equal mirrored pair without arithmetic, copies one exact representation to both
mirrored positions, and safely averages only genuinely unequal pairs. This retains exact
subnormal entries, produces bit-symmetric output including signed zeros, and avoids overflow
near the finite `float64` maximum. `Q_c` may contain correlations. It, the input covariance
`P`, and discrete process noise `Q_d` must be finite, symmetric, and positive semidefinite.
Symmetry is checked by scale-normalizing each unequal mirrored pair before comparison, so a
large unrelated diagonal cannot hide a local asymmetry and an underflowed absolute tolerance
cannot make unequal subnormal entries appear equal. Positive-semidefinite validation rejects
every explicit negative diagonal, accepts valid positive-subnormal diagonals, and normalizes
positive diagonal scales before its eigendecomposition, so a large eigenvalue cannot hide a
small indefinite principal block. A scale-aware eigenvalue tolerance admits only
floating-point eigensolver roundoff. The covariance prediction is

```text
P_next = Phi P Phi.T + Q_d
P_next = 0.5 (P_next + P_next.T).
```

For positive duration, the composed prediction builds `F` and `G` at the pre-propagation
nominal state, forms `Phi` and `Q_d`, propagates the nominal state, and then propagates `P`.
Prediction intervals are finite and nonnegative. At exactly zero duration, composed
prediction validates the IMU inputs and gravity through nominal propagation, validates
`Q_c` and the time step through zero-matrix discretization, and validates `P` through
covariance propagation without building `F/G` or subtracting biases. It returns independently
owned exact nominal-state and covariance copies; discretization returns exact identity `Phi`
and exact zero `Q_d` without avoidable arithmetic.

### Verification evidence

Unit tests compare every nonzero and zero block of `F` and `G` independently. A
15-column central-difference Jacobian injects positive and negative perturbations at a
nontrivial attitude, velocity, bias, specific force, and angular rate. It recovers the
next attitude error from `R_nominal_next.T @ R_perturbed_next` and compares the result
with `I + F dt` at `dt = 0.04, 0.02, 0.01, 0.005` seconds. The observed norm error
decreases approximately quadratically when the step is halved, as expected for a
first-order transition approximation.

Covariance tests cover diagonal and correlated positive-semidefinite noise, scales from the
smallest positive `float64` subnormal through approximately `1e24`, locally adversarial
symmetry and definiteness cases, fixed-seed randomized positive-definite inputs, zero process
noise, zero-duration prediction, and a 750-step prediction sequence. They verify finite
results, unit quaternion norm, bit-symmetric covariance, exact subnormal preservation, and
the absence of materially negative covariance eigenvalues.

## Consequences and limitations

This increment provides a deterministic mathematical prediction core, not a complete
estimator. First-order discretization is intended as a high-rate baseline; its error is
not equivalent to an exact matrix exponential. The propagation also assumes constant
IMU measurements and biases over each interval and uses a constant scalar gravity
magnitude.

It explicitly provides none of the following:

- measurement updates;
- a Kalman gain or innovation;
- a covariance reset Jacobian after measurement injection;
- delayed-measurement handling;
- an estimator runner;
- estimator configuration or a manifest schema;
- NIS/NEES analysis or a Monte Carlo consistency campaign;
- exact matrix-exponential or Van Loan discretization;
- Earth rotation, Coriolis effects, gravity estimation, or a geodetic model;
- controller, ROS 2, PX4, or C++ integration.
