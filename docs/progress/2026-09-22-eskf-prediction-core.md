# 15-State ESKF Prediction Core

- Date: 2026-09-22
- Status: Complete; ready for publication review
- Recommended subject: `feat: add ESKF prediction core`

## Scope

This milestone establishes the ROS/PX4-independent prediction mathematics for a 15-state
error-state Kalman filter (ESKF). It adds:

- an immutable nominal state;
- right-local error-state injection;
- bias-corrected IMU nominal propagation;
- continuous error-state and noise-input matrices `F` and `G`;
- first-order discrete transition and process-noise matrices `Phi` and `Q_d`;
- symmetric covariance propagation; and
- one composed prediction boundary using a consistent pre-step linearization.

This is prediction only. It does not add local-position or barometric measurement models,
measurement updates, an innovation, a Kalman gain, Joseph-form covariance updates,
post-update injection, a covariance-reset Jacobian, delayed-measurement handling, gating,
consistency monitoring, an estimator runner, configuration or manifest changes, artifact
changes, controller integration, ROS 2, PX4, or C++ integration.

The authoritative convention decision is
[ADR 0004](../decisions/0004-eskf-error-state-conventions.md). This record summarizes the
implementation and verification evidence without replacing that decision.

## Frames, state, and error convention

The world frame `W` is north-east-down (NED), and the body/sensor frame `B` is
forward-right-down (FRD). `q_WB` is a Hamilton scalar-first unit quaternion that actively
maps body-coordinate vectors into world coordinates and obeys

```text
q_dot_WB = 0.5 q_WB ⊗ [0, omega_B].
```

The nominal state is

```text
x_nominal = (
    position_W,             # (3,), NED, m
    velocity_W,             # (3,), NED, m/s
    q_WB,                   # (4,), Hamilton scalar-first, B to W
    accelerometer_bias_B,   # (3,), FRD, m/s²
    gyroscope_bias_B,       # (3,), FRD, rad/s
).
```

The local error ordering is

```text
delta_x = [
    delta_position_W,           # 0:3
    delta_velocity_W,           # 3:6
    delta_theta_B,              # 6:9
    delta_accelerometer_bias_B, # 9:12
    delta_gyroscope_bias_B,     # 12:15
].
```

Attitude error is right-multiplicative and body-local:

```text
R_true_WB = R_nominal_WB Exp(skew(delta_theta_B))
q_true_WB = q_nominal_WB ⊗ delta_q(delta_theta_B).
```

Position, velocity, and both bias errors are additive. Bias errors use true minus nominal.
The rotation-vector quaternion exponential uses a Taylor branch near zero and the existing
quaternion-normalization boundary after injection.

## Nominal IMU propagation

The IMU measurements follow the repository's additive-bias model. Prediction subtracts the
nominal biases:

```text
f_B = specific_force_measurement_B - accelerometer_bias_B
omega_B = angular_velocity_measurement_B - gyroscope_bias_B
gravity_W = [0, 0, gravity_acceleration]
acceleration_W = gravity_W + R_WB f_B.
```

With measurements treated as constant over one interval:

```text
position_next_W = position_W + velocity_W dt + 0.5 acceleration_W dt²
velocity_next_W = velocity_W + acceleration_W dt
q_next_WB = normalize(q_WB ⊗ delta_q(omega_B dt)).
```

Both nominal biases remain constant. A finite nonnegative `dt` is accepted. At `dt = 0`,
the function validates its inputs and returns an independently owned, byte-for-byte equal
nominal state without normalizing or otherwise changing the supplied valid quaternion.

## Continuous error and noise dynamics

Let `R = R_WB`, with `f_B` and `omega_B` defined above. In 3-by-3 blocks:

```text
        delta_p  delta_v  delta_theta  delta_b_a  delta_b_g
F = [      0        I          0           0          0     ]
    [      0        0       -R[f_B]x      -R          0     ]
    [      0        0       -[omega_B]x    0         -I     ]
    [      0        0          0           0          0     ]
    [      0        0          0           0          0     ].
```

The continuous noise ordering is

```text
w = [n_a_B, n_g_B, n_wa_B, n_wg_B],
```

and the complete noise-input matrix is

```text
          n_a_B  n_g_B  n_wa_B  n_wg_B
G = [       0      0       0       0   ]
    [      -R      0       0       0   ]
    [       0     -I       0       0   ]
    [       0      0       I       0   ]
    [       0      0       0       I   ].
```

Every unspecified block is exact zero. Independent construction during closeout checked every
block, sign, bias correction, rotation use, shape, and zero region.

## Discretization and covariance

The high-rate first-order discretization is

```text
Phi = I + F dt
Q_d_raw = G Q_c G.T dt
Q_d = 0.5 (Q_d_raw + Q_d_raw.T).
```

The explicit final symmetrization removes only floating-point antisymmetry from a matrix that
is symmetric analytically. At zero duration, `Phi` is exact identity and `Q_d` is exact zero,
without performing the otherwise unnecessary matrix products.

Covariance propagation uses

```text
P_raw = Phi P Phi.T + Q_d
P_next = 0.5 (P_raw + P_raw.T).
```

`Q_c`, `Q_d`, and `P` are required to be finite, symmetric, and positive semidefinite. The
audited validator:

- compares each mirrored pair against its own local magnitude;
- cannot use an unrelated large diagonal to excuse a small local asymmetry;
- rejects every explicit negative diagonal, including the smallest negative subnormal;
- requires a zero diagonal to have an exact zero row and column;
- diagonally normalizes positive scales before eigendecomposition; and
- tolerates only scale-aware eigensolver roundoff near a zero eigenvalue.

Valid diagonal and correlated matrices spanning approximately `1e-24` through `1e24` were
accepted. Material asymmetry and locally indefinite principal blocks remained rejected even
when another diagonal or eigenvalue was `1e24`.

## Validation and ownership contracts

All array boundaries validate exact shapes and finite values. Matrix multiplication and
finite-input overflow execute under warning-suppressing error-state guards and are converted
to stable `ValueError` messages. The nominal state is frozen, slotted, and identity-equal;
all five stored arrays are independently owned, C-contiguous `float64`, read-only, and share
no memory with caller arrays. Injection, nominal propagation, matrix construction,
discretization, covariance propagation, and composed prediction do not mutate inputs.

The state constructor validates every shape before value-domain checks, requires the
repository's strict unit-quaternion tolerance, and does not canonicalize quaternion sign.
Equivalent `q_WB` and `-q_WB` therefore remain distinct stored representations of the same
physical attitude.

## RED/GREEN development record

The original bounded implementation proceeded in four direct phases:

| Phase | RED evidence | GREEN evidence |
| --- | --- | --- |
| Nominal state and injection | 28 failed | 28 passed |
| Nominal IMU propagation | 21 failed, 28 passed | 49 passed |
| Continuous `F` and `G` | 7 failed, 49 passed | 56 passed |
| Discretization, covariance, and composition | 34 failed, 56 passed | 90 passed |

The independent closeout audit then found substantive defects not covered by the original
tests:

1. zero-duration nominal propagation, discretization, and composed prediction were rejected;
2. one global matrix magnitude could hide material asymmetry in a small unrelated block;
3. one global eigenvalue scale could hide a negative diagonal or indefinite small principal
   block;
4. correlated `Q_d` was symmetric only to floating-point tolerance rather than explicitly;
5. the first local symmetry fix used `np.spacing(max_float)`, which emitted an overflow
   warning before the intended covariance-overflow boundary; and
6. symmetrizing before checking a zero-diagonal row could underflow the smallest explicit
   subnormal coupling to zero.

Focused regressions produced seven intended failures with 88 tests deselected. The narrow
corrections made all eight selected adversarial and zero-time cases pass. The full ESKF run
then exposed the `np.spacing` warning through the existing warning-as-error overflow test;
that test passed after replacing the calculation with finite per-entry `eps * local_scale`.
A final focused regression exposed the subnormal zero-diagonal coupling with one intended
failure and two passing parameter cases; moving the zero-row check before symmetrization made
all three pass. The corrected ESKF file passes all 96 tests.

## Independent Jacobian evidence

The closeout probe did not import ESKF test helpers. It independently implemented quaternion
products, rotation-vector exponentials, body-to-world rotation matrices, skew matrices, and
the right-local SO(3) logarithm used to recover post-propagation attitude error.

Thirty-two deterministic fixed-seed cases covered identity and nonidentity attitudes,
nonzero biases and measurements in every axis, small and moderate angular rates, and four
positive time steps. Every case injected positive and negative perturbations in all 15 error
columns and formed a central-difference transition Jacobian. Frobenius discrepancies from
`I + F dt` ranged from `3.946774401124e-06` to `3.091348780205e-03`; the maximum discrepancy
divided by `dt²` was `1.373932791202e+01`.

A separate representative halving study at `dt = 0.04, 0.02, 0.01, 0.005` seconds observed:

```text
Frobenius errors:
9.636024565280e-03
2.409007436684e-03
6.022519240339e-04
1.505629265767e-04

halving ratios:
3.999997849
3.999999569
4.000001446
```

The approximately fourfold reduction under step halving independently corroborates the
expected quadratic discrepancy of the first-order discrete transition.

## Empirical process-noise and covariance evidence

A separate fixed-seed Monte Carlo probe constructed a nonidentity attitude, independently
assembled `G`, and used an anisotropic correlated `Q_c`. It mapped 250,000 independent
standard-normal samples through a Cholesky factor, scaled them by `sqrt(dt)`, and then applied
`G`. The empirical covariance differed from independently calculated `G Q_c G.T dt` by
`4.097838024872e-03` relative Frobenius norm, or approximately `0.410%`.

The position block was exactly zero, the expected correlated cross block was nonzero, and
doubling the interval doubled empirical covariance to floating-point precision. Production
`Q_d` matched the independent analytical covariance and was bit-symmetric.

Twelve additional fixed-seed SPD prediction cases remained finite, symmetric, and
positive-semidefinite to numerical precision.

## Long-run evidence

After 750 predictions at `dt = 0.002` seconds with nonzero specific force, angular rate, and
process noise:

```text
quaternion norm:                 1
minimum covariance eigenvalue:  5.366834415969e-06
maximum covariance asymmetry:   0
```

Every nominal-state field and covariance entry remained finite. This exactly corroborated
the earlier long-run evidence without hardcoding it as a test oracle.

## Final verification

The completed closeout passed:

- 96 focused ESKF tests;
- 385 focused rotation, IMU, dynamics, integration, simulation, run-generation, and ESKF
  tests;
- 1,458 repository tests;
- Ruff lint;
- Ruff format verification over 73 Python files;
- mypy over 19 source files; and
- the aggregate `make check` gate.

There were no failures, errors, skips, xfails, or warnings.

## Next exact milestone

The next bounded milestone is measurement-update design for the existing local-position and
barometric-altitude boundaries. It must make explicit decisions for the measurement models,
innovation and Kalman gain, Joseph-form covariance update, error-state injection, and the
covariance-reset Jacobian. Rejection/gating and consistency testing require later explicit
design decisions. Measurement updates and estimator integration remain outside this
prediction-core milestone.
