# 0004: ESKF Error-State Conventions

Measurement correction is specified by
[ADR 0005](0005-eskf-measurement-updates.md).

The first-order equations below remain the default. The explicit sampled-IMU endpoint
alternative and its matched discrete covariance are specified in
[ADR 0010](0010-eskf-endpoint-propagation.md); both use these physical error conventions.

## Context

The error-state Kalman filter (ESKF) tracks a best estimate, called the nominal
state, alongside uncertainty in its small local error. Prediction advances both
using inertial measurement unit (IMU) samples. The repository uses a north-east-down
(NED) world frame `W`, a forward-right-down (FRD) body and sensor frame `B`, and
Hamilton scalar-first quaternions. The quaternion `q_WB` maps body-coordinate vectors
into world coordinates and obeys

```math
\dot q_{WB}=\tfrac12 q_{WB}\otimes[0,\boldsymbol\omega_B].
```

Mixing left- and right-local attitude errors would change Jacobian frames and signs.
The same convention therefore applies to prediction and measurement correction.

## Decision

### Nominal state

The nominal state is

```math
x=(\mathbf p_W,\mathbf v_W,q_{WB},\mathbf b_{a,B},\mathbf b_{g,B}).
```

The entries are position (3 scalars, m), velocity (3, m/s), orientation
(4, unit quaternion), accelerometer bias (3, m/s²), and gyroscope bias
(3, rad/s). Their code fields are `position_W`, `velocity_W`, `q_WB`,
`accelerometer_bias_B`, and `gyroscope_bias_B`, respectively.

The quaternion sign is not canonicalized. State arrays are finite, independently
owned, C-contiguous `float64` values and are exposed read-only. Quaternion norm evaluation
converts finite arithmetic overflow or invalid arithmetic into the existing unit-norm
validation error without leaking a runtime warning. The constructor and prediction
use the rotation utility's squared-norm check with `rtol=1e-12, atol=1e-12`.

### Error state and injection

The 15-state error ordering is

```math
\delta x=[\delta\mathbf p_W^T,\delta\mathbf v_W^T,\delta\boldsymbol\theta_B^T,\delta\mathbf b_{a,B}^T,\delta\mathbf b_{g,B}^T]^T.
```

Each block has three coordinates. In the stored array, the blocks occupy
slices `0:3`, `3:6`, `6:9`, `9:12`, and `12:15`, in that order.

Position, velocity, and bias errors are additive, with bias errors defined as true
minus nominal. Attitude uses a right-multiplicative body-local error:

```math
\begin{aligned}
R_{\mathrm{true},WB}&=R_{\mathrm{nom},WB}\,\mathrm{Exp}([\delta\boldsymbol\theta_B]_\times),\\
q_{\mathrm{true},WB}&=q_{\mathrm{nom},WB}\otimes\delta q(\delta\boldsymbol\theta_B),\\
\delta q(\boldsymbol\phi)&=\begin{bmatrix}
\cos(\lVert\boldsymbol\phi\rVert/2)\\
\dfrac{\sin(\lVert\boldsymbol\phi\rVert/2)}{\lVert\boldsymbol\phi\rVert}\boldsymbol\phi
\end{bmatrix}.
\end{aligned}
```

The implementation uses a Taylor expansion near zero and normalizes at the existing
quaternion normalization boundary. Right multiplication matches the repository's
body-rate kinematics because both the angular rate and the local perturbation are
expressed in `B` for $`\dot q_{WB}=\tfrac12 q_{WB}\otimes[0,\boldsymbol\omega_B]`$.

### IMU and gravity conventions

The measurement and bias signs are

```math
\begin{aligned}
\mathbf f_{m,B}&=\mathbf f_{\mathrm{true},B}+\mathbf b_{a,B}+\mathbf n_{a,B},\\
\boldsymbol\omega_{m,B}&=\boldsymbol\omega_{\mathrm{true},B}+\mathbf b_{g,B}+\mathbf n_{g,B}.
\end{aligned}
```

The subscript $`m`$ denotes a measured value. Here the bias terms are the true sensor biases, and $`\mathbf n_a,\mathbf n_g`$ are measurement noise.

Nominal propagation therefore subtracts both nominal biases. In NED coordinates,
gravity is $`\mathbf g_W=[0,0,g]^T`$; positive world `z` is down. IMU
measurements and corrected acceleration and angular rate are constant over one
prediction interval. Nominal biases remain constant during that interval.

### Continuous error dynamics

Let $`R=R_{WB}`$,
$`\mathbf f=\mathbf f_m-\mathbf b_a`$, and
$`\boldsymbol\omega=\boldsymbol\omega_m-\mathbf b_g`$. With 3-by-3 blocks and
$`[\cdot]_\times`$ denoting the skew-symmetric cross-product matrix, the continuous state matrix is

```math
F=\begin{bmatrix}
0&I&0&0&0\\
0&0&-R[\mathbf f]_\times&-R&0\\
0&0&-[\boldsymbol\omega]_\times&0&-I\\
0&0&0&0&0\\
0&0&0&0&0
\end{bmatrix}.
```

The continuous process-noise ordering is

```math
\mathbf w=[\mathbf n_{a,B}^{T},\mathbf n_{g,B}^{T},\mathbf n_{wa,B}^{T},\mathbf n_{wg,B}^{T}]^{T},
```

where the final two terms drive accelerometer- and gyroscope-bias random walks. The
noise-input matrix is

```math
G=\begin{bmatrix}
0&0&0&0\\
-R&0&0&0\\
0&-I&0&0\\
0&0&I&0\\
0&0&0&I
\end{bmatrix}.
```

All unlisted blocks in `F` and `G` are exactly zero.

### First-order covariance prediction

For the continuous process-noise spectral-density matrix `Q_c` in the fixed noise
ordering, the high-rate baseline discretization is

```math
\begin{aligned}
\Phi&=I+F\Delta t,\\
Q_d^{\mathrm{raw}}&=GQ_cG^T\Delta t,\\
Q_d&=\tfrac12\bigl(Q_d^{\mathrm{raw}}+(Q_d^{\mathrm{raw}})^T\bigr).
\end{aligned}
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

```math
\begin{aligned}
P_{k+1}^{\mathrm{raw}}&=\Phi P_k\Phi^T+Q_d,\\
P_{k+1}&=\tfrac12\bigl(P_{k+1}^{\mathrm{raw}}+(P_{k+1}^{\mathrm{raw}})^T\bigr).
\end{aligned}
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

The first-order predictor is a high-rate baseline; its discrete transition is
not an exact matrix exponential. It assumes constant IMU measurements and biases
over each interval and a constant scalar gravity magnitude.

Measurement correction and covariance reset are described in
[ADR 0005](0005-eskf-measurement-updates.md); bounded measurement replay in
[ADR 0006](0006-eskf-sensor-replay.md); consistency evaluation in
[ADR 0008](0008-eskf-consistency-evaluation.md); and estimator validation in
[ADR 0009](0009-eskf-completion-validation.md).
[Online feedback](0013-estimated-state-mission-feedback.md) composes these
estimator equations with the mission controller.

The model does not include delayed-measurement rewind, asynchronous IMU handling,
exact matrix-exponential or Van Loan discretization, Earth rotation, Coriolis
effects, gravity estimation, or a geodetic model.
