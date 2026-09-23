# 0005: ESKF Measurement Correction and Covariance Reset

- Date: 2026-09-23
- Status: Accepted and implemented
- Extends: [ADR 0004](0004-eskf-error-state-conventions.md)
- Implementation: `src/quadrotor_math/eskf.py`
- Evidence: `tests/unit/test_eskf_update.py` and the
  [verification record](../progress/2026-09-23-eskf-measurement-updates-and-contract-audit.md)

## Context and scope

The prediction core already represents NED position and velocity, a Hamilton scalar-first
body-to-world quaternion, and FRD accelerometer and gyroscope biases. This increment adds
correction from the existing local Cartesian position and positive-up altitude sensors.
It completes the bounded measurement-update milestone identified by the published README
and prediction-core closeout. It does not introduce a sensor scheduler consumer or a full
estimator runner.

The existing nominal state has 16 stored scalars and a unit-quaternion constraint. Its
15-dimensional local error remains

$$
\delta x=(\delta p_W,\delta v_W,\delta\theta_B,\delta b_{a,B},\delta b_{g,B}),
\qquad R_{\mathrm{true},WB}=R_{\mathrm{nominal},WB}\operatorname{Exp}([\delta\theta_B]_\times).
$$

Position and velocity errors are in metres and metres per second; attitude error is in
radians; IMU bias errors are in metres per second squared and radians per second. Covariance
entries have the products of the corresponding state-error units. All rotations and bias
signs retain ADR 0004.

## Measurement models and assumptions

The position model, expressed in the NED world frame, is

$$
h_p(x)=p_W+\bar b_{p,W},\qquad H_p=[I_3\;0\;0\;0\;0]\in\mathbb R^{3\times15}.
$$

The positive-up altitude model is

$$
h_h(x)=h_{\mathrm{ref}}-e_3^T p_W+\bar b_h,
\qquad H_h=[-e_3^T\;0\;0\;0\;0]\in\mathbb R^{1\times15},
\qquad e_3=(0,0,1)^T.
$$

The overbars identify explicit *assumed* constant sensor biases. These parameters and the
reference altitude are supplied by the caller, normally from nominal sensor configuration.
They do not add position-sensor bias states and are never read from truth inside the update.
All predicted and observed values are in metres. A positive altitude innovation produces a
negative NED-down position correction when the vertical position gain is informative.

`measurement_noise_covariance_W` is a discrete per-observation covariance in square metres;
`measurement_noise_variance` is the scalar altitude variance in square metres. Neither is a
standard deviation nor a continuous-time noise density. For independent configured sensor
axes, the caller forms the diagonal matrix from squared nominal standard deviations.
Correlated covariance within a position observation is supported. The observation noise must
be independent of the prior error. Multiple separate update calls additionally require the
caller to account for any correlation between observations; naively applying correlated
position and altitude readings as independent measurements is not justified.

Every observation must refer to the current nominal state's epoch. Acquisition time and
delivery time can differ in the run artifacts. These functions do not infer an epoch, rewind
state, interpolate, reject stale data, or reinterpret a delayed measurement as current.

## Correction and Joseph covariance

For a generic observation dimension `m > 0`, define the prior covariance
$P\in\mathbb R^{15\times15}$, observation $z\in\mathbb R^m$, prediction
$h\in\mathbb R^m$, error Jacobian $H\in\mathbb R^{m\times15}$, and noise covariance
$R\in\mathbb R^{m\times m}$. The local error mean before each call is zero. The update uses

$$
r=z-h,\qquad S=HPH^T+R,\qquad SK^T=HP,\qquad \widehat{\delta x}=Kr.
$$

Here `r` is the innovation, `S` is its predicted covariance, and `K` is the Kalman gain.
The implementation solves for the gain without forming a matrix inverse. Let
$D=\operatorname{diag}(\sqrt{S_{ii}})$ and factor
$D^{-1}SD^{-1}=LL^T$. Two linear solves followed by unscaling recover `K`. Diagonal scaling
reduces sensitivity to disparate measurement variances; it does not cure rank deficiency.

The covariance before coordinate reset uses Joseph form:

$$
A=I_{15}-KH,\qquad P_J=APA^T+KRK^T.
$$

Both terms are positive semidefinite analytically. The implementation removes floating-point
antisymmetry using the existing exact-entry-preserving symmetrizer and validates the result.
This formulation avoids directly subtracting two nearly equal covariance matrices.

`P` and `R` may be positive semidefinite, including zero-noise directions. `S` must be
numerically positive definite under the scaled Cholesky factorization. A singular innovation
covariance raises `ValueError`. There is no diagonal jitter, pseudoinverse, negative-variance
clipping, outlier gate, or silent observation rejection. Correlated and zero-noise inputs are
covered by explicit tests.

## Injection and covariance reset

The existing injection adds the position, velocity, and IMU-bias corrections and right
multiplies the nominal quaternion by the rotation-vector exponential. The posterior local
error is then defined about this updated nominal state, so its covariance must also change
coordinates. This reset operation follows the ESKF construction in Solà (2017), Section 6;
the implementation specializes it to the project's fixed-gravity 15-state convention.

Write $\phi=\widehat{\delta\theta_B}$ and perturb the pre-reset attitude error about that
correction by $\epsilon$. The post-reset rotation error is

$$
g(\epsilon)=\operatorname{Log}\left(\operatorname{Exp}(-[\phi]_\times)
\operatorname{Exp}([\phi+\epsilon]_\times)\right).
$$

The right Jacobian identity
$\operatorname{Exp}(\phi+\epsilon)\simeq\operatorname{Exp}(\phi)
\operatorname{Exp}(J_r(\phi)\epsilon)$ gives $Dg(0)=J_r(\phi)$. Using the closed form
from Solà, Deray, and Atchuthan, equation (143),

$$
J_r(\phi)=I_3-\frac{1-\cos\theta}{\theta^2}[\phi]_\times
+\frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,
\qquad \theta=\|\phi\|.
$$

Thus

$$
\Gamma=\operatorname{diag}(I_6,J_r(\phi),I_6),\qquad P^+=\Gamma P_J\Gamma^T.
$$

All attitude cross-covariances are transformed. IMU bias coordinates remain body/sensor
coordinates, so their reset blocks are identity. The code evaluates the exact local
Jacobian, not merely `I - 0.5*skew(phi)`. For angles below `1e-4` radians it uses Taylor
coefficients through fourth order; elsewhere it uses a unit-axis form and a half-angle sine
to avoid unnecessary powers and subtraction near zero. At zero correction, `Gamma` is exact
identity. This exact derivative still transports covariance only to first order in residual
uncertainty; it does not make the nonlinear posterior exactly Gaussian or validate large
attitude uncertainty.

The error mean is reset implicitly: the returned correction is diagnostic, and the next
call again assumes a zero local error mean. Applying the returned correction a second time
would be an error.

## Public interfaces and ownership

| Identifier | Inputs and output |
| --- | --- |
| `eskf_local_position_measurement_model` | Nominal state and assumed NED bias `(3,)`; returns prediction `(3,)` and `H (3,15)` |
| `eskf_barometric_altitude_measurement_model` | Nominal state, reference altitude, assumed altitude bias; returns prediction `(1,)` and `H (1,15)` |
| `update_eskf_linear_measurement` | Nominal state, `P`, `z`, `h`, `H`, `R`; returns one `EskfMeasurementUpdate` |
| `update_eskf_local_position` | Nominal state, `P`, NED measurement, discrete covariance, assumed bias |
| `update_eskf_barometric_altitude` | Nominal state, `P`, scalar altitude, variance, reference altitude, assumed bias |
| `eskf_reset_jacobian` | Correction `(15,)`; returns `Gamma (15,15)` |
| `reset_eskf_covariance` | Pre-reset covariance and correction; returns transported covariance |
| `EskfMeasurementUpdate` | Owned posterior state, posterior covariance, innovation, innovation covariance, gain, and injected correction |

Array inputs use explicit shapes and real float64-compatible values without implicit
reshaping. The generic update checks all array shapes before value domains. Model wrappers
validate their model parameters before composing that generic boundary. Covariance symmetry
and positive semidefiniteness reuse the locally scaled checks from ADR 0004. Finite arithmetic
overflow is converted to `ValueError`; inputs remain unchanged if validation or arithmetic
fails.

Result arrays are independently owned, C-contiguous float64 and read-only. The result is
frozen, slotted, and identity-equal. Its nominal state is independently copied. Model and
standalone Jacobian/covariance functions return owned arrays, rather than views of caller
storage. Read-only flags protect ordinary API use, not adversarial buffer mutation.

Zero innovation still updates covariance. When the resulting correction is exactly zero,
the nominal state is copied byte for byte without quaternion renormalization. The ESKF state
constructor now uses the downstream rotation utility's squared-norm tolerance
`rtol=1e-12, atol=1e-12`, removing the previous mismatch between norm and squared-norm tests.

## Verification and remaining boundaries

Analytic scalar and diagonal cases verify signs, gains, and covariance reduction. Correlated
updates are compared with independent conditional-Gaussian formulas. All 15 measurement
Jacobian columns are checked by central differences. A Rodrigues/logarithm construction
independently differentiates the reset map at zero, small, and moderate corrections; tests
also verify every affected covariance cross block and quaternion-sign equivalence.

Numerical tests cover malformed inputs, nonfinite data, local asymmetry/indefiniteness,
singular `S`, zero `R`, subnormal variances, diagonal scales `1e-300` through `1e300`, and
finite overflow with warnings treated as errors. Fixed-seed cases exercise both observation
dimensions. Composition tests cover independent same-epoch sequential/joint observations,
stationary and constant-velocity vertical-bias drift correction, and rotating motion with
known constant world acceleration.

These are deterministic core-regression results. They do not establish general bias
observability, calibrated sensor performance, NIS/NEES consistency, outlier robustness,
delayed-measurement correctness, or closed-loop flight performance. This mathematical
increment added no runner, estimator configuration, manifest extension, controller,
ROS 2/PX4 adapter or dependency. The subsequent [ADR 0006](0006-eskf-sensor-replay.md)
implements bounded replay and in-memory configuration, with explicit stale rejection;
it does not change these timestamp-free correction equations. Gate G2 baseline control
remains open.

## References

- Joan Solà, *Quaternion kinematics for the error-state Kalman filter*, 2017,
  [arXiv:1711.02508](https://arxiv.org/abs/1711.02508), Section 6.
- Joan Solà, Jeremie Deray, Dinesh Atchuthan, *A micro Lie theory for state estimation in
  robotics*, 2018, revision 9 (2021),
  [arXiv:1812.01537v9](https://arxiv.org/abs/1812.01537v9), equations (68) and (143).
