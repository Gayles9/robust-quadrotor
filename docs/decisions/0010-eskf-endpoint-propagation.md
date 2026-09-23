# 0010: Sampled-IMU Endpoint Propagation and Calibration Evaluation

- Date: 2026-09-23
- Baseline: `bf767d913967b7fa1a9ac11c32413658bf94fa1c`
- Status: Implemented and verified; design fixed before implementation and held-out evaluation
- Evidence: [calibration verification record](../progress/2026-09-23-eskf-endpoint-calibration.md)
- Extends: ADRs 0004–0009

## Evidence and scope

The original completion campaign reported full-state NEES coverage 88.8367% and
mean 18.358807 (15 degrees of freedom). Development-only noiseless and step-halving
probes exposed a deterministic integration contribution; they did not uniquely
attribute the entire discrepancy. The existing prediction freezes world acceleration
and uses a left-held body angular rate, with `Phi=I+F*dt`. Improving covariance alone
cannot remove the resulting deterministic integration error on changing motion.

Add an explicit endpoint propagation option for paired instantaneous IMU samples.
Retain the original default and its frozen experiment so historical results remain
reproducible. The new option uses a discrete sample-noise contract. It does not
reinterpret a continuous spectral density as a sample covariance. Position/altitude
observations, NED/FRD signs, right-local errors, prior requirements and stale rejection
retain their established meanings. There are still 15 physical error degrees of
freedom. Six temporary noise variables retain the shared endpoint's information.

## Nominal discrete map

Let `h=t[k+1]-t[k]>0`, `R=R_WB[k]`, and `n_hat[k]` be the conditional mean of the
six IMU sample noises at the current epoch. Accelerometer units are m/s²; gyro units
are rad/s. Subtract the current estimated biases and that mean from the old sample;
subtract the estimated biases from the new sample, whose prior noise mean is zero:

```
f0 = f_m[k]   - b_a - n_hat_a[k]
w0 = w_m[k]   - b_g - n_hat_g[k]
f1 = f_m[k+1] - b_a
w1 = w_m[k+1] - b_g
phi = h/2 * (w0+w1)
E = Exp(skew(phi))
R1 = R E
a0 = g_W + R f0
a1 = g_W + R1 f1
p1 = p + h*v + h²/6 * (2*a0+a1)
v1 = v + h/2 * (a0+a1)
b_a1 = b_a; b_g1 = b_g
```

Quaternion evaluation uses the existing Hamilton exponential and normalization.
The translation integrates linearly interpolated world acceleration. It is exact
for constant or linearly changing world acceleration when attitudes are exact.
The exponential of the averaged rate is second order on smooth rotating trajectories;
noncommuting angular rates leave higher-order coning error. This is not exact
continuous inertial integration. Both endpoints are available at `t[k+1]`; no future
row beyond the output epoch is read. At the first epoch, no prediction is performed.

## Matched discrete Jacobians and shared noise

Independent samples have covariance `Sigma` (6 by 6, accelerometer then gyro).
Bias endpoint increments `u=[u_a,u_g]` are independent of samples and have covariance
`W*h`, with `W` a 6 by 6 bias-increment spectral density. Full within-sample and
within-walk correlations are supported; sample/walk and inter-sample independence
are explicit assumptions. The bias at the second endpoint is `b+u`.

The joint local error is `z=[e_x, n[k]-n_hat[k]]`, dimension 21. Its covariance is
`C`, initially `diag(P0,Sigma)` because the known prior is independent of IMU noise.
New drivers are `r=[n[k+1],u]` (12 components) with `D=diag(Sigma,W*h)`.
Define derivative matrices `A=dz1/dz` and `B=dz1/dr`. They are the derivatives of
the actual discrete nominal map, not an unrelated continuous approximation.
The following differential equations specify every block (all vectors are length 3):

```
df0 = -db_a - dn_a0
df1 = -db_a - dn_a1 - du_a
dphi = -h*db_g - h/2*(dn_g0+dn_g1+du_g)
dtheta1 = E.T*dtheta0 + Jr(phi)*dphi
da0 = -R*skew(f0)*dtheta0 + R*df0
da1 = -R1*skew(f1)*dtheta1 + R1*df1
dp1 = dp0 + h*dv0 + h²/6*(2*da0+da1)
dv1 = dv0 + h/2*(da0+da1)
db1 = db0 + du
dn1 = dn_new
C1 = A*C*A.T + B*D*B.T
```

`Jr` is the SO(3) right Jacobian, already implemented by the attitude block of
`eskf_reset_jacobian`. These rotation derivatives follow Solà, *Quaternion kinematics
for the error-state Kalman filter* (2017), sections 4.3.3–4.3.5,
<https://arxiv.org/abs/1711.02508>. The sample-memory derivation here follows directly
from differentiating the stated discrete map. Bias increments affect the endpoint
measurement subtraction as well as final bias covariance; their signs are tested.

Adjacent intervals share `n[k]`. Dropping its state cross-covariance or treating
averaged sample errors as independent understates accumulated variance. For example,
with one-dimensional acceleration noise of variance `sigma²`, zero prior, fixed
attitude and zero bias noise, N trapezoidal velocity increments have variance
`h²*sigma²*(N-1/2)` for N>=1, not `N*h²*sigma²/2`.

## Correction and reset

Each ordinary observation has joint Jacobian `Hj=[H,0]`. Compute the full 21-row
gain using the existing scaled Cholesky solve, condition the six noise means, inject
the physical 15-vector correction and form the full Joseph covariance. Reset it with
`diag(J_reset,I6)`, including all physical/noise cross terms. Return the physical
15-by-15 marginal in the established replay result and measurement diagnostics.
The temporary noise memory is retained internally until the next prediction.
Stale, pending, disabled and rejected observations do not condition any joint state.
The next sensor at the same epoch uses the actual posterior, including noise memory.

All new arrays are independently owned, finite and read-only. Covariances use existing
local scale-aware PSD validation. Invalid inputs, nonpositive intervals, overflow or
singular innovations fail explicitly and atomically. No jitter, covariance clipping
or noise inflation is introduced. The endpoint option requires zero in the unused
legacy continuous covariance field to reject ambiguous noise declarations.

## Verification plan fixed before implementation

1. Reproduce the baseline suite and retain the original campaign and provenance.
2. Analytic equilibrium, constant-rate rotation, constant/linear world acceleration
   and smooth-motion step refinement: demonstrate second-order global convergence.
3. Central differences for all 21 prior and 12 driver columns, at nontrivial attitude,
   biases, endpoint force/rate and conditional sample mean. Compare actual nonlinear
   perturbations and right-local output errors against `A` and `B`.
4. Exact one-dimensional multi-interval sample-noise variance, full joint conditioning
   against an independent linear-Gaussian calculation, nonzero noise posterior mean,
   attitude cross-covariance reset, correlated PSD inputs and deterministic Monte Carlo.
5. Replay causality, same-epoch order, rejection/stale/pending/disabled behavior,
   variable intervals, ownership and malformed/overflow/singular cases. Preserve all
   original default-path tests and original report compatibility.
6. Development ablations: original prediction, endpoint nominal with old covariance
   (diagnostic only), and fully matched endpoint propagation. No ablation is a
   supported production mode. Compare exact/noiseless and noisy input on paired data;
   report remaining nonlinear/local-Gaussian limitations honestly.

## Fresh frozen protocol

The original version-1 protocol remains unchanged. Version 2 selects endpoint
propagation and adds a paired `first_order` replay of exactly the same measurements
and prior. It uses the original physical distributions, noise standard deviations,
bias walks, observation frequencies, fault plan, gates and targets, without rescaling.

| Partition | Independent seeds | Duration and variants |
| --- | --- | --- |
| Smoke | 20–21 | .4 s; all original eight variants plus first-order comparison |
| Development | 5000–5004 | 30 s; stationary, translating/yaw, excited |
| Held-out validation | 50000–50099 | 30 s; 100 paired nominal/dead/first-order, 20 paired faults, 10 four-way Q/R sensitivity trials: 380 replays |

Development and deterministic probes may guide debugging. The validation partition
is not run until equations, implementation, tests and protocol are fixed. Any change
informed by its results requires a newly declared validation partition. Q sensitivity
scales both sample covariance and bias-increment density; R sensitivity is unchanged.
Record source/protocol hashes, exact commands, versions, failures and all seed results.

Accept the correction as an improvement only if mathematical/compatibility tests pass,
all 100 nominal and 20 gated trials remain finite and below existing divergence limits,
existing accuracy/bias/fault/recovery targets hold, and nominal position RMSE does not
degrade by more than 5% relative to the paired first-order mean. Report 15-state NEES,
both NIS distributions and pointwise independent-seed mean bands. Retain the original
90–98% descriptive consistency investigation range and the expected NEES mean of 15;
do not turn correlated epochs into extra independent samples. A miss stays visible.
Passing this finite protocol establishes evidence only for its distribution, not
universal calibration, global observability or flight readiness.
