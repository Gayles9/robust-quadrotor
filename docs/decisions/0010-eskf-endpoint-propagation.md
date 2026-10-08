# 0010: Sampled-IMU Endpoint Propagation and Calibration Evaluation

- Evidence: [calibration verification record (ZIP)](../../evidence/development-records.zip)
- Extends: ADRs 0004–0009

## Evidence and scope

The original completion campaign reported full-state NEES coverage 88.8367% and
mean 18.358807 (15 degrees of freedom). Development-only noiseless and step-halving
probes exposed a deterministic integration contribution; they did not uniquely
attribute the entire discrepancy. The existing prediction freezes world acceleration
and uses a left-held body angular rate, with $`\Phi=I+F\Delta t`$. Improving covariance alone
cannot remove the resulting deterministic integration error on changing motion.

Endpoint propagation uses both IMU samples at the ends of a completed interval,
rather than holding the first sample constant. It is an explicit option; the
first-order default and its experiment remain reproducible. The endpoint option
uses a discrete sample-noise contract. It does not
reinterpret a continuous spectral density as a sample covariance. Position/altitude
observations, NED/FRD signs, right-local errors, prior requirements and stale rejection
retain their established meanings. There are still 15 physical error degrees of
freedom. Six temporary noise variables retain the shared endpoint's information.

## Nominal discrete map

Let $`h=t_{k+1}-t_k>0`$, $`R=R_{WB}[k]`$, and $`\hat{\mathbf n}[k]`$ be the conditional mean of the
six IMU sample noises at the current epoch. Accelerometer units are m/s²; gyro units
are rad/s. Subtract the current estimated biases and that mean from the old sample;
subtract the estimated biases from the new sample, whose prior noise mean is zero:

```math
\begin{aligned}
\mathbf f_0&=\mathbf f_m[k]-\mathbf b_a-\hat{\mathbf n}_a[k],
&\boldsymbol\omega_0&=\boldsymbol\omega_m[k]-\mathbf b_g-\hat{\mathbf n}_g[k],\\
\mathbf f_1&=\mathbf f_m[k+1]-\mathbf b_a,
&\boldsymbol\omega_1&=\boldsymbol\omega_m[k+1]-\mathbf b_g,\\
\boldsymbol\phi&=\tfrac h2(\boldsymbol\omega_0+\boldsymbol\omega_1),
&E&=\mathrm{Exp}([\boldsymbol\phi]_\times),\quad R_1=RE,\\
\mathbf a_0&=\mathbf g_W+R\mathbf f_0,
&\mathbf a_1&=\mathbf g_W+R_1\mathbf f_1,\\
\mathbf p_1&=\mathbf p+h\mathbf v+\tfrac{h^2}{6}(2\mathbf a_0+\mathbf a_1),
&\mathbf v_1&=\mathbf v+\tfrac h2(\mathbf a_0+\mathbf a_1),\\
\mathbf b_{a,1}&=\mathbf b_a,
&\mathbf b_{g,1}&=\mathbf b_g.
\end{aligned}
```

Quaternion evaluation uses the existing Hamilton exponential and normalization.
The translation integrates linearly interpolated world acceleration. It is exact
for constant or linearly changing world acceleration when attitudes are exact.
The exponential of the averaged rate is second order on smooth rotating trajectories;
noncommuting angular rates leave higher-order coning error. This is not exact
continuous inertial integration. Both endpoints are available at $`t_{k+1}`$; no future
row beyond the output epoch is read. At the first epoch, no prediction is performed.

## Matched discrete Jacobians and shared noise

Independent samples have covariance $`\Sigma`$ (6 by 6, accelerometer then gyro).
Bias endpoint increments $`\mathbf u=[\mathbf u_a^T,\mathbf u_g^T]^T`$ are independent of samples and have covariance
$`hW`$, with $`W`$ a 6 by 6 bias-increment spectral density. Full within-sample and
within-walk correlations are supported; sample/walk and inter-sample independence
are explicit assumptions. The bias at the second endpoint is $`\mathbf b+\mathbf u`$.

The joint local error is $`\mathbf z=[\mathbf e_x^T,(\mathbf n[k]-\hat{\mathbf n}[k])^T]^T`$, dimension 21. Its covariance is
$`C`$, initially $`\mathrm{diag}(P_0,\Sigma)`$ because the known prior is independent of IMU noise.
New drivers are $`\mathbf r=[\mathbf n[k+1]^T,\mathbf u^T]^T`$ (12 components) with $`D=\mathrm{diag}(\Sigma,hW)`$.
Define derivative matrices $`A=\frac{\partial\mathbf z_1}{\partial\mathbf z}`$ and $`B=\frac{\partial\mathbf z_1}{\partial\mathbf r}`$. They are the derivatives of
the actual discrete nominal map, not an unrelated continuous approximation.
The following differential equations specify every block (all vectors are length 3):

```math
\begin{aligned}
\delta\mathbf f_0&=-\delta\mathbf b_a-\delta\mathbf n_{a,0},\\
\delta\mathbf f_1&=-\delta\mathbf b_a-\delta\mathbf n_{a,1}-\delta\mathbf u_a,\\
\delta\boldsymbol\phi&=-h\delta\mathbf b_g-\tfrac h2(\delta\mathbf n_{g,0}+\delta\mathbf n_{g,1}+\delta\mathbf u_g),\\
\delta\boldsymbol\theta_1&=E^T\delta\boldsymbol\theta_0+J_r(\boldsymbol\phi)\delta\boldsymbol\phi,\\
\delta\mathbf a_0&=-R[\mathbf f_0]_\times\delta\boldsymbol\theta_0+R\delta\mathbf f_0,\\
\delta\mathbf a_1&=-R_1[\mathbf f_1]_\times\delta\boldsymbol\theta_1+R_1\delta\mathbf f_1,\\
\delta\mathbf p_1&=\delta\mathbf p_0+h\delta\mathbf v_0+\tfrac{h^2}{6}(2\delta\mathbf a_0+\delta\mathbf a_1),\\
\delta\mathbf v_1&=\delta\mathbf v_0+\tfrac h2(\delta\mathbf a_0+\delta\mathbf a_1),\\
\delta\mathbf b_1&=\delta\mathbf b_0+\delta\mathbf u,\qquad
\delta\mathbf n_1=\delta\mathbf n_{\mathrm{new}},\\
C_1&=ACA^T+BDB^T.
\end{aligned}
```

$`J_r`$ is the SO(3) right Jacobian, already implemented by the attitude block of
`eskf_reset_jacobian`. These rotation derivatives follow Solà, *Quaternion kinematics
for the error-state Kalman filter* (2017), sections 4.3.3–4.3.5,
<https://arxiv.org/abs/1711.02508>. The sample-memory derivation here follows directly
from differentiating the stated discrete map. Bias increments affect the endpoint
measurement subtraction as well as final bias covariance; their signs are tested.

Adjacent intervals share $`\mathbf n[k]`$. Dropping its state cross-covariance or treating
averaged sample errors as independent understates accumulated variance. For example,
with one-dimensional acceleration noise of variance $`\sigma^2`$, zero prior, fixed
attitude and zero bias noise, N trapezoidal velocity increments have variance
$`h^2\sigma^2(N-\tfrac12)`$ for N>=1, not $`Nh^2\sigma^2/2`$.

## Correction and reset

Each ordinary observation has joint Jacobian $`H_j=[H,\ 0]`$. Compute the full 21-row
gain using the existing scaled Cholesky solve, condition the six noise means, inject
the physical 15-vector correction and form the full Joseph covariance. Reset it with
$`\mathrm{diag}(J_{\mathrm{reset}},I_6)`$, including all physical/noise cross terms. Return the physical
15-by-15 marginal in the established replay result and measurement diagnostics.
The temporary noise memory is retained internally until the next prediction.
Stale, pending, disabled and rejected observations do not condition any joint state.
The next sensor at the same epoch uses the actual posterior, including noise memory.

All arrays are independently owned, finite and read-only. Covariances use existing
local scale-aware PSD validation. Invalid inputs, nonpositive intervals, overflow or
singular innovations fail explicitly and atomically. No jitter, covariance clipping
or noise inflation is introduced. The endpoint option requires zero in the unused
legacy continuous covariance field to reject ambiguous noise declarations.

## Verification

1. Regression checks preserve the original first-order campaign and its provenance.
2. Analytic equilibrium, constant-rate rotation, constant/linear world acceleration
   and smooth-motion step refinement: demonstrate second-order global convergence.
3. Central differences for all 21 prior and 12 driver columns, at nontrivial attitude,
   biases, endpoint force/rate and conditional sample mean. Compare actual nonlinear
   perturbations and right-local output errors against $`A`$ and $`B`$.
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

## Endpoint evaluation protocol (version 2)

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
The evidence records source/protocol hashes, commands, environment versions,
failures and all seed results.

Accept the correction as an improvement only if mathematical/compatibility tests pass,
all 100 nominal and 20 gated trials remain finite and below existing divergence limits,
existing accuracy/bias/fault/recovery targets hold, and nominal position RMSE does not
degrade by more than 5% relative to the paired first-order mean. Report 15-state NEES,
both NIS distributions and pointwise independent-seed mean bands. Retain the original
90–98% descriptive consistency investigation range and the expected NEES mean of 15;
do not turn correlated epochs into extra independent samples. A miss stays visible.
Passing this finite protocol establishes evidence only for its distribution, not
universal calibration, global observability or flight readiness.
