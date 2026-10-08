# Nonlinear release uncertainty and flight comparison

First-order covariance can predict zero uncertainty in a direction that has
small but real nonlinear variation. This predictor carries the complete
conditional Gaussian distribution through the first release interval. Its
flight comparison changes only that first prediction and retains the original
velocity prior.

The [recorded comparison](../results/nonlinear-release.md) passed the uncertainty
checks but did not resolve the hover requirement. The predictor remains an
experimental first-interval correction.

## Conditional nonlinear moments

The 33 zero-mean Gaussian latent variables `X` have covariance
`diag(C0,R_imu,h Q_bias)`. Six rotation variables are
`eta=[delta_theta0,delta_phi]`, where

```text
delta_phi = -h*delta_bg - h/2*(old_gyro_noise + new_gyro_noise + gyro_bias_increment).
```

The 18-entry vector `Z` contains
`[delta_p0,delta_v0,terminal_bias_error6,new_sample_noise6]`. Force error `zeta`
is terminal accelerometer bias plus new accelerometer noise, a linear selector
`E Z`. Selectors retain all correlations from the original latent ordering.
Gaussian conditioning gives

```text
Z | eta = K eta + epsilon
K = Cov(Z,eta) Cov(eta)^-1
D = Cov(Z) - K Cov(eta,Z), epsilon independent of eta.
```

A scaled linear solve evaluates conditioning without forming an explicit
inverse. At a rotation node,
`Ri=R0 Exp(delta_theta0) Exp(phi+delta_phi)`. The non-attitude output is affine
in `Z`: `Y=bi+Mi Z`. The matrix `Mi` preserves position/velocity/bias/noise and
subtracts the rotated force-error contribution.

The law of total covariance combines conditional covariance and variability
of conditional means, retaining cross-covariance with right-local attitude.
Positive tensor order-five Gauss-Hermite quadrature uses at most 15,625 nodes.
The intrinsic quaternion mean has tolerance 1e-13 rad and at most eight
corrections. All moments use that final mean; bias and new-noise means retain
their exact unconditional values.

Only structurally deterministic zero rows are integrated out of the latent
covariance. Small positive eigenvalues remain. Unsupported singular conditioning
or a non-positive-semidefinite residual is rejected. There is no covariance
floor, pseudoinverse removal of output error, empirical inflation or fitted
parameter. The theoretical covariance is a positive weighted sum of conditional
covariances and centered outer products.

## Mathematical acceptance

Independent checks cover the analytic omitted yaw/noise variances, zero-noise
and linear limits, correlated moments, equivalent quaternion signs, positive
semidefiniteness, full-rank recovery for the supported exact-velocity profile,
fresh-sample correlations, input rejection and ownership. Explicit conditional
covariance sums check optimized algebra.

Routing remains first-interval only, with exception restoration, causal
same-epoch observations and complete online/replay agreement. Later intervals
use the ordinary predictor.

The original campaign report SHA-256 is
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
All 17 releases use their actual saved first post-release measurement and causal
time-zero observation posterior. Truth is used only for scoring. Both original
and diagnostically conditioned velocity priors are evaluated:

- Order-five and independent order-seven integration must differ by at most
  0.001 in full covariance whitened by the reference, 1e-6 in maximum
  standardized Euclidean mean and 1e-8 rad in attitude mean. The
  mean-consistent ballistic limit is also checked.
- Each measured endpoint pair has 20,000 fresh paired Gaussian pushforwards,
  using PCG64 `SeedSequence([0x52454C36,case_index])`. Common 33-dimensional
  standard-normal draws pair the priors. Physical errors are recentered at the
  nonlinear mean with attitude transport and sample memory included.
- All 21 covariance directions must exist for this nonzero-noise profile.
  Empirical whitened covariance eigenvalues must lie in [0.90,1.10], with
  maximum absolute whitened mean at most 0.05. Every outcome is retained;
  pseudoinverse scoring and resampling are excluded.

These checks validate a declared conditional Gaussian pushforward, not an exact
Bayesian posterior or a physical support sensor. Force-quadrature bounds and
ordinary estimator approximations remain separate limitations. Mathematical
validity of exact velocity conditioning does not demonstrate its flight benefit.

## Isolated flight comparison

Only the nonlinear predictor with the **original velocity prior** enters this
comparison, after its uncertainty checks pass. The exact-prior calculation
remains a separate diagnostic result.

The 25 flights comprise hover, nominal tracking and wind tracking at seeds
47001/47002/47003, plus supervision off/on pairs for eight faults at seed 47004.
They use authenticated saved baselines, unchanged support and IMU data, gyro
integration, subsequent prediction, controller, Q/R, gains and limits. All
planned valid outcomes are retained. Execution uses at most two processes and
no shared-thread patches.

Complete startup is scored against 8 cm hover, 15 cm RMSE/final-position and
15 cm/s final-speed limits, with original completion and fault-response rules.
Every hover must pass; clean hover peaks and whole-flight RMSE cannot regress
by more than 1e-12. Evidence includes scores and signed changes, full histories,
first-prediction traces, estimator replay, command/supervision reconstruction
and named-noise pairing.

Implementation, measured benefit and adoption are separate conclusions. A failed
flight gate leaves the predictor experimental. The
[combined-prior study](combined-supported-velocity-comparison.md) separately
examines adding the supported velocity constraint; neither study alone changes
the default mission API.
