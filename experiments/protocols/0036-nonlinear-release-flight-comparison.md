# ADR 0036: nonlinear release uncertainty and isolated flight comparison

Status: frozen before implementation and numerical evaluation, 2026-09-29.

## Audit and scope

Audit PR 34 merge `85f0d78d867122500c9f988c9daef60967aca4ce`, tree
`f85f5fd305e02f63a22f549002e50b5095ba2736`. Local main is clean and agrees with
live GitHub. Recheck the release/prior/nonlinear pre-arm tests and independently
verify the preceding evidence before implementing. ADR 0035 correctly identifies
nonlinear uncertainty missing from the first-order singular covariance.

Implement one experiment-only nonlinear moment predictor for the first release
interval, then execute the already frozen boundary-only flight comparison.
Keep the existing supported acquisition, IMU, gyro integration, subsequent
ordinary intervals, controller, Q/R, gains, seeds and flight limits. Do not add
zero-velocity conditioning to a flight, geometric tuning or mass compensation.

## Conditional nonlinear moments

The 33 zero-mean Gaussian latents X have covariance diag(C0,R_imu,h Q_bias).
Define six rotation variables eta=[delta_theta0,delta_phi], where

```text
delta_phi = -h*delta_bg - h/2*(old_gyro_noise + new_gyro_noise + gyro_bias_increment).
```

Let Z have 18 entries [delta_p0,delta_v0,terminal_bias_error6,new_sample_noise6].
The force error zeta is terminal accelerometer bias plus new accelerometer noise,
so it is a linear selector E Z. Derive these selectors from the original latent
ordering, including all cross terms. Gaussian conditioning gives

```text
Z | eta = K eta + epsilon
K = Cov(Z,eta) Cov(eta)^-1
D = Cov(Z) - K Cov(eta,Z), epsilon independent of eta.
```

Use a scaled solve, not an explicit inverse. At a rotation node,
Ri=R0 Exp(delta_theta0) Exp(phi+delta_phi). The non-attitude output is affine
in Z: Y=bi+Mi Z, where Mi preserves p/v/bias/noise and subtracts the rotated
force-error contribution. Integrate its conditional mean and covariance by the
law of total covariance, retaining cross-covariance with right-local attitude.
Use a positive tensor order-five Gauss-Hermite rule (at most 15,625 nodes).
Compute the intrinsic quaternion mean with tolerance 1e-13 rad and at most eight
corrections. Express all moments at that final mean. Bias and new-noise means
retain their analytically exact unconditional values.

Integrate only structurally deterministic zero rows out of the latent covariance;
never discard a small positive eigenvalue, add a floor, or pseudoinvert away an
output error. Reject unsupported singular conditioning or a non-PSD residual.
The theoretical covariance is a positive weighted sum of conditional covariances
and centered outer products. No empirical inflation or fitted parameter is used.

## Fixed verification and uncertainty acceptance

1. Reproduce the analytic yaw/noise omitted variances from ADR 0035, zero-noise
   and linear limits, correlated conditional moments, quaternion sign invariance,
   PSD, full-rank recovery for the supported exact-velocity profile, and fresh
   sample correlations. Independently compare explicit conditional covariance
   sums to any optimized algebra. Test input rejection and finite ownership.
2. Preserve first-only routing, exception restoration, same-epoch observations,
   complete online/replay equality and unchanged original initial velocity prior.
   Tests must demonstrate that later intervals still use the original map.
3. Authenticate the original ADR 0031 campaign (report SHA-256
   `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`).
   Reconstruct all 17 support/releases/configurations. Use each saved first
   measured post-release endpoint and its causal time-zero observation posterior.
   Truth is for scoring/audit only and never enters the candidate.
4. For both original and diagnostically conditioned velocity priors at every
   configuration, compare order five with an independent order-seven integration
   reference: full covariance discrepancy whitened by the reference <=0.001;
   maximum standardized Euclidean mean discrepancy <=1e-6; attitude mean
   difference <=1e-8 rad. Also check the mean-consistent ballistic limit.
5. For each of the 17 measured endpoint pairs retain 20,000 fresh paired Gaussian
   pushforwards, PCG64 SeedSequence([0x52454C36,case_index]). Common 33-dimensional
   standard-normal draws pair original/exact-velocity priors. Recenter physical
   errors at the nonlinear output mean, including attitude transport and sample
   memory. All 21 covariance directions must exist under this nonzero-noise
   profile: no pseudoinverse scoring. Require empirical whitened covariance
   eigenvalues in [0.90,1.10] and max absolute whitened mean <=0.05. This population
   size makes a 21-dimensional covariance check meaningful without changing the
   previous 1.10 underprediction cap. Retain every outcome without resampling.

This verifies the declared conditional Gaussian pushforward, not an exact Bayesian
posterior or hardware support model. Numerical force-quadrature bounds and ordinary
ESKF approximations remain separately stated. Passing the exact-prior mathematical
check does not authorize its use in a flight or demonstrate its performance benefit.

## Isolated flight campaign and stopping rule

The nonlinear predictor with the **original velocity prior** is the only new
flight candidate. Its original-prior uncertainty checks must pass first; retain
the separate exact-prior diagnostic result whether it passes or fails. Run exactly
25 flights: nine clean aligned jobs (hover, nominal tracking, wind tracking under
seeds 47001/47002/47003) and both off/on supervision arms for eight original fault
jobs under seed 47004. Compare to authenticated saved baselines, never rerun them.
Do not stop the campaign early because a valid metric fails and do not retry a
valid flight. Infrastructure/evidence failures are distinct and must be recorded.

Preserve full startup, original 8 cm hover, 15 cm RMSE/final-position, 15 cm/s
final-speed, completion and fault-response limits. Require all three hover cases
to pass, no clean hover peak or whole-flight RMSE regression beyond 1e-12, and
all original fault-response conditions. Record every baseline/candidate score,
signed changes, complete histories, timing, first-prediction trace, estimator
replay, command/supervision reconstruction and named-noise pairing. Use at most
two worker processes; do not share process-local patches between threads.

If any acceptance condition fails, retain the candidate as experimental, publish
the actual flight outcome and stop. Do not tune after seeing it or automatically
add the velocity constraint. A future combined-prior comparison needs a separate
frozen scope; fresh qualification still precedes promotion to a normal mission.

## Closeout

Pass relevant/full software gates and hosted CI. Preserve mathematical and flight
evidence outside Git with independent integrity/reproduction checks. Report three
separate outcomes: implemented, measured flight benefit, adopted. Keep failed
metrics visible; a completed investigation is not flight qualification.
