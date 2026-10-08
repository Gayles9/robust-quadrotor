# ADR 0035: one-sided prediction after supported release

Status: frozen before implementation and numerical evaluation, 2026-09-29.

## Audit and scope

Audit PR 33 merge `7d47303feeb9544cbf7fc750576a9c8016aeba12`, tree
`aadf749ed190cb8ec61d8ad3ddd160b50b1195bd`. Restore this exact published tree
after finding an older local snapshot. Fresh warning-strict endpoint, supported
start and velocity-prior checks pass 94 tests in 4.75 s. The independent evidence
verifier authenticates all 65 preceding payloads and reproduces all 17 rejections.
Authenticate and rescore the original 34-flight campaign. No preceding defect
other than the already documented release discretization is demonstrated.

Implement one experiment-only first-interval map. Preserve the supported sample,
existing post-release IMU, gyro rule, sample ownership, controller, Q/R and all
subsequent prediction equations. No stationarity assumption applies after release.
No truth, motor command or inferred simultaneous right-limit measurement enters
the prediction. This step completes the mathematical/component gate and reconsiders
the exact velocity prior; it does not run a new closed-loop scientific campaign.

## Discrete map and derivatives

At the first post-release sample, h=0.0025 s, retain the existing corrected gyro
trapezoid: phi=h*(w0+w1)/2; R1=R0*Exp(phi). Use only the right accelerometer
endpoint for the open flight interval: a1=g_W+R1*f1. Set

```text
v1 = v0 + h*a1
p1 = p0 + h*v0 + h^2*a1/2.
```

The old accelerometer sample is not an open-interval force. Its conditional
covariance is still allowed to correlate with other input states; propagate the
complete joint distribution without deleting cross terms. Retain the old gyro
conditional mean and new sample memory exactly once. Let T be the existing
right-local attitude derivative across the 33 input columns (physical 15, old
noise 6, new noise 6, endpoint bias increment 6). Then

```text
D = -R1*[f1]_x*T - R1*(selector_ba + selector_new_accel + selector_walk_accel)
dv1 = dv0 + h*D
dp1 = dp0 + h*dv0 + h^2*D/2.
```

Keep the attitude, bias and new-noise rows unchanged. Split into A21x21 and
B21x12; C1=A*C0*A^T+B*diag(R_imu,h*Q_bias)*B^T. This is a first-order
Gaussian approximation, not exact nonlinear propagation. No floors or inflation.

For exact attitude and world acceleration with Lipschitz constant L on (0,h],
the force-quadrature bounds are ||e_v||<=L*h^2/2 and ||e_p||<=L*h^3/3.
Constant world acceleration, including ballistic release, is exact. The map is
only first order if repeatedly used; restricting it to the first interval gives
O(h^2) velocity and O(h^3) position startup errors. Gyro/attitude integration error
is separate. A rotating constant-rate manufactured case checks that separation.

## Frozen component and uncertainty gates

- Independently evaluate the nominal map and all 33 Jacobian columns by central
  differences (absolute 5e-10, relative 2e-7). Check covariance via those derivatives
  (absolute 1e-12). Include correlated priors/noise, nonzero conditional means,
  singular zero-velocity initialization, quaternion sign and rejected input.
- Verify exact ballistic and constant-acceleration limits, analytic linear-force
  errors/bounds, and h-halving convergence. A separate batch latent calculation
  must reproduce first-plus-ordinary intervals and intervening observations;
  it must detect loss or double counting of the first new sample.
- A one-shot, explicitly installed experiment adapter must route only the first
  prediction, enforce the frozen interval and fresh supported sample, reject
  incomplete/repeated use, and restore all patched functions on exceptions.
  Ordinary online/replay paths must remain unchanged outside the context.
- Reconstruct all 17 authenticated release configurations. Preserve their original
  and conditioned covariance. Report the old/new ballistic endpoint, full A/B/C,
  rank and the original 99% down-bias screen for both priors.
- Evaluate finite nonlinear uncertainty explicitly. A declared Gaussian prior
  may have a singular tangent covariance, but a zero-variance direction must not
  be presented as an exact constraint when the nonlinear map has nonzero variance
  in it. Derive a yaw/accelerometer-noise counterexample analytically, check it
  with a fixed positive Gauss-Hermite rule, and retain the result even if small.
  For each release and both priors, retain 5,000 nonlinear Gaussian pushforward
  errors using PCG64 SeedSequence([0x52454C35, case_index]); paired modes use the
  same standard-normal draws. Compare with its paired linear propagation. This
  is a conditional local Gaussian study, not fresh alignment or flight validation.
  Report marginal velocity means/variance ratios and residuals in the deterministic
  tangent relation. Do not pseudoinvert away or regularize missing uncertainty.

Accept the discrete-map component if its algebra, ownership and checks pass.
The exact velocity prior remains no-go if a claimed zero-variance relation has
analytically nonzero variance. Report the removed deterministic bias separately
from that uncertainty decision; do not mistake a passed ballistic screen for
full covariance validity. This stopping rule requires no new scientific flight.

## Subsequent flight comparison, frozen before combination

After the uncertainty prerequisite is closed, run boundary-only regression first:
exactly 25 candidate flights against the authenticated saved ADR 0031 baseline,
nine aligned clean jobs (seeds 47001/47002/47003) and both off/on arms of all eight
fault jobs (seed 47004). Do not rerun saved baselines. Keep full startup, original
8 cm hover, 15 cm RMSE/position, 15 cm/s final speed and completion/response
limits. Report every outcome; require all three hover cases to pass, no clean
hover-peak or RMSE regression beyond 1e-12 and every original fault-response
condition. Only a separately frozen subsequent comparison may add the velocity
constraint. Known seeds are regression; fresh validation still precedes integration.

## Closeout

Retain independently verifiable equations, full results, raw Gaussian errors and
source/protocol/input identities outside Git. Run relevant and full software
gates and hosted CI. Publish the accepted component and the actual prior decision,
including rejection. No controller tuning, geometric reopening, mass-compensation
combination, new sensor, covariance floor or changed flight threshold is in scope.
