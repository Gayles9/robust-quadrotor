# Prediction across the support-release boundary

Removing support changes force abruptly. The last supported accelerometer
sample describes the instant before release and should not be averaged as if it
represented the newly airborne interval. This first-interval estimator map uses
the available post-release accelerometer sample while preserving gyro integration
and sample correlations.

The correction uses no truth, motor command or inferred simultaneous
right-limit measurement. It applies once; later intervals retain the ordinary
endpoint equations. It assumes no stationarity after release.

## Discrete map

For the first post-release sample, `h=0.0025 s`, gyro integration remains
`phi=h*(w0+w1)/2` and `R1=R0*Exp(phi)`. The right accelerometer endpoint gives
`a1=g_W+R1*f1`, then

```text
v1 = v0 + h*a1
p1 = p0 + h*v0 + h^2*a1/2.
```

The old accelerometer measurement is not an open-interval force. Its conditional
covariance may still correlate with other input states, so the full joint
distribution is retained. The old gyro conditional mean and new sample memory
are each used once.

Let `T` be the right-local attitude derivative over 33 input columns: 15 physical
states, six old-noise entries, six new-noise entries and six endpoint bias
increments. The changed derivative rows are

```text
D = -R1*[f1]_x*T - R1*(selector_ba + selector_new_accel + selector_walk_accel)
dv1 = dv0 + h*D
dp1 = dp0 + h*dv0 + h^2*D/2.
```

Attitude, bias and new-noise rows remain unchanged. Partitioning derivatives
into `A21x21` and `B21x12` gives
`C1=A*C0*A^T+B*diag(R_imu,h*Q_bias)*B^T`. This is first-order Gaussian
propagation, not exact nonlinear uncertainty; it uses no floors or inflation.

## Accuracy and uncertainty limits

With exact attitude and world acceleration having Lipschitz constant `L` on
`(0,h]`, force quadrature satisfies `||e_v||<=L*h^2/2` and
`||e_p||<=L*h^3/3`. Constant world acceleration, including ballistic release,
is exact. Repeated use would be first order; restricting the map to the first
interval gives O(h²) velocity and O(h³) position startup errors. Gyro/attitude
integration error is separate and checked with a rotating constant-rate example.

Independent central differences check all 33 Jacobian columns with absolute
tolerance 5e-10 and relative tolerance 2e-7. Covariance agreement has absolute
tolerance 1e-12. Fixtures include correlated priors/noise, nonzero conditional
means, a singular zero-velocity prior, equivalent quaternion signs and invalid
inputs. Constant acceleration, linear-force error bounds and h-halving verify
the nominal map. An independent batch latent calculation checks the first and
ordinary subsequent intervals with intervening observations and detects omitted
or double-counted sample noise.

A one-shot experiment adapter enforces the first interval and fresh supported
sample, rejects incomplete/repeated use and restores patched functions on
exceptions. Ordinary online and replay paths remain unchanged outside it.

The 17 authenticated release configurations retain their original and exact
velocity-conditioned covariances. The screen reports both old/new ballistic
endpoints, full A/B/C, rank and the original 99% down-bias test for each prior.

A singular tangent covariance can miss nonlinear variability. An analytic
yaw/accelerometer-noise example and fixed positive Gauss-Hermite check expose
that limitation. For each release, 5,000 paired nonlinear Gaussian pushforwards
use PCG64 `SeedSequence([0x52454C35, case_index])`, with the same standard-normal
draws for both priors and a paired linear propagation. Reports retain marginal
velocity means, variance ratios and residuals in the deterministic tangent
relation. Missing uncertainty is neither pseudoinverted away nor regularized.

Passing nominal algebra and sample-ownership checks accepts the discrete map.
An exact velocity prior still fails the uncertainty requirement if a declared
zero-variance relation has analytically nonzero variance. Removing deterministic
release bias and validating covariance are separate results.

## Flight comparison boundary

Once nonlinear uncertainty is resolved, the boundary-only regression uses
25 candidate flights against saved controls: nine aligned clean jobs at seeds
47001/47002/47003 and off/on arms of eight faults at seed 47004. It retains full
startup, 8 cm hover, 15 cm RMSE/position, 15 cm/s final speed and original
completion/response limits. All three hovers must pass, clean hover peak and
RMSE cannot regress by more than 1e-12, and every fault-response condition must
pass. Adding exact velocity conditioning is a separate comparison.

[Nonlinear release uncertainty](nonlinear-release-flight-comparison.md) supplies
the missing moment calculation. Known-seed regression does not substitute for
fresh validation or ordinary mission integration.
