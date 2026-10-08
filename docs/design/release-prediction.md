# First prediction after support removal

Removing physical support changes the forces on the vehicle abruptly. The
accelerometer sample taken while supported therefore cannot represent the
force acting just after release. This experiment-only correction treats that
first prediction interval explicitly, removing a 1.22625 cm/s ballistic velocity
bias in all 17 saved release configurations.

The correction passes its equation, derivative, sample-ownership and replay
checks. Its first-order uncertainty model does not justify an exact
zero-velocity prior, which would declare the initial velocity known without
uncertainty. It misses nonlinear uncertainty in three directions that its
covariance reports as exact constraints. The [nonlinear model](../results/nonlinear-release.md)
handles those terms; the [combined-prior study](../results/combined-supported-prior.md)
evaluates their flight effect.

The [release specification](../decisions/release-aware-prediction.md) and
[verification evidence (ZIP archive)](../../evidence/development-records.zip) distinguish
model correctness from flight performance. This study reuses saved release
configurations and does not run new flights. Its input campaign's 10.18 cm hover
peak exceeds the 8 cm requirement. The cascade controller remains the default.

## Causal first interval

The fresh sample at release measures the supported left limit: the condition
immediately before support disappears. The first
post-release sample arrives $`h=0.0025\;\mathrm s`$ later. Support disappears between them,
so averaging their accelerometer values treats a force discontinuity as a ramp.
No additional simultaneous sample is available or invented.

Correct the gyro endpoints for their estimated bias and the old sample's
conditional noise mean. Preserve the existing orientation rule:

```math
\phi=\frac h2(\omega_0+\omega_1),\quad
R_1=R_0\mathrm{Exp}(\phi),\quad a_1=g_W+R_1 f_1.
```

Use the first post-release specific force for the open interval:

```math
v_1=v_0+h a_1,\qquad p_1=p_0+h v_0+\tfrac12h^2 a_1.
```

Neither true force nor true velocity enters this map. The supported accelerometer
sample is validated but supplies no open-interval force. Its input correlations
are retained in the full joint covariance; the old gyro still contributes.

For constant world acceleration, including ballistic release, the new rule is
exact. For differentiable acceleration with $`\lVert da/dt\rVert\leq L`$ and exact attitude,

```math
\|e_v\|\le Lh^2/2,\qquad \|e_p\|\le Lh^3/3.
```

These follow by integrating $`a(h)-a(t)`$, with weight $`h-t`$ for position. Linear
acceleration attains both bounds; halving h reduces those errors by four and
eight. This is a first-order force rule if repeatedly applied. It is used once;
all subsequent intervals use the existing second-order endpoint rule. Gyro
integration and finite sensor/bias uncertainty are separate approximations.
A constant-rate rotating-force case independently verifies orientation and the
force bounds. No uniform L bound for the entire flight campaign is claimed.

## Full sample-noise covariance

The error vector is [position, velocity, right-local attitude, accelerometer
bias, gyro bias, current sample noise], with 21 entries. The independent inputs
are the next six sample errors and six endpoint bias increments. Let T be the
existing orientation derivative over all 33 latent inputs. Then

```math
\begin{aligned}
D_a&=-R_1[f_1]_\times T-R_1(S_{b_a}+S_{n_{a1}}+S_{\Delta b_a}), \\
D_v&=S_v+hD_a, \\
D_p&=S_p+hS_v+\tfrac12h^2D_a.
\end{aligned}
```

Attitude, bias and fresh-noise rows retain their existing definitions. Split
these rows into $`A\in\mathbb R^{21\times21}`$ and $`B\in\mathbb R^{21\times12}`$ and propagate

```math
C_1=AC_0A^T+B\mathrm{diag}(R_{IMU},hQ_b)B^T.
```

Independent finite perturbations check every column and the full covariance.
A batch latent-Gaussian calculation also checks the first interval followed by
ordinary intervals and position corrections. In particular the first new
accelerometer noise appears once in release velocity and again, with the
correct correlation, in the next interval. Discarding that memory is incorrect.

`ReleasePrediction` installs the map explicitly for one online experiment or
one replay, validates the supported sample, interval and noise profile, then
routes later predictions unchanged. It restores the original functions on
normal and exceptional exits and cannot be reinstalled. It does not condition
velocity or authorize arming. The caller still owns fixture authentication and
unique acquisition/session use; no process-local object proves hardware support.

## Why the first-order model cannot justify exact zero velocity

In the mean-consistent ballistic case, the conditioned first-order covariance
has rank 18. Write R for its nominal orientation, $`n_a`$ for new sample error and
$`b_a`$ for terminal bias error. Its linear error relation is

```math
r=\delta v_1+hR(\delta b_a+\delta n_a)=0.
```

The actual nonlinear velocity contains the uncertain rotation as well. Even a
simple independent Gaussian yaw psi and force error z gives

```math
\begin{aligned}
v_x&=-hz\cos\psi, \\
v_y&=-hz\sin\psi, \\
r_x&=hz(1-\cos\psi), \\
r_y&=-hz\sin\psi.
\end{aligned}
```

For yaw variance $`s^2`$ and force variance $`\sigma_z^2`$,

```math
\mathrm{Var}(r_y)=h^2\sigma_z^2(1-e^{-2s^2})/2,
```

```math
\mathrm{Var}(r_x)=h^2\sigma_z^2
(3/2-2e^{-s^2/2}+e^{-2s^2}/2).
```

Both are positive although first-order propagation reports zero. At 3 degrees
heading sigma, 0.035 m/s² illustrative force sigma and $`h=0.0025\;\mathrm s`$, the omitted
standard deviations are 0.208 and 4.575 micrometres/s. Fixed positive quadrature
agrees with the exact variance within 4.34e-25 (m/s)². This analytic counterexample
is independent of the campaign samples and is not a fitted covariance adjustment.

The 17 conditional Gaussian studies each retain 5,000 paired draws under the
original and exact-zero-velocity priors. The latter give 0.606–6.744 micrometres/s
standard deviation in the three supposedly exact relations. Their marginal
velocity variance ratios look reasonable (0.9507–1.0365), but marginal checks
cannot establish a correct full joint covariance. The paired linear propagation
satisfies the relation to 1.58e-19 m/s. No pseudoinverse removes the missing
uncertainty, and no covariance floor is inserted.

This is a finite nonlinear pushforward of the declared local Gaussian input,
not a new physical alignment population or flight validation. The small omitted
terms do not establish large flight error. They establish that an exact singular
Gaussian constraint is not justified by this first-order calculation. The old
broad prior remains rank 21; that is also not proof of full calibration.

## Reproduce and interpret the result

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.release_prediction_validation --campaign results/independent-supported-start --output results/release-prediction
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.release_prediction_validation --campaign results/independent-supported-start --verify results/release-prediction
uv run pytest -q tests/unit/test_release_prediction.py
```

The input is the authenticated 34-flight independent supported-start campaign.
Both commands exit zero when execution/verification succeeds; the explicit
report decision still rejects the exact velocity prior. Generated evidence includes all 170,000
nonlinear outputs and their paired linear outputs, full covariances, the old
screens and an independent NumPy-only verifier.

The [nonlinear joint model](../results/nonlinear-release.md) retains the missing
rotation/bias/noise products and passes the full uncertainty checks. Its isolated
flight comparison with the broad velocity prior does not fix the failed hover
requirement. The [combined-prior comparison](../results/combined-supported-prior.md)
passes its tested absolute limits but fails the separate no-regression criterion.
Fresh validation remains necessary before ordinary mission integration. These
results concern startup estimation; they do not establish mass compensation or
general geometric-controller performance.
