# Supported velocity prior: correct at release, overconfident after propagation

The exact zero-velocity prior was **rejected with the original release integration**.
Its conditioning algebra and support checks pass, but every one of the 17 saved
release configurations fails the predeclared necessary uncertainty screen. No new
mission flights are run. The ordinary supported-hover result remains 10.18 cm
against the 8 cm limit; the screen establishes no tracking improvement.
A prior is the estimator's starting belief about a state and its uncertainty.
Here, the question is whether known stationary support justifies zero initial
velocity uncertainty.

The [verification record (ZIP)](../../evidence/development-records.zip) retains
the measured results. The [study protocol](../decisions/supported-velocity-prior.md)
specifies the uncertainty screen and requires it to pass before flight evaluation.

## Why condition velocity at all?

The external fixture promises that the vehicle is stationary in the world frame
through the fresh release sample. The previous navigation prior nevertheless
retains a 0.03 m/s initial velocity standard deviation. Its mean is already zero.
For this ideal fixture, using the known zero velocity to condition the prior is
mathematically valid. It does not require an in-flight observation or truth input
to the controller.

The condition must come from a genuine world-stationary support contract. Zero
acceleration alone is insufficient: a platform moving at constant velocity can
have zero acceleration and quiet IMU readings. The experiment therefore requires
an explicit zero-world-velocity assertion bound to the release support identity
and current epoch. The simulation fixture substantiates it with constant pose,
zero velocity/rate/motor speeds and balanced support force. A hardware procedure
would have to establish its own velocity bound and uncertainty independently.

## One-time Gaussian conditioning

The endpoint covariance $`C`$ contains 15 physical errors and six retained IMU
sample-noise errors. Let $`H`$ select its three velocity errors. With zero prior
velocity mean and an exact independent support constraint, conditioning gives

```math
S=HCH^T,\qquad K=CH^TS^{-1},\qquad C^+=(I-KH)C(I-KH)^T.
```

The implementation solves the three-dimensional system. It does not invert
the 21x21 covariance. The accepted profile has no velocity cross terms with
position, alignment, bias or fresh sample noise. The update therefore zeros
only the initial velocity block; every other covariance entry and nominal mean
is unchanged. Joint rank decreases from 21 to 18. Positive semidefinite covariance
is valid; a fabricated positive floor would misrepresent this exact constraint.

The experiment-only conditioner consumes one local attempt, including rejection.
It checks identity, sample sequence, profile, units, frame, support flags and the
fresh 0.5025 s endpoint. Missing, moving, stale, revoked or mismatched support,
nonzero prior velocity, unsupported cross terms, an already singular velocity
block and non-independent fresh sample noise are rejected. The caller still owns
physical authentication and unique acquisition allocation across processes.
The returned candidate is not permission to arm or a normal mission initializer.

## The first interval changes the conclusion

The fresh sample is acquired while supported. Immediately afterward, support
disappears. Position and velocity stay continuous, but acceleration jumps.
The existing endpoint map integrates a linear interpolation between the two
endpoint accelerations:

```math
\begin{aligned}
\hat{\mathbf v}(h)&=\mathbf v(0)+\tfrac h2(\mathbf a(0^-)+\mathbf a(h)),\\
\hat{\mathbf p}(h)&=\mathbf p(0)+h\mathbf v(0)+h^2\left(\frac{\mathbf a(0^-)}3+\frac{\mathbf a(h)}6\right).
\end{aligned}
```

An exact counterexample is a stationary, motor-off, no-drag release into free
fall. Acceleration is zero under support and $`\mathbf g_W`$ throughout the open flight
interval. Thus $`\mathbf a(0^-)=\mathbf0`$, $`\mathbf a(h)=\mathbf g_W`$, while the analytic flight solution is

```math
\begin{aligned}
\mathbf v(h)&=\mathbf v(0)+h\mathbf g_W,\\
\mathbf p(h)&=\mathbf p(0)+h\mathbf v(0)+\tfrac12h^2\mathbf g_W,\\
\hat{\mathbf v}(h)-\mathbf v(h)&=-\tfrac h2\mathbf g_W,\\
\hat{\mathbf p}(h)-\mathbf p(h)&=-\tfrac{h^2}{3}\mathbf g_W.
\end{aligned}
```

At $`h=0.0025\;\mathrm s`$ and $`g=9.81\;\mathrm{m/s^2}`$, the NED-down velocity error is
**-0.0122625 m/s**, and the down position error is **-0.0000204375 m**.
This is an analytic boundary test, not a measured mission score or the exact
motor-ramp error in the archived flights. It isolates the discontinuity using
mean-consistent IMU values and the actual reported release uncertainty.

## Independent uncertainty calculation

Let $`r`$ be the down row of $`R_{WB}`$, $`P_{b_a}`$ the initial accelerometer-bias covariance,
$`C_{a0}`$ the retained fresh accelerometer-noise block, $`R_{a1}`$ the next sample's
noise covariance, and $`Q_{b_a}`$ the bias-walk spectral density. For the accepted
independent velocity profile, the down variance of this boundary map is

```math
\begin{aligned}
\sigma_{v_D}^2={}&C_{v_D,v_D}+h^2rP_{b_a}r^T\\
 &+\tfrac{h^2}{4}r(C_{a0}+R_{a1})r^T+\tfrac{h^3}{4}rQ_{b_a}r^T.
\end{aligned}
```

The first-order down projection of the tilt term vanishes because
$`\mathbf e_D^T R_{WB}[R_{WB}^T\mathbf g_W]_\times=\mathbf0^T`$. The full alignment/bias and sample-noise
covariances are still retained and propagated; the evidence verifier separately
reconstructs all 21x21 entries with specialized analytic derivatives.

| Quantity | Existing prior | Conditioned prior |
| --- | ---: | ---: |
| Initial velocity standard deviation, m/s | 0.03 | 0 |
| Initial joint rank | 21 | 18 |
| First down-velocity standard deviation, m/s | 0.0300001771 | 0.0001030783 |
| First 99% marginal radius, m/s | 0.077275335 | 0.000265512 |
| Absolute deterministic down-velocity error, m/s | 0.0122625 | 0.0122625 |
| Error / reported standard deviation | 0.40875 | 118.9630 |
| Necessary screen | Pass | Fail |

All 17 release configurations have this conclusion. They share the same
uncertainty profile; these are configuration checks, not 17 independent
statistical trials. Passing the broad original prior's screen does not validate
its calibration or justify using it to hide integration error. Failing the narrow
candidate's screen is sufficient to reject this combination of prior and map.

## Interpretation and related results

The investigation meets its acceptance criteria and rejects this candidate.
The 25 conditional regression flights are skipped under the predeclared stop rule.
The result does not establish whether a velocity prior would improve hover after
the release model is corrected. The previous truth-feedback oracle remains a
separate demonstration of available navigation-channel headroom.

The [release-aware first prediction](../design/release-prediction.md) removes
this ballistic error and passes its component checks. Its first-order uncertainty
still omits nonlinear rotation/noise products in three singular covariance
directions. The [nonlinear model](nonlinear-release.md) accounts for those terms,
and the [combined comparison](combined-supported-prior.md) evaluates the velocity
prior with that model. These are different prior/prediction combinations; their
results do not change the failure of the original combination examined here.

## Reproduce

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_velocity_prior --campaign SAVED_INDEPENDENT/campaign --output NEW_SCREEN
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.supported_velocity_prior --campaign SAVED_INDEPENDENT/campaign --verify NEW_SCREEN
```

Both commands exit 1 for this valid candidate rejection. Verification rederives
the saved screen without flights. The independent evidence verifier uses NumPy
only and does not import the experiment or estimator implementation.
