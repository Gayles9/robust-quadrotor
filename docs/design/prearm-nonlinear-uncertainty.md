# Nonlinear uncertainty for supported pre-arm alignment

The uncertainty derivation passes its frozen acceptance. The subsequent
[standalone component](prearm-component.md) now implements its support and
release contract and passes reference checks. Flight integration is separate.
This study improves the reported joint uncertainty; it makes
only a negligible change to the inclination estimate itself.

The [frozen decision](../decisions/0028-nonlinear-prearm-uncertainty.md) records one
candidate and all acceptance criteria before execution. The
[verification record](../archive/records/prearm-nonlinear-uncertainty.md) preserves
the old failure, independent results, provenance and software checks.

## What the first-order model missed

The original design retained attitude/bias cross-covariance correctly to first
order. Its complete nonlinear Gaussian errors nevertheless produced maximum
normalized variance 1.11216 against a 1.10 limit. Reconstructing the original
5,000 errors, covariance matrices, whitened errors and gate statistics gives
exact equality. The failure is not caused by discarding rejected trials.

Let R_0 be the inclination estimate at retained zero heading, u=R_0^T e_D,
eta the accelerometer-mean error, and psi the independent heading error. Define
t=Log(R_0^T R(s-eta,0)). The exact relative rotation is

\[
 R_0^T R(s-\eta,\psi)=\operatorname{Exp}(u\psi)\operatorname{Exp}(t).
\]

The leading terms of its logarithm are

\[
 \delta\theta=t+u\psi+\tfrac12(u\psi\times t)+\cdots,
 \qquad t\simeq-J\eta.
\]

The mixed heading/inclination term therefore has derivative
-0.5 u cross J[:,j] with respect to psi and eta_j. Independent matrix finite
differences confirm its sign and magnitude. Independent latent variables do
not imply that this nonlinear product has zero variance.

Near level, the leading extra transverse variance is

\[
 \frac{\sigma_\psi^2 V_a}{4g^2}.
\]

The first-order transverse variance conditional on terminal accelerometer bias
is (V_a-C_a²/P_aT)/g². At the frozen profile, their ratio is **0.07811**.
The omitted term is small compared with marginal inclination uncertainty but
about 7.8% of the much smaller conditional variance. That explains why a
marginal axis-error check can look good while the complete joint check misses.
This leading-term calculation explains the scale; it is not added as a fitted
diagonal adjustment. The candidate propagates the full nonlinear map.

## Keep the supported-stationary information boundary

The inputs remain the measured IMU window, declared noise/bias/heading priors
and an external supported-stationary assertion. True attitude, true bias, flight
scores and commanded attitude never enter the candidate. The harness alone
uses truth to evaluate errors.

The original 201 samples span 0.5 s at 400 Hz. Motors remain off and support
continues through the fresh release sample at 0.5025 s. All sample timing,
support, motion and data rejection rules remain required. Gravity supplies no
heading observation, and this single orientation supplies no independent
accelerometer-bias calibration. Existing controller and estimator behavior is
unchanged; arbitrary flight is not treated as a stationary interval.

## Exact Gaussian conditioning before the nonlinear map

Retain the window covariance sums from ADR 0027. The joint Gaussian model of
mean accelerometer error eta and terminal accelerometer-bias error b has

\[
 \operatorname{Cov}(\eta)=V_a,\quad
 \operatorname{Cov}(b,\eta)=C_a,\quad
 \operatorname{Cov}(b)=P_{aT}.
\]

All are scalar multiples of identity in this fixed profile. Equivalently,

\[
 b=K\eta+\epsilon,\quad K=C_a V_a^{-1},\quad
 \operatorname{Cov}(\epsilon)=D=P_{aT}-C_aV_a^{-1}C_a^T,
 \quad \epsilon\perp\eta.
\]

This is a Gaussian conditional decomposition, not a new accelerometer-bias
measurement. The bias prior mean stays zero. The total terminal bias covariance
is still K V_a K^T+D=P_aT. Independent explicit random-walk increments reproduce
the same decomposition. The independent terminal gyro-bias covariance remains
Sigma_g/N+W_g A_N exactly as before.

## One positive-weight nonlinear moment calculation

Use four independent standard-normal latent coordinates z. Set

\[
 \eta_i=\sqrt{V_a}\,z_{i,1:3},\qquad
 \psi_i=\sigma_\psi z_{i,4},\qquad
 R_i=R(s-\eta_i,\psi_i),\qquad b_i=K\eta_i.
\]

The inverse gravity-direction map R is the same ZYX inclination construction
as the original design, now evaluated at every latent point. Five-point
Gauss-Hermite quadrature in each dimension gives **625 fixed nodes**. For
physicists' Hermite nodes x_j and weights w_j, the standard-normal rule uses
sqrt(2)x_j and w_j/sqrt(pi). Tensor-product weights are positive and sum to one.
The one-dimensional rule integrates polynomials through degree nine exactly;
independent standard-normal moment tests verify the implementation.

Starting at R_0, compute the rotation mean by

\[
 \mu=\sum_i w_i\operatorname{Log}(\bar R^T R_i),\qquad
 \bar R\leftarrow\bar R\operatorname{Exp}(\mu).
\]

Require ||mu||<=1e-13 rad with at most eight corrections. Failure rejects the
candidate. The solver recomputes all moments at the final rotation mean, so it
does not reuse a covariance expressed around the old mean without transport.
Let e_i=Log(R_bar^T R_i)-mu. The joint covariance is

\[
 P=\sum_i w_i
 \begin{bmatrix}e_i\\b_i\\0\end{bmatrix}
 \begin{bmatrix}e_i\\b_i\\0\end{bmatrix}^T
 +\operatorname{diag}(0_{3\times3},D,P_{gT}).
\]

This representation retains attitude/bias cross terms and adds only the
mathematically required independent residual. Positive weights and positive
conditional covariance make it PSD by construction; the frozen nonzero-noise
profile also passes strict Cholesky checks. No regularization or inflation is
used. With a linearized rotation map, the same node calculation reproduces
every block of the previous first-order covariance.

The rotation-mean correction is a representation change, not observed heading
information. The underlying Euler heading samples still have their original
mean zero and variance (3 degrees)²; independent checks retain both. The largest
rotation-mean shift in the fresh nominal population is 1.737e-7 rad, about
0.000010 degree. No practical pointing improvement is claimed from that shift.

This is a **local Gaussian pushforward at the measured mean**. It does not claim
to be an exact Bayesian posterior conditioned on the nonlinear constraint
||s-eta||=g. The magnitude gate remains a rejection check, and the covariance
is tested empirically under the declared populations. Conditioning on that
constraint or accepting arbitrary correlated priors would require another
derivation, not an undocumented extension of this one.

## Integration accuracy and empirical calibration

A fixed order-seven rule is an accuracy reference at twelve predeclared
orientation/magnitude combinations. It is not a second candidate. Its maximum
covariance difference, whitened by its covariance, is **1.917e-12**, far below
the 0.001 numerical tolerance. Maximum mean difference is 7.09e-18 rad against
1e-8. The integration error at those points is negligible relative to the
empirical calibration margin; this is not a uniform error bound over all poses.

The new populations use independently frozen version-two PCG64 streams, with
the same distributions as ADR 0027. Each contains 5,000 trials, and all trials
are retained, including any rejections. The original 5,000 Gaussian trials
are checked separately as a regression set.

| Check | First-order model | Nonlinear model | Required limit |
| --- | --- | --- | --- |
| Original Gaussian maximum normalized variance | 1.112164 | 1.061394 | <=1.10 |
| Fresh Gaussian maximum normalized variance | 1.142382 | 1.082222 | <=1.10 |
| Fresh Gaussian maximum absolute whitened mean | Recorded in raw paired errors | 0.020453 | <=0.05 |
| Fresh nominal rejection | Same measurement gates | 0/5,000 | <=1% |

Fresh nominal axis-error p99 is 0.186317 degree and maximum is 0.237789 degree.
Its reported local 99% radii span 0.533317–0.700221 degree, within the retained
0.75-degree budget. Gaussian marginal axis coverage is 99.26%; that descriptive
coverage does not replace the nine-dimensional calibration gate.

The independent covariance gate now passes with 8.22% maximum excess variance
against a 10% allowance. It is a finite simulation result, not proof of perfect
calibration or hardware readiness. In particular, the same constant-acceleration
counterexample still passes the IMU gates: external support remains essential.

## Handoff and next component

The nonlinear candidate returns `q_WB`, the full covariance at that mean, the
computed mean shift and convergence diagnostics. Accelerometer-bias mean and
gyro-bias estimate remain as specified in the original contract. During the
supported one-sample hold, add only the new bias-walk variances and preserve
the nonlinear cross-covariance. The fresh independent IMU sample initializes
endpoint sample memory; earlier samples are never replayed as fresh data.

The [standalone component](prearm-component.md) now completes that implementation
step with explicit support provenance, sample/clock ownership, fixed priors,
latched rejection and one-time fresh-sample release. It reproduces all 15,000
archived covariances exactly and passes complete-session handoff checks.

The later [supported-start flight comparison](../results/supported-start-flight.md) now
passes the known-seed hover limit and improves nominal/wind tracking using this
component, with the full original scoring and controller. Mass mismatch and
the old free-flight failures remain separately labelled. The completed
[independent validation](../results/independent-supported-start.md) retains one
hover-limit failure. Later [combined-prior](../results/combined-supported-prior.md)
and [geometric](../results/final-geometric.md) studies are separate comparisons;
a hardware support procedure and broader qualification remain unestablished.

## Reproduce and references

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.prearm_nonlinear_uncertainty --prior-evidence results/prearm-alignment --output results/prearm-nonlinear
uv run pytest -q tests/unit/test_prearm_nonlinear_uncertainty.py
```

The prior path must contain the exact authenticated ADR 0027 report and NPZ.
Use a new output directory. The source fingerprint binds the experiment and
core source; the report also records the frozen protocol and software versions.

Rotation log/exp and right-local conventions follow the project's existing
ESKF and [Sola's quaternion treatment](https://arxiv.org/abs/1711.02508).
The Gaussian quadrature rule is described in
[NIST DLMF section 3.5](https://dlmf.nist.gov/3.5).
The conditional-bias construction, mixed term and joint-moment equations above
are derived specifically for this stationary-window design.
