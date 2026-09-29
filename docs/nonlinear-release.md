# Nonlinear joint release uncertainty

The experiment-only first-release predictor now includes the nonlinear
rotation/bias/sample-noise terms that first-order propagation omitted. Its full
joint uncertainty passes the fixed mathematical checks under both the original
and exact supported-velocity priors. Mathematical acceptance does not establish
better flight performance or authorize using the exact prior in a mission.

The [frozen protocol](decisions/0036-nonlinear-release-flight-comparison.md)
separates that prerequisite from 25 isolated flight comparisons. Only the original
velocity prior enters these flights. Controller settings, support conditions,
sensor draws, subsequent estimator intervals and acceptance thresholds remain
unchanged. The [verification record](progress/2026-09-29-nonlinear-release.md)
reports implementation, measured benefit and adoption separately.

This isolates the release-prediction package from velocity conditioning. It does
not separately identify the flight effects of the one-sided force rule, nonlinear
mean adjustment and nonlinear covariance; they form one frozen candidate.

## Derivation

Use the [one-sided release map](release-prediction.md): the first post-release
force represents the open interval, while the old and new gyros determine its
orientation increment. The supported accelerometer value does not cross the
force discontinuity. Let the 33 Gaussian latents be the old joint state/sample
error, six fresh sample errors and six bias increments:

\[
X\sim\mathcal N(0,\Sigma),\qquad
\Sigma=\operatorname{diag}(C_0,R_{IMU},hQ_b).
\]

The nonlinear dependence is limited to six rotation variables. Define

\[
\eta=T X=[\delta\theta_0,\delta\phi],\qquad
\delta\phi=-h\delta b_g-\frac h2(n_{g0}+n_{g1}+\Delta b_g).
\]

Define the 18 affine variables
\(Z=U X=[\delta p_0,\delta v_0,\delta b_1,n_1]\).
The terminal accelerometer bias plus fresh accelerometer noise is a selector
\(E Z\). All old cross-correlations remain in \(\Sigma\). Gaussian conditioning
gives

\[
V=T\Sigma T^T,\quad K=U\Sigma T^T V^{-1},\quad
D=U\Sigma U^T-KV K^T,\qquad Z\mid\eta\sim\mathcal N(K\eta,D).
\]

The implementation uses scaled Cholesky solves rather than an inverse. It removes
only exactly deterministic zero rows. It neither deletes small positive
variances nor adds a covariance floor. Unsupported nonstructural singularity is
rejected explicitly.

At a quadrature node,
\(R_i=R_0\operatorname{Exp}(\delta\theta_0)
\operatorname{Exp}(\phi+\delta\phi)\).
For the non-attitude output, let F preserve the affine variables and add
\(h\delta v_0\) to position. Let G have position block \(h^2I/2\), velocity
block \(hI\), and zeros elsewhere. Relative to the deterministic nominal,

\[
Y_i=FZ+G[R_i(f_1-EZ)-R_{nom}f_1]
     =b_i+M_iZ,\quad M_i=F-GR_iE.
\]

Consequently the conditional mean is \(m_i=b_i+M_iK\eta_i\), and its
conditional covariance is \(M_iDM_i^T\). The total moments are

\[
\bar Y=\sum_iw_i m_i,\qquad
C_{YY}=\sum_iw_i\{M_iDM_i^T+(m_i-\bar Y)(m_i-\bar Y)^T\}.
\]

Compute the intrinsic quaternion mean, express each node's right-local attitude
error there, and retain its covariance and cross-covariance with Y. This yields
the complete 21-dimensional output, including the fresh sample's correlation
with navigation. The bias and fresh-noise means retain their analytically exact
values. The p/v means include the nonlinear correction; this is not merely a
covariance substitution.

The positive order-five tensor Gauss-Hermite rule has at most 15,625 nodes.
Order seven supplies a separately evaluated accuracy reference. A direct sum of
conditional covariances checks the implementation's optimized algebra.
Quaternion-sign invariance, analytic yaw/noise variance, zero-noise/linear limits,
full rank, ownership, first-only routing and online/replay equality are tested.

## Mathematical evidence

All 17 authenticated release configurations use their causal time-zero observation
posterior and saved first measured endpoint pair. Each has 20,000 fixed fresh
Gaussian draws shared by the two prior variants: 340,000 outcomes per prior and
680,000 total. The entire joint covariance is whitened by its Cholesky factor;
there is no pseudoinverse hiding a missing direction.

| Fixed check | Original prior | Exact supported-velocity prior | Limit |
| --- | --- | --- | --- |
| Joint output rank | 21/21 | 21/21 | 21 |
| Whitened covariance eigenvalues, all cases | 0.92801–1.07211 | 0.92819–1.07529 | 0.90–1.10 |
| Maximum absolute whitened mean | 0.01925 | 0.02037 | 0.05 |
| Maximum measured order-five/seven covariance discrepancy | 2.02e-12 | 1.07e-8 | 0.001 |
| Maximum ballistic order-five/seven covariance discrepancy | 1.41e-12 | 1.20e-8 | 0.001 |

Both prior variants pass. These are conditional Gaussian pushforwards, not fresh
physical alignment populations, exact Bayesian posteriors or hardware evidence.
The one-sided force approximation, ordinary later ESKF approximations and real
support availability remain separate limitations.

## Isolation and reproduction

`nonlinear_release_prediction` opts into exactly one first prediction for an
online experiment or saved replay. It preserves original initial velocity
uncertainty and restores the original functions on exit. Independent worker
processes isolate the process-local routing; threaded use is unsupported.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.nonlinear_release_validation --campaign results/independent-supported-start --output results/nonlinear-release-uncertainty
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.nonlinear_release_validation --campaign results/independent-supported-start --verify results/nonlinear-release-uncertainty
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.nonlinear_release_campaign --campaign results/independent-supported-start --uncertainty results/nonlinear-release-uncertainty --output results/nonlinear-release-flights --workers 2
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.nonlinear_release_campaign --campaign results/independent-supported-start --uncertainty results/nonlinear-release-uncertainty --verify results/nonlinear-release-flights --workers 2
```

The mathematical command returns success only when its original-prior gate
passes. The flight command returns failure for a completed but rejected
comparison; that is different from an execution or evidence exception. Saved
verification reconstructs results without rerunning a scientific flight.
Evidence is written through durable no-clobber staging and retained outside Git.

The initializer and correction remain experimental. The subsequent
[combined-prior study](combined-supported-prior.md) uses these boundary-only
results as controls under its separately frozen scope. Fresh validation must
precede normal integration. Cascade remains the default; geometric qualification
and mass compensation are separate unresolved work.

The completed 25-flight comparison passes six of nine clean no-regression
comparisons and all eight fault-response comparisons. Only two of three hover
flights meet the unchanged 8 cm limit. RMSE gains are 0.43–0.91 mm across the
nine clean cases, while each hover peak rises slightly. Overall acceptance fails;
the corrected uncertainty is not itself a successful hover-performance fix.
