# 0008: Aligned ESKF Consistency Evaluation and Frozen Nominal Ensemble

- Depends on: [error convention](0004-eskf-error-state-conventions.md),
  [correction](0005-eskf-measurement-updates.md),
  [replay](0006-eskf-sensor-replay.md), [NIS](0007-eskf-innovation-gating.md)
- Evidence: [verification record (ZIP)](../../evidence/development-records.zip)

## Responsibility and isolation

Consistency evaluation asks whether reported uncertainty agrees with actual
estimation error across repeated simulated trials. It does not change
prediction, correction, gating, sensor generation, random streams, or persisted artifacts.
`eskf_consistency.py` reads truth only to construct an evaluation reference, extract
physical estimation errors and calculate normalized estimation error squared (NEES).
`consistency_statistics.py` calculates central chi-square reference intervals and
pointwise ensemble summaries. `experiments/eskf_consistency.py` composes those boundaries
with existing run generation and replay for a fixed nominal study.

The execution order is generation, measurement-only adaptation, fused replay, paired
dead reckoning, then truth-based evaluation. Neither replay receives the reference
history. The prior comes from declared analytic scenario parameters and an independent
random draw, not from a simulated truth row. This is a controlled known-pose prior
experiment; it is not an estimator initialization algorithm for an unknown flight.

## Epoch and state-error contract

The generated artifact has truth/bias rows $j=0,\ldots,N$ at $t_j=j\Delta t$.
The existing adapter exposes full-rate, paired, zero-delay IMU rows $1,\ldots,N$;
replay row $k$ therefore corresponds to truth row $j=k+1$. No prediction precedes
the first replay epoch $\Delta t$, and the last IMU sample has no following interval.
`eskf_reference_from_run_artifact` selects exactly those completed truth rows.
It requires the canonical fixed clock and resolvable scheduler spacing. Every consumed
position, velocity, quaternion and bias row, including the origin, is validated.

`EskfReferenceHistory` owns explicit increasing epochs and copied `EskfNominalState`
references. `evaluate_eskf_replay` requires exact equality of reference and replay time
arrays. There is no nearest-neighbour join, interpolation or tolerance that can hide
an off-by-one error. The estimate is the final post-update state at that epoch; its
covariance is the corresponding reset covariance after all same-epoch corrections.

Write $\hat{\boldsymbol p}_W,\hat{\boldsymbol v}_W,\hat R_{WB},
\hat{\boldsymbol b}_{a,B},\hat{\boldsymbol b}_{g,B}$ for the nominal estimate and
superscript $\star$ for the evaluation reference. World vectors use NED; body vectors
use FRD; $R_{WB}$ maps body coordinates to world coordinates. The 15-vector is

$$
\boldsymbol e = \begin{bmatrix}
\boldsymbol p_W^\star-\hat{\boldsymbol p}_W\\
\boldsymbol v_W^\star-\hat{\boldsymbol v}_W\\
\operatorname{Log}(\hat R_{WB}^{T}R_{WB}^\star)\\
\boldsymbol b_{a,B}^\star-\hat{\boldsymbol b}_{a,B}\\
\boldsymbol b_{g,B}^\star-\hat{\boldsymbol b}_{g,B}
\end{bmatrix}.
$$

Here `Log` returns a three-dimensional rotation vector, not a matrix logarithm.
The block units are m, m/s, rad, m/s² and rad/s. The attitude error is right-local:
$R_{WB}^\star=\hat R_{WB}\operatorname{Exp}([\boldsymbol e_\theta]_\times)$.
It is expressed in the nominal body's local coordinates, consistent with injection
and the covariance in ADR 0004; it is not an Euler-angle subtraction or world-local error.

For the unit scalar-first relative quaternion
$q_{BB}=\hat q_{WB}^{-1}\otimes q_{WB}^\star=(w,\boldsymbol u)$, choose its sign so
$w\ge0$. With $s=\|\boldsymbol u\|$, the principal rotation vector is

$$
\boldsymbol e_\theta=
\frac{2\operatorname{atan2}(s,w)}{s}\boldsymbol u,
\qquad s>0.
$$

The limiting multiplier is 2 as $s\to0$; the implementation uses that limit below
$10^{-8}$, where the omitted correction is at floating-point roundoff scale. The
relative quaternion is normalized before taking the log. At exactly $w=0$, the first
nonzero vector component is chosen positive. This makes the exact-pi tie deterministic
and invariant to the signs of either input quaternion. The principal angle is at most
$\pi$, with an unavoidable discontinuity there; a large attitude error does not satisfy
the local Gaussian assumptions merely because the log can be computed.

All returned arrays are finite, independent, C-contiguous, read-only float64 copies.
Malformed types, shapes, epochs and nonfinite results fail explicitly. Read-only NumPy
buffers are an ownership contract, not an adversarial security boundary; evaluation
revalidates inputs that callers may have deliberately made writable.

## NEES and its valid covariance domain

For right-local error $\boldsymbol e\in\mathbb R^{15}$ and its covariance
$P\in\mathbb R^{15\times15}$,

$$\epsilon_x=\boldsymbol e^T P^{-1}\boldsymbol e.$$

The dimensionless statistic expresses error relative to predicted uncertainty, including
cross-correlations. `normalized_estimation_error_squared` requires finite, symmetric,
numerically positive-definite $P$. It reuses the core's covariance validation and
diagonally scaled Cholesky factorization. If $D=\operatorname{diag}(\sqrt{P_{ii}})$ and
$D^{-1}PD^{-1}=LL^T$, solve $L\boldsymbol z=D^{-1}\boldsymbol e$ and return
$\|\boldsymbol z\|^2$. A scaled Euclidean norm avoids premature square underflow.
No inverse, pseudoinverse, jitter or covariance floor is used.

Singular positive-semidefinite covariance can be valid for prediction, but it does not
define this full 15-degree-of-freedom statistic, even for zero error. Such a case is an
evaluation failure, not NEES zero, a rejected measurement, or a lower-dimensional result.
An unrepresentable statistic also raises `ValueError`.

## Sampled IMU noise and continuous process covariance

The simulator's white-noise standard deviations $\boldsymbol\sigma_a$ [m/s²] and
$\boldsymbol\sigma_g$ [rad/s] describe independent **samples**. They are not the
continuous spectral densities expected by the prediction core. For a full-rate
left-held sample over $\Delta t$, a velocity noise increment is proportional to
$\boldsymbol n_a\Delta t$, giving variance $\sigma_a^2\Delta t^2$ per axis.
The core uses $Q_d=GQ_cG^T\Delta t$. Matching the leading velocity and attitude
increments therefore gives

$$
Q_c=\operatorname{diag}\!\left(
\boldsymbol\sigma_a^{\odot2}\Delta t,
\boldsymbol\sigma_g^{\odot2}\Delta t,
\boldsymbol\eta_a^{\odot2},
\boldsymbol\eta_g^{\odot2}\right),
$$

in noise order $[n_a,n_g,n_{wa},n_{wg}]$. The symbol $\odot2$ means componentwise
squaring. The bias random-walk densities $\boldsymbol\eta_a,\boldsymbol\eta_g$
already have bias-units per square-root second, so they are squared without an
additional $\Delta t$ in $Q_c$. Their discrete bias-increment variances are
$\boldsymbol\eta^{\odot2}\Delta t$.

`sampled_imu_continuous_noise_covariance` is an explicit helper: positive finite interval,
validated IMU parameters, owned diagonal `(12,12)` result, and failure when a positive
variance underflows to zero or a result overflows. It scales standard deviations by
$\sqrt{\Delta t}$ before squaring to avoid premature square overflow/underflow.
The existing nominal replay adapter still requires caller-supplied `continuous_noise_covariance`.

This conversion is not an exact discrete covariance derivation. The nominal position
update includes half the acceleration times $\Delta t^2$, whereas the existing
first-order $GQ_cG^T\Delta t$ has no direct position-noise block or higher-order cross
terms. First-order $\Phi=I+F\Delta t$ and nonlinear attitude/reset approximations also
limit the interpretation of chi-square consistency. Those equations are unchanged.

The generator advances bias random walks once before the first observation at $\Delta t$.
The experiment's prior bias anchor is the configured bias at time zero. Consequently its
initial covariance adds $\eta^2\Delta t$ to each sampled prior-bias variance. Omitting
this term would understate uncertainty at the first estimator epoch.

## Statistical summaries without temporal or selection bias

If a zero-mean Gaussian error has the stated positive-definite covariance, its whitened
components are independent unit normal variables and the squared norm follows
$\chi_d^2$, where $d=15$ for NEES, 3 for position NIS and 1 for altitude NIS. The
[NIST distribution reference][nist-distribution] and [critical-value table][nist-table]
provide the distribution and central two-sided interpretation.
These are assumptions for comparison, not conclusions guaranteed by an ESKF.

For $M$ independent seeds at a **common epoch**,

$$
\bar\epsilon_k=\frac1M\sum_{i=1}^M\epsilon_{i,k},\qquad
\frac{\chi^2_{Md,0.025}}{M}\le\bar\epsilon_k\le
\frac{\chi^2_{Md,0.975}}{M}
$$

is the pointwise 95% reference interval under that model. Individual coverage compares
each score to $[\chi^2_{d,0.025},\chi^2_{d,0.975}]$, including equality. Arrays are
organized `(seeds, epochs)`; sample count for the mean interval is the number of seeds,
not the number of time samples. Temporal means, temporal coverage and the fraction of
epoch means inside the band are explicitly descriptive. They are neither independent
replicates nor simultaneous confidence guarantees. Repeated testing at many epochs can
produce out-of-band values even for an ideal model.

`chi_square_central_95_interval` supports integer degrees of freedom 1 through 15,000,
including 1,000 independent 15-state seeds. It is a bounded, dependency-free calculation,
not a general statistical library. With $t=x/2$, integration by parts gives the survival
probabilities

$$
Q_{2r}(x)=e^{-t}\sum_{j=0}^{r-1}\frac{t^j}{j!},\qquad
Q_{2r+1}(x)=\operatorname{erfc}(\sqrt t)+
\sum_{j=0}^{r-1}\frac{e^{-t}t^{j+1/2}}{\Gamma(j+3/2)}.
$$

Positive terms are evaluated using logarithms and `lgamma`, summed with `fsum`, then
bracketed and inverted with 60 bisection iterations. Validation occurs before the bounded
cache, so Python booleans and numerically equal non-integer types cannot bypass it.
Tests use an exact two-degree exponential CDF and NIST values; independent SciPy audit
values cover larger ensembles without adding SciPy to project dependencies.

NIS comes directly from pre-update replay events, including every scored `REJECTED`
event. Accepted-only scores are never used for coverage. Every stream records fused,
rejected, stale, disabled, pending, unscored and total counts. The nominal protocol
enables diagnostics with both thresholds absent. Generic event extraction also supports
gated records. Even when all scores are retained, a gated nonlinear filter can depart
from ideal Gaussian assumptions because earlier selection changes its state history.

## Frozen nominal protocol v1

The protocol and seed partition were specified before running the study. Its canonical
sorted finite JSON and SHA-256 are emitted before execution and included in the report.
Each trial also contains the full existing run manifest, exact prior state/covariance,
explicit $Q_c$, measured-event counts, score histories and physical error metrics.

| Quantity | Frozen value |
| --- | --- |
| Cases | `hover` (scenario ID 0); `translating_yaw` (ID 1) |
| Initial position and attitude | NED origin, identity `q_WB` |
| Hover velocity and rate | Zero |
| Translating/yaw velocity and rate | NED `[0.5,-0.25,0.1]` m/s; FRD `[0,0,0.4]` rad/s |
| Mass and inertia | 1 kg; diagonal `[0.01,0.01,0.02]` kg m² |
| Rotor positions | FRD `[[.1,.1,0],[.1,-.1,0],[-.1,-.1,0],[-.1,.1,0]]` m |
| Rotor parameters | Spins `[1,-1,1,-1]`, `kf=1e-5`, `km=1e-7`, equal ideal speeds `sqrt(9.81/(4e-5))` rad/s |
| Environment | Gravity 9.81 m/s²; zero wind/drag; matched truth/nominal parameters |
| Numerical method | Projected RK4 truth, `dt=0.01` s |
| Samples and delay | IMU every .01 s, position every .05 s, altitude every .1 s; all zero-delay |
| White-noise sample standard deviations | Acceleration .03 m/s²; gyro .002 rad/s; position .15 m; altitude .1 m |
| Bias random-walk densities | Acceleration .001 m/s²/√s; gyro .0001 rad/s/√s, each axis |
| Deterministic biases and altitude datum | All biases zero; altitude reference 100 m |
| Independent prior block standard deviations | Position .2 m; velocity .1 m/s; attitude .01 rad; acceleration bias .02 m/s²; gyro bias .002 rad/s, each axis |
| Gate policy | Diagnostics-only; no NIS rejection thresholds |
| Descriptive divergence limits | Position error norm >10 m or attitude error norm >π/2 at any epoch |
| Coverage investigation band | Descriptive individual central95 coverage .90–.98; not a statistical CI gate |

Both cases have analytic deterministic motion: balanced thrust with zero roll/pitch
maintains constant world velocity, and the symmetric inertia/equal opposing rotor moments
permit constant yaw. With rate $\Omega$, $p_W(t)=v_Wt$ and
$q_{WB}(t)=(\cos(\Omega t/2),0,0,\sin(\Omega t/2))$. These formulas initialize the
independent prior anchor at $\Delta t$ and provide independent truth-model tests. They
do not read numerical truth histories. Pure yaw and constant translation are limited
excitation, so this study does not establish general bias observability.

| Partition | Seeds for each case | Completed truth steps | Replay interval |
| --- | --- | --- | --- |
| `smoke` | 0, 1 | 21 | .01 to .21 s |
| `development` | 1000 through 1009 | 201 | .01 to 2.01 s |
| `validation` | 20000 through 20099 | 201 | .01 to 2.01 s |

The independent prior uses `PCG64(SeedSequence([0x45534B46,1,scenario_id,root_seed],
pool_size=4))`, draws exactly 15 standard normals, scales by the declared block standard
deviations, and injects the negative error into the analytic anchor. Its domain tag is
separate from all existing named sensor streams. Sensor streams and their derivation
remain unchanged. The cases reuse sensor root seeds and are reported separately; their
results must not be pooled as independent scenarios. The validation partition is held
out from implementation fixtures and parameter choice, not a held-out trajectory family.

Each seed has paired fused and dead-reckoning replay from the same prior and measured IMU.
The latter disables both observation fusion flags. For each physical block $b$, the
reported RMSE is $\sqrt{N^{-1}\sum_k\|\boldsymbol e_{b,k}\|^2}$, not a mixed-unit
15-vector norm. Final bias norms and maximum position/attitude norms are also reported.
Per-seed RMSE distributions contain mean, median, linearly interpolated 95th percentile,
and maximum. The report includes all planned seeds. Numerical failures retain seed,
stage and error; any such failure suppresses that case's ensemble summary. Finite
divergent trials remain in the ensemble and are counted. A singular NEES calculation
also counts as an evaluation failure, rather than silently discarding that sample.

The CLI captures actual installed versions, Git HEAD and dirty status, plus a SHA-256
over sorted source/experiment Python paths, `pyproject.toml` and `uv.lock`, with path and
byte-length delimiters. It detects source-byte changes during the run, uses UTC timestamps,
and creates the output exclusively. It never overwrites an existing result. The same
source, locked environment and protocol reproduce the numerical report; the generation
timestamp is intentionally not deterministic. A dirty tree is reported honestly.
The report is an analysis product, not a resumable filter state or an artifact-schema
change. Generated JSON/logs are not committed.

## Limits and revisiting the decision

Confidence-band agreement is evidence for the declared nominal distribution, not
a general proof of consistency, Gaussianity, robustness or observability.
[ADR 0009](0009-eskf-completion-validation.md) covers stronger excitation and
observation faults, including a full-state NEES undercoverage finding.
[ADR 0010](0010-eskf-endpoint-propagation.md) describes the endpoint propagation
alternative and its separate calibration campaign. Each protocol has its own
seeds and assumptions; their results must be interpreted separately.

[nist-distribution]: https://www.itl.nist.gov/div898/handbook/eda/section3/eda3666.htm
[nist-table]: https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm
