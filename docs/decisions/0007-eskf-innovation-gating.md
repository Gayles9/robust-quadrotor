# 0007: Pre-Update ESKF Innovation Diagnostics and Outlier Gating

- Extends: [ADR 0005](0005-eskf-measurement-updates.md) and
  [ADR 0006](0006-eskf-sensor-replay.md)
- Implementation: `eskf_innovation.py`, `eskf_replay.py`, `eskf_run_replay.py`
- Evidence: `test_eskf_innovation.py`, `test_eskf_gating.py`, `test_eskf_run_replay.py`,
  [verification record (ZIP)](../../evidence/development-records.zip)

## Context and scope

A fresh observation can be numerically valid but incompatible with the current estimate
and its assumed uncertainty. The existing correction applies every fresh enabled value.
The replay layer provides a separate pre-correction diagnostic and an optional fixed upper gate
for local position and barometric altitude. A rejected observation must not inject an
error, contract the covariance, or contaminate the prior used for the next observation.

Normalized innovation squared (NIS) measures how surprising an observation is
relative to the predicted measurement uncertainty. Each sensor has an explicit
threshold, and every rejected observation remains in the diagnostic record.
Initialization, IMU hold, observation order and stale policy follow ADR 0006.
The mathematical correction primitives remain ungated; replay decides whether
to call them.

## Observation mathematics

Let $`\hat x^-`$ be the nominal state immediately before one observation, and let
$`P^-\in\mathbb R^{15\times15}`$ be its right-local error covariance in ADR 0004 order.
For measurement $`z\in\mathbb R^m`$, model prediction $`h(\hat x^-)\in\mathbb R^m`$,
error Jacobian $`H\in\mathbb R^{m\times15}`$ and discrete observation-noise covariance
$`R\in\mathbb R^{m\times m}`$, the linearized observation error is

```math
\begin{aligned}
z-h(\hat x^-)&\simeq H\delta x+v, \\
\mathbb E[\delta x]&=0, \\
\mathrm{Cov}(\delta x)&=P^-, \\
\mathbb E[v]&=0, \\
\mathrm{Cov}(v)&=R.
\end{aligned}
```

The model assumes observation noise $`v`$ is independent of prior error $`\delta x`$.
The **innovation** is the measured-minus-predicted residual, and its predicted covariance is

```math
r=z-h(\hat x^-),\qquad S=H P^- H^T+R.
```

$`S`$ represents uncertainty from both the prior and the measurement. Dividing an error only
by a sensor standard deviation would omit the prior uncertainty. `compute_eskf_linear_innovation`
validates $`P^-`$ and $`R`$ as finite symmetric positive semidefinite matrices, then forms $`r`$
and $`S`$ with the same multiplication/symmetrization order as the correction core.

The replay models are exactly those in ADR 0005:

| Observation | Dimension | Model | Nonzero Jacobian block |
| --- | --- | --- | --- |
| Local position | $`m=3`$ | $`h_p=\hat p_W+b^{\mathrm{nom}}_{p,W}`$ | $`H_p[:,0:3]=I_3`$ |
| Barometric altitude | $`m=1`$ | $`h_a=h_{\mathrm{ref}}-\hat p_{W,z}+b^{\mathrm{nom}}_a`$ | $`H_a[0,2]=-1`$ |

$`\hat p_W`$ and local-position residuals use NED coordinates in metres. Altitude is a
positive-up scalar in metres, while $`\hat p_{W,z}`$ is positive down. Sensor biases and
the altitude datum are explicit nominal assumptions; neither is recovered from truth.
For these two models $`r`$ has units m and $`S`$ has units m². The generic function also
supports other positive observation dimensions when the caller supplies consistent units.

## Whitening and NIS without an inverse

The implementation requires $`S`$ to be numerically positive definite. Define the diagonal
matrix $`D=\mathrm{diag}(\sqrt{S_{11}},\ldots,\sqrt{S_{mm}})`$ and factor the
dimensionless correlation-scaled matrix:

```math
C=D^{-1}SD^{-1}=LL^T,\qquad y=L^{-1}D^{-1}r,
\qquad \epsilon_\nu=y^Ty=r^TS^{-1}r.
```

$`L\in\mathbb R^{m\times m}`$ is lower triangular. The **whitened innovation**
$`y\in\mathbb R^m`$ is dimensionless, as is NIS $`\epsilon_\nu\ge0`$. NIS measures
squared residual length relative to its predicted uncertainty, including correlations.
The equalities use inverses as mathematical notation; code divides by diagonal scales
and solves a Cholesky system. It never constructs a matrix inverse.

`EskfInnovation` accepts only $`r`$ and $`S`$, computes $`y`$ and NIS itself, and stores all
three arrays as independent C-contiguous read-only float64 copies. Callers cannot supply
an inconsistent precomputed NIS. `math.hypot` computes the norm before squaring so that
individually underflowing squared components can still yield a representable total.
Normal float64 rounding remains, including zero for a total below representable precision.

Shapes must be exact, arrays real and finite, and the locally scaled PSD/symmetry rules
remain those of the ESKF core. Singular $`S`$, failed factorization, nonfinite arithmetic,
or unrepresentable whitening/NIS raises `ValueError`. There is no pseudoinverse, jitter,
variance floor, clipping or covariance inflation. Numerical failure is not an outlier
classification. For example, zero residual does not make singular $`S`$ acceptable.

## Explicit statistical policy

For a correctly specified zero-mean Gaussian innovation with covariance $`S`$, whitening
yields independent standard-normal components. NIS then follows a chi-square distribution
with $`m`$ degrees of freedom. An upper-tail test compares NIS with a fixed critical value.
NIST's [distribution definition][nist-distribution] and [critical-value table][nist-table]
provide the reference for the explicit preset below.

`EskfInnovationPolicy` has two optional, strictly positive finite dimensionless fields:

- `local_position_nis_threshold` for the complete 3D position residual;
- `barometric_altitude_nis_threshold` for scalar altitude.

A threshold is a NIS value, not a confidence probability, variance or standard deviation.
Booleans, strings, complex values, zero, negatives and nonfinite scalars are rejected.
An absent threshold means diagnostics-only for that sensor. Position gating is joint
across all three axes; it does not test or accept axes separately.

The explicit factory `chi_square_99_percent_eskf_innovation_policy()` returns the rounded
table entries 11.345 for 3D position and 6.635 for altitude. They approximate the 99th
percentiles; they are not exact computed quantiles. Under the stated Gaussian model this
corresponds to approximately 1% marginal false rejection per observation. It is not a
sequence-wide false-alarm guarantee, an outlier-detection guarantee or measured filter
consistency. Successive filter innovations need not satisfy the ideal model in practice.

Thresholds must be chosen before evaluation. The implementation does not estimate them
from observed residuals, retry rejected values, alter $`Q_c/R`$, consult truth labels, or
adapt confidence levels. A poor prior, incorrect bias/noise assumptions or a large
linearization error can reject a valid physical measurement. Prolonged rejection can
leave the filter drifting. Statistical gating alone does not establish robustness.

## Three execution modes and compatibility

`EskfReplayConfiguration` appends `innovation_policy`, defaulting to `None`:

| Configuration | Pre-update diagnostics | Statistical rejection |
| --- | --- | --- |
| `innovation_policy=None` | No additional scoring | None; legacy execution |
| `innovation_policy=EskfInnovationPolicy()` | Both fresh enabled sensors | None |
| Policy with one or both finite thresholds | Both fresh enabled sensors | Only sensors with a threshold |

The nominal replay helper forwards an explicit optional policy; it never infers one from
truth, nominal noise or generated measurements. All existing positional constructors
remain valid because new configuration and event fields are appended with defaults.

The unscored default is intentional. A finite correction does not imply representable
NIS: zero prior covariance can give zero gain and a valid unchanged state even when a
finite residual squared would overflow. Enabling diagnostics opts into the documented
finite scoring domain. The default retains the original correction domain without
performing unnecessary whitening. Within the common domain, diagnostics-only leaves
state, covariance and correction bytes unchanged in the verified cases.

## Gate placement and event records

The runner retains the ADR 0006 precedence and position-before-altitude order:

1. Pending records remain pending; delivered older acquisitions become `STALE`.
2. A fresh observation with its fusion flag off becomes `DISABLED`.
3. A fresh enabled observation is scored only when a policy is present.
4. With threshold $`\tau`$, reject if and only if $`\epsilon_\nu>\tau`$. Equality is accepted.
5. Otherwise invoke the unchanged sensor-specific correction, injection and reset.

`REJECTED` never reaches correction. Its state and covariance remain exactly the
pre-observation values: no state injection, Joseph update, reset or added process noise
is performed for that observation. Time propagation before the observation is unaffected.
The next same-epoch sensor uses this unchanged prior after rejection, or the actual
posterior after acceptance. The runner does not score every sensor against a cached
beginning-of-epoch prior.

`EskfReplayEvent` appends `innovation: EskfInnovation | None` and
`nis_threshold: float | None` to its existing observation/status/update fields:

| Event status | `update` | `innovation` and threshold |
| --- | --- | --- |
| Unscored `FUSED` | Full correction result | Both absent |
| Scored `FUSED` | Full correction result | Pre-correction diagnostics; threshold optional and passed |
| `REJECTED` | Absent | Required diagnostics and a threshold strictly below NIS |
| `STALE`, `DISABLED`, `PENDING` | Absent | Both absent |

The observation retains sensor kind, source row, acquisition/delivery indices and measured
value. Constructors reject contradictions between status, threshold, dimension and update
diagnostics. Scored fused residual/covariance must match the correction's diagnostics.
Each event and result owns its nested copies. Records for rejected measurements remain
available, avoiding silent deletion of large innovations from downstream analysis.

Results remain in memory. The 35-array artifact and manifest formats are unchanged and
do not persist the policy or estimator events. A caller must retain its explicit estimator
configuration to reproduce a scored replay. No RNG or truth field is accessed. Any later
numeric error aborts without returning a partial result or mutating caller-owned inputs.

## Verification and remaining limits

Analytic tests cover scalar/diagonal residuals, generic observation dimensions, correlated
covariance with an independent solve reference, measurement-coordinate invariance and
exact agreement with the existing update diagnostics. Extreme-scale, finite, dtype,
shape, covariance, singularity, overflow, subnormal-norm and ownership tests define the
numeric boundary. An independent closed-form CDF calculation checks the rounded preset.

Seeded known-Gaussian innovation tests validate whitening and predeclared NIS moment and
acceptance bounds. They are tests of this statistic, not a consistency evaluation of the
nonlinear ESKF. Gate tests cover threshold equality and adjacent representable values,
correction-call traps, same-epoch sequential priors, eligibility precedence, independent
sensor thresholds, no hidden gate on errors, and unchanged caller buffers on later failure.

Generated and saved/loaded sensor integration tests cover fixed one-record and three-record
corruption fixtures, both signs and both sensors, exact rejection-versus-omission equality,
subsequent fresh fusion, and identical persisted-input replay diagnostics. Corruption is
test-only; the production sensor generator is unchanged. An independent six-case baseline
comparison checks default replay output bytes. Exact counts and commands are recorded in
the linked verification record rather than inferred from the statistical reference.

[ADR 0008](0008-eskf-consistency-evaluation.md) describes aligned NEES and a
100-seed-per-case nominal NIS/NEES campaign using this gate contract.
[ADR 0009](0009-eskf-completion-validation.md) supplies excited-motion bias
evidence, fault precision/recall and Q/R sensitivity. Its first-order campaign
shows full-state NEES undercoverage; gating alone is not evidence of statistical
calibration. New observation dimensions or correlated sensor noise require
matching measurement covariances and reference distributions.

[nist-distribution]: https://www.itl.nist.gov/div898/handbook/eda/section3/eda3666.htm
[nist-table]: https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm
