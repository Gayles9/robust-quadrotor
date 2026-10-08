# Exact velocity information at supported release

If an external fixture holds the vehicle stationary in the world, its velocity
at release is known to be zero. That is stronger information than zero
acceleration or quiet IMU measurements. This experiment checks how to represent
that constraint, and whether the first estimator prediction carries enough
uncertainty after support disappears.

The navigation oracle shows combined position/velocity-feedback headroom; it
does not establish the benefit of a one-time velocity constraint. This candidate
keeps controller and estimator equations, gains, Q/R, sensor draws, support
duration and scoring unchanged.

The [recorded screen](../results/supported-velocity-prior.md) rejected all 17
release configurations with the original integration model. No new flights were
run for this candidate; the flight criteria below describe a conditional stage
that was not reached.

## Gaussian conditioning

For the 21-state endpoint covariance `C`, velocity selector `H`, exact supported
velocity zero, prior nominal velocity zero and $`R_s=0`$, the calculation is

```math
S=HCH^T,\qquad K=CH^TS^{-1},\qquad C^+=(I-KH)C(I-KH)^T.
```

Implementation uses a linear solve rather than forming a full inverse. The
accepted input has a positive-definite 3×3 velocity covariance, zero velocity
mean, no velocity cross terms with other physical states or sample noise, and
independent fresh sample noise. Other means and all non-velocity covariance
entries remain unchanged. The resulting exact-zero block is positive
semidefinite and rank deficient; no numerical floor is inserted. This contract
does not cover correlated attitude correction or uncertain hardware support.

A separate world-zero-velocity assertion binds the same support identity and
fresh observed epoch. Zero world acceleration, motors off or quiet IMU data
alone are insufficient: a steadily moving platform is a counterexample.
Release identity, sample sequence, clock/profile, current support flags and
deadline are checked. Missing, stale, revoked, moving or mismatched support is
rejected. The local one-shot conditioner is consumed even when it rejects.
The caller remains responsible for unique IDs and physical authentication.

Saved constant pose, zero velocity/motors and force balance substantiate the
simulation fixture's assertion. Feedback does not read that truth. The output
is an estimator prior, not an arm command.

## Release uncertainty screen

The first IMU sample is taken just before support disappears. During the first
interval $`h=0.0025\;\mathrm s`$, the original endpoint map assumes linearly interpolated
acceleration. A stationary, motor-off, no-drag release gives a deterministic
counterexample: $`\mathbf a(0^-)=\mathbf0`$ while $`\mathbf a(t>0)=\mathbf g_W`$. The estimator's position and velocity
are compared with the analytic ballistic solution.

The input campaign report has SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Its 17 release configurations comprise three hover, three nominal tracking,
three wind tracking and eight fault cases. Each retains the full alignment,
bias and fresh-sample uncertainty, evaluated both before and after velocity
conditioning. Mean-consistent endpoint IMU values avoid adding another random
draw. An independent NED-down velocity variance calculation must agree with the
production map within 1e-12.

The necessary uncertainty screen requires the known absolute down-velocity bias
to fit inside the two-sided Gaussian 99% marginal radius
`2.5758293035489004 * sigma_down`. Reported quantities include ranks, full
covariance, predicted and analytic endpoints, signed bias, standard deviation,
radius and bias/sigma ratio for every case.

Failure means the claimed uncertainty omits a known release error. Passing is
not sufficient for calibration or flight qualification. A broad prior is not
an implicit discretization-error model. A failed candidate screen yields zero
new scientific flights and retains all 17 outcomes.

## Conditional flight criteria

The planned regression stage is eligible only if every release passes. It
contains exactly 25 candidate flights: nine aligned clean jobs and supervision
off/on arms for the eight fault jobs, compared with saved baselines. Complete
startup remains scored. The limits are 8 cm hover, 15 cm position/RMSE,
15 cm/s final speed, and original completion and fault-response requirements.
All three hovers must pass; clean hover peaks and whole-flight RMSE cannot
regress by more than 1e-12. These are known-seed regression checks, not fresh
validation.

One-shot behavior, rejection, conditioning, rank, ownership, preserved blocks,
independent covariance reconstruction and the analytic release example are
verified separately. A justified rejection is a useful mathematical result,
but does not improve ordinary flight performance. The release model is developed
in [one-sided prediction](release-aware-prediction.md).
