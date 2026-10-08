# ADR 0034: supported velocity prior and release uncertainty gate

Status: frozen before implementation and numerical evaluation, 2026-09-28.

## Audit and scope

Audit PR 32 merge `a54145ed1e8cdd49cea8ffda5925bb5da12d054c`, tree
`8e73691278f8add04260b284fa38ef3b18e82678`, matching clean local and live GitHub
main. Fresh warning-strict oracle, support, pre-arm and documentation checks pass
119 tests in 11.87 s. The preceding independent verifier authenticates all 26
payloads and reconstructs the paired scores, commands, trace and noise draws.
No preceding implementation or evidence defect is demonstrated.

ADR 0033 establishes combined position/velocity-feedback headroom, not the benefit
of a one-time velocity constraint. Implement one experiment-only conditioning
candidate using genuinely world-stationary support at the fresh release epoch.
Keep production source, controller, estimator equations, gains, Q/R, sensor draws,
support duration and existing scoring unchanged. Do not reopen geometric control,
combine mass compensation, tune a covariance floor or assume in-flight stationarity.

## Conditioning and contract

With the 21-state endpoint covariance `C`, velocity selector `H`, exact supported
velocity zero, prior nominal velocity zero and `R_s=0`, use

```text
S = H*C*H^T
K = C*H^T*S^-1
C_plus = (I-K*H)*C*(I-K*H)^T.
```

Use a solve, not a full covariance inverse. This candidate accepts only the
existing independent initial velocity block: zero velocity mean, positive
definite 3x3 velocity covariance, no velocity cross terms with other physical
states or sample noise, and independent fresh sample noise. Other means and all
non-velocity covariance entries must be unchanged. The exact zero block is PSD
and rank deficient; never insert a floor. Do not generalize to a correlated
attitude correction or hardware support variance in this step.

Require a separate explicit world-zero-velocity assertion, bound to the same
external support identity and fresh observed epoch. Zero world acceleration,
motors off or quiet IMU data alone is insufficient (a moving platform is a
counterexample). Check release identity, sample sequence, clock/profile, current
support flags and deadline; reject missing, stale, revoked, moving or mismatched
support. A local one-shot conditioner is consumed even on rejection. The caller
still owns unique acquisition IDs and physical authentication across processes.
The experiment fixture may substantiate the external assertion from its saved
constant pose, zero velocities/motors and force balance; flight feedback may not
read that truth. The returned prior is a diagnostic candidate, not arm permission.

## Mandatory release screen before flight comparisons

The current first IMU sample is a supported left limit. The first interval has
duration `h=0.0025 s`; support disappears immediately after sample zero. The
endpoint map assumes a linearly interpolated acceleration between samples.
Derive its exact deterministic counterexample for a stationary, motor-off,
no-drag release into free fall: `a(0-)=0`, `a(t>0)=g_W`.
Compare its predicted position/velocity to the analytic ballistic solution.
This is an integration test case, not a new mission flight or a change to the
campaign plant, sensor noise or flight score.

Authenticate the complete original ADR 0031 campaign, report SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Reconstruct support, release and aligned configuration for each of its 17 jobs:
three hover, three nominal tracking, three wind tracking and eight fault cases.
For each release, evaluate the same analytic free-fall boundary using its full
unaltered alignment/bias/fresh-sample uncertainty, both before and after exact
velocity conditioning. Use mean-consistent endpoint IMU values, not another
noise draw. Independently derive the NED-down velocity variance from the saved
covariance/noise blocks and compare it to the production map within 1e-12.

A necessary uncertainty screen is that the absolute known deterministic down
velocity bias fits inside the reported two-sided Gaussian 99% marginal radius
`2.5758293035489004 * sigma_down`. Failure demonstrates an unrepresented release
error too large for the proposed uncertainty. Passing is not sufficient for
calibration, hardware validity or flight qualification. Report baseline and
candidate ranks, full covariance, predicted endpoint, analytic endpoint, signed
bias, standard deviation, radius and bias/sigma ratio for every job.

The existing documentation already warns about this force discontinuity. If any
candidate release fails this screen, reject the candidate before flight, retain
all 17 outcomes and execute **zero new scientific flights**. Do not use a broad
prior as a hidden discretization-error model, repair the endpoint map or retune
uncertainty in this step. Establish the next bounded release-integration design.

Only if every release passes may the planned regression stage run: exactly 25
candidate flights (nine aligned clean jobs and both supervised/off arms for eight
fault jobs), compared with authenticated saved baselines, never rerun baselines.
Retain full startup, all original 8 cm hover, 15 cm position/RMSE, 15 cm/s final
speed, completion and fault-response conditions. Require all three hover flights
to pass, no hover peak or whole-flight RMSE regression beyond 1e-12 for clean jobs,
and every original fault-response condition to pass. No valid outcome is repeated.
These known seeds are regression only; fresh validation precedes integration.

## Verification and closeout

Test one-shot and rejection paths, exact Gaussian conditioning and rank/ownership,
preserved non-velocity blocks, independent endpoint/noise covariance reconstruction,
and the analytic release counterexample. Authenticate serialized evidence and
independently reconstruct every reported decision. Pass relevant tests, full
software gates and hosted CI; preserve evidence outside Git and publish the
decision. A justified candidate rejection completes this experiment successfully,
but does not improve or qualify ordinary flight performance.
