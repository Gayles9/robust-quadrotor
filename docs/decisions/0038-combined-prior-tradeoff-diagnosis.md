# ADR 0038: saved combined-prior tradeoff diagnosis

Status: scope and acceptance frozen before implementation or diagnostic results,
2026-09-29.

## Preceding audit

Audit published commit `5bf719f6b10eafcf736de9956c96e9d035d23726` (PR 36),
tree `30e080436a33195c6c513ea6ffc81ca64f97892b`. The execution source and frozen
ADR 0037 match the original campaign fingerprints exactly after restoring a
cleared checkout. Fresh warning-strict checks pass 91 relevant tests in 8.23 s
and 31 existing observation/response diagnostic tests in 1.55 s. Ruff, formatting,
mypy and documentation checks pass. The separate archived NumPy-only checker
authenticates all 202 payloads and all 25 candidate scores/prior identities.
Hosted CI for PR 36 is pending when this protocol is frozen; merge requires
success. This protocol adds no flight behavior and does not promote the candidate.

The three hover peaks pass 8 cm, but five of nine clean comparisons regress.
Existing reports, covariance ownership, single first-prediction routing and
reconstruction contracts show no demonstrated defect. Exact replay alone does
not prove that a physically better prior improves each finite noisy trajectory.

## Fixed inputs and scope

Use all nine clean jobs (hover, nominal tracking and wind, seeds 47001..47003)
in original aligned, boundary-only and combined form: 27 complete histories.
Authenticate all payloads referenced by the three reports, preserving the full
34/25/25 flight ledgers and fault outcomes. Reconstruct every selected clean
history with its own initial covariance and first-prediction rule. Keep all
previous criteria, support requirements, controller/noise parameters and defaults.

| Report | SHA-256 |
| --- | --- |
| Original | `b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` |
| Boundary | `54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2` |
| Combined | `6d44ab913156fafe3846a630065cdaad9eee2dc9c64c1ea50234b82fbfa314fa` |

No new scientific flights, oracle intervention, seed/gain search, covariance
inflation, measurement gate changes or production integration are in scope.
Truth is available only for offline physical/error accounting. Reconstructing
the fixed support acquisition and unit-test fixtures is not a new flight trial.

## Algebra and explanation

At every fresh observation, independently solve the full 21-state Gaussian
gain, correction and Joseph covariance with the existing independent rotation
reset quadrature. Compare the correction, covariance, sample-noise mean and NIS
against the actual update. Record rejected observations with zero effective gain.
Keep ordered position/altitude updates and compare complete saved replay.

Split each innovation into the saved sensor noise and physical prediction error.
For paired effective gains and innovations, use the fixed ordered identity
`delta_c = (K_candidate - K_control) nu_control + K_candidate delta_nu`.
This is an accounting identity in each update's coordinate conventions, not a
counterfactual filter with a gain or noise source removed. Compare every matched
event on the common time prefix and report differing dispositions and durations.

Reconstruct requested acceleration as reference forcing plus true-state PD
feedback minus position-error and velocity-error feedback. Retain the existing
sample-held response decomposition, including attitude estimation/tracking,
motor, drag, limits, mass and integration remainder. For true tracking error e
and additive response channels e_i, report signed MSE shares
`mean(e dot e_i)`, whose sum is `mean(e dot e)`. Changes use each flight's full
duration; do not silently truncate the frozen score. Signed contributions may
cancel and must not be presented as additive norms or causal percentages.

For the nine hover histories only, run the existing near-level sampled cascade
model with its original gains, inertia, motor lag and clocks. Report full
horizontal RMS/peak discrepancy and paired trajectory differences. No coefficient
may be fitted. Model agreement means RMS discrepancy at most 0.002 m in every
hover history; failing this explanatory check must remain visible. It is distinct
from the exact diagnostic acceptance and from nonlinear flight qualification.

## Acceptance and stopping decision

1. Authenticate the three fixed complete reports/payloads and rescore all 84
   saved outcomes. Reconstruct all 27 selected clean histories exactly, including
   commands, guards, plant/motors, noise pairing and first-prediction traces.
2. Independent corrections, covariance and sample means agree within 1e-10;
   NIS within 1e-9. Gain/innovation and requested-acceleration identities close
   to 1e-12. Existing force and response bounds remain 1e-11 and 1e-10.
   Signed MSE closure must be within 1e-12 m². Investigate discrepancies at
   their source; do not tune thresholds after seeing results.
3. Test hand-computable signs, covariance-induced gain changes, noise/error
   decomposition, rejection, ordered update ownership, correlated cancellation,
   mismatched dimensions/clocks, and failure-preserving report authentication.
   Pass relevant quality checks and hosted CI before merging publication.
4. Preserve every diagnostic outcome and explicit model limitations. Establish
   whether the measured redistribution of corrections/feedback explains the
   tradeoff without claiming finite samples prove population optimality.
5. If no specific correctable defect or independently justified new hypothesis
   is demonstrated, close this startup candidate without another tuning trial.
   Retain the failed ADR 0037 gate and open fresh qualification, mass-offset,
   geometric-control and integration requirements. Record the next exact task.

Successful diagnosis may conclude that no further startup implementation is
justified. It cannot retroactively pass a flight gate.
