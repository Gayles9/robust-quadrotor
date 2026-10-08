# Diagnosing residual hover error

This analysis explains why supported alignment can improve startup yet leave a
hover failure. It reconstructs saved feedback and physical motion, then accounts
for how different forcing terms contribute to the final trajectory. It changes
no flight behavior and cannot turn a failed flight into a passing one.

## Saved inputs and scope

The six saved nominal-hover histories use seeds 47001, 47002 and 47003, each
with supported unaligned and aligned initialization. The aligned seed 47001
flight is the primary failure; the other five provide descriptive comparisons.
The input campaign report has SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
All 34 original campaign scores and decisions are independently reconstructed
before using the selected histories.

Complete histories retain the inclusive 5..11 s hover window, 0.08 m hover peak,
0.15 m whole-flight RMSE/final-position limits, and original speed and completion
rules. The analysis introduces no new flight, oracle intervention, gain search
or controller change.

## Trajectory accounting

Saved commands are reconstructed from the original gains and estimated state,
along with each physical plant/motor interval. The diagnosis distinguishes true
tracking error, estimated tracking error, navigation error, thrust-axis
estimation/tracking, bias error, motor response and drag.

A sampled linear PD response decomposition retains reference forcing, initial
state, navigation forcing, command limits, attitude estimation and tracking,
mass, motor lag, drag and within-step integration residuals. The summed response
must reconstruct the complete saved physical trajectory. Signed vectors and
contributions are reported because vector norms are not additive percentages.

The acceleration terms form an explicitly ordered identity. Their response is
**pathwise accounting**: every recorded forcing history stays fixed. Removing
one term from that sum is not a nonlinear simulation of an improved estimator
or controller. Correlated feedback terms can cancel one another.

The fixed 0..0.5 s release interval is reported separately, including its
propagated contribution, but it does not isolate what would happen under
different physical support. Support is absent after release in all these
flights.

The optional near-level sampled cascade model derives its coefficients from
existing gains, inertia, motor lag and clocks, without fitting them to the
failure. Model discrepancy and neglected terms remain explicit. Eigenvalues
of this approximation do not qualify nonlinear stability.

## Accuracy requirements and interpretation

| Reconstructed quantity | Maximum residual |
| --- | --- |
| Controller commands | 1e-12 |
| Plant/motor steps | 1e-12 |
| Acceleration identity | 1e-11 m/s² |
| Full response in position and velocity | 1e-10 m and m/s |

Independent analytic and synthetic cases check NED signs, sample holds, impulse
response, correlated cancellation, release timing and failure-preserving
scoring. Residuals outside these bounds indicate a reconstruction problem,
not a reason to retune acceptance.

The result distinguishes quantitatively explained mechanisms from effects that
remain coupled or unidentified. A useful diagnosis may conclude that more
information is needed before an implementable correction is justified.
[Navigation-feedback isolation](navigation-feedback-isolation.md) tests the
causal headroom suggested by this accounting. Mass compensation, normal mission
integration and hardware readiness remain separate questions.
