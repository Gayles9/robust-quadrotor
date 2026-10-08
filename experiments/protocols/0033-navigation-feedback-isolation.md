# ADR 0033: supported-hover navigation-feedback isolation

Status: protocol and acceptance frozen before implementation/execution, 2026-09-28.

## Audit and purpose

Audit PR 31 merge `b8ee6b25c2023e5f026ac6c04e551c76f966c2df`, tree
`0af8ad0eea57a6f03428c85916dde27f16119eb1`, matching local and live GitHub main.
Fresh warning-strict diagnosis, earlier-oracle, supported-start and documentation
checks pass 60 tests in 8.43 s. The preceding independent verifier authenticates
30 payloads and reconstructs all six complete response histories and scores.
No preceding defect or evidence damage is demonstrated.

ADR 0032 explains most of the failed seed47001 hover displacement through the
saved navigation-error forcing. That pathwise accounting holds correlated error
histories fixed; it cannot predict the nonlinear result of changing feedback.
This single intervention tests whether the outer navigation channel offers
useful headroom before choosing an implementable information/estimation design.

## Frozen intervention

Run exactly one new complete nominal-hover flight, seed47001, supported aligned,
with supervision on. Compare with the already saved ADR 0031 aligned baseline;
do not rerun or select a new baseline. Authenticate campaign report SHA-256
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63` and the complete
selected history, configuration, release/support and diagnostic records. The
preceding diagnosis report is
`949375db59e656659f8c7378060094b427a2958c1a17ccbc211cff0fa1efe928`.

At each 20 ms outer-controller epoch, replace only its input position and
velocity with the simultaneous simulated true position and velocity. Keep:

- the full ESKF algorithm, initialization, covariance, sample ownership, actual
  measured inputs and causal observation processing;
- the original observer tuple for guards/completion and the original estimated
  attitude and body-rate inputs to the inner controller;
- every controller gain, motor/plant parameter, reference, limit, health policy
  and supervision rule;
- the actual supported acquisition, one fresh supported endpoint, release with
  zero motor speeds and the complete subsequent flight transient;
- the original named random streams and all bias walks.

Sensor values and estimator trajectories will change when physical motion
changes. Identical raw measurements are not expected; reconstruct the same random
draws after removing each flight's physical signal and bias. The supported first
accelerometer sample must use the supported left-limit force in both arms.
No truth attitude/rate, ideal sensor, altered guard, estimator reset, command
smoothing, mass compensation, geometric controller or extra run is allowed.
The adapter is process-local, restores on exceptions, and never runs alongside
another mission thread. Distinct seed47821 smoke executions may test dataflow;
they are not scientific campaign outcomes or seed screening.

## Acceptance and interpretation

1. Save all new payloads and full outer-input trace; authenticate/decode before
   scoring. Reconstruct every ESKF state/covariance/event, changed outer command,
   unchanged inner law, estimated guard/completion decision, health/supervision
   transition and plant/motor interval. Require exact command/estimator dataflow,
   plant/motor residual <=1e-12 and named-draw residual <=1e-12. Reconstruct the
   saved baseline as well. No terminal-epoch command is permitted.
2. Trace all outer epochs against simultaneous saved truth and estimates. Verify
   the observer returns the identical estimated tuple to the mission. Test normal
   and exceptional restoration, wrong channel/epoch, tampering and saved replay.
3. Retain complete scoring from flight zero, inclusive hover window 5..11 s,
   hover peak <=0.08 m, whole-flight/final-position <=0.15 m, final true speed
   <=0.15 m/s and completed mission. Preserve all outcomes and metric regressions.
4. Declare useful navigation-channel headroom only if the oracle completes and
   passes every original flight condition, strictly reduces hover peak, and has
   whole-flight RMSE <= saved baseline +1e-12 m. Otherwise report the actual
   partial improvement or failure without adding runs or changing thresholds.
   Even a passing oracle is diagnostic, never flight qualification or a guarantee
   that an implementable estimator can reach the same result.
5. If headroom is demonstrated, identify one concrete implementable next design
   with explicit sensor/operating assumptions. Define its scope; do not implement
   it in this step. Otherwise state the remaining coupled uncertainty. Do not
   choose gains, Q/R or priors by fitting this one outcome.
6. Pass relevant tests, full software gates and hosted CI; preserve generated
   evidence outside Git and publish the decision. Production algorithms/defaults,
   normal mission integration, the separate mass failure and geometric qualification
   remain unchanged. A valid failure is a completed experiment, not a reason to
   retry. An implementation/evidence defect must be disclosed and fixed at source.
