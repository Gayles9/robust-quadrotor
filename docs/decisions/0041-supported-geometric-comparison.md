# Decision 0041: one supported-start geometric comparison, then pause

Frozen before implementation or candidate scientific flight execution on
2026-09-29. Audited baseline:
`b855329d9d83f7c211883fcd24e390bdb2977218` (merged PR #40).

## Audit and question

The preceding milestone changes documentation/reporting only. Local and remote
main agree; the working tree is clean. Fresh warning-strict checks of geometric
control/filter/shaping, supported starts, combined conditioning and nonlinear
release pass 102 tests. The existing source accepts geometric estimated missions
and explicit supported configurations, but the saved command auditor currently
handles cascade control only. Extend that auditor for this bounded experiment.

ADR 0020's coherent-force geometric candidate passed its spline comparison but
failed both full hovers. The later supported initialization/release combination
has only been evaluated with cascade control. Test that specific combination;
do not reopen a gain, filter or sensor search. The current request supersedes
the proposed middleware-interface design. Professor-facing report preparation
is deferred until after this study and its pause decision.

## One fixed candidate and matched controls

Use ADR 0020's third-section feedback force and derivative jets at 10 rad/s,
original position and geometric attitude gains, unchanged clocks, limits,
sensor distributions and ESKF correction policies. No rebasing, measured
derivatives, vertical integral or new measurement. Combine the existing 0.5 s
externally supported alignment, one-time exact zero-world-velocity prior and
order-five nonlinear first release prediction. Later prediction is ordinary
ESKF. Physical support is an explicit fixture assumption, not an IMU inference
or a hardware claim.

Three clean arms share the support fixture, initial truth, motors initially off,
pre-arm acquisition and every named flight random stream:

1. `unaligned_geometric`: same coherent controller, independent original prior
   with elapsed bias-walk covariance, ordinary endpoint prediction; no alignment.
2. `combined_geometric`: the one candidate with all three initialization fixes.
3. `combined_cascade`: identical combined initialization with the original cascade
   gains, including explicit minimum-snap permission. This is the fair controller
   comparator; do not substitute the differently tuned v2 cascade.

The first comparison isolates the complete estimation/startup package under the
same supported physics. It does not isolate the three fixes individually or
relabel the earlier freely flying baseline. Both geometric arms use the same
unchanged shaped-reference function and fresh filter memory.

## Development ledger and gates

Run exactly 28 scientific flights: three arms for each full hover at seeds 30
and 93012, the original nominal minimum-snap mission at seed 30 and the original
mild-wind minimum-snap mission at seed 31 (12 flights); then the eight existing
supported fault cases at seed 47004, each with candidate supervision off/on
(16 flights). Complete and retain the entire development ledger even after a
performance failure. Use at most two worker processes and no shared-thread
patches. Software smoke cases use seed 40 and cannot count as performance data.

Freeze acceptance before seeing outcomes:

- Every candidate clean flight must complete, with full-mission time-integrated
  RMSE <=0.15 m, peak <=0.25 m, final position error <=0.08 m and final true speed
  <=0.08 m/s. No command limiting or rotor saturation is allowed. The explicitly
  stationary initial motor value at t=0 is permitted; actual speeds must be
  strictly inside (0,900) rad/s at every later sample, and all commanded speeds
  must be inside that interval. This exception is required by the support fixture.
- Both full hover windows are inclusive 5..65 s, covered in full, with peak
  <=0.08 m. The worst candidate hover peak must be strictly lower than the worst
  matched unaligned-geometric peak. Record every individual improvement/regression.
- Both spline cases must have candidate RMSE <= matched combined-cascade RMSE;
  the cascade spline flights must also pass their physical conditions.
- Integrated squared actual moment over each complete candidate clean flight
  must be <=2 times the matched combined-cascade effort. Retain whole-flight
  effort even if completion times differ; do not trim a unfavorable interval.
- All eight fault-response comparisons must pass the existing causal timing,
  priority, supervision and recovery criteria. This is numerical abort evidence,
  not a physical fallback demonstration. Persistent-fault aborts need not complete.
- Every stored outcome must pass independent saved ESKF/command/plant/noise,
  support and fault-ledger reconstruction. For each combined arm, compare its
  first prediction against order-seven quadrature using the existing ADR 0036
  accuracy limits. Errors, missing/duplicate cases and failed audits fail closed.

This gate deliberately permits individual metric regressions within absolute
limits. It does not require every hover to beat every comparator. Trajectory
accuracy cannot be traded away against cascade, and at most the original factor
of two in moment effort is allowed. Earlier failed no-regression studies keep
their original decisions; these new prospective criteria cannot change them.

## Conditional fresh validation and stopping

Only a fully passing authenticated development report permits fresh validation.
Freeze four full hovers at reserved seeds 95000..95003 and four spline cases at
96000..96003 (even nominal, odd mild wind), each with the same three arms, plus
the same eight candidate fault pairs at new seed 97000: exactly 40 flights.
Use identical scoring and comparisons, with no parameter change after development.
Do not even construct these seed-dependent configurations before the gate passes.

Any failed development gate ends scientific execution before fresh validation.
A passing development/fresh comparison would establish this bounded supported
simulation improvement only; broad qualification, true-state shaped-controller
regression and production integration remain separate requirements. There is no
automatic default promotion. Preserve code, all outcomes, source/protocol hashes
and evidence; publish the tested study and pause controller improvement. The next
user-directed task is professor-facing report preparation, not another test round.
This experiment cannot prove that no future controller or estimator can do better.
