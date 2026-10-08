# Geometric control with supported initialization

This study tests whether supported alignment and improved release prediction
help a geometric controller that previously passed its spline comparison but
failed both full hovers. The comparison uses a fixed controller profile, so it
isolates the combined startup treatment without adding a gain or sensor search.

## Candidate and matched controls

The controller uses the third-section coherent feedback force and derivative
jets at 10 rad/s, original position and geometric attitude gains, and unchanged
clocks, limits, sensor distributions and estimator correction policies. It adds
no rebasing, measured derivatives, vertical integral or new measurement.

The startup package combines 0.5 s externally supported alignment, one-time
exact zero-world-velocity conditioning and order-five nonlinear first-release
prediction. Later prediction uses the ordinary estimator. Support is an
explicit ideal fixture assumption, not an inference from IMU quietness or a
hardware claim.

Three arms share support, initial truth, initially stopped motors, acquisition
samples and every named flight random stream:

| Arm | Controller and initialization |
| --- | --- |
| `unaligned_geometric` | Coherent geometric controller; independent original prior plus elapsed bias-walk covariance; ordinary endpoint prediction |
| `combined_geometric` | The same controller with the complete startup package |
| `combined_cascade` | Original cascade gains with the same complete startup package and explicit minimum-snap permission |

The cascade comparator retains its original gains, rather than the separately
tuned v2 cascade. The first two arms isolate the complete estimation/startup
package under the same support physics, not each correction separately. They
do not relabel the earlier freely flying baseline. Both geometric arms use the
same shaped-reference function and fresh filter memory.

## Development cases and requirements

The development ledger contains 28 scientific flights. Three arms run each full
hover at seeds 30 and 93012, the original nominal minimum-snap mission at seed
30, and the original mild-wind minimum-snap mission at seed 31: 12 flights.
Eight supported fault cases at seed 47004 add candidate supervision off/on
pairs: 16 flights. All outcomes remain in the ledger, including failures.
Execution uses at most two processes without shared-thread patches. Seed 40
is reserved for software smoke cases, which do not count as performance data.

The prospective development requirements are:

- Every candidate clean flight completes with full-mission time-integrated
  RMSE at most 0.15 m, peak at most 0.25 m, final position error at most 0.08 m
  and final true speed at most 0.08 m/s.
- There is no command limiting or rotor saturation. Actual rotor speed may be
  zero only at the initial supported t=0 sample. All later actual speeds and
  every commanded speed are strictly inside (0,900) rad/s.
- Both inclusive 5..65 s hover windows are fully covered, with peak at most
  0.08 m. The worst candidate hover peak is strictly below the worst matched
  unaligned-geometric peak. Individual improvements and regressions remain
  visible.
- Candidate RMSE on each spline is no greater than its matched combined-cascade
  RMSE, and both cascade spline flights meet their physical requirements.
- Integrated squared actual moment over each full candidate clean flight is
  at most twice its matched cascade value. Complete-flight effort remains
  scored even when completion times differ.
- All eight fault pairs meet the existing causal timing, priority, supervision
  and recovery criteria. Persistent-fault numerical aborts need not complete
  the mission and do not demonstrate physical fallback.
- Saved estimator, command, plant, random-draw, support and fault ledgers pass
  independent reconstruction. Each combined first prediction agrees with
  order-seven quadrature under the nonlinear-release accuracy limits.

These criteria allow some individual metric regressions within absolute limits.
They do not require every hover to beat every comparator. Spline accuracy and
the factor-of-two effort bound remain required, and these criteria do not
retroactively change earlier no-regression results.

## Conditional independent validation

Only a fully passing authenticated development report makes the separate
validation ledger eligible: four full hovers at seeds 95000..95003 and four
splines at 96000..96003, with even seeds nominal and odd seeds mild wind, each
using all three arms. Eight candidate fault pairs at seed 97000 bring that
ledger to 40 flights. Scoring and parameters remain fixed after development.
A failed development gate leaves this conditional ledger unexecuted.

Even a passing comparison would establish only bounded supported-simulation
improvement. Broader qualification, shaped-controller true-state regression,
production integration and default promotion are separate decisions. The
[final comparison](final-geometric-comparison.md) explains the subsequent
bounded tuning and validation design.
