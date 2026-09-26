# 2026-09-25: Trajectory bounds, timing and mission integration

## Scope and preceding audit

The trajectory-mission package followed PR #12. I audited published main
`76db6eefea09aa25be236b1246cdc21ba44d1557`; post-merge CI `36142551257` passed and
36 fresh minimum-snap tests passed in 1.54 s. Review found no new solver defect.
Its representation constructor intentionally does not promise C3 continuity or
rest endpoints, so mission consumers now validate those properties themselves.
[ADR 0016](../decisions/0016-trajectory-feasibility-and-missions.md) fixed the
mathematics, boundaries, timing budget and mission acceptance before implementation.

## Implementation

- Whole-curve Bernstein bounds for NED geometry, speed, acceleration, nominal
  thrust, tilt and fixed-yaw reference angular rate, with subdivision and
  numerical padding. Unsupported numerical representation fails explicitly.
- Uniform geometry-preserving retiming, all attempts retained, explicit failure
  and bounded scale/attempt count. No minimum-time claim or gain search.
- An owned polynomial mission-segment subclass and true-state preflight; existing
  supervisor, controller clocks, gains, plant, motors and sampled guards retained.
- Five reproducible full-mission cases, independent histories, hashes and metrics.

The historical fixed position-mission protocol was serialized before and after
the change and remains byte-identical. Its dataclass schema was preserved by
using a subclass only for polynomial segments. Estimated-feedback polynomial
execution remains outside scope and is rejected explicitly.

## Fixed mission evidence

The first run passed every declared case without tuning. The final run added a
source-stability check, passed it, and reproduced all five NPZ files byte for byte.
Reference timing uses no observed tracking results. Seven scales were tried;
the first passing scale is 3.814697265625, giving a 25.2587890625 s reference and
completion at 25.76 s, including the original sampled landing dwell.

| Case | Full-mission position RMSE [cm] | Peak [cm] | Final error [cm] | Limiting |
| --- | ---: | ---: | ---: | --- |
| Nominal | 3.265913 | 6.675213 | 0.183712 | None |
| Half plant step | 3.265913 | 6.675213 | 0.183712 | None |
| Positive initial offset | 3.365934 | 6.675212 | 0.183712 | None |
| Negative initial offset | 3.365888 | 6.675214 | 0.183712 | None |
| Mild wind and drag | 3.924654 | 7.736319 | 2.709032 | None |

All satisfy the frozen 15 cm RMSE, 25 cm peak and original terminal 8 cm /
8 cm/s / 0.5 s dwell / 8 s timeout conditions. All arrays are finite; no inner,
outer or rotor-bound limiting occurs. Maximum actual tilt is 2.391 degrees.
The nominal actual speed reaches 0.612397 m/s while the reference is bounded by
0.6 m/s; the planning speed limit is not an actual-vehicle speed constraint.
Nominal/refined common-epoch differences are at most 1.89e-10 m position and
2.42e-6 degrees attitude (the latter near arccos numerical resolution), within
the original 5 mm / 0.05 degree refinement criteria.

The unchanged matched true-state controller is used here. These results do not
reclassify the original version-1/version-2 estimated-feedback validation failures
or resolve the 8 cm full-hover requirement. The mild-wind case is a fixed bounded
example, not robust qualification against an uncertainty distribution.

## Verification and limitations

New focused tests independently verify analytic speed extrema, a between-endpoint
geofence excursion, direct polynomial evaluations, finite-difference SO(3) rates,
retiming derivative/cost scaling, exhausted search budgets, array ownership,
malformed representations, phase boundaries, preserved historical serialization,
preflight before plant execution and honest scoring of an abort at time zero.
The initial test runs failed because the new APIs did not yet exist; the subsequent
focused suite passes. Routine formatting/import/zip-strict issues were corrected.

Numerical hull padding is not formal interval arithmetic. Reference bounds do not
prove motor/torque, drag-compensated, closed-loop or hardware feasibility. There
are no obstacles, independent segment-time optimization, estimator changes or
geometric control in this package. The next scope is a bounded geometric tracking
controller and fair true-state comparison using the same trajectory and plant.

Final local command: `OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check`.
Result: **3,228 tests passed in 400.66 s**, with Ruff lint, all 187 files formatted
and strict mypy clean for 56 source files. The 40 added tests preserve all 3,188
previous tests. Documentation links and `git diff --check` pass.

Final execution source SHA-256:
`53af1a5c48ff3562ca99538f7723a6715b6418cdf8a40abe6a268c523d7d8842`.
Frozen protocol SHA-256:
`ebd6309a06069f19f9aaf4d45791412bf91e377cc269fc0bdff4ccf724246172`.
All five archives authenticate; independently integrated squared errors reproduce
the reported RMSE and peaks. The checked figure shows the complete path, all
case errors, reference/actual speeds and the attempted timing scales.
The final publication and hosted-CI receipts accompany the evidence archive and
master engineering log; no generated histories or plots are committed to Git.
