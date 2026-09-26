# Fixed-duration minimum-snap implementation — 2026-09-25

## Scope and mathematical work

The preceding [repository audit](2026-09-25-repository-audit.md) starts from
`cad64cf37d945fcf2f2186d070e8f845d19edf25`. I retained the stronger existing
feedback reference with its measured limitations and completed the
bounded planning package. [ADR 0015](../decisions/0015-minimum-snap-trajectory.md)
was written before the solver implementation and fixes its acceptance criteria.

`src/quadrotor_math/minimum_snap.py` implements seventh-degree position segments
in normalized time, fixed waypoints/durations, endpoint v/a/j, C3 knots,
integrated squared snap, evaluation through the fourth derivative and a
fixed-yaw `PositionReference` adapter. The Hermite map eliminates equality
constraints. A factor of the analytic snap Gram matrix gives the reduced
least-squares objective. Time, geometry and variable scaling precede the SVD
solve; rank, condition, stationarity, endpoint residuals and cancellation are
checked without changing the objective. Core runtime still depends only on NumPy.

The uniqueness proof, derivative units, T^-7 cost law, API and numerical domain
are in the [trajectory guide](../trajectories.md). Positive-definiteness is on the
feasible variation space, not the full coefficient Hessian, whose cubic block
is correctly zero. No vehicle model or estimator state enters the optimizer.

## Tests, failures and corrections

- Initial focused collection failed because the new solver module did not exist.
- Analytic rest-to-rest coefficients/cost and an independent full coefficient-space
  equality-constrained oracle then passed after implementation.
- A test requested a scaled terminal time one float64 unit beyond the stored
  interval. The test now queries each curve's stored endpoint; production retains
  its explicit no-extrapolation contract.
- An extreme-duration review found a real numerical issue in the first solver:
  scaling residual tolerance by coefficient size could hide metre-scale endpoint
  cancellation. The corrected check uses expected derivative scales and a
  conservative evaluation-roundoff bound. A regression rejects that example even
  though the reduced linear-system condition is only about 17.
- Static checks caught an ambiguous test variable and two long example strings;
  these were corrected. Ruff also formats Python snippets in Markdown, so the
  new guide's example was formatted before the final gate.

The **36 focused tests** cover physical-time constraints, nonuniform durations,
nonzero endpoint derivatives, independent optimization and quadrature, feasible
cost perturbations, cubic recovery, translation/rotation/time scaling, ownership,
invalid data, out-of-range evaluation, natural derivative continuity and the
64-segment boundary. The example test also verifies safe refusal to overwrite
an existing output directory.

A separate deterministic audit (seed 250925) compared 12 further problems with
the independent coefficient-space oracle: maximum coefficient discrepancy
**2.43e-10**. Twenty-four additional duration-scaling cases obeyed the inverse
seventh-power cost law. These are bounded numerical checks, not flight trials.

## Example result

`experiments.minimum_snap_example` produces a numerical path, derivatives,
independently checked metrics and a figure. For the fixed four-waypoint example:

| Quantity | Result |
| --- | ---: |
| Optimized snap cost [m²/s⁷] | 99.4414949595 |
| Feasible stop-at-every-waypoint cost [m²/s⁷] | 10837.2595160494 |
| Cost reduction | 99.0824% |
| Independent quadrature cost [m²/s⁷] | 99.4414949595 |
| Maximum waypoint residual [m] | 2.58e-14 |
| Maximum C3 knot mismatch, respective derivative units | 4.63e-13 |
| Scaled reduced-system condition | 8.8878 |

The comparison has identical waypoint times and endpoint constraints. It is
neither an energy comparison nor a guarantee that the curve is flight-feasible.
The generated NPZ/JSON/PNG and audit logs are retained outside Git.

## Verification commands

```bash
uv sync --locked
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
uv run pytest -q -W error tests/unit/test_minimum_snap.py tests/unit/test_minimum_snap_example.py
uv run python -m experiments.minimum_snap_example --output /tmp/NEW_minimum_snap
```

The final complete local gate passes **3,188 tests in 330.32 s** with warnings
as errors. Ruff lint passes, Ruff format reports 181 files formatted, and mypy
passes 54 source files. Hosted CI and the merged commit are recorded in the
publication receipt and PR; no partial test run is counted as success.
All 33 pre-existing active runtime modules retain their original blob hashes;
the unused `vectors.py` scaffold alone is removed. Dependencies, lockfile, CI,
controller gains, estimator equations, physical limits, sensor noise and original
feedback scoring are unchanged. No experimental startup policy is added.

## Next exact action

Audit the published solver, then define and implement one bounded package for
trajectory feasibility/time allocation and the true-state mission-runner adapter.
Fix the allowed mission, bounds, retiming budget and tracking/limit metrics first.
Keep the estimated-feedback hover shortfall open. Do not fold geometric control,
new sensor assumptions, fault accommodation or ROS/PX4 into that integration.
