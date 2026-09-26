# 2026-09-26: Geometric replacement and fresh validation

## Scope and source identity

Audited base: `23936ebd8403a3620f755a53143fa7c9f641d491`. Main matched the live
repository, and 238 fresh controller/mission checks passed before implementation.
The original geometric source remained unavailable, so I rebuilt the controller
and required fresh verification. [ADR 0017](../decisions/0017-geometric-reimplementation.md)
froze the fresh acceptance campaign before implementation. No gains were tuned.

Implemented the twice-differentiated force-to-attitude map, geometric moving-frame
moment law with full inertia, immutable causal derivative filter, and opt-in
geometric paths in both true-state and ESKF mission runners. Analytic HOLD,
quintic SMOOTH and minimum-snap derivatives are supported; STEP is rejected.
The cascade default, estimator, plant, sensors and toolchain remain unchanged.

This replaces the corrected filtered design; it does not recreate the rejected
nominal-model derivative closure. Projection is recomputed at each inner tick.
Historical test counts and controller histories are not inherited qualifications.

## Fresh evidence

Execution source SHA-256: `73d4a5b0db2e6cfb8f6cb09d907ba597becd8f37ee4fb7e6d5b16135950d14d0`. Source stayed unchanged
during all 28 planned executions. Protocol SHA-256:
`e0b751ed1ed89d6a24a68171814fea43d4cb14914305c52528e57f0cbed798b1`.

| True-state case | Cascade RMSE (cm) | Geometric RMSE (cm) | Geometric peak (cm) |
| --- | ---: | ---: | ---: |
| nominal | 3.265913 | 0.202813 | 0.595374 |
| refined | 3.265913 | 0.202813 | 0.595374 |
| offset_positive | 3.365934 | 1.017507 | 3.297564 |
| offset_negative | 3.365888 | 1.012433 | 3.287579 |
| mild_wind | 3.924654 | 3.569739 | 7.506940 |
| reversed_wind | 3.772044 | 3.665280 | 7.643064 |
| wind_offset | 4.036801 | 3.749173 | 7.506926 |

All seven geometric flights passed the original physical criteria and per-case
RMSE no-regression condition. First-five mean RMSE ratio is
`0.32737115046888376` (limit 0.80). No controller limiting or actual/commanded
rotor-bound contact occurred. Both controllers pass half-step refinement.

| Estimated spline case | Seed | Cascade RMSE (cm) | Geometric RMSE (cm) | Effort ratio |
| --- | ---: | ---: | ---: | ---: |
| nominal | 30 | 7.4059 | 6.0517 | 10.860 |
| nominal | 31 | 6.7787 | 5.1082 | 11.767 |
| mild_wind | 30 | 6.8223 | 6.1278 | 10.859 |
| mild_wind | 31 | 7.7454 | 7.0095 | 11.799 |

Every estimated spline completes and passes physical tracking limits, but all
four geometric/cascade effort ratios exceed the frozen 2x limit. Effort is the
integral of squared actual body moment (N² m² s), not electrical energy.

| Full 5..65 s hover | Seed | Peak error (cm) | 8 cm result |
| --- | ---: | ---: | --- |
| cascade | 30 | 6.2139 | Pass |
| geometric | 30 | 8.7649 | Fail |
| cascade | 93012 | 8.8686 | Fail |
| geometric | 93012 | 13.6855 | Fail |

Hover compares the rebuilt geometric profile with the established v2 cascade.
Their gains/priors differ as fully recorded in the protocol: this is operational
context, not an isolated control-law comparison. The spline pairs use matched
physical inputs, priors, noise, translational gains and clocks.

**Overall noisy performance qualification fails.** All 28 planned flights
complete without limiting, but the geometric full hovers miss the original
8 cm bound, and noisy spline effort fails. The controller stays experimental;
it does not replace the cascade default. No physical threshold was relaxed.

## Software checks and independent verification

RED: the new tests failed collection because the replacement modules did not
exist. GREEN: 54 new focused tests pass, including independent rotation finite
differences, full-inertia moving-reference energy identity, direct filter transfer
recurrence, frequency/noise bounds, transactional memory, estimate-only command
reconstruction, exact offline ESKF replay, abort clocks and fail-closed case accounting.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest tests/unit/test_geometric_control.py tests/unit/test_geometric_filter.py tests/unit/test_geometric_missions.py tests/unit/test_geometric_campaign.py -q -W error
PATH=/tmp/quadrotor-audit/pinned-tools/bin:$PATH OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.geometric_reimplementation_validation --output NEW_DIR --workers 3
```

**3,282 tests passed in 230.14 s**, warnings as errors; Ruff lint/format clean
(199 Python files), strict mypy clean on 60 source/experiment files.
The campaign exits 1 to preserve its failed performance status.

One full-hover cascade data archive failed its retained SHA-256 and ZIP check
after execution. Deterministic replay reproduced the original metrics and every
original file hash, and restored the damaged payload exactly. The cause of the
file corruption was not established; no numerical result or expected hash changed.
A standalone NumPy verifier then authenticated all 96 payload files and rescored
all 28 records. Both repeats match every saved array, including the noisy
measurements and covariances. All five legacy cascade histories match the prior
retained archive exactly. The verifier imports no production controller or scorer.

## Decision and next bounded action

Keep this implementation available as experimental source. Preserve all historical
failures and these fresh failures. Reimplementation is complete; tuning and noisy
qualification are not. Next diagnose the large torque-effort contribution and
full-hover position error using these saved measurements before choosing one
bounded correction. Do not infer a Kalman-filter bug or a fundamental performance
limit from this campaign. Fault handling, ROS/PX4 and hardware remain separate.
