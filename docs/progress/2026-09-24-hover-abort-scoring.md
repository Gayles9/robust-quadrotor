# Hover-abort scoring audit — 2026-09-24

Audited main commit: `23197e95feace859c4d7405c4e2574e06f2c33bb`.

## Finding and bounded correction

The hover scorer combines a NumPy time comparison with Python short-circuit
`and`. If the run stops before 65 seconds, the expression returns the first
operand, a NumPy boolean. The finite canonical JSON encoder cannot serialize
that scalar. Consequently a valid early-abort history can prevent publication
of its failed trial record. The defect was exposed and reproduced during the
separate estimated-feedback design audit; the affected scoring expression is
also present in the merged baseline recorded above.

`score_case` now explicitly converts the complete `60_second_hover` condition
to a Python `bool`. The required duration, full scoring window and .08 m target
are unchanged. This changes neither the controller nor the simulated dynamics.
Existing completed-hover metrics retain their JSON representation.

Two new regression cases construct actual geofence aborts: one at initialization
with no commands and one after the first 2.5 ms plant step. Each verifies the
recorded abort, a false hover condition, a failed overall result, native boolean
conditions and successful canonical JSON encoding. Both reproduced the defect
before the correction. This checks the failure path without weakening a
successful-flight criterion.

## Fresh verification

Verification date: **2026-09-24 UTC**. Python 3.12.14, uv 0.12.3 and the unchanged
lockfile were used in an isolated checkout of the recorded main commit plus this
correction. The exact source tree is preserved by the correction's Git commit.

| Command or gate | Observed result |
| --- | --- |
| `uv sync --locked` | Successful; no dependency or lockfile change |
| `PYTEST_ADDOPTS='-W error' make check` | Successful; **2,861 tests passed in 112.99 s** |
| Ruff lint and format, within `make check` | Passed; 139 files already formatted |
| mypy, within `make check` | No issues in 41 source files |
| Two abort regressions before correction | Both fail on the non-native boolean |

The full gate includes the existing position/attitude, mission, sensor, estimator,
reproducibility and evidence-validation regressions. Fresh GitHub CI and exact-tree
verification are required before this correction is merged.

## Qualification boundary

This correction is isolated from the unqualified estimated-feedback development
branch. That investigation retained all fixed, development and held-out results;
its two frozen controller profiles passed 29/30 and 28/30 fresh cases, respectively.
They did not meet the unchanged all-case hover qualification. A successful
software gate does not turn those performance misses into passes.

The detailed investigation remains with the estimated-feedback pull request.
Only the demonstrated scoring defect and its regression evidence are included
in this correction. No planner or new control architecture is introduced.
