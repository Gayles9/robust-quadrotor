# Repository audit and feedback closeout — 2026-09-25

## Audited state and correctness

Main: `f45f610776ec1c6877e896d7f63ec17c87f1d322`.
Estimated-feedback draft: `cad64cf37d945fcf2f2186d070e8f845d19edf25`.
All 179 checked-out files match GitHub blob hashes; the local source tree equals
remote tree `49d95760ad2306195a7c73f8a9883661b5c4cd6b` before edits.

Reviewed the causal online estimator, estimate-to-controller boundary, slow
sensor delivery, mission clocks/guards, independent truth baseline and scoring.
The existing checks include independent offline replay, command reconstruction,
truth-isolation and physical/numerical tests. Fresh baseline command:

`OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check`

Result: **3,153 passed in 335.18 s**, Ruff lint passed, 172 files formatted,
mypy passed on 52 source files. Python 3.12.14, uv 0.12.3 and the existing lockfile
were used. No new production filter/controller defect was demonstrated. The
previous aborted-hover serialization defect is already fixed in main via PR #11.
This is a bounded source-and-regression audit, not a proof of all code paths.

## Cleanup

- Replace the 138,191-byte front page with a concise project overview and current
  boundaries. Preserve its substantive plant/sensor/run guide in `docs/foundations.md`;
  remove duplicated status histories and point to their dated technical records.
- Add a documentation map and repair stale current-status/next-step statements.
- Remove the unused initial `vectors.squared_norm` scaffold and its sole test.
  Repository-wide reference search found no production/experiment callers. Its
  dated learning record remains in history; no active numerical path changes.
- Keep test coverage, lockfile, CI configuration and reproducible negative
  feedback experiments. Failed experiments are evidence, not unused controllers
  silently promoted into production. No generated histories or logs enter Git.
- Reviewed all 12 remote branches: main, the active feedback draft and ten prior
  feature/fix branches. Historical branch refs are retained; commit history is not
  rewritten or deleted as part of a content cleanup.

## Same-case baseline selection

I decided to retain the best existing reference if the final idea did not
improve it. The last recoverable session note reports that the subsequent
startup-readiness attempt timed out at 12 s in all six cases. Its implementation
is absent from the published tree; this audit neither reproduces nor promotes it.
The unfinished baseline comparison is completed below using published code.

`experiments.feedback_baseline_comparison` runs **both existing profiles** on the
same six already-observed full hover missions. Gains, prior, sensors, limits and
the original 5..65-s scoring interval are unchanged. No fresh seed is opened.
Selection preference was stated before execution: lower worst and mean hold peak,
with control effort reported. This comparison is development evidence, not a
new held-out qualification or an estimate of general failure probability.

| Observed seed | Version 1 peak [cm] | Version 2 peak [cm] |
| --- | ---: | ---: |
| 30 | 7.4774 | 6.2139 |
| 91001 | 8.1394 | 6.5987 |
| 93003 | 10.4950 | 9.1403 |
| 93012 | 8.3814 | 8.8686 |
| 8200 | 5.6763 | 5.9166 |
| 8201 | 5.1326 | 4.7356 |
| Worst | **10.4950** | **9.1403** |
| Mean | **7.5504** | **6.9123** |

Both profiles complete all six missions with no actuator limiting. Version 1
passes the existing mission conditions in 3/6 cases; version 2 in 4/6. Mean
integrated squared actual moment is 0.01221190 versus 0.02519988 N² m² s, about
2.06 times higher for version 2. This is an effort proxy, not electrical energy.
All twelve saved compact histories pass independent SHA-256 checks and exact
recomputation of hold peaks and effort. Every hold has 24,001 samples. The common
historical cases reproduce their recorded metrics exactly.

**Decision: retain explicit `design_version=2` as the working numerical reference**
because it lowers worst and mean tracking error on this comparison. It is not
uniformly better on every seed and retains its original hover limitation. Version
1 remains reproducible; its historical default argument and protocol hashes are
unchanged. Select version 2 explicitly in experiments.

Original fresh outcomes remain 29/30 and 28/30. No scoring threshold or window is
weakened, and neither profile is declared qualified. Accepting this limited
baseline permits the independent planner milestone; it does not close the
estimated-feedback performance requirement or authorize more tuning.

## Reproduction and next work

```bash
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.feedback_baseline_comparison \
  --workers 3 --output /tmp/NEW_baseline_comparison
```

The compact comparison traces retain time, truth/reference position, actual
moment and limiting flags. Complete covariance/replay evidence remains in the
original frozen campaigns. The comparison does not replace those archives.

[ADR 0015](../../decisions/0015-minimum-snap-trajectory.md) freezes the next package:
fixed-duration minimum-snap position splines with independent optimization and
constraint checks. Feasibility/time allocation and mission integration follow
separately. The final gate is recorded in the
[minimum-snap closeout](minimum-snap.md).
