# 2026-09-26: Published baseline and geometric source recovery audit

This is a historical recovery record. The source blocker was subsequently
resolved by the [rebuilt implementation](2026-09-26-geometric-reimplementation.md)
and the [completed tuning audit](2026-09-26-geometric-project-audit.md).

## Outcome and scope

I audited the published baseline and retained evidence before resuming geometric
controller tuning. At that point the later implementation was unavailable in
the accessible workspaces and had never been published. Source recovery was
required before further tuning; this did not establish a controller performance
limit. No gains, thresholds, scoring
windows, production algorithms or tests were changed in this audit.

Audited main: `4ba53489dc02b0cef1035df9abc466d4308916b4`.
Its 196 files were recovered through the authenticated GitHub connection and
checked against their Git blob hashes. The reconstructed tree is exactly
`985e107145cc77fceb956efad04ad56df2e9349d`; the signed merge commit also matches.
Existing push CI [36146939424](https://github.com/Gayles9/robust-quadrotor/actions/runs/36146939424)
passed on that exact main commit.

## Recovery findings

The retained master engineering log ends with the September 25 filtered-reference
package, section 28. It records geometric work as uncommitted on
`codex/geometric-tracking-2026-09-25`, based on
`03e9f02ffc94c3fc3c0fe3d8b0f0c0e795f414a1`. That branch is absent remotely.
All 14 current remote branch trees were inspected; none contains the geometric
modules. No open pull request was present when recovery began. The previously
recorded scratch checkout no longer contains project files in this environment;
The separate WSL checkout was not available to this audit.

Three retained evidence ZIPs were inspected in full. Each explicitly excludes
the controller source. They retain numerical histories, protocols, reports,
diagnostic/verification scripts and checksums. All archive integrity checks and
**82 payload SHA-256 checks** pass. Verification scripts depend on the missing
project modules and cannot restore them.

A later September 25 chat reports geometric ESKF integration, 3,462 passing
software tests, and failed qualification due to a 29.9 cm startup peak, wind
regression and high effort. It also reports unsuccessful evidence/log saving
and no commit or push. These are recovered conversation claims: the integration
source and its raw campaign outputs were not available for fresh verification.
They are not promoted into current reproducible results.

## Retained evidence independently rescored

A standalone audit imports no geometric-controller or experiment implementation.
It checks ZIP and per-history hashes, finite arrays, monotonic timestamps,
completion, inner/outer limiting flags, strict rotor bounds, and recomputes
time-integrated position RMSE, peak/final error, final speed and squared-moment
effort directly from the arrays. All **33 recorded entries, representing 26
distinct saved histories**, match their reported metrics to relative tolerance
1e-12 and absolute tolerance 1e-14. These are saved flights, not new simulations.

The seven retained filtered true-state flights satisfy the original 15 cm RMSE,
25 cm peak and 8 cm / 8 cm/s terminal bounds, with no limiting. They retain their
per-case RMSE advantage over cascade and the <=1.10 raw-causal RMSE-ratio guard.
The original five-case mean RMSE ratio to cascade remains 0.3273714380.

| Retained filtered case | RMSE (cm) | Peak (cm) | Final error (cm) |
| --- | ---: | ---: | ---: |
| Nominal | 0.202813 | 0.595374 | 0.179699 |
| Half plant step | 0.202813 | 0.595374 | 0.179699 |
| Positive offset | 1.017519 | 3.297607 | 0.179699 |
| Negative offset | 1.012425 | 3.287545 | 0.179699 |
| Mild wind | 3.569739 | 7.506940 | 2.715589 |
| Reversed wind | 3.665280 | 7.643063 | 2.720627 |
| Wind and offset | 3.749177 | 7.506926 | 2.715589 |

The original nominal-model wind comparison remains a failure: 6.769758 cm RMSE
versus 3.924654 cm cascade. The causal correction and derivative filter improved
that retained true-state case. This does not establish noisy-estimator mission
qualification, robust performance over an uncertainty distribution, or hardware
readiness. Original estimated-feedback 29/30 and 28/30 campaigns and the unmet
8 cm full-hold condition remain unchanged.

## Published-code audit and checks

Reviewed the published estimate-to-controller boundary, sensor delivery/event
order, distinct truth safety guards, controller signs/limits, mission preflight,
terminal recording and trajectory scoring, together with their existing
regression checks. No new production defect was demonstrated by this review.
The pre-edit documentation scan found no broken targets among 194 local links.
The whole-repository warning-as-error gate is recorded in the session evidence.

```bash
uv sync --locked
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
git diff --check
```

The pinned uv 0.12.3 is installed in an isolated task directory and prepended to
PATH; the repository's tool version and lockfile are unchanged. Python 3.12.14,
NumPy 2.5.2, Ruff 0.16.2, mypy 1.20.2 and pytest 9.1.1 are used.

Fresh full-suite result: **3,228 tests passed in 288.27 s**, with warnings as
errors. Ruff lint passed, all 189 discovered Python files were already formatted,
and strict mypy passed on 56 source files. All executable source, tests,
configuration and lockfile bytes remain identical to the audited main commit.
This is a bounded review and regression audit, not proof of every code path.

## Next exact action

Recover the uncommitted geometric checkout or a source patch from the preceding
session, including controller, causal/filtered reference stages, ESKF mission
composition, experiment protocols and tests. Check its identity against retained
source fingerprints before running it. If recovery is impossible, a replacement
is a new implementation requiring fresh verification; it cannot inherit the
missing implementation's pass counts or qualification claims.

Once source is available, audit it, freeze a bounded tuning campaign against the
original startup, wind, effort and hover criteria, and retain every attempt.
The intended follow-up was to publish the tested implementation and its results.
This audit makes no claim that tuning is complete or that further improvement
is impossible.
