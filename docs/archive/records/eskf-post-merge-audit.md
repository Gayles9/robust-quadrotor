# ESKF Completion: Post-Merge Technical Audit

Date: 2026-09-23 (UTC).

Audited baseline: `34f6c78732f30cefaeadf9e242ce76e286522844`, the merge of
[PR #5](https://github.com/Gayles9/robust-quadrotor/pull/5).

## Scope and conclusion

The review covered every new implementation module in PR #5, its tests, package/CI
configuration and documentation, plus the adjacent prediction, correction, replay and
consistency contracts. It checked the analytic position/rotation derivatives, NED/FRD
signs, right-local attitude errors and resets, bias timing, sampled-noise conversion,
fault ownership and source identities, Monte Carlo independence, failure accounting,
provenance and plotting. The new scalar/position-velocity examples and dependency boundary
were also reviewed. This is an automated source audit and numerical verification record,
not an assertion of independent human review or exhaustive proof.

Corrections are confined to experiment/report handling, regression tests and documentation.
No ESKF, dynamics, sensor, scheduling, configuration or artifact-schema production module
changes. The existing first-order propagation, Q/R values, gate thresholds, random streams,
seed partitions and frozen protocol hashes remain unchanged. No controller or integration
scope is added. The measured full-state NEES limitation remains open.

## Findings and corrections

| Finding | Consequence | Correction and evidence |
| --- | --- | --- |
| Duplicate `(family, seed)` records were accepted as independent trials | An ensemble could count a repeated realization twice and narrow the seed-mean reference interval incorrectly | Reject duplicate identities before summarization; regression uses two copies of one seed |
| Group `planned_count` came only from supplied records | A truncated trial list or missing variant could appear complete | Reconcile all job identities and variants with the declared protocol, including failed records |
| Plot input checked only the report version and finite JSON | Modified protocol metadata, counters, assessments or stale aggregate values could be plotted as evidence | Verify the frozen protocol/hash and reconstruct all groups, failure/divergence counts and assessments before creating output |
| Protocol metadata returned the mutable `SENSITIVITY` table directly | Editing a returned metadata object could change later experiment multipliers | Return a separate mapping; regression verifies that metadata mutation cannot change experiment behavior |
| Any name beginning `fault_` was accepted | A typo could silently run the ungated fault variant | Validate the exact supported variant names before replay |
| Dropout indices and NEES plot clocks assumed 100 Hz | A valid 20 Hz diagnostic could index outside its history or attach wrong times | Store actual epochs and select the last epoch at/before each declared diagnostic time; test against independently selected covariance rows |
| Fault shading described acquisition windows as delivery windows and extended smoke axes into absent future intervals | Plot timing could be misread | Label delayed acquisitions, clip shading to the recorded horizon and use actual observation timestamps |
| Undefined precision/recall produced empty, unlabeled bars | An unavailable ratio could be mistaken for zero | Show `N/A` explicitly and keep sensor/metric colors stable; inspect the short-horizon figure |

`tests/unit/test_eskf_validation_audit.py` adds 22 regression cases. The existing
incomplete-ensemble plot test now constructs actual failed trial records and derives
their summaries, rather than supplying deliberately inconsistent aggregate fields.
It still verifies that failures suppress every affected complete-ensemble figure.

## Report compatibility and interpretation

The experiment report schema advances to version 2. Every completed variant includes
`time_s`; available dropout diagnostics include `dropout_time_s`, and aggregate dropout
summaries include `time_s`. Requested diagnostic instants are 7.9, 9.99 and 11.0 seconds.
For an off-grid diagnostic run, the recorded value is taken at the last available epoch
at or before each requested instant, and its actual timestamp is retained. Frozen-protocol
runs still use exactly 7.9, 9.99 and 11.0 seconds.

`validate_validation_report` returns an owned normalized view after checking the protocol,
complete trial identities, variant identities, clocks, summaries and assessment. Version 1
remains readable: its implicit clock is recovered only from the verified original protocol.
Existing explicit timestamps are checked rather than replaced. The input report and its
software/source provenance are not rewritten. This is a consistency check for the analysis
report; it is not an execution signature, estimator checkpoint or replacement run-artifact
schema. Recomputing aggregates cannot authenticate fabricated underlying measurements.

Failed trials remain present and suppress their complete-group ensembles. Finite divergent
trials remain included. Corrected plots continue to retain rejected NIS and the measured
NEES miss. Stored reports are checked under the locked analysis environment; figures record
the original input file hash.

## Numerical compatibility evidence

The original 100-trajectory, 280-variant held-out JSON has SHA-256
`d3c1a4f575fbd0cd107ac1f61625332446125879ffa6649f10ebc873083f09bc`.
Its complete groups and assessment recompute exactly. The report is accepted through the
validated version-1 compatibility path, preserving all original numerical evidence.

A fresh replay of development seed 4000 runs all eight excited-motion variants for 30 s:
nominal, dead reckoning, gated/ungated faults and the four Q/R sensitivity variants.
Every pre-existing reported numerical field matches the retained development trial exactly
after canonical JSON serialization. Only the newly explicit timestamps are additional.
This is a regression comparison using a previously evaluated development seed, not a new
held-out acceptance campaign. The full held-out ensemble was revalidated from stored trial
data rather than rerun or retuned.

Full-state NEES central95 coverage remains **88.836721%**, below the declared 90–98%
investigation band. None of the identified report-boundary defects occurred in the retained
original campaign. Their correction does not repair or relabel the covariance-calibration
finding. The [original completion record](eskf-completion.md) remains the source
for the measured performance, consistency investigation and limits of the first-order model.

## Verification and publication

All commands below ran on 2026-09-23 with Python 3.12.14, NumPy 2.5.2, Ruff 0.16.2,
mypy 1.20.2, pytest 9.1.1 and Matplotlib 3.11.2. The project remains pinned to uv 0.12.3.
The reviewed executable-source/configuration SHA-256 is
`8e634423d5c14dfbf1c43af5a7abde46acbcab52096538a435c1dffe8d2bf4f1`.
The fresh smoke records that digest, the audited baseline commit and a dirty working tree,
accurately identifying its pre-commit execution. Documentation is excluded from this
executable-source digest. Published commit/tree and hosted CI verification are recorded in
the accompanying pull request after publication.

Here `$EVIDENCE` denotes a new output directory outside Git. Python commands run from the
repository root; `.venv` contains the locked dependencies. The final local checks run the
same lint/format/type/test stages as `make check`, with warnings promoted to errors in pytest.

| Command or experiment entry point | Verified outcome |
| --- | --- |
| `.venv/bin/ruff check .` | Passed |
| `.venv/bin/ruff format --check .` | 110 files formatted |
| `.venv/bin/mypy src experiments` | No issues in 31 source files |
| `.venv/bin/pytest -W error` | **2449 passed in 58.05 s**; no failed/skipped tests or warnings |
| `.venv/bin/pytest tests/unit/test_eskf_validation_audit.py tests/unit/test_eskf_validation_experiment.py -W error -q` | 38 passed; included in the final full suite |
| `.venv/bin/python -m experiments.eskf_validation --partition smoke --workers 2 --output "$EVIDENCE/reviewed-smoke.json"` | All 16 variants complete; zero numerical failures or nominal/gated divergence; report schema 2 |
| `run_validation_trial` on the `planned_jobs("development")` entry for excited seed 4000, with `OPENBLAS_NUM_THREADS=1` | Eight 30-second variants; every pre-existing numerical field matches the retained development record |
| `validate_validation_report` on the original full held-out JSON | All 100 trial identities and 280 variants retained; groups and assessment exactly reproduce the saved evidence |
| `.venv/bin/python -m experiments.plot_eskf_validation --input "$EVIDENCE/validation.json" --output "$EVIDENCE/reviewed-validation-plots"` | Six figures from original schema-1 evidence; visual review preserves rejected NIS and out-of-band NEES |
| `.venv/bin/python -m experiments.plot_eskf_validation --input "$EVIDENCE/reviewed-smoke.json" --output "$EVIDENCE/reviewed-smoke-plots"` | Six figures from schema-2 evidence; short-horizon fault axes and undefined metrics visually checked |
| `git diff --check`; protected-file comparison; relative Markdown link checks | Clean diff; all production source, dependencies, CI, repository instructions and runtime schemas unchanged; links resolve |

The initial regression batch reproduced 18 failures before implementation; the final
audit test file contains 22 passing cases. Schema-1 conversion, duplicate/missing records,
changed protocol hashes, mismatched clocks, stale counters/assessments, failed ensembles,
worker determinism, numerical regression and headless rendering are covered. Original
raw results and their provenance remain intact. Generated JSON, plots and logs stay outside
Git as required by `AGENTS.md`.
