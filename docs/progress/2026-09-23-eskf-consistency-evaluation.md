# ESKF Consistency Evaluation and Frozen Nominal Ensemble

- Date: 2026-09-23
- Status: Implemented, audited, locally verified, published code and hosted code CI verified
- Baseline: [`d5a073f4f98ed2fe8be32f840c0bb43a206ad321`](https://github.com/Gayles9/robust-quadrotor/commit/d5a073f4f98ed2fe8be32f840c0bb43a206ad321)
- Code commit: [`b4e5719e074f2bbb90e0c6229abb86c0de17228a`](https://github.com/Gayles9/robust-quadrotor/commit/b4e5719e074f2bbb90e0c6229abb86c0de17228a)
- Tested/published code tree: `1490d24319fd0d8ac3b4a26a6b8c3586084c7ea0`
- Review: [pull request #4](https://github.com/Gayles9/robust-quadrotor/pull/4)
- Hosted code CI: [run 35896788372](https://github.com/Gayles9/robust-quadrotor/actions/runs/35896788372), successful
- Contract: [ADR 0008](../decisions/0008-eskf-consistency-evaluation.md)

## Completed scope and frozen plan

The preceding gating milestone identified aligned consistency evaluation as the next
bounded estimator step. The current main revision and all 93 tracked blobs were verified
before editing; the fresh baseline passed all 2,157 tests. Current code, tests, ADRs and
the README took precedence over older project-state summaries. The master plan supplies
broader validation targets, not evidence that unimplemented capabilities exist.

The implementation plan fixed these boundaries before executing the new experiment:

1. Pair completed truth/bias rows exactly with post-update replay epochs; extract the
   existing right-local 15-state error and require positive-definite covariance for NEES.
2. Derive an explicit sampled-IMU-to-continuous-noise conversion, retaining the prediction
   core's first-order discretization. Account for the first pre-acquisition bias step.
3. Calculate central95 chi-square intervals and pointwise independent-seed means, with
   no temporal pooling or acceptance-conditioned NIS summaries.
4. Freeze two nominal cases, all physical/noise/prior parameters, independent prior
   randomness, disjoint seed partitions, horizon, divergence limits and failure policy.
5. Implement focused tests and the evaluation runner; execute smoke/development and
   then the fixed validation partition without fitting parameters to its results.
6. Audit numerical/statistical contracts, preserve old source/test bytes, run all checks,
   publish the exact tested tree, review hosted CI, and complete the technical documentation.

`eskf_consistency.py` contains right-local error extraction, scaled-Cholesky NEES,
owned reference/result histories, the truth-only adapter and explicit noise conversion.
`consistency_statistics.py` contains bounded central95 chi-square quantiles and ensemble
statistics. `experiments/eskf_consistency.py` composes generation, independent priors,
measurement-only fused/dead-reckoning replay, then evaluation and finite JSON reporting.
The full equations, interfaces, units and frozen parameters are in ADR 0008.

## Verification strategy and audit

| Boundary | Independent evidence |
| --- | --- |
| Right-local geometry | Injection/extraction inversion; independent relative-matrix log; nonidentity frame/axis checks; tiny angles; both sides of π; exact-π tie; all quaternion sign pairs |
| NEES | Diagonal analytical values; independent correlated solves; extreme consistent unit scaling; zero-error cases; seeded Gaussian moments |
| Covariance domain | Reject nonfinite, asymmetric, singular, indefinite, wrong-shape and non-real input; reject unrepresentable score; no hidden regularization |
| Reference history | Exact epoch equality; row-one offset; actual bias-row correspondence; all consumed truth rows revalidated; sensor-access traps; independent read-only ownership |
| Noise conversion | Per-sample velocity/attitude increment variance; continuous bias-density scaling; positive interval; underflow/overflow rejection |
| Statistical interval | Exact two-degree exponential quantiles; NIST rounded entries; large-DOF independent SciPy oracle; invalid integer/type domain |
| Ensemble semantics | Seed-axis means; unchanged confidence width when time columns repeat; inclusive coverage bounds; large finite means; no empty/nonfinite/negative samples |
| Nominal model | Independent analytic position and scalar-first yaw quaternion for both generated cases |
| Prior and truth isolation | Explicit independent Gaussian draw; first-bias-step covariance; nominal-only access trap; truth history inaccessible until both replays finish |
| NIS accounting | Rejected scores retained; disabled scores excluded explicitly and counted; exact common-epoch alignment required |
| Failures and reproducibility | Failed seeds/stages retained; singular NEES reported as failure; finite divergent runs retained; byte-identical repeated numerical reports; exclusive output; source-change detection |

There are **201 new cases**: 152 geometric/covariance/reference/noise tests, 28 statistics
tests, and 21 experiment/integration tests. The previous 2,157 cases remain unchanged,
giving **2,358 total tests**. The full suite passes both normally and with warnings
treated as errors. No tolerances in existing tests were changed.

An independent SciPy 1.17.0 audit compared both central95 quantiles at degrees
1, 2, 3, 15, 30, 100, 101, 300, 1500, 14999 and 15000. The largest relative difference
was approximately **1.05e-12**, within the predeclared 2e-11 audit tolerance. SciPy was
used only in a separate audit environment; the project, tests and experiment need only
the existing NumPy dependency and Python standard library.

All 93 baseline blobs are unchanged in the code commit, which adds exactly six files.
Documentation then changes six existing Markdown files and adds the ADR and this record.
The plant, ESKF prediction/correction/replay/gating modules, existing tests, lockfile,
CI, package configuration, random streams, manifest versions and artifact schema remain
byte-identical. GitHub's six added blobs and complete code tree match the local verified
hashes. This establishes compatibility of the existing implementation; it does not assert
universal bit reproducibility across different software versions or hardware.

## Frozen campaigns and provenance

The full cases use 201 truth steps of .01 s: truth spans 0..2.01 s and replay spans
.01..2.01 s. Both cases use the same declared physical/noise model, with independently
sampled priors and diagnostics-only fusion. Each full trial has 201 NEES epochs,
40 position scores and 20 altitude scores. The validator independently checked all
seed IDs, epoch arrays, score denominators and reported means/coverage from raw scores.

| Partition | Seeds per case | Trials across two cases | Numerical failures | Fused/dead divergence flags |
| --- | --- | --- | --- | --- |
| Smoke | 0, 1 | 4, shortened to 21 truth steps | 0 | 0 / 0 |
| Development | 1000..1009 | 20 | 0 | 0 / 0 |
| Validation | 20000..20099 | 200 | 0 | 0 / 0 |

The protocol hashes are:

| Partition | Canonical protocol SHA-256 |
| --- | --- |
| Smoke | `dff8b6f9c71dcbcb35166b58a93cc2904c07a9230da70eed8a06f903b9467e24` |
| Development | `a3c03690c07ddd4800a6cc15f58a894e17a1c639964aed4842188a6865da33d1` |
| Validation | `4f86502b06b120289ea4bcc6027fd76648370d6cbd74c202babd05dfbcbc30ad` |

The validation report completed at **2026-09-23T17:32:39.511178+00:00**. It honestly
records the baseline HEAD with `git_worktree_clean=false`, because the evaluated new
code had not yet been published. Its source/configuration digest is
`0832c28d2bceef146115b9612200585dea8a4ac6f5578cd554f840e7ed20b877`.
Those exact source bytes were then published as the code commit above. A separately
materialized clean checkout of that published commit, installed with the locked
environment, reproduced the smoke report's numerical content exactly, with
`git_worktree_clean=true` and the same source digest as validation. Provenance and
generation timestamps were compared separately from numerical content.

## Validation results and interpretation

Each row below uses all 100 planned seeds for that case. Coverage is the descriptive
fraction of individual scores inside their central95 interval, averaged over the
reported epochs. It is not a confidence interval based on 20,100 independent states.
Position and altitude degrees of freedom are 3 and 1; full-state NEES uses 15.

| Case | Mean NEES | NEES coverage | Mean position NIS | Position NIS coverage | Mean altitude NIS | Altitude NIS coverage |
| --- | --- | --- | --- | --- | --- | --- |
| Hover | 15.326469 | 96.219% | 3.029228 | 95.125% | 1.006163 | 95.850% |
| Translating yaw | 14.828364 | 95.537% | 3.031717 | 95.050% | 1.010659 | 95.050% |

All six descriptive coverages lie within the frozen .90–.98 investigation band. This
is bounded agreement with the reference distribution under the stated nominal model.
The fraction of epoch means inside the **pointwise independent-seed** interval is
100% for NEES, 97.5% for position NIS and 95% for altitude NIS, in both cases.
Those fractions are descriptive too; no simultaneous or temporal-independence claim
is made. No observation was rejected because the protocol deliberately had no gate.

Position RMSE is the time RMS of the three-dimensional position-error norm. The table
shows the distribution of that quantity across seeds; reduction compares the two
**means of per-trial RMSE**, not a pooled time/seed RMSE or a claim for every seed.

| Case | Fused mean RMSE (m) | Dead-reckoning mean RMSE (m) | Mean reduction | Fused median (m) | Fused p95 (m) | Fused maximum (m) |
| --- | --- | --- | --- | --- | --- | --- |
| Hover | 0.106205 | 0.399326 | 73.404% | 0.104798 | 0.141511 | 0.170695 |
| Translating yaw | 0.107411 | 0.393330 | 72.692% | 0.107884 | 0.137861 | 0.155026 |

The zero-failure/100-seed and >50% mean position-improvement targets are therefore met
for these declared short nominal cases. This does not close the master plan's broader
held-out trajectory, fault, excitation or robustness requirements.

Mean velocity RMSE changes from 0.230927 to 0.160165 m/s for hover and from 0.224927 to
0.156106 m/s for translating yaw. Mean attitude RMSE changes from 0.017362 to 0.016398 rad
and from 0.015263 to 0.014483 rad, respectively. Mean accelerometer-bias RMSE improves
only slightly (0.031483 to 0.031260 m/s²; 0.030832 to 0.030741 m/s²), and gyroscope-bias
RMSE is essentially unchanged (hover is slightly higher after fusion). These limited-
excitation, two-second replays provide no claim of broad bias convergence or observability.
The report retains per-trial final bias norms for further analysis without selecting
successful cases after the fact.

The smaller development partition produced descriptive NEES coverage of **99.303%** for
hover and **100%** for translating yaw, outside the frozen investigation band. This finding
was retained, not tuned away. At the first epoch, 9/10 hover and 10/10 translating-yaw
errors were already inside the individual band; later epochs share those initial errors
and slowly observed bias/attitude components. Thus time samples cannot turn ten seeds
into thousands of independent coverage trials. The unchanged 100-seed validation result
provides broader nominal evidence. It does not prove a unique cause for the development
overcoverage or eliminate the documented first-order/nonlinear covariance approximations.

## Exact verification record

All commands below ran on **2026-09-23**. The local environment was Python 3.12.14,
uv 0.12.3, NumPy 2.5.2, pytest 9.1.1, Ruff 0.16.2 and mypy 1.20.2.
Except for the baseline row, tests exercise the source/test bytes published as
`b4e5719e074f2bbb90e0c6229abb86c0de17228a`.

| Command | Verified outcome |
| --- | --- |
| `uvx --from uv==0.12.3 uv run make check` at baseline | 2,157 passed; Ruff/formatting/mypy passed |
| `uvx --from uv==0.12.3 uv run make check` on implementation | **2,358 passed** in 63.90 s; Ruff clean; 93 formatted files; mypy clean over 25 source files |
| `uvx --from uv==0.12.3 uv run pytest -W error -q` | **2,358 passed** in 48.63 s; no warnings |
| `uvx --from uv==0.12.3 uv run pytest -q -W error tests/unit/test_eskf_consistency.py tests/unit/test_consistency_statistics.py tests/unit/test_eskf_consistency_experiment.py` | **201 passed** |
| `uvx --from uv==0.12.3 uv run pytest --collect-only -q tests/unit/test_eskf_consistency.py tests/unit/test_consistency_statistics.py tests/unit/test_eskf_consistency_experiment.py` | 152 + 28 + 21 = **201 cases** |
| `uvx --from uv==0.12.3 uv run python experiments/eskf_consistency.py --partition smoke --output ../evidence/eskf-consistency/smoke.json` | Exit 0; four complete trials |
| `uvx --from uv==0.12.3 uv run python experiments/eskf_consistency.py --partition development --output ../evidence/eskf-consistency/development.json` | Exit 0; 20 complete trials |
| `uvx --from uv==0.12.3 uv run python experiments/eskf_consistency.py --partition validation --output ../evidence/eskf-consistency/validation.json` | Exit 0; 200 complete trials; unchanged frozen protocol |
| `uvx --from uv==0.12.3 uv run --locked python experiments/eskf_consistency.py --partition smoke --output ../published-smoke.json` in the clean published checkout | Exit 0; exact numerical agreement with prepublication smoke; source digest matches validation |
| `GIT_INDEX_FILE=/tmp/quadrotor-audit/evidence/eskf-consistency/code.index git diff --cached --check d53dcf32ea534f22babd19235da1de2053f38d26` | Clean isolated code-publication diff |
| Hosted `uv sync --locked`, then `make check`, run 35896788372 | Successful; **2,358 passed** in 68.09 s; Ruff clean; 93 formatted files; mypy clean over 25 source files |

The launcher and output paths identify this audit environment. Normal checkout usage is
`uv sync --locked` followed by the README commands and any unused output path. Durations
vary by host. Ruff's formatted-file count includes checked Markdown and is not a Python
module count. Generated JSON and large logs are intentionally not committed; the frozen
runner reconstructs them with complete configuration/provenance records.

Documentation closeout repeated `uvx --from uv==0.12.3 uv run make check`: **2,358 passed**
in 53.63 s, Ruff passed, **95 files** were already formatted and mypy passed over **25 source
files**. The eight documentation files have balanced code fences, 55 valid relative links
and no unresolved drafting markers. All six published source/test blob hashes remained
unchanged through closeout. The three new README study commands correspond to the executed
smoke, development and validation runs above; only output locations differ.

## Remaining boundary and next step

This completes the agreed nominal consistency-evaluation increment. It does not add a
controller, production fault generator, delayed-state correction, asynchronous IMU,
resumable estimator format, adaptive covariance/gating, or flight integration. First-order
process covariance remains an approximation. Validation seeds are held out; the two
trajectory families are intentionally fixed and short. G2 and full G3 remain open.

The next bounded evaluation increment is a **frozen observation-fault campaign** over
the measurement-only interface. Specify deterministic corruption and missing-observation
fixtures, unchanged baseline pairing, fixed gate policies, disjoint seed partitions,
fault labels and recovery/error/rejection/failure metrics before evaluation. Preserve
all trials, including numerical failures, and do not tune gates on held-out results.
Delayed fusion, stronger bias excitation and model-mismatch studies should retain their
own explicit contracts; none is implied complete by this nominal campaign.
