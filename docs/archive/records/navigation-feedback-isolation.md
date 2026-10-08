# Supported-hover navigation-feedback isolation

Date: 2026-09-28. Scope: ADR 0033, exactly one new scientific flight compared
with the saved aligned seed47001 baseline. Production behavior is unchanged.

## Audit and frozen protocol

Audit PR 31 merge `b8ee6b25c2023e5f026ac6c04e551c76f966c2df`, implementation
`c8b5c1c3ba0e432e9a258fbacf48352523adab68`, tree
`0af8ad0eea57a6f03428c85916dde27f16119eb1`. Local main is clean and equals
the live GitHub main reference. [PR 31](https://github.com/Gayles9/robust-quadrotor/pull/31)
records 3,779 hosted tests in 387.24 s and the exact tested/merged tree.

Fresh warning-strict diagnosis, earlier-oracle, support and documentation tests
pass 60 tests in 8.43 s. The preceding bundle independently authenticates 30
payloads and reconstructs all six full response histories and scores. Source,
published derivation and next-step scope agree. No preceding defect or new
evidence damage is demonstrated.

[ADR 0033](../../decisions/0033-navigation-feedback-isolation.md) is frozen before
implementation or execution, SHA-256
`ef331171b7ec2e658247f64af2b1d59c34f831a18b6b7a6bb01573b795b67b7e`.
The original campaign report is
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
The previous diagnostic report is
`949375db59e656659f8c7378060094b427a2958c1a17ccbc211cff0fa1efe928`.
The new runner authenticates all original campaign references and recomputes all
34 scores, then reconstructs the selected supported acquisition, release,
configuration and full baseline history. The baseline is never rerun as a flight.

## Implementation and checks

`experiments/navigation_feedback_oracle.py` wraps the existing simulation
observer and replaces only the arguments at `compute_position_control`.
The observer returns the identical estimated tuple for guards, completion and
inner feedback. The adapter saves all 775 outer-input records, including epoch,
original estimated position/velocity and supplied true position/velocity.
Process-local patches restore on normal and exceptional exit; no mission threads
are used concurrently in that process. The supported first measurement adapter
and original sensor/plant/controller code remain unchanged.

The verifier replays the baseline and oracle ESKF, full covariances/events,
health/supervision, estimated guard/completion logic and command dataflow. Only
the documented outer arguments are replaced during oracle command reconstruction.
Every saved chunk is authenticated and decoded before scoring. The independent
named-stream check reconstructs actual noise after removing each physical signal,
using the supported first acceleration sample correctly. Bias draws agree exactly.

Thirteen new tests cover the original observer tuple, untouched inner/guard
inputs, normal/exceptional patch restoration, wrong input rejection, a distinct
seed47821 smoke pair, saved history parity/replay, independent command checks,
four trace-tampering cases, changed noise/bias rejection and strict comparison
conditions. Initial collection fails because the new module does not yet exist.
All 13 pass in 4.16 s after implementation. One typing error requires narrowing
the already-checked optional release value before `asdict`; it is corrected
before full execution. The combined relevant suite passes 52 tests in 9.52 s.
The complete warning-strict suite passes **3,792 tests in 668.78 s**. Documentation
validation checks 130 Markdown files, 575 links and 23 Python blocks; Ruff lint
and formatting (301 files), mypy (89 source files), and whitespace checks pass.

One full oracle then executes successfully. Every plant/motor reconstruction
residual is zero. Maximum independently reconstructed named-noise error is
6.970119e-15, below 1e-12. A separate `--verify` completes full saved baseline
and oracle replay without another flight. No valid result is repeated, replaced,
cropped or rescored with new thresholds.

## Results

| Metric | Original aligned baseline | Outer-navigation oracle |
| --- | ---: | ---: |
| Hover peak, cm | 10.180776 | 1.989620 |
| Whole-flight RMSE, cm | 6.489135 | 3.925627 |
| Final position error, cm | 1.217038 | 0.744815 |
| Final true speed, cm/s | 2.135696 | 0.136219 |
| Duration, s | 15.5 | 15.5 |
| Terminal state | COMPLETE | COMPLETE |
| Initial axis-estimation error, degrees | 0.032469 | 0.032469 |
| First-second axis-error peak, degrees | 0.034121 | 0.034095 |
| Full axis-error peak, degrees | 0.133470 | 0.133861 |

Hover peak falls **80.4571%** and whole-flight RMSE **39.5046%**. The original
8 cm hover and 15 cm RMSE/final-position limits and 15 cm/s final-speed limit all
pass for the oracle. Full scoring includes flight zero and the original
inclusive 5..11 s hover window. The peak occurs at 5.76 s with NED error
`[0.0138982002733, -0.0140847414092, 0.0020782161550] m`.
The slight full-history attitude-error regression is explicitly retained.

Common-prefix random-draw maximum discrepancies are 7.105427e-15 for the
accelerometer, 1.387779e-17 for the gyro and 2.220446e-16 for slow observations.
The original covariance, state, sample and bias initialization match exactly.
Raw measurements and later filter trajectories appropriately change with motion.

**The experiment and the frozen useful-headroom gate both succeed.** The
improvement occurs with the original cascade and nearly unchanged attitude-error
scale. This isolates a useful target in combined outer position/velocity feedback;
it does not separate p-only from v-only effects or demonstrate a filter defect.
Truth-assisted feedback remains diagnostic. Normal mission integration and the
actual 10.18 cm hover failure stay open, as does the separate mass requirement.

## Next candidate

The [guide](../../results/navigation-feedback-isolation.md) defines a supported zero-velocity
prior as the next bounded candidate. Genuine world-stationary support is already
required through release, but the navigation prior still carries 0.03 m/s initial
velocity deviation from the old free-flight configuration. Conditioning that
uncertainty on valid support uses available boundary information; it is not an
in-flight truth replacement or a gravity/velocity observation inferred from a
quiet IMU. The initial mean is already zero.

Derive and check the covariance/rank and endpoint contract, then implement and
evaluate one opt-in experiment candidate under a separately frozen protocol.
Exact zero velocity is a modeled fixture condition; a hardware procedure must
justify its own residual uncertainty. Preserve position and alignment/bias
uncertainties, reject invalid/stale support and retain all original flight limits.
Include the support-to-flight force discontinuity and endpoint integration error
in uncertainty checks: exact initial velocity does not guarantee exact first-step
propagation. Reject an overconfident candidate.
Do not choose an arbitrary covariance floor or tune Q/R/gains against this seed.
Use all three known hover seeds and existing clean/fault conditions as regression;
fresh validation remains necessary before ordinary integration. The oracle does
not prove this one-time velocity constraint will reproduce its 80.46% benefit.

## Commands and evidence

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_residual_hover_diagnostic.py tests/unit/test_early_flight_diagnostic.py tests/unit/test_supported_start.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python PRIOR_DIAGNOSIS/verify_evidence.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_navigation_feedback_oracle.py tests/unit/test_early_flight_diagnostic.py tests/unit/test_supported_start.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.navigation_feedback_oracle --campaign SAVED_INDEPENDENT/campaign --output NEW_ORACLE
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.navigation_feedback_oracle --campaign SAVED_INDEPENDENT/campaign --verify NEW_ORACLE
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python ORACLE_BUNDLE/verify_evidence.py
uv run python scripts/check_docs.py
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q
```

Execution uses the existing `.venv/bin/python`, Ruff and mypy directly; the
commands above are equivalent in the pinned environment. No production source,
dependency, tool version or system configuration changes. Executed source digest:
`474ffd9f8fb7c714a0655fa03a93e30c84382ef6d5a977f8e16a7950acd1c56e`.
New report SHA-256:
`231648423ac0dc130534f05e8d04eef2646430e5bea054fce536cc0ea3ff315f`.

The evidence bundle retains both complete selected raw histories and covariance
chunks, original support/release/configuration, all new diagnostic/outer-input
records, frozen protocol, original campaign report, paired PNG/PDF figure,
plotting source, execution/replay logs and a NumPy-only independent verifier.
That verifier independently binds the original baseline to its fixed report,
recomputes both full scores and requested outer commands, validates all oracle
input epochs, checks initial covariance/sample parity and paired noise draws,
and reconstructs the headroom decision. Full plots are visually inspected.
Generated evidence stays outside Git; publication identities and hosted CI are
recorded in the publishing PR.

The final independent verifier authenticates 26 payloads and passes both full
scores, outer-command histories, terminal ownership, input trace and paired-noise
checks. Archive `quadrotor-navigation-feedback-isolation-2026-09-28.zip` contains
27 members, 21,038,900 bytes, SHA-256
`84ddc1595d27b7c81b580b71d377552834409c36b49b7a2e4fd4e80b410142e1`.
Every archive member is compared byte-for-byte by SHA-256 with its source before
publication.
