# Independent supported-start validation

Date: 2026-09-28. Scope: the single frozen ADR 0031 repeatability and
observation-fault campaign, with no production changes or post-result tuning.

## Preceding audit and publication

The preceding comparison is published and merged as
[PR 29](https://github.com/Gayles9/robust-quadrotor/pull/29), implementation
`72f7311a8b97425c837f0c507ac1fe6b4a6cf235`, merge
`70ef9fcc3bb479349b68c3a8ac705bbabcb1f01f`. Its merged tree exactly matches
the tested `63c59159445f6641a47a94e2c2cd74d6e6cb0b98`.
[Hosted CI](https://github.com/Gayles9/robust-quadrotor/actions/runs/36498124194)
passes 3,757 tests in 591.88 s, plus documentation, lint, formatting and typing.

A fresh audit passes 130 warning-strict relevant tests in 6.57 s and authenticates
all 66 previous evidence files, reconstructing eight new and four original
full-flight scores. No preceding defect is demonstrated. The
[frozen protocol](../../decisions/0031-independent-supported-start-validation.md)
is SHA-256 `528e107783af0bd7eb85c58c3e276c82fd75982b561c703f6b6410d00b08020b`.
It was written before implementing the runner or executing any full campaign job.

## Implementation and verification design

The new protocol reseeds the existing configuration factory without changing
the navigation prior, noise scales, controller, mission, wind, schedules or
fault definitions. Seeds 47001..47003 cover three clean cases each; seed 47004
covers all eight existing fault cases. No seed was screened or substituted.
Known development seeds and reserved geometric qualification seeds remain
separate. The exact 17-pair plan contains 34 flights.

The supported-start helper now accepts an explicit experiment configuration,
fault plan and supervision choice while preserving the original defaults.
Clean pairs compare alignment; fault pairs compare supervision under the same
aligned prior. Original sensor draws are verified against the exhaustive
pre-fault acquisition ledger, including sources later dropped or altered.
The independent fault mapping and saved ESKF replay still use actual delivered
measurements. Every plant/motor interval, command, guard, health transition and
supervisor decision is reconstructed before scoring.

Twelve new tests cover fixed job identities, full reseeding with unchanged laws,
dropout/offset/delay source verification, tampered records, strict individual
acceptance, missing/reordered jobs, rejected support that cannot fly, and saved
response reconstruction. Initial collection fails before the module exists.
The first smoke run catches an invalid attempt to replace a faulted result's
measurement ledger without replacing its matching event/schedule records. The
fix passes original observations directly to the noise verifier and leaves the
validated flight result intact. A test comparison of dataclass/array fault
values and a NumPy typing annotation were corrected before full execution.

The targeted supported-start, independent-campaign and original-robustness tests
then pass **48 tests in 42.94 s**. The complete warning-strict suite passes
**3,769 tests in 677.31 s**. Lint and formatting pass; strict mypy passes 86
source files. An intermediate redirected invocation emitted
no usable pytest summary and is not counted. All valid flight outcomes are
retained; no production source, dependency pin, threshold or controller tuning
changes are part of this milestone.

## Results

The three fresh hover peaks improve from 11.220063, 6.823515 and 6.868656 cm
to 10.180776, 5.463810 and 5.241372 cm respectively. Seed 47001 still misses
the unchanged 8 cm limit, so the frozen integration decision is **no-go**.
All six fresh nominal/wind tracking cases pass their original flight conditions.
All 34 planned flights have valid results, with zero rejected acquisitions and
zero numerical failures. Eight of nine clean comparisons and all eight fault
comparisons pass. The runner exits 1 to report the valid failed integration gate.

| Case | Seed | Unaligned RMSE, cm | Aligned RMSE, cm | Clean comparison |
| --- | ---: | ---: | ---: | --- |
| Hover | 47001 | 8.916440 | 6.489135 | FAIL: 10.180776 cm hover peak |
| Nominal tracking | 47001 | 8.356946 | 6.301776 | PASS |
| Wind tracking | 47001 | 8.391758 | 7.305856 | PASS |
| Hover | 47002 | 6.230716 | 4.087184 | PASS |
| Nominal tracking | 47002 | 6.112051 | 4.263220 | PASS |
| Wind tracking | 47002 | 5.862344 | 4.134687 | PASS |
| Hover | 47003 | 4.872766 | 4.013492 | PASS |
| Nominal tracking | 47003 | 4.774863 | 3.966930 | PASS |
| Wind tracking | 47003 | 4.418983 | 4.406492 | PASS |

Every aligned whole-flight RMSE improves, but individual terminal metrics do
not all improve. For example, seed-47002 hover final position error increases
from 1.588563 to 2.814400 cm while staying below 15 cm. The seed-47003 wind RMSE
change is only about 0.28%. Three seeds are not a population guarantee.

| Fault, seed 47004 | Supervision off RMSE, cm | Supervision on RMSE, cm | On terminal time, s | Response |
| --- | ---: | ---: | ---: | --- |
| Position dropout | 5.260611 | 6.035660 | 6.8025 | PASS: position timeout |
| Position rejection | 5.260611 | 6.035660 | 6.8025 | PASS: position timeout |
| Position delay | 5.260611 | 6.035660 | 6.8025 | PASS: position timeout |
| Altitude dropout | 5.170153 | 6.016267 | 6.2450 | PASS: altitude timeout |
| Altitude rejection | 5.170153 | 6.016267 | 6.2450 | PASS: altitude timeout |
| Altitude delay | 5.170153 | 6.016267 | 6.2450 | PASS: altitude timeout |
| Position recovery | 4.968285 | 4.968285 | 17.5000 | PASS: completes |
| Position loss during landing | 5.046901 | 5.499591 | 14.0050 | PASS: position timeout |

Fault RMSE values above cover each execution's own complete duration; differing
abort times make these descriptive scores, not an improvement comparison.
All disabled-supervision runs complete at 17.5 s. The aligned supervision pairs
have exact common-prefix parity. Recovery is confirmed at 6.6 s before expiry,
with complete on/off payload equality and passing flight conditions. Persistent
position faults meet their 1.0075 s onset ceiling; altitude faults meet 0.2875 s.
All aborts occur at the first sampled epoch satisfying the existing floating-point
deadline rule, without terminal-epoch commands. Loss during landing ends at
25.843955 cm/s true speed: the response is a correct numerical abort, not a
completed or safely landed flight.

Every recorded plant and motor reconstruction residual is zero; maximum
reconstructed sensor-noise discrepancy is 6.561e-15 against 1e-12. All estimator
states/covariances/events, commands, health and supervision reconstruct exactly.

At the failed aligned hover peak, t=6.2825 s, true position error is
`[0.028264051, -0.096928342, 0.013071350]` m. Estimated position error is
`[0.014257223, -0.080460922, 0.005598250]` m. The error is mainly horizontal;
the full-flight thrust-axis estimation error stays below 0.134 degrees. These
are observations from saved data, not a new causal intervention or a diagnosis
that proves which estimator/controller assumption is responsible.

## Commands

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_supported_start.py tests/unit/test_prearm_alignment.py tests/unit/test_early_flight_diagnostic.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python ../supported-start-bundle/verify_evidence.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_supported_repeatability.py tests/unit/test_supported_start.py tests/unit/test_robustness_validation.py -o faulthandler_timeout=45
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -m experiments.supported_repeatability --output ../supported-independent-campaign --workers 4
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q -o faulthandler_timeout=90
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
.venv/bin/python scripts/check_docs.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python ../supported-independent-bundle/verify_evidence.py
```

## Evidence integrity and identity

The final packaging audit detects three payloads damaged after their per-flight
save/decode/replay: a truncated `nominal_tracking-47001/trial-000-data.npz`, and
empty `trial-000-cov-000.npz` files in `wind_tracking-47001` and
`nominal_hover-47003`. Damaged copies and their actual/expected hashes are
preserved. Recovery requires exact original digests: one identical frozen flight
regenerates the data archive, and ESKF replay of authenticated saved observations
regenerates the two covariance chunks. All regenerated companion history and
diagnostic hashes must also agree before replacement. This adds one identical
evidence-regeneration execution, not a new scientific seed or tuned candidate.
The report, metrics, thresholds and decision are unchanged. The underlying
filesystem cause is not established.

All required recovery hashes match exactly. The independent NumPy-only bundle
verifier then authenticates **234 files**, independently reconstructs all **34
full-flight scores and 17 comparison decisions**, checks support/covariance/sample
handoffs, and verifies response deadlines and the preserved recovery receipts.
The complete clean and fault trajectories are plotted without removing startup
or abort segments; PNG plots are visually checked and PDF versions are retained.
Final documentation checks pass 124 files, 545 local links and 23 Python/JSON
examples. Git diff checks pass and production/dependency files are unchanged.

Executed source SHA-256:
`4e8c086968645ed189f379f4b306ac85b48bc6bea13125912e5dc1b6e1f283da`.
Campaign report SHA-256:
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Provenance records Python 3.12.14, NumPy 2.5.2, the audited merged base and an
uncommitted implementation tree. It does not claim a post-merge rerun. Generated
histories remain outside Git. The closing PR binds hosted CI and publication.

The complete evidence archive is
`quadrotor-independent-supported-start-2026-09-28.zip`, 368,311,620 bytes,
SHA-256 `17772bf4a29d605dd3c795e894e6224168b387398ee4b9e3634653a309828beb`.
It is retained as `libfile_cadfd95e288c8191b384cc529170a478`, version 0.
All archived entries are reauthenticated directly from the completed ZIP.
The archive contains all 34 histories and covariance chunks, support/release
records, diagnostics, full trajectories, plots, independent verification code,
test evidence and the original damaged payloads with exact recovery receipts.

## Decision and next step

Keep the initializer experiment-only. The independent experiment establishes
a repeatable improvement on this small seed set, but does not satisfy every
individual hover condition. Mass mismatch remains the previous failed
requirement; it was not rerun or relabeled. The cascade remains default and
geometric qualification remains open. No hardware or population reliability
claim follows from this campaign.

Next, freeze a bounded offline diagnosis of the retained failed fresh-seed
hover. Separate physical tracking, navigation-estimation error, attitude/bias
error and the supported-release transient before proposing another change.
Do not proceed directly to normal integration or another gain search.
