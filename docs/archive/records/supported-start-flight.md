# Supported-start flight verification

Date: 2026-09-28. Scope: eight frozen experiment-only flights and authenticated
comparison against the original campaign.

## Audit and frozen decision

Audited main `f72cb3521d64a8a6d5726fb0c89dcca3ced48166` (merged PR 28), matching
GitHub with a clean worktree. Fresh warning-strict component, inherited pre-arm
and documentation tests pass 133 tests in 1.09 s. Seven preserved evidence
payloads authenticate and reconstruct all 15,000 numerical comparisons and 200
handoffs. No preceding defect is demonstrated. The original ADR 0023 report,
every referenced payload and all original execution metrics also authenticate.

[ADR 0030](../../decisions/0030-supported-start-flight-evaluation.md) freezes the
physical boundary, seeds, eight flights and acceptance before implementation.
The production pre-arm component, estimator, controllers, dependency pins and
performance thresholds remain unchanged. Reserved geometric qualification seeds
remain unopened. No post-result tuning or alternate flight candidate was run.

## Physical and causal comparison

The [experiment guide](../../results/supported-start-flight.md) distinguishes three
conditions: archived original moving/spinning initialization, explicit fixture
release without alignment, and the identical fixture release with alignment.
Only the initial alignment estimate/covariance differs between the supported
arms. Original true position/attitude and p/v prior information are retained;
zero true velocity/rate and zero actual motor speeds are explicit consequences
of the new support condition, not hidden edits to the old flight.

The fixture supplies the gravity/drag reaction, records constant pose and zero
motion/motors, and maintains support through the fresh sample at 0.5025 s.
It then releases with continuous state and motor speed. The first endpoint
sample is the supported left limit; the original ESKF's approximation across
the ensuing force step is retained. No ground-contact or physical landing model
is implied. No stationary interval is assumed during subsequent flight.

Support acquisition has independently named version-four PCG64 streams. The
fresh sample uses original flight draw zero; every later measurement and bias
increment uses the original flight streams and schedules. Reconstructed maximum
draw discrepancy is 7.023e-15 in the twelve original/new histories, against
1e-12. The exact fresh sample and all 21x21 endpoint covariance entries agree.

## All retained results

| Metric, cm | Original free flight | Supported unaligned | Supported aligned |
| --- | ---: | ---: | ---: |
| Hover peak, inclusive 5..11 s | 10.756338 | 8.588088 | 2.506294 |
| Hover full-flight RMSE | 7.660495 | 7.420963 | 3.624351 |
| Nominal tracking full-flight RMSE | 6.244457 | 6.780493 | 4.258048 |
| Wind tracking full-flight RMSE | 6.717405 | 7.198188 | 5.039704 |
| Mass tracking full-flight RMSE | 42.393311 | 42.780350 | 42.525925 |

The aligned hover passes the unchanged 8 cm peak limit, all whole-flight and
terminal limits, and both frozen hover comparisons. Peak reduction is **70.8%**
against the physically identical supported control and **76.7%** against the
different original start. Its early thrust-axis estimation error peaks at
0.105652 degrees versus 2.999932 degrees in the supported unaligned arm.

Aligned nominal and wind tracking pass every original flight condition. Their
final position errors are 6.027688 and 8.342076 cm. These are slightly higher
than their supported controls' 5.880290 and 8.157272 cm, within the original
15 cm limit. Preserving passing conditions does not mean every metric improved.

Mass mismatch remains a clear failure in all three conditions. Aligned RMSE is
42.525925 cm, final error 43.351818 cm and `landing_timeout` occurs at 25 s.
Its final speed passes, as before. Better inclination does not remove the
controller's missing mass compensation. No mass requirement is marked complete.

Flight terminal times remain 15.5 s for hover, 17.5 s for nominal/wind translation
and 25 s for the mass timeout. Each new execution additionally uses 0.5025 s of
support. All original flight samples, startup errors and the complete hover
window are included. The full takeoff transient is not confused with the
separately specified 5..11 s hover peak.

## Verification and execution history

Fifteen new tests independently check equilibrium with mass/wind, gravity sign,
pose/motor continuity, paired configurations and p/v isolation, full endpoint
reconstruction, sample identity and actual RNG continuity, rejection before
flight, adapter restoration on exception, noise-tampering rejection, full-time
scores and both comparator conditions. The initial tests fail before the new
module exists. Mechanical type-check issues were resolved before any full flight.

All eight saved flight histories authenticate and reconstruct every estimator
state, covariance and event; both controllers; mission guards and supervision;
and every nonlinear plant/motor interval. Plant and motor residuals are exactly
zero. Each saved history is decoded before scoring. An independent NumPy-only
verifier authenticates 66 files and reconstructs all eight new and four original
full-flight scores, covariance handoffs and comparison decisions. The complete
trajectories are plotted without cropping the startup or mass failure.

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_supported_start.py tests/unit/test_prearm_alignment.py tests/unit/test_early_flight_diagnostic.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -m experiments.supported_start_validation --baseline ../startup-inputs/original/campaign --output ../supported-start-campaign --workers 2
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python ../supported-start-bundle/verify_evidence.py
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q -o faulthandler_timeout=60
```

Targeted checks pass **130 tests in 6.73 s**. Lint/format checks pass and strict
mypy passes 84 source files. Python 3.12.14 / NumPy 2.5.2 and existing pins are
unchanged. An initial local full-suite run was stopped after prolonged lack of
progress near the existing headless-plot test. That unchanged test passes in
isolation in 0.93 s; no source defect was established. The full suite was
restarted with timeout diagnostics and without concurrent flight work. The
interrupted run is preserved and is not counted as passing. The restarted
warning-strict full suite passes **3,757 tests in 524.16 s**. Documentation checks
pass 121 files, 529 local links and 23 Python/JSON examples; Ruff checks pass
with 283 files already formatted. An initial GitHub upload was blocked pending
confirmation of publishing authorization; that authorization is now explicit.
The closing pull request records hosted CI and publication identities.

| Identity | SHA-256 |
| --- | --- |
| Frozen ADR 0030 | `18e84e9fa7b4502a2ffaf8ffcbd868282168925d5519ab44e1ac0c4d0d37ac39` |
| Executed source | `251d74b5069fc0c926bf9039a7dc1fe8a3d3940e318933ecbd4908924d6a9dd1` |
| Original report | `ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09` |
| New report | `e6a63d72057419504598aded02ac491da490db73ac02d6f344d9aab07de85b17` |
| Complete evidence ZIP | `fe1ee76eedef44d0bb469e1d76323ae322036e13831003f650ea77e03a33c645` |

The 110,310,671-byte `quadrotor-supported-start-flight-2026-09-28.zip` is retained
outside Git, identity `libfile_999f9d834a10819194e3b99d8876247b`, version 0.
It contains all eight full histories/covariances, support and release records,
diagnostics, original report, complete comparison trajectories, plots and
independent verification code. Source and protocol are bound by their recorded
fingerprints; execution honestly records the audited base and an uncommitted
worktree rather than claiming a post-merge rerun.

## Decision and next step

**The frozen supported-initialization comparison passes.** This demonstrates a
substantial tested startup benefit under genuine modeled support. All-four-case
flight qualification remains false because mass mismatch still fails. Local
software gates pass; publication and hosted CI remain pending.

Keep the initializer experiment-only. Next, freeze one independent supported-start
validation campaign covering repeatability and observation-fault responses before
considering normal mission integration. These known seeds do not establish a
population guarantee, robust mass performance, hardware support or geometric
qualification. Cascade remains the default controller.
