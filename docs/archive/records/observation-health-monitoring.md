# Observation health monitoring — 2026-09-28

## Preceding milestone audit

Starting commit: `21b37eed5849de754281a94c3fbcc814083663a7`, the merged
startup/hover closeout. All 227 baseline files were checked against their Git
blob hashes; the reconstructed tree matched
`1cda35ecc2d1885e33ea9f672f13ac8c9c35a313` exactly. The signed baseline commit
object was also reconstructed and matched its original hash.

I checked current source, tests, status, the startup study and the next-step
plan. ESKF online events follow position/altitude order after prediction and
correction. Older acquisitions are stale and never fused. Pending records are
appended to completed replay ledgers, not delivered to the online interface.
Mission truth and estimate guards are separate, and terminal epochs are still
observed. No preceding controller or estimator defect required a change.

Fresh bounded audit:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_geometric_transient_study.py tests/unit/test_eskf_online.py tests/unit/test_estimated_mission.py tests/unit/test_geometric_missions.py
```

Result: **89 passed in 141.75 s** before behavioral changes. Python 3.12.14 and
the repository's pinned uv 0.12.3/locked dependencies were used. The host's uv
version differed, so the pinned executable was installed in an isolated
temporary tool directory; project versions and dependencies were unchanged.

## Scope and design

[ADR 0021](../../decisions/0021-observation-health-monitoring.md) was written before
implementation. The monitor uses delivered ESKF statuses and the current clock
to report independent position/altitude availability. Explicit schedule-based
ages, rejection runs, recovery counts and a bounded recent-delivery window
govern six states. State transitions, accepted acquisition/delivery times,
latest delivery times and counts are exposed in immutable records.

The original integration sketch appended history to the mission result.
Inspection showed that the existing evidence writer automatically serializes
all result fields. I replaced that sketch with an optional caller-owned monitor
whose history is separate. The final production integration adds preflight
policy checks and one passive call after the estimator; mission results,
archive schemas and evidence writers are unchanged.

Invalid calls consume no clock or identity. Reset begins a new stream. Loss
of one sensor does not imply loss of the other. Pending information cannot
improve live health, and stale delivery cannot refresh accepted age. Freshness
does not establish accuracy, observability or flight readiness. No supervisor
action, estimator tuning or controller change is part of this milestone.

## Focused verification

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run pytest -q tests/unit/test_observation_health.py tests/unit/test_observation_health_mission.py
```

The first focused executions passed **49 monitor cases in 0.72 s** and
**14 integration cases in 20.51 s**. They cover exact threshold equality and
the next representable clock value, initial silence, independent sensor loss,
isolated/repeated rejection, recovery interruption, delayed/out-of-order stale
delivery, disabled fusion, pending/future refusal, identity reuse, invalid
scalars/counts, immutable snapshots, reset and atomic retry.

Six paired mission configurations exercise noisy cascade, noisy geometric,
delayed observation delivery, disabled fusion, forced rejection and immediate
abort. All original packed fields match byte for byte, including commands,
truth, measurement values, estimator corrections/covariance and mission
outcomes. Saved data and covariance archives have identical hashes with
monitoring on/off. Authentication and reconstruction pass; deliberately damaged
payloads are rejected. Replaying only delivered events at each saved clock
reproduces the entire health trace exactly, including the terminal epoch.
These are deterministic software regression fixtures, not new held-out flight
qualifications. Their generated temporary payloads are excluded from Git.

## Full gate and final review

With the pinned uv 0.12.3 executable first on PATH:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' make check
git diff --check
```

The complete warning-strict gate passed **3,401 tests in 609.44 s**. Ruff lint
and formatting passed (226 files checked by the formatter), and strict mypy
passed for 67 source, experiment and maintenance-script files. The documentation
check covers 92 Markdown files, 370 local links and 21 Python/JSON examples.
The guide's complete Python example was also executed and produced 21 observed
epochs. The final documentation cross-reference edit was checked again after
the full gate; no implementation or test expectation changed during that gate.

The final audit retains 218 baseline files byte for byte. Existing changes
comprise eight documentation files and 33 added lines in estimated_mission.py;
six new files contain the monitor, two test modules, guide, decision and this
record. Existing estimator/controller/plant algorithms, experiments, tests,
environment, dependencies and CI workflow are unchanged. All original saved
flight fields remain present, and no generated evidence is committed. The
publication PR and its hosted checks identify the final commit and merge tree.

## Remaining boundary and next step

The monitor reports evidence and takes no flight action. The cascade remains
the default, original noisy-hover failures remain open, and the reserved
geometric qualification seeds remain unopened. The next bounded task is the
supervisor policy for persistent health degradation; thresholds and actions
need their own acceptance criteria before integration.
