# Supported velocity prior and release uncertainty

Date: 2026-09-28. Scope: ADR 0034, one experiment-only supported velocity prior,
mandatory release screen and conditional flight regression. Production unchanged.

## Audit and protocol

Audit [PR 32](https://github.com/Gayles9/robust-quadrotor/pull/32), merged as
`a54145ed1e8cdd49cea8ffda5925bb5da12d054c`, tree
`8e73691278f8add04260b284fa38ef3b18e82678`. Clean local main matches live GitHub.
The preceding hosted run passes 3,792 tests in 626.10 s. Fresh warning-strict
oracle/support/pre-arm/documentation checks pass 119 tests in 11.87 s; the
independent preceding verifier authenticates all 26 payloads and its paired
scores, commands, trace and actual noise draws. No new defect or damaged evidence
is demonstrated in that milestone.

The source and existing support guide confirm that a supported left-limit IMU
sample is averaged across the discontinuous first flight interval. ADR 0033
explicitly requires checking this integration error before reducing uncertainty.
[ADR 0034](../../decisions/0034-supported-velocity-prior.md) freezes the candidate,
17-release necessary screen and stop-before-flight rule before implementation
and numerical evaluation, SHA-256
`75d832aaa8165b646b36c055dc167d07bd94a022980b634f1503476132b53f76`.

## Implementation and verification

`experiments/supported_velocity_prior.py` provides a local one-shot, fail-closed
conditioner. It requires a separate explicit zero-world-velocity support assertion,
fresh matching release identity/epoch/sample ownership and the independent
zero-mean velocity profile. Conditioning uses a 3x3 solve and the Joseph form,
with exact support variance zero. Every non-velocity covariance entry and state
mean stays unchanged. Initial joint rank is 18 instead of 21; no covariance
floor, in-flight observation, new sensor or production integration is introduced.

Thirty-one tests cover exact conditioning, rank, PSD and array ownership; one-shot
use after success/rejection; moving, revoked, stale and mismatched support;
sample/profile/clock/units failures; unsupported prior and sample-noise cross terms;
the analytic ballistic counterexample; scalar variance and all 21x21 propagated
covariance entries; retained new sample memory; and the zero-uncertainty limit.
Collection first fails with the expected missing-module error. The initial
implementation passes 30 tests in 0.50 s; an additional complete-covariance test
brings the total to 31. Initial long-line lint findings are fixed by formatting
before evaluation. The combined relevant suite passes 174 tests in 11.03 s;
mypy passes 90 files and targeted Ruff passes.

The runner authenticates every original campaign payload and recomputes all 34
saved flight scores. For each of its 17 jobs it regenerates the exact support
acquisition, release and aligned configuration, checks those against archived
bytes, applies the candidate once and evaluates the same analytic release case.
No baseline or candidate mission is flown. The separate saved verification
rederives all 17 screens and matches the stored result; exit 1 denotes the valid
candidate rejection, not an execution error.

The separate NumPy-only verifier also reconstructs every candidate release and
all pre/post 21x21 covariances, analytic position/velocity errors and decisions.
Its first attempt incorrectly requires bitwise equality with decimal `0.5025`;
the saved endpoint is `201*0.0025`. Correct that verifier assertion to the existing
1e-12 s clock tolerance, retaining exact support/sample epoch equality. The
verifier then passes all 17 cases without changing any saved result or threshold.
Documentation checks pass 133 files, 593 links and 23 Python/JSON examples; Ruff
lint/format (306 files) and mypy (90 files) pass. The full warning-strict suite
passes **3,823 tests in 804.25 s**. Hosted receipts accompany the publication
closeout.

## Findings and acceptance

For motor-off, no-drag free fall after support removal, the physical acceleration
is `g_W` throughout the open first interval. The map instead interpolates from
zero supported acceleration to `g_W`. At 2.5 ms, its deterministic errors are
`-g*h/2 = -0.0122625 m/s` in down velocity and
`-g*h^2/3 = -0.0000204375 m` in down position.

The candidate's predicted down-velocity standard deviation is
`0.00010307825076731747 m/s`; the two-sided Gaussian 99% marginal radius is
`0.00026551197888501825 m/s`. The deterministic error is **118.9630 reported
standard deviations**, or about 46.18 times that radius. All 17 candidate release
screens fail. The original prior's standard deviation is about 0.0300001771 m/s
and contains the same bias inside its necessary radius; that does not establish
calibration or justify retaining a broad prior as an integration-error model.

The 17 checks share an uncertainty profile and are not independent statistical
trials. This analytic limiting case is not a newly measured flight score, nor
the exact error of the campaign motor-ramp trajectories. It is sufficient to
reject the proposed uncertainty with the unchanged release map.

**The scoped investigation succeeds; the prior-only candidate is rejected.**
All 25 conditional regression flights are skipped exactly as frozen, with zero
new scientific flights. No attempt is made to fit a floor, select a different
seed, repair the map during this experiment or repeat a valid outcome. The
ordinary 10.18 cm hover failure, separate mass failure and geometric qualification
are unchanged. The cascade remains the default.

## Next exact scope

Derive and validate one release-aware first prediction interval using the first
post-release IMU sample causally. Handle force discontinuity, full gyro/sample
noise memory, finite-difference Jacobians and numerical-error bounds explicitly.
Use the ballistic counterexample and smooth-force checks before changing flight
behavior. Keep later intervals unchanged. Do not invent a second release sample,
use true velocity/force, hide the discrepancy in Q/R or claim that repairing this
small interval will recover the oracle's whole-flight improvement. A separately
frozen comparison follows only after the revised boundary model is credible.

## Commands and evidence

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_navigation_feedback_oracle.py tests/unit/test_supported_start.py tests/unit/test_prearm_alignment.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python PRIOR_ORACLE/verify_evidence.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_supported_velocity_prior.py tests/unit/test_prearm_alignment.py tests/unit/test_supported_start.py tests/unit/test_eskf_endpoint.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.supported_velocity_prior --campaign SAVED_INDEPENDENT/campaign --output NEW_SCREEN
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.supported_velocity_prior --campaign SAVED_INDEPENDENT/campaign --verify NEW_SCREEN
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python SCREEN_BUNDLE/verify_evidence.py
uv run python scripts/check_docs.py
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q
```

Execution uses the existing `.venv/bin` tools directly; the commands above are
equivalent in the pinned environment. No dependencies or tool versions change.
Executed source SHA-256:
`c8d4dce45e707c2fd66ee709bcc2635ad5ed64994e04a3b898f4354a87cc963e`.
Result SHA-256:
`e7e97281771aca2db616117fbb8f2effa591c6719d5b15ca4354e06be5d840ca`.

Generated evidence remains outside Git. The bundle retains the original report,
all 17 exact support/release/configuration records, all candidate releases and
complete pre/post 21x21 covariances, frozen protocol, command receipts and a
NumPy-only independent verifier. Publication identities and hosted checks are
recorded in the publishing PR.

The final standalone verifier authenticates 65 payloads and independently
reconstructs all 17 candidate rejections. Archive
`quadrotor-supported-velocity-prior-2026-09-28.zip` contains 66 members, 452,482
bytes, SHA-256
`69a4c0fa0e40856e2a42802443a7fd4914fcd503488c321814b43c0533b65bc9`.
Every ZIP member is authenticated against its source before publication.
