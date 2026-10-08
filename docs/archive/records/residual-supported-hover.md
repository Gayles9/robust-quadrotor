# Residual supported-hover diagnosis

Date: 2026-09-28. Scope: ADR 0032 offline diagnosis, six saved hover histories,
zero new scientific flights and no change to production behavior.

## Audit and frozen scope

Audit the PR 30 merge `5e532ab1f8b991643aff7628ea59d69d2915cacc`, matching
the live GitHub main reference and clean local worktree. Its implementation is
`3b3a99d2853c9bedb7cccf54289fcdd540db0b4d`; its tested tree is
`df3f13c98682fd87ee20f1fb519c2295d32f898c`.
[PR 30](https://github.com/Gayles9/robust-quadrotor/pull/30) records successful
[hosted CI](https://github.com/Gayles9/robust-quadrotor/actions/runs/36501013806),
including 3,769 tests in 620.03 s.

Fresh warning-strict supported-start, independent-validation, robustness and
documentation checks pass 59 tests in 45.59 s. The independent prior-bundle
verifier authenticates 234 payloads and reconstructs all 34 scores and 17
decisions. No new evidence damage or preceding implementation defect is found.
The prior archive's three exact-byte recoveries remain disclosed in its record;
this step makes no replacement or new flight to regenerate data.

The [protocol](../../decisions/0032-residual-supported-hover-diagnosis.md) is frozen
before diagnostic code or analysis. Its SHA-256 is
`91a12de4522cf308354f79549e4c180debbddb9741eb00096d59d5bb828c2ec9`.
The fixed input report is
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`.
Seeds47001/47002/47003 and both prior modes are retained; no threshold,
controller, prior, random stream, support boundary or flight score changes.

## Method and checks

`residual_hover_diagnostic` authenticates every campaign payload reference and
recomputes all 34 original flight scores. It compares the six hover configurations
against reconstructed original dataclasses, then rebuilds every outer/inner
command and each projected RK4 plant/motor interval. It separates navigation,
attitude estimation/tracking, motor, mass, drag, command limits, reference and
integration terms in an explicitly ordered acceleration identity.

`hover_response` propagates each term through the original sample-held PD
response. A separate unfitted horizontal cascade uses the original attitude/rate
gains, motor lag and nested clocks, with saved error histories as forcing.
The [guide](../../results/residual-supported-hover.md) derives both models and states the
limits. No counterfactual flight or proposed deployable correction is hidden
inside the response decomposition.

Ten new tests cover analytic held acceleration, signed cancellation, impulse
increments, causal ownership/shape/finiteness rejection, exact motor-lag flow,
matrix composition, nested update clocks, release-window boundary, authenticated
input rejection and a separate seed47811 smoke reconstruction. The initial smoke
test correctly encounters immutable saved arrays while injecting tampering; the
test now copies its target before changing it. Six initial typing errors were
fixed before executing the diagnosis. Neither issue changed a scientific history.

The first targeted run after these corrections passes 48 tests in 9.01 s; a
subsequent release-boundary test is included in the final full gate. The diagnostic
then completes all six fixed histories without an execution or evidence failure.
All saved command, plant and motor reconstruction residuals are exactly zero.
Maximum acceleration closure is 3.553e-15 m/s²; full position/velocity response
closure is 4.552e-15 in the respective SI units, below 1e-10.

The complete warning-strict suite passes **3,779 tests in 560.88 s**. Final
documentation checks cover 127 files, 559 local links and 23 examples. Ruff lint
and formatting pass (296 files); strict mypy passes 88 source files. Hosted CI
and the exact publication/merge identities are recorded in the publishing PR.

## Results and decision

| Seed/mode | Original hover peak, cm | Peak time, s | Local-model horizontal RMS discrepancy, mm |
| --- | ---: | ---: | ---: |
| 47001 unaligned | 11.220063 | 5.2700 | 1.388958 |
| 47001 aligned | 10.180776 | 6.2825 | 0.761056 |
| 47002 unaligned | 6.823515 | 6.6775 | 0.684527 |
| 47002 aligned | 5.463810 | 5.0000 | 0.442811 |
| 47003 unaligned | 6.868656 | 6.4875 | 0.365639 |
| 47003 aligned | 5.241372 | 10.9650 | 0.168795 |

These are the same six original outcomes. The local model changes no flight
metric; its maximum full-history horizontal discrepancy is 3.158568 mm.
All six histories have zero limited inner/outer command rows. The sampled
controller/plant model has spectral radius 0.9831523666410144. Its stable
local poles do not qualify the joint nonlinear estimator/controller.

The primary failed case has true error `[2.826405, -9.692834, 1.307135] cm`
at 6.2825 s. Navigation position and velocity response contributions in East
are -2.571280 and -5.157843 cm. Attitude-estimation and tracking contributions
are -1.027470 and -0.943079 cm. Motor/within-step/initial terms nearly cancel
in East; mass and drag are zero. The full signed vector table is in the guide.
Estimated tracking norm is 8.190585 cm while navigation-error norm is 2.287388 cm.
The vehicle's 10.180776 cm displacement is physical.

The combined initial-state and nonreference forcing confined to intervals before
0.5 s contributes `[ -0.383142, -0.661970, 0.219269 ] mm` at the later peak,
norm 0.795664 mm. This is the direct retained response in the chosen decomposition;
later coupled feedback may still carry earlier influence. It is not a physical
experiment that removes startup. In the failed flight's hover window, navigation
position/velocity error RMS norms are 2.449365 cm and 2.059157 cm/s, axis-estimation
RMS/peak are 0.086990/0.120522 degrees, accelerometer-bias RMS is 0.008120 m/s²,
and gyro-bias RMS is 0.000179 rad/s. These are measured error magnitudes, not
independent causal allocations to bias estimation or proof of a filter bug.

**The diagnostic acceptance succeeds. Flight qualification remains no-go.**
The strongest supported direction is an outer-loop position/velocity channel
isolation test. No new controller/estimator law is justified merely by subtracting
correlated response components. Keep the cascade default, alignment experimental,
the 8 cm hover gate open and mass compensation separate.

## Reproduction and evidence

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_supported_repeatability.py tests/unit/test_supported_start.py tests/unit/test_robustness_validation.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python PRIOR_BUNDLE/verify_evidence.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q tests/unit/test_residual_hover_diagnostic.py tests/unit/test_early_flight_diagnostic.py tests/unit/test_supported_start.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.residual_hover_diagnostic --campaign SAVED_INDEPENDENT/campaign --output NEW_DIAGNOSIS
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python DIAGNOSTIC_BUNDLE/verify_evidence.py
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments scripts
uv run python scripts/check_docs.py
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' uv run python -m pytest -q
```

Execution uses the existing pinned environment's `.venv/bin/python`, Ruff and
mypy directly, equivalent to the commands above. Python/NumPy/tool pins and all
production source remain unchanged. Executed experiment/source fingerprint:
`0525ab8ea72eee176b4a7410511fc698ef6a4187d1138227d3354a7acb89668f`.
Diagnostic report SHA-256:
`949375db59e656659f8c7378060094b427a2958c1a17ccbc211cff0fa1efe928`.

The separate evidence bundle includes the six original raw noncovariance
payloads, original configurations/report, six full derived signal histories,
frozen protocol, two full-history figures in PNG/PDF, plotting source and an
independent NumPy-only verifier. Original covariance/event reconstruction remains
in the preceding complete 34-flight archive. Every original field in each derived
history is compared with its authenticated raw payload. The verifier recomputes
all six scores, acceleration identities, full and first-0.5-s response histories
and local-model discrepancy metrics. Generated data and plots stay outside Git.

The independent verifier passes before packaging. Final archive validation
authenticates every one of its 31 ZIP members (30 payloads plus manifest) directly
from the completed archive. Filename:
`quadrotor-residual-hover-diagnosis-2026-09-28.zip`; size 95,100,214 bytes;
SHA-256 `2f2c74ebdfd44dc848c3236a1ed63ece1be58c800ee9bb99578c39f99098f516`.

The next exact action is to freeze and run one seed47001 aligned diagnostic
with true position/velocity supplied only to the outer controller, retaining all
estimated guard, completion and attitude/rate inputs and the original full scoring.
Its result will decide whether to design an implementable navigation-information
change. It cannot itself qualify truth-assisted feedback for deployment.
