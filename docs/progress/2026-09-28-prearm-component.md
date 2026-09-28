# Standalone pre-arm component verification

Date: 2026-09-28. Scope: one supported acquisition and estimator handoff component.

## Audit and frozen scope

Audited main `81e28ddbd3c61d01a4383803277b70f4611a9724` (PR 27), matching
GitHub with a clean starting worktree. Fresh warning-strict pre-arm/documentation
checks pass 53 tests in 0.50 s. Six archived payloads authenticate and reconstruct
all 15,000 preceding outcomes, including normalized variance 1.061394 on the
original regression and 1.082222 on the fresh Gaussian set. No preceding defect
was demonstrated.

[ADR 0029](../decisions/0029-standalone-prearm-alignment.md) freezes the interface,
physical support boundary and acceptance before implementation. The numerical
model, acquisition length, priors, uncertainty limits, controller, ESKF algorithms
and flight seeds remain unchanged. The offline ADR 0028 implementation stays an
independent reference. The production modules import no experiment code.

## Implemented behavior

The [component guide](../prearm-component.md) specifies explicit acquisition,
stream, clock and support provenance; contemporaneous interval evidence;
mechanical support and motors-off assertions; and revocation. The session
requires 201 consecutive paired samples over 0.5 s, owns its inputs and
diagnostics, latches rejection and releases once with a fresh sample at 0.5025 s.
Incomplete windows, sample reuse/gaps, invalid clocks and expired readiness
cannot produce an endpoint. Time checks are caller driven, with no background
clock or implication that a stale READY snapshot remains valid.

Only the approved independent alignment prior/profile is accepted. Explicitly
independent p/v means and covariance survive unchanged. The approved positive
quadrature retains attitude/terminal-bias cross covariance. Release adds only
one interval of bias walk and initializes independent fresh sample-noise memory.
The fresh measurement is not recycled as alignment data. There is no reset,
automatic retry, motor API or flight integration.

## Frozen numerical acceptance

| Check | Result | Required |
| --- | --- | --- |
| Archived moment populations | All 15,000 retained | No selection |
| Maximum rotation disagreement | 2.578175e-17 rad | <=2e-14 rad |
| Maximum error-coordinate disagreement | 1.110900e-16 rad | <=2e-14 rad |
| Covariance difference whitened by reference | Exactly zero | <=1e-9 |
| Rejection outcomes | Identical for all 15,000 | Identical |
| Original Gaussian maximum normalized variance | 1.061394 | <=1.10 |
| Fresh Gaussian maximum normalized variance | 1.082222 | <=1.10 |
| Fresh Gaussian maximum absolute whitened mean | 0.020453 | <=0.05 |
| Nominal rejection | 0/5,000 | <=1% |
| Complete acquisition/release sessions | 200/200 released | All 200 |
| Complete-session compatibility statistics | Exactly equal to reference | Unchanged gates |
| Full endpoint covariance | Exactly equal to independent block assembly | Preserve all blocks |

The complete sessions are the first 100 fresh nominal and first 100 fresh
Gaussian trials, with no substitutions. Their independent release bias-walk
and sample-noise streams use SeedSequence([0x50524541,3,partition,trial]).
The evidence retains each release covariance, sample, truth bias, p/v, IDs,
ownership result and comparison. The standalone NumPy verifier authenticates
seven payloads, reconstructs all 15,000 comparisons/calibration outcomes and
all 200 endpoint matrices, and independently checks fresh sample construction.

## Tests, defects and provenance

Eighty new black-box component tests cover state transitions, support continuity,
revocation, clock boundaries, malformed inputs, unsupported priors, sample IDs,
immutable ownership, complete covariance handoff, simultaneous failure reasons,
motion/numerical rejection and the required constant-acceleration ambiguity.
The initial suite fails before the module exists. A reversed-interval test
fixture was corrected to distinguish constructor rejection from session rejection.
A later regression test demonstrates NumPy unsigned sample-ID overflow; input
IDs now normalize to arbitrary-precision Python integers before sequencing.

The complete 15,000/200 study was rerun after that input fix, with identical
numerical outcomes and no model or threshold changes. An earlier full test run
was deliberately interrupted and restarted on corrected source; it is not
counted as a passing gate. Both study executions remain locally preserved.

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_prearm_alignment.py tests/unit/test_prearm_nonlinear_uncertainty.py tests/unit/test_prearm_alignment_feasibility.py tests/unit/test_documentation.py
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python -m experiments.prearm_component_validation --prior-evidence ../prearm-nonlinear-evidence --output ../prearm-component-final
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error .venv/bin/python ../prearm-component-final/verify_evidence.py
.venv/bin/python scripts/check_docs.py
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q
```

Targeted checks pass **133 tests in 1.41 s**. Lint and formatting pass; strict
mypy passes 82 source files. Python 3.12.14 and NumPy 2.5.2; all dependency pins
remain unchanged. Final full-suite and GitHub CI execution results are recorded
in the closing pull request.

| Identity | SHA-256 |
| --- | --- |
| Frozen ADR 0029 | `583d68b3f321fd175a6bb942956e0c430098ce4b66f0af2613e0268964abe505` |
| Executed source fingerprint | `635869b710d23f8cbbd6284cf2fc780001ad76fb1f8ca3103e68b07bcc0e5c4e` |
| Report | `1faa2ccea90f8ba43acd3245462edb22842781c6050e34181411397d6f1f33e5` |
| Trial payload | `3c89634e2f0527db6d86c7af17f73a1617d209e5ad08384025ebbaddb67acc0b` |
| Evidence ZIP | `cd5efaa8d9be8819e1895fe3adce495ab8e8bdba16d9a81f11a192e62fc904bd` |

The 14,382,527-byte evidence ZIP is retained outside Git as
`quadrotor-prearm-component-2026-09-28.zip`, file identity
`libfile_05ca978aa34c8191a63451b1d2a8a2ef`, version 0. The report honestly records
execution from the audited base with an uncommitted worktree; its source
fingerprint binds the subsequently published implementation.

## Completion and limits

The bounded component meets its numerical and behavioral acceptance. Completion
also requires the repository software gates recorded in the closing PR. It is
go for a separately scoped supported-start flight evaluation, with the original
controller, common flight noise, full scoring and unchanged limits.

No improved hover result is claimed here. The original free-flight and mass-case
failures remain open, the cascade stays default, and geometric qualification is
separate. Support authenticity and cross-session acquisition allocation belong
to the external owner. Constant acceleration can pass the numeric gates; the
component is not a hardware interlock. Clock qualification, real-time scheduling,
hardware support procedures and arbitrary correlated priors are outside scope.
