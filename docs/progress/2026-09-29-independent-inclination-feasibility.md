# Independent-inclination feasibility closeout

Date: 2026-09-29. Scope: [ADR 0040](../decisions/0040-independent-inclination-feasibility.md).
The [contract](../independent-inclination-feasibility.md) contains the complete
measurement, uncertainty, calibration, timing and eligibility requirements.

## Audit and scope

The audited predecessor is PR 38, merged at
`0129d658e905a644f06da1c51731ffe85c2655a9`, tree
`17fa3b7f92ff80804972bfc7aeddd3329756cf01`. The remote and clean local main agree.
Fresh relevant warning-strict tests pass 57 cases in 1.54 s. The preceding
independent artifact checker verifies all 27 histories, 144 Gram matrices and
13,608 observation records. No preceding numerical implementation defect is
demonstrated. The new protocol was frozen before derivation code and results.

The source audit confirms IMU, local position and barometric altitude at the
current mission/replay boundary. No independent external orientation model or
calibrated data source is documented. The historical compatibility spike is
simulation evidence, not an orientation sensor contract. The original scope is
simulation-first, with vision and hardware deferred. A physically justified
simulated sensor does not require purchased hardware, but it does require an
explicit new model and defensible assumptions; those cannot be fabricated to
close an existing flight requirement.

## Completed mathematics

One experiment-only module evaluates deterministic geometric and Gaussian
fixtures. It adds no observation type, sensor stream, ESKF update, production
behavior or flight. It binds the preceding source and saved error-budget report.

- Explicit external-reference, target-mount and body/world rotation composition.
- Two-coordinate tangent chord residual and right-local 21-state Jacobian.
- Angular-noise covariance projection, chart invariance, hemisphere rejection
  and exact body-axis twist invariance.
- Full joint Gaussian conditioning with state/noise correlation, and the
  restricted sequential decorrelation identity for state-independent noise.
- True-rate/age bound, conditional two-dimensional NIS threshold and finite
  sequence Gaussian union bound, without selecting a physical noise/rate model.
- Local information comparison with calibrated inclination versus two unknown
  constant inclination-bias coordinates.
- Explicit missing-source/uncertainty ledger and no-go for implementation.

The preceding guide's "heading free" phrase is clarified: body-down leaves a
right rotation about the body-down axis unobserved. World-vertical rotation
usually changes the observed direction when tilted. The earlier level-hover
rank result is unchanged; no production equation required repair.

## Results

| Check | Result | Frozen acceptance |
| --- | ---: | ---: |
| Maximum right-local/world-noise Jacobian error, four poses | 4.932e-11 | 1e-8 |
| Exact right body-axis twist direction difference | 0 | 1e-12 |
| Correlated joint versus decorrelated sequential conditioning | 1.111e-16 | 1e-12 |
| Unknown-bias analytic nullspace residual | 0 | 1e-12 |
| Calibrated inclination model | Rank 13 of 15 | Verify existing local result |
| Two unknown inclination biases | Rank 13 of 17 | Retain four explicit null directions |

The dimensionless correlated Gaussian fixture changes posterior mean by
0.033050 in Euclidean norm and covariance by 0.126654 in Frobenius norm when
cross-sensor covariance is falsely set to zero. These are algebraic fixture
differences, not predicted flight errors or proposed sensor covariances. A
separate fixture verifies state/noise correlation using an independent joint
Gaussian transformation.

The existing single-input inclination ceiling is 0.00815603346 rad. With an
illustrative true-rate bound of 1 rad/s, 8.156 ms of age consumes that entire
allowance. No actual rate/latency requirement is adopted from this example.
The conditional two-dimensional 99% NIS threshold is 9.21034037, derived from
the radial tail; it is not installed, and existing gates stay unchanged.
For 1,000 zero-mean tangent Gaussian marginals and 1% sequence failure budget,
the union-bound radius is 4.798526 times the maximum marginal standard deviation.
Neither calculation supplies missing bias, posterior error or physical residual
bounds. The simultaneous allocation remains explicitly unknown.

An additional NumPy-only checker verifies the saved rotations with Rodrigues'
formula and the Gaussian results through precision-block conditioning, then
checks the nullspace and probability/budget identities. It passes independently
of the experiment's quaternion and gain-form implementations.

## Acceptance and decision

The conditional mathematical contract meets its acceptance criteria. The source
and simultaneous-budget requirements for implementation do not. Eight missing
items remain explicit: source/model, calibration, noise/bias, acquisition timing,
outage policy, correlations, posterior joint error bounds, and physical/vertical/
initial residual allowance.

Close the independent-inclination extension as **no-go for a standalone
component under current assumptions**. No synthetic sensor campaign or scientific
flight is run. No camera, motion-capture system or calibration is assumed to
exist. The local unknown-bias result does not prove every possible external
orientation design will fail; it explains a requirement that must be met before
such a scope could be reopened.

The original cascade remains the default. Experimental alignment/release,
geometric and vertical-compensation options keep their recorded limitations.
The startup study stays closed and its failed no-regression comparison is
preserved. This closeout does not pass G2, solve mass mismatch or authorize
middleware/hardware integration.

## Next exact action

Consolidate the existing original-sensor operating envelope and update the
technical report to the current merged evidence. Audit each implemented
capability and each passing/failed requirement against the authoritative report
or source, freeze the report scope, update the mathematical derivations and
results, compile/inspect the PDF, and preserve editable LaTeX and evidence links.
Keep old failed gates and conditional assumptions explicit. No further sensor
research, controller tuning or flight campaign belongs in that deliverable.

This is a finite project deliverable within the original scope. The report's
completion must remain distinct from a decision to integrate or to change the
original flight acceptance criteria.

## Verification and reproducibility

Thirty-one new tests pass in 0.19 s. They cover independent finite differences,
frame order, body twist versus world yaw, chart invariance, antipodal/nonunit/
invalid inputs, information-form and joint-distribution Gaussian checks,
correlation rejection, age bounds, Gaussian radial tails and analytic nullspace.
The first run passed 30 tests and exposed a degenerate frame-order fixture:
two particular 90-degree compositions happened to give the same direction.
The wrong-order fixture was corrected to a distinguishable permutation; no
implementation tolerance or acceptance criterion changed. Static checking also
caught array/list annotations before execution, which were corrected.

The full warning-strict suite passes 3,949 tests in 363.09 s. Ruff, formatting
(343 files), strict mypy (101 source files), documentation (151 files, 681 local
links, 23 Python/JSON examples) and the independent artifact checker pass.
Hosted CI results are preserved with the evidence. Commands:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q tests/unit/test_inclination_feasibility.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m pytest -q
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src experiments scripts
.venv/bin/python scripts/check_docs.py
```

The guide provides the derivation runner command. Generated evidence retains
the frozen protocol, source budget report, all geometry/Gaussian fixtures,
the decision and missing-requirement ledger, independent verifier and
test/publication receipts outside Git.

| Item | SHA-256 |
| --- | --- |
| Preceding execution source | `d83889b7189ab898aadb87bfa258259ec30e381b14f0d550d207343835216f95` |
| Derivation execution source | `520ce47589df872cef06f3851bd4cd4b6b82f5cbe6cceeae7cc8fd96647a2984` |
| Frozen ADR 0040 | `2cc199533eff8b5f57b75be4afc168863bd4a22f43851f5a4e8f83fadedfb8d9` |
| Derivation report | `f0b03f6d39ec746bcad7d71aba337191d6bac9c43c63af4ead777a2be8280164` |
| Original error-budget report | `b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb` |
